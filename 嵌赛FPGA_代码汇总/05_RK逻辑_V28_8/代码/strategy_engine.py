"""Decision engine: scene branch -> nav / traffic / lane -> FPGA display.

Priority is exclusive, exactly one model line runs per round:
    nav present  -> navigation only
    traffic      -> traffic only
    lane         -> lane only

Lane uses the v2 grey model (lane_v2.py, G640). Traffic uses
the housing detector plus traffic_rules.judge, which validates the boxes
geometrically before mapping box count to left/straight/right.

⚠️ 2026-10-31：二值通道整个删除。全部模型只吃 FPGA 的 MODE 1（YRG）。
"""
import time
from dataclasses import dataclass
import numpy as np
from protocol import Display, RED, GREEN, BLUE, YELLOW, COLOR_NAMES
import lane_stability
import traffic_boxes
import traffic_rules
import cop_traffic_signature
from lane_geometry import select_lanes
from lane_v2 import decode as lane_v2_decode
from lane_stability import LaneSolidStreaks


def _person_in_front(scene, h=720, w=1280, y_min=.55, x_min=.25):
    for box in scene:
        if box['class_name'] != 'person':
            continue
        x1, y1, x2, y2 = box['xyxy']
        if (y1+y2)*.5 >= h*y_min and x_min*w <= (x1+x2)*.5 <= (1-x_min)*w:
            return True
    return False


# 斑马线几何判据（V26）。
#
# 2026-10-31 板上日志实测：报警一共 132 轮，但绝大多数是同一帧里的
# `[0, 0, 1178, 132]` 反复回放 —— 那是一个**横跨整幅画面、贴在顶部**的框，
# 面积占 30%，根本不是路面斑马线（cy 0.09）。它来自场景缓存（30 轮复用）
# 加上 `WarningHold` 的锁存，于是一次误检能亮一两秒。
#
# 官方 8 段素材 700 帧的 crosswalk 实测（128 个框）：
#   cy   中位 0.853，p10 0.739        ← 斑马线在路面下半部
#   area 中位 0.021，最大 0.353       ← 只有 3 个框超过 0.25
#   cy<0.45 的只有 2 个，且都是 1/3 画面的大框 + 低分（0.23~0.27）
# 所以「cy ≥ 0.45 且 area ≤ 0.25」只砍掉 3/128，留下的仍是 125 个真斑马线，
# 却能把板上那种"整幅顶部条"一刀切掉。
CROSSWALK_MIN_CY = 0.45
CROSSWALK_MAX_AREA = 0.25


def _crosswalk_in_view(scene, h=720, w=1280):
    """路面斑马线：位置在下半部、面积不占满整幅画面。

    `merge_crosswalk` 会把邻近的黄白标线并成一整块，所以面积上限必须留足
    （官方最大真斑马线 0.353），不能用"太大的就是错的"来卡。
    """
    if h <= 0 or w <= 0:
        return False
    for box in scene:
        if box['class_name'] != 'crosswalk':
            continue
        x1, y1, x2, y2 = box['xyxy']
        cy = (y1 + y2) * .5 / h
        area = (x2 - x1) * (y2 - y1) / (h * w)
        if cy >= CROSSWALK_MIN_CY and area <= CROSSWALK_MAX_AREA:
            return True
    return False


def _carry(colour):
    """A colour that may survive into a non-navigation branch.

    BLUE means "the route says go this way" and only the navigation branch can
    produce it. Letting it through `retained()` leaks a stale route arrow onto a
    lane or traffic frame, which reads as an instruction that no longer applies.
    """
    return None if colour == BLUE else colour


def _nav_label(class_name):
    """Split a nav6 label into (turn, digit count). 'right_3' -> ('right', 3)."""
    turn, sep, digits = class_name.rpartition('_')
    if not sep or not digits.isdigit():
        return None, None
    return turn, int(digits)


def eligible_navigation(nav_boxes):
    """Resolve the blue arrow from the six-class nav detections.

    nav6 fused direction and distance digit count into a single label, so one
    inference answers both questions and there is no OCR to confirm or hold.

        right_2 / left_2   <100 m, the turn is imminent -> that turn
        right_3 / left_3   >=100 m, still far           -> blue STRAIGHT
        uturn_*            FPGA has no U-turn enable bit -> blue STRAIGHT

    The 3-digit rule is unchanged from V15.9: a long-distance manoeuvre reads
    as "keep going straight", and the operator sees the turn arrow only when
    the countdown reaches two digits.
    """
    readings = []
    if not nav_boxes:
        return None, {'boxes': [], 'readings': readings,
                      'selected': None, 'reason': 'no_supported_arrow_detection'}
    box = max(nav_boxes, key=lambda b: b['score'])
    turn, digits = _nav_label(box['class_name'])
    if turn is None:
        return None, {'boxes': nav_boxes, 'readings': readings,
                      'selected': None, 'reason': 'unknown_nav_class'}
    if turn in ('left', 'right'):
        if digits == 2:
            selected, reason = turn, 'confirmed_two_digits_turn'
        else:
            selected, reason = 'straight', 'three_or_more_digits_straight'
    else:
        selected, reason = 'straight', 'uturn_not_supported_straight'
    readings.append({'class_name': box['class_name'], 'turn': turn,
                     'digits': digits, 'score': box['score'],
                     'selected': selected})
    return selected, {'boxes': nav_boxes, 'readings': readings,
                      'selected': selected, 'reason': reason}


def branch_family(branch):
    """本轮实际执行的分支 -> 模型族。None = 本轮没有可用结论。

    `traffic_hold` 归到 `traffic`（显示的仍是灯色）；`traffic_hold+lane`
    归到 `lane`——那一支的箭头是车道分支重画的。
    """
    if not branch:
        return None
    if branch == 'cop':
        return 'cop'
    if branch.startswith('navigation'):
        return 'navigation'
    if branch == 'traffic_hold+lane' or branch.startswith('lane'):
        return 'lane'
    if branch.startswith('traffic'):
        return 'traffic'
    return None


@dataclass
class BranchSwitch:
    """Detect that this round entered a different model line than the last one.

    Red / yellow / blue / green mean different things depending on which branch
    produced them: blue is "the route says go this way", yellow/red are lane
    markings, red/green are lights. Carrying a colour across a mode change makes
    the operator read a stale instruction in the new context (e.g. a blue route
    arrow still lit on a junction frame, or a yellow "may change lane" left on a
    red light).

    So a mode change starts from a blank screen: whatever the previous branch
    was saying no longer applies once the question itself changed.

    ⚠️ 2026-10-28：判断依据从「场景标签」改成「**实际跑了哪条分支**」。

    用户原话（2026-10-28 08:50）："在决策模型决策后，如果和决策前进入的模式
    不同，比如从导航进入红绿灯，或者从导航到车道，……加上清空当前所有箭头，
    相当于进入一个新的状态自然要清空原有的箭头信息。"

    按标签判的老写法有两个洞，第一次上板日志都兑现了：

      * `lane + traffic_light` 双标签帧被当作"无法判断"直接放过。但红绿灯
        优先级更高，这类帧实际上 116/118 轮都由灯箱分支吃掉了——于是
        「车道 → 红绿灯」和「红绿灯 → 车道」这一整类切换**从来不算切换**。
      * `traffic_hold` 与 `traffic` 之间、`traffic_hold+lane` 与 `lane`
        之间同样按标签看都是"同类"，但屏幕上的语义完全不同：前者是灯的颜色，
        后者是路面的标线。

    现在只看本轮真正执行的那一支。`traffic_hold` 归到 `traffic` 一族
    （它显示的仍是上一轮灯色）；`traffic_hold+lane` 归到 `lane`——那一支的
    箭头是车道分支重画的，底色不该算灯。
    """
    last: str = None
    switches: int = 0
    candidate: str = None
    streak: int = 0
    # 1 = 决策一变就清（2026-10-28 用户要求）。设 2 可挡住单轮混合帧造成的抖动，
    # 但代价是切换后的第一轮仍会带着旧状态，所以默认 1。
    family_stable_rounds: int = 1

    def family(self, branch):
        return branch_family(branch)

    def peek(self, branch):
        """会不会切换——只看，不改状态。用于分支内部提前避开旧状态。

        导航分支必须在"要不要沿用上一轮的蓝"这一步就知道本轮是不是切进来的：
        从车道切进导航时，那块蓝属于上一个状态，不能沿用它。
        导航在场景里优先级最高，`'nav' in targets` 就足以确定本轮跑的是导航，
        所以这里用标签是安全的（traffic/lane 才不能靠标签判断）。
        """
        fam = self.family(branch)
        if fam is None or self.last is None or fam == self.last:
            return False
        streak = self.streak + 1 if fam == self.candidate else 1
        return streak >= self.family_stable_rounds

    def observe(self, branch, provisional=False):
        """True when this round must discard the previous branch's arrows.

        `provisional=True` 表示这一支其实没有拿到自己的证据（灯箱解不出灯、
        车道两侧都没有线），只是沿用了上一轮的显示。这种轮次不能算作
        "进入新状态"，否则一次漏检就会把好好的灯态/车道态清空。
        """
        fam = None if provisional else self.family(branch)
        if fam is None:
            if provisional and self.last is None:
                # 开机头几轮如果都是"沿着上一轮"，至少把族记下来，
                # 否则第一轮真证据到来时又会被当成"没有上一次"。
                self.last = self.family(branch)
            return False            # 无结论的一轮不算切换，等它自己恢复
        if fam == self.last:
            self.candidate, self.streak = None, 0
            return False
        if fam == self.candidate:
            self.streak += 1
        else:
            self.candidate, self.streak = fam, 1
        if self.last is not None and self.streak < self.family_stable_rounds:
            # 新族还没连续出现够久：视为噪声，不判定切换
            return False
        previous, self.last = self.last, fam
        self.candidate, self.streak = None, 0
        if previous is None:        # 程序启动后的第一轮，没有"上一次"可清
            return False
        self.switches += 1
        return True


@dataclass
class EmptyStreak:
    """连续多少帧场景模型什么都没检到。

    2026-10-29：`--scene-retries` 每次空返回都重问一次，代价是**真·空白画面**
    （镜头对着墙、黑屏、视频切换的黑场）每轮多跑一次 480ms 的场景推理，
    单轮从 ~700ms 涨到 ~1200ms。用户反馈"实际看起来慢"就是这个原因之一。

    所以只在「刚刚变空」的头几轮重问——那才是"本该有东西却没检到"的高发区。
    连续空了 `retry_frames` 轮之后就不再重问，直接按空画面处理（清屏）。
    """
    retry_frames: int = 3
    streak: int = 0

    def should_retry(self):
        return self.streak < self.retry_frames

    def observe(self, found):
        self.streak = 0 if found else self.streak + 1
        return self.streak

    def reset(self):
        self.streak = 0


@dataclass
class LastGood:
    """最后一次「屏幕上真的有颜色」的显示，按分支族记住。

    2026-10-28 板上日志：车道 155 轮里 0 轮全黑，但 `lane_hold` 17 轮里有
    5 轮全黑（29%）；`traffic_hold` 191 轮里 75 轮全黑（39%）。原因不是模型
    没检到，而是这些 hold 分支只回退到**上一轮**——而上一轮可能本来就是黑的，
    于是一次漏检就把屏幕永久留空，直到下一帧真的解出东西为止。

    用户的原话是「要保证不要出现没有箭头的情况」。所以记住每个族**最近一次
    有颜色**的显示，hold 分支在黑屏时回退到它，而不是回退到上一轮。
    切换分支时清空（旧状态的颜色在新语境里含义不同，见 BranchSwitch）。
    """
    nav: object = None
    traffic: object = None
    lane: object = None
    age: dict = None

    def __post_init__(self):
        if self.age is None:
            self.age = {'navigation': 0, 'traffic': 0, 'lane': 0}

    @staticmethod
    def has_colour(disp):
        return disp is not None and any(
            c is not None for c in (disp.left, disp.straight, disp.right))

    def remember(self, family, disp):
        if family is None or not self.has_colour(disp):
            return
        setattr(self, family, disp)
        self.age[family] = 0

    def forget(self, family):
        if family in self.age:
            setattr(self, family, None)
            self.age[family] = 0

    def recall(self, family, limit=0):
        """拿回该族最近一次有颜色的显示；超过 limit 轮就当它过期。"""
        if family is None:
            return None
        if limit and self.age.get(family, 0) > limit:
            return None
        return getattr(self, family, None)

    def tick(self):
        for k in self.age:
            self.age[k] += 1


@dataclass
class SceneEmptyHold:
    """Hold arrows across one-off scene classifier misses before clearing.

    Only the absence of ALL three nav/traffic/lane labels can count toward
    clearing. A present label resets the counter; invalid submodels cannot
    increment it. Use confirm_frames=1 to clear immediately (strict mode).
    """
    confirm_frames: int = 2
    empty: int = 0

    def __post_init__(self):
        if type(self.confirm_frames) is not int or self.confirm_frames < 1:
            raise ValueError('confirm_frames must be positive integer')
    def observe(self, targets):
        if targets:
            self.empty = 0
            return False
        self.empty = min(self.confirm_frames, self.empty + 1)
        return self.empty >= self.confirm_frames


@dataclass
class WarningHold:
    """Multi-frame confirmation for the yellow edge-warning bit.

    The warning is latched to the FPGA on every request, so a single-frame
    false `crosswalk` box lights it immediately. Requiring N consecutive
    positives drops that to roughly p**N while keeping the latency cost at
    (N-1) rounds, which is acceptable for a warning that only drives braking
    advice. Set confirm_frames=1 to restore the immediate behaviour.

    2026-10-31 V26：确认帧数 2 → 1（用户要求"斑马线灵敏度调高"），同时
    加上 `max_hold` 上限。板上日志里报警一共 132 轮，锁存段长度是
    49/30/30/23 轮——一旦置位就不再基于新证据刷新，场景缓存 30 轮复用
    同一帧，于是一次误检亮一两秒。`max_hold` 让它在没有新证据时自动
    放开，重新走确认流程。
    """
    confirm_frames: int = 1
    max_hold: int = 4
    streak: int = 0
    latched: bool = False
    held: int = 0

    def __post_init__(self):
        if type(self.confirm_frames) is not int or self.confirm_frames < 1:
            raise ValueError('warning confirm_frames must be positive integer')
        if type(self.max_hold) is not int or self.max_hold < 1:
            raise ValueError('warning max_hold must be positive integer')

    def observe(self, present):
        if present:
            self.streak = min(self.confirm_frames, self.streak + 1)
            if self.streak >= self.confirm_frames:
                self.latched = True
                self.held = 0
            return self.latched
        self.streak = 0
        # 证据消失：先按 max_hold 沿用，超过就放开，要求重新确认。
        if self.latched:
            self.held += 1
            if self.held < self.max_hold:
                return True
        self.latched = False
        self.held = 0
        return False


@dataclass
class NavigationHold:
    """Keep a blue arrow lit while the scene classifier briefly sees nothing.

    A blue arrow says "stay on this route", which a few rounds of silence does
    not contradict. The lane/traffic colours say "this side of the road is
    solid/dashed" or "this light is red", and those are re-read every round, so
    they keep the short SceneEmptyHold rule.

    Measured on rk3568_v15_10.log: 52 of 108 navigation rounds were dark,
    because 2 consecutive scene misses tripped SceneEmptyHold. Holding for
    `hold_frames` rounds drops that to 8.

    hold_frames=0 restores the old behaviour. Only a BLUE arrow qualifies.
    """
    hold_frames: int = 10
    misses: int = 0

    def __post_init__(self):
        if type(self.hold_frames) is not int or self.hold_frames < 0:
            raise ValueError('nav hold_frames must be a non-negative integer')

    def reset(self):
        self.misses = 0

    def observe(self, nav_present):
        """Return True while a previously lit blue arrow should stay lit."""
        if nav_present:
            self.misses = 0
            return True
        self.misses += 1
        return self.misses <= self.hold_frames


@dataclass
class NavDirectionHold:
    """Debounce the nav6 direction family so the turn arrow stops blinking.

    The model splits each turn into a 2-digit and a 3-digit class, and the two
    sit right on the 100 m boundary. On the board log (V17 gray, idx150-199) the
    label alternated right_3 -> right_2 -> right_3 -> right_2 across consecutive
    frames, so the straight arrow and the turn arrow traded places every round.
    The flipping frames were the weak ones (0.49 / 0.53 / 0.78) while the stable
    frames scored 0.90+.

    So: keep showing the current direction until a different one is seen
    `confirm_frames` times in a row, OR once in a single high-confidence frame.
    A confident reading should never be delayed -- the delay only exists to
    reject boundary noise.

    Only the direction family is debounced, not the digit count: a turn that
    moves from 3 digits to 2 is still "turn right" and must not flicker.
    """

    confirm_frames: int = 2
    confident: float = 0.85
    shown: str = None            # direction currently on screen
    candidate: str = None
    streak: int = 0

    def __post_init__(self):
        if type(self.confirm_frames) is not int or self.confirm_frames < 1:
            raise ValueError('nav confirm_frames must be a positive integer')

    def reset(self):
        self.shown = None
        self.candidate = None
        self.streak = 0

    def observe(self, direction, score):
        """Return the direction the display should use this round."""
        if direction is None:
            return self.shown
        if score >= self.confident:
            self.shown, self.candidate, self.streak = direction, None, 0
            return direction
        if direction == self.shown:
            self.candidate, self.streak = None, 0
            return direction
        if direction == self.candidate:
            self.streak += 1
        else:
            self.candidate, self.streak = direction, 1
        if self.streak >= self.confirm_frames:
            self.shown, self.candidate, self.streak = direction, None, 0
            return direction
        # Not confirmed yet: hold what is already on screen. With nothing on
        # screen there is nothing to hold, so show the reading.
        return self.shown if self.shown is not None else direction


@dataclass
class TrafficHoldLimit:
    """Stop showing a light colour once it has gone unconfirmed for too long.

    `traffic_hold` re-asserts the last resolved light while the housing model
    cannot resolve one. That is right for a flicker, but the board log showed it
    running 56.9 s (idx366-429) with only 12 of 64 rounds actually confirming a
    green -- about 47 s of video. If the light changes during that stretch the
    display keeps saying GO.

    A stale GO is the dangerous direction, so a green expires faster than a red.
    Expiry does not invent a colour: it drops the arrow, which the protocol
    reads as "not being told to go", and the warning bit is unaffected.

    The limits count TOTAL consecutive rounds a colour may be displayed:
    the confirming round counts as round one, then `expire()` allows
    limit-1 more before returning None.

    hold_frames=0 disables the limit (the pre-2026-10-28 behaviour).
    """

    hold_frames: int = 15
    green_frames: int = 6
    stale_green: int = 0
    stale_red: int = 0

    def __post_init__(self):
        if type(self.hold_frames) is not int or self.hold_frames < 0:
            raise ValueError('traffic hold_frames must be a non-negative integer')
        if type(self.green_frames) is not int or self.green_frames < 0:
            raise ValueError('traffic green_frames must be a non-negative integer')

    def reset(self):
        self.stale_green = 0
        self.stale_red = 0

    def observe(self, colours):
        """Take the colours a resolved round actually produced."""
        self.reset()
        if GREEN in colours:
            self.stale_green = 1
        elif RED in colours:
            self.stale_red = 1

    def expire(self, colour, bump=True):
        """Extend the stale counter for a colour we are about to keep showing.

        Returns the colour to show: unchanged while within the limit, else None.

        ⚠️ `bump=False` 只查询、不计数。2026-10-29 审查发现调用方对左/直/右
        三个方向各调一次 `expire()`，而"1 框"布局会把同一种颜色铺满三个方向
        ——计数器一轮涨 3，红灯的上限（15 轮）实际只剩 5 轮就熄。
        现在改成：同一轮里同一个颜色只推进一次。
        """
        if colour is None or self.hold_frames <= 0:
            return colour
        if colour == GREEN:
            if bump:
                self.stale_green += 1
            if self.stale_green > self.green_frames:
                return None
        elif colour == RED:
            if bump:
                self.stale_red += 1
            if self.stale_red > self.hold_frames:
                return None
        return colour

    def expire_display(self, disp):
        """对一块屏上的三个方向做过期判断，每个颜色每轮只计数一次。

        ⚠️ 2026-10-29 用户硬性要求：**红绿灯模式下每时每刻必须同时有三个箭头**。
        旧实现逐侧过期，于是出现"灯不变、箭头从右到直再到左依次消失"：

            第266轮 无框 -> L=red  S=red  R=green
            第271轮 无框 -> L=red  S=red  R=hidden   ← 绿先超时（上限 6 轮）
            第273轮 无框 -> L=red  S=hidden R=hidden ← 直/红再超时（上限 15 轮）

        所以过期改成**捆绑判定**：三个方向一起过期、一起保留。只要还有一个
        方向在时限内，三个就都留着。要熄就整块熄，不允许出现半个灯。
        """
        colours = [getattr(disp, side) for side in ('left', 'straight', 'right')]
        seen = set()
        alive = {}
        for colour in colours:
            if colour is None:
                continue
            first = colour not in seen
            seen.add(colour)
            alive[colour] = self.expire(colour, bump=first)
        # 捆绑：任一方向过期，整块一起过期。绝不逐侧熄灭。
        any_expired = any(c is not None and alive.get(c) is None
                          for c in colours)
        if any_expired:
            return {side: None for side in ('left', 'straight', 'right')}
        return {side: (alive.get(getattr(disp, side))
                       if getattr(disp, side) is not None else None)
                for side in ('left', 'straight', 'right')}


@dataclass
class LayoutHold:
    """框数决定方向语义，所以框数抖动必须挡掉。

    `traffic_rules` 按框数分配方向：1 框 = 左+直同色，2 框 = 左+直，
    3 框 = 左+直+右。同一个路口上灯箱模型会在 1/2/3 框之间来回跳
    （v17_5 那段 192 轮：1框 30 / 2框 6 / 3框 9，且交错出现）。
    **框数一变，左/直/右的颜色归属整体错位**，屏幕上就是"红绿灯乱跳"。

    所以：新框数必须连续 `confirm_frames` 轮出现才采纳。
    只有 1 轮不相同 = 当作模型抖动，继续按上一次的框数解释。

    0 框不走这里：那是 NoBoxHold 的职责（它管"灯是不是没了"），
    而框数抖动这里管"这盏灯属于哪个方向"。
    """

    confirm_frames: int = 2
    count: int = 0           # 采纳的框数
    candidate: int = 0
    streak: int = 0

    def observe(self, boxes):
        """返回本轮该按几个框来解释方向（上限 3：超出部分 traffic_rules 不用）。"""
        n = min(len(boxes), 3)
        if n == 0:
            # 无框不是"布局变了"，交给 NoBoxHold；这里保持当前布局。
            self.candidate, self.streak = 0, 0
            return self.count
        if n == self.count:
            self.candidate, self.streak = 0, 0
            return self.count
        if self.count == 0:
            # 还没定过布局：第一帧直接采纳，不然每个路口的头几轮都是空的。
            self.count, self.candidate, self.streak = n, 0, 0
            return self.count
        if n == self.candidate:
            self.streak += 1
        else:
            self.candidate, self.streak = n, 1
        if self.streak >= max(1, self.confirm_frames):
            self.count, self.candidate, self.streak = n, 0, 0
        return self.count

    def reset(self):
        self.count = self.candidate = self.streak = 0


@dataclass
class NoBoxHold:
    """无框时先沿用上一轮灯色，连续 N 轮都没有框才清零。

    2026-10-29 用户定：单轮无框不足以判定"灯没了"（灯箱模型偶尔漏检一次很正常），
    但一直挂着已经消失的灯也不行。所以给一个短窗口：

        第1轮无框 -> 沿用上一轮灯色
        第2轮无框 -> 沿用
        第3轮无框 -> 清零

    窗口按"连续"计，中间只要有一帧重新检出框就归零。
    hold_frames=0 表示永远沿用（旧的 traffic_hold 行为）。
    """
    hold_frames: int = 3
    streak: int = 0

    def reset(self):
        self.streak = 0

    def observe(self, has_box):
        if has_box:
            self.streak = 0
            return True
        self.streak += 1
        if self.hold_frames <= 0:
            return True
        return self.streak < self.hold_frames


# 真灯箱在 1280×720 上至少这么大：板上实测真灯箱 27×61 ~ 63×135（≈1600~8500 像素），
# 而高速路/远景里的误检是 10×20 ~ 22×47（≈200~1000 像素）。
# 2026-10-29 高速路幻影红绿灯就是卡在这里：场景模型给了 0.90 分的"灯"，
# 灯箱模型把这些小块也当框交上来，判色一律 off → 三个红灯，并长期继承。
MIN_HOUSING_PIXELS = 700


@dataclass
class TrafficEvidenceGuard:
    """红绿灯分支只有在**灯箱模型真的给出灯箱大小的框**时才允许接管。

    ## 为什么需要它（2026-10-29 高速路幻影红绿灯）

    场景模型（merged5）在高速路上把隔离墩/路灯/远处的标志稳定判成
    `traffic_light`（0.86~0.93，连续 192 轮）。而红绿灯分支的入口条件只是
    「场景里有 traffic_light 标签」，于是整段高速都跑红绿灯分支：

      * 灯箱模型交上来的框是 10×20 的非灯小方块，判色一律 `off`；
      * `off` 按红灯处理 → 屏幕三红；
      * 没有框的轮次落进 `traffic_hold` / `last_good`，继续把上一轮红灯抄下来；
      * 场景答案又被 `scene-every 2` 缓存复用，**模型没机会自我纠正**。

    结果就是用户看到的「无缘无故出现红绿灯，并且长期保持」。

    ## 判据

    只有面积 ≥ `MIN_HOUSING_PIXELS` 的框才算"这里有真灯"。真灯一旦出现，
    本闸门进入 `active`；此后要连续 `release_frames` 轮都拿不到灯箱大小的框，
    才放行退出（真路口灯箱漏检最长实测 9 轮，v17_1；这里取 12 留余量）。

    ## 为什么退出是粘性的

    高速上没有真灯，所以退出后再也不会被激活，幻影红绿灯从出现到消失
    **只发生一次**（约 release_frames 轮），而不是一直挂到视频结束。
    程序启动就在高速上时 `active` 一开始就是 False，幻影根本不会出现。
    """

    active: bool = False
    streak: int = 0
    release_frames: int = 12
    min_pixels: int = MIN_HOUSING_PIXELS

    def observe(self, boxes):
        """-> 'pass'（进红绿灯分支）/ 'hold'（还没退出，进分支但会全红）/
        'release'（不是路口，让给车道/空分支）。"""
        sized = [b for b in boxes
                 if (b[2] - b[0]) * (b[3] - b[1]) >= self.min_pixels]
        if sized:
            self.active, self.streak = True, 0
            return 'pass'
        if not self.active:
            return 'release'
        self.streak += 1
        if self.release_frames > 0 and self.streak < self.release_frames:
            return 'hold'
        self.active, self.streak = False, 0
        return 'release'

    def reset(self):
        self.active, self.streak = False, 0


def _lit_housing(boxes, yrg):
    """只保留**里面真的有纯红/纯绿像素**的灯箱框。

    颜色判定和 traffic_rules._colour_of 用同一套规则（数 254/255），
    这样"闸门认为有灯"和"判色时能读到灯"是同一件事，不会互相矛盾。
    """
    out = []
    h, w = yrg.shape[:2]
    for b in boxes:
        x1 = max(int(round(b[0])), 0); y1 = max(int(round(b[1])), 0)
        x2 = min(int(round(b[2])), w); y2 = min(int(round(b[3])), h)
        if x2 - x1 < 2 or y2 - y1 < 2:
            continue
        c = yrg[y1:y2, x1:x2]
        if (c == 254).sum() >= traffic_rules.MIN_COLOUR_PIXELS \
                or (c == 255).sum() >= traffic_rules.MIN_COLOUR_PIXELS:
            out.append(b)
    return out


@dataclass
class SceneThrottle:
    """Reuse the scene result for a few cycles to cut the 1024x1024 inference.

    On a clean lane-only video the scene classifier returns {lane} every cycle
    and costs ~310 ms of NPU time to say nothing new. With every>1 only every
    N-th cycle actually runs it; the cached result is reused in between.

    Cache expires after `max_hold` reuses so a scene change cannot be missed
    indefinitely. Any exception in the scene model does not touch the cache.

    Navigation used to be exempt from caching entirely, because a cached miss
    blanks the route arrow. That cost a full scene inference (468 ms on the
    board) on every navigation round. `NavigationHold` now covers the miss case,
    so a navigation answer is cached too, but only for `nav_max_hold` reuses and
    only until the nav model itself comes back empty -- `force_next()` is called
    then, so the round after any nav miss pays for a real scene pass. The goal
    is to stop paying 468 ms to re-confirm the same route screen, not to let a
    junction go unnoticed for long.
    """
    every: int = 2
    max_hold: int = 30
    nav_max_hold: int = 2
    # ⚠️ 2026-10-31 V26：30 → 2。用户原话「决策模型可以改成两轮一触发」，
    # 并对红绿灯/斑马线/交警的粘滞提出了明确意见。
    # V25 的 30 轮复用导致三个连锁问题（全部有板上日志证据）：
    #   * 场景真跑率只有 15.7%（103/655），同一帧的路口表情被复制 30 次；
    #   * 斑马线报警锁存 49/30/30/23 轮 = 一两秒不灭，其实是同一帧；
    #   * 交警手势判定每次都是在缓存的旧帧上做。
    # 退出红绿灯本来就不靠场景模型（靠 TrafficEvidenceGuard），复用没有
    # 安全上的必要性，只是当初为了省 520ms。现在与 --scene-every 对齐。
    traffic_max_hold: int = 2
    cycle: int = 0
    held: int = 0
    cached: object = None
    forced: bool = False

    def __post_init__(self):
        if type(self.every) is not int or self.every < 1:
            raise ValueError('scene_every must be a positive integer')
        if type(self.max_hold) is not int or self.max_hold < 0:
            raise ValueError('scene_max_hold must be >= 0')
        if type(self.nav_max_hold) is not int or self.nav_max_hold < 0:
            raise ValueError('scene_nav_max_hold must be >= 0')

    def _cached_is_nav(self):
        return any(isinstance(b, dict) and b.get('class_name') == 'nav'
                   for b in (self.cached or ()))

    def _cached_is_traffic(self):
        """缓存答案里有红绿灯 -> 可以复用（`lane+traffic_light` 双标签也算）。

        ⚠️ 2026-10-29 V20 上板回归：用户报「转换场景还是响应太慢」，
        而这份日志 26 轮 `scene_cached` **全是 false**，每轮白付 520ms。

        原因是我在 V20 把判据写成 `'traffic_light' in names and 'lane' not in
        names`：路口帧几乎都是 `lane + traffic_light` 双标签，于是这个豁免
        从来没生效过。**每轮 1160ms 里有 520ms 是重复确认"这还是路口"**。

        退出红绿灯不靠场景模型，靠 `TrafficEvidenceGuard`：
        灯箱再也给不出灯箱尺寸的框 → 判 release → 去掉 traffic_light 标签、
        并 `force_next()` 强制下一轮真跑场景。所以复用这个答案是安全的。
        """
        names = {b.get('class_name') for b in (self.cached or ())
                 if isinstance(b, dict)}
        return 'traffic_light' in names

    def force_next(self):
        """Make the next round run the scene model for real."""
        self.forced = True

    def should_run(self):
        if self.cached is None or self.forced:
            return True
        # ⚠️ 2026-10-28 板上日志：**空答案绝不能被复用**。
        # 第 729 轮起连续 202 轮全黑，原因就是这个缓存把一个"什么都没有"的
        # 场景结果反复喂下去：模型每 2 轮真跑一次（耗时 545ms，证明图是好的），
        # 但中间那轮复用空答案，屏幕上就什么都没有。
        if not self.cached:
            return True
        if self.held >= self.max_hold:
            return True
        # A cached navigation answer hides a real junction, so it may only be
        # reused for a couple of rounds, and every nav miss forces a re-run.
        if self._cached_is_nav():
            # 2026-10-29 提速：导航画面本身很稳定（地图界面几乎不会"变成"别的
            # 场景），而每确认一次要白付 ~485ms 的场景推理。板上 437 轮里
            # 导航轮花了 20.8 秒在重复确认"这还是导航"。
            # 仍然保留上限：导航模型一旦空返回就 force_next()，那才是真的
            # 需要重新看场景的时刻。
            return self.held >= self.nav_max_hold
        if self._cached_is_traffic():
            # 见 traffic_max_hold 的说明：红绿灯不靠场景模型进出，
            # 每轮重跑只会多付 520ms。这里给它单独的上限。
            return self.held >= self.traffic_max_hold
        return (self.cycle % self.every) == 0

    def observe(self, scenes):
        self.cached = scenes
        self.held = 0
        self.forced = False
        self.cycle += 1
        return scenes

    def reuse(self):
        self.held += 1
        self.cycle += 1
        return self.cached


def decide_pair(gray, detectors, person_y=.55, person_x=.25,
                previous_display=None, lane_streaks=None, lane_dash_ratio=.05,
                empty_scene_hold=None, lane_backend='v2',
                lane_roi_top=.44, lane_roi_bottom=.97, lane_row_step=3,
                lane_band_top=0.10, lane_band_bot=0.90,
                lane_chain_min_rows=3, lane_dash_frac=.5,
                scene_throttle=None, warning_hold=None, nav_hold=None,
                branch_switch=None, traffic_hold_limit=None,
                nav_direction_hold=None, frame_is_thresholded=False,
                last_good=None, last_good_hold=0,
                last_good_traffic_hold=None, scene_retries=0,
                empty_streak=None, no_box_hold=None,
                traffic_guard=None, layout_hold=None,
                cop_signature=None):
    """YRG 灰度帧 -> 显示状态 + 诊断。

    lane_backend='v2' 走新车道模型（预处理和几何解码都在 lane_v2.py）；
    'angle' 是旧连通域算法，只保留做对照，它才用到 lane_dash_ratio 等扫描参数。

    `frame_is_thresholded=True` 表示这一帧的取值集合塌缩成 {0,253} ——
    FPGA 那边的模式 1 还没把 `median_data`（真灰度）接出来，回来的其实是
    阈值图。这不是 RK 端二值化的结果，而是上游给错了图。灯箱框照样能检，
    但框内数不到 254/255，红绿灯会全判 off —— 与其显示一个假的"全停"，
    这一轮不判灯，落到 traffic_hold 沿用上一轮（错误汇集第 7 条）。
    """
    if gray.shape != (720, 1280):
        raise ValueError('gray image must be 720x1280')
    if lane_streaks is None:
        lane_streaks = LaneSolidStreaks(3)
    old = previous_display if previous_display is not None else Display()
    t = time.perf_counter()
    scene_cached = False
    scene_retried = 0
    if scene_throttle is not None and not scene_throttle.should_run():
        scenes = scene_throttle.reuse()
        scene_cached = True
    else:
        scenes = detectors.scene(gray)
        # ⚠️ 2026-10-28 板上日志：一次"什么都没检到"不能直接接受。
        # 车道/红绿灯画面很清晰却整屏没箭头，就是因为这一帧场景模型空返回，
        # 于是三条分支全都不跑。空答案再问一次（最多 --scene-retries 次），
        # 仍然空才认。真·空画面多花一次 480ms，比黑屏强。
        allow_retry = empty_streak is None or empty_streak.should_retry()
        while scenes == [] and scene_retried < scene_retries and allow_retry:
            scene_retried += 1
            scenes = detectors.scene(gray)
        if scene_throttle is not None:
            scene_throttle.observe(scenes)
    timing = {'scene_model_ms': round((time.perf_counter()-t)*1000, 2),
              'scene_cached': scene_cached}
    if scene_retried:
        timing['scene_retries'] = scene_retried
    if empty_streak is not None and not scene_cached:
        timing['scene_empty_streak'] = empty_streak.observe(bool(scenes))
    # 交警手势的判定放"计算"这一步（紧接场景模型），但**早退**必须等
    # finish() 定义出来之后再做 —— 见下面 `cop_kind is not None` 那一段。
    cop_state = None
    cop_kind = None
    if cop_signature is not None:
        # 判据是手势分类模型（cls_v2yrg3）对本帧 YRG 的输出。
        # 模型没装（cop=None）时 cop_classify 返回 None，按 miss 处理，
        # 永远不会进入交警模式 —— 与"关掉该分支"等价，不会静默失效成误触发。
        #
        # ⚠️ observe() 也必须包在 try 里。kind_from_logits() 对非 3 类
        # 输出会 raise ValueError；若只包住 cop_classify，形状错误会穿透到
        # run_live 的 infer_fail_streak，连续 20 轮后触发程序重启。交警模型是
        # 可选件，任何异常都只能让它本轮按 miss 处理。
        cop_logits = None
        cop_ran = False
        cop_error = None
        try:
            cop_logits = detectors.cop_classify(gray)
            cop_ran = cop_logits is not None
            cop_kind = cop_signature.observe(cop_logits)
        except Exception as exc:
            # 错误必须可见——每轮都写进 cop_signature.error，板上日志里
            # 连续出现同一个 error 就是模型/运行时出了问题。
            cop_error = type(exc).__name__
            cop_kind = cop_signature.observe(None)
        cop_state = cop_signature.snapshot()
        cop_state['ran'] = cop_ran
        if cop_error is not None:
            cop_state['error'] = cop_error
    names = {box['class_name'] for box in scenes}
    # 2026-10-31 V26：斑马线不再只看"类名里有 crosswalk"。
    # 板上 132 轮报警里绝大多数是同一帧的整幅顶部条 [0,0,1178,132]
    # （cy 0.09 / area 30%），被场景缓存 30 轮 + WarningHold 锁存放大成
    # 一两秒的假报警。几何判据见 _crosswalk_in_view。
    raw_warning = (_crosswalk_in_view(scenes, gray.shape[0], gray.shape[1])
                   if scenes else False) or _person_in_front(
                       scenes, y_min=person_y, x_min=person_x)
    if warning_hold is not None:
        warning = warning_hold.observe(raw_warning)
        warning_streak = warning_hold.streak
    else:
        warning = raw_warning
        warning_streak = None
    targets = names.intersection({'nav', 'traffic_light', 'lane'})
    # ⚠️ 2026-10-29 高速路幻影红绿灯：场景标签说"有红绿灯"**不足以下结论**。
    # 场景模型会把高速上的隔离墩/路灯判成 traffic_light（0.9 分，连续上百轮）。
    # 真灯必须有灯箱模型给出灯箱大小的框（见 TrafficEvidenceGuard）。
    traffic_scene_seen = 'traffic_light' in targets
    traffic_verified = traffic_scene_seen
    traffic_guard_state = None
    if traffic_scene_seen and traffic_guard is not None and 'nav' not in targets:
        try:
            probe = detectors.housing(gray)
            probe_error = None
        except Exception as exc:
            probe, probe_error = [], type(exc).__name__
        # ⚠️ 2026-10-29 用户：「识别到一个框的三红三绿」。
        # 一个框里数不到任何纯红/纯绿像素，说明**这盏灯根本没亮**，
        # 或者这个框根本不是灯箱。1 框布局会把它铺成三个方向的红
        # （用户看到的就是"一个框判出三个红灯"）。
        # 所以闸门要的证据不是"有框"，而是"**有亮着的灯**"。
        lit = _lit_housing(probe, gray) if probe else []
        traffic_guard_state = traffic_guard.observe(lit)
        result_guard = {'state': traffic_guard_state,
                        'boxes': len(probe),
                        'lit_lamps': len(lit),
                        'active': traffic_guard.active,
                        'streak': traffic_guard.streak,
                        'downgraded': traffic_guard_state == 'release'}
        if traffic_guard_state == 'release':
            # 场景说红绿灯、灯箱说没有真灯：这是一帧路面画面。
            # 去掉 traffic_light，让车道分支接手（场景答案不缓存，见 force_next）。
            targets = targets - {'traffic_light'}
            if 'lane' not in targets:
                targets = targets | {'lane'}
            traffic_verified = False
            if scene_throttle is not None:
                # 幻影标签会被 scene-every 缓存一直复用，模型永远没机会改口。
                scene_throttle.force_next()
        if probe_error is not None:
            result_guard['error'] = probe_error
        # 上面这次探针已经把灯箱模型跑过了，正式分支里不要再跑一次。
        prefetched_housing = probe if traffic_verified else None
    else:
        result_guard = None
        prefetched_housing = None
    if traffic_scene_seen and traffic_guard is None:
        # 没传闸门：保持旧行为，入口条件仍是场景标签（对照用）。
        traffic_verified = True

    # 切换只能在知道"本轮真正跑了哪条分支"之后才判定（2026-10-28）：
    # 按标签判会把 116/118 轮被灯箱抢走的双标签帧漏掉，见 BranchSwitch 说明。
    # 本轮的切换判定放在 `finish()` 里做，拿到 branch 之后再调 branch_switch。
    switched = False
    # Yellow means "this marking is dashed, you MAY change lanes" and red means
    # "solid, you may NOT": both read the lane painting. They are meaningless
    # once a traffic light owns the frame, because the lanes underneath are
    # being re-drawn by the junction. Clear them instead of carrying two
    # different yellow/red semantics onto one screen. Checked here, before the
    # branch dispatch, because the traffic branch intentionally falls through
    # to the lane branch when it cannot resolve the lights.
    stale_lane_markings = ('traffic_light' in targets and 'nav' not in targets)
    lane_streaks_at_entry = lane_streaks
    # 本轮分支没有拿到自己的证据、只是沿用上一轮显示时，置一个标记。
    # 目前只有「车道两侧都没解出线」这一支会置它。
    provisional_holds = []
    result = {'scenes': scenes, 'branch': 'none', 'traffic': None,
              'navigation': None, 'lane': None, 'edge_warning': warning,
              'edge_warning_raw': raw_warning,
              'warning_streak': warning_streak,
              'branch_switched': False,
              'scene_targets': sorted(targets), 'timing_ms': timing,
              'traffic_scene_seen': traffic_scene_seen,
              'traffic_verified': traffic_verified,
              'traffic_guard': result_guard,
              'cop_signature': cop_state,
              'algorithm': 'v6_v7_angle'}

    def finish(disp, branch, reason, fresh=None):
        """统一出口：先判切换，切换成立就把上一次的状态清掉。

        2026-10-28 用户要求：「每次决策模型改变决策以后就记得清掉上一次的所有
        状态」。清的是**上一次留下来的**东西：

          * 上一支的箭头（屏幕上的红/黄/绿/蓝含义随分支而变）
          * 各分支自己的迟滞记忆（实线计数、灯色沿用计时、导航方向去抖）

        但本轮分支**自己刚算出来的**颜色必须留着——否则每次切换都会黑屏一轮。
        `fresh` 就是本轮新鲜算出的方向集合；切换发生时集合外的方向一律清空。
        """
        nonlocal switched
        provisional = bool(provisional_holds)
        # 纯沿用上一轮显示的轮次（灯箱解不出灯、车道两侧无证据）不是"决策
        # 变了"，不参与切换判定；但程序刚启动那几轮要让它把族记下来，
        # 否则第一次真正的切换会被当成"没有上一次"而漏掉。
        if provisional and branch_switch is not None and branch_switch.last is None:
            provisional = False
        switched = (branch_switch.observe(branch, provisional=provisional)
                    if branch_switch is not None else False)
        result['branch_switched'] = switched
        if switched:
            lane_streaks_at_entry.reset()
            if traffic_hold_limit is not None:
                traffic_hold_limit.reset()
            if nav_direction_hold is not None:
                nav_direction_hold.reset()
            keep = fresh if fresh is not None else ()
            disp = Display(left=disp.left if 'left' in keep else None,
                           straight=disp.straight if 'straight' in keep else None,
                           right=disp.right if 'right' in keep else None,
                           edge_warning=disp.edge_warning)
        result['branch'] = branch
        result['display_reason'] = reason
        # ⚠️ 2026-10-29 用户硬性要求：**红绿灯模式下每时每刻必须同时有三个箭头。**
        # 这是捆绑性条件，不是"尽量"。任何 traffic* 出口只要不是三个齐全，
        # 就先按"缺的那个沿用上一轮/记忆"补齐；实在补不上就用红灯兜底
        # （红灯=停止，是安全侧），绝不允许出现半个灯或全空。
        if branch_family(branch) == 'traffic' and branch != 'traffic_hold+lane':
            # 2026-10-29 用户硬性要求：红绿灯模式**每时每刻**必须三个箭头，
            # 这是捆绑条件，连"无框清零"也不例外。所以这里不设例外。
            # 补不上就用红灯兜底：红灯=停止，是安全侧。
            missing = [k for k in ('left', 'straight', 'right')
                       if getattr(disp, k) is None]
            if missing:
                filler = last_good.recall('traffic', limit=0)                     if last_good is not None else None
                fixed = {}
                for k in ('left', 'straight', 'right'):
                    v = getattr(disp, k)
                    if v is None and filler is not None:
                        v = getattr(filler, k)
                    if v is None:
                        v = RED
                    fixed[k] = v
                disp = Display(left=fixed['left'], straight=fixed['straight'],
                               right=fixed['right'],
                               edge_warning=disp.edge_warning)
                result['traffic_arrows_filled'] = missing
                reason += '+arrows_forced_three'
                result['display_reason'] = reason
                if last_good is not None:
                    last_good.remember('traffic', disp)
        # 2026-10-28 用户要求「要保证不要出现没有箭头的情况」。
        # 记住每个族最近一次真的有颜色的显示；切换分支时清掉该族的记忆。
        if last_good is not None:
            last_good.tick()
            fam = branch_family(branch)
            if switched and fam is not None:
                last_good.forget(fam)
            elif fresh and LastGood.has_colour(disp):
                # ⚠️ 只有**本轮自己算出颜色**才刷新记忆。
                # 2026-10-29 审查：原来只要屏上有颜色就 remember()，于是
                # 回填出来的旧绿灯（或 retained 沿用的旧灯色）下一轮又会被
                # 当成"新鲜颜色"记住，年龄归零 —— LastGood 变成了永动机，
                # 过期的绿灯可以无限亮下去。`fresh` 为空/None 说明这一轮
                # 没有任何新证据，只是沿用。
                last_good.remember(fam, disp)
            elif fam is not None:
                limit = last_good_hold
                if fam == 'traffic' and last_good_traffic_hold is not None:
                    limit = last_good_traffic_hold
                fell_back = last_good.recall(fam, limit=limit)
                if fell_back is not None:
                    # ⚠️ 红绿灯族回填之前必须先过 TrafficHoldLimit。
                    # 2026-10-29 审查：回填发生在 expire() 之后，一旦回填成功，
                    # 下一轮 expire(None) 不再计数、age 也不重置 —— 过期的绿灯
                    # 会**无限亮下去**，正好是"过期绿灯是危险方向"要防的那件事。
                    # 回填只是"别黑屏"，不能连安全时限一起复活。
                    # 2026-10-29：红绿灯的"多久算过期"由 NoBoxHold 统一回答
                    # （连续 N 轮无框就全红），不再有按颜色计时的第二套账。
                    pass
                    if fell_back is not None:
                        disp = fell_back
                        result['last_good_fallback'] = fam
                        reason += '+last_good'
                        result['display_reason'] = reason
        result['display'] = disp.as_dict()
        result['previous_display'] = old.as_dict()
        result['display_changed'] = disp != old
        return disp, result

    # Navigation BLUE must not survive when the current frame has a navigation
    # target but the OCR does not confirm one/two digits. Other previously valid
    # Non-BLUE colors can still be retained as directed by the scene-hold rule.
    #
    # On a mode change the previous arrows are discarded outright: every branch
    # reads `baseline`, so blanking it here means a lane->traffic switch cannot
    # leave a yellow "may change lane" beside a red light, and a nav->lane
    # switch cannot leave a blue route arrow on a marking frame. Note this only
    # affects this one round -- from the next round on `old` is the new state.
    baseline = old

    # A blue arrow restored after a restart belongs to a previous state: when
    # this round is itself a switch into navigation, that blue must not be
    # carried. The round's own verdict (or the straight fallback) supplies the
    # arrow. Peek, because the switch is only committed in finish().
    entering_nav = (branch_switch is not None
                    and branch_switch.peek('navigation_blue'))
    nav_baseline = Display() if entering_nav else baseline

    # Leaving navigation drops the debounce memory: coming back should not
    # resurrect a direction from a different stretch of road. The nav-hold paths
    # read `baseline` directly, so they are unaffected.
    if nav_direction_hold is not None and 'nav' not in targets:
        nav_direction_hold.reset()

    def lane_marking(colour):
        """Clear a stale LANE arrow when a traffic light owns the frame.

        YELLOW is the only colour that identifies a lane marking: the lane
        branch is its sole producer, while traffic only ever emits red/green.
        So yellow is unambiguous and can be cleared safely. RED is NOT touched
        even though the lane branch also uses it, because the traffic state is
        itself red/green and the two are indistinguishable from a Display --
        clearing red would wipe the retained light state this branch exists to
        hold. Leaving a red "no lane change" arrow up is also the safe side.
        """
        if stale_lane_markings and colour == YELLOW:
            return None
        return _carry(colour)

    def retained(keep_blue=False):
        # `keep_blue` marks the "we do not know what this frame is" paths: a
        # scene-classifier miss does not contradict a blue route arrow, so those
        # holds keep everything. Every path that has POSITIVELY identified the
        # frame as lane or traffic drops BLUE, because a blue arrow on a lane
        # frame reads as a route instruction that no longer applies.
        conv = (lambda c: c) if keep_blue else lane_marking
        # The expiry limit belongs to the traffic branch only: a red on a lane
        # frame means "solid marking", not "stop", and must not age out. The
        # traffic_hold fallback into lane applies it itself before calling here.
        lf, sf, rf = baseline.left, baseline.straight, baseline.right
        return Display(left=conv(lf), straight=conv(sf), right=conv(rf),
                       edge_warning=warning)

    def retained_lane_frame():
        """车道帧上沿用上一轮：黄/红是标线，绿是灯——绿灯必须擦掉。

        2026-10-28 上板：`lane_hold`（车道两侧都没解出线）把上一轮的绿灯
        整块抄过来，用户看到的就是"识别车道时又出现红绿灯"。绿灯在车道帧上
        没有任何合法含义（车道分支只产黄/红），所以一律清掉；红保留，因为
        红既是"实线"也是"停止"，两种读法都是安全侧。这是用户 2026-10-28
        明确要求的"就是要一起擦掉"。
        """
        disp = retained()
        if GREEN in (disp.left, disp.straight, disp.right):
            result['lane_green_cleared'] = True
            disp = Display(left=None if disp.left == GREEN else disp.left,
                           straight=None if disp.straight == GREEN else disp.straight,
                           right=None if disp.right == GREEN else disp.right,
                           edge_warning=disp.edge_warning)
        return disp

    # Clear directional arrows IF AND ONLY IF the scene lacks all 3 targets.
    # ⚠️ 交警手势早退必须放在 finish() 定义**之后**：之前放在场景模型后面
    # 直接 return finish(...) 会 UnboundLocalError（e2e 抓到，不是单测）。
    # ⚠️ 这一支会跳过下面的 empty_scene_hold / nav_hold 推进。交警模式
    # 期间这两个 hold 冻结，退出后第一轮会带着进入前的计数；影响仅限切换首轮，
    # 为避免动到冻结逻辑暂不改变（改动需要单独上板验证）。
    if cop_kind is not None and cop_signature is not None:
        cop_disp = cop_traffic_signature.cop_display(cop_kind, warning=warning)
        if cop_disp is not None:
            cop_dir = cop_traffic_signature.KIND_TO_DIRECTION[cop_kind]
            if cop_state is not None:
                cop_state['direction'] = cop_dir
            # ⚠️ fresh 必须带上本轮方向：切进 cop 族时 finish() 会清空
            # "本轮没新算出来的"箭头，fresh=() 会把刚画的这个绿箭头也擦掉。
            return finish(cop_disp, 'cop', f'cop_fingerprint_{cop_kind}',
                          fresh=(cop_dir,))
    if empty_scene_hold is not None:
        can_clear = empty_scene_hold.observe(targets)
        result['scene_empty_streak'] = empty_scene_hold.empty
        result['scene_empty_confirm_frames'] = empty_scene_hold.confirm_frames
    else:
        can_clear = not targets
    if nav_hold is not None:
        nav_holding = nav_hold.observe('nav' in targets)
        result['nav_hold_misses'] = nav_hold.misses
        result['nav_hold_frames'] = nav_hold.hold_frames
    else:
        nav_holding = False
    if not targets:
        lane_streaks.reset()
        # A single scene-classifier miss does not contradict a blue route arrow.
        # Without this, two misses cleared the screen mid-route -- 52 of the 108
        # navigation rounds in rk3568_v15_10.log were blank.
        if nav_holding and (baseline.left == BLUE or baseline.straight == BLUE
                            or baseline.right == BLUE):
            return finish(retained(keep_blue=True), 'navigation_hold',
                          'nav_scene_miss_hold_blue')
        if can_clear:
            return finish(Display(edge_warning=warning), 'none', 'scene_no_nav_traffic_lane_clear')
        # The scene said nothing, so the frame type is unknown: a blue route
        # arrow is not contradicted by silence and must not blink off.
        # Without this, two misses cleared the screen mid-route -- 52 of the 108
        # navigation rounds in rk3568_v15_10.log were blank.
        return finish(retained(keep_blue=True), 'scene_empty_hold',
                      'one_off_scene_miss_keep_previous')

    # Highest priority: the five-class model says this frame is NAVIGATION.
    # A navigation frame must never end without a blue arrow: blinking it off
    # on the odd frame where the nav detector finds nothing is what the operator
    # sees as flicker, and the previous red/green light state is stale the
    # moment the screen shows a route instead of a junction.
    if 'nav' in targets:
        t = time.perf_counter()
        try:
            boxes = detectors.navigation(gray)
            direction, evidence = eligible_navigation(boxes)
        except Exception as exc:
            direction, evidence = None, {
                'boxes': [], 'readings': [], 'selected': None,
                'reason': 'navigation_submodel_error',
                'error_type': type(exc).__name__}
        timing['navigation_model_and_digits_ms'] = round((time.perf_counter()-t)*1000, 2)
        result['navigation'] = evidence
        lane_streaks.reset()
        if direction is None and scene_throttle is not None:
            # The scene answer that put us here may itself have been reused.
            # The nav model just came back empty, so the next round must ask the
            # scene model for real instead of reusing "still navigation".
            scene_throttle.force_next()
        if direction is None:
            # Arrow submodel saw nothing at all: keep the previous blue arrow
            # rather than dropping it, then falling through to stale lights.
            # `baseline`, not `old`: on a mode switch the previous arrows were
            # discarded, and falling back to a route arrow restored from a
            # previous process would contradict the blank screen this round
            # started from. With no blue there, the straight fallback applies.
            if nav_baseline.left == BLUE or nav_baseline.straight == BLUE \
                    or nav_baseline.right == BLUE:
                evidence['reason'] += '+hold_previous_blue'
                return finish(retained(keep_blue=True), 'navigation_hold',
                              'nav_blue_missing_hold_previous')
            if evidence.get('reason') == 'navigation_submodel_error':
                # ⚠️ 2026-10-29 审查：`direction is None` 有两种来源——
                # 「画面里真没箭头」和「导航子模型抛异常」（上面的 except 把
                # 异常吃成 direction=None）。后者若落进 default_straight，
                # 模型已经挂了、屏幕却给出"直行"指示，故障完全不可见。
                # 这里没有任何蓝可以沿用，就黑屏，让故障显形。
                return finish(Display(edge_warning=warning), 'navigation_hold',
                              'nav_submodel_error_no_arrow', fresh=())
            direction = 'straight'
            evidence['reason'] += '+default_straight'
        elif nav_direction_hold is not None:
            # Debounce the direction family against the 100 m boundary labels;
            # see NavDirectionHold. The displayed direction, not the raw one,
            # is what goes to the FPGA.
            score = (evidence.get('readings') or [{}])[0].get('score', 0.0)
            held = nav_direction_hold.observe(direction, score)
            if held is not None and held != direction:
                evidence['reason'] += f'+held_{held}_over_{direction}'
                direction = held
        kwargs = {'left': None, 'straight': None, 'right': None}
        kwargs[direction] = BLUE
        return finish(Display(**kwargs, edge_warning=warning),
                      'navigation_blue', evidence['reason'], fresh=(direction,))

    # Second priority: traffic presence suppresses the lane branch regardless
    # of whether the housing detector can currently give a usable result.
    if 'traffic_light' in targets:
        t = time.perf_counter()
        # A housing arity/shape anomaly must degrade this round to traffic_hold,
        # not abort the whole round into run_live's handler, which also resets
        # lane_streaks and empty_scene_hold and costs the lane branch its
        # accumulated solid-confirm count.
        try:
            boxes = prefetched_housing if prefetched_housing is not None \
                else detectors.housing(gray)
            housing_error = None
        except Exception as exc:
            boxes, housing_error = [], type(exc).__name__
        timing['traffic_model_ms'] = round((time.perf_counter()-t)*1000, 2)
        t = time.perf_counter()
        if frame_is_thresholded:
            # 阈值图里没有 254/255。judge() 会把每一侧都算成 off → 全方向停止，
            # 那是凭空制造的"红灯"。这一轮只检框、不判色，落到 traffic_hold。
            verdict = traffic_rules.TrafficResult(
                boxes=[], colours=[], layout='thresholded_frame_no_colour',
                reason='frame_is_thresholded_colour_unreadable')
        else:
            # ⚠️ 2026-10-31：**先剔误检，再让 LayoutHold 数框**。
            #
            # 顺序反了就会出事：板端轮 11 只检到 1 个框（其实是数字误检），
            # LayoutHold 记下"这个路口是 1 框"；轮 13 来了 3 个真框，旧代码
            # 按 1 框去裁，把得分最高的那个留下 —— 恰好是数字，两个真灯被扔掉。
            # 误检必须在计数之前消失，LayoutHold 看到的才是真实布局。
            raw_count = len(boxes)
            boxes, rejected = traffic_boxes.filter_boxes(boxes, gray)
            if rejected:
                result['traffic_boxes_rejected'] = [
                    {'box': [round(float(v), 1) for v in b[:4]],
                     'score': round(float(b[4]), 3) if len(b) > 4 else None,
                     'why': why,
                     'ar': round(f['ar'], 2) if f else None,
                     'cy': round(f['cy'], 3) if f else None,
                     'cf': round(f['cf'], 3) if f else None,
                     # V27：长边三等分里最大一格的占比（颜色集中度）
                     'share': round(f['share'], 2) if f else None,
                     'h': f['h'] if f else None}
                    for b, why, f in rejected]
            # 框数决定方向语义，抖一轮就整体错位：
            #   轮 62 三框 [红,绿,绿] → 左停/直行/右行
            #   轮 72 两框 [红,红]    → 2 框规则把右绿当直行灯，直行从绿翻红
            # 所以框数未经 confirm_frames 轮确认时，本轮**不重新解释方向**，
            # 沿用上一轮灯色；也**绝不裁剪真框**（裁剪会毁掉真实证据）。
            box_count_unconfirmed = False
            want = 0
            if layout_hold is not None:
                want = layout_hold.observe(boxes)
                result['layout_hold'] = {'boxes': len(boxes), 'using': want,
                                         'streak': layout_hold.streak}
                if raw_count != len(boxes):
                    result['layout_hold']['raw_boxes'] = raw_count
                if want != len(boxes) and want > 0:
                    box_count_unconfirmed = True
            # ⚠️ 2026-10-31 安全侧：过滤后只剩 0/1 个框时**不降级布局**。
            # 1 框布局会把右转默认判成"可通行"，而它原本可能是三框路口里
            # 只漏检到一盏灯 —— 那会凭空放行一个右转。宁可沿用上一轮。
            if not box_count_unconfirmed and want > 1 and len(boxes) < 2:
                box_count_unconfirmed = True
                result['layout_hold']['downgrade_blocked'] = True
            if box_count_unconfirmed:
                verdict = traffic_rules.TrafficResult(
                    boxes=[], colours=[], layout='box_count_unconfirmed',
                    reason=f'boxes_{len(boxes)}_vs_held_{want}_hold_previous')
            else:
                verdict = traffic_rules.judge(boxes, gray, w=gray.shape[1],
                                              h=gray.shape[0])
        if housing_error is not None:
            verdict.reason += f'+housing_submodel_error_{housing_error}'
        timing['traffic_rules_ms'] = round((time.perf_counter()-t)*1000, 2)
        result['traffic'] = {'boxes': boxes,
                             'valid_boxes': len(verdict.boxes),
                             'colours': verdict.colours,
                             'layout': verdict.layout,
                             'states': verdict.states,
                             'reason': verdict.reason}
        colors = {}
        for side, state in verdict.states.items():
            if state == traffic_rules.STATE_GO:
                colors[side] = GREEN
            elif state == traffic_rules.STATE_STOP:
                colors[side] = RED
            else:
                colors[side] = None
        if verdict.states:
            # 有框 -> 一定是三个方向齐全（traffic_rules 保证）。
            if no_box_hold is not None:
                no_box_hold.reset()
            if traffic_hold_limit is not None:
                traffic_hold_limit.observe(colors.values())
            # ⚠️ 2026-10-29 高速幻影红绿灯：`off`（框内数不到 254/255）按红灯
            # 显示是**安全侧**——灯真的灭了绝不能显示绿灯。但它不是"读到的颜色"。
            #
            # 旧写法把 off 转成的 RED 当本轮新鲜颜色传给 finish()，于是
            # last_good 把它记下来，之后一直继承：幻影框一出现，红灯就再也
            # 消不掉（v17_5 idx244-435，192 轮）。现在 off 仍然显示红，
            # 但**不进入 fresh**，也就不会刷新记忆。
            # "有没有真读到颜色"以 verdict.colours 为准，不能看 colors 是否
            # 非空——1 框布局的 right 默认就是 GO，那是规则给的，不是读出来的。
            known = bool(verdict.colours) and any(
                c != 'off' for c in verdict.colours)
            if not known:
                # `off` 已经由 traffic_rules 映射成 STOP(=RED)，所以这里
                # colors 里没有 None，箭头照样是红的（安全侧）。要做的只是
                # 别让这个红进 fresh。把判成 off 的方向记下来便于排查。
                result['traffic_off_as_red'] = list(verdict.states)
            for side in ('left', 'straight', 'right'):
                if colors[side] is None:
                    colors[side] = RED
                    result.setdefault('traffic_off_as_red', []).append(side)
            # ⚠️ 2026-10-29 用户硬要求「三个箭头一组，要么都没要么都有」。
            # 上面只补了 None；但"无框且还在 NoBoxHold 窗口内"的那条路
            # 会走 traffic_hold 沿用上一轮，而上一轮可能是缺方向的。
            # 所以这里对任何缺口都按红灯补齐（红灯=停止，是安全侧）。
            gaps = [k for k in ('left', 'straight', 'right')
                    if colors[k] is None]
            if gaps:
                result['traffic_arrows_filled'] = gaps
            lane_streaks.reset()
            disp = Display(left=colors.get('left'), straight=colors.get('straight'),
                           right=colors.get('right'), edge_warning=warning)
            if known:
                return finish(disp, 'traffic', verdict.reason,
                              fresh=tuple(s for s, c in colors.items() if c is not None))
            # 整轮只读到 `off`：箭头照样按红显示（安全侧），但这不是"读到的
            # 颜色"，fresh 为空 → finish() 不会刷新 last_good。这一条就是
            # 幻影红灯能长期挂着的根因，别为了"看起来是 traffic 分支"而把它
            # 当成新鲜证据。
            return finish(disp, 'traffic', verdict.reason + '+off_only_no_fresh',
                          fresh=())
        if verdict.layout == 'no_housing_box' and housing_error is None:
            # 无框：2026-10-29 用户定「先沿用，超过 N 轮再清」。
            # 单轮漏检不该把灯灭掉，但一直挂着已经消失的灯也不行。
            keep_holding = (no_box_hold.observe(False)
                            if no_box_hold is not None else False)
            result['no_box_streak'] = (no_box_hold.streak
                                       if no_box_hold is not None else None)
            if keep_holding:
                result['traffic_hold'] = True
                provisional_holds.append('traffic_hold')
            elif not traffic_verified:
                # ⚠️ 2026-10-29：**没有灯箱实证时，无框 = 这里没有红绿灯**。
                # 以前不管有没有实证，N 轮无框就铺三个红灯。放到高速上，
                # 幻影标签 + 灯箱无框 = 永久三红。没有真灯就不该有灯色。
                lane_streaks.reset()
                if traffic_hold_limit is not None:
                    traffic_hold_limit.reset()
                if no_box_hold is not None:
                    no_box_hold.reset()
                return finish(Display(edge_warning=warning), 'no_traffic_light',
                              'scene_says_traffic_but_no_housing_box')
            else:
                lane_streaks.reset()
                if traffic_hold_limit is not None:
                    traffic_hold_limit.reset()
                if no_box_hold is not None:
                    no_box_hold.reset()
                # 连续 N 轮无框 = 这块画面上确实没有灯。按用户硬性要求，
                # 红绿灯模式下仍然要给满三个箭头，所以落到红灯兜底
                # （停止是安全侧），而不是黑屏。
                return finish(Display(left=RED, straight=RED, right=RED,
                                      edge_warning=warning), 'traffic',
                              'no_housing_box_all_red',
                              fresh=('left', 'straight', 'right'))
        # 其余情况：二值帧（只有灰阶、没有 254/255）、灯箱子模型报错、
        # 或灯箱解不出颜色 —— 都拿不到颜色，沿用上一轮，且不跑车道。
        # 注意子模型报错时**不能**当成"无框"去清屏：那是我们的故障，
        # 不是画面里真的没有灯。连续失败由 --max-infer-failures 兜底。
        result['traffic_hold'] = True
        provisional_holds.append('traffic_hold')

    # Third priority: lane marking type from the v2 segmentation model.
    #
    # ⚠️ 只要场景里有红绿灯标签，这一轮就**不再让车道刷新箭头**（2026-10-28）。
    # 旧写法是「灯箱解不出灯 → 落到 traffic_hold → 继续跑车道」，于是路口帧
    # 顶着上一轮灯色的同时又被画上黄/红车道箭头：用户看到的"红绿灯时出现
    # 车道"。车道标线在路口本来就要重画，读出来的东西也不可信；沿用灯色即可。
    if 'lane' in targets and 'traffic_light' not in targets:
        t = time.perf_counter()
        try:
            # ⚠️⚠️ 2026-10-31 用户硬性要求：「RK 端坚决不要做二值化」。
            # 车道直接吃 YRG 灰度帧；模型内部不做任何阈值（lane_v2.prepare
            # 只有一次缩放）。二值采集通道已整个删除，这里没有"要不要二值帧"
            # 的分支 —— 也就没有"拿不到就跳过"的状态了。
            mask = detectors.lane_mask(gray)
            if mask is None:
                branch = 'traffic_hold' if result.get('traffic_hold') else 'lane_hold'
                return finish(retained_lane_frame(), branch,
                              'lane_model_returned_none')
            result['lane_source'] = 'grey'
            timing['lane_model_ms'] = round((time.perf_counter()-t)*1000, 2)
            t = time.perf_counter()
            if lane_backend == 'angle':
                lanes = select_lanes(mask, ratio_threshold=lane_dash_ratio)
            else:
                # V17_3：参考带放宽到 --lane-band-top（默认 0.10，原来是 0.20），
                # 让画面上部的车道线也参与统计。用户要求「可以判别全屏，没必要
                # 只取下面一半」。
                lanes = lane_v2_decode(
                    mask, min_gap_frac=0.0,
                    band_top=lane_band_top, band_bot=lane_band_bot)
                lanes['algorithm'] = 'v2_geometry'
            timing['lane_geometry_ms'] = round((time.perf_counter()-t)*1000, 2)
        except Exception as exc:
            # Same reasoning as the housing guard: one bad frame holds the
            # previous arrows instead of aborting the round and resetting the
            # solid-confirm streaks.
            result['lane_error'] = type(exc).__name__
            branch = 'traffic_hold' if result.get('traffic_hold') else 'lane_hold'
            return finish(retained(), branch, 'lane_submodel_error_hold')
        result['lane'] = lanes
        # The lane detector owns LEFT and RIGHT only -- it says nothing about a
        # straight arrow. Carrying the previous straight over left a green/blue
        # straight arrow from an earlier traffic or navigation round burning on
        # the FPGA screen for the whole lane branch (observed: 20+ rounds).
        # Keep it only while a traffic label is present but unusable, where the
        # anti-flicker rule intentionally holds the last known light state.
        straight_held = baseline.straight if result.get('traffic_hold') else None
        attrs = {'left': _carry(baseline.left), 'right': _carry(baseline.right),
                 'straight': _carry(straight_held)}
        updates = {}
        for side in ('left', 'right'):
            kind = lanes[side]['kind']
            # ⚠️ 2026-10-29 大改：**只有"想保持的颜色和屏幕一致"时才允许按住。**
            #
            # 旧写法是"证据弱就一律不改色"。代价实测：车道 84 轮里 16 段
            # weak_hold，最长按了 3.18 秒；而日志里最长那段更荒唐 ——
            # 模型连续 4 轮都判 dash、屏幕本来就是黄的，什么都没变，纯属白按。
            #
            # 更关键的是它挡住了真正的变化：用户变道后模型一旦给出不同的
            # kind（dash ↔ solid），那道命令是**实质变化**，不该被"证据弱"拦下。
            # 用户原话：「你说的这种按住，实在是太浪费时间了」。
            #
            # 现在只在"模型想保持的颜色 == 屏幕现有颜色"时按住；
            # 想变成别的颜色就立刻生效。
            weak = lanes[side].get('evidence') == 'weak'
            want = lane_streaks.preview_colour(side, kind)
            if weak and want is not None and want == attrs[side]:
                updates[side] = 'weak_hold_previous'
                continue
            color, action, _ = lane_streaks.update_side(side, kind, attrs[side])
            attrs[side] = color
            updates[side] = action
        result['lane_updates'] = updates
        result['lane_solid_streaks'] = {'left': lane_streaks.left,
                    'right': lane_streaks.right, 'confirm_frames': lane_streaks.confirm_frames}
        if lanes['left']['kind'] == 'none' and lanes['right']['kind'] == 'none':
            branch = 'traffic_hold+lane_hold' if result.get('traffic_hold') else 'lane_hold'
            # 两侧都没解出线：这一轮没有任何车道证据，显示是原样沿用的。
            # 不能算"进入了车道状态"，否则一次漏检就把灯态清空。
            provisional_holds.append('lane_hold')
            return finish(retained_lane_frame(), branch,
                          'lane_scene_no_valid_edges_hold')
        branch = 'traffic_hold+lane' if result.get('traffic_hold') else 'lane'
        # A colour that changed on weak evidence is the signature of a lane
        # misread, and it is invisible in the aggregate stats. Flag it so the
        # operator can dump the frame instead of waiting for the symptom again.
        weak_changes = {side: {'was': COLOR_NAMES[old_color] if old_color is not None else 'hidden',
                               'now': COLOR_NAMES[attrs[side]] if attrs[side] is not None else 'hidden',
                               'ev': lanes[side].get('ev'),
                               'x': lanes[side].get('x'),
                               'kind': lanes[side]['kind']}
                        for side, old_color in (('left', baseline.left),
                                                ('right', baseline.right))
                        if lanes[side].get('evidence') == 'weak'
                        and attrs[side] != old_color}
        if weak_changes:
            result['lane_weak_evidence_change'] = weak_changes
        reason = 'dash_yellow_immediate_solid_red_confirmed_otherwise_hold'
        if result.get('traffic_hold'):
            reason = 'traffic_unknown_hold_lane_refresh_' + reason
        return finish(Display(**attrs, edge_warning=warning), branch, reason,
                      fresh=tuple(k for k, v in attrs.items() if v is not None))

    # Traffic label present but unusable, and no lane branch to fall back on.
    if result.get('traffic_hold'):
        held = retained()
        # 2026-10-29：不再按颜色计时过期。沿用多久完全由 NoBoxHold 决定：
        # 连续 --no-box-hold-frames 轮没检出框 -> 全红（安全侧），
        # 在那之前保持上一轮灯色。这样三个箭头永远同时存在。
        return finish(held, 'traffic_hold',
                      'traffic_scene_classifier_unknown_hold')

    # Scene target 'nav' is present but no valid number or supported direction.
    lane_streaks.reset()
    return finish(retained(), 'navigation_hold', 'nav_invalid_no_new_blue_preserve_other_colors')
