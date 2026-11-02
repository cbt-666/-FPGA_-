"""红绿灯：灯箱检框 → 数像素判色 → 按框数判左右直能否通行。

模型（`Rk3568/红绿灯灯箱/`）只回答"画面里哪里有灯箱"，不判颜色也不判方向。
颜色由板端在 YRG 图上数像素得出（该包实测 96.5%），方向由下面的框数规则得出。

⚠️ 框的数量直接决定谁是哪个方向，从左到右读（2026-10-28 用户定死）：

    1 框  → 只有直行灯。直行看这个框的颜色。
            左转不说（本轮不显示），右转默认可通行。
    2 框  → 左框 = 左转灯，右框 = 直行灯。
            右转默认可通行（中国红灯一般不禁右转，见交规 38 条，且右转箭头灯极少装）。
    3 框  → 左框 = 左转，中框 = 直行，右框 = 右转 —— 右转也被红绿灯约束。

⚠️ 不要再用"一个框代表三个方向"。板上唯一的框常常是横排三灯箱里最大的那一个，
按同色处理会把直行的红当成左转的红，凭空造出"左转禁止"。单框只谈直行。
⚠️ 不用 2 框去猜"左+右（没有独立直行灯）"：那等于把右框的红算到直行头上，
把直行判死。真实灯组里 2 个框就是"左转 + 直行"，右转不装灯。

## 为什么不再按几何丢弃框（2026-10-28 用户要求）

面板上做过三轮几何过滤：中心太低 / 宽高比 / 尺寸 / 成排校验。板上日志的实测结果：

    248 轮有灯箱结果，只有 14 轮（5.6%）通过过滤；`dropped` 里 `tall` 累计 386 次，
    `low` 只有 3 次 —— 过滤几乎全在按宽高比扔真框。

同一份日志按"全部保留"重放：有效判定 14 → **149** 轮，三种布局
（`single_straight` 12 / `left_straight` 62 / `left_straight_right` 75）都能稳定出来。

框数是决定语义的输入，用几何先扔掉一部分就等于把"3 框"读成"2 框"、"2 框"读成"1 框"，
方向会系统性错位——这比多保留一个误检严重得多。所以几何过滤全部移除。

误检的代价由下游承担：框内数不到 254/255 就是 `off`，`off` 按红灯处理（保守），
且 `TrafficHoldLimit` 会给假绿灯设上限。
"""
from __future__ import annotations

from dataclasses import dataclass, field

from protocol import RED, GREEN, BLUE, YELLOW  # noqa: F401  (BLUE/YELLOW for callers)

STATE_STOP = 'stop'
STATE_GO = 'go'

# 没有右转灯时右转的默认状态。中国红灯一般不禁止右转，且右转箭头灯极少装，
# 所以 1 框 / 2 框布局里右转是明确的 GO，而不是"不知道"。
DEFAULT_RIGHT = STATE_GO

# 框内纯红/纯绿像素少于这个数，认为这个框没在发光。
MIN_COLOUR_PIXELS = 20


def _colour_of(box, yrg):
    """框内数纯红/纯绿像素。

    直接吃 YRG 原图：FPGA 已经把真红编码成 254、真绿编码成 255，
    其余全是灰度 0~253。所以这里不用还原 RGB，也不用阈值 —— 数等值像素即可，
    既精确又快。这也正是"全灰度链路"能成立的关键：
    导航/车道/红绿灯三个模型吃同一张图，颜色信息以两个特殊值的形式保留在里头。
    """
    x1, y1, x2, y2 = box[:4]
    x1 = max(int(round(x1)), 0)
    y1 = max(int(round(y1)), 0)
    x2 = min(int(round(x2)), yrg.shape[1])
    y2 = min(int(round(y2)), yrg.shape[0])
    if x2 - x1 < 2 or y2 - y1 < 2:
        return 'off'
    c = yrg[y1:y2, x1:x2]
    n_red = int((c == 254).sum())
    n_green = int((c == 255).sum())
    if n_red < MIN_COLOUR_PIXELS and n_green < MIN_COLOUR_PIXELS:
        return 'off'
    return 'red' if n_red >= n_green else 'green'


def _colour_to_state(colour):
    return STATE_GO if colour == 'green' else STATE_STOP


@dataclass
class TrafficResult:
    states: dict = field(default_factory=dict)      # {'left','straight','right'} -> stop/go/None
    boxes: list = field(default_factory=list)       # 参与判决的框，按 cx 从左到右
    colours: list = field(default_factory=list)     # 与 boxes 一一对应
    layout: str = 'none'
    reason: str = ''


def _left_to_right(boxes):
    """按框中心 x 排序。

    只排序，不丢弃：框数就是方向数，"从左到右依次是左/直/右"靠的是顺序。
    """
    return sorted(boxes, key=lambda b: (b[0] + b[2]) / 2.0)


def judge(boxes, yrg, w=1280, h=720):
    """灯箱框 + YRG 原图 -> 左右直三个方向能否通行。

    boxes 是检测器输出的原图坐标 xyxy(+score)；yrg 是 FPGA 的 YRG 单通道图。

    ⚠️ 2026-10-28 上板后按用户要求改成「**每轮必须给满三个方向**」：

        1 框 → 左 = 该框颜色，直行 = 该框颜色，右 = 绿（默认可右转）
        2 框 → 左 = 左框，直行 = 右框，右 = 绿（保持原样）
        3 框 → 左/直/右 依次取三个框（保持原样）
        0 框 → 三个方向都不输出（states 为空），由调用方清零上一次箭头

    旧版单框只给 `straight`，`left` 留 None，屏幕上左边就一直是暗的——
    用户看到的就是"左边有时候会不显示"。
    """
    kept = _left_to_right(boxes)[:3]     # 多于 3 个时最右侧的忽略
    colours = [_colour_of(b, yrg) for b in kept]

    if not kept:
        # 无框：不输出任何方向，调用方负责清零上一次的箭头。
        # ⚠️ layout 必须是 `no_housing_box`。2026-10-29 审查发现原来写的是
        # `none`，而 strategy_engine 判的是 `layout == 'no_housing_box'` ——
        # 于是"无框就清零"这条用户明确要求的分支**从来没执行过**，
        # 屏幕一直显示上一盏灯，直到 LastGood 把它无限续命。
        return TrafficResult(states={}, boxes=[], colours=[], layout='no_housing_box',
                             reason='no_housing_box')

    if len(kept) == 1:
        # 一个框：颜色同时代表左转和直行，右转默认可通行。
        lc = _colour_to_state(colours[0])
        return TrafficResult(
            states={'left': lc, 'straight': lc, 'right': DEFAULT_RIGHT},
            boxes=kept, colours=colours,
            layout='single_all_three', reason='one_housing_left_and_straight')

    if len(kept) == 2:
        # 左框 = 左转灯，右框 = 直行灯；右转默认可通行。
        return TrafficResult(
            states={'left': _colour_to_state(colours[0]),
                    'straight': _colour_to_state(colours[1]),
                    'right': DEFAULT_RIGHT},
            boxes=kept, colours=colours,
            layout='left_straight', reason='two_housings_left_and_straight')

    # 3 框：左转 / 直行 / 右转，右转也被红绿灯约束。
    return TrafficResult(
        states={'left': _colour_to_state(colours[0]),
                'straight': _colour_to_state(colours[1]),
                'right': _colour_to_state(colours[2])},
        boxes=kept, colours=colours,
        layout='left_straight_right', reason='three_housings_lsr')
