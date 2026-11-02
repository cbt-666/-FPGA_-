"""交警手势识别 —— 手势类别 -> 通行语义 的解译层。

系统对路口交通指挥手势做端到端识别，模型输出手势类别，本模块把类别解译为
通行方向，并维护进入 / 退出交警模式的时序状态。

## 职责边界

    手势分类模型（96x96, FP16）  ->  类别序号
    本模块 CLASS_TO_KIND         ->  语义代号
    本模块 KIND_TO_DIRECTION     ->  左 / 直 / 右
    本模块 cop_display()         ->  单个箭头 + 颜色

类别集合与网络结构解耦：模型输出维度 N 由训练配置给定，本模块只维护
「类别序号 -> 语义代号 -> 方向」这条映射链。新增或调整手势语义只需改映射表。

当前映射（交通警察 8 类指挥手势中影响通行的类别 + 拒识类）：

    类 0 -> 左转（左转弯 / 左转弯待转）
    类 1 -> 直行
    类 2 -> 其他（other，拒识：未作指挥手势 / 非交警场景）

## 输入契约（与交付包 README 逐条对齐，改动前先对照）

    yrg (720,1280 uint8)                     <- FPGA 单通道
      -> yrg_single_to_bgr()                 254->红 / 255->绿 / 其余灰度
      -> resize (96,96) INTER_AREA           直接拉伸，不补黑边
      -> BGR2RGB
      -> NHWC uint8 0..255                   归一化已烘进 RKNN（mean/std=127.5）

⚠️ **不要走 `fpga_rg_gray()` 那条路。** 包里 `infer_cls.py` 是给"本地 BGR
视频"用的，它自己再做一遍红绿判定；板上 FPGA 已经判完了，RK 端只有单通道
YRG，重做一遍等于用 RK 的阈值去覆盖 FPGA 的阈值，两边不一致时模型看到的是
没见过的图。

## 时序判决

单帧结果不直接输出，经「因果多数票 + 连续确认」过滤：

    other / 低于门槛          -> miss（连续 RELEASE_ROUNDS 次即退出交警模式）
    有效手势且过门槛          -> 票窗追加；同一类连续 LOCK_ROUNDS 轮 -> 进入交警模式

按类别分别设门槛与确认轮数（`CONF_MIN_BY_KIND` / `LOCK_ROUNDS_BY_KIND`），
因为不同手势的类间可分性不同，统一门槛会用一个类的最优点惩罚另一个类。

## 设计要点

- **因果**：只依赖当轮及此前结果，不引入未来帧，满足实时在线要求。
- **严格连续**：miss 一律清零连击计数，散点误检无法累积成一次触发。
- **NaN 防护**：判据写成 `not (conf >= CONF_MIN)` 而非 `conf < CONF_MIN`；
  softmax 遇非有限值时退化为均匀分布，防止 NaN 被误判为"过线"。
- **单箭头硬协议**：交警模式屏幕上只出现一个箭头，FPGA 以此为交警模式特征
  并联动语音；补成三箭头会让整段逻辑失效（三箭头规则只约束红绿灯分支）。
"""
from __future__ import annotations

from collections import deque

import numpy as np

from protocol import Display, GREEN, RED

# 模型类别序号 -> 语义代号。
# 代号沿用模型自己的类别编号（0/1/2），日志里 `cop_fingerprint_0` 就是"类 0"，
# 中间不隔一层字母，少一次翻译就少一次接错。
CLASS_TO_KIND = {
    0: '1',   # 类 0 = 左转弯 / 左转弯待转  -> 左转
    1: '2',   # 类 1 = 直行信号              -> 直行
    2: None,  # 类 2 = other（拒识）
}

# 语义代号 -> 箭头方向。
# 手势 -> 通行语义的映射集中在 `01_交警手势识别/说明/算法说明.md` 第 2 节，
# 与这里的表一一对应；调整语义只改这两处，不动网络。
KIND_TO_DIRECTION = {
    '1': 'left',
    '2': 'straight',
}

# 箭头颜色随手势语义走：
#   左转 -> 绿
#   直行 -> 红
# 其余分支的箭头颜色一个都没动（导航蓝、车道黄/红、红绿灯红/绿）。
# ⚠️ 交警模式仍然是"只有一个箭头"——FPGA 正是靠单箭头判定交警模式，
#    换色不改箭头个数，所以这条硬协议不受影响。但**不要假设交警箭头
#    一定是绿的**：直行那个是红的。
KIND_TO_COLOUR = {
    '1': GREEN,
    '2': RED,
}

# 判定为有效手势所需的最低置信度。
#
# 分类置信度门槛按类别分别设定：不同手势的类间可分性不同，统一门槛会
# 用一个类的最优点去惩罚另一个类。`CONF_MIN` 是类 0（左转）的默认值，
# 按类别的完整表见 `CONF_MIN_BY_KIND`。
#
# 选点方法：在整段连续帧上复放（逐帧不抽样），取「有效手势段全程不掉线」
# 且「非交警场景不误触发」之间的最大余量。
#
# ⚠️ 单靠置信度切不开所有误判 —— 拒识能力由 `other` 类承担，连续确认
# (`LOCK_ROUNDS`) 是第二道防线。调门槛前先看这两条一起的效果。
CONF_MIN = 0.95

# 输出尺寸（固定值，改模型才改这里）
IMG_SIZE = 96

# 多数票窗口（轮）。板上约 0.7 s/轮 -> 约 8 s。
WINDOW = 12
# 连续几轮"当前模型原话 == 窗口多数票"才进入交警模式。
# 这里是左转类的默认值，直行类见 LOCK_ROUNDS_BY_KIND。
LOCK_ROUNDS = 2
# 连续几轮 other / 低置信即退出交警模式：一帧不是交警就退出，切走最快。
RELEASE_ROUNDS = 1

# 按类别分别配置。不同手势的类间可分性不同，统一门槛会用一个类的最优点
# 去惩罚另一个类；确认轮数同理。
CONF_MIN_BY_KIND = {'1': CONF_MIN, '2': 0.90}
LOCK_ROUNDS_BY_KIND = {'1': LOCK_ROUNDS, '2': LOCK_ROUNDS}


def yrg_single_to_bgr(yrg):
    """板上唯一的 YRG 重建：254 -> 纯红、255 -> 纯绿、其余灰度复制三通道。

    与交付包 README 的 `yrg_single_channel_to_bgr()` 逐行一致。
    ⚠️ 这里**不做**任何红绿判定 —— 颜色是 FPGA 给的。
    """
    out = np.repeat(yrg[:, :, None], 3, axis=2).astype(np.uint8)
    out[yrg == 254] = (0, 0, 255)
    out[yrg == 255] = (0, 255, 0)
    return out


def to_tensor(yrg):
    """YRG 单通道 -> 模型要的 NHWC uint8 RGB 1x96x96x3。

    归一化在 RKNN graph 内部（mean/std = 127.5），这里只给 0..255。
    模型是 96x96 方形，1280x720 直接拉伸 —— ⚠️ 不要 letterbox 补黑边：
    补黑边后有效区域占比变小，实测置信度腰斩（两段对比素材上 1.00 -> 0.55/0.63）。
    """
    import cv2
    bgr = yrg_single_to_bgr(yrg)
    rgb = cv2.cvtColor(cv2.resize(bgr, (IMG_SIZE, IMG_SIZE),
                                  interpolation=cv2.INTER_AREA),
                       cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb[None])


def softmax(v):
    # 任何一个分量非有限 -> 整条输出不可信，直接退化成均匀分布（置信度 1/3，
    # 必然低于门槛）。不能只把 NaN 当 -inf：`[1, NaN, NaN]` 那样剩一个有限值
    # 时会算出 conf=1.0，安全门反而被打开。
    v = np.asarray(v, np.float32)
    if not np.all(np.isfinite(v)):
        return np.full(v.shape, 1.0 / v.size, np.float32)
    e = np.exp(v - v.max())
    return e / e.sum()


def kind_from_logits(logits):
    """(3,) 原始输出 -> ('1'|'2'|None, 类别序号, 置信度)。"""
    p = np.asarray(logits, np.float32).reshape(-1)
    if p.shape != (3,):
        raise ValueError(f'交警分类模型应输出 3 类，得到 {p.shape}')
    probs = softmax(p)
    idx = int(probs.argmax())
    return CLASS_TO_KIND[idx], idx, float(probs[idx])


class TrafficSignature:
    """因果多数票 + 连续确认的手势状态机。

    `locked` 为 None 表示未进入交警模式（正常逻辑照常跑）；
    为 '1'/'2' 表示当前已确认的手势语义（代号 = 模型自己的类号）。

    每轮证据来自手势分类模型的 N 类输出。`other` 类或置信度低于
    `CONF_MIN_BY_KIND` 中该类门槛，一律按 miss 处理 —— 这是"本轮没有可用
    手势证据"的唯一表达。门槛与确认轮数按类别单独配置。
    """

    def __init__(self, window=WINDOW, lock_rounds=None,
                 release_rounds=RELEASE_ROUNDS, conf_min=None,
                 conf_by_kind=None, lock_by_kind=None):
        if window < 1 or release_rounds < 1:
            raise ValueError('window / release_rounds 必须 >= 1')
        # 不传 conf_min / lock_rounds 时使用按类别的默认表；
        # 传了单值则两类都用这个值（测试 / 手工调参的兼容路径）。
        if conf_min is None:
            self.conf_min = float(CONF_MIN)
            self.conf_by_kind = {k: float(CONF_MIN_BY_KIND[k])
                                 for k in KIND_TO_DIRECTION}
            for k, v in (conf_by_kind or {}).items():
                if k in self.conf_by_kind:
                    self.conf_by_kind[k] = float(v)
        else:
            self.conf_min = float(conf_min)
            self.conf_by_kind = {k: self.conf_min for k in KIND_TO_DIRECTION}
        for v in self.conf_by_kind.values():
            if not 0.0 < v <= 1.0:
                raise ValueError('conf_min 必须在 (0, 1] 之间')
        if lock_rounds is None:
            self.lock_rounds = int(LOCK_ROUNDS)
            self.lock_by_kind = {k: int(LOCK_ROUNDS_BY_KIND[k])
                                 for k in KIND_TO_DIRECTION}
            for k, v in (lock_by_kind or {}).items():
                if k in self.lock_by_kind:
                    self.lock_by_kind[k] = int(v)
        else:
            self.lock_rounds = int(lock_rounds)
            self.lock_by_kind = {k: self.lock_rounds for k in KIND_TO_DIRECTION}
        for v in self.lock_by_kind.values():
            if v < 1:
                raise ValueError('lock_rounds 必须 >= 1')
        self.window = int(window)
        self.release_rounds = int(release_rounds)
        self._votes = deque(maxlen=self.window)
        # 当前正在连续确认的候选手势；与 _votes 的多数票分开记。
        # 只用"多数票 != locked"累加 streak 会让旧票把新证据带偏。
        self._candidate = None
        self._streak = 0
        self._miss = 0
        self.locked = None
        self.lock_count = 0
        self.release_count = 0
        self.last_conf = 0.0
        self.last_index = -1
        self.last_kind = None
        # 模型输出形状错误等异常时的诊断；正常轮次为 None。
        self.last_error = None

    # ---- 内部 ----
    def _majority(self):
        """严格多数票；打平或没有票 -> None（不猜）。

        只有两种有效手势，所以"得票最多的那种唯一"就等于严格多数。
        不写死 'A'/'B'：语义代号换了这里不用跟着改。
        """
        if not self._votes:
            return None
        counts = {}
        for v in self._votes:
            counts[v] = counts.get(v, 0) + 1
        best = max(counts.values())
        winners = [k for k, n in counts.items() if n == best]
        return winners[0] if len(winners) == 1 else None

    def _miss_round(self):
        """本轮没有可用证据：只允许退出，不允许进入交警模式。"""
        self._miss += 1
        self._candidate = None
        self._streak = 0
        if self._miss >= self.release_rounds:
            # 连续 miss 达到 release 阈值时清空票窗：
            #   * 已进入交警模式时退出，否则陈票会让它立刻跳回去；
            #   * 未进入时也清，否则整段 miss 之后旧票还在窗口里，
            #     新手势证据刚出现时会被旧票压住。
            self._votes.clear()
            if self.locked is not None:
                self.locked = None
                self.release_count += 1
        return self.locked

    # ---- 对外 ----
    def observe(self, logits):
        """喂一轮模型原始输出 (N,)。

        `logits` 为 None 表示本轮模型没跑（关掉手势识别 / 帧不可用），
        与"模型跑了但判 other"同样按 miss 处理。
        """
        if logits is None:
            self.last_kind = None
            self.last_index = -1
            self.last_conf = 0.0
            self.last_error = None
            return self._miss_round()
        try:
            raw, idx, conf = kind_from_logits(logits)
        except (ValueError, TypeError) as exc:
            # 模型输出形状不对（放错 .rknn / 输出被改）只算 miss，
            # 不能让它穿透到 run_live 的 infer_fail_streak 触发重启。
            self.last_kind = None
            self.last_index = -1
            self.last_conf = 0.0
            self.last_error = type(exc).__name__
            return self._miss_round()
        self.last_error = None
        self.last_index = idx
        self.last_conf = conf
        self.last_kind = raw          # 诊断用：模型原话（含低于门槛的）
        conf_min = self.conf_by_kind.get(raw, self.conf_min)
        if raw is None or not (conf >= conf_min):
            # other 类 / 置信度不足 = 本轮没有可用的手势证据。普通路面、
            # 路口空镜、导航屏、未作指挥手势的画面全部走这里，不会进入交警模式。
            #
            # ⚠️ 必须写成 `not (conf >= ...)` 而不是 `conf < ...`：
            # conf 为 NaN 时 `NaN < 0.95` 是 False，会把 NaN 放行成"过线"。
            # softmax 已剔除非有限值，这里是第二道锁。
            return self._miss_round()

        self._miss = 0
        self._votes.append(raw)
        vote = self._majority()
        if vote is None:
            # 打平/没票：本轮不构成"同一手势的连续证据"。
            self._candidate = None
            self._streak = 0
            return self.locked
        # ⚠️ 连续计数必须同时满足两点：
        #   ① 当前这一轮的模型原话就是 vote（raw == vote）；
        #   ② 窗口多数票连续 N 轮都是同一个手势。
        # 只看"多数票 != locked"会让旧票在新证据上累加 streak：
        # 先零散攒了 5 票直行、始终未达确认轮数，后来连续 3 轮都判左转，
        # 旧写法会在第 3 轮把 locked 锁成直行（方向反）。
        if raw != vote:
            self._candidate = None
            self._streak = 0
            return self.locked
        if self.locked == vote:
            self._candidate = vote
            self._streak = 0
            return self.locked
        if vote == self._candidate:
            self._streak += 1
        else:
            self._candidate = vote
            self._streak = 1
        need = self.lock_by_kind.get(vote, self.lock_rounds)
        if self._streak >= need:
            self.locked = vote
            self._candidate = vote
            self._streak = 0
            self.lock_count += 1
        return self.locked

    def snapshot(self):
        votes = list(self._votes)
        # 两个方向各自计数。字段名用 kinds 而不是 A/B：语义代号随手势集合变，
        # 这里不跟着改。
        return {'kind': self.locked,
                'votes': len(votes),
                'votes_by_kind': {k: votes.count(k) for k in KIND_TO_DIRECTION},
                # 当前连续确认中的候选手势（未进入交警模式时也有值，便于看板排查）
                'candidate': self._candidate,
                # 按类别的门槛/连续轮数，板上直接能看出某一类为什么过/不过
                'conf_min': dict(self.conf_by_kind),
                'lock_rounds': dict(self.lock_by_kind),
                'streak': self._streak,
                'miss': self._miss,
                'lock_count': self.lock_count,
                'release_count': self.release_count,
                # 本轮模型给出的类别序号 / 置信度 / 原始归类
                'cls_index': self.last_index,
                'cls_conf': round(self.last_conf, 4),
                'cls_kind': self.last_kind,
                # 模型输出形状错误等（正常轮次为 None）
                'error': self.last_error}


def cop_display(kind, warning=False):
    """手势语义 -> **只有一个箭头**的显示状态。

    ⚠️ 其余两个方向必须是 None。FPGA 正是靠"只有一个箭头"来判定交警模式
    并播报语音；补成三箭头会让整段演示失效（那条三箭头规则只约束红绿灯分支）。

    颜色随手势语义走：
      左转 -> LEFT **绿**
      直行 -> STRAIGHT **红**
    换的只是颜色，箭头个数仍是 1，所以 FPGA 的单箭头判据照样成立。

    `warning` 只写 edge_warning 位（bit 3），不会增加任何箭头：交警模式下
    斑马线/行人报警位不被静默丢弃。默认 False 保持旧调用兼容。
    """
    direction = KIND_TO_DIRECTION.get(kind)
    if direction is None:
        return None
    colour = KIND_TO_COLOUR.get(kind, GREEN)
    if direction == 'left':
        return Display(left=colour, edge_warning=bool(warning))
    if direction == 'right':
        return Display(right=colour, edge_warning=bool(warning))
    if direction == 'straight':
        return Display(straight=colour, edge_warning=bool(warning))
    raise ValueError(f'未知箭头方向 {direction}')
