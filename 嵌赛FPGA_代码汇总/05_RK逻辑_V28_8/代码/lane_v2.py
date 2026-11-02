"""车道模型 v2（G640 灰度）+ 几何解码。

模型包：`Rk3568/车道_A_B1_B2_C/v2_768/`

| 代号 | 输入 | 算力 | 官方留出 |
|---|---|---|---|
| G640 | 灰度，640×360     | 0.44× 旧 A 档 | 0.890（本次复测 0.870）|

⚠️ 2026-10-31：**二值版 A768 已整个删除**。用户要求「把所有和二值化有关的
东西都删掉、车道换成灰度模型、不再请求二值化、RK 端坚决不要做二值化」。
本文件现在只认识灰度输入，不存在任何阈值参数，也没有"传错变体"的分支。

原始应用自包含实现，不再依赖旧包的 `lane_decision.py`：

  * 预处理：FPGA 亮度公式 `Y=(77R+150G+29B)>>8`，**不是 OpenCV 的 BT.601**
  * 输入：YRG 帧 -> 亮度灰度，**不做任何阈值**
  * 输出：`1×3×H×W` logits，逐像素 argmax → 0 背景 / 1 白虚线 / 2 实线
  * 几何：下部参考带按列统计 → 自车中心左右各取最近一簇 → 簇内虚线多则判虚线

为什么不做门控
--------------
旧 A 档模型在纯黑图上会吐出约 9k 个「实线」像素，所以 `lane_scan.gate()`
拿二值原图做交集是必须的。v2 模型实测纯黑图前景像素 **0 个**，
幻觉问题在新权重里已经不存在，因此门控整层去掉 —— 这也正是灰度版
能独立成立的前提（灰度输入没有二值图可拿来当门控）。
"""
from __future__ import annotations

import numpy as np

# ---- 几何解码参数：模型包里实测标定过，别凭直觉改 ----
# 参考带上界。曾经在官方留出集上把它从 0.45 收紧到 0.40（官方 +2 点），
# 但那是过拟合：官方素材大多是路口接近画面，车道本来就几乎不起作用，
# 而模型在这些画面里只输出近场（y>0.70）。
# 换到 LaneMarking8K（清晰路面标线，另一域）后趋势完全相反：
#     上界   官方 G640   8K G640
#     0.45     88%        61.9%
#     0.40     88%        60.0%
#     0.25     89%        69.4%
#     0.20     88%        73.8%
# 0.20 在两个域上都不差，且在真正需要判车道的清晰标线上大幅领先，故取 0.20。
# 教训：只用一个域调参必然过拟合，至少要跨两个域看。
BAND_TOP = 0.20
BAND_BOT = 0.90          # 参考带 y 下界
CLUSTER_THRESH = 0.020   # 列占比阈值
# 中线附近的留白。2026-10-28 上板后默认关掉：斜视角下本车道的线会落到
# 画面中间，留白会把它整条切掉，那一侧永远判 none（用户：「中间3%那个算法
# 去掉，不要舍弃」）。参数保留是为了还能用命令行做对照。
MIN_GAP_FRAC = 0.0
# 虚线判定系数：dash 像素和 × 该系数 >= solid 像素和 才算虚线。
# 用户要求「虚线的权重要稍微高一些」。实测左侧明明是虚线却偶发跳实线，
# 原因是远处虚线段在画面里被拉长，solid 像素局部反超。
# 2026-10-29 用户：「虚线的阈值还要再低一些，让虚线更容易被判定」。
# 1.35 -> 1.60。配合 --solid-confirm-frames 2（实线要连续两轮），
# 虚线更容易成立、实线更难确认，两边一起把"跳实线"压下去。
DASH_BIAS = 1.60
# 簇宽上限，挡横穿的斑马线/停止线。
# 0.18 会误杀：官方留出集里有 3 条合法车道线宽度是 144~153 px（768 宽下的 0.18W=138），
# 超一点就被整簇丢弃，判成"无车道线"。放宽到 0.20（153 px）能救回 2 条，
# 方向准确率 G640 87%→88%，再往上不再涨。
MAX_WIDTH_FRAC = 0.20

# ---- 证据分级 ----
# 一簇内「虚线像素 + 实线像素」的列占比之和。官方留出集 100 个方向实测：
# 非零证据 p10≈0.10 / p50≈0.66 / p90≈4.8（单位是"占比×列数"）。
# 取 p10 作为分界：低于它的判定只由零星几列撑起，换一帧就翻面。
WEAK_EVIDENCE = 0.10

VARIANTS = {
    # name: (宽, 高)
    'gray640': (640, 360),
}


def fpga_gray(bgr):
    """FPGA 亮度公式。逐位一致，别换成 cv2.cvtColor(BGR2GRAY)。"""
    if bgr.ndim != 3:
        raise ValueError('需要 BGR 三通道图')
    b, g, r = (bgr[:, :, i].astype(np.int32) for i in (0, 1, 2))
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def prepare(gray, variant='gray640'):
    """灰度图 -> 模型输入 [1,1,H,W] float32 0..255（归一化在 ONNX 图内）。

    ⚠️⚠️ 2026-10-31 用户硬性要求：「RK 端坚决不要做二值化」。
    这里**只有一次缩放**，没有阈值、没有 where、没有 percentile
    —— 任何"把灰度变成黑白"的运算都不允许出现在 RK 端。
    """
    import cv2
    if variant not in VARIANTS:
        raise ValueError(f'未知车道变体 {variant}（只支持 {sorted(VARIANTS)}）')
    w, h = VARIANTS[variant]
    if gray.dtype != np.uint8 or gray.ndim != 2:
        raise ValueError('gray 必须是 uint8 单通道')
    v = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(v.astype(np.float32))[None, None]


def _clusters(score, thresh, x_lo, x_hi):
    """[x_lo,x_hi) 内 score>thresh 的连续段 -> [(中心x, 宽度, 峰值)]。

    中心取峰值平台中点而非 argmax：满覆盖的实线会形成一整段相同的平台值，
    argmax 只返回最左一格，边界位置会偏半个线宽。
    """
    seg = score[x_lo:x_hi] > thresh
    out = []
    i, n = 0, len(seg)
    while i < n:
        if seg[i]:
            j = i
            while j < n and seg[j]:
                j += 1
            sub = score[x_lo + i:x_lo + j]
            pk = float(sub.max())
            hit = np.flatnonzero(sub >= pk - 1e-6)
            k = i + int(hit[0] + hit[-1]) // 2
            out.append((x_lo + k, j - i, pk))
            i = j
        else:
            i += 1
    return out


def decode(mask, thresh=CLUSTER_THRESH, max_width_frac=MAX_WIDTH_FRAC,
           min_gap_frac=MIN_GAP_FRAC, band_top=BAND_TOP, band_bot=BAND_BOT):
    """3 类掩码 -> {'left': {...}, 'right': {...}}，各含 kind 与 evidence。

    ⚠️ 2026-10-28 上板后按用户要求改了两处（V17_3）：

    1. **不再在画面中线附近留 3% 空隙**（`min_gap_frac` 默认传 0）。
       中国路口/斜视角下，本车所在侧的车道线会落到画面中间，原来的空隙会
       直接把它切掉，那一条边就永远判成 none。用户原话：「中间3%那个算法
       去掉，不要舍弃」。
    2. **参考带放宽**，`band_top` 由 `lane_v2.BAND_TOP`（0.20）改成调用方传入，
       好让上面的车道线也参与统计。用户原话：「可以判别全屏，没必要只取下面
       一半」。默认值仍保留 0.20，行为不变。

    另外，虚线判据改成**比较两类的相对占比**而不是绝对像素和：实测左侧
    明明是虚线却偶发跳成实线，是因为远处虚线段在画面里被拉长，实线像素
    局部反超。虚线的判据因此加一个系数 `DASH_BIAS`（虚线更容易成立），
    与用户「虚线的权重要稍微高一些」的要求一致。
    """
    if mask.ndim != 2:
        raise ValueError('mask 必须是 2D')
    h, w = mask.shape
    a, b = int(h * band_top), int(h * band_bot)
    band = mask[a:max(b, a + 1)]
    bh = max(band.shape[0], 1)
    dash = (band == 1).sum(0).astype(np.float32) / bh
    solid = (band == 2).sum(0).astype(np.float32) / bh
    both = np.maximum(dash, solid)
    cx = w // 2
    gap = max(int(w * min_gap_frac), 0)
    max_w = max(int(w * max_width_frac), 2)

    def side(lo, hi):
        cs = [c for c in _clusters(both, thresh, lo, hi) if 1 <= c[1] <= max_w]
        if not cs:
            return {'kind': 'none', 'x': None, 'ev': 0.0, 'evidence': 'none'}
        x, wid, _ = min(cs, key=lambda t: abs(t[0] - cx))
        x0 = max(x - wid // 2 - 2, 0)
        x1 = min(x + wid // 2 + 3, w)
        sd = float(dash[x0:x1].sum())
        ss = float(solid[x0:x1].sum())
        if sd + ss < 1e-6:
            return {'kind': 'none', 'x': int(x), 'ev': 0.0, 'evidence': 'none'}
        ev = sd + ss
        return {'kind': 'dash' if sd * DASH_BIAS >= ss else 'solid', 'x': int(x),
                'ev': round(ev, 3),
                'evidence': 'weak' if ev < WEAK_EVIDENCE else 'ok'}

    return {'left': side(0, max(cx - gap, 1)), 'right': side(cx + gap, w)}


def mask_from_logits(out):
    """1×3×H×W 或 1×H×W×3 logits -> uint8 掩码（原地比较，比 argmax 快）。"""
    a = np.asarray(out)
    if a.ndim != 4:
        raise ValueError(f'期望 4 维输出，得到 {a.shape}')
    if a.shape[1] == 3:
        c0, c1, c2 = a[0, 0], a[0, 1], a[0, 2]
    elif a.shape[3] == 3:
        c0, c1, c2 = a[0, :, :, 0], a[0, :, :, 1], a[0, :, :, 2]
    else:
        raise ValueError(f'无法识别车道输出通道: {a.shape}')
    m = ((c1 > c0) & (c1 >= c2)).astype(np.uint8)
    m += 2 * ((c2 > c0) & (c2 > c1)).astype(np.uint8)
    return m
