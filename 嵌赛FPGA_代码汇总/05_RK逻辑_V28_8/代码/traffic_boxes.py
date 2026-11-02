"""灯箱框过滤：把"模型觉得像灯、其实不是灯"的框挡在判色之前。

## 为什么需要它

灯箱模型只回答"画面里哪里有像灯箱的东西"，它会稳定地交上三类误检：

  * 倒计时数字（"58"、"8"）—— 笔画填满整框
  * 限速牌（"60"）—— 红色圆牌，框内没有纯红/纯绿
  * 尾灯 / 钩形鬼影 —— 反射、压缩残块

一块假框一旦进入 `traffic_rules.judge()`，就会占用 LEFT/STRAIGHT/RIGHT
里的一个位置，让**同组真灯的整体方向语义错位**——这是"红绿灯乱跳"的直接来源。

## 判据（V27，用户 2026-10-31 定的终极版）

**第一步：长边三等分颜色集中度**

沿框的**较长边**把框等分成三格，统计纯红(254)/纯绿(255)像素落在哪一格：

```
真灯  —— 灯珠只占灯箱的一小段，颜色集中在 <0.55 格
       （画面上就是一个亮圆点/箭头落在灯箱某一格）
数字  —— 笔画铺满整框，颜色摊在 2~3 格，最大一格占比 <0.55
```

红绿**分别**算，取较大的那个占比。不合并算：横排灯箱里"左转红 + 直行绿"
同时亮时，合并会把两种颜色摊到两格，把真灯误杀。

新增的第三条：框内色像素 < `UNLIT_PIXELS` → `unlit`（没亮的框不占方向位）。
这条是第一步的自然结果，但单独给个 reason，日志里好认。

**第二步：纵向高度统计（剔行人指示灯 / 计数牌）**

对第一步留下的框，按 `cy/H` 归成同一行（容差 `ROW_CY_TOL`），
行内 `h < ROW_H_RATIO × 行内中位 h` 的框判为陪衬（`row_small`），
离行心太远的框判为离群（`row_far`）。

⚠️ 行内不足 `ROW_MIN_ITEMS` 个框时**整行跳过**——没有参照就不判断。
单帧只框到一两个框时，误删真灯的代价远大于漏删一个数字。

## 为什么用 YRG 数 254/255 而不是还原 RGB

FPGA 把真红编码成 254、真绿编码成 255，其余是灰度 0~253。所以"这个框里
颜色落在哪一格"直接在这张单通道图上算——精确、零阈值，和
`traffic_rules._colour_of` 用的是同一套判据，不会出现"闸门觉得有灯、
判色却读不到"的矛盾。

⚠️ **只在真灰度帧上用**。二值帧（阈值图，取值塌缩成 {0,253}）里根本
没有 254/255，所有框都会被判成 `unlit`；二值帧走 traffic_hold 是另一条路。
"""
from __future__ import annotations

import numpy as np

# 长边等分成几格（= 小窗口宽度 = 长边 / 3）
THIRDS = 3

# 小窗口最多能装下多少比例的颜色像素。低于它 = 颜色摊开了 = 不是灯。
# 用户 2026-10-31 要求「70% 大致分布在一个小框内」。
# ⚠️ 窗口必须能**沿长边滑动**，不能死钉在三等分边界上：
# 模型框不精确，灯珠常偏在一侧，固定三格会把它切成两格
# （板端 38 框实测：固定 @0.70 真灯只剩 28/38，可滑动是 37/38）。
SHARE_MIN = 0.70

# 框内纯红/纯绿像素少于这个数 = 这盏灯没亮（与 traffic_rules 保持一致）。
UNLIT_PIXELS = 20

# ---- 第二步：行内高度统计 ----
ROW_CY_TOL = 0.10        # 同一行的 cy/H 容差
ROW_H_RATIO = 0.88       # 行内 h < 此比例 × 中位 h -> 判为陪衬（数字/指示牌）
ROW_MIN_ITEMS = 3        # 行内至少几个框才做高度统计（不足则整行跳过）


def _crop(yrg, box):
    h, w = yrg.shape[:2]
    x1 = max(int(round(box[0])), 0)
    y1 = max(int(round(box[1])), 0)
    x2 = min(int(round(box[2])), w)
    y2 = min(int(round(box[3])), h)
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    return yrg[y1:y2, x1:x2], (x1, y1, x2, y2)


def thirds_share(crop, colour, n=THIRDS):
    """沿长边取一个「宽度 = 长边/n」的窗口，滑动找最集中的位置。

    返回 (窗口内占比, 该颜色的像素总数)。

    横长框沿列滑、竖长框沿行滑 —— 灯珠永远只占"长边方向"的一小段。

    ⚠️ 为什么滑动而不是固定三等分：模型给的框不精确，灯珠经常偏在框的
    一侧（板端 seq1 的两盏灯 share 只有 0.60/0.58）。死钉在三等分边界上
    会把灯珠切进相邻格，0.70 的门槛下会误杀大量真灯。滑动等价于"颜色
    在长边上是否有 1/3 长度的一段连续集中"，这才是判据想表达的东西。
    """
    mask = (crop == colour)
    total = int(mask.sum())
    if total == 0:
        return 0.0, 0
    h, w = mask.shape
    profile = mask.sum(axis=0 if w >= h else 1).astype(np.int32)
    length = int(profile.shape[0])
    win = max(1, int(round(length / float(n))))
    if win >= length:
        return 1.0, total
    cum = np.concatenate(([0], np.cumsum(profile)))
    best = int((cum[win:] - cum[:-win]).max())
    return best / float(total), total


def box_features(yrg, box):
    """一个框的外观特征。返回 None 表示框退化（面积太小）。"""
    got = _crop(yrg, box)
    if got is None:
        return None
    c, (x1, y1, x2, y2) = got
    h, w = yrg.shape[:2]
    bw, bh = x2 - x1, y2 - y1
    share_red, n_red = thirds_share(c, 254)
    share_green, n_green = thirds_share(c, 255)
    share = max(share_red if n_red >= UNLIT_PIXELS else 0.0,
                share_green if n_green >= UNLIT_PIXELS else 0.0)
    ncolour = n_red + n_green
    return {
        'ar': max(bw, bh) / float(max(1, min(bw, bh))),
        'cy': (y1 + y2) / 2.0 / h,
        'cf': ncolour / float(bw * bh),
        'share': share,
        'ncolour': ncolour,
        'n_red': n_red,
        'n_green': n_green,
        'w': bw, 'h': bh,
    }


def reject_reason(f):
    """第一步：颜色是否集中在长边的某一格。保留返回 None。"""
    if f is None:
        return 'degenerate'
    if f['ncolour'] < UNLIT_PIXELS:
        return 'unlit'
    if f['share'] < SHARE_MIN:
        return 'spread'
    return None


def mark_row_outliers(survived):
    """第二步：行内高度统计，给陪衬框/离群框打上 reason。只改 reason 不删元素。"""
    live = [f for f in survived if f['reason'] == 'ok']
    if len(live) < ROW_MIN_ITEMS:
        return survived
    cys = sorted(f['cy'] for f in live)
    med_cy = cys[len(cys) // 2]
    row = [f for f in live if abs(f['cy'] - med_cy) <= ROW_CY_TOL]
    if len(row) < ROW_MIN_ITEMS:
        return survived
    med_h = sorted(f['h'] for f in row)[len(row) // 2]
    for f in live:
        if abs(f['cy'] - med_cy) > ROW_CY_TOL:
            f['reason'] = 'row_far'
        elif f['h'] < ROW_H_RATIO * med_h:
            f['reason'] = 'row_small'
    return survived


def filter_boxes(boxes, yrg):
    """-> (保留的框, 被拒的 [(box, reason, features)])。

    ⚠️ 框数是方向语义的输入。多留一个假框 = 整体方向错位；
    多删一个真框 = 凭空禁止一个方向。所以两条判据都只在"有参照"时生效：
    颜色集中度看框本身，高度统计看同一行里的其他框。
    """
    survived, rejected = [], []
    for b in boxes:
        f = box_features(yrg, b)
        why = reject_reason(f)
        if why is None:
            f['reason'] = 'ok'
            f['box'] = b
            survived.append(f)
        else:
            rejected.append((b, why, f))

    mark_row_outliers(survived)
    kept, late = [], []
    for f in survived:
        if f['reason'] == 'ok':
            kept.append(f['box'])
        else:
            late.append((f['box'], f['reason'], f))
    return kept, rejected + late
