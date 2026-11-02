#!/usr/bin/env python3
"""改进版解码器 —— 去掉「硬性中心分界」，改用消失点估计 + 形状判据

原 decode() 的两个假设在【视角不正】时会崩（用户 2026-10-08 指出）：
  1. 「左边界必在左半、右边界必在右半」—— 视角一斜，消失点偏离画面中心，
     自车车道的左线可能落在右半（或反之），硬分界会把它劈成两半
  2. 宽度上限 max_width_frac(7~10%) —— 在官方正视素材上校准的。视角越斜，
     车道线在**列方向**的投影越宽（近水平的线横扫很大 x 范围），很容易超限被误杀

改法：
  · ref_x 用「参考带内线像素的质心」估计，而不是写死 W//2
  · 丢簇判据从「宽度上限」改成「形状」：车道线是**细长的** ——
    要求该簇在带内**竖直跨度够大**（跨越多行），横穿的斑马线/假阳性块竖直跨度小
  · 宽度上限放宽到 0.25（只挡掉横穿整屏的东西，不再误杀斜视下的长线）
"""
import numpy as np


def _clusters(score, thresh, x_lo, x_hi):
    seg = score[x_lo:x_hi] > thresh
    out, i, n = [], 0, len(seg)
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


def decode2(mask, thresh=0.020, min_width=1, max_width_frac=0.25,
            min_vext=0.35):
    """mask: HxW uint8, 0=背景 1=虚线 2=实线

    min_vext: 簇的竖直跨度下限（占参考带高度的比例）。
              车道线是长条 -> 跨很多行；斑马线/误检大块 -> 跨行少。
    """
    H, W = mask.shape
    a, b = int(H * 0.45), int(H * 0.90)
    band = mask[a:b]
    bh = max(b - a, 1)
    dash = (band == 1).sum(0).astype(np.float32) / bh
    solid = (band == 2).sum(0).astype(np.float32) / bh
    both = np.maximum(dash, solid)

    # ref_x = 带内线像素的横向质心（粗略的"自车正前方"），视角斜了它会跟着移
    cols = np.flatnonzero(both > thresh)
    if cols.size:
        wts = both[cols]
        ref_x = int(round((cols * wts).sum() / max(wts.sum(), 1e-6)))
    else:
        ref_x = W // 2
    ref_x = int(np.clip(ref_x, W * 0.15, W * 0.85))

    max_w = max(int(W * max_width_frac), min_width + 1)

    def vext(x0, x1):
        """该列区间内，带里有线像素的行数占比"""
        sub = band[:, max(x0, 0):min(x1, W)]
        rows = (sub > 0).any(1)
        return float(rows.mean())

    def side(lo, hi, anchor):
        cs = []
        for x, w, _pk in _clusters(both, thresh, lo, hi):
            if not (min_width <= w <= max_w):
                continue
            x0, x1 = max(x - w // 2 - 2, 0), min(x + w // 2 + 3, W)
            if vext(x0, x1) < min_vext:
                continue
            cs.append((x, w, x0, x1))
        if not cs:
            return 'none', None, ref_x
        x, w, x0, x1 = min(cs, key=lambda t: abs(t[0] - anchor))
        sd, ss = float(dash[x0:x1].sum()), float(solid[x0:x1].sum())
        if sd + ss < 1e-6:
            return 'none', None, ref_x
        return ('dash' if sd > ss else 'solid'), int(x), ref_x

    L, lx, _ = side(0, ref_x, ref_x)
    R, rx, _ = side(ref_x, W, ref_x)
    return {
        'left': L, 'left_x': lx, 'right': R, 'right_x': rx, 'ref_x': ref_x,
        'can_change_left': L == 'dash', 'can_change_right': R == 'dash',
    }
