# -*- coding: utf-8 -*-
"""红绿灰度化（中国场景校准版）+ 灯箱预填

═══ 与旧 v11 的区别 ═══
v11 在中国场景下把【蓝色天空】判成绿(天空 G-R≈58 通过 G>R+28)，
导致整片天空染绿。根因: 缺 B 侧约束 —— 真绿灯 G>B，天空 G<B。

修正后的绿判据:  G >= 60  &  G > R + 45  &  G > B
   实测(官方34颗人工标注真绿灯): 单灯检出 94.1%
   树冠 FPR 49%→0.7%,  天空 FPR 70%→0.3%

红判据沿用 v11 (官方实测保留率红 94%):
    R >= 96  &  R*5 > G*8  &  R + 16 >= B
"""
import numpy as np
import cv2

# ---- 绿（中国场景校准）----
# 扫描结果(G>=80 & G-R>=55 & G>B & minArea>=80 & fill>=0.5):
#   真绿灯单灯检出 94.1% (32/34),  树冠/天空假斑 0.81/大区
GR_MIN = 80
GR_SUB_R = 55
GR_NEED_B = True

# ---- 红（沿用 v11）----
RD_MIN = 96
RD_MUL_G = 5        # R*5 > G*8  == R > 1.6G
RD_ADD_B = 16

# ---- 灰度 ----
W_R, W_G, W_B = 77, 150, 29


def _y(r, g, b):
    return ((W_R * r + W_G * g + W_B * b) >> 8).astype(np.uint8)


def rg_gray(bgr):
    """红绿保留灰度化（中国校准版）-> BGR uint8, 命中处纯红/纯绿, 其余灰度"""
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    red_keep = (r >= RD_MIN) & (r * RD_MUL_G > g * 8) & (r + RD_ADD_B >= b)
    green_keep = (g >= GR_MIN) & (g - r >= GR_SUB_R) & (g > b)
    out = np.dstack([_y(r, g, b)] * 3)
    out[red_keep] = (0, 0, 255)
    out[green_keep] = (0, 255, 0)
    return out


def masks(bgr):
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((r >= RD_MIN) & (r * RD_MUL_G > g * 8) & (r + RD_ADD_B >= b),
            (g >= GR_MIN) & (g - r >= GR_SUB_R) & (g > b))


def lamp_blobs(gray, min_area=80, ar_lo=0.30, ar_hi=2.2, fill_min=0.50):
    """灰度图上找灯珠斑块（此时只有真红/真绿有色）"""
    hsv = cv2.cvtColor(gray, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    H = hsv[:, :, 0].astype(np.int16)
    m = ((S >= 150) & (V >= 60)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < min_area:
            continue
        ar = w / max(h, 1)
        if not (ar_lo <= ar <= ar_hi):
            continue
        if a / max(w * h, 1) < fill_min:
            continue
        hh = float(np.median(H[lab == i]))
        col = "red" if (hh < 15 or hh > 165) else ("green" if 35 <= hh <= 95 else None)
        if col is None:
            continue
        out.append((x, y, x + w, y + h, col, a, w, h, 0.0, 255.0))
    return out


def expand_to_housing(gray, blobs, orient="h", span=3.05, thick=1.15,
                      snap=True):
    """把"单颗灯珠斑块"扩成"整个灯箱"，作为标注预填。

    灯箱几何（中国横排，三灯位）:
        灯珠直径 d ≈ 斑块高 h
        灯箱宽 ≈ span*d  (三灯位排开)
        灯箱高 ≈ thick*d (含壳体边框)
        灯珠在灯箱内的位置: 左位/中位/右位 -> 据此把灯箱框对齐

    orient="h" 横排(中国主干道) / "v" 竖排(S2TLD等)
    snap=True 时在候选框附近找【暗壳边缘】微调，否则纯几何推算。
    """
    H, W = gray.shape[:2]
    yv = gray.mean(axis=2) if gray.ndim == 3 else gray
    out = []
    for b in blobs:
        x1, y1, x2, y2, col = b[0], b[1], b[2], b[3], b[4]
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        if orient == "h":
            d = float(y2 - y1)
            w, h = span * d, thick * d
        else:
            d = float(x2 - x1)
            w, h = thick * d, span * d
        bx1, by1, bx2, by2 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2

        if snap:
            # 在候选框边界附近找暗壳(壳比灯珠暗很多)
            m = 40.0
            r1, r2 = max(0, int(by1 - m)), min(H, int(by2 + m))
            c1, c2 = max(0, int(bx1 - m)), min(W, int(bx2 + m))
            if r2 - r1 > 8 and c2 - c1 > 8:
                sub = yv[r1:r2, c1:c2]
                prof_hi = sub.mean(axis=1)   # 每行亮度
                prof_wd = sub.mean(axis=0)   # 每列亮度
                if orient == "h":
                    # 上下找暗壳
                    mid = sub.shape[0] // 2
                    k = mid
                    while k > 0 and prof_hi[k] > 70:
                        k -= 1
                    if k > 0:
                        by1 = r1 + k
                    k = mid
                    while k < sub.shape[0] - 1 and prof_hi[k] > 70:
                        k += 1
                    if k < sub.shape[0] - 1:
                        by2 = r1 + k
                else:
                    mid = sub.shape[1] // 2
                    k = mid
                    while k > 0 and prof_wd[k] > 70:
                        k -= 1
                    if k > 0:
                        bx1 = c1 + k
                    k = mid
                    while k < sub.shape[1] - 1 and prof_wd[k] > 70:
                        k += 1
                    if k < sub.shape[1] - 1:
                        bx2 = c1 + k
                    by1 = cy - (span * (x2 - x1)) / 2
                    by2 = cy + (span * (x2 - x1)) / 2

        # 越界修正: 保持尺寸, 平移回来
        if bx1 < 0:
            bx2 -= bx1; bx1 = 0
        if by1 < 0:
            by2 -= by1; by1 = 0
        if bx2 > W:
            bx1 -= (bx2 - W); bx2 = W
        if by2 > H:
            by1 -= (by2 - H); by2 = H
        bx1 = max(0, bx1); by1 = max(0, by1)
        out.append((int(bx1), int(by1), int(bx2), int(by2), col,
                    b[5], int(bx2 - bx1), int(by2 - by1)))
    return out


def merge_row(blobs, y_tol=0.75, x_gap=2.6, size_tol=0.75):
    """同一灯箱的灯珠合成大框（仅当多颗灯同时亮时才会合并）"""
    if not blobs:
        return []
    bs = sorted(blobs, key=lambda b: b[0])
    used = [False] * len(bs); groups = []
    for i in range(len(bs)):
        if used[i]:
            continue
        g = [bs[i]]; used[i] = True; ch = True
        while ch:
            ch = False
            gx1 = min(b[0] for b in g); gx2 = max(b[2] for b in g)
            gy1 = min(b[1] for b in g); gy2 = max(b[3] for b in g)
            gcy = (gy1 + gy2) / 2; gh = gy2 - gy1
            gha = np.mean([b[7] for b in g]); gwa = np.mean([b[6] for b in g])
            for j in range(len(bs)):
                if used[j]:
                    continue
                b = bs[j]
                bcy = (b[1] + b[3]) / 2; bh = b[7]; bw = b[6]
                if abs(gcy - bcy) > max(gh, bh) * y_tol:
                    continue
                if abs(bh - gha) > max(gha, bh) * size_tol:
                    continue
                if abs(bw - gwa) > max(gwa, bw) * (size_tol + 0.3):
                    continue
                if b[0] > gx2:
                    gap = b[0] - gx2
                elif b[2] < gx1:
                    gap = gx1 - b[2]
                else:
                    gap = 0
                if gap > max(bw, gwa) * x_gap:
                    continue
                g.append(b); used[j] = True; ch = True
        groups.append(g)
    res = []
    for g in groups:
        x1 = min(b[0] for b in g); y1 = min(b[1] for b in g)
        x2 = max(b[2] for b in g); y2 = max(b[3] for b in g)
        main = max(g, key=lambda b: b[5])[4]
        res.append((x1, y1, x2, y2, main, len(g), x2 - x1, y2 - y1))
    return res
