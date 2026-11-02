#!/usr/bin/env python3
"""队友 FPGA 变换的精确 Python 复刻 —— rgb888_yrg

Verilog 原文 (红绿灯/rgb888_ycbcr.v):
    parameter RED_MIN=96, GREEN_MIN=56, GREEN_MARGIN=16
    red_keep_in   = (r >= RED_MIN) && (r > 2g) && (r > 2b)
    green_keep_in = (g >= GREEN_MIN) && (g > r+GREEN_MARGIN) && (g > b+GREEN_MARGIN)
    Y = (77R + 150G + 29B) >> 8
    data = red_keep ? 24'hFF0000 : green_keep ? 24'h00FF00 : {Y,Y,Y}

注意：输出是**纯红/纯绿/灰**三值图，不是保留原始红绿像素。
"""
import numpy as np
import cv2

RED_MIN = 96
GREEN_MIN = 56
GREEN_MARGIN = 16


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_u(p, img):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if ok:
        buf.tofile(p)
    return ok


def fpga_rg_gray(img_bgr, red_min=RED_MIN, green_min=GREEN_MIN,
                 green_margin=GREEN_MARGIN, median=False):
    """精确复刻 rgb888_yrg。img_bgr: HxWx3 uint8 (OpenCV BGR)"""
    b = img_bgr[:, :, 0].astype(np.int32)
    g = img_bgr[:, :, 1].astype(np.int32)
    r = img_bgr[:, :, 2].astype(np.int32)

    red_keep = (r >= red_min) & (r > 2 * g) & (r > 2 * b)
    grn_keep = (g >= green_min) & (g > r + green_margin) & (g > b + green_margin)

    y = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
    out = np.stack([y, y, y], axis=2)

    if median:
        # 队友 chain: grayscale -> median_filter_3x3
        out = cv2.medianBlur(out, 3)
        # 中值滤波后重新贴色（与硬件顺序一致需确认，这里先给两种看看）
        # 实际上硬件是在 rgb 域判色后再滤波的话，颜色会被拉花；
        # 这里按「滤波在灰度域、颜色后叠加」实现
        b2 = cv2.medianBlur(img_bgr, 3)[:, :, 0].astype(np.int32)
        g2 = cv2.medianBlur(img_bgr, 3)[:, :, 1].astype(np.int32)
        r2 = cv2.medianBlur(img_bgr, 3)[:, :, 2].astype(np.int32)
        red_keep = (r2 >= red_min) & (r2 > 2 * g2) & (r2 > 2 * b2)
        grn_keep = (g2 >= green_min) & (g2 > r2 + green_margin) & (g2 > b2 + green_margin)

    out[red_keep] = (0, 0, 255)      # BGR 纯红
    out[grn_keep] = (0, 255, 0)      # BGR 纯绿
    return out


def stats(img_bgr, **kw):
    b = img_bgr[:, :, 0].astype(np.int32)
    g = img_bgr[:, :, 1].astype(np.int32)
    r = img_bgr[:, :, 2].astype(np.int32)
    rk = (r >= kw.get('red_min', RED_MIN)) & (r > 2 * g) & (r > 2 * b)
    gk = (g >= kw.get('green_min', GREEN_MIN)) & \
         (g > r + kw.get('green_margin', GREEN_MARGIN)) & \
         (g > b + kw.get('green_margin', GREEN_MARGIN))
    n = img_bgr.shape[0] * img_bgr.shape[1]
    return 100.0 * rk.sum() / n, 100.0 * gk.sum() / n


if __name__ == '__main__':
    import sys, glob, os
    tag = sys.argv[1] if len(sys.argv) > 1 else 'enable'
    files = sorted(glob.glob('D:/ADAS_Datasets/临时_队友资料/tl_all/%s/*.jpg' % tag))
    print('目录 %s, 共 %d 张' % (tag, len(files)))

    # 统计红/绿命中率
    rs, gs = [], []
    for f in files[:60]:
        im = imread_u(f)
        if im is None:
            continue
        a, b_ = stats(im)
        rs.append(a); gs.append(b_)
    print('红像素占比  mean=%.3f%%  max=%.2f%%' % (np.mean(rs), np.max(rs)))
    print('绿像素占比  mean=%.3f%%  max=%.2f%%' % (np.mean(gs), np.max(gs)))
    print('红=0 的图数: %d/%d' % (sum(1 for x in rs if x == 0), len(rs)))
    print('绿=0 的图数: %d/%d' % (sum(1 for x in gs if x == 0), len(gs)))

    # 拼 before/after 对比图
    sel = files[:8]
    rows = []
    for f in sel:
        im = imread_u(f)
        tr = fpga_rg_gray(im)
        h, w = 200, 356
        a = cv2.resize(im, (w, h))
        b_ = cv2.resize(tr, (w, h))
        cv2.putText(a, 'before '+os.path.basename(f), (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        ra, gb = stats(im)
        cv2.putText(b_, 'after  R%.2f%% G%.2f%%' % (ra, gb), (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        rows.append(np.hstack([a, b_]))
    out = np.vstack(rows)
    dst = 'D:/ADAS_Datasets/临时_队友资料/cmp_%s.jpg' % tag
    imwrite_u(dst, out)
    print('对比图:', dst)
