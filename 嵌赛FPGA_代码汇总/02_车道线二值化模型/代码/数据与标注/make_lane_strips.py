#!/usr/bin/env python3
"""车道线条带 v2 —— 密集插值 + 放大，肉眼可辨虚实

问题：CULane 标注 y 每 10px 一个点，y∈[590,410] 只有 19 个点，
      直接画条带太扁看不见。
做法：把折线沿 y 方向插值到每 2px 一个采样点（~90 点），再纵向放大 4 倍。

每组 3 列：
  A = 原始灰度 (Y=(77R+150G+29B)>>8)
  B = A + 3x3 中值滤波        (FPGA 第2级)
  C = B + 二值化(>110)        (FPGA 第3级)
实线 → A/B/C 都连续
虚线 → A/B 有条纹, C 断成一截截
"""
import zipfile, os, sys
import numpy as np
import cv2

CULANE = r'D:\ADAS_Datasets\06_车道线_lane\_raw_culane.zip'
OUT = r'D:\ADAS_Datasets\临时_队友资料\lane_strips'
SCALE = 4


def imwrite_u(p, img):
    ok, buf = cv2.imencode('.png', img)
    if ok:
        buf.tofile(p)


def rgb2gray_fpga(bgr):
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def parse_lines(txt):
    out = []
    for line in txt.strip().split('\n'):
        v = line.split()
        if len(v) >= 4:
            out.append(np.array([float(x) for x in v]).reshape(-1, 2))
    return out


def dense(pts, y_lo, y_hi, step=2.0):
    """按 y 排序后线性插值，返回 (xs, ys) 稠密序列"""
    if pts is None or len(pts) < 2:
        return None, None
    p = pts[np.argsort(pts[:, 1])]
    if len(p) < 2 or p[0, 1] == p[-1, 1]:
        return None, None
    ys = p[:, 1]
    xs = p[:, 0]
    lo = max(y_lo, ys[0])
    hi = min(y_hi, ys[-1])
    if hi - lo < 20:
        return None, None
    grid = np.arange(lo, hi, step)
    if len(grid) < 8:
        return None, None
    return np.interp(grid, ys, xs), grid


def profile(img, xs, ys, half=2):
    H, W = img.shape[:2]
    out = []
    for x, y in zip(xs, ys):
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= yi < H):
            continue
        x0, x1 = max(0, xi - half), min(W, xi + half + 1)
        if x1 <= x0:
            continue
        out.append(int(img[yi, x0:x1].max()))
    return np.array(out, dtype=np.uint8)


def main():
    z = zipfile.ZipFile(CULANE)
    labels = sorted(n for n in z.namelist() if n.endswith('.lines.txt'))
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    step = max(1, len(labels) // N)
    pick = labels[::step][:N]
    os.makedirs(OUT, exist_ok=True)

    SW, GAP = 34, 10
    made = 0
    for i, lp in enumerate(pick):
        ip = lp.replace('.lines.txt', '.jpg')
        try:
            bgr = cv2.imdecode(np.frombuffer(z.read(ip), np.uint8), cv2.IMREAD_COLOR)
            txt = z.read(lp).decode('utf-8', errors='ignore')
        except Exception:
            continue
        if bgr is None:
            continue
        gray = rgb2gray_fpga(bgr)
        med = cv2.medianBlur(gray, 3)
        bina = np.where(med > 110, 255, 0).astype(np.uint8)

        cols = []          # 每条线一组三列
        for pts in parse_lines(txt):
            xs, ys = dense(pts, 415, 590)
            if xs is None:
                continue
            trio = [profile(img, xs, ys, 2) for img in (gray, med, bina)]
            if min(len(t) for t in trio) < 20:
                continue
            L = min(len(t) for t in trio)
            cols.append([t[:L] for t in trio])

        if not cols:
            continue
        H = max(len(c[0]) for c in cols) * SCALE
        W = (SW + GAP) * 3 * len(cols) + GAP
        canvas = np.full((H + 16, W, 3), 255, np.uint8)

        for k, tri in enumerate(cols):
            for m, s in enumerate(tri):
                if m != 2:
                    hi = max(int(np.percentile(s, 95)), 1)
                    s = np.clip(s.astype(np.float32) / hi * 255, 0, 255).astype(np.uint8)
                band = np.repeat(np.repeat(s[:, None], SW, 1), SCALE, 0)
                x0 = GAP + k * 3 * (SW + GAP) + m * (SW + GAP)
                canvas[16:16 + len(band), x0:x0 + SW] = np.stack([band] * 3, 2)
            xsep = GAP + k * 3 * (SW + GAP) + 3 * (SW + GAP) - GAP // 2
            canvas[16:, xsep:xsep + 2] = (180, 180, 255)

        for m, lbl in enumerate(['A raw', 'B med', 'C bin']):
            cv2.putText(canvas, lbl, (GAP + m * (SW + GAP), 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)
        cv2.putText(canvas, os.path.basename(ip)[:28], (W - 260, 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1)
        imwrite_u(os.path.join(OUT, 'strip_%02d.png' % i), canvas)
        made += 1

    print('生成 %d 张条带 ->' % made, OUT)
    print()
    print('怎么看：')
    print('  实线样本 → A/B/C 三列都是连续亮带')
    print('  虚线样本 → A 是明暗相间的方块序列')
    print('  对比 A 与 C 就知道虚线在二值化后是否还留下断口')


if __name__ == '__main__':
    main()
