#!/usr/bin/env python3
"""虚实判定 v2 —— 用「凹陷深度」而非绝对阈值，并输出可视化供肉眼核验

度量：
  沿车道线采样亮度剖面 prof
  归一化 p = prof / percentile(prof, 90)
  dash_score = p 中低于 0.6 的占比        (虚线→高, 实线→低)

同时存三张图：
  *.prof.png   剖面曲线（虚线应呈方波，实线应近似直线）
  *.vis.jpg    原图上按 dash_score 着色（绿=实线 红=虚线）
"""
import zipfile, sys
import numpy as np
import cv2

CULANE = r'D:\ADAS_Datasets\06_车道线_lane\_raw_culane.zip'
OUT = r'D:\ADAS_Datasets\临时_队友资料\lane_dash'


def imwrite_u(p, img):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])
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
        if len(v) < 4:
            continue
        out.append(np.array([float(x) for x in v]).reshape(-1, 2))
    return out


def sample_profile(img, pts, half=1):
    H, W = img.shape[:2]
    prof = []
    for x, y in pts:
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= yi < H):
            continue
        x0, x1 = max(0, xi - half), min(W, xi + half + 1)
        if x1 <= x0:
            continue
        prof.append(int(img[yi, x0:x1].max()))
    return np.array(prof, dtype=np.int32)


def dash_score(prof):
    if len(prof) < 10:
        return None
    hi = np.percentile(prof, 90)
    if hi < 30:                      # 太暗，没有车道线
        return None
    p = prof / hi
    return float((p < 0.6).mean())


def main():
    z = zipfile.ZipFile(CULANE)
    labels = sorted(n for n in z.namelist() if n.endswith('.lines.txt'))

    N = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    step = max(1, len(labels) // N)
    pick = labels[::step][:N]

    import os
    os.makedirs(OUT, exist_ok=True)

    scores, shown = [], 0
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

        vis = bgr.copy()
        curves = []
        for pts in parse_lines(txt):
            use = pts[pts[:, 1] > 400]
            if len(use) < 10:
                continue
            prof = sample_profile(gray, use)
            sc = dash_score(prof)
            if sc is None:
                continue
            scores.append(sc)
            col = (0, 0, 255) if sc > 0.15 else (0, 255, 0)   # 红=疑虚线 绿=疑实线
            cv2.polylines(vis, [use.astype(np.int32)], False, col, 3)
            curves.append((sc, prof))

        # 只存每条线得分差异大的图，便于对比
        if curves:
            ss = [c[0] for c in curves]
            if (max(ss) - min(ss) > 0.12 or shown < 4) and shown < 12:
                shown += 1
                tag = 's%02d' % shown
                imwrite_u(os.path.join(OUT, tag + '.jpg'), vis)
                # 剖面曲线
                h, w = 110, 420
                canvas = np.full((h * len(curves), w, 3), 245, np.uint8)
                for k, (sc, prof) in enumerate(curves):
                    p = prof / max(np.percentile(prof, 90), 1)
                    xs = (np.arange(len(p)) / max(len(p) - 1, 1) * (w - 20)).astype(int) + 10
                    ys = (h - 18 - np.clip(p, 0, 1.2) / 1.2 * (h - 30)).astype(int)
                    ys = np.clip(ys, 2, h - 2)
                    col = (0, 0, 255) if sc > 0.15 else (0, 140, 0)
                    for a, b in zip(zip(xs[:-1], ys[:-1]), zip(xs[1:], ys[1:])):
                        cv2.line(canvas, a, b, col, 1)
                    cv2.putText(canvas, 'line%d dash=%.2f' % (k, sc), (8, k * h + 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                imwrite_u(os.path.join(OUT, tag + '_prof.png'), canvas)
        if (i + 1) % 20 == 0:
            print('  %d/%d' % (i + 1, len(pick)))

    s = np.array(scores)
    print()
    print('车道线样本 %d 条' % len(s))
    print('dash_score 分布: mean=%.3f median=%.3f' % (s.mean(), np.median(s)))
    print('  <0.05 (疑似实线): %5.1f%%' % (100 * (s < 0.05).mean()))
    print('  0.05~0.15      : %5.1f%%' % (100 * ((s >= 0.05) & (s < 0.15)).mean()))
    print('  >0.15 (疑似虚线): %5.1f%%' % (100 * (s > 0.15).mean()))
    print()
    print('可视化:', OUT)


if __name__ == '__main__':
    main()
