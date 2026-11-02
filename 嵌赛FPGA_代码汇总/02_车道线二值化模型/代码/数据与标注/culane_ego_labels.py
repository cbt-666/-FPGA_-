#!/usr/bin/env python3
"""CULane -> (左边界, 右边界) 虚实标签

CULane 标注：每行一条车道线的 (x,y) 序列（y 从 590 递减到 410）。
本脚本：
  1. 取 y=580（近处）那一列，找画面中心左右两侧最近的两条线
     -> 分别是自车车道的 左边界 / 右边界
  2. 沿每条线的折线取亮度剖面（在 FPGA 灰度图上）
  3. 判断虚实：剖面归一化后低亮度占比（虚线高）
  4. 输出可视化供人工核验

关键：CULane 给的是**真实车道线位置**，不用我猜，所以剖面一定落在线上。
"""
import zipfile
import numpy as np
import cv2
import os
import sys
import csv

CULANE = r'D:\ADAS_Datasets\06_车道线_lane\_raw_culane.zip'
OUT = r'D:\ADAS_Datasets\06_车道线_lane\_culane_check'


def imwrite_u(p, img, q=90):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


def fpga_gray(bgr):
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def parse_lines(txt):
    out = []
    for line in txt.strip().split('\n'):
        v = line.split()
        if len(v) >= 4:
            p = np.array([float(x) for x in v]).reshape(-1, 2)
            out.append(p[np.argsort(p[:, 1])])       # 按 y 升序
    return out


def x_at(line, y):
    """线在指定 y 处的 x（线性插值）"""
    ys = line[:, 1]
    if y < ys[0] or y > ys[-1]:
        return None
    return float(np.interp(y, ys, line[:, 0]))


def profile(G, line, y0, y1, half=2):
    """沿 line 在 [y0,y1] 区间采样亮度，横向 ±half 取最大"""
    H, W = G.shape
    vs, ys = [], []
    for y in range(y0, y1):
        x = x_at(line, y)
        if x is None:
            continue
        xi = int(round(x))
        if xi - half < 0 or xi + half >= W:
            continue
        vs.append(int(G[y, xi - half:xi + half + 1].max()))
        ys.append(y)
    return np.array(vs, np.float32), np.array(ys)


def classify(vs):
    """虚实判定：归一化剖面的低亮度占比 + 对比度检查"""
    if len(vs) < 30:
        return None, 'short', None
    v = vs.astype(np.float32)
    hi = np.percentile(v, 90)
    lo = np.percentile(v, 10)
    if hi < 40:
        return None, 'dark', None
    if hi - lo < 15:
        return None, 'flat', None
    p = v / hi
    return float((p < 0.62).mean()), 'ok', (lo, hi)


def main():
    z = zipfile.ZipFile(CULANE)
    labels = sorted(n for n in z.namelist() if n.endswith('.lines.txt'))
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    step = max(1, len(labels) // N)
    pick = labels[::step][:N]
    os.makedirs(OUT, exist_ok=True)
    print('抽样 %d 张' % len(pick))

    rows = []
    shown = 0
    for i, lp in enumerate(pick):
        ip = lp.replace('.lines.txt', '.jpg')
        try:
            bgr = cv2.imdecode(np.frombuffer(z.read(ip), np.uint8), cv2.IMREAD_COLOR)
            txt = z.read(lp).decode('utf-8', errors='ignore')
        except Exception:
            continue
        if bgr is None:
            continue
        G = fpga_gray(bgr)
        H, W = G.shape
        cx = W / 2.0
        lines = parse_lines(txt)
        # 在近处 y 处找中心左右最近的两条线
        yprobe = int(H * 0.97)          # 最底行附近
        cand = []
        for ln in lines:
            x = x_at(ln, yprobe)
            if x is None:
                # 用该线最底端
                if ln[-1, 1] < H - 5:
                    continue
                x = ln[-1, 0]
            cand.append((x, ln))
        L = max([c for c in cand if c[0] < cx], key=lambda t: t[0], default=None)
        R = min([c for c in cand if c[0] >= cx], key=lambda t: t[0], default=None)

        rec = {'file': os.path.basename(ip)}
        vis = bgr.copy()
        for side, c in (('L', L), ('R', R)):
            if c is None:
                rec[side] = 'none'
                continue
            vs, ys = profile(G, c[1], 425, 585)
            sc, why, rng = classify(vs)
            if sc is None:
                rec[side] = 'unk:' + why
                col = (0, 200, 255)
            else:
                rec[side] = 'dash' if sc > 0.18 else 'solid'
                col = (0, 0, 255) if rec[side] == 'dash' else (0, 255, 0)
            # 画线
            pts = c[1][(c[1][:, 1] >= 420) & (c[1][:, 1] <= 590)]
            if len(pts) > 1:
                cv2.polylines(vis, [pts.astype(np.int32)], False, col, 4)
                cv2.putText(vis, '%s:%s' % (side, rec[side]),
                            (int(pts[len(pts) // 2][0]) - 40, int(pts[len(pts) // 2][1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
        rows.append(rec)
        if i % 12 == 0 and shown < 24:
            shown += 1
            imwrite_u(os.path.join(OUT, 'c_%03d.jpg' % i), vis)
        if (i + 1) % 100 == 0:
            print('  %d/%d' % (i + 1, len(pick)))

    import collections
    cnt = collections.Counter((r['L'], r['R']) for r in rows)
    print()
    print('左右标签组合分布:')
    for k, v in cnt.most_common(12):
        print('   L=%-12s R=%-12s %4d (%.1f%%)' % (k[0], k[1], v, 100 * v / len(rows)))
    with open(os.path.join(OUT, 'labels.csv'), 'w', newline='', encoding='utf-8') as fh:
        w = csv.DictWriter(fh, fieldnames=['file', 'L', 'R'])
        w.writeheader()
        w.writerows(rows)
    print()
    print('可视化:', OUT)


if __name__ == '__main__':
    main()
