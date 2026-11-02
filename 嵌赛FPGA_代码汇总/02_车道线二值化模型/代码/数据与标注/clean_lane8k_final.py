#!/usr/bin/env python3
"""LaneMarking8K 最终清洗 —— 绿蒙版 + 模糊 + 过暗，三重过滤

关键：清晰度必须在**统一尺度**上比较，否则小图天然吃亏。
把所有图缩放到短边 480 再算拉普拉斯方差。

输出 clean_final.csv，含每张图的判定与理由。
"""
import cv2
import numpy as np
import glob
import os
import csv
import collections

ROOT = r'D:\ADAS_Datasets\06_车道线_lane\LaneMarking8K\Night Vision Dataset'
OUT = r'D:\ADAS_Datasets\06_车道线_lane\clean_final.csv'

NORM_SHORT = 480      # 统一缩放到短边，再算清晰度
BLUR_THR = 55         # 归一化后拉普拉斯方差下限
DARK_THR = 55         # 平均亮度下限
GREEN_THR = 0.55      # 绿蒙版像素占比上限


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def assess(im):
    h, w = im.shape[:2]
    b = im[:, :, 0].astype(np.int16)
    g = im[:, :, 1].astype(np.int16)
    r = im[:, :, 2].astype(np.int16)
    # 绿蒙版
    gf = float(((g - np.maximum(r, b)) > 25).mean())
    # 灰度
    G = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
    # 归一化尺度下的清晰度
    s = NORM_SHORT / min(h, w)
    if abs(s - 1.0) > 0.02:
        Gs = cv2.resize(G, (max(1, int(w * s)), max(1, int(h * s))),
                        interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    else:
        Gs = G
    lap = float(cv2.Laplacian(Gs, cv2.CV_64F).var())
    return gf, lap, float(G.mean()), float(G.std()), w, h


def main():
    rows = []
    for sp in sorted(os.listdir(ROOT)):
        spd = os.path.join(ROOT, sp)
        if not os.path.isdir(spd):
            continue
        for cl in sorted(os.listdir(spd)):
            cld = os.path.join(spd, cl)
            if not os.path.isdir(cld):
                continue
            fs = sorted(glob.glob(os.path.join(cld, '*.jpg')))
            bad = collections.Counter()
            for f in fs:
                im = imread_u(f)
                if im is None:
                    bad['unreadable'] += 1
                    continue
                gf, lap, br, sd, w, h = assess(im)
                reasons = []
                if gf > GREEN_THR:
                    reasons.append('green')
                if lap < BLUR_THR:
                    reasons.append('blur')
                if br < DARK_THR:
                    reasons.append('dark')
                rows.append((sp, cl, f, '%.4f' % gf, '%.1f' % lap, '%.1f' % br,
                             w, h, int(bool(reasons)), '|'.join(reasons)))
                for rr in reasons:
                    bad[rr] += 1
            print('  %-11s %-31s %5d 张  绿%-4d 糊%-4d 暗%-4d'
                  % (sp, cl, len(fs), bad['green'], bad['blur'], bad['dark']))

    with open(OUT, 'w', newline='', encoding='utf-8') as fh:
        w_ = csv.writer(fh)
        w_.writerow(['split', 'cls', 'path', 'green_frac', 'lapvar', 'bright',
                     'w', 'h', 'is_bad', 'reason'])
        w_.writerows(rows)

    tot = len(rows)
    badn = sum(r[8] for r in rows)
    print()
    print('总计 %d 张；剔除 %d (%.1f%%)；保留 %d 张'
          % (tot, badn, 100 * badn / tot, tot - badn))

    by = collections.defaultdict(lambda: [0, 0])
    for r in rows:
        by[r[1]][0] += 1
        by[r[1]][1] += r[8]
    print()
    print('%-32s %7s %7s %7s' % ('类别', '总', '剔除', '保留'))
    for k in sorted(by):
        t, b_ = by[k]
        print('%-32s %7d %7d %7d' % (k, t, b_, t - b_))
    print()
    print('清单:', OUT)


if __name__ == '__main__':
    main()
