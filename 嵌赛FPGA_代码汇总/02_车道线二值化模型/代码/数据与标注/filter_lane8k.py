#!/usr/bin/env python3
"""扫描 LaneMarking8K，剔除绿蒙版污染图，生成干净清单

绿蒙版判定（Roboflow 增强出错产物：整图蒙了一层半透明绿）：
    frac = 像素中 (G - max(R,B) > 25) 的比例
    frac > 0.55  -> 判定为绿蒙版污染

同时记录：分辨率、亮度，便于后续分层采样。

输出: _clean_manifest.csv  (split,cls,path,green_frac,brightness,w,h)
"""
import cv2
import numpy as np
import glob
import os
import csv
import sys

ROOT = r'D:\ADAS_Datasets\06_车道线_lane\LaneMarking8K\Night Vision Dataset'
OUT = r'D:\ADAS_Datasets\06_车道线_lane\clean_manifest.csv'


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def green_frac(im):
    b = im[:, :, 0].astype(np.int16)
    g = im[:, :, 1].astype(np.int16)
    r = im[:, :, 2].astype(np.int16)
    m = g - np.maximum(r, b)
    return float((m > 25).mean()), float(m.mean())


def main():
    rows = []
    splits = sorted(os.listdir(ROOT))
    for sp in splits:
        spdir = os.path.join(ROOT, sp)
        if not os.path.isdir(spdir):
            continue
        for cl in sorted(os.listdir(spdir)):
            cldir = os.path.join(spdir, cl)
            if not os.path.isdir(cldir):
                continue
            fs = sorted(glob.glob(os.path.join(cldir, '*.jpg')))
            n_bad = 0
            for f in fs:
                im = imread_u(f)
                if im is None:
                    continue
                gf, gm = green_frac(im)
                h, w = im.shape[:2]
                bad = gf > 0.55
                if bad:
                    n_bad += 1
                rows.append((sp, cl, f, '%.4f' % gf, '%.2f' % gm,
                             '%.1f' % im.mean(), w, h, int(bad)))
            print('  %-12s %-32s %5d 张, 污染 %d (%.1f%%)'
                  % (sp, cl, len(fs), n_bad, 100 * n_bad / max(len(fs), 1)))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', newline='', encoding='utf-8') as fh:
        wtr = csv.writer(fh)
        wtr.writerow(['split', 'cls', 'path', 'green_frac', 'green_mean',
                      'brightness', 'w', 'h', 'is_bad'])
        wtr.writerows(rows)

    import collections
    tot = len(rows)
    bad = sum(r[8] for r in rows)
    print()
    print('总计 %d 张，污染 %d 张 (%.1f%%)，干净 %d 张'
          % (tot, bad, 100 * bad / tot, tot - bad))
    print('清单:', OUT)

    # 按类别汇总
    by = collections.defaultdict(lambda: [0, 0])
    for r in rows:
        by[(r[0], r[1])][0] += 1
        by[(r[0], r[1])][1] += r[8]
    print()
    print('%-12s %-32s %7s %7s %7s' % ('split', 'cls', '总', '污染', '干净'))
    for k in sorted(by):
        t, b_ = by[k]
        print('%-12s %-32s %7d %7d %7d' % (k[0], k[1], t, b_, t - b_))


if __name__ == '__main__':
    main()
