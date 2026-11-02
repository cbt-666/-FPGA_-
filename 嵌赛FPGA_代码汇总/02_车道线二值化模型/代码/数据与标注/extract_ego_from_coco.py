#!/usr/bin/env python3
"""从 A 套 COCO 标注自动推导「自车车道左右边界」虚实标签

原理：A 套逐实例标注了每条车道线的类型（dotted=虚线 / solid=实线）。
     画面底部最靠近中心的两条线，就是自车车道的左右边界。
     -> 自动得到 (左类型, 右类型)，无需人工标注。

输出训练集：FPGA 灰度图 + (L, R) 标签
"""
import json
import os
import csv
import collections
import numpy as np
import cv2

SRC = r'D:\ADAS_Datasets\06_车道线_lane\A_实线虚线_实例分割_1133张_COCO'
OUT = r'D:\ADAS_Datasets\06_车道线_lane\_train_A'
WIN = (448, 192)
# 只看这四类；random-line(导流线) / road-sign-line / divider-line 不是车道边界
USE = {'dotted-line': 0, 'solid-line': 1}     # 0=dash 1=solid
CROP_TOP = 0.50


def fpga_gray(bgr):
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def poly_x_at(seg, y_ref, band=14):
    """多边形在高度 y_ref 处的 x（取该水平带内所有点的均值）

    引擎盖占了画面下方，车道线标注通常只延伸到 ~0.64H，
    所以不能要求「延伸到底部」，要在统一参考高度上比较左右。
    """
    xs = []
    for poly in seg:
        a = np.array(poly, np.float64).reshape(-1, 2)
        if len(a) == 0:
            continue
        sel = np.abs(a[:, 1] - y_ref) <= band
        if sel.sum():
            xs.append(a[sel, 0])
    if not xs:
        return None
    return float(np.concatenate(xs).mean())


def main():
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    rows = []
    stat = collections.Counter()
    for split in ('train2017', 'valid2017'):
        js = {'train2017': 'instances_train2017.json',
              'valid2017': 'instances_val2017.json'}.get(split)
        if js is None or not os.path.exists(os.path.join(SRC, 'annotations', js)):
            # 可能是 val2017
            js = 'instances_val2017.json'
        p = os.path.join(SRC, 'annotations', js)
        if not os.path.exists(p):
            continue
        d = json.load(open(p, encoding='utf-8'))
        cats = {c['id']: c['name'] for c in d['categories']}
        imgd = {i['id']: i for i in d['images']}
        per = collections.defaultdict(list)
        for a in d['annotations']:
            per[a['image_id']].append(a)
        idir = 'train2017' if 'train' in js else 'valid2017'
        if not os.path.isdir(os.path.join(SRC, idir)):
            idir = 'train2017'

        for iid, anns in per.items():
            info = imgd.get(iid)
            if info is None:
                continue
            ip = os.path.join(SRC, idir, info['file_name'])
            if not os.path.exists(ip):
                continue
            im = cv2.imdecode(np.fromfile(ip, np.uint8), cv2.IMREAD_COLOR)
            if im is None:
                continue
            H, W = im.shape[:2]
            cx = W / 2.0
            # 车道线标注高度不一（引擎盖遮挡），自适应找一个左右都有线的参考高度：
            # 从最靠近自车的位置（y 最大）往上试，取第一个成功的
            lanes = []
            for yf in (0.92, 0.88, 0.84, 0.80, 0.76, 0.72, 0.68, 0.64, 0.60):
                y_ref = H * yf
                cur = []
                for a in anns:
                    nm = cats.get(a['category_id'])
                    if nm not in USE:
                        continue
                    x = poly_x_at(a.get('segmentation', []), y_ref)
                    if x is not None:
                        cur.append((x, USE[nm]))
                Lc = [c for c in cur if c[0] < cx]
                Rc = [c for c in cur if c[0] >= cx]
                if Lc and Rc:
                    L = max(Lc, key=lambda t: t[0])
                    R = min(Rc, key=lambda t: t[0])
                    if R[0] - L[0] >= W * 0.20:
                        lanes = [L, R]
                        break
            if len(lanes) < 2:
                stat['找不到'] += 1
                continue
            L, R = lanes
            # 裁剪 + 缩放
            crop = im[int(H * CROP_TOP):, :]
            s = 1300 / crop.shape[1]
            crop = cv2.resize(crop, (1300, int(crop.shape[0] * s)),
                              interpolation=cv2.INTER_CUBIC)
            g = fpga_gray(crop)
            g = cv2.resize(g, WIN, interpolation=cv2.INTER_AREA)
            name = 'a_%05d.png' % len(rows)
            ok, buf = cv2.imencode('.png', g)
            if not ok:
                continue
            buf.tofile(os.path.join(OUT, name))
            rows.append((name, L[1], R[1], info['file_name']))
            stat['ok'] += 1

    with open(os.path.join(OUT, 'labels.csv'), 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['img', 'L', 'R', 'src'])
        for n, l, r, s in rows:
            w.writerow([n, 'dash' if l == 0 else 'solid',
                        'dash' if r == 0 else 'solid', s])

    print('成功提取 %d 张' % len(rows))
    print('跳过原因:', dict(stat))
    c = collections.Counter((l, r) for _, l, r, _ in rows)
    print()
    print('标签分布:')
    for k, v in c.most_common():
        print('   L=%-6s R=%-6s %4d' % ('dash' if k[0] == 0 else 'solid',
                                        'dash' if k[1] == 0 else 'solid', v))
    print()
    print('输出:', OUT)


if __name__ == '__main__':
    main()
