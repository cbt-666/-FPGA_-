#!/usr/bin/env python3
"""导航视频 2776 帧 自动标注 (6 类: 方向 x 位数)

标签: 按人工核定的 8 段边界, 每段固定 (方向, 位数)
框:   卡片图标框 —— 里面有箭头和距离数字, 尺寸与旧标注一致 (w~140 h~60)
       ① 模型检出的卡片框直接用
       ② 缺的时候: 用相邻好卡片做模板, 顺序跟踪白色卡片, 再按固定比例裁出
          左侧"箭头+数字"区域; 遇到模型卡片框就重新锚定
       ③ 再缺就前后帧线性插值

输出:
  D:\\ADAS_Datasets\\47_导航连续帧\\annotations_auto.json
  D:\\ADAS_Datasets\\48_导航6类\\   (YOLO)
"""
import os
import json
import shutil
import numpy as np
import cv2

B = r'D:\ADAS_Datasets'
SRC = os.path.join(B, '47_导航连续帧', 'images')
PER = os.path.join(B, '47_导航连续帧', 'per_frame.json')
ROOT = os.path.join(B, '48_导航6类')
ANN = os.path.join(B, '47_导航连续帧', 'annotations_auto.json')

SEGMENTS = [
    (0,    15,   'right', 3),
    (16,   43,   None,    None),
    (44,   1206, 'right', 3),
    (1207, 1451, 'right', 2),
    (1452, 1452, 'right', 3),
    (1453, 1842, 'left',  3),
    (1843, 2198, 'left',  2),
    (2199, 2511, None,    None),
    (2512, 2775, 'uturn', 3),
]
NAMES = ['right_3', 'right_2', 'left_3', 'left_2', 'uturn_3', 'uturn_2']
CLS_ID = {n: i for i, n in enumerate(NAMES)}

# 卡片 -> 训练框: 直接用卡片框 (与旧标注同款, 箭头+距离数字都在里面)


def seg_of(f):
    for a, b, c, d in SEGMENTS:
        if a <= f <= b:
            return c, d
    return None, None


def is_card(box):
    w, h = box[2] - box[0], box[3] - box[1]
    return h <= 115 and 90 <= w <= 240


def model_card(dets, cls):
    c = [d for d in dets if d['cls'] == cls and is_card(d['box'])]
    return max(c, key=lambda x: x['conf'])['box'] if c else None


def model_3d(dets, cls):
    c = [d for d in dets if d['cls'] == cls and (d['box'][3] - d['box'][1]) > 130]
    return max(c, key=lambda x: x['conf'])['box'] if c else None


def card_from_3d(b):
    """地图3D箭头框 -> 同一屏左上角卡片框 (导航界面固定布局)

    实测比例: 卡片左沿 = 3D左沿 - 0.12w, 卡片宽 = 1.17w, 卡片高 = 0.29h
    """
    x1, y1, x2, y2 = b
    w, h = x2 - x1, y2 - y1
    return [x1 - 0.12 * w, y1, x1 + 1.05 * w, y1 + 0.29 * h]


def derive(gaps, anchors, byf):
    """缺口帧的卡片框: 优先用地图3D箭头框按固定布局推算, 其次相邻 anchor 插值"""
    out = {}
    for a, b in gaps:
        for f in range(a, b + 1):
            cls, _ = seg_of(f)
            k = model_3d(byf[f]['dets'], cls)
            if k is not None:
                out[f] = card_from_3d(k)
    rest = {}
    for a, b in gaps:
        for f in range(a, b + 1):
            if f in out or f in anchors:
                continue
            prev = [g for g in anchors if g < f]
            nxt = [g for g in anchors if g > b]
            if prev:
                rest[f] = list(anchors[max(prev)])
            elif nxt:
                rest[f] = list(anchors[min(nxt)])
    out.update(rest)
    return out


def main():
    rows = json.load(open(PER, encoding='utf-8'))
    byf = {r['frame']: r for r in rows}
    frames = sorted(byf)

    # ---- 1) 模型卡片框 ----
    anchors = {}
    for f in frames:
        cls, _ = seg_of(f)
        if cls is None:
            continue
        b = model_card(byf[f]['dets'], cls)
        if b is not None:
            anchors[f] = [float(v) for v in b]
    print('模型卡片框 %d 帧' % len(anchors))

    # ---- 2) 找缺口 ----
    gaps = []
    for a, b, cls, nd in SEGMENTS:
        if cls is None:
            continue
        af = sorted(f for f in anchors if a <= f <= b)
        if not af:
            gaps.append([a, b])
            continue
        if af[0] > a:
            gaps.append([a, af[0] - 1])
        for p, q in zip(af, af[1:]):
            if q - p > 1:
                gaps.append([p + 1, q - 1])
        if af[-1] < b:
            gaps.append([af[-1] + 1, b])
    print('缺口 %d 段, 共 %d 帧' % (len(gaps), sum(b - a + 1 for a, b in gaps)))
    for g in gaps:
        print('   %d-%d (%d)' % (g[0], g[1], g[1] - g[0] + 1))

    # ---- 3) 跟踪补框 ----
    filled = derive(gaps, anchors, byf)
    print('推算补框 %d 帧' % len(filled))

    # ---- 4) 汇总: 卡片框 -> 子框 ----
    got = {}
    for f, b in anchors.items():
        got[f] = list(b)
    for f, b in filled.items():
        got[f] = list(b)

    # ---- 5) 剩余插值 ----
    valid = sorted(got)
    interp = 0
    for f in frames:
        cls, _ = seg_of(f)
        if cls is None or f in got:
            continue
        prev = max([g for g in valid if g < f], default=None)
        nxt = min([g for g in valid if g > f], default=None)
        if prev is None and nxt is None:
            continue
        if prev is None:
            got[f] = list(got[nxt])
        elif nxt is None:
            got[f] = list(got[prev])
        else:
            t = (f - prev) / float(nxt - prev)
            got[f] = [got[prev][i] * (1 - t) + got[nxt][i] * t for i in range(4)]
        interp += 1
    print('插值 %d 帧' % interp)

    # ---- 6) 落盘 ----
    out = []
    for f in frames:
        cls, nd = seg_of(f)
        rec = {'frame': f, 'file': 'nav_%06d.jpg' % f,
               'cls': cls, 'digits': nd, 'box': None}
        if cls is not None and f in got:
            x1, y1, x2, y2 = got[f]
            x1 = min(max(0.0, x1), 1919.0)
            y1 = min(max(0.0, y1), 1079.0)
            x2 = min(max(x1 + 8, x2), 1920.0)
            y2 = min(max(y1 + 8, y2), 1080.0)
            rec['box'] = [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]
        out.append(rec)
    json.dump(out, open(ANN, 'w', encoding='utf-8'), ensure_ascii=False)
    print('标注 ->', ANN)

    # ---- 7) YOLO ----
    tr, va = os.path.join(ROOT, 'train'), os.path.join(ROOT, 'val')
    for d in (tr, va):
        for sub in ('images', 'labels'):
            p = os.path.join(d, sub)
            if os.path.isdir(p):
                for fn in os.listdir(p):
                    os.remove(os.path.join(p, fn))
            os.makedirs(p, exist_ok=True)
    n = {'train': 0, 'val': 0, 'boxes': 0}
    for r in out:
        split = 'val' if r['frame'] % 10 == 7 else 'train'
        dst = tr if split == 'train' else va
        shutil.copyfile(os.path.join(SRC, r['file']),
                        os.path.join(dst, 'images', r['file']))
        with open(os.path.join(dst, 'labels',
                               r['file'].replace('.jpg', '.txt')), 'w') as fp:
            if r['box']:
                x1, y1, x2, y2 = r['box']
                fp.write('%d %.6f %.6f %.6f %.6f\n' % (
                    CLS_ID['%s_%d' % (r['cls'], r['digits'])],
                    (x1 + x2) / 2 / 1920, (y1 + y2) / 2 / 1080,
                    (x2 - x1) / 1920, (y2 - y1) / 1080))
                n['boxes'] += 1
        n[split] += 1
    with open(os.path.join(ROOT, 'data.yaml'), 'w', encoding='utf-8') as fp:
        fp.write('path: %s\ntrain: train/images\nval: val/images\n\n' % ROOT)
        fp.write('nc: %d\nnames:\n' % len(NAMES))
        for i, nm in enumerate(NAMES):
            fp.write('  %d: %s\n' % (i, nm))
    print('数据集 ->', ROOT)
    print('  train %d | val %d | 框 %d' % (n['train'], n['val'], n['boxes']))


if __name__ == '__main__':
    main()
