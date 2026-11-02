"""分别评估彩色 / 灰度输入, 以及各视频来源的表现."""
import os
import json
import cv2
import numpy as np
from ultralytics import YOLO

W = r'D:\ADAS_Datasets\_runs\housing_n1280\weights\best.pt'
VAL = r'D:\ADAS_Datasets\42_housing_1c\val'
CONF, IOU = 0.25, 0.45


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def yolo_line(lb):
    out = []
    if not os.path.isfile(lb):
        return out
    for line in open(lb):
        q = line.split()
        if len(q) < 5:
            continue
        out.append([float(x) for x in q[1:5]])
    return out


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1, bx2, by2 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


def match(preds, gts, thr=0.5):
    """简单贪心匹配, 返回 tp/fp/fn"""
    used = set()
    tp = 0
    for p in preds:
        best, bi = 0.0, -1
        for i, g in enumerate(gts):
            if i in used:
                continue
            v = iou(p, g)
            if v > best:
                best, bi = v, i
        if best >= thr and bi >= 0:
            used.add(bi)
            tp += 1
    return tp, len(preds) - tp, len(gts) - tp


def main():
    model = YOLO(W)
    files = sorted(os.listdir(os.path.join(VAL, 'images')))
    groups = {}
    for f in files:
        kind = 'gray' if f.endswith('_g.png') else 'color'
        groups.setdefault(kind, []).append(f)

    print('%-7s %5s %6s %6s %6s %6s %6s' % ('输入', '图数', 'GT框', 'TP', 'FP', 'FN', '召回'))
    for kind, fs in sorted(groups.items()):
        TP = FP = FN = NG = 0
        for f in fs:
            img = imread(os.path.join(VAL, 'images', f))
            H, Wd = img.shape[:2]
            gts = yolo_line(os.path.join(VAL, 'labels', os.path.splitext(f)[0] + '.txt'))
            r = model.predict(img, imgsz=1280, conf=CONF, iou=IOU, verbose=False, rect=True)[0]
            preds = []
            if r.boxes is not None:
                for b in r.boxes:
                    x1, y1, x2, y2 = [float(q) for q in b.xyxy[0]]
                    preds.append([(x1 + x2) / 2 / Wd, (y1 + y2) / 2 / H,
                                  (x2 - x1) / Wd, (y2 - y1) / H])
            tp, fp, fn = match(preds, gts)
            TP += tp; FP += fp; FN += fn; NG += len(gts)
        rec = TP / max(TP + FN, 1)
        print('%-7s %5d %6d %6d %6d %6d %5.1f%%' % (kind, len(fs), NG, TP, FP, FN, 100 * rec))

    # 各视频来源
    print('\n按来源:')
    print('%-10s %5s %6s %6s %6s %6s %6s' % ('来源', '图数', 'GT框', 'TP', 'FP', 'FN', '召回'))
    bysrc = {}
    for f in files:
        src = f.split('_')[0] if not f.startswith('p') else f.split('_')[1]
        bysrc.setdefault(src, []).append(f)
    for src, fs in sorted(bysrc.items()):
        TP = FP = FN = NG = 0
        for f in fs:
            img = imread(os.path.join(VAL, 'images', f))
            H, Wd = img.shape[:2]
            gts = yolo_line(os.path.join(VAL, 'labels', os.path.splitext(f)[0] + '.txt'))
            r = model.predict(img, imgsz=1280, conf=CONF, iou=IOU, verbose=False, rect=True)[0]
            preds = []
            if r.boxes is not None:
                for b in r.boxes:
                    x1, y1, x2, y2 = [float(q) for q in b.xyxy[0]]
                    preds.append([(x1 + x2) / 2 / Wd, (y1 + y2) / 2 / H,
                                  (x2 - x1) / Wd, (y2 - y1) / H])
            tp, fp, fn = match(preds, gts)
            TP += tp; FP += fp; FN += fn; NG += len(gts)
        print('%-10s %5d %6d %6d %6d %6d %5.1f%%'
              % (src, len(fs), NG, TP, FP, FN, 100 * TP / max(TP + FN, 1)))


if __name__ == '__main__':
    main()
