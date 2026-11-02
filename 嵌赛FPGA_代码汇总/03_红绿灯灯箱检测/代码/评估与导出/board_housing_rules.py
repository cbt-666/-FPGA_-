"""新模型在板端真实帧上的效果 + 过滤规则（长宽比/数字框/鬼影/cy）实测.

规则来自 桌面\\灯箱误检过滤阈值建议.md，逐条照抄，不改阈值。
"""
import os
import sys
import json
import glob

import cv2
import numpy as np
from ultralytics import YOLO

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

_HERE = os.path.dirname(os.path.abspath(__file__))
_LOCAL_PT = os.path.join(_HERE, 'housing_best.pt')
NEW = _LOCAL_PT if os.path.isfile(_LOCAL_PT) else \
    r'D:\ADAS_Datasets\_runs\housing_rg_n1280\weights\best.pt'
OLD = r'D:\ADAS_Datasets\_runs\housing_n1280b\weights\best.pt'
BOARD = r'D:\AI\CLaude_workplace\临时文件\traffic_captures\traffic_captures'
CONF = 0.25


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def box_features(img, x1, y1, x2, y2):
    H, W = img.shape[:2]
    a, b = max(0, int(x1)), max(0, int(y1))
    c, d = min(W, int(x2)), min(H, int(y2))
    if c <= a or d <= b:
        return None
    z = img[b:d, a:c]
    B = z[:, :, 0].astype(np.int16)
    G = z[:, :, 1].astype(np.int16)
    R = z[:, :, 2].astype(np.int16)
    mR = ((R > 150) & (G < 110) & (B < 110))
    mG = ((G > 150) & (R < 110) & (B < 110))
    nR, nG = int(mR.sum()), int(mG.sum())
    m = (mR if nR >= nG else mG).astype(np.uint8)

    bw, bh = c - a, d - b
    f = dict(cf=(nR + nG) / float(bw * bh),
             ar=max(bw, bh) / float(max(1, min(bw, bh))),
             cy=(y1 + y2) / 2.0 / H,
             ncolor=nR + nG, hull_fill=0.0)

    if f['ncolor'] >= 20:
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        big = int(stats[i, cv2.CC_STAT_AREA])
        comp = (lab == i).astype(np.uint8)
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            cn = max(cnts, key=cv2.contourArea)
            ha = cv2.contourArea(cv2.convexHull(cn))
            if ha > 0:
                f['hull_fill'] = big / ha
    return f


def keep_box(f):
    if not (1.3 <= f['ar'] <= 3.5):
        return False, 'AR'
    if f['cy'] > 0.62:
        return False, 'cy'
    if f['cf'] > 0.25 and f['hull_fill'] < 0.80 and f['ncolor'] >= 200:
        return False, 'digit'
    if 0.03 < f['cf'] < 0.20 and f['hull_fill'] < 0.60:
        return False, 'ghost'
    return True, 'ok'


def main():
    new = YOLO(NEW)
    old = YOLO(OLD)

    rows = []
    for rd in sorted(glob.glob(os.path.join(BOARD, '*/*'))):
        jp = os.path.join(rd, 'result.json')
        ip = os.path.join(rd, 'input.png')
        if not (os.path.exists(jp) and os.path.exists(ip)):
            continue
        d = json.load(open(jp, encoding='utf-8'))
        if d.get('gray_is_binary'):
            continue
        img = imread(ip)
        if img is None:
            continue
        rows.append((d['capture_seq'], img))

    print('板端红绿帧 %d 张\n' % len(rows))

    for tag, model in (('旧(混灰度)', old), ('新(纯红绿)', new)):
        raw = kept = 0
        kills = {}
        for seq, img in rows:
            r = model.predict(img, imgsz=1280, conf=CONF, iou=0.45,
                              verbose=False, rect=True)[0]
            if r.boxes is None:
                continue
            for b in r.boxes:
                x1, y1, x2, y2 = [float(q) for q in b.xyxy[0]]
                f = box_features(img, x1, y1, x2, y2)
                if f is None:
                    continue
                raw += 1
                ok, why = keep_box(f)
                if ok:
                    kept += 1
                else:
                    kills[why] = kills.get(why, 0) + 1
        print('%-12s 原始框 %3d  ->  规则后 %3d   拦截明细 %s'
              % (tag, raw, kept, kills))

    # ---- 新模型被拦掉的框逐条列出 ----
    print('\n新模型被拦掉的框（逐条）:')
    for seq, img in rows:
        r = new.predict(img, imgsz=1280, conf=CONF, iou=0.45,
                        verbose=False, rect=True)[0]
        if r.boxes is None:
            continue
        for b in r.boxes:
            x1, y1, x2, y2 = [float(q) for q in b.xyxy[0]]
            f = box_features(img, x1, y1, x2, y2)
            if f is None:
                continue
            ok, why = keep_box(f)
            if not ok:
                print('  seq%-3d conf %.2f  被拦:%-6s AR %.2f  CF %.3f  hull %.2f  '
                      '色像素 %4d  cy %.2f  框 %dx%d'
                      % (seq, float(b.conf[0]), why, f['ar'], f['cf'],
                         f['hull_fill'], f['ncolor'], f['cy'],
                         int(x2 - x1), int(y2 - y1)))


if __name__ == '__main__':
    main()
