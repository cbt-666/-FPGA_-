"""新灯箱模型在板端真实红绿帧上的目视验证 + 新旧对照图."""
import os
import sys
import json
import glob

import cv2
import numpy as np
from ultralytics import YOLO

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

OLD = r'D:\ADAS_Datasets\_runs\housing_n1280b\weights\best.pt'
NEW = r'D:\ADAS_Datasets\_runs\housing_rg_n1280\weights\best.pt'
BOARD = r'D:\AI\CLaude_workplace\临时文件\traffic_captures\traffic_captures'
OUTDIR = r'D:\AI\CLaude_workplace\临时文件\demo'
CONF = 0.25


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def imwrite(p, img):
    cv2.imencode('.png', img)[1].tofile(p)


def predict(model, img):
    r = model.predict(img, imgsz=1280, conf=CONF, iou=0.45,
                      verbose=False, rect=True)[0]
    out = []
    if r.boxes is not None:
        for b in r.boxes:
            x1, y1, x2, y2 = [float(q) for q in b.xyxy[0]]
            out.append((x1, y1, x2, y2, float(b.conf[0])))
    return out


def main():
    old, new = YOLO(OLD), YOLO(NEW)
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
        rows.append((d['capture_seq'], d, img, os.path.dirname(rd)))

    print('板端红绿帧 %d 张' % len(rows))
    tot_o = tot_n = tot_log = 0
    for seq, d, img, _ in rows:
        po = predict(old, img)
        pn = predict(new, img)
        tot_o += len(po); tot_n += len(pn); tot_log += len(d['valid_boxes'])

    print('  %-22s %s' % ('板端日志 valid_boxes 合计', tot_log))
    print('  %-22s %d  (平均 %.1f/帧)' % ('旧模型 conf0.25 检出', tot_o, tot_o / len(rows)))
    print('  %-22s %d  (平均 %.1f/帧)' % ('新模型 conf0.25 检出', tot_n, tot_n / len(rows)))

    # 选出新旧差异最大的 4 帧做对照
    scored = []
    for seq, d, img, pth in rows:
        po = predict(old, img)
        pn = predict(new, img)
        scored.append((abs(len(po) - len(pn)), seq, d, img, po, pn))
    scored.sort(key=lambda x: -x[0])
    pick = scored[:4]

    tiles = []
    for _, seq, d, img, po, pn in pick:
        def draw(boxes, tag):
            v = img.copy()
            for x1, y1, x2, y2, c in boxes:
                cv2.rectangle(v, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
                cv2.putText(v, '%.2f' % c, (int(x1), max(14, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(v, '%s  %d框' % (tag, len(boxes)), (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(v, 'seq %s' % seq, (12, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            return v
        a = draw(po, 'OLD')
        b = draw(pn, 'NEW(rg)')
        sep = np.full((a.shape[0], 8, 3), 60, np.uint8)
        pair = np.hstack([a, sep, b])
        pair = cv2.resize(pair, (1600, int(pair.shape[0] * 1600 / pair.shape[1])),
                          interpolation=cv2.INTER_AREA)
        tiles.append(pair)

    if tiles:
        wmax = max(t.shape[1] for t in tiles)
        pad = [cv2.copyMakeBorder(t, 0, 0, 0, wmax - t.shape[1],
                                  cv2.BORDER_CONSTANT, value=(0, 0, 0)) for t in tiles]
        sheet = np.vstack(pad)
        out = os.path.join(OUTDIR, '10_板端_新旧对照.png')
        imwrite(out, sheet)
        print('\n已生成:', out)


if __name__ == '__main__':
    main()
