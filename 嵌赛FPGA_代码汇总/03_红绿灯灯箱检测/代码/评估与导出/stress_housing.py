"""泛化压力测试: 用几种"劣化输入"验证模型是否真学到灯箱结构, 而不是靠颜色捷径.
对验证集(彩色)分别施加:
  base      原始彩色
  gray      真灰度
  dark      整体压暗 (x0.45)
  bright    整体提亮 (x1.6)
  blur      高斯模糊 (夜间/远距/失焦)
  lowres     先降到 640 宽再放大回来 (远距小目标)
  rgonly     只保留红绿 (FPGA 实际域)
输出: 每种条件下的召回/误检, 以及三档漏检
"""
import os
import sys
import cv2
import numpy as np
from ultralytics import YOLO

W = sys.argv[1] if len(sys.argv) > 1 else r'D:\ADAS_Datasets\_runs\housing_n1280\weights\best.pt'
VAL = r'D:\ADAS_Datasets\42_housing_1c\val'
CONF = 0.25
W_R, W_G, W_B = 77, 150, 29


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def to_gray(bgr):
    b, g, r = [bgr[:, :, i].astype(np.float32) for i in range(3)]
    y = np.clip((W_R * r + W_G * g + W_B * b) / 256.0, 0, 255).astype(np.uint8)
    return np.dstack([y, y, y])


def rg_only(bgr):
    """FPGA 红绿保留灰度化: 只留纯红/纯绿, 其余灰度"""
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    red = (r >= 96) & (r * 5 > g * 8) & (r + 16 >= b)
    grn = (g >= 80) & (g - r >= 55) & (g > b)
    out = np.clip((W_R * r + W_G * g + W_B * b) / 256.0, 0, 255).astype(np.uint8)
    out = np.dstack([out] * 3)
    out[red] = (0, 0, 255)
    out[grn] = (0, 255, 0)
    return out


def yolo_line(lb):
    out = []
    if not os.path.isfile(lb):
        return out
    for line in open(lb):
        q = line.split()
        if len(q) >= 5:
            out.append([float(x) for x in q[1:5]])
    return out


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1, bx2, by2 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    u = a[2] * a[3] + b[2] * b[3] - inter
    return inter / u if u > 0 else 0.0


def match(preds, gts, thr=0.5):
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
            used.add(bi); tp += 1
    return tp, len(preds) - tp, len(gts) - tp


COND = {
    'base': lambda x: x,
    'gray': to_gray,
    'rgonly': rg_only,
    'dark': lambda x: np.clip(x.astype(np.float32) * 0.45, 0, 255).astype(np.uint8),
    'bright': lambda x: np.clip(x.astype(np.float32) * 1.6, 0, 255).astype(np.uint8),
    'blur': lambda x: cv2.GaussianBlur(x, (7, 7), 2.5),
    'lowres': lambda x: cv2.resize(cv2.resize(x, (640, 360)), (1280, 720)),
}


def main():
    model = YOLO(W)
    files = [f for f in sorted(os.listdir(os.path.join(VAL, 'images')))
             if not f.endswith('_g.png')]
    print('%-8s %5s %6s %6s %6s %6s %8s' % ('条件', '图数', 'GT', 'TP', 'FP', 'FN', '召回'))
    rows = []
    for name, fn in COND.items():
        TP = FP = FN = NG = 0
        for f in files:
            img = fn(imread(os.path.join(VAL, 'images', f)))
            H, Wd = img.shape[:2]
            gts = yolo_line(os.path.join(VAL, 'labels', os.path.splitext(f)[0] + '.txt'))
            r = model.predict(img, imgsz=1280, conf=CONF, iou=0.45,
                              verbose=False, rect=True)[0]
            preds = []
            if r.boxes is not None:
                for b in r.boxes:
                    x1, y1, x2, y2 = [float(q) for q in b.xyxy[0]]
                    preds.append([(x1 + x2) / 2 / Wd, (y1 + y2) / 2 / H,
                                  (x2 - x1) / Wd, (y2 - y1) / H])
            tp, fp, fn2 = match(preds, gts)
            TP += tp; FP += fp; FN += fn2; NG += len(gts)
        rec = 100.0 * TP / max(TP + FN, 1)
        print('%-8s %5d %6d %6d %6d %6d %7.1f%%' % (name, len(files), NG, TP, FP, FN, rec))
        rows.append((name, rec, FP))

    print('\n最差条件:', min(rows, key=lambda r: r[1])[0])


if __name__ == '__main__':
    main()
