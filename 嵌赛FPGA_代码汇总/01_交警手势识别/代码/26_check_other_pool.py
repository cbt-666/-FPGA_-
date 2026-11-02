"""抽样检查候选 other 数据源，避免混入合成图/非道路场景。"""
import os
import random

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
DS = "D:/ADAS_Datasets"
CANDIDATES = [
    ("07_官方素材_official", "官方素材(最重要·同域)"),
    ("02_行人_pedestrian", "行人"),
    ("05_红绿灯_traffic_light", "红绿灯"),
    ("06_车道线_lane", "车道线"),
    ("04_交通锥路障_cone", "交通锥"),
    ("03_斑马线_crosswalk", "斑马线(可能含合成)"),
]


def collect(d, limit=None):
    out = []
    for r, _, fs in os.walk(os.path.join(DS, d)):
        for f in fs:
            if f.lower().endswith((".jpg", ".jpeg", ".png")):
                out.append(os.path.join(r, f))
                if limit and len(out) >= limit:
                    return out
    return out


def imread_u(p):
    try:
        return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


rows = []
random.seed(0)
for d, label in CANDIDATES:
    files = collect(d)
    if not files:
        print(f"{d}: 空")
        continue
    pick = random.sample(files, min(6, len(files)))
    cells = []
    for p in pick:
        im = imread_u(p)
        if im is None:
            continue
        h = 170
        w = int(im.shape[1] * h / max(im.shape[0], 1))
        w = min(w, 300)
        cells.append(cv2.resize(im, (w, h)))
    if not cells:
        print(f"{d}: 全部读不出")
        continue
    wmax = max(c.shape[1] for c in cells)
    cells = [cv2.copyMakeBorder(c, 0, 0, 0, wmax - c.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for c in cells]
    strip = np.hstack(cells)
    head = np.zeros((26, strip.shape[1], 3), np.uint8)
    cv2.putText(head, f"{label}  n={len(files)}", (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    rows.append(np.vstack([head, strip]))
    print(f"{d}: {len(files)} 张")

wmax = max(r.shape[1] for r in rows)
rows = [cv2.copyMakeBorder(r, 0, 0, 0, wmax - r.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for r in rows]
sheet = np.vstack(rows)
out = os.path.join(ROOT, "out", "other_pool_check.jpg")
ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
open(out, "wb").write(buf.tobytes())
print("saved", out, sheet.shape)
