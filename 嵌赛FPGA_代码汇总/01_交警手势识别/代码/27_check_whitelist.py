"""检查通过白名单筛选后的 other 数据池视觉质量（尤其 YRG 之后）。"""
import os
import random
import sys

import cv2
import numpy as np

sys.path.insert(0, r"D:/ADAS_Datasets/_code")
from rg_gray_fpga_exact import fpga_rg_gray

ROOT = os.path.dirname(os.path.abspath(__file__))
DS = "D:/ADAS_Datasets"

POOL = [
    ("07_官方素材_official/frames_lane", "官方帧 frames_lane", 531),
    ("07_官方素材_official/frames_tl", "官方帧 frames_tl", 8),
    ("02_行人_pedestrian/WiderPerson", "WiderPerson 街景", 9000),
    ("02_行人_pedestrian/LanePedestrian", "LanePedestrian 车道行人", 1397),
    ("04_交通锥路障_cone/TrafficCones5k", "TrafficCones5k", 542),
    ("06_车道线_lane/LaneMarking8K", "LaneMarking8K 车载", 7973),
    ("05_红绿灯_traffic_light/TrafficRules", "TrafficRules 红绿灯", None),
    ("04_交通锥路障_cone/ObstacleDetection", "ObstacleDetection", 24326),
]


def collect(d):
    out = []
    base = os.path.join(DS, d)
    for r, _, fs in os.walk(base):
        for f in fs:
            if f.lower().endswith((".jpg", ".jpeg", ".png")):
                out.append(os.path.join(r, f))
    return out


def rd(p):
    try:
        return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


rows = []
random.seed(1)
for d, label, _ in POOL:
    files = collect(d)
    if not files:
        print(f"{label}: 空/不存在")
        continue
    pick = random.sample(files, min(5, len(files)))
    cells = []
    for p in pick:
        im = rd(p)
        if im is None:
            continue
        h = 165
        w = min(int(im.shape[1] * h / max(im.shape[0], 1)), 290)
        a = cv2.resize(im, (w, h))
        yrg = cv2.resize(fpga_rg_gray(im), (w, h))
        cells.append(np.hstack([a, yrg]))
    if not cells:
        print(f"{label}: 读不出")
        continue
    wmax = max(c.shape[1] for c in cells)
    cells = [cv2.copyMakeBorder(c, 0, 0, 0, wmax - c.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for c in cells]
    strip = np.hstack(cells)
    head = np.zeros((26, strip.shape[1], 3), np.uint8)
    cv2.putText(head, f"{label}   n={len(files)}   (左RGB / 右YRG)", (6, 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 1)
    rows.append(np.vstack([head, strip]))
    print(f"{label}: {len(files)} 张")

if not rows:
    sys.exit("无可用数据")
wmax = max(r.shape[1] for r in rows)
rows = [cv2.copyMakeBorder(r, 0, 0, 0, wmax - r.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for r in rows]
sheet = np.vstack(rows)
if sheet.shape[0] > 3000:
    sc = 3000 / sheet.shape[0]
    sheet = cv2.resize(sheet, (int(sheet.shape[1] * sc), 3000))
out = os.path.join(ROOT, "out", "other_pool_whitelist.jpg")
ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
open(out, "wb").write(buf.tobytes())
print("saved", out, sheet.shape)
