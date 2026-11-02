# -*- coding: utf-8 -*-
"""构建【灯箱标注池】v2 —— 预填 = v13 ∪ 经典CV (取并集, 保召回)

采样策略（按视频特点）:
  · 箭头密集视频 (112454红箭头 / 113452红箭头 / 113421绿箭头 / 112507绿箭头) -> STEP=8
  · 其余 -> STEP=20
  · 视频级隔离: val = 113452 + 113539

产物: D:\\ADAS_Datasets\\35_灯箱标注池\\
"""
import os, sys, json, glob, collections
import numpy as np, cv2

sys.path.insert(0, r"D:\ADAS_Datasets\_code")
from housing_cv import lamp_blobs, merge_row
from ultralytics import YOLO

VID = r"C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材"
DST = r"D:\ADAS_Datasets\35_灯箱标注池"
IMG = os.path.join(DST, "images")
os.makedirs(IMG, exist_ok=True)

VAL_VIDS = {"video_20260929_113452.mp4", "video_20260929_113539.mp4"}
ARROW_VIDS = {"video_20260929_112454.mp4", "video_20260929_113452.mp4",
              "video_20260929_113421.mp4", "video_20260929_112507.mp4"}
MAX_PER_VID = 60
MIN_GAP = 40
CLS_OK = {"red", "green", "yellow"}


def imwrite_u(p, im):
    cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 94])[1].tofile(p)


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0


def dedupe(items, thr=0.35):
    items = sorted(items, key=lambda b: -(b[3] - b[1]) * (b[2] - b[0]))
    out = []
    for it in items:
        if all(iou(it, o) < thr for o in out):
            out.append(it)
    return out


m13 = YOLO(r"D:\ADAS_Datasets\_runs\s2_v13\weights\best.pt")
prefill, meta = {}, {}
tot = collections.Counter()

for v in sorted(glob.glob(os.path.join(VID, "*.mp4"))):
    name = os.path.basename(v)
    split = "val" if name in VAL_VIDS else "train"
    step = 8 if name in ARROW_VIDS else 20
    cap = cv2.VideoCapture(v)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    picked, last = [], -10 ** 9
    for fi in range(0, n, step):
        if len(picked) >= MAX_PER_VID:
            break
        if fi - last < MIN_GAP:
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, f = cap.read()
        if not ok:
            continue
        H, W = f.shape[:2]
        boxes = []

        # --- 源1: 经典CV ---
        for x1, y1, x2, y2, col, nb, w, h in merge_row(lamp_blobs(f)):
            if col in CLS_OK and y1 < H * 0.55 and w >= 12 and h >= 10:
                boxes.append((float(x1), float(y1), float(x2), float(y2), col, "cv"))
        # --- 源2: v13 (低阈值保召回) ---
        try:
            r = m13.predict(f, imgsz=1280, conf=0.20, iou=0.45,
                            verbose=False, device=0)[0]
            cvs = None
            if r.boxes is not None:
                for b, c in zip(r.boxes.xyxy.cpu().numpy(),
                                r.boxes.cls.cpu().numpy()):
                    nm = m13.names[int(c)]
                    if nm not in CLS_OK:
                        continue
                    x1, y1, x2, y2 = [float(t) for t in b]
                    if y1 >= H * 0.55:
                        continue
                    boxes.append((x1, y1, x2, y2, nm, "v13"))
        except Exception:
            pass

        boxes = dedupe(boxes)
        if not boxes:
            continue
        tag = name.replace("video_20260929_", "").replace(".mp4", "")
        fn = f"{tag}_{fi:06d}.jpg"
        imwrite_u(os.path.join(IMG, fn), f)
        prefill[fn] = [{"cls": b[4], "x1": b[0], "y1": b[1],
                        "x2": b[2], "y2": b[3], "src": b[5]} for b in boxes]
        picked.append(fi); last = fi
    cap.release()
    meta[name] = {"split": split, "step": step, "frames": picked, "n": len(picked)}
    tot[split] += len(picked)
    print(f"{name}  {split}  step={step}  {len(picked)} 帧")

json.dump(prefill, open(os.path.join(DST, "_prefill.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
json.dump({"val_videos": sorted(VAL_VIDS), "meta": meta, "total": dict(tot),
           "note": "预填=v13∪经典CV, 需人工增删修正"},
          open(os.path.join(DST, "_pool_meta.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)

n_box = sum(len(v) for v in prefill.values())
print(f"\n总帧数: {sum(tot.values())}  (train {tot['train']} / val {tot['val']})")
print(f"预填框: {n_box}")
print("类别:", dict(collections.Counter(b["cls"] for v in prefill.values() for b in v)))
print("来源:", dict(collections.Counter(b["src"] for v in prefill.values() for b in v)))
