# -*- coding: utf-8 -*-
"""构建【灯箱标注池】—— 从官方 8 个视频挑帧，自动预填灯箱框。

规则:
  · 视频级隔离: val = 113452 + 113539 (整段留出), 其余为 train
  · 每视频每 15 帧(=0.5s) 采样一次, 跑预填
  · 去重: 同一视频内选中帧至少间隔 60 帧
  · 只保留【上部 55%】区域的预填框 (红绿灯是悬空的)
  · 预填框面积限制, 过滤掉大块假阳

产物: D:\\ADAS_Datasets\\35_灯箱标注池\\
        images/           全尺寸帧
        _prefill.json     {file: [ {cls,x1,y1,x2,y2}, ... ]}
        _pool_meta.json   视频来源 / 划分
"""
import os, sys, json, glob, collections
import numpy as np, cv2

sys.path.insert(0, r"D:\ADAS_Datasets\_code")
from housing_cv import lamp_blobs, merge_row          # 复用判据

VID = r"C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材"
DST = r"D:\ADAS_Datasets\35_灯箱标注池"
IMG = os.path.join(DST, "images")
os.makedirs(IMG, exist_ok=True)

VAL_VIDS = {"video_20260929_113452.mp4", "video_20260929_113539.mp4"}
STEP = 15          # 采样步长
MIN_GAP = 60       # 同视频选中帧最小间隔
MAX_PER_VID = 60   # 每视频上限


def imwrite_u(p, im):
    cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 94])[1].tofile(p)


prefill, meta = {}, {}
tot = collections.Counter()
for v in sorted(glob.glob(os.path.join(VID, "*.mp4"))):
    name = os.path.basename(v)
    split = "val" if name in VAL_VIDS else "train"
    cap = cv2.VideoCapture(v)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    picked, last = [], -10 ** 9
    for fi in range(0, n, STEP):
        if len(picked) >= MAX_PER_VID:
            break
        if fi - last < MIN_GAP:
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, f = cap.read()
        if not ok:
            continue
        H, W = f.shape[:2]
        try:
            bxs = merge_row(lamp_blobs(f))
        except Exception:
            continue
        keep = []
        for x1, y1, x2, y2, col, nb, w, h in bxs:
            if y1 > H * 0.55:
                continue
            if (w * h) > (W * H) * 0.004:
                continue
            if w < 12 or h < 10:
                continue
            keep.append({"cls": col, "x1": float(x1), "y1": float(y1),
                         "x2": float(x2), "y2": float(y2), "n_lamp": int(nb)})
        if not keep:
            continue
        tag = f"{name.replace('video_20260929_','').replace('.mp4','')}_{fi:06d}"
        fn = tag + ".jpg"
        imwrite_u(os.path.join(IMG, fn), f)
        prefill[fn] = keep
        picked.append(fi); last = fi
    cap.release()
    meta[name] = {"split": split, "frames": picked, "n": len(picked)}
    tot[split] += len(picked)
    print(f"{name}  {split}  {len(picked)} 帧")

json.dump(prefill, open(os.path.join(DST, "_prefill.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
json.dump({"val_videos": sorted(VAL_VIDS), "meta": meta,
           "total": dict(tot)}, open(os.path.join(DST, "_pool_meta.json"), "w",
          encoding="utf-8"), ensure_ascii=False, indent=1)

n_box = sum(len(v) for v in prefill.values())
print(f"\n总帧数: {sum(tot.values())}  (train {tot['train']} / val {tot['val']})")
print(f"预填框总数: {n_box}")
cls = collections.Counter(b["cls"] for v in prefill.values() for b in v)
print("预填框类别:", dict(cls))
