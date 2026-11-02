# -*- coding: utf-8 -*-
"""构建【中国红绿灯 · 灯箱标注池】

输入域 = 队友 FPGA 输出域: 红绿保留灰度化（中国校准版 rg_gray_cn）
标注物 = 灯箱（三个灯位合成的大框），色标 = red / green / off

来源:
  ① 官方 8 视频 (1920x1080, 深圳横排)  —— 与比赛同域，最高价值
  ② S2TLD (中国, 竖排)                  —— 免费规模，补竖排

划分: 【按整个视频】隔离（官方）；S2TLD 用其自带 train/val
产物: D:\\ADAS_Datasets\\36_中国灯箱池\\
        images_png/   灰度化后的 PNG (无损)
        _prefill.json {file: [{cls,x1,y1,x2,y2}, ...]}
        _pool_meta.json
"""
import os, sys, json, glob, collections, shutil
import numpy as np, cv2

sys.path.insert(0, r"D:\ADAS_Datasets\_code")
from rg_gray_cn import (rg_gray, lamp_blobs, merge_row,
                        expand_to_housing)

VID = r"C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材"
S2 = r"D:\ADAS_Datasets\08_中国红绿灯\S2TLD_v13"
DST = r"D:\ADAS_Datasets\36_中国灯箱池"
IMG = os.path.join(DST, "images_png")

VAL_VIDS = {"video_20260929_113452.mp4", "video_20260929_113539.mp4"}
# ⚠️ 111440 = 纯导航屏视频(无真实红绿灯) -> 排除
EXCLUDE_VIDS = {"video_20260929_111440.mp4"}
VAL_STRIDE = 4          # val 视频抽稀倍率
OFFICIAL_STEP = 45      # 官方视频采样步长 (0.66s/帧)
S2_CAP = {"train": 170, "val": 65}
MAX_BOX_W = 0.16        # 灯箱宽度上限(占全图宽)
MAX_BOX_H = 0.13        # 灯箱高度上限
MAX_BOX_AREA = 0.006    # 灯箱面积上限

for d in (DST, IMG):
    os.makedirs(d, exist_ok=True)


def rd(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def wr_png(p, im):
    cv2.imencode(".png", im)[1].tofile(p)


def _iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter)
    return inter / ua if ua > 0 else 0


def prefill_of(gray, W, H):
    """灯珠 -> 灯箱框（每簇一个框）"""
    blobs = lamp_blobs(gray)
    if not blobs:
        return []
    merged = merge_row(blobs)
    used = set()
    cands = []
    for m in merged:
        if m[5] >= 2:                      # 多灯同时亮 -> 已是灯箱
            cands.append((m[0], m[1], m[2], m[3], m[4], m[5]))
            for b in blobs:
                if (b[0] >= m[0] - 2 and b[2] <= m[2] + 2
                        and b[1] >= m[1] - 2 and b[3] <= m[3] + 2):
                    used.add((b[0], b[1], b[2], b[3]))
    singles = [b for b in blobs if (b[0], b[1], b[2], b[3]) not in used]
    for e in expand_to_housing(gray, singles, orient="h"):
        cands.append((e[0], e[1], e[2], e[3], e[4], e[5]))

    cands.sort(key=lambda b: -((b[2]-b[0])*(b[3]-b[1])))
    keep = []
    for c in cands:
        if all(_iou(c, k) < 0.40 for k in keep):
            keep.append(c)

    out = []
    for x1, y1, x2, y2, col, nb in keep:
        w, h = x2 - x1, y2 - y1
        if y1 > H * 0.60:                       # 红绿灯必然在上部
            continue
        if w > W * MAX_BOX_W or h > H * MAX_BOX_H:
            continue                            # 排除公交/路牌等大块
        if (w * h) > (W * H) * MAX_BOX_AREA:
            continue
        if w < 14 or h < 12:
            continue
        out.append({"cls": col, "x1": float(x1), "y1": float(y1),
                    "x2": float(x2), "y2": float(y2), "n_lamp": int(nb)})
    return out


prefill, meta = {}, {}
cnt = collections.Counter()

# ---------------- ① 官方视频 ----------------
print("=" * 68)
print("① 官方视频")
print("=" * 68)
for v in sorted(glob.glob(os.path.join(VID, "*.mp4"))):
    name = os.path.basename(v)
    if name in EXCLUDE_VIDS:
        print(f"  {name[-12:]:<14} 跳过(非红绿灯视频)")
        continue
    split = "val" if name in VAL_VIDS else "train"
    step = OFFICIAL_STEP * (VAL_STRIDE if split == "val" else 1)
    cap = cv2.VideoCapture(v)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tag = name.replace("video_20260929_", "").replace(".mp4", "")
    got = 0
    for fi in range(0, n, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, f = cap.read()
        if not ok:
            continue
        g = rg_gray(f)
        H, W = g.shape[:2]
        pf = prefill_of(g, W, H)
        if not pf:
            continue
        fn = f"off_{tag}_{fi:06d}.png"
        wr_png(os.path.join(IMG, fn), g)
        prefill[fn] = pf
        cnt[split] += 1
        got += 1
    cap.release()
    meta[f"official/{name}"] = {"split": split, "step": step, "n": got}
    print(f"  {name[-12:]:<14} {split:<5} step={step:<4} {got:4d} 帧")

# ---------------- ② S2TLD ----------------
print()
print("=" * 68)
print("② S2TLD (中国竖排)")
print("=" * 68)
for split, capn in S2_CAP.items():
    src = os.path.join(S2, split, "images")
    files = sorted(glob.glob(os.path.join(src, "*.jpg")))
    if not files:
        print(f"  {split}: 找不到图 {src}")
        continue
    stride = max(1, len(files) // capn)
    got = 0
    for p in files[::stride][:capn]:
        f = rd(p)
        if f is None:
            continue
        g = rg_gray(f)
        H, W = g.shape[:2]
        pf = prefill_of(g, W, H)
        if not pf:
            continue
        fn = f"s2_{split}_{os.path.splitext(os.path.basename(p))[0]}.png"
        wr_png(os.path.join(IMG, fn), g)
        prefill[fn] = pf
        cnt[split] += 1
        got += 1
    meta[f"s2tld/{split}"] = {"split": split, "stride": stride, "n": got}
    print(f"  S2TLD {split:<5} 源{len(files):5d}张 stride={stride:<3} 取 {got:4d} 帧")

# ---------------- 落盘 ----------------
json.dump(prefill, open(os.path.join(DST, "_prefill.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
json.dump({"val_videos": sorted(VAL_VIDS), "meta": meta, "counts": dict(cnt),
           "transform": "rg_gray_cn (G>=80 & G-R>=55 & G>B | R>=96 & R*5>G*8 & R+16>=B)",
           "classes": ["red", "green", "off"]},
          open(os.path.join(DST, "_pool_meta.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)

nb = sum(len(v) for v in prefill.values())
print()
print("=" * 68)
print(f"总帧数: {sum(cnt.values())}   (train {cnt['train']} / val {cnt['val']})")
print(f"预填框: {nb}")
print("类别:", dict(collections.Counter(b["cls"] for v in prefill.values() for b in v)))
print("目录  :", IMG)
sz = sum(os.path.getsize(os.path.join(IMG, f)) for f in os.listdir(IMG))
print(f"体积  : {sz/1024/1024:.0f} MB")
