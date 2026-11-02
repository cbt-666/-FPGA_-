# -*- coding: utf-8 -*-
"""建池: TrafficRules (中国场景, 横排灯箱) -> 红绿灰度化 + 灯箱框反推

⚠️ 与 36 池的区别: 这里的框来自【TrafficRules 自带的真实人工标注】(8类),
   不是 CV 猜的, 所以可靠得多。但仍然:
     · 放进 _prefill.json 只作【参考】(按 A 键显示虚线)
     · 工具的 rec() 已改为【不再自动灌框】, 所以不会污染用户标注

TrafficRules 8 类 -> 灯箱的 3 类:
   F0/F1=圆形 红/绿   L0/L1=左转 红/绿
   S0/S1=直行 红/绿   R0/R1=右转 红/绿
   => 0/2/4/6 -> red     1/3/5/7 -> green

用法: python build_tr_pool.py [--split train|val]
"""
import os, sys, json, glob, collections, argparse
import numpy as np, cv2

sys.path.insert(0, r"D:\ADAS_Datasets\_code")
from rg_gray_cn import rg_gray, expand_to_housing, merge_row

TR = r"D:\ADAS_Datasets\05_红绿灯_traffic_light\TrafficRules"
CLS_MAP = {0: "red", 1: "green", 2: "red", 3: "green",
           4: "red", 5: "green", 6: "red", 7: "green"}
NAMES = {0: "F0", 1: "F1", 2: "L0", 3: "L1", 4: "S0", 5: "S1", 6: "R0", 7: "R1"}


def housing_from_gt(boxes, W, H):
    """boxes: [(x1,y1,x2,y2,cls)] 像素系 -> 灯箱 [(x1,y1,x2,y2,'red'|'green')]
    一个标注图里可能只标了 1~3 颗灯。策略:
      · 多颗同 y 的灯 -> 直接合并
      · 单颗灯 -> 按三灯位几何扩成灯箱
    """
    if not boxes:
        return []
    # 先按 y 相邻 + x 相近 合并
    bs = sorted(boxes, key=lambda b: b[0])
    used = [False] * len(bs)
    groups = []
    for i in range(len(bs)):
        if used[i]:
            continue
        g = [bs[i]]; used[i] = True; ch = True
        while ch:
            ch = False
            gx1 = min(b[0] for b in g); gx2 = max(b[2] for b in g)
            gy1 = min(b[1] for b in g); gy2 = max(b[3] for b in g)
            gcy = (gy1 + gy2) / 2; gh = gy2 - gy1
            for j in range(len(bs)):
                if used[j]:
                    continue
                b = bs[j]
                bcy = (b[1] + b[3]) / 2; bh = b[3] - b[1]
                if abs(gcy - bcy) > max(gh, bh) * 0.65:
                    continue
                gapx = (b[0] - gx2) if b[0] > gx2 else (gx1 - b[2] if b[2] < gx1 else 0)
                if gapx > max(gh, bh) * 1.3:
                    continue
                g.append(b); used[j] = True; ch = True
        groups.append(g)

    out = []
    for g in groups:
        x1 = min(b[0] for b in g); y1 = min(b[1] for b in g)
        x2 = max(b[2] for b in g); y2 = max(b[3] for b in g)
        # 主色 = 面积最大的那颗 (亮着的)
        main = max(g, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        col = CLS_MAP[main[4]]
        if len(g) == 1:
            # 单颗 -> 按三灯位几何扩  (夹缝)
            d = float(y2 - y1)
            w, h = 3.05 * d, 1.15 * d
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            x1, x2 = cx - w / 2, cx + w / 2
            y1, y2 = cy - h / 2, cy + h / 2
        else:
            # 多颗 -> 已有横向延展, 上下略放宽含壳体
            pad = (y2 - y1) * 0.10
            y1 -= pad; y2 += pad
            x1 -= (y2 - y1) * 0.12; x2 += (y2 - y1) * 0.12
        x1 = max(0, x1); y1 = max(0, y1)
        x2 = min(W, x2); y2 = min(H, y2)
        if x2 - x1 < 12 or y2 - y1 < 10:
            continue
        out.append((x1, y1, x2, y2, col))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", default="train", choices=["train", "val"])
    a = ap.parse_args()
    split = a.split
    DST = (chr(68)+chr(58)+chr(92)+"ADAS_Datasets"+chr(92)+"37_TR_housing_val"
           if split == "val" else
           chr(68)+chr(58)+chr(92)+"ADAS_Datasets"+chr(92)+"37_TR_housing")
    IMG = os.path.join(DST, "images_png")
    os.makedirs(IMG, exist_ok=True)

    files = sorted(glob.glob(os.path.join(TR, split, "labels", "*.txt")))
    print(f"TrafficRules {split}: {len(files)} 图")
    ref, meta, stat = {}, {}, collections.Counter()
    for p in files:
        ip = p.replace(os.sep + "labels" + os.sep, os.sep + "images" + os.sep)\
              .replace(".txt", ".jpg")
        im = cv2.imdecode(np.fromfile(ip, np.uint8), cv2.IMREAD_COLOR)
        if im is None:
            continue
        H, W = im.shape[:2]
        bx, names = [], []
        for ln in open(p):
            t = ln.split()
            if len(t) < 5:
                continue
            c = int(t[0]); cx, cy, w, h = [float(v) for v in t[1:5]]
            bx.append(((cx - w / 2) * W, (cy - h / 2) * H,
                       (cx + w / 2) * W, (cy + h / 2) * H, c))
            names.append(NAMES[c])
        hs = housing_from_gt(bx, W, H)
        # 灰度化
        g = rg_gray(im)
        fn = f"tr_{os.path.splitext(os.path.basename(p))[0]}.png"
        cv2.imencode(".png", g)[1].tofile(os.path.join(IMG, fn))
        ref[fn] = [{"cls": h[4], "x1": float(h[0]), "y1": float(h[1]),
                    "x2": float(h[2]), "y2": float(h[3]), "src": "gt"}
                   for h in hs]
        meta[fn] = {"gt": names, "n_lamp": len(bx), "n_housing": len(hs)}
        stat[f"{len(bx)}灯->{len(hs)}箱"] += 1
        for h in hs:
            stat["cls_" + h[4]] += 1

    json.dump(ref, open(os.path.join(DST, "_prefill.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    json.dump({"source": "TrafficRules", "split": split, "n": len(ref),
               "classes": ["red", "green", "off"],
               "note": "框由 TrafficRules 真实人工标注反推, 只作参考(A键显示)"},
              open(os.path.join(DST, "_pool_meta.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(f"  产出 {len(ref)} 图, 参考框 {sum(len(v) for v in ref.values())}")
    print("  统计:", {k: v for k, v in stat.items() if not k.startswith("cls_")})
    print("  类别:", {k[4:]: v for k, v in stat.items() if k.startswith("cls_")})
    print("  目录:", DST)


if __name__ == "__main__":
    main()
