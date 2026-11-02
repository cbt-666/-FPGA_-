#!/usr/bin/env python3
"""用用户标注的官方帧做同域微调数据（自训练 + 标签校正）

难点：官方帧只有**图像级**标签（这条边界是虚还是实），没有像素级掩码。
做法（自训练 + 人工校正）：
  1. 用当前模型预测掩码
  2. 在掩码上找出自车车道的左/右边界簇
  3. 把这两个簇的类别**按用户标签强制改正**（虚=1 / 实=2）
     —— 人工标签是这里的强监督信号
  4. 其它线簇（旁边车道的线，用户没标）保持模型预测
  5. 用这批"校正后的掩码"微调模型

风险：会继承模型的定位误差。所以评估必须用**留出的官方帧**，
并且要同时报告「微调前 vs 微调后」，否则等于自欺欺人。

用法:
  python build_official_set.py --ckpt lane_seg_gray_gray.pt --mode gray --holdout 50
"""
import os
import sys
import csv
import json
import argparse
import collections
import numpy as np
import cv2
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import train_lane_seg as T                                     # noqa: E402
from train_lane_seg import Net, fpga_gray, MEAN, STD           # noqa: E402
from lane_decision import boundary_profile, _clusters          # noqa: E402

LABELS = r'C:\Users\cbt\Desktop\嵌赛FPGA\lane_labels2.csv'
MANIFEST = r'D:\ADAS_Datasets\06_车道线_lane\_label_set3\manifest.json'
MANIFEST2 = r'D:\ADAS_Datasets\06_车道线_lane\_label_set2\manifest.json'
VIDDIR = r'C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材'
OUTDIR = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
CACHE = r'D:\ADAS_Datasets\06_车道线_lane\_seg_cache\OF'
HALF = (1024, 576)
CLS = {'dash': 1, 'solid': 2}


def nearest_cluster(mask, side):
    """在预测掩码里找自车车道该侧的边界簇，返回 (x0, x1) 或 None"""
    H, W = mask.shape
    dash, solid = boundary_profile(mask)
    both = np.maximum(dash, solid)
    cx = W // 2
    gap = max(int(W * 0.04), 8)
    lo, hi = (0, cx - gap) if side == 'left' else (cx + gap, W)
    cs = [c for c in _clusters(both, 0.03, lo, hi) if c[1] >= 2]
    if not cs:
        return None
    c = min(cs, key=lambda t: abs(t[0] - cx))
    x, w, _ = c
    return max(x - w // 2 - 3, 0), min(x + w // 2 + 4, W)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', default='lane_seg_gray_gray.pt')
    ap.add_argument('--mode', default='gray')
    ap.add_argument('--res', default='512x288')
    ap.add_argument('--holdout', type=int, default=50)
    a = ap.parse_args()
    w, h = a.res.split('x')
    T.WIN = (int(w), int(h))

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    net.load_state_dict(torch.load(os.path.join(OUTDIR, a.ckpt), map_location=dev))
    net.eval()

    # 用户标注：只取两边都是 dash/solid 的（X 的跳过）
    lab = {}
    for r in csv.DictReader(open(LABELS, encoding='utf-8')):
        L, R = (r.get('L') or '').strip(), (r.get('R') or '').strip()
        if L in CLS and R in CLS:
            lab[r['img']] = (L, R)
    print('可用标注 %d 条（跳过路口 %d 条）' % (
        len(lab), sum(1 for r in csv.DictReader(open(LABELS, encoding='utf-8'))
                      if (r.get('X') or '').strip() == '1')))

    man = {}
    for mp in (MANIFEST, MANIFEST2):
        if os.path.exists(mp):
            for m in json.load(open(mp, encoding='utf-8')):
                man.setdefault(m['img'], m)

    # 划分：按帧号排序后交错取，避免相邻帧泄漏
    keys = sorted(lab.items(), key=lambda kv: (man.get(kv[0], {}).get('vid', ''),
                                               man.get(kv[0], {}).get('frame', 0)))
    step = max(len(keys) // a.holdout, 1)
    val_keys = keys[::step][:a.holdout]
    val_names = set(k for k, _ in val_keys)
    tr_keys = [(k, v) for k, v in keys if k not in val_names]
    print('训练 %d / 留出 %d' % (len(tr_keys), len(val_keys)))
    json.dump([k for k, _ in val_keys],
              open(os.path.join(OUTDIR, 'official_val.json'), 'w'), ensure_ascii=False)

    caps = {}
    # 留出帧放到 _holdout（不在 A/B/OF 之下），保证 collect() 永远看不到它，
    # 否则模型选择会用到留出集 -> 指标虚高
    for split, items, root in (('train', tr_keys, CACHE),
                               ('valid', val_keys,
                                r'D:\ADAS_Datasets\06_车道线_lane\_seg_cache\_holdout')):
        og = os.path.join(root, split, 'gray')
        om = os.path.join(root, split, 'mask')
        os.makedirs(og, exist_ok=True)
        os.makedirs(om, exist_ok=True)
        for f in os.listdir(og):
            os.remove(os.path.join(og, f))
        for f in os.listdir(om):
            os.remove(os.path.join(om, f))

        n_ok = n_fix = 0
        for img, (L, R) in items:
            m = man.get(img)
            if m is None:
                continue
            v = m['vid']
            if v not in caps:
                caps[v] = cv2.VideoCapture(os.path.join(VIDDIR, v))
            c = caps[v]
            c.set(cv2.CAP_PROP_POS_FRAMES, m['frame'])
            ok, fr = c.read()
            if not ok:
                continue
            g0 = fpga_gray(fr)
            mid = cv2.resize(g0, HALF, interpolation=cv2.INTER_AREA)
            cv2.imencode('.png', mid)[1].tofile(os.path.join(og, img.replace('.jpg', '.png')))

            inp = T.preprocess(fr, a.mode)
            x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
            x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
            with torch.no_grad():
                pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
            # 校正左右边界的类别
            for side, want in (('left', CLS[L]), ('right', CLS[R])):
                seg = nearest_cluster(pm, side)
                if seg is None:
                    continue
                x0, x1 = seg
                # 只改"本侧、远离另侧边界"的列，避免误伤旁边的线
                if side == 'left':
                    x1 = min(x1, int(pm.shape[1] * 0.48))
                else:
                    x0 = max(x0, int(pm.shape[1] * 0.52))
                if x1 <= x0:
                    continue
                col = pm[:, x0:x1]
                col[col > 0] = want
                pm[:, x0:x1] = col
                n_fix += 1
            mm = cv2.resize(pm, T.WIN, interpolation=cv2.INTER_NEAREST)
            cv2.imencode('.png', mm.astype(np.uint8))[1].tofile(
                os.path.join(om, img.replace('.jpg', '.png')))
            n_ok += 1
        print('%s: %d 张, 校正边界 %d 处' % (split, n_ok, n_fix))
    for c in caps.values():
        c.release()
    print('输出', CACHE)


if __name__ == '__main__':
    main()
