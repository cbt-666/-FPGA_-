#!/usr/bin/env python3
"""在留出的官方帧上评估车道模型 —— 判准「微调前 vs 微调后」

用 build_official_set.py 存下的 official_val.json（不会进入训练），
配合用户在 lane_labels2.csv 里的标注。

用法:
  python eval_official_lane.py --ckpt lane_seg_gray_gray.pt
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
from train_lane_seg import Net, MEAN, STD                      # noqa: E402
from lane_decision import decode                               # noqa: E402

LABELS = r'C:\Users\cbt\Desktop\嵌赛FPGA\lane_labels2.csv'
VAL = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg\official_val.json'
MANIFEST = r'D:\ADAS_Datasets\06_车道线_lane\_label_set3\manifest.json'
MANIFEST2 = r'D:\ADAS_Datasets\06_车道线_lane\_label_set2\manifest.json'
VIDDIR = r'C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材'
OUTDIR = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
CLS = ('dash', 'solid')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', default='lane_seg_gray_gray.pt')
    ap.add_argument('--mode', default='gray')
    ap.add_argument('--res', default='512x288')
    ap.add_argument('--th', type=int, default=None, help='二值化阈值(默认 BIN_TH)')
    a = ap.parse_args()
    if a.th is not None:
        T.BIN_TH = a.th
    w, h = a.res.split('x')
    T.WIN = (int(w), int(h))

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    net.load_state_dict(torch.load(os.path.join(OUTDIR, a.ckpt), map_location=dev))
    net.eval()

    val = set(json.load(open(VAL, encoding='utf-8')))
    lab = {}
    for r in csv.DictReader(open(LABELS, encoding='utf-8')):
        L, R = (r.get('L') or '').strip(), (r.get('R') or '').strip()
        if L in CLS and R in CLS and r['img'] in val:
            lab[r['img']] = (L, R)
    man = {}
    for mp in (MANIFEST, MANIFEST2):
        if os.path.exists(mp):
            for m in json.load(open(mp, encoding='utf-8')):
                man.setdefault(m['img'], m)
    print('留出官方帧 %d 张（有有效标注 %d）' % (len(val), len(lab)))

    caps = {}
    ok_t = ok_d = n = 0
    conf = collections.Counter()
    for img, (L, R) in lab.items():
        m = man.get(img)
        if m is None:
            continue
        v = m['vid']
        if v not in caps:
            caps[v] = cv2.VideoCapture(os.path.join(VIDDIR, v))
        c = caps[v]
        c.set(cv2.CAP_PROP_POS_FRAMES, m['frame'])
        okf, fr = c.read()
        if not okf:
            continue
        inp = T.preprocess(fr, a.mode)
        x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
        x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
        with torch.no_grad():
            pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
        r = decode(pm)
        for side, gt in (('left', L), ('right', R)):
            n += 1
            conf[(gt, r[side])] += 1
            if r[side] == gt:
                ok_t += 1
            if r['can_change_' + side] == (gt == 'dash'):
                ok_d += 1
    for c in caps.values():
        c.release()

    if n == 0:
        print('没有可用样本')
        return
    nd = sum(1 for k in lab.values() for g in k if g == 'dash')
    print()
    print('=== %s (mode=%s)  留出官方帧 n=%d ===' % (a.ckpt, a.mode, n))
    print('边界类型准确率 %.3f' % (ok_t / n))
    print('变道许可准确率 %.3f   (真值中虚线占比 %.3f, 全判可变道基线 %.3f)'
          % (ok_d / n, nd / n, nd / n))
    print()
    for tv in CLS:
        sub = {p: c for (t, p), c in conf.items() if t == tv}
        tot = sum(sub.values())
        s = '  GT %-6s' % tv
        for p in ('dash', 'solid', 'none'):
            s += '  %s=%3d(%5.1f%%)' % (p, sub.get(p, 0),
                                        100.0 * sub.get(p, 0) / max(tot, 1))
        print(s)


if __name__ == '__main__':
    main()
