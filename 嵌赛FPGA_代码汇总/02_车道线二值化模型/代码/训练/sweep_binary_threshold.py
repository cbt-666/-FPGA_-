#!/usr/bin/env python3
"""二值化阈值扫描 —— 官方留出帧上，不同阈值 × 不同模型的变道许可准确率

背景：用户反馈队友的二值化图「路面黑、线白」，分离很干净，
且比我用 130 渲染出来的暗得多 -> 阈值可能远高于 130。
本脚本回答：是不是阈值选错了，而不是「二值化本身不行」。

三种模型都测：
  · gray_off     灰度的官方同域微调模型（最强，0.880）—— 看它对二值输入的容忍度
  · binary       二值跨域模型
  · binary_offbin 二值官方同域微调模型
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


def load_frames(res):
    w, h = res.split('x')
    T.WIN = (int(w), int(h))
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
    caps, frames = {}, []
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
        frames.append((fr, L, R))
    for c in caps.values():
        c.release()
    return frames


def run(net, frames, dev, th):
    ok_d = n = 0
    conf = collections.Counter()
    for fr, L, R in frames:
        inp = T.preprocess(fr, 'binary', th)
        x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
        x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
        with torch.no_grad():
            pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
        r = decode(pm)
        for side, gt in (('left', L), ('right', R)):
            n += 1
            p = r[side]
            conf[(gt, p)] += 1
            if r['can_change_' + side] == (gt == 'dash'):
                ok_d += 1
    return ok_d / max(n, 1), conf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--res', default='512x288')
    a = ap.parse_args()
    frames = load_frames(a.res)
    nd = sum((L == 'dash') + (R == 'dash') for _f, L, R in frames)
    base = nd / (2 * len(frames))
    print('留出官方帧 %d 张 = %d 方向 | 平凡基线 %.3f' % (len(frames), 2 * len(frames), base))
    print()
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    ths = [110, 130, 150, 170, 190, 210, 230]
    print('%-20s %s' % ('模型', '  '.join('th=%d' % t for t in ths)))
    for ck in ('lane_seg_gray_off.pt', 'lane_seg_binary.pt', 'lane_seg_binary_offbin.pt'):
        net = Net().to(dev)
        net.load_state_dict(torch.load(os.path.join(OUTDIR, ck), map_location=dev))
        net.eval()
        row = []
        for t in ths:
            d, _c = run(net, frames, dev, t)
            row.append('%.3f' % d)
        print('%-20s %s' % (ck.replace('lane_seg_', '').replace('.pt', ''), '  '.join(row)))
    print()
    print('对照：灰度输入下的最强结果 = 0.880（不走二值化）')


if __name__ == '__main__':
    main()
