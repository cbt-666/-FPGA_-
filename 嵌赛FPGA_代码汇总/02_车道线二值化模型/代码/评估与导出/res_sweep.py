#!/usr/bin/env python3
"""分辨率扫描：同一个模型在不同输入尺寸下的「变道判决」准确率

为什么做这个
------------
模型算量 ∝ 输入像素数（所有 Conv 都在全图或成比例的网格上跑）。
所以「能不能降分辨率」=「能不能直接砍掉大部分耗时」。

已知线索（实测数据.md 4.2）：
  灰度模型 512×288 → 0.880，1024×576 → 0.770，1280×720 → 0.800
  **灰度反而低分辨率最好**，与二值模型（越高越好）相反。
  说明灰度用的是粗亮度结构，不是细节纹理。

本脚本用**同一套解码器**在留出 50 帧上复测这条曲线，
给出「分辨率 vs 判决准确率 vs 相对算量」的对照表。
"""
import os
import sys
import csv
import json
import time
import argparse
import collections

import numpy as np
import cv2
import torch

sys.path.insert(0, r'D:\ADAS_Datasets\_code')
import train_lane_seg as T                                    # noqa: E402
from train_lane_seg import Net, MEAN, STD, fpga_gray          # noqa: E402
from lane_decision import decode                              # noqa: E402

LABELS = r'C:\Users\cbt\Desktop\嵌赛FPGA\lane_labels2.csv'
VAL = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg\official_val.json'
RUN = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
VIDDIR = r'C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材'
MANIFESTS = (r'D:\ADAS_Datasets\06_车道线_lane\_label_set2\manifest.json',
             r'D:\ADAS_Datasets\06_车道线_lane\_label_set3\manifest.json')


def load_val():
    val = set(json.load(open(VAL, encoding='utf-8')))
    lab = {}
    with open(LABELS, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            L, R = (r.get('L') or '').strip(), (r.get('R') or '').strip()
            if (L in ('dash', 'solid') or R in ('dash', 'solid')) and r['img'] in val:
                lab[r['img']] = (L, R)
    man = {}
    for mp in MANIFESTS:
        if os.path.exists(mp):
            for m in json.load(open(mp, encoding='utf-8')):
                if m.get('vid') and m.get('frame') is not None:
                    man.setdefault(m['img'], m)
    return lab, man


def grab(caps, man, img):
    m = man.get(img)
    if m is None:
        return None
    v = m['vid']
    if v not in caps:
        caps[v] = cv2.VideoCapture(os.path.join(VIDDIR, v))
    c = caps[v]
    c.set(cv2.CAP_PROP_POS_FRAMES, int(m['frame']))
    ok, fr = c.read()
    return fr if ok else None


def eval_one(net, dev, lab, man, win, mode, th, verbose=False):
    """在指定输入尺寸下评测，返回 (判决准确率, 类型准确率, 未检出数, 中位耗时ms)"""
    T.WIN = win
    T.BIN_TH = th if th else T.BIN_TH
    caps = {}
    n = ok_d = ok_t = none = 0
    ts = []
    for img, (L, R) in sorted(lab.items()):
        fr = grab(caps, man, img)
        if fr is None:
            continue
        g = fpga_gray(fr)
        if mode == 'gray':
            v = cv2.resize(g, win, interpolation=cv2.INTER_AREA)
        else:
            b = np.where(g > T.BIN_TH, 255, 0).astype(np.uint8)
            h0, w0 = g.shape
            fx, fy = w0 / win[0], h0 / win[1]
            if abs(fx - round(fx)) < 1e-6 and abs(fy - round(fy)) < 1e-6 and round(fx) >= 2:
                f = int(round(fx))
                hh, ww = h0 // f * f, w0 // f * f
                v = b[:hh, :ww].reshape(hh // f, f, ww // f, f).max(axis=(1, 3))
                v = cv2.resize(v, win, interpolation=cv2.INTER_NEAREST)
            else:
                v = cv2.resize(b, win, interpolation=cv2.INTER_NEAREST)
        x = torch.from_numpy(v.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
        x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
        t0 = time.perf_counter()
        with torch.no_grad():
            pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
        ts.append((time.perf_counter() - t0) * 1000)
        r = decode(pm)
        for side, gt in (('left', L), ('right', R)):
            if gt not in ('dash', 'solid'):
                continue
            n += 1
            if r[side] == 'none':
                none += 1
            if r[side] == gt:
                ok_t += 1
            if r['can_change_' + side] == (gt == 'dash'):
                ok_d += 1
    for c in caps.values():
        c.release()
    if n == 0:
        return None
    return ok_d / n, ok_t / n, none, float(np.median(ts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', default='lane_seg_gray_off.pt')
    ap.add_argument('--mode', default='gray')
    ap.add_argument('--th', type=int, default=None)
    ap.add_argument('--res', default='512x288,384x216,256x144,640x360,768x432,1024x576')
    a = ap.parse_args()

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    net.load_state_dict(torch.load(os.path.join(RUN, a.ckpt), map_location=dev))
    net.eval()
    lab, man = load_val()
    print('权重 %s  mode=%s  留出帧 %d' % (a.ckpt, a.mode, len(lab)))
    print('%12s %10s %10s %8s %10s %10s' %
          ('输入', '判决准确率', '类型准确率', '未检出', '相对算量', '中位ms'))
    base_px = None
    rows = []
    for rs in a.res.split(','):
        w, h = [int(v) for v in rs.lower().split('x')]
        r = eval_one(net, dev, lab, man, (w, h), a.mode, a.th)
        if r is None:
            print('%12s  无样本' % rs)
            continue
        acc, acct, none, ms = r
        px = w * h
        if base_px is None:
            base_px = px
        print('%12s %10.3f %10.3f %8d %9.2fx %10.1f'
              % (rs, acc, acct, none, px / base_px, ms))
        rows.append(dict(res=rs, acc=acc, acc_type=acct, none=none,
                         rel_compute=px / base_px, ms=ms))
    out = r'D:\AI\CLaude_workplace\临时文件\res_sweep_%s.json' % a.mode
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    print('\n-> %s' % out)


if __name__ == '__main__':
    main()
