#!/usr/bin/env python3
"""在留出官方帧上出「输入 / 预测 / 判定」对照图

用户在验收标准里关心的是「左右能否变道」，所以不只画掩码，
还把 GT 与 PRED 的判定并排写在图上，对错用颜色区分。
"""
import os
import sys
import csv
import json
import argparse
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
    ap.add_argument('--ckpt', default='lane_seg_gray_off.pt')
    ap.add_argument('--mode', default='gray')
    ap.add_argument('--res', default='512x288')
    ap.add_argument('--th', type=int, default=None, help='二值化阈值')
    ap.add_argument('--n', type=int, default=8)
    ap.add_argument('--out', default=r'D:\ADAS_Datasets\临时_队友资料\lane_holdout.jpg')
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

    caps = {}
    good, bad = [], []
    for img, (L, R) in sorted(lab.items()):
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
        is_ok = ((r['can_change_left'] == (L == 'dash')) and
                 (r['can_change_right'] == (R == 'dash')))
        tile = render(fr, inp, pm, r, L, R, is_ok, a.mode)
        (good if is_ok else bad).append(tile)
    for c in caps.values():
        c.release()

    picks = good[:max(a.n - 3, 1)] + bad[:3]
    while len(picks) < a.n:
        picks += picks[:1] or [np.zeros((315, 560, 3), np.uint8)]
    picks = picks[:a.n]
    while len(picks) % 2:
        picks.append(np.zeros_like(picks[0]))
    im = np.vstack([np.hstack(picks[i:i + 2]) for i in range(0, len(picks), 2)])
    cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 88])[1].tofile(a.out)
    print('saved', a.out)
    print('对 %d 张 / 错 %d 张 (本次评估共 %d 张)' % (len(good), len(bad), len(good) + len(bad)))


def render(fr, inp, pm, r, L, R, is_ok, mode):
    """左半：变换后输入（模型真正看到的）  右半：预测掩码 + 判定"""
    H, W = fr.shape[:2]
    out = fr.copy()
    mm = cv2.resize(pm, (W, H), interpolation=cv2.INTER_NEAREST)
    ov = np.zeros_like(out)
    ov[mm == 1] = (0, 255, 0)      # 虚线 绿
    ov[mm == 2] = (0, 0, 255)      # 实线 红
    sel = mm > 0
    out[sel] = (out[sel] * 0.45 + ov[sel] * 0.55).astype(np.uint8)
    # 参考带
    cv2.rectangle(out, (0, int(H * 0.45)), (W, int(H * 0.90)), (255, 255, 0), 2)
    for key, col in (('left_x', (255, 160, 0)), ('right_x', (0, 160, 255))):
        x = r.get(key)
        if x is not None:
            cv2.line(out, (x, int(H * 0.42)), (x, int(H * 0.93)), col, 3)
    # 左上角贴一小块「变换后输入」，让用户核对变换
    th = int(H * 0.22)
    tw = int(th * inp.shape[1] / inp.shape[0])
    small = cv2.cvtColor(cv2.resize(inp, (tw, th), interpolation=cv2.INTER_NEAREST),
                         cv2.COLOR_GRAY2BGR)
    out[6:6 + th, 6:6 + tw] = small
    cv2.rectangle(out, (6, 6), (6 + tw, 6 + th), (255, 255, 255), 2)
    cv2.putText(out, mode, (12, 6 + th - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2)

    cv2.rectangle(out, (0, 0), (W, 54), (0, 0, 0), -1)
    cv2.putText(out, 'GT   L=%-5s R=%-5s' % (L, R), (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    col = (0, 255, 0) if is_ok else (0, 0, 255)
    cv2.putText(out, 'PRED L=%-5s R=%-5s  %s'
                % (r['left'], r['right'], 'OK' if is_ok else 'X'), (8, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
    return cv2.resize(out, (560, 315))


if __name__ == '__main__':
    main()
