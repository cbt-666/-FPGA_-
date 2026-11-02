#!/usr/bin/env python3
"""对单张图跑完整链路：队友二值化 -> 模型 -> 边界虚实 -> 变道决策

用法:
  python predict_one.py <图片路径> [--model gray|binrel|binary] [--ckpt xxx.pt]

输出一张对照图，左到右分别是：
  ① 原图 + 决策叠加
  ② 队友二值化(固定130) 的结果     <- 队友现在实际喂给模型的
  ③ 相对阈值二值化的结果            <- 建议的改法
  ④ 模型预测的分割掩码(绿=虚线 红=实线)
"""
import os
import sys
import argparse
import numpy as np
import cv2
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import train_lane_seg as T                                     # noqa: E402
from train_lane_seg import (Net, fpga_gray, binarize_pool,      # noqa: E402
                            binarize_rel_pool, MEAN, STD, BIN_TH)
from lane_decision import decode, visualize                      # noqa: E402

OUTDIR = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
PANEL = (470, 264)


def load(ckpt, res):
    w, h = res.split('x')
    T.WIN = (int(w), int(h))
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    net.load_state_dict(torch.load(os.path.join(OUTDIR, ckpt), map_location=dev))
    net.eval()
    return net, dev


@torch.no_grad()
def infer(net, inp, dev):
    x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
    x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
    return net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)


def label(im, txt, col=(0, 255, 255)):
    cv2.rectangle(im, (0, 0), (im.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(im, txt, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('img')
    ap.add_argument('--model', default='gray', choices=['gray', 'binrel', 'binary'])
    ap.add_argument('--ckpt', default=None)
    ap.add_argument('--res', default='512x288')
    ap.add_argument('--out', default=r'D:\ADAS_Datasets\临时_队友资料\predict_result.jpg')
    a = ap.parse_args()
    ck = a.ckpt or {'gray': 'lane_seg_gray_gray.pt',
                    'binrel': 'lane_seg_binrel_rel.pt',
                    'binary': 'lane_seg_binary_512.pt'}[a.model]

    bgr = cv2.imdecode(np.fromfile(a.img, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        print('读不到图片:', a.img)
        return
    net, dev = load(ck, a.res)

    # 三个输入版本
    g0 = fpga_gray(bgr)
    mid = cv2.resize(g0, (1024, 576), interpolation=cv2.INTER_AREA)
    v_fixed = binarize_pool(mid, T.WIN, BIN_TH)      # 队友现状
    v_rel = binarize_rel_pool(mid, T.WIN)            # 相对阈值
    v_gray = cv2.resize(g0, T.WIN, interpolation=cv2.INTER_AREA)

    inp = {'gray': v_gray, 'binrel': v_rel, 'binary': v_fixed}[a.model]
    pm = infer(net, inp, dev)
    res = decode(pm)

    H, W = bgr.shape[:2]
    vis = visualize(bgr, cv2.resize(pm, (W, H), interpolation=cv2.INTER_NEAREST), res)
    verdict = 'L:%s(%s)  R:%s(%s)' % (
        res['left'], '可变道' if res['can_change_left'] else '直行',
        res['right'], '可变道' if res['can_change_right'] else '直行')
    cv2.putText(vis, verdict, (8, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (0, 0, 0), 5)
    cv2.putText(vis, verdict, (8, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (0, 255, 255), 2)

    def panel(x, txt, col=(0, 255, 255)):
        p = cv2.cvtColor(cv2.resize(x, PANEL, interpolation=cv2.INTER_NEAREST),
                         cv2.COLOR_GRAY2BGR) if x.ndim == 2 else cv2.resize(x, PANEL)
        label(p, txt, col)
        return p

    row = np.hstack([
        cv2.resize(vis, PANEL),
        panel(v_fixed, '1. teammate binarize (fixed 130)'),
        panel(v_rel, '2. relative threshold (suggested)', (0, 200, 255)),
        panel((pm.astype(np.uint8) * 90), '3. model mask  (green=dash red=solid)'),
    ])
    # 掩码上色
    m = row[:, 3 * PANEL[0]:]
    mm = np.zeros_like(m)
    hh, ww = PANEL[1] - 0, PANEL[0]
    sub = cv2.resize(pm, (PANEL[0], PANEL[1]), interpolation=cv2.INTER_NEAREST)
    mm[sub == 1] = (0, 255, 0)
    mm[sub == 2] = (0, 0, 255)
    row[:, 3 * PANEL[0]:] = np.maximum(m // 3, mm)
    label(row[:, 3 * PANEL[0]:], '3. model mask (green=dash / red=solid)')

    cv2.imencode('.jpg', row, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tofile(a.out)
    print('模型 %s (%s)' % (a.model, ck))
    print('决策: 左=%s(%s)  右=%s(%s)'
          % (res['left'], '可变道' if res['can_change_left'] else '直行',
             res['right'], '可变道' if res['can_change_right'] else '直行'))
    print('saved', a.out)


if __name__ == '__main__':
    main()
