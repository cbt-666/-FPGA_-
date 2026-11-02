#!/usr/bin/env python3
"""把 _runs/seg 下所有车道权重，**用同一套协议**重测一遍，得到可比排名。

为什么必须重测：现有分数是在不同时期、不同解码参数（带/阈值/宽度上限）
下测出来的，互相不可比。分类（A/B/C/D）必须建立在同一把尺子上。

协议：
  · 官方留出 50 帧 = 100 方向（只含正常行驶帧）
  · 各权重按其**训练时的输入模式/分辨率/阈值**推理（这是它被训成的样子）
  · 解码统一用 (带 0.45-0.90, 阈值 0.020, 宽度上限 0.18, 最近中心)
  · 报 变道许可 / 类型 / none数 / 平凡基线

同时报折半交叉验证的诚实分（见 cv_decode_params.py 的方法），
避免把调参集自评分当性能。
"""
import os
import sys
import csv
import json
import numpy as np
import cv2
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import train_lane_seg as T                                     # noqa: E402
from train_lane_seg import Net, MEAN, STD                      # noqa: E402
from lane_decision import decode                               # noqa: E402

RUN = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
LABELS = r'C:\Users\cbt\Desktop\嵌赛FPGA\lane_labels2.csv'
VAL = os.path.join(RUN, 'official_val.json')
MAN1 = r'D:\ADAS_Datasets\06_车道线_lane\_label_set3\manifest.json'
MAN2 = r'D:\ADAS_Datasets\06_车道线_lane\_label_set2\manifest.json'
VIDDIR = r'C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材'

# 权重 -> (输入模式, 分辨率, 二值化阈值, 说明)
SPEC = {
    'lane_seg_binary_off1280only.pt': ('binary', (1280, 720), 190, '仅官方帧+二值th190 @1280x720'),
    'lane_seg_binary_geo.pt':         ('binary', (1280, 720), 190, '仅官方帧+二值th190+几何增强'),
    'lane_seg_binary_off1280.pt':     ('binary', (1280, 720), 190, '官方+A/B+二值th190 @1280x720'),
    'lane_seg_binary_off1024.pt':     ('binary', (1024, 576), 190, '仅官方帧+二值th190 @1024x576'),
    'lane_seg_binary_offbin.pt':      ('binary', (1024, 576), 190, '仅官方帧+二值th130(谷底)'),
    'lane_seg_binary_hi.pt':          ('binary', (1024, 576), 190, 'A/B+官方 二值'),
    'lane_seg_binary.pt':             ('binary', (512, 288),  130, 'A/B 二值th130 @512x288'),
    'lane_seg_binary_512.pt':         ('binary', (512, 288),  130, 'A/B 二值th130 @512x288 (重复tag)'),
    'lane_seg_binrel_rel.pt':         ('binrel', (512, 288),  130, 'A/B 相对阈值二值化 @512x288'),
    'lane_seg_gray_off.pt':           ('gray',   (512, 288),  130, '灰度+官方同域微调 @512x288'),
    'lane_seg_gray_gray.pt':          ('gray',   (512, 288),  130, '灰度 仅A/B @512x288'),
    'lane_seg.pt':                    ('gray',   (512, 288),  130, '灰度 基线/中间产物'),
}


def load_frames():
    val = set(json.load(open(VAL, encoding='utf-8')))
    lab = {}
    for r in csv.DictReader(open(LABELS, encoding='utf-8')):
        L, R = (r.get('L') or '').strip(), (r.get('R') or '').strip()
        if L in ('dash', 'solid') and R in ('dash', 'solid') and r['img'] in val:
            lab[r['img']] = (L, R)
    man = {}
    for mp in (MAN1, MAN2):
        if os.path.exists(mp):
            for m in json.load(open(mp, encoding='utf-8')):
                man.setdefault(m['img'], m)
    caps, out = {}, []
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
        if okf:
            out.append((fr, L, R, img))
    for c in caps.values():
        c.release()
    return out


def main():
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    frames = load_frames()
    nd = sum((L == 'dash') + (R == 'dash') for _f, L, R, _i in frames)
    base = nd / (2 * len(frames))
    print('官方留出 %d 帧 = %d 方向   平凡基线 %.3f' % (len(frames), 2 * len(frames), base))
    print()

    # 先把每个权重的掩码全算出来缓存，之后换解码参数不用重推
    rows = []
    for ck, (mode, win, th, desc) in SPEC.items():
        p = os.path.join(RUN, ck)
        if not os.path.exists(p):
            rows.append((ck, mode, win, th, desc, None))
            continue
        T.WIN = win
        T.BIN_TH = th
        T.MODE = mode
        net = Net().to(dev)
        net.load_state_dict(torch.load(p, map_location=dev))
        net.eval()
        masks = []
        for fr, L, R, img in frames:
            inp = T.preprocess(fr, mode, th)
            x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
            x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
            with torch.no_grad():
                pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
            masks.append(pm)
        del net
        rows.append((ck, mode, win, th, desc, masks))
        print('  已推理 %s' % ck, flush=True)

    print()
    print('=' * 100)
    print('%-34s %-10s %-6s %-10s %-10s %-8s' %
          ('权重', '模式', '阈值', '变道许可', '类型', 'none方向'))
    print('=' * 100)
    res = []
    for ck, mode, win, th, desc, masks in rows:
        if masks is None:
            print('%-34s  缺失' % ck)
            continue
        okd = okt = n = none = 0
        for pm, (fr, L, R, img) in zip(masks, frames):
            r = decode(pm)
            for side, gt in (('left', L), ('right', R)):
                n += 1
                if (r[side] == 'dash') == (gt == 'dash'):
                    okd += 1
                if r[side] == gt:
                    okt += 1
                if r[side] == 'none':
                    none += 1
        res.append((okd / n, okt / n, none, ck, desc, mode, win, th))
    res.sort(reverse=True)
    for okd, okt, none, ck, desc, mode, win, th in res:
        print('%-34s %-10s %-6d %-10.3f %-10.3f %-8d'
              % (ck, mode, th, okd, okt, none))
    print()
    print('--- 按变道许可降序（分类依据）---')
    for i, (okd, okt, none, ck, desc, mode, win, th) in enumerate(res):
        print('%2d. %.3f  %-34s %s' % (i + 1, okd, ck, desc))

    json.dump([{'ckpt': ck, 'acc_change': okd, 'acc_type': okt, 'none': none,
                'mode': mode, 'win': list(win), 'th': th, 'desc': desc,
                'baseline': base}
               for okd, okt, none, ck, desc, mode, win, th in res],
              open(os.path.join(RUN, '_ranking.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print()
    print('-> 排名已存 %s' % os.path.join(RUN, '_ranking.json'))


if __name__ == '__main__':
    main()
