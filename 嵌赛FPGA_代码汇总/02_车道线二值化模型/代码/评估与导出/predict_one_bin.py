#!/usr/bin/env python3
"""单张「已二值化」图片 -> 模型判定 + 掩码可视化 + 多缩放鲁棒性

用户给的是**已经二值化**的图，所以不再做灰度/二值化，
只做「阈值归一 -> 贴到模型输入 -> ImageNet 归一化」。
"""
import os
import sys
import argparse
import numpy as np
import cv2
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import train_lane_seg as T                                     # noqa: E402
from train_lane_seg import Net, MEAN, STD                      # noqa: E402
from lane_decision import decode                               # noqa: E402

DIR = r'C:\Users\cbt\Desktop\嵌赛FPGA\自己找的图片\车道\二值化'
CKPT = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg\lane_seg_binary_off1280only.pt'
CN = {'dash': '虚线', 'solid': '实线', 'none': '未检出'}
R32 = lambda v: max(int(round(v / 32)) * 32, 64)
HOWS = ['native', 'stretch1280', 'letterbox', 'stretch512']


def prep(b, how):
    h, w = b.shape[:2]
    if how == 'native':
        return cv2.resize(b, (R32(w), R32(h)), interpolation=cv2.INTER_NEAREST)
    if how.startswith('stretch'):
        tw, th = (1280, 720) if how.endswith('1280') else (512, 288)
        return cv2.resize(b, (tw, th), interpolation=cv2.INTER_NEAREST)
    if how == 'letterbox':
        s = min(1280 / w, 720 / h)
        nw, nh = max(int(round(w * s)), 1), max(int(round(h * s)), 1)
        r = cv2.resize(b, (nw, nh), interpolation=cv2.INTER_NEAREST)
        c = np.zeros((720, 1280), np.uint8)
        x0, y0 = (1280 - nw) // 2, (720 - nh) // 2
        c[y0:y0 + nh, x0:x0 + nw] = r
        return c
    raise ValueError(how)


def draw(inp, pm, r, lab):
    vis = cv2.cvtColor(inp, cv2.COLOR_GRAY2BGR)
    ov = vis.copy()
    ov[pm == 1] = (0, 255, 0)
    ov[pm == 2] = (0, 0, 255)
    sel = pm > 0
    vis[sel] = (vis[sel] * 0.40 + ov[sel] * 0.60).astype(np.uint8)
    H, W = vis.shape[:2]
    cv2.rectangle(vis, (0, int(H * 0.45)), (W, int(H * 0.90)), (255, 255, 0), 3)
    for k, col in (('left_x', (255, 160, 0)), ('right_x', (0, 160, 255))):
        x = r.get(k)
        if x is not None:
            cv2.line(vis, (x, int(H * 0.42)), (x, int(H * 0.93)), col, 3)
    txt = 'L=%s %s   R=%s %s' % (
        r['left'], 'CHANGE' if r['can_change_left'] else 'KEEP',
        r['right'], 'CHANGE' if r['can_change_right'] else 'KEEP')
    cv2.rectangle(vis, (0, 0), (W, 40), (0, 0, 0), -1)
    cv2.putText(vis, lab, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(vis, txt, (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default='4.png')
    ap.add_argument('--th', type=int, default=190)
    ap.add_argument('--out', default=r'D:\ADAS_Datasets\临时_队友资料\pred4.jpg')
    a = ap.parse_args()
    T.BIN_TH = a.th

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    net.load_state_dict(torch.load(CKPT, map_location=dev))
    net.eval()

    p = os.path.join(DIR, a.file)
    g = cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_GRAYSCALE)
    if g is None:
        print('读不到 %s' % p)
        return
    oh, ow = g.shape[:2]
    u, c = np.unique(g, return_counts=True)
    top = sorted(zip(c, u), reverse=True)[:3]
    print('文件   : %s' % a.file)
    print('尺寸   : %dx%d  (%.2f MP)   模型训练分辨率 1280x720 (0.92 MP)'
          % (ow, oh, ow * oh / 1e6))
    print('灰度众数: %s' % [(int(v), round(100.0 * n / g.size, 1)) for n, v in top])
    print('长宽比 : %.3f   训练域 16:9 = 1.778' % (ow / oh))
    print()
    print('模型   : %s' % os.path.basename(CKPT))
    print()

    tiles, verdicts = [], []
    for how in HOWS:
        T.WIN = (512, 288) if how == 'stretch512' else (1280, 720)
        b = np.where(g > a.th, 255, 0).astype(np.uint8)
        inp = prep(b, how)
        x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
        x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
        with torch.no_grad():
            pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
        r = decode(pm)
        verdicts.append((r['left'], r['right']))
        print('%-12s  左=%-4s -> %-12s | 右=%-4s -> %-12s  (虚%d 实%d px)'
              % (how, CN[r['left']],
                 '可以变道' if r['can_change_left'] else '禁止变道',
                 CN[r['right']],
                 '可以变道' if r['can_change_right'] else '禁止变道',
                 int((pm == 1).sum()), int((pm == 2).sum())))
        tiles.append(cv2.resize(draw(inp, pm, r, how), (640, 360)))

    uniq = set(verdicts)
    # ⚠️ 稳定性要看**决策**(能否变道)，不是线型标签：
    # "未检出" 和 "实线" 都映射到"禁止变道"，线型不同但决策相同，不算不稳定。
    decs = [((a == 'dash'), (b == 'dash')) for a, b in verdicts]
    from collections import Counter
    print()
    if len(set(decs)) == 1:
        print('=> 决策稳定：左=%s  右=%s'
              % ('可变道' if decs[0][0] else '禁止变道',
                 '可变道' if decs[0][1] else '禁止变道'))
        if len(uniq) > 1:
            print('   （线型标签有 %d 种，但都映射到同一决策，不影响结论）' % len(uniq))
    else:
        cnt = Counter(decs)
        main_d, n = cnt.most_common(1)[0]
        print('=> ⚠️ 决策不稳定（%d 种不同答案）。多数票：左=%s 右=%s (%d/%d)'
              % (len(set(decs)),
                 '可变道' if main_d[0] else '禁止变道',
                 '可变道' if main_d[1] else '禁止变道', n, len(decs)))
        for d, k in cnt.most_common():
            print('     左=%s 右=%s  出现 %d 次'
                  % ('可变道' if d[0] else '禁变', '可变道' if d[1] else '禁变', k))

    im = np.vstack([np.hstack(tiles[0:2]), np.hstack(tiles[2:4])])
    cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tofile(a.out)
    print()
    print('可视化 %s' % a.out)


if __name__ == '__main__':
    main()
