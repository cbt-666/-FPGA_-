#!/usr/bin/env python3
"""同一批二值图，用多种"贴到模型输入"的方式各喂一遍，看结论是否稳定

用户要求"转成 1280×720 再喂一遍"。但代码本来就已升采样到 1280×720
（`_fit(b, WIN, nearest=True)`），直接重跑结果必然相同。
真正会影响结果的是**放大方式**：
  · native      —— 按原始尺寸就近取 /32 的尺寸喂（线保持真实细度）
  · stretch     —— 直接拉伸到 1280×720（长宽比 1.641 -> 1.778，横向拉宽 8%）
  · letterbox   —— 保持比例缩到高度 720 再补黑边到 1280×720
  · stretch512  —— 拉伸到 512x288（训练时的低分辨率）
结论若四种一致 -> 说明模型对预处理不敏感，结论可信；
若不一致 -> 说明这批图正落在模型的脆弱面上。
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

DEFAULT_DIR = r'C:\Users\cbt\Desktop\嵌赛FPGA\自己找的图片\车道\二值化'
CKPT = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg\lane_seg_binary_off1280only.pt'
CN = {'dash': '虚线', 'solid': '实线', 'none': '未检出'}
R32 = lambda v: max(int(round(v / 32)) * 32, 64)


def prep(b, how, th):
    """b: 已是二值的 uint8 图 -> 模型输入尺寸的单通道图"""
    h, w = b.shape[:2]
    if how == 'native':
        return cv2.resize(b, (R32(w), R32(h)), interpolation=cv2.INTER_NEAREST)
    if how.startswith('stretch'):
        tw, thh = (1280, 720) if how.endswith('1280') else (512, 288)
        return cv2.resize(b, (tw, thh), interpolation=cv2.INTER_NEAREST)
    if how == 'letterbox':
        s = min(1280 / w, 720 / h)
        nw, nh = max(int(round(w * s)), 1), max(int(round(h * s)), 1)
        r = cv2.resize(b, (nw, nh), interpolation=cv2.INTER_NEAREST)
        canvas = np.zeros((720, 1280), np.uint8)
        x0, y0 = (1280 - nw) // 2, (720 - nh) // 2
        canvas[y0:y0 + nh, x0:x0 + nw] = r
        return canvas
    raise ValueError(how)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default=DEFAULT_DIR)
    ap.add_argument('--th', type=int, default=190)
    ap.add_argument('--out', default=r'D:\ADAS_Datasets\临时_队友资料\user_bin_variants.jpg')
    a = ap.parse_args()

    T.BIN_TH = a.th
    files = sorted(f for f in os.listdir(a.dir)
                   if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')))
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'

    HOWS = ['native', 'stretch1280', 'letterbox', 'stretch512']
    results = {}
    for how in HOWS:
        net = Net().to(dev)
        net.load_state_dict(torch.load(CKPT, map_location=dev))
        net.eval()
        T.WIN = (1280, 720) if how != 'stretch512' else (512, 288)
        per = {}
        for f in files:
            g = cv2.imdecode(np.fromfile(os.path.join(a.dir, f), np.uint8),
                             cv2.IMREAD_GRAYSCALE)
            if g is None:
                continue
            b = np.where(g > a.th, 255, 0).astype(np.uint8)
            inp = prep(b, how, a.th)
            x = torch.from_numpy(inp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
            x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
            with torch.no_grad():
                pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)
            r = decode(pm)
            per[f] = (r, pm, inp)
        results[how] = per

    # ---- 文字报告 ----
    for f in files:
        oh, ow = cv2.imdecode(np.fromfile(os.path.join(a.dir, f), np.uint8),
                              cv2.IMREAD_GRAYSCALE).shape[:2]
        print('══ %s  (原图 %dx%d) ══' % (f, ow, oh))
        verdicts = []
        for how in HOWS:
            if f not in results[how]:
                continue
            r = results[how][f][0]
            v = (r['left'], r['right'])
            verdicts.append(v)
            print('   %-12s L=%-4s(%s)  R=%-4s(%s)'
                  % (how, CN[r['left']], '可变道' if r['can_change_left'] else '禁变',
                     CN[r['right']], '可变道' if r['can_change_right'] else '禁变'))
        uniq = set(verdicts)
        print('   -> %s' % ('四种方式结论完全一致 ✅' if len(uniq) == 1
                            else '⚠️ 结论不一致（%d 种不同答案）' % len(uniq)))
        print()

    # ---- 拼图 ----
    tiles = []
    for f in files:
        row = []
        for how in HOWS:
            if f not in results[how]:
                continue
            r, pm, inp = results[how][f]
            vis = cv2.cvtColor(inp, cv2.COLOR_GRAY2BGR)
            ov = vis.copy()
            ov[pm == 1] = (0, 255, 0)
            ov[pm == 2] = (0, 0, 255)
            sel = pm > 0
            vis[sel] = (vis[sel] * 0.35 + ov[sel] * 0.65).astype(np.uint8)
            H, W = vis.shape[:2]
            cv2.rectangle(vis, (0, int(H * 0.45)), (W, int(H * 0.90)), (255, 255, 0), 2)
            for k, col in (('left_x', (255, 160, 0)), ('right_x', (0, 160, 255))):
                xx = r.get(k)
                if xx is not None:
                    cv2.line(vis, (xx, int(H * 0.42)), (xx, int(H * 0.93)), col, 3)
            cv2.putText(vis, how, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 5)
            cv2.putText(vis, how, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
            t = 'L=%s %s' % (r['left'], 'CHG' if r['can_change_left'] else 'KEEP')
            t2 = 'R=%s %s' % (r['right'], 'CHG' if r['can_change_right'] else 'KEEP')
            cv2.putText(vis, t, (6, H - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(vis, t2, (6, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            row.append(cv2.resize(vis, (420, 236)))
        if row:
            while len(row) < 4:
                row.append(np.zeros_like(row[0]))
            tiles.append(np.hstack(row))
    if tiles:
        cv2.imencode('.jpg', np.vstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 88])[1].tofile(a.out)
        print('可视化 %s' % a.out)


if __name__ == '__main__':
    main()
