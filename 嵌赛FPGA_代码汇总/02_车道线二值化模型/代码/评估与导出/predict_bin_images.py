#!/usr/bin/env python3
"""把【已经二值化】的图片直接喂给训练好的模型，只输出模型的判定

注意：输入已是二值图，所以**不再重复做灰度化/二值化**，
只做「阈值归一(清理压缩伪影) -> 贴到 1280x720 -> ImageNet 归一化」。
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
OUTDIR = r'D:\ADAS_Datasets\临时_队友资料'
CN = {'dash': '虚线', 'solid': '实线', 'none': '未检出'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default=DEFAULT_DIR)
    ap.add_argument('--th', type=int, default=190)
    ap.add_argument('--res', default='1280x720')
    ap.add_argument('--out', default=os.path.join(OUTDIR, 'user_bin_pred.jpg'))
    a = ap.parse_args()

    w, h = a.res.split('x')
    T.WIN = (int(w), int(h))
    T.BIN_TH = a.th

    files = sorted(f for f in os.listdir(a.dir)
                   if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')))
    print('目录: %s' % a.dir)
    print('找到 %d 张图' % len(files))
    print('模型: %s' % os.path.basename(CKPT))
    print('输入规格: 阈值%d / %dx%d' % (a.th, T.WIN[0], T.WIN[1]))
    print()

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = Net().to(dev)
    net.load_state_dict(torch.load(CKPT, map_location=dev))
    net.eval()

    tiles = []
    for f in files:
        p = os.path.join(a.dir, f)
        g = cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_GRAYSCALE)
        if g is None:
            print('%s : 读取失败' % f)
            continue
        oh, ow = g.shape[:2]
        # 已是二值图 -> 只做阈值归一（清理 jpg 压缩产生的灰边），阈值化对纯 0/255 是恒等变换
        b = np.where(g > a.th, 255, 0).astype(np.uint8)
        # 贴到训练分辨率（最近邻，保持二值语义）
        binp = T._fit(b, T.WIN, nearest=True)

        x = torch.from_numpy(binp.astype(np.float32) / 255.0).unsqueeze(0).repeat(3, 1, 1)
        x = (x - torch.from_numpy(MEAN).view(3, 1, 1)) / torch.from_numpy(STD).view(3, 1, 1)
        with torch.no_grad():
            pm = net(x.unsqueeze(0).to(dev)).argmax(1)[0].cpu().numpy().astype(np.uint8)

        r = decode(pm)
        n1, n2 = int((pm == 1).sum()), int((pm == 2).sum())
        print('── %s  (原图 %dx%d) ──' % (f, ow, oh))
        print('   左侧: %-4s  -> %s' % (CN[r['left']],
              '可以变道' if r['can_change_left'] else '禁止变道(直行)'))
        print('   右侧: %-4s  -> %s' % (CN[r['right']],
              '可以变道' if r['can_change_right'] else '禁止变道(直行)'))
        print('   掩码像素: 虚线 %d, 实线 %d' % (n1, n2))
        print()

        # 拼一张可视化（左=喂进去的输入，右=模型掩码+判定）
        H, W = binp.shape
        left = cv2.cvtColor(binp, cv2.COLOR_GRAY2BGR)
        right = cv2.cvtColor(binp, cv2.COLOR_GRAY2BGR)
        ov = right.copy()
        ov[pm == 1] = (0, 255, 0)
        ov[pm == 2] = (0, 0, 255)
        sel = pm > 0
        right[sel] = (right[sel] * 0.35 + ov[sel] * 0.65).astype(np.uint8)
        cv2.rectangle(right, (0, int(H * 0.45)), (W, int(H * 0.90)), (255, 255, 0), 2)
        for k, col in (('left_x', (255, 160, 0)), ('right_x', (0, 160, 255))):
            xx = r.get(k)
            if xx is not None:
                cv2.line(right, (xx, int(H * 0.42)), (xx, int(H * 0.93)), col, 3)
        row = np.hstack([left, right])
        cv2.putText(row, 'INPUT(binarized)', (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 0), 5)
        cv2.putText(row, 'INPUT(binarized)', (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2)
        cv2.putText(row, 'MODEL OUTPUT', (W + 8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 0), 5)
        cv2.putText(row, 'MODEL OUTPUT', (W + 8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2)
        txt = 'L=%-4s %s   R=%-4s %s' % (
            CN[r['left']], 'CHANGE' if r['can_change_left'] else 'KEEP',
            CN[r['right']], 'CHANGE' if r['can_change_right'] else 'KEEP')
        cv2.putText(row, f, (8, row.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2)
        cv2.putText(row, txt, (W + 8, row.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2)
        tiles.append(cv2.resize(row, (1280, 360)))

    if tiles:
        cv2.imencode('.jpg', np.vstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tofile(a.out)
        print('可视化已保存 %s' % a.out)


if __name__ == '__main__':
    main()
