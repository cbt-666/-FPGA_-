#!/usr/bin/env python3
"""导航 6 类 —— 全帧微调 (有意过拟合演示视频)

用户目标: 在 导航.mp4 这 2776 帧上逐帧零错。
该视频即演示素材, 因此把全部帧并入训练集是符合目标的 —— 但这意味着
之后的精度不再代表泛化能力, 只看"这视频上对不对"。

起点: nav6_1280/weights/best.pt (epoch 50, 已学会 6 类)
数据: 49_导航6类_灰度, 但全部 2776 帧都放进 train (val 复制同一批, 仅用于记录曲线)

⚠️ 上次在 epoch 50 因系统内存耗尽 (OpenCV 分配失败) 崩溃,
   本次降低并发并定期回收, 若再崩可从 last.pt 继续。
"""
import os
import gc
import shutil
import warnings
warnings.filterwarnings('ignore')

import cv2
import numpy as np
import torch

B = r'D:\ADAS_Datasets'
SRC_GRAY = os.path.join(B, '49_导航6类_灰度')
SRC_COLOR = os.path.join(B, '48_导航6类')
FULL = os.path.join(B, '52_导航6类_720_blur')
BASE_W = os.path.join(B, '_runs', 'nav6_full1280', 'weights', 'best.pt')


def build():
    """train = 全部 2776 帧 (灰度), val = 同一批 (仅为让 ultralytics 能跑)"""
    for split in ('train', 'val'):
        for sub in ('images', 'labels'):
            p = os.path.join(FULL, split, sub)
            os.makedirs(p, exist_ok=True)
            for fn in os.listdir(p):
                os.remove(os.path.join(p, fn))
    n = 0
    for split_in in ('train', 'val'):
        si = os.path.join(SRC_GRAY, split_in, 'images')
        sl = os.path.join(SRC_GRAY, split_in, 'labels')
        for f in sorted(os.listdir(si)):
            if not f.endswith('.jpg'):
                continue
            shutil.copyfile(os.path.join(si, f),
                            os.path.join(FULL, 'train', 'images', f))
            shutil.copyfile(os.path.join(sl, f.replace('.jpg', '.txt')),
                            os.path.join(FULL, 'train', 'labels',
                                         f.replace('.jpg', '.txt')))
            n += 1
    # val 用小样本即可 (只为触发验证)
    fs = sorted(os.listdir(os.path.join(FULL, 'train', 'images')))[::40]
    for f in fs:
        shutil.copyfile(os.path.join(FULL, 'train', 'images', f),
                        os.path.join(FULL, 'val', 'images', f))
        shutil.copyfile(os.path.join(FULL, 'train', 'labels', f.replace('.jpg', '.txt')),
                        os.path.join(FULL, 'val', 'labels', f.replace('.jpg', '.txt')))
    for c in (os.path.join(FULL, 'train', 'labels.cache'),
              os.path.join(FULL, 'val', 'labels.cache')):
        if os.path.exists(c):
            os.remove(c)
    yml = os.path.join(FULL, 'data.yaml')
    with open(yml, 'w', encoding='utf-8') as f:
        f.write('path: %s\ntrain: train/images\nval: val/images\n\n' % FULL)
        f.write('nc: 6\nnames:\n')
        for i, nm in enumerate(['right_3', 'right_2', 'left_3',
                                'left_2', 'uturn_3', 'uturn_2']):
            f.write('  %d: %s\n' % (i, nm))
    print('全帧训练集 %d 张, val %d 张 -> %s' % (n, len(fs), FULL), flush=True)
    return yml


def main():
    yml = os.path.join(FULL, 'data.yaml')
    gc.collect()
    from ultralytics import YOLO
    print('GPU:', torch.cuda.get_device_name(0), flush=True)
    m = YOLO(BASE_W)
    m.train(
        data=yml,
        epochs=25, imgsz=1280, batch=2, workers=0, device='0',
        project=os.path.join(B, '_runs'), name='nav6_blur',
        exist_ok=True, seed=0, deterministic=False,
        hsv_h=0, hsv_s=0, hsv_v=0,
        fliplr=0, flipud=0, mixup=0.0, copy_paste=0.0,
        degrees=2.0, translate=0.08, scale=0.20, shear=0.0, mosaic=0.0,
        optimizer='auto', cos_lr=True, patience=40, cache=False,
        plots=False, verbose=True,
    )


if __name__ == '__main__':
    main()
