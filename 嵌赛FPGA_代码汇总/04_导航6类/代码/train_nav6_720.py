#!/usr/bin/env python3
"""导航 6 类 (方向 x 位数) 训练 —— 方形 1280 训练, 导出时再补 720

为什么方形训练:
   ultralytics 的训练期增强 (Mosaic / RandomPerspective) 假定 imgsz 是方形整数,
   非方形 (736,1280) 会直接崩。已验证的路线是: 方形 1280 训练 ->
   导出 ONNX 时对 FPN Concat 打补丁, 使 720 高度可推理 (旧模型即此路线, 实测 98.4%)。

输入: 49_导航6类_灰度 (Y=(77R+150G+29B)>>8, 与板上一致)
起点: navarrow_nomask_v2 (已验证箭头检测能力), 只学"位数"这一新维度

⚠️ fliplr 必须为 0 —— 左右转会互换
"""
import os
import warnings
warnings.filterwarnings('ignore')

import torch

DATA = r'D:\ADAS_Datasets\49_导航6类_灰度\data.yaml'
BASE_W = r'D:\ADAS_Datasets\_runs\navarrow_nomask_v2\weights\best.pt'
RUNS = r'D:\ADAS_Datasets\_runs'


def main():
    from ultralytics import YOLO
    print('GPU:', torch.cuda.get_device_name(0), flush=True)
    m = YOLO(BASE_W)
    m.train(
        data=DATA,
        epochs=120, imgsz=1280, batch=4, workers=0, device='0',
        project=RUNS, name='nav6_1280',
        exist_ok=True, seed=0, deterministic=True,
        hsv_h=0, hsv_s=0, hsv_v=0,
        fliplr=0, flipud=0, mixup=0.0, copy_paste=0.0,
        degrees=2.0, translate=0.08, scale=0.20, shear=0.0, mosaic=0.0,
        optimizer='auto', cos_lr=True, patience=40, cache=False,
        plots=True, verbose=True,
    )


if __name__ == '__main__':
    main()
