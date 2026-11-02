#!/usr/bin/env python3
"""导航箭头 —— 在 1280x720 几何下微调

为什么需要:
   原模型按 736(方形letterbox) 训练并导出, 但 FPGA 输入是 1280x720。
   720 时 FPN 的 P5 上采样得到 44, P4 是 45 -> 需要 Concat 补丁插值 1px,
   这个错位模型没见过 -> 直接导出精度掉 4.6 个点(98.4% -> 93.8%)。

   所以在 720 几何下微调, 让模型学会这个尺寸。

⚠️ 必须同时打两个补丁:
   1. Concat.forward 容忍尺寸差
   2. check_imgsz 不要取整到 32 的倍数(720 会被改成 736),
      也不要 max_dim=1 把矩形压成方形
"""
import warnings
warnings.filterwarnings('ignore')

import torch
import torch.nn.functional as F
from ultralytics.nn.modules.conv import Concat


def patch_concat():
    def fwd(self, x):
        hw = [t.shape[2:] for t in x]
        mh = max(h for h, _ in hw)
        mw = max(w for _, w in hw)
        out = []
        for t, (h, w) in zip(x, hw):
            if (h, w) != (mh, mw):
                t = F.interpolate(t, size=(mh, mw), mode='nearest')
            out.append(t)
        return torch.cat(out, self.d)
    Concat.forward = fwd


def patch_imgsz():
    import ultralytics.engine.trainer as TR
    import ultralytics.engine.validator as VA

    def _keep(imgsz, stride=32, min_dim=1, max_dim=2, floor=0):
        if isinstance(imgsz, (list, tuple)):
            return list(imgsz)
        return imgsz
    TR.check_imgsz = _keep
    VA.check_imgsz = _keep


def main():
    patch_concat()
    patch_imgsz()

    from ultralytics import YOLO
    m = YOLO(r'D:\ADAS_Datasets\_runs\navarrow_nomask_v2\weights\best.pt')
    print('GPU:', torch.cuda.get_device_name(0), flush=True)
    m.train(
        data=r'D:\ADAS_Datasets\45_导航箭头_灰度_全图\data.yaml',
        epochs=80, imgsz=(720, 1280), batch=4, workers=0, device='0',
        project=r'D:\ADAS_Datasets\_runs', name='navarrow_720',
        exist_ok=True, seed=0, deterministic=True,
        hsv_h=0, hsv_s=0, hsv_v=0,
        fliplr=0, flipud=0, mixup=0.0, copy_paste=0.0,
        degrees=3.0, translate=0.1, scale=0.4, shear=0.0, mosaic=0.0,
        optimizer='auto', cos_lr=True, patience=40, cache=False,
        plots=True, verbose=True,
    )


if __name__ == '__main__':
    main()
