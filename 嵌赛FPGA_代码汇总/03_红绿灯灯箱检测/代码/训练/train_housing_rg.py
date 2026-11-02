"""训练灯箱检测模型 —— 纯红绿灰度域版 (去全灰度干扰).

与 train_housing.py 唯一区别: 数据换成本 build_housing_rg.py 生成的 42_housing_1c_rg。
超参、输入尺寸、增强全部保持不变, 保证与旧版是"只换数据域"的干净对照。

用法: python train_housing_rg.py [n1280|n1408|s1280]
"""
import sys
import os
from ultralytics import YOLO

RUNS = r'D:\ADAS_Datasets\_runs'
DATA = r'D:\ADAS_Datasets\42_housing_1c_rg\data.yaml'

CFG = {
    'n1280': dict(model='yolov8n.pt', imgsz=1280, batch=8, name='housing_rg_n1280'),
    'n1408': dict(model='yolov8n.pt', imgsz=1408, batch=4, name='housing_rg_n1408'),
    's1280': dict(model='yolov8s.pt', imgsz=1280, batch=4, name='housing_rg_s1280'),
}


def main():
    key = sys.argv[1] if len(sys.argv) > 1 else 'n1280'
    if key not in CFG:
        print('用法: python train_housing_rg.py %s' % '|'.join(CFG)); return
    c = CFG[key]
    weights = os.path.join(r'D:\ADAS_Datasets\_code', c['model'])
    model = YOLO(weights)
    model.train(
        data=DATA,
        epochs=200,
        imgsz=c['imgsz'],
        batch=c['batch'],
        rect=True,                 # 16:9 批次不再补成正方形
        device=0,
        workers=0,                 # Windows 共享内存会炸
        project=RUNS,
        name=c['name'],
        exist_ok=True,
        seed=0,
        deterministic=True,
        cos_lr=True,
        close_mosaic=30,
        patience=50,
        amp=False,                 # Blackwell 分配器 INTERNAL ASSERT
        cache=False,
        plots=True,
        verbose=True,
        # ---- 泛化 ----
        degrees=2.0,
        translate=0.15,
        scale=0.7,
        shear=1.0,
        perspective=0.0003,
        flipud=0.0,                # 灯箱不上下颠倒
        fliplr=0.5,
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.5,   # 亮度/饱和度扰动: 对应 FPGA 灰度差异
        mosaic=1.0,
        mixup=0.0,
        copy_paste=0.0,
        erasing=0.2,
    )


if __name__ == '__main__':
    main()
