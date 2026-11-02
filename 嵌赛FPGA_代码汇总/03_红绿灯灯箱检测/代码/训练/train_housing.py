"""训练灯箱检测模型 (单类, 只输出框坐标).

显存 8G 约束下的选择:
  yolov8n + imgsz=1280 + batch=4
  理由见 results: 框宽中位 17px@640 -> 1280 才是合理分辨率.

用法: python train_housing.py [n1408|n1280|s1280]
"""
import sys
import os
from ultralytics import YOLO

RUNS = r'D:\ADAS_Datasets\_runs'
DATA = r'D:\ADAS_Datasets\42_housing_1c\data.yaml'

CFG = {
    'n1280': dict(model='yolov8n.pt', imgsz=1280, batch=8, name='housing_n1280b'),
    'n1408': dict(model='yolov8n.pt', imgsz=1408, batch=4, name='housing_n1408'),
    's1280': dict(model='yolov8s.pt', imgsz=1280, batch=4, name='housing_s1280'),
}


def main():
    key = sys.argv[1] if len(sys.argv) > 1 else 'n1280'
    if key not in CFG:
        print('用法: python train_housing.py %s' % '|'.join(CFG)); return
    c = CFG[key]
    weights = os.path.join(r'D:\ADAS_Datasets\_code', c['model'])
    model = YOLO(weights)
    model.train(
        data=DATA,
        epochs=200,
        imgsz=c['imgsz'],
        batch=c['batch'],
        rect=True,                 # 16:9 批次不再补成正方形, 省约 44% 算力
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
        degrees=2.0,               # 小角度旋转: 相机微抖
        translate=0.15,            # 平移: 灯在画面里的位置变化很大
        scale=0.7,                 # 尺度: 远/近灯箱差一个数量级
        shear=1.0,
        perspective=0.0003,        # 轻微透视: 行车视角起伏
        flipud=0.0,                # 不能上下翻: 灯箱不上下颠倒
        fliplr=0.5,                # 可左右翻: 左右对称
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.5,   # 颜色/亮度扰动 (对应 FPGA 灰度差异)
        mosaic=1.0,                # 小目标检测关键
        mixup=0.0,
        copy_paste=0.0,
        erasing=0.2,               # 随机遮挡: 模拟灯箱被树/电线杆挡住
    )


if __name__ == '__main__':
    main()
