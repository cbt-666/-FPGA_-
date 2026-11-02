"""精度验证：RKNN 模拟器 vs PyTorch，在同一批真实帧上对比。

RKNN 配置了 mean=127.5 / std=127.5，与训练端的 (x/255-0.5)/0.5 = x/127.5-1 等价，
所以模拟器输入应当直接喂 uint8 的 RGB 图（不自己做归一化）。
"""
import os
import sys

import cv2
import numpy as np
import torch
import importlib.util

for _p in (r"D:/ADAS_Datasets/_code", "/mnt/d/ADAS_Datasets/_code"):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break
from rg_gray_fpga_exact import fpga_rg_gray

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")
RKNN_PATH = os.path.join(OUT, "cls_v2yrg3_rk3568_fp16.rknn")
OFFICIAL = r"C:/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]
NAMES = ["1", "2", "other"]

spec = importlib.util.spec_from_file_location("t20", os.path.join(ROOT, "20_train_v2_letterbox.py"))
t20 = importlib.util.module_from_spec(spec)
sys.modules["t20"] = t20
spec.loader.exec_module(t20)


def grab(path, n, step=None):
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tg = set(np.linspace(0, max(total - 2, 0), n).round().astype(int).tolist()) if step is None \
        else set(range(0, total, step))
    out, i = [], 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i in tg:
            out.append(f)
        i += 1
    cap.release()
    return out


def make_input(frame, img):
    """产出两份等价输入：torch 用 float RGB CHW，rknn 用 uint8 RGB HWC。"""
    yrg = fpga_rg_gray(frame)
    small = cv2.resize(yrg, (img, img), interpolation=cv2.INTER_AREA)
    rgb = small[:, :, ::-1]                                   # BGR->RGB
    x = (rgb.astype(np.float32) / 255.0 - 0.5) / 0.5
    return np.ascontiguousarray(x.transpose(2, 0, 1))[None], np.ascontiguousarray(rgb)


def softmax(v):
    e = np.exp(v - v.max())
    return e / e.sum()


def main():
    from rknn.api import RKNN

    ck = torch.load(os.path.join(OUT, "cls_v2yrg3.pt"), map_location="cpu", weights_only=False)
    img = ck["img"]
    model = t20.Net(3)
    model.load_state_dict(ck["state_dict"])
    model.eval()

    rk = RKNN(verbose=False)
    if rk.load_rknn(RKNN_PATH) != 0:
        print("load_rknn 失败")
        return 1
    if rk.init_runtime(target=None) != 0:
        print("init_runtime 失败（模拟器）")
        return 1

    # 收集测试样本：两视频 + 留出官方视频 + 两张截图
    samples = []
    for path, gt in VIDEOS:
        for f in grab(path, 60, step=6):
            samples.append((f, NAMES.index(gt), f"video{gt}"))
    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    for v in vids[6:]:
        for f in grab(os.path.join(OFFICIAL, v), 10):
            samples.append((f, 2, "other"))
    for fn, gt in [("paste_1.png", "1"), ("paste_0.png", "2")]:
        p = os.path.join(ROOT, fn)
        if os.path.exists(p):
            im = cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
            samples.append((im, NAMES.index(gt), "paste"))

    print(f"样本数: {len(samples)}")

    agree = 0
    max_diff = 0.0
    per_cls = {}
    for frame, gt, src in samples:
        xt, xr = make_input(frame, img)
        with torch.no_grad():
            pt = softmax(model(torch.from_numpy(xt)).numpy()[0])
        onnx_out = rk.inference(inputs=[xr], data_format="nhwc")[0][0]
        rknn_p = softmax(onnx_out)
        max_diff = max(max_diff, float(np.abs(pt - rknn_p).max()))
        a, b = int(pt.argmax()), int(rknn_p.argmax())
        agree += int(a == b)
        k = per_cls.setdefault(src, [0, 0])
        k[1] += 1
        k[0] += int(b == gt)

    print(f"\n决策一致率 PyTorch vs RKNN: {agree}/{len(samples)} = {agree/len(samples)*100:.1f}%")
    print(f"概率最大偏差: {max_diff:.6f}")
    print("\nRKNN 各自准确率:")
    for src, (ok, n) in per_cls.items():
        print(f"  {src:8s} {ok}/{n} = {ok/n*100:.1f}%")

    rk.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
