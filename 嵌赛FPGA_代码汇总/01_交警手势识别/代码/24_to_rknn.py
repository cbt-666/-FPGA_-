"""构建 rk3568 fp16 .rknn 并在模拟器上验证精度（PyTorch vs RKNN）。

注意：rknn-toolkit2 的模拟器只支持 load_onnx+build，不支持 load_rknn。
"""
import os
import sys

import cv2
import numpy as np
import torch
import importlib.util

for _p in ("/mnt/d/ADAS_Datasets/_code", r"D:/ADAS_Datasets/_code"):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break
from rg_gray_fpga_exact import fpga_rg_gray

from rknn.api import RKNN

ROOT = "/mnt/d/AI/CLaude_workplace/临时文件/two_video_cls"
OUT = os.path.join(ROOT, "out")
ONNX = os.path.join(OUT, "cls_v2yrg3.onnx")
RKNN_OUT = os.path.join(OUT, "cls_v2yrg3_rk3568_fp16.rknn")
OFFICIAL = "/mnt/c/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"
VIDEOS = [("/mnt/d/QQ20261031-194504.mp4", "1"), ("/mnt/d/QQ20261031-195747.mp4", "2")]
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
    yrg = fpga_rg_gray(frame)
    small = cv2.resize(yrg, (img, img), interpolation=cv2.INTER_AREA)
    rgb = small[:, :, ::-1]
    x = (rgb.astype(np.float32) / 255.0 - 0.5) / 0.5
    return np.ascontiguousarray(x.transpose(2, 0, 1))[None], np.ascontiguousarray(rgb)


def softmax(v):
    e = np.exp(v - v.max())
    return e / e.sum()


def main():
    ck = torch.load(os.path.join(OUT, "cls_v2yrg3.pt"), map_location="cpu", weights_only=False)
    img = ck["img"]
    model = t20.Net(3)
    model.load_state_dict(ck["state_dict"])
    model.eval()

    rk = RKNN(verbose=False)
    r = rk.config(mean_values=[[127.5] * 3], std_values=[[127.5] * 3],
                  target_platform="rk3568", float_dtype="float16", optimization_level=3)
    print(f"config ret={r}", flush=True)
    r = rk.load_onnx(model=ONNX)
    print(f"load_onnx ret={r}", flush=True)
    r = rk.build(do_quantization=False)
    print(f"build ret={r}", flush=True)
    if r != 0:
        return 1

    r = rk.init_runtime(target=None)
    print(f"init_runtime(模拟器) ret={r}", flush=True)
    if r != 0:
        return 1

    # ---- 模拟器精度验证 ----
    samples = []
    for path, gt in VIDEOS:
        for f in grab(path, 60, step=6):
            samples.append((f, NAMES.index(gt), f"video{gt}"))
    if os.path.isdir(OFFICIAL):
        vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
        for v in vids[6:]:
            for f in grab(os.path.join(OFFICIAL, v), 10):
                samples.append((f, 2, "other-holdout"))
    for fn, gt in [("paste_1.png", "1"), ("paste_0.png", "2")]:
        p = os.path.join(ROOT, fn)
        if os.path.exists(p):
            im = cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
            samples.append((im, NAMES.index(gt), "paste"))

    print(f"\n模拟器验证样本数: {len(samples)}", flush=True)
    agree = 0
    max_diff = 0.0
    per_src = {}
    maxerr = []
    for frame, gt, src in samples:
        xt, xr = make_input(frame, img)
        with torch.no_grad():
            pt = softmax(model(torch.from_numpy(xt)).numpy()[0])
        out = rk.inference(inputs=[xr], data_format="nhwc")[0][0]
        rp = softmax(out)
        max_diff = max(max_diff, float(np.abs(pt - rp).max()))
        a, b = int(pt.argmax()), int(rp.argmax())
        agree += int(a == b)
        s = per_src.setdefault(src, [0, 0, 0])
        s[1] += 1
        s[0] += int(b == gt)
        s[2] += int(a == gt)
        if b != gt:
            maxerr.append((src, gt, NAMES[b], float(rp.max())))

    print(f"决策一致率 PyTorch vs RKNN: {agree}/{len(samples)} = {agree/len(samples)*100:.1f}%")
    print(f"概率最大偏差: {max_diff:.6f}")
    print(f"\n{'来源':<16s} {'RKNN准确率':>12s} {'PyTorch准确率':>14s}")
    for src, (ok_r, n, ok_t) in per_src.items():
        print(f"{src:<16s} {ok_r}/{n} ({ok_r/n*100:.0f}%)   {ok_t}/{n} ({ok_t/n*100:.0f}%)")
    if maxerr:
        print("\nRKNN 错判明细（前 10 条）:")
        for e in maxerr[:10]:
            print(f"  {e}")

    r = rk.export_rknn(RKNN_OUT)
    if r == 0 and os.path.exists(RKNN_OUT):
        print(f"\n导出成功: {os.path.basename(RKNN_OUT)}  {os.path.getsize(RKNN_OUT)/1e6:.3f} MB")
    rk.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
