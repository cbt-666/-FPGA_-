"""关键测试：FPGA 输入是 1280x720 (16:9)，但这两个视频是近方形/竖幅。

video1 = 932x904 (1.03)   video2 = 898x996 (0.90)   FPGA = 1280x720 (1.78)

如果演示时走 FPGA 管线，必然发生 letterbox 或拉伸。测一下影响。
"""
import os
import sys

import cv2
import numpy as np
import torch
import importlib.util

sys.path.insert(0, r"D:/ADAS_Datasets/_code")
from rg_gray_fpga_exact import fpga_rg_gray

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")

spec = importlib.util.spec_from_file_location("t12", os.path.join(ROOT, "12_train_yrg.py"))
t12 = importlib.util.module_from_spec(spec)
sys.modules["t12"] = t12
spec.loader.exec_module(t12)

FPGA_W, FPGA_H = 1280, 720
CASES = [(r"D:/QQ20261031-194504.mp4", "1", 6.0), (r"D:/QQ20261031-195747.mp4", "2", 3.0)]


def frame_at(path, sec):
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(sec * fps))
    ok, f = cap.read()
    cap.release()
    return f


def to_fpga_stretch(img):
    """直接拉伸到 1280x720（不改宽高比）。"""
    return cv2.resize(img, (FPGA_W, FPGA_H), interpolation=cv2.INTER_LINEAR)


def to_fpga_letterbox(img):
    """等比缩放到 1280x720 内，空白补黑（保持宽高比）。"""
    canvas = np.zeros((FPGA_H, FPGA_W, 3), np.uint8)
    sc = min(FPGA_W / img.shape[1], FPGA_H / img.shape[0])
    nw, nh = int(img.shape[1] * sc), int(img.shape[0] * sc)
    small = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    x, y = (FPGA_W - nw) // 2, (FPGA_H - nh) // 2
    canvas[y:y + nh, x:x + nw] = small
    return canvas


def load(tag, n_cls):
    ck = torch.load(os.path.join(OUT, f"cls_{tag}.pt"), map_location="cpu", weights_only=False)
    m = t12.Net(n_cls)
    m.load_state_dict(ck["state_dict"])
    m.eval()
    if torch.cuda.is_available():
        m.cuda()
    return m, ck["img"]


def predict(model, bgr, img, dev, names):
    yrg = fpga_rg_gray(bgr)
    y = cv2.resize(yrg, (img, img), interpolation=cv2.INTER_AREA).astype(np.float32)
    x = (y[:, :, ::-1] / 255.0 - 0.5) / 0.5
    x = np.ascontiguousarray(x.transpose(2, 0, 1))[None]
    with torch.no_grad():
        p = torch.softmax(model(torch.from_numpy(x).float().to(dev)), 1).cpu().numpy()[0]
    k = int(p.argmax())
    return names[k], float(p[k]), p


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    m2, i2 = load("yrg2", 2)
    m3, i3 = load("yrg3", 3)
    N2, N3 = ["1", "2"], ["1", "2", "other"]

    for path, gt, sec in CASES:
        f = frame_at(path, sec)
        h, w = f.shape[:2]
        ar = w / h
        print(f"\n{'='*70}")
        print(f"{os.path.basename(path)}  原始 {w}x{h}  宽高比={ar:.2f}  期望=类{gt}")
        print(f"{'='*70}")
        variants = [("原始本机比例", f),
                    ("FPGA 拉伸 1280x720", to_fpga_stretch(f)),
                    ("FPGA 补黑 1280x720", to_fpga_letterbox(f))]
        for vn, im in variants:
            for tag, model, ip, names in [("yrg2", m2, i2, N2), ("yrg3", m3, i3, N3)]:
                pred, conf, probs = predict(model, im, ip, dev, names)
                ok = "OK " if pred == gt else "NG "
                ps = " ".join(f"{n}={v:.3f}" for n, v in zip(names, probs))
                print(f"  [{vn:20s}] {tag}: {ok} {pred:<5s} {conf:.4f}  [{ps}]")
            print()


if __name__ == "__main__":
    main()
