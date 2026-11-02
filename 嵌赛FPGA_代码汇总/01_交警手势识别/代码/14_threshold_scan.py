"""回答"能不能用置信度滤掉其他图片"：扫描阈值，看拒识率 vs 目标保留率。"""
import os
import sys

import cv2
import numpy as np
import torch

sys.path.insert(0, r"D:/ADAS_Datasets/_code")
from rg_gray_fpga_exact import fpga_rg_gray

import importlib.util

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")
OFFICIAL = r"C:/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]

spec = importlib.util.spec_from_file_location("t12", os.path.join(ROOT, "12_train_yrg.py"))
t12 = importlib.util.module_from_spec(spec)
sys.modules["t12"] = t12
spec.loader.exec_module(t12)


def predict_conf(model, frames, img, dev):
    out = []
    with torch.no_grad():
        for k in range(0, len(frames), 16):
            b = []
            for f in frames[k:k + 16]:
                y = fpga_rg_gray(cv2.resize(f, (img * 2, img * 2), interpolation=cv2.INTER_AREA))
                y = cv2.resize(y, (img, img), interpolation=cv2.INTER_AREA).astype(np.float32)
                x = (y[:, :, ::-1] / 255.0 - 0.5) / 0.5
                b.append(np.ascontiguousarray(x.transpose(2, 0, 1)))
            t = torch.from_numpy(np.stack(b)).float().to(dev)
            p = torch.softmax(model(t), 1).cpu().numpy()
            out += [float(p[i].max()) for i in range(len(p))]
    return np.array(out)


def grab(path, n):
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tg = set(np.linspace(0, max(total - 2, 0), n).round().astype(int).tolist())
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


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ck = torch.load(os.path.join(OUT, "cls_yrg2.pt"), map_location="cpu", weights_only=False)
    model = t12.Net(2)
    model.load_state_dict(ck["state_dict"])
    model.eval()
    model.to(dev)
    img = ck["img"]

    target = np.concatenate([predict_conf(model, grab(p, 120), img, dev) for p, _ in VIDEOS])
    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    hold = np.concatenate([predict_conf(model, grab(os.path.join(OFFICIAL, v), 30), img, dev) for v in vids[6:]])
    allo = np.concatenate([predict_conf(model, grab(os.path.join(OFFICIAL, v), 30), img, dev) for v in vids])

    print(f"目标两视频  n={len(target)}  置信度 min={target.min():.4f} p1={np.percentile(target,1):.4f} mean={target.mean():.4f}")
    print(f"留出官方    n={len(hold)}    置信度 min={hold.min():.4f} p50={np.percentile(hold,50):.4f} mean={hold.mean():.4f}")
    print(f"全部官方    n={len(allo)}    置信度 mean={allo.mean():.4f}")
    print()
    print("阈值    目标保留率   留出拒识率   全部官方拒识率")
    for th in [0.90, 0.95, 0.98, 0.99, 0.995, 0.999, 0.9999]:
        keep = np.mean(target >= th)
        rej = np.mean(hold < th)
        reja = np.mean(allo < th)
        flag = "  <-- 只能保住一半目标" if keep < 0.95 else ""
        print(f"{th:<8.4f} {keep*100:8.1f}% {rej*100:11.1f}% {reja*100:13.1f}%{flag}")
    print()
    print(f"目标最小置信度 = {target.min():.6f}，任何低于此值的阈值都会误杀目标帧")
    print(f"留出集最大置信度 = {hold.max():.6f}，高于此值才能 100% 拒掉留出")


if __name__ == "__main__":
    main()
