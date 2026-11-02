import os

import cv2
import numpy as np
import PIL.Image
import torch
import torch.nn as nn
from torchvision import models, transforms

ROOT = os.path.dirname(os.path.abspath(__file__))
CKPT = os.path.join(ROOT, "out", "cls2.pt")
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]
STEP = 3   # 每 3 帧取一帧做全片扫描


def build_model():
    model = models.mobilenet_v3_small(weights=None)
    model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, 2)
    return model


def run(model, tf, frames, device, idx_to_name):
    x = torch.stack([tf(PIL.Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))) for f in frames]).to(device)
    with torch.no_grad():
        p = model(x).argmax(1).tolist()
    return [idx_to_name[i] for i in p]


def main():
    ck = torch.load(CKPT, map_location="cpu", weights_only=False)
    IMG, classes = ck["img"], ck["classes"]
    idx_to_name = {i: c for i, c in enumerate(classes)}
    model = build_model()
    model.load_state_dict(ck["state_dict"])
    model.eval()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)

    norm = transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    tf = transforms.Compose([transforms.Resize((IMG, IMG)), transforms.ToTensor(), norm])

    for path, label in VIDEOS:
        cap = cv2.VideoCapture(path)
        frames = []
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            if i % STEP == 0:
                frames.append(f)
            i += 1
        cap.release()

        rgb = run(model, tf, frames, device, idx_to_name)
        acc = sum(p == label for p in rgb) / len(rgb)

        # 模拟 FPGA YRG 灰度（保留红绿语义的亮度化）并与 RGB 三通道复制
        grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
        gray3 = [cv2.cvtColor(g, cv2.COLOR_GRAY2BGR) for g in grays]
        gr = run(model, tf, gray3, device, idx_to_name)
        accg = sum(p == label for p in gr) / len(gr)

        wrong = [f"{k*STEP}" for k, (p, q) in enumerate(zip(rgb, gr)) if p != label or q != label]
        print(f"video{label}: frames={len(frames)} RGB_acc={acc*100:.1f}% GRAY_acc={accg*100:.1f}%"
              f" wrong_at={wrong[:12]}{'...' if len(wrong) > 12 else ''}")

    # 全灰输入（两路视频都变灰）下的崩溃测试
    print("note: GRAY 测试是把彩图转灰后喂给 RGB 三通道模型，用于看颜色丢失后还稳不稳")


if __name__ == "__main__":
    main()
