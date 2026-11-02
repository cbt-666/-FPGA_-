import os

import cv2
import numpy as np
import torch

from importlib import import_module
net = import_module("06_train_small") if False else None

ROOT = os.path.dirname(os.path.abspath(__file__))
CKPT = os.path.join(ROOT, "out", "cls2_small.pt")
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]
STEP = 3

import nn_def
build = nn_def.build


def prep(img, img_size):
    img = cv2.resize(img, (img_size, img_size), interpolation=cv2.INTER_AREA).astype(np.float32)
    x = (img[:, :, ::-1] / 255.0 - 0.5) / 0.5
    return torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1))).float()


def main():
    ck = torch.load(CKPT, map_location="cpu", weights_only=False)
    IMG, classes = ck["img"], ck["classes"]
    model = build()
    model.load_state_dict(ck["state_dict"])
    model.eval()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(dev)

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

        def predict(imgs):
            out = []
            with torch.no_grad():
                for k in range(0, len(imgs), 16):
                    x = torch.stack([prep(f, IMG) for f in imgs[k:k + 16]]).to(dev)
                    out += [classes[p] for p in model(x).argmax(1).tolist()]
            return out

        rgb = predict(frames)
        gray3 = [np.stack([cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)] * 3, -1) for f in frames]
        gr = predict(gray3)
        a_rgb = sum(p == label for p in rgb) / len(rgb)
        a_gr = sum(p == label for p in gr) / len(gr)
        bad_rgb = [k * STEP for k, p in enumerate(rgb) if p != label]
        bad_gr = [k * STEP for k, p in enumerate(gr) if p != label]
        print(f"video{label}: n={len(frames)} RGB={a_rgb*100:.1f}% GRAY={a_gr*100:.1f}%")
        if bad_rgb:
            print(f"   RGB wrong frames: {bad_rgb[:10]}")
        if bad_gr:
            print(f"   GRAY wrong frames: {bad_gr[:10]}")

    # 交叉混淆矩阵（全片 RGB）
    print("\n--- 混淆（RGB 全片）---")
    for path, label in VIDEOS:
        cap = cv2.VideoCapture(path)
        frames = []
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            if i % 10 == 0:
                frames.append(f)
            i += 1
        cap.release()
        with torch.no_grad():
            preds = []
            for k in range(0, len(frames), 16):
                x = torch.stack([prep(f, IMG) for f in frames[k:k + 16]]).to(dev)
                preds += [classes[p] for p in model(x).argmax(1).tolist()]
        n1 = preds.count("1")
        print(f"真{label}: 判1={n1} 判2={len(preds)-n1}  (n={len(preds)})")


if __name__ == "__main__":
    main()
