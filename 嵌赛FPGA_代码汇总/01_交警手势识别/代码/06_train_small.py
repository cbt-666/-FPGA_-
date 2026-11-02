"""超轻量 2 类 CNN：64x64 输入，~8 万参数。

关键：训练时以 50% 概率把图转灰度，逼模型学形状而非颜色，
避免"彩色 100% / FPGA 灰度全错"的捷径。
"""
import os
import time

import cv2
import numpy as np
import PIL.Image
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

ROOT = os.path.dirname(os.path.abspath(__file__))
DS = os.path.join(ROOT, "dataset")
OUT = os.path.join(ROOT, "out")
IMG = 64
EPOCHS = 120
BATCH = 8
LR = 2e-3
GRAY_P = 0.5


class TwoClassNet(nn.Module):
    def __init__(self, n=2):
        super().__init__()
        self.f = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1, bias=False), nn.BatchNorm2d(16), nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                                     # 32
            nn.Conv2d(16, 32, 3, padding=1, bias=False), nn.BatchNorm2d(32), nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                                     # 16
            nn.Conv2d(32, 64, 3, padding=1, bias=False), nn.BatchNorm2d(64), nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                                     # 8
            nn.Conv2d(64, 96, 3, padding=1, bias=False), nn.BatchNorm2d(96), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1),
        )
        self.head = nn.Linear(96, n)

    def forward(self, x):
        return self.head(torch.flatten(self.f(x), 1))


class Cls2Dataset(Dataset):
    def __init__(self, split, train):
        self.items = []
        for ci, cname in enumerate(sorted(os.listdir(os.path.join(DS, split)))):
            d = os.path.join(DS, split, cname)
            for f in sorted(os.listdir(d)):
                if f.endswith(".jpg"):
                    self.items.append((os.path.join(d, f), ci))
        self.train = train
        self.rng = np.random.default_rng(0)

    def __len__(self):
        return len(self.items)

    def _aug(self, img):
        h, w = img.shape[:2]
        s = int(min(h, w) * 0.85)
        y = self.rng.integers(0, h - s + 1)
        x = self.rng.integers(0, w - s + 1)
        img = img[y:y + s, x:x + s]
        if self.rng.random() < 0.5:
            img = img[:, ::-1]
        img = cv2.resize(img, (IMG, IMG), interpolation=cv2.INTER_AREA)
        if self.rng.random() < GRAY_P:
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = np.stack([g] * 3, -1)
        a = 1.0 + self.rng.uniform(-0.35, 0.35)
        b = self.rng.uniform(-40, 40)
        img = np.clip(img.astype(np.float32) * a + b, 0, 255)
        return img

    def __getitem__(self, i):
        path, label = self.items[i]
        raw = np.fromfile(path, np.uint8)
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if self.train:
            img = self._aug(img)
        else:
            img = cv2.resize(img, (IMG, IMG), interpolation=cv2.INTER_AREA).astype(np.float32)
        x = (img[:, :, ::-1] / 255.0 - 0.5) / 0.5      # BGR->RGB, 归一到 [-1,1]
        return torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1))).float(), label


def main():
    os.makedirs(OUT, exist_ok=True)
    torch.manual_seed(0)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    tr = DataLoader(Cls2Dataset("train", True), batch_size=BATCH, shuffle=True, num_workers=0)
    va = DataLoader(Cls2Dataset("val", False), batch_size=BATCH, shuffle=False, num_workers=0)

    model = TwoClassNet().to(dev)
    print("params", sum(p.numel() for p in model.parameters()))
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=EPOCHS)
    crit = nn.CrossEntropyLoss()

    t0 = time.time()
    for ep in range(EPOCHS):
        model.train()
        tot = ok = 0
        lsum = 0.0
        for _ in range(25):
            for x, y in tr:
                x, y = x.to(dev), y.to(dev)
                opt.zero_grad(set_to_none=True)
                o = model(x)
                loss = crit(o, y)
                loss.backward()
                opt.step()
                lsum += loss.item() * y.size(0)
                ok += (o.argmax(1) == y).sum().item()
                tot += y.size(0)
            if ep >= 30 and ok == tot:
                break
        sch.step()
        if ep % 20 == 0 or ep == EPOCHS - 1:
            model.eval()
            vc = vt = 0
            with torch.no_grad():
                for x, y in va:
                    x, y = x.to(dev), y.to(dev)
                    vc += (model(x).argmax(1) == y).sum().item()
                    vt += y.size(0)
            print(f"ep{ep:03d} loss={lsum/tot:.4f} train={ok/tot*100:.1f}% val={vc/vt*100:.1f}%")

    torch.save({"state_dict": model.state_dict(), "classes": ["1", "2"], "img": IMG},
               os.path.join(OUT, "cls2_small.pt"))
    sz = os.path.getsize(os.path.join(OUT, "cls2_small.pt")) / 1e6
    print(f"saved cls2_small.pt  {sz:.2f}MB  elapsed={time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
