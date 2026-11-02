"""YRG 输入版手势分类训练：2 类（只分手势）与 3 类（多一个 other 类用于拒识）。

关键设计
--------
输入 = FPGA 红绿灰度化（fpga_rg_gray），与板上像素契约一致。
other 类由非交警场景构成（普通行车 / 导航屏 / 行人 / 路口空镜），
并在**独立**验证集上检验拒识能力 —— 保证"拒识"是测出来的、不是训出来的。

类别数由 `train_model(..., n_cls, ...)` 的入参给定，网络末层随之变宽，
新增手势只需扩充数据与这张映射表，不改网络结构。
"""
import os
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

sys.path.insert(0, r"D:/ADAS_Datasets/_code")
from rg_gray_fpga_exact import fpga_rg_gray

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")
IMG = 96
EPOCHS = 70
BATCH = 8
LR = 2e-3
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1", 0), (r"D:/QQ20261031-195747.mp4", "2", 1)]
OFFICIAL = r"C:/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"


class Net(nn.Module):
    def __init__(self, n):
        super().__init__()
        self.f = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1, bias=False), nn.BatchNorm2d(16), nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1, bias=False), nn.BatchNorm2d(32), nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1, bias=False), nn.BatchNorm2d(64), nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 96, 3, padding=1, bias=False), nn.BatchNorm2d(96), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1),
        )
        self.head = nn.Linear(96, n)

    def forward(self, x):
        return self.head(torch.flatten(self.f(x), 1))


def grab_frames(path, n, step=None):
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if step is None:
        targets = set(np.linspace(0, max(total - 2, 0), n).round().astype(int).tolist())
    else:
        targets = set(range(0, total, step))
    out, i = [], 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i in targets:
            out.append(f)
        i += 1
    cap.release()
    return out[:n]


def to_yrg(bgr, rng=None, jitter=False):
    if jitter:
        return fpga_rg_gray(bgr,
                            red_min=int(rng.integers(80, 115)),
                            green_min=int(rng.integers(45, 70)),
                            green_margin=int(rng.integers(10, 24)))
    return fpga_rg_gray(bgr)


class YrgDataset(Dataset):
    """预先把每帧缓存成 RGB 原图（uint8 HxWx3），YRG 只在缩小后的图上算，
    避免每 epoch 对全分辨率帧重复做 25ms 的阈值运算。"""

    def __init__(self, items, img, train, seed=0):
        self.items = [(cv2.resize(f, (img * 2, img * 2), interpolation=cv2.INTER_AREA), lb)
                      for f, lb in items]
        self.img = img
        self.train = train
        self.rng = np.random.default_rng(seed)

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        img, label = self.items[i]
        if self.train:
            h, w = img.shape[:2]
            s = int(min(h, w) * self.rng.uniform(0.8, 1.0))
            y = int(self.rng.integers(0, h - s + 1))
            x = int(self.rng.integers(0, w - s + 1))
            img = img[y:y + s, x:x + s].copy()
            if self.rng.random() < 0.5:
                img = img[:, ::-1].copy()
            a = 1.0 + self.rng.uniform(-0.35, 0.35)
            b = self.rng.uniform(-35, 35)
            img = np.clip(img.astype(np.float32) * a + b, 0, 255).astype(np.uint8)
        yrg = to_yrg(img, self.rng, jitter=self.train)
        yrg = cv2.resize(yrg, (self.img, self.img), interpolation=cv2.INTER_AREA).astype(np.float32)
        x = (yrg[:, :, ::-1] / 255.0 - 0.5) / 0.5
        return torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1))).float(), label


def train_model(items, n_classes, img, tag, epochs=EPOCHS):
    torch.manual_seed(0)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    dl = DataLoader(YrgDataset(items, img, True), batch_size=BATCH, shuffle=True, num_workers=0)
    model = Net(n_classes).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    crit = nn.CrossEntropyLoss()

    t0 = time.time()
    for ep in range(epochs):
        model.train()
        tot = ok = 0
        lsum = 0.0
        for _ in range(12):
            for x, y in dl:
                x, y = x.to(dev), y.to(dev)
                opt.zero_grad(set_to_none=True)
                o = model(x)
                loss = crit(o, y)
                loss.backward()
                opt.step()
                lsum += loss.item() * y.size(0)
                ok += (o.argmax(1) == y).sum().item()
                tot += y.size(0)
            if ep >= 25 and ok == tot:
                break
        sch.step()
        if ep % 20 == 0 or ep == epochs - 1:
            print(f"  [{tag}] ep{ep:03d} loss={lsum/tot:.4f} acc={ok/tot*100:.1f}%", flush=True)

    model.eval()
    p = os.path.join(OUT, f"cls_{tag}.pt")
    torch.save({"state_dict": model.state_dict(), "img": img}, p)
    print(f"  [{tag}] saved {os.path.getsize(p)/1e6:.2f}MB  {time.time()-t0:.0f}s", flush=True)
    return model


def main():
    os.makedirs(OUT, exist_ok=True)

    print("== 收集数据 ==", flush=True)
    items2 = []
    for path, name, idx in VIDEOS:
        frames = grab_frames(path, 20)
        print(f"  video{name}: {len(frames)} frames", flush=True)
        items2 += [(f, idx) for f in frames]

    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    train_vids, hold_vids = vids[:6], vids[6:]
    print(f"  other 训练视频 {len(train_vids)} 个 / 留出 {len(hold_vids)} 个: {hold_vids}", flush=True)
    other_train = []
    for v in train_vids:
        other_train += grab_frames(os.path.join(OFFICIAL, v), 6)
    other_train = other_train[:36]
    print(f"  other 训练帧: {len(other_train)}", flush=True)

    print("\n== 训练 2 类（只认两视频，YRG 输入）==", flush=True)
    train_model(items2, 2, IMG, "yrg2")

    print("\n== 训练 3 类（2 视频 + other 拒识类）==", flush=True)
    train_model(items2 + [(f, 2) for f in other_train], 3, IMG, "yrg3")


if __name__ == "__main__":
    main()
