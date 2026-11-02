"""v2: 加入随机黑边(letterbox)增强，专治播放器截图 / 有效区域不一致的输入。

部署版即本脚本配置：黑边鲁棒 + 宽高比混合，保证板上 16:9 拉伸与各种取景
比例下判决稳定。

新增增强：以 60% 概率把画面放进一个更大的黑色画布（横 1.0~2.5 倍、纵 1.0~1.3 倍），
模拟播放器左右黑条与上下 UI 区域。其余增强（随机裁、翻转、亮度、YRG 阈值抖动）保持不变。
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
LETTERBOX_P = 0.6


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


def grab_frames(path, n):
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
    return out[:n]


def letterbox_aug(img, rng):
    """把画面放进更大的黑色画布，模拟播放器截图。"""
    h, w = img.shape[:2]
    mw = rng.uniform(1.0, 2.5)
    mh = rng.uniform(1.0, 1.3)
    cw, ch = max(int(w * mw), w), max(int(h * mh), h)
    canvas = np.zeros((ch, cw, 3), np.uint8)
    x = int(rng.integers(0, cw - w + 1))
    y = int(rng.integers(0, ch - h + 1))
    canvas[y:y + h, x:x + w] = img
    return canvas


def to_yrg(bgr, rng=None, jitter=False):
    if jitter:
        return fpga_rg_gray(bgr,
                            red_min=int(rng.integers(80, 115)),
                            green_min=int(rng.integers(45, 70)),
                            green_margin=int(rng.integers(10, 24)))
    return fpga_rg_gray(bgr)


class YrgDataset(Dataset):
    def __init__(self, items, img, train, seed=0, letterbox_p=0.0):
        self.items = [(cv2.resize(f, (img * 2, img * 2), interpolation=cv2.INTER_AREA), lb)
                      for f, lb in items]
        self.img = img
        self.train = train
        self.rng = np.random.default_rng(seed)
        self.letterbox_p = letterbox_p

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        img, label = self.items[i]
        if self.train:
            if self.letterbox_p > 0 and self.rng.random() < self.letterbox_p:
                img = letterbox_aug(img, self.rng)
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


def train_model(items, n_classes, img, tag, letterbox_p, epochs=EPOCHS):
    torch.manual_seed(0)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    dl = DataLoader(YrgDataset(items, img, True, letterbox_p=letterbox_p),
                    batch_size=BATCH, shuffle=True, num_workers=0)
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
    train_vids = vids[:6]
    other_train = []
    for v in train_vids:
        other_train += grab_frames(os.path.join(OFFICIAL, v), 6)
    other_train = other_train[:36]
    print(f"  other 训练帧: {len(other_train)}（留出: {vids[6:]}）", flush=True)

    print("\n== v2-3类（含黑边增强）==", flush=True)
    train_model(items2 + [(f, 2) for f in other_train], 3, IMG, "v2yrg3", LETTERBOX_P)

    print("\n== v2-2类（含黑边增强）==", flush=True)
    train_model(items2, 2, IMG, "v2yrg2", LETTERBOX_P)


if __name__ == "__main__":
    main()
