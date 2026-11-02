import os
import sys
import time

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchvision import datasets, models, transforms

ROOT = os.path.dirname(os.path.abspath(__file__))
DS = os.path.join(ROOT, "dataset")
OUT = os.path.join(ROOT, "out")
IMG = 64          # FPGA 输入 1280x720 按 20x 下采样
EPOCHS = 120
BATCH = 8
LR = 1e-3
REPEATS = 25      # epoch 内重复采样，配合 batch=8 保证每轮多步


def build_model():
    model = models.mobilenet_v3_small(weights=None)
    model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, 2)
    return model


def main():
    os.makedirs(OUT, exist_ok=True)
    torch.manual_seed(0)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    norm = transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    train_tf = transforms.Compose([
        transforms.Resize((IMG, IMG)),
        transforms.RandomCrop(IMG, padding=6),
        transforms.RandomHorizontalFlip(),
        transforms.ColorJitter(brightness=0.3, contrast=0.3),
        transforms.ToTensor(),
        norm,
    ])
    eval_tf = transforms.Compose([
        transforms.Resize((IMG, IMG)),
        transforms.ToTensor(),
        norm,
    ])

    train_ds = datasets.ImageFolder(os.path.join(DS, "train"), transform=train_tf)
    val_ds = datasets.ImageFolder(os.path.join(DS, "val"), transform=eval_tf)
    print("classes:", train_ds.classes, [train_ds.class_to_idx[c] for c in train_ds.classes])

    train_loader = DataLoader(train_ds, batch_size=BATCH, shuffle=True, num_workers=0, drop_last=False)
    val_loader = DataLoader(val_ds, batch_size=BATCH, shuffle=False, num_workers=0)

    model = build_model().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=EPOCHS)
    crit = nn.CrossEntropyLoss()

    t0 = time.time()
    for ep in range(EPOCHS):
        model.train()
        tot = correct = 0
        loss_sum = 0.0
        for _ in range(REPEATS):
            for x, y in train_loader:
                x, y = x.to(device), y.to(device)
                opt.zero_grad(set_to_none=True)
                out = model(x)
                loss = crit(out, y)
                loss.backward()
                opt.step()
                loss_sum += loss.item() * y.size(0)
                correct += (out.argmax(1) == y).sum().item()
                tot += y.size(0)
            if ep >= 20 and correct == tot:
                break
        sched.step()

        if ep % 10 == 0 or ep == EPOCHS - 1:
            model.eval()
            vc = vt = 0
            with torch.no_grad():
                for x, y in val_loader:
                    x, y = x.to(device), y.to(device)
                    vc += (model(x).argmax(1) == y).sum().item()
                    vt += y.size(0)
            print(f"ep{ep:03d} loss={loss_sum/tot:.4f} train={correct/tot*100:.1f}% val={vc/vt*100:.1f}%")

    model.eval()
    vc = vt = 0
    per_class = {c: [0, 0] for c in train_ds.classes}
    with torch.no_grad():
        for x, y in val_loader:
            x, y = x.to(device), y.to(device)
            pred = model(x).argmax(1)
            vc += (pred == y).sum().item()
            vt += y.size(0)
            for p, t in zip(pred.tolist(), y.tolist()):
                per_class[train_ds.classes[t]][1] += 1
                if p == t:
                    per_class[train_ds.classes[t]][0] += 1
    print(f"FINAL val acc = {vc/vt*100:.1f}% ({vc}/{vt})")
    for c, (ok, n) in per_class.items():
        print(f"  class {c}: {ok}/{n} = {ok/n*100:.1f}%")

    torch.save({"state_dict": model.state_dict(), "classes": train_ds.classes, "img": IMG},
               os.path.join(OUT, "cls2.pt"))
    print(f"saved {os.path.join(OUT, 'cls2.pt')}  elapsed={time.time()-t0:.1f}s")


if __name__ == "__main__":
    sys.exit(main())
