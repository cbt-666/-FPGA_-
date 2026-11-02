"""评估：置信度分布 + 拒识阈值扫描（YRG 输入）。"""
import os
import sys

import cv2
import numpy as np
import torch

sys.path.insert(0, r"D:/ADAS_Datasets/_code")
from rg_gray_fpga_exact import fpga_rg_gray

from importlib import import_module
import importlib.machinery, importlib.util

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")
OFFICIAL = r"C:/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]


def _load_net():
    spec = importlib.util.spec_from_file_location("t12", os.path.join(ROOT, "12_train_yrg.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["t12"] = mod
    spec.loader.exec_module(mod)
    return mod.Net


Net = _load_net()


def load(tag, n_classes):
    ck = torch.load(os.path.join(OUT, f"cls_{tag}.pt"), map_location="cpu", weights_only=False)
    m = Net(n_classes)
    m.load_state_dict(ck["state_dict"])
    m.eval()
    if torch.cuda.is_available():
        m.cuda()
    return m, ck["img"]


def predict(model, frames, img, dev):
    res = []
    with torch.no_grad():
        for k in range(0, len(frames), 16):
            batch = []
            for f in frames[k:k + 16]:
                y = fpga_rg_gray(f)
                y = cv2.resize(y, (img, img), interpolation=cv2.INTER_AREA).astype(np.float32)
                x = (y[:, :, ::-1] / 255.0 - 0.5) / 0.5
                batch.append(np.ascontiguousarray(x.transpose(2, 0, 1)))
            t = torch.from_numpy(np.stack(batch)).float().to(dev)
            p = torch.softmax(model(t), 1).cpu().numpy()
            res += [(int(a), float(p[i][a]), p[i]) for i, a in enumerate(p.argmax(1))]
    return res


def grab(path, n=120, step=None):
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


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    hold = vids[6:]

    for tag, n_cls, names in [("yrg2", 2, ["1", "2"]), ("yrg3", 3, ["1", "2", "other"])]:
        p = os.path.join(OUT, f"cls_{tag}.pt")
        if not os.path.exists(p):
            print(f"[{tag}] 权重不存在，跳过")
            continue
        model, img = load(tag, n_cls)
        print(f"\n{'='*64}\n[{tag}]  {n_cls} 类 {names}\n{'='*64}")

        for path, label in VIDEOS:
            frames = grab(path, n=120, step=3)
            res = predict(model, frames, img, dev)
            idx = names.index(label)
            acc = np.mean([a == idx for a, _, _ in res])
            conf = np.array([c for _, c, _ in res])
            print(f"  video{label}: n={len(res)} 准确率={acc*100:.1f}% "
                  f"置信度 min={conf.min():.3f} p5={np.percentile(conf,5):.3f} "
                  f"mean={conf.mean():.3f}")

        print(f"  --- 留出官方视频（{len(hold)} 个，训练时未见过）---")
        hold_confs = []
        hold_rej = 0
        for v in hold:
            frames = grab(os.path.join(OFFICIAL, v), n=30, step=None)
            res = predict(model, frames, img, dev)
            other_idx = names.index("other") if "other" in names else -1
            n_other = sum(1 for a, _, _ in res if a == other_idx)
            conf = np.array([c for _, c, _ in res])
            hold_confs.append(conf)
            rej = n_other / len(res)
            print(f"    {v[-16:]:20s} n={len(res):3d} 判other={n_other:3d} ({rej*100:5.1f}%) "
                  f"平均置信={conf.mean():.3f}")

        allc = np.concatenate(hold_confs)
        print(f"  留出集整体：平均置信度 {allc.mean():.3f}  (>0.9 的帧占比 {np.mean(allc>0.9)*100:.1f}%)")


if __name__ == "__main__":
    main()
