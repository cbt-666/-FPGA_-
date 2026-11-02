"""v1 vs v2 全量对比 + 两种黑边裁剪算法对比。

裁剪算法：
  C1 亮度阈值（旧）：列均值 > 12 视为内容
  C2 连通段方差（新）：找最长的"非整列纯黑"连通段，再从两端按黑列剥离
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
OFFICIAL = r"C:/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]
PASTED = [("paste_1.png", "1", "video1"), ("paste_0.png", "2", "video2")]


def _net_mod(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


t12 = _net_mod(os.path.join(ROOT, "12_train_yrg.py"), "t12")
t20 = _net_mod(os.path.join(ROOT, "20_train_v2_letterbox.py"), "t20")


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def crop_threshold(img, thresh=12):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    nz = np.where(g.mean(axis=0) > thresh)[0]
    return img[:, nz[0]:nz[-1] + 1] if len(nz) else img


def crop_variance(img, black_thresh=6.0):
    """列标准差极小 -> 纯色列（黑边）。取最长连续内容段，再两端剥离。"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    col_std = g.std(axis=0)
    content = col_std > black_thresh
    # 找最长连续 True 段
    best_s = best_e = -1
    s = None
    for i, v in enumerate(content):
        if v and s is None:
            s = i
        elif not v and s is not None:
            if i - s > best_e - best_s:
                best_s, best_e = s, i
            s = None
    if s is not None and len(content) - s > best_e - best_s:
        best_s, best_e = s, len(content)
    if best_s < 0:
        return img
    return img[:, best_s:best_e]


def load_model(module, tag, n_cls):
    p = os.path.join(OUT, f"cls_{tag}.pt")
    if not os.path.exists(p):
        return None, None
    ck = torch.load(p, map_location="cpu", weights_only=False)
    m = module.Net(n_cls)
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
    return names[k], float(p[k])


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


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    MODELS = [
        ("v1-3类", t12, "yrg3", 3, ["1", "2", "other"]),
        ("v2-3类", t20, "v2yrg3", 3, ["1", "2", "other"]),
        ("v1-2类", t12, "yrg2", 2, ["1", "2"]),
        ("v2-2类", t20, "v2yrg2", 2, ["1", "2"]),
    ]
    loaded = []
    for label, mod, tag, nc, names in MODELS:
        m, im = load_model(mod, tag, nc)
        if m is not None:
            loaded.append((label, m, im, names))
            print(f"loaded {label}")

    # ---------- 1. 原始帧回归 ----------
    print(f"\n{'='*78}\n1) 原始帧（不该退化）\n{'='*78}")
    for path, gt in VIDEOS:
        frames = grab(path, 120, step=3)
        print(f"\n  video{gt} n={len(frames)}")
        for label, m, im, names in loaded:
            accs = [predict(m, f, im, dev, names)[0] == gt for f in frames]
            confs = [predict(m, f, im, dev, names)[1] for f in frames]
            print(f"    {label:8s} acc={np.mean(accs)*100:5.1f}%  平均置信={np.mean(confs):.4f}")

    # ---------- 2. 截图（三种裁剪） ----------
    print(f"\n{'='*78}\n2) 真实截图（原样 / 阈值裁边 / 方差裁边）\n{'='*78}")
    for fn, gt, tag in PASTED:
        raw = imread_u(os.path.join(ROOT, fn))
        c1 = crop_threshold(raw)
        c2 = crop_variance(raw)
        print(f"\n  {fn} ({tag}, 期望类{gt})  原={raw.shape[1]}x{raw.shape[0]}")
        print(f"    阈值法 -> {c1.shape[1]}x{c1.shape[0]}   方差法 -> {c2.shape[1]}x{c2.shape[0]}")
        for vn, im_in in [("原样", raw), ("阈值裁", c1), ("方差裁", c2)]:
            parts = []
            for label, m, im, names in loaded:
                pred, conf = predict(m, im_in, im, dev, names)
                mark = "OK" if pred == gt else "NG"
                parts.append(f"{label}:{mark} {pred}({conf:.2f})")
            print(f"    {vn:6s}  " + "  ".join(parts))

    # ---------- 3. 合成黑边增强测试 ----------
    print(f"\n{'='*78}\n3) 合成黑边（画面缩到画布宽度的 50% / 33%）\n{'='*78}")

    def make_lb(img, scale_w):
        h, w = img.shape[:2]
        cw, ch = int(w / scale_w), h + 60
        canvas = np.zeros((ch, cw, 3), np.uint8)
        x, y = (cw - w) // 2, 30
        canvas[y:y + h, x:x + w] = img
        return canvas

    for path, gt in VIDEOS:
        f = grab(path, 20)[10]
        for sw in (0.5, 0.33):
            im_in = make_lb(f, sw)
            parts = []
            for label, m, im, names in loaded:
                pred, conf = predict(m, im_in, im, dev, names)
                mark = "OK" if pred == gt else "NG"
                parts.append(f"{label}:{mark}({conf:.2f})")
            print(f"  video{gt} 画面占比{int(sw*100)}%  " + "  ".join(parts))

    # ---------- 4. 留出官方视频（拒识） ----------
    print(f"\n{'='*78}\n4) 留出官方视频拒识（训练未见）\n{'='*78}")
    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    for v in vids[6:]:
        frames = grab(os.path.join(OFFICIAL, v), 30)
        parts = []
        for label, m, im, names in loaded:
            if "other" not in names:
                parts.append(f"{label}:N/A")
                continue
            preds = [predict(m, f, im, dev, names)[0] for f in frames]
            rej = preds.count("other") / len(preds)
            parts.append(f"{label}:{rej*100:.0f}%")
        print(f"  {v[-18:]:20s} " + "  ".join(parts))


if __name__ == "__main__":
    main()
