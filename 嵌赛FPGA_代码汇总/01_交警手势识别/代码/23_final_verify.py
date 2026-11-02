"""最终验证：v2 模型 + 暗像素占比裁边，全场景回归。"""
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


def _mod(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


t12 = _mod(os.path.join(ROOT, "12_train_yrg.py"), "t12")
t20 = _mod(os.path.join(ROOT, "20_train_v2_letterbox.py"), "t20")


def rd(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def crop_black_bars(img, dark=20, frac_thresh=0.60):
    """从两侧向内剥离黑边：整列超过 60% 像素暗于 20 即视为黑边。"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    frac = (g < dark).mean(axis=0)
    w = len(frac)
    l = 0
    while l < w - 1 and frac[l] > frac_thresh:
        l += 1
    r = w - 1
    while r > l and frac[r] > frac_thresh:
        r -= 1
    return img[:, l:r + 1]


def load(mod, tag, nc):
    ck = torch.load(os.path.join(OUT, f"cls_{tag}.pt"), map_location="cpu", weights_only=False)
    m = mod.Net(nc)
    m.load_state_dict(ck["state_dict"])
    m.eval()
    if torch.cuda.is_available():
        m.cuda()
    return m, ck["img"]


def pred(model, bgr, img, dev, names):
    yrg = fpga_rg_gray(bgr)
    y = cv2.resize(yrg, (img, img), interpolation=cv2.INTER_AREA).astype(np.float32)
    x = (y[:, :, ::-1] / 255.0 - 0.5) / 0.5
    with torch.no_grad():
        p = torch.softmax(model(torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1)))[None]
                                .float().to(dev)), 1).cpu().numpy()[0]
    return names[int(p.argmax())], float(p.max()), yrg


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
    M = [("v1-3类", t12, *load(t12, "yrg3", 3), ["1", "2", "other"]),
         ("v2-3类", t20, *load(t20, "v2yrg3", 3), ["1", "2", "other"]),
         ("v1-2类", t12, *load(t12, "yrg2", 2), ["1", "2"]),
         ("v2-2类", t20, *load(t20, "v2yrg2", 2), ["1", "2"])]

    print("=" * 80)
    print("1) 真实截图：原样 vs 暗占比裁边（v2 模型 + 裁边，应全部正确）")
    print("=" * 80)
    tiles = []
    for fn, gt, tag in PASTED:
        raw = rd(os.path.join(ROOT, fn))
        cut = crop_black_bars(raw)
        print(f"\n  {fn} ({tag} 期望类{gt})")
        print(f"    原 {raw.shape[1]}x{raw.shape[0]}  ->  裁边后 {cut.shape[1]}x{cut.shape[0]}")
        for vn, im_in in [("原样  ", raw), ("裁黑边", cut)]:
            parts = []
            for label, _, m, im, names in M:
                p, c, _ = pred(m, im_in, im, dev, names)
                parts.append(f"{label}:{'OK' if p == gt else 'NG'} {p}({c:.2f})")
            print(f"    {vn}  " + "   ".join(parts))
        p, c, yrg = pred(M[1][2], cut, M[1][3], dev, M[1][4])
        tiles.append((f"{fn} 裁边后 -> {p} {c:.3f}", cut, yrg, gt, p, c))

    print("\n" + "=" * 80)
    print("2) 合成黑边回归（v2 应全对，v1 已知会错）")
    print("=" * 80)

    def lb(img, sw):
        h, w = img.shape[:2]
        cw, ch = int(w / sw), h + 60
        c = np.zeros((ch, cw, 3), np.uint8)
        c[30:30 + h, (cw - w) // 2:(cw - w) // 2 + w] = img
        return c

    for path, gt in VIDEOS:
        f = grab(path, 20)[10]
        for sw in (0.5, 0.33):
            im_in = lb(f, sw)
            row = []
            for label, _, m, im, names in M:
                p, c, _ = pred(m, im_in, im, dev, names)
                row.append(f"{label}:{'OK' if p == gt else 'NG'}({c:.2f})")
            print(f"  video{gt} 画面{int(sw*100)}%   " + "  ".join(row))

    print("\n" + "=" * 80)
    print("3) 留出官方视频拒识（v2-3类 应与 v1 持平或更好）")
    print("=" * 80)
    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    for v in vids[6:]:
        frames = grab(os.path.join(OFFICIAL, v), 30)
        row = []
        for label, _, m, im, names in M:
            if "other" not in names:
                row.append(f"{label}:N/A")
                continue
            rej = np.mean([pred(m, f, im, dev, names)[0] == "other" for f in frames])
            row.append(f"{label}:{rej*100:.0f}%")
        print(f"  {v[-18:]:20s} " + "  ".join(row))

    rows = []
    for title, im, yrg, gt, p, c in tiles:
        h = 300
        a = cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h))
        b = cv2.resize(yrg, (a.shape[1], h))
        pair = np.hstack([a, b])
        bar = np.zeros((36, pair.shape[1], 3), np.uint8)
        bar[:] = (60, 200, 60) if p == gt else (50, 50, 220)
        cv2.putText(bar, f"{title}", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        rows.append(np.vstack([pair, bar]))
    wmax = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 0, 0, wmax - r.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for r in rows]
    sheet = np.vstack(rows)
    o = os.path.join(OUT, "final_pasted.jpg")
    ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
    open(o, "wb").write(buf.tobytes())
    print(f"\nsaved {o}")


if __name__ == "__main__":
    main()
