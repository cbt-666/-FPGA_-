"""实测用户粘贴的两张播放器截图，走 FPGA 红绿灰度化 -> 模型。

变体：
  A. 原样（含左右黑边 + 上下播放器 UI）
  B. 自动裁掉左右黑边
  C. 裁黑边 + 再裁上下 UI 区域
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

spec = importlib.util.spec_from_file_location("t12", os.path.join(ROOT, "12_train_yrg.py"))
t12 = importlib.util.module_from_spec(spec)
sys.modules["t12"] = t12
spec.loader.exec_module(t12)

CASES = [("paste_0.png", "2", "video2 QQ20261031-195747"),
         ("paste_1.png", "1", "video1 QQ20261031-194504")]


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def crop_side_bars(img, thresh=12):
    """裁掉左右纯黑竖条。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    col_mean = gray.mean(axis=0)
    nz = np.where(col_mean > thresh)[0]
    if len(nz) == 0:
        return img, (0, img.shape[1])
    return img[:, nz[0]:nz[-1] + 1], (int(nz[0]), int(nz[-1] + 1))


def crop_ui(img):
    """在已裁左右黑边的图上，再裁掉顶部标题栏与底部控制条。"""
    h = img.shape[0]
    return img[int(h * 0.045):int(h * 0.885)]


def load(tag, n_cls):
    ck = torch.load(os.path.join(OUT, f"cls_{tag}.pt"), map_location="cpu", weights_only=False)
    m = t12.Net(n_cls)
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
    return names[k], float(p[k]), p, yrg


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    m2, img2 = load("yrg2", 2)
    m3, img3 = load("yrg3", 3)
    N2, N3 = ["1", "2"], ["1", "2", "other"]

    tiles = []
    for fn, gt, desc in CASES:
        p = os.path.join(ROOT, fn)
        raw = imread_u(p)
        if raw is None:
            print(f"读不到 {p}")
            continue
        side, (c0, c1) = crop_side_bars(raw)
        sidei = crop_ui(side)

        variants = [("A 原样(含黑边+UI)", raw),
                    ("B 裁左右黑边", side),
                    ("C 裁黑边+裁UI", sidei)]

        print(f"\n{'='*72}")
        print(f"{desc}   期望=类{gt}   原图={raw.shape[1]}x{raw.shape[0]}")
        print(f"左右黑边检测: 内容区 x=[{c0}, {c1}]  宽度={c1-c0}")
        print(f"{'='*72}")

        for vn, im in variants:
            print(f"\n  [{vn}]  {im.shape[1]}x{im.shape[0]}")
            for tag, model, img, names in [("yrg2", m2, img2, N2), ("yrg3", m3, img3, N3)]:
                pred, conf, probs, yrg = predict(model, im, img, dev, names)
                ok = "OK " if pred == gt else "NG "
                ps = "  ".join(f"{n}={v:.3f}" for n, v in zip(names, probs))
                print(f"      {tag}: {ok} pred={pred:<5s} conf={conf:.4f}   [{ps}]")
                if tag == "yrg3":
                    tiles.append((f"{os.path.basename(fn)} {vn}", im, yrg, gt, pred, conf))

    rows = []
    for title, im, yrg, gt, pred, conf in tiles:
        h = 260
        a = cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h))
        b = cv2.resize(yrg, (a.shape[1], h))
        pair = np.hstack([a, b])
        bar = np.zeros((36, pair.shape[1], 3), np.uint8)
        bar[:] = (60, 200, 60) if pred == gt else (50, 50, 220)
        cv2.putText(bar, f"{title}   GT:{gt} -> {pred}  {conf:.3f}", (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        rows.append(np.vstack([pair, bar]))
    wmax = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 0, 0, wmax - r.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for r in rows]
    sheet = np.vstack(rows)
    o = os.path.join(OUT, "test_pasted_real.jpg")
    ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
    open(o, "wb").write(buf.tobytes())
    print(f"\nsaved {o} {sheet.shape}")


if __name__ == "__main__":
    main()
