"""最终演示图：YRG 输入 + 3 类模型预测 + 置信度 + other 拒识。"""
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
NAMES = ["1", "2", "other"]
COLORS = {"1": (60, 200, 60), "2": (60, 120, 240), "other": (140, 140, 140)}

spec = importlib.util.spec_from_file_location("t12", os.path.join(ROOT, "12_train_yrg.py"))
t12 = importlib.util.module_from_spec(spec)
sys.modules["t12"] = t12
spec.loader.exec_module(t12)


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


def tile(frame, pred, conf, gt):
    h = 220
    w = int(frame.shape[1] * h / frame.shape[0])
    a = cv2.resize(frame, (w, h))
    y = fpga_rg_gray(frame)
    b = cv2.resize(y, (w, h))
    cv2.putText(a, "RGB", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(b, "YRG (FPGA)", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    bar = np.zeros((36, w * 2, 3), np.uint8)
    bar[:] = COLORS[pred]
    ok = pred == gt
    txt = f"GT:{gt}  PRED:{pred}  {conf:.3f}" + ("" if ok else "  X")
    cv2.putText(bar, txt, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([np.hstack([a, b]), bar])


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ck = torch.load(os.path.join(OUT, "cls_yrg3.pt"), map_location="cpu", weights_only=False)
    model = t12.Net(3)
    model.load_state_dict(ck["state_dict"])
    model.eval()
    model.to(dev)
    img = ck["img"]

    tiles = []
    sources = [(r"D:/QQ20261031-194504.mp4", "1", 4), (r"D:/QQ20261031-195747.mp4", "2", 4)]
    vids = sorted(f for f in os.listdir(OFFICIAL) if f.lower().endswith((".mp4", ".avi", ".mov")))
    for v in vids[6:]:
        sources.append((os.path.join(OFFICIAL, v), "other", 2))

    for path, gt, k in sources:
        frames = grab(path, k, step=max(1, int(400 / max(k, 1))))
        for f in frames:
            y = fpga_rg_gray(cv2.resize(f, (img * 2, img * 2), interpolation=cv2.INTER_AREA))
            y = cv2.resize(y, (img, img), interpolation=cv2.INTER_AREA).astype(np.float32)
            x = (y[:, :, ::-1] / 255.0 - 0.5) / 0.5
            t = torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1)))[None].float().to(dev)
            with torch.no_grad():
                p = torch.softmax(model(t), 1).cpu().numpy()[0]
            tiles.append(tile(f, NAMES[int(p.argmax())], float(p.max()), gt))

    wmax = max(t.shape[1] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, 0, 0, wmax - t.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for t in tiles]
    rows = [np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)]
    wrow = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 0, 0, wrow - r.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for r in rows]
    sheet = np.vstack(rows)
    out = os.path.join(OUT, "demo_yrg3.jpg")
    ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
    open(out, "wb").write(buf.tobytes())
    print("saved", out, sheet.shape)


if __name__ == "__main__":
    main()
