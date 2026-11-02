"""测试用户截图对应的画面：先做 FPGA 红绿灰度化，再喂给两模型。

三种输入逐级加难度：
  A. 原始帧（视频原生画面）
  B. 截图模拟（黑边 letterbox + 顶部/底部播放器区域）
  C. 全屏截图模拟（更宽的纯黑边）
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

CASES = [
    # (视频, 期望类, 取帧时间秒)
    (r"D:/QQ20261031-194504.mp4", "1", 6.0),
    (r"D:/QQ20261031-195747.mp4", "2", 3.0),
]


def frame_at(path, sec):
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(sec * fps))
    ok, f = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"read fail {path}@{sec}")
    return f


def letterbox(img, canvas_w, canvas_h, top=0, bottom=0):
    """把画面居中放进黑色画布，模拟截图里的黑边 + 播放器区域。"""
    canvas = np.zeros((canvas_h, canvas_w, 3), np.uint8)
    ch = canvas_h - top - bottom
    sc = min(canvas_w / img.shape[1], ch / img.shape[0])
    nw, nh = int(img.shape[1] * sc), int(img.shape[0] * sc)
    small = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    x = (canvas_w - nw) // 2
    y = top + (ch - nh) // 2
    canvas[y:y + nh, x:x + nw] = small
    return canvas


def load(tag, n_cls):
    ck = torch.load(os.path.join(OUT, f"cls_{tag}.pt"), map_location="cpu", weights_only=False)
    m = t12.Net(n_cls)
    m.load_state_dict(ck["state_dict"])
    m.eval()
    if torch.cuda.is_available():
        m.cuda()
    return (m, ck["img"])


def predict(model, bgr, img, dev, names):
    """严格走板上链路：FPGA 红绿灰度化 -> resize -> 归一化。"""
    yrg = fpga_rg_gray(bgr)
    y = cv2.resize(yrg, (img, img), interpolation=cv2.INTER_AREA).astype(np.float32)
    x = (y[:, :, ::-1] / 255.0 - 0.5) / 0.5
    x = np.ascontiguousarray(x.transpose(2, 0, 1))[None]
    with torch.no_grad():
        p = torch.softmax(model(torch.from_numpy(x).float().to(dev)), 1).cpu().numpy()[0]
    k = int(p.argmax())
    return names[k], float(p[k]), p, yrg


def green_share(bgr):
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    b = bgr[:, :, 0].astype(np.int32)
    m = (g >= 56) & (g > r + 16) & (g > b + 16)
    return 100.0 * m.sum() / m.size


def main():
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    models = {"yrg2": (load("yrg2", 2), ["1", "2"]),
              "yrg3": (load("yrg3", 3), ["1", "2", "other"])}

    tiles = []
    for path, gt, sec in CASES:
        f = frame_at(path, sec)
        h, w = f.shape[:2]
        variants = [
            ("A 原始帧", f),
            ("B 截图模拟(黑边+UI)", letterbox(f, int(w * 1.8), int(h * 1.16), top=int(h * 0.06), bottom=int(h * 0.10))),
            ("C 全屏截图模拟", letterbox(f, int(w * 2.2), int(h * 1.4))),
        ]

        print(f"\n{'='*70}")
        print(f"视频={os.path.basename(path)}  原图={w}x{h}  时间={sec}s  期望=类{gt}")
        print(f"{'='*70}")

        for vname, im in variants:
            print(f"  [{vname}] 尺寸={im.shape[1]}x{im.shape[0]}  绿像素占比={green_share(im):.2f}%")
            for mtag, ((model, ck_img), names) in models.items():
                pred, conf, probs, yrg = predict(model, im, ck_img, dev, names)
                # 重新取 img（来自 ckpt）
                ok = "OK " if pred == gt else "NG "
                pstr = " ".join(f"{n}={v:.3f}" for n, v in zip(names, probs))
                print(f"      {mtag}: {ok} pred={pred:<5s} conf={conf:.4f}   [{pstr}]")
                if mtag == "yrg3" and vname.startswith("A"):
                    tiles.append((f"{vname}", im, yrg, f"{gt}", pred, conf))
                elif mtag == "yrg3" and vname.startswith("B"):
                    tiles.append((vname, im, yrg, gt, pred, conf))

    # 保存可视化
    out_rows = []
    for title, im, yrg, gt, pred, conf in tiles:
        h = 200
        a = cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h))
        b = cv2.resize(yrg, (a.shape[1], h))
        pair = np.hstack([a, b])
        bar = np.zeros((34, pair.shape[1], 3), np.uint8)
        ok = pred == gt
        bar[:] = (60, 200, 60) if ok else (50, 50, 220)
        cv2.putText(bar, f"{title}  GT:{gt} -> {pred}  {conf:.3f}", (6, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        out_rows.append(np.vstack([pair, bar]))
    wmax = max(r.shape[1] for r in out_rows)
    out_rows = [cv2.copyMakeBorder(r, 0, 0, 0, wmax - r.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for r in out_rows]
    sheet = np.vstack(out_rows)
    p = os.path.join(OUT, "test_pasted_frames.jpg")
    ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
    open(p, "wb").write(buf.tobytes())
    print(f"\nsaved {p} {sheet.shape}")


if __name__ == "__main__":
    main()
