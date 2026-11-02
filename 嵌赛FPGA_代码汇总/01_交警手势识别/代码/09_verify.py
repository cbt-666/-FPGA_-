import os

import cv2
import numpy as np
import onnxruntime as ort
import torch

import nn_def

ROOT = os.path.dirname(os.path.abspath(__file__))
ONNX = os.path.join(ROOT, "out", "cls2_small.onnx")
CKPT = os.path.join(ROOT, "out", "cls2_small.pt")
OFFICIAL = r"C:/Users/cbt/Desktop/嵌赛FPGA/官方资料/紫光同创杯赛题三参考素材"


def prep(img, s):
    img = cv2.resize(img, (s, s), interpolation=cv2.INTER_AREA).astype(np.float32)
    x = (img[:, :, ::-1] / 255.0 - 0.5) / 0.5
    return np.ascontiguousarray(x.transpose(2, 0, 1))[None]


def onnx_predict(sess, imgs, s):
    out = []
    for f in imgs:
        logits = sess.run(None, {"images": prep(f, s)})[0][0]
        e = np.exp(logits - logits.max())
        out.append((int(np.argmax(logits)), float((e / e.sum())[np.argmax(logits)])))
    return out


def grab(path, step=10, limit=30):
    cap = cv2.VideoCapture(path)
    fr, i = [], 0
    while len(fr) < limit:
        ok, f = cap.read()
        if not ok:
            break
        if i % step == 0:
            fr.append(f)
        i += 1
    cap.release()
    return fr


def main():
    ck = torch.load(CKPT, map_location="cpu", weights_only=False)
    IMG, classes = ck["img"], ck["classes"]
    model = nn_def.build()
    model.load_state_dict(ck["state_dict"])
    model.eval()
    sess = ort.InferenceSession(ONNX, providers=["CPUExecutionProvider"])

    print("=== 1) ONNX vs PyTorch 一致性（训练用 40 帧）===")
    agree = total = 0
    for part, vid in [("video1", r"D:/QQ20261031-194504.mp4"), ("video2", r"D:/QQ20261031-195747.mp4")]:
        for f in grab(vid, step=18, limit=20):
            x = torch.from_numpy(prep(f, IMG))
            with torch.no_grad():
                pt = int(model(x).argmax(1))
            on = onnx_predict(sess, [f], IMG)[0][0]
            agree += int(pt == on)
            total += 1
    print(f"   一致 {agree}/{total} = {agree/total*100:.1f}%")

    print("\n=== 2) 目标视频全片（非训练帧）===")
    for name, vid, label in [("video1", r"D:/QQ20261031-194504.mp4", "1"),
                             ("video2", r"D:/QQ20261031-195747.mp4", "2")]:
        frames = grab(vid, step=3, limit=500)
        res = onnx_predict(sess, frames, IMG)
        acc = sum(classes[p] == label for p, _ in res) / len(res)
        conf = np.mean([c for _, c in res])
        print(f"   {name}: n={len(frames)} acc={acc*100:.1f}% 平均置信度={conf:.3f}")

    print("\n=== 3) 边界测试：无关视频（模型不该认，只应有低置信度）===")
    others = []
    for f in sorted(os.listdir(OFFICIAL)):
        if f.lower().endswith((".mp4", ".avi", ".mov")):
            others.append((f"官方/{f}", os.path.join(OFFICIAL, f)))
    for name, path in others:
        frames = grab(path, step=20, limit=15)
        if not frames:
            continue
        res = onnx_predict(sess, frames, IMG)
        n1 = sum(1 for p, _ in res if classes[p] == "1")
        conf = np.mean([c for _, c in res])
        print(f"   {name[:34]:36s} 判1={n1:2d}/{len(res):2d}  平均置信度={conf:.3f}")


if __name__ == "__main__":
    main()
