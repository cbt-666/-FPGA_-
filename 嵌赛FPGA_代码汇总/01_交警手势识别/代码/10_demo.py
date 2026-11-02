import os

import cv2
import numpy as np
import onnxruntime as ort

ROOT = os.path.dirname(os.path.abspath(__file__))
ONNX = os.path.join(ROOT, "out", "cls2_small.onnx")
IMG = 64
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]
COLORS = {"1": (60, 200, 60), "2": (60, 120, 240)}


def prep(img, s):
    im = cv2.resize(img, (s, s), interpolation=cv2.INTER_AREA).astype(np.float32)
    x = (im[:, :, ::-1] / 255.0 - 0.5) / 0.5
    return np.ascontiguousarray(x.transpose(2, 0, 1))[None]


def main():
    sess = ort.InferenceSession(ONNX, providers=["CPUExecutionProvider"])
    tiles = []
    for path, label in VIDEOS:
        cap = cv2.VideoCapture(path)
        frames, i = [], 0
        while len(frames) < 8:
            ok, f = cap.read()
            if not ok:
                break
            if i % 35 == 0:
                frames.append(f)
            i += 1
        cap.release()
        for f in frames:
            logits = sess.run(None, {"images": prep(f, IMG)})[0][0]
            e = np.exp(logits - logits.max())
            prob = e / e.sum()
            p = int(np.argmax(prob))
            name = str(p + 1)
            ok = name == label
            h = 210
            w = int(f.shape[1] * h / f.shape[0])
            im = cv2.resize(f, (w, h))
            bar = np.zeros((34, w, 3), np.uint8)
            bar[:] = COLORS[name]
            txt = f"GT:{label}  PRED:{name}  {prob[p]:.2f}" + ("" if ok else "  X")
            cv2.putText(bar, txt, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(np.vstack([im, bar]))
    wmax = max(t.shape[1] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, 0, 0, wmax - t.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for t in tiles]
    rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
    wrow = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 0, 0, wrow - r.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for r in rows]
    sheet = np.vstack(rows)
    out = os.path.join(ROOT, "out", "demo_predictions.jpg")
    ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 90])
    open(out, "wb").write(buf.tobytes())
    print("saved", out, sheet.shape)


if __name__ == "__main__":
    main()
