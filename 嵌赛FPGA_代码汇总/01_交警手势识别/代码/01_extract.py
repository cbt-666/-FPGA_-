import os
import shutil

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "raw")
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "video1", 0), (r"D:/QQ20261031-195747.mp4", "video2", 1)]
PER_VIDEO = 20


def imwrite_safe(path, img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError(f"encode failed: {path}")
    with open(path, "wb") as f:
        f.write(buf.tobytes())


def main():
    if os.path.isdir(RAW):
        shutil.rmtree(RAW)
    for path, tag, _ in VIDEOS:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open {path}")
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        targets = set(np.linspace(0, n - 2, PER_VIDEO).round().astype(int).tolist())
        out = os.path.join(RAW, tag)
        os.makedirs(out, exist_ok=True)
        i = 0
        k = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i in targets:
                imwrite_safe(os.path.join(out, f"{tag}_{k:02d}_f{i:04d}.jpg"), frame)
                k += 1
            i += 1
        cap.release()
        print(f"{tag}: total_frames={i} saved={k} size={int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")


if __name__ == "__main__":
    main()
