"""先量一下：这两个视频经过 FPGA 红绿灰度化后，红/绿像素各占多少。

如果某一类的"纯红/纯绿像素占比"和另一类差很多，模型就会靠颜色抄近道，
在别的场景上必然失效。
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, r"D:/ADAS_Datasets/_code")
from rg_gray_fpga_exact import fpga_rg_gray, stats

ROOT = os.path.dirname(os.path.abspath(__file__))
VIDEOS = [(r"D:/QQ20261031-194504.mp4", "1"), (r"D:/QQ20261031-195747.mp4", "2")]


def main():
    for path, label in VIDEOS:
        cap = cv2.VideoCapture(path)
        rs, gs = [], []
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            if i % 15 == 0:
                r, g = stats(f)
                rs.append(r)
                gs.append(g)
            i += 1
        cap.release()
        print(f"video{label}: n={len(rs)} 红像素占比 mean={np.mean(rs):.3f}% max={np.max(rs):.2f}% "
              f"| 绿像素占比 mean={np.mean(gs):.3f}% max={np.max(gs):.2f}%")

    # 存一组 before/after 对比图
    tiles = []
    for path, label in VIDEOS:
        cap = cv2.VideoCapture(path)
        i = 0
        got = 0
        while got < 3:
            ok, f = cap.read()
            if not ok:
                break
            if i % 90 == 0:
                tr = fpga_rg_gray(f)
                h = 240
                a = cv2.resize(f, (int(f.shape[1] * h / f.shape[0]), h))
                b = cv2.resize(tr, (a.shape[1], h))
                cv2.putText(a, f"RGB v{label}", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(b, f"YRG v{label}", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                tiles.append(np.hstack([a, b]))
                got += 1
            i += 1
        cap.release()
    wmax = max(t.shape[1] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, 0, 0, wmax - t.shape[1], cv2.BORDER_CONSTANT, value=(30, 30, 30)) for t in tiles]
    sheet = np.vstack(tiles)
    out = os.path.join(ROOT, "out", "yrg_preview.jpg")
    ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 90])
    open(out, "wb").write(buf.tobytes())
    print("saved", out, sheet.shape)


if __name__ == "__main__":
    main()
