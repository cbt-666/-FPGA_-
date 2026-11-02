import os

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "raw")
THUMB_W = 240


def load_grid(tag):
    files = sorted(f for f in os.listdir(os.path.join(RAW, tag)) if f.endswith(".jpg"))
    cells = []
    for f in files:
        img = cv2.imdecode(np.fromfile(os.path.join(RAW, tag, f), dtype=np.uint8), cv2.IMREAD_COLOR)
        h = int(img.shape[0] * THUMB_W / img.shape[1])
        cells.append(cv2.resize(img, (THUMB_W, h)))
    hmax = max(c.shape[0] for c in cells)
    cells = [cv2.copyMakeBorder(c, 0, hmax - c.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(40, 40, 40)) for c in cells]
    rows = []
    for r in range(0, len(cells), 5):
        chunk = cells[r:r + 5]
        while len(chunk) < 5:
            chunk.append(np.full_like(cells[0], 40))
        rows.append(np.hstack(chunk))
    return np.vstack(rows)


grids = [load_grid(t) for t in ("video1", "video2")]
w = max(g.shape[1] for g in grids)
pad = [cv2.copyMakeBorder(g, 0, 0, 0, w - g.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for g in grids]
sheet = np.vstack([pad[0], np.full((12, w, 3), 255, np.uint8), pad[1]])

out = os.path.join(ROOT, "contact_sheet.jpg")
ok, buf = cv2.imencode(".jpg", sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
open(out, "wb").write(buf.tobytes())
print("saved", out, sheet.shape)
