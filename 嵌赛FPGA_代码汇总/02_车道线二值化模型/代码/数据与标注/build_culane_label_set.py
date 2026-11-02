#!/usr/bin/env python3
"""生成 CULane 标注集 —— 供浏览器标注工具使用

CULane 已给出每条车道线的精确坐标。本脚本：
  1. 取近处 y=580 那一列，找画面中心左右最近的两条线
     -> 自车车道的 左边界(L) / 右边界(R)
  2. 把这两条线画在图上（粗亮线 + 端点标记），用户只需判断虚实
  3. 输出 图片 + manifest.json 给标注工具

挑选策略：只保留左右边界都检测到、且长度足够的样本，保证标注有效。
"""
import zipfile
import numpy as np
import cv2
import os
import json
import sys

CULANE = r'D:\ADAS_Datasets\06_车道线_lane\_raw_culane.zip'
OUTDIR = r'D:\ADAS_Datasets\06_车道线_lane\_label_set'
TARGET = int(sys.argv[1]) if len(sys.argv) > 1 else 400
MAXSCAN = TARGET * 8          # 最多扫这么多找候选


def imwrite_u(p, img, q=88):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


def parse_lines(txt):
    out = []
    for line in txt.strip().split('\n'):
        v = line.split()
        if len(v) >= 6:
            p = np.array([float(x) for x in v]).reshape(-1, 2)
            out.append(p[np.argsort(p[:, 1])])
    return out


def x_at(line, y):
    ys = line[:, 1]
    if y < ys[0] or y > ys[-1]:
        return None
    return float(np.interp(y, ys, line[:, 0]))


def main():
    z = zipfile.ZipFile(CULANE)
    labels = sorted(n for n in z.namelist() if n.endswith('.lines.txt'))
    print('CULane 标注 %d 条，目标采集 %d 张' % (len(labels), TARGET))
    os.makedirs(OUTDIR, exist_ok=True)

    step = max(1, len(labels) // MAXSCAN)
    cand = labels[::step]
    meta = []
    for i, lp in enumerate(cand):
        if len(meta) >= TARGET:
            break
        ip = lp.replace('.lines.txt', '.jpg')
        try:
            raw = z.read(ip)
            bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            txt = z.read(lp).decode('utf-8', errors='ignore')
        except Exception:
            continue
        if bgr is None:
            continue
        H, W = bgr.shape[:2]
        cx = W / 2.0
        lines = parse_lines(txt)

        # 在近处探测左右边界
        yprobe = H - 12
        cds = []
        for ln in lines:
            x = x_at(ln, yprobe)
            if x is None:
                if abs(ln[-1, 1] - (H - 1)) > 60:
                    continue
                x = ln[-1, 0]
            cds.append((x, ln))
        if not cds:
            continue
        Lc = max([c for c in cds if c[0] < cx], key=lambda t: t[0], default=None)
        Rc = min([c for c in cds if c[0] >= cx], key=lambda t: t[0], default=None)
        if Lc is None or Rc is None:
            continue
        # 两条线不能太近（否则不是自车车道）
        if Rc[0] - Lc[0] < W * 0.18:
            continue
        # 各自要有足够长的可见段
        ok = True
        for c in (Lc, Rc):
            m = c[1][(c[1][:, 1] >= H * 0.72) & (c[1][:, 1] <= H)]
            if len(m) < 8:
                ok = False
        if not ok:
            continue

        vis = bgr.copy()
        ycrop0 = int(H * 0.50)

        # 只在车道线上打「小十字刻度」，绝不沿线覆盖 —— 虚实要靠线本身判断
        for c, col, tag in ((Lc, (0, 0, 255), 'L'), (Rc, (0, 255, 0), 'R')):
            pts = c[1][(c[1][:, 1] >= ycrop0)]
            if len(pts) < 4:
                continue
            # 沿线的走向，取 3 个位置打刻度
            for frac in (0.18, 0.5, 0.82):
                k = int(len(pts) * frac)
                k = min(max(k, 1), len(pts) - 2)
                x, y = int(pts[k][0]), int(pts[k][1])
                # 刻度方向垂直于车道线走向
                dx = pts[k + 1][0] - pts[k - 1][0]
                dy = pts[k + 1][1] - pts[k - 1][1]
                n = max((dx * dx + dy * dy) ** 0.5, 1e-6)
                px, py = -dy / n, dx / n          # 垂线单位向量
                L = 16
                cv2.line(vis,
                         (int(x - px * L), int(y - py * L)),
                         (int(x + px * L), int(y + py * L)), (0, 0, 0), 7)
                cv2.line(vis,
                         (int(x - px * L), int(y - py * L)),
                         (int(x + px * L), int(y + py * L)), col, 4)
            # 标签放在最上端刻度外侧，不压线
            x0, y0 = int(pts[0][0]), int(pts[0][1])
            cv2.putText(vis, tag, (x0 - 40, y0 + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 8)
            cv2.putText(vis, tag, (x0 - 40, y0 + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, col, 4)

        # 只保留路面区域并放大（车道线在这里才看得清）
        crop = vis[ycrop0:, :]
        s = 1300 / crop.shape[1]
        crop = cv2.resize(crop, (1300, int(crop.shape[0] * s)),
                          interpolation=cv2.INTER_CUBIC)

        name = 'img_%04d.jpg' % len(meta)
        imwrite_u(os.path.join(OUTDIR, name), crop, 94)
        # CULane 的 basename 会在不同文件夹重复（都是 00000.jpg），
        # 必须存 zip 内的完整相对路径，否则训练时找不到对应原图
        meta.append({'img': name, 'src': ip})
        if len(meta) % 50 == 0:
            print('  已采集 %d' % len(meta))

    with open(os.path.join(OUTDIR, 'manifest.json'), 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=1)
    print()
    print('采集完成 %d 张 -> %s' % (len(meta), OUTDIR))


if __name__ == '__main__':
    main()
