#!/usr/bin/env python3
"""决定性实验：车道线虚实能否从像素自动判定？

思路：
  CULane 给每条车道线的逐 y 采样点 (x,y)。
  沿这条折线采样亮度：
    实线 → 亮度剖面连续
    虚线 → 亮/暗交替（有间隙）

关键问题不是「现在能不能测出来」，而是
  「过了 FPGA 那套 gray → 3x3中值 → 二值化 之后，间隙还在不在」
因为模型看到的只有处理后的图。

测三个版本：
  A. 原始彩色 → 灰度
  B. 灰度 → 3x3 中值        (FPGA 第 2 级)
  C. 灰度 → 中值 → 二值化    (FPGA 第 3 级, 取合适阈值)
"""
import zipfile, io, os, sys
import numpy as np
import cv2

CULANE = r'D:\ADAS_Datasets\06_车道线_lane\_raw_culane.zip'
OUT = r'D:\ADAS_Datasets\临时_队友资料'


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_u(p, img):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if ok:
        buf.tofile(p)
    return ok


def rgb2gray_fpga(bgr):
    """队友的 Y = (77R + 150G + 29B) >> 8"""
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def parse_lines(txt):
    """CULane: 每行 x1 y1 x2 y2 ... ，y 从 590 递减到 410"""
    out = []
    for line in txt.strip().split('\n'):
        v = line.split()
        if len(v) < 4:
            continue
        pts = np.array([float(x) for x in v]).reshape(-1, 2)
        out.append(pts)
    return out


def sample_profile(img, pts, half=1):
    """沿折线采样，每点取横向 ±half 的最大值（抗轻微错位）"""
    H, W = img.shape[:2]
    prof = []
    for x, y in pts:
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= yi < H):
            continue
        x0, x1 = max(0, xi - half), min(W, xi + half + 1)
        if x1 <= x0:
            continue
        prof.append(int(img[yi, x0:x1].max()))
    return np.array(prof, dtype=np.int32)


def gap_stats(prof, dark_ratio=0.75):
    """亮=线, 暗=间隙。返回 (长度, 暗段数, 最长暗段, 亮占比)"""
    if len(prof) < 8:
        return None
    thr = max(prof.max() * dark_ratio, prof.mean() * 0.6)
    seq = (prof >= thr).astype(int)
    # 去首尾暗
    nz = np.where(seq > 0)[0]
    if len(nz) < 5:
        return None
    seq = seq[nz[0]:nz[-1] + 1]
    runs = []
    cur = 0
    for v in seq:
        if v == 0:
            cur += 1
        else:
            if cur:
                runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    runs = [r for r in runs if r >= 1]
    return (len(seq), len(runs), max(runs) if runs else 0, seq.mean())


def main():
    z = zipfile.ZipFile(CULANE)
    names = z.namelist()
    labels = [n for n in names if n.endswith('.lines.txt')]
    labels.sort()
    print('CULane 标注总数:', len(labels))

    N = int(sys.argv[1]) if len(sys.argv) > 1 else 400
    step = max(1, len(labels) // N)
    pick = labels[::step][:N]
    print('抽样:', len(pick))

    recs = {'A_raw': [], 'B_median': [], 'C_bin': []}
    panels = []

    for i, lp in enumerate(pick):
        ip = lp.replace('.lines.txt', '.jpg')
        try:
            buf = np.frombuffer(z.read(ip), dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            txt = z.read(lp).decode('utf-8', errors='ignore')
        except Exception:
            continue
        if bgr is None:
            continue

        gray = rgb2gray_fpga(bgr)
        med = cv2.medianBlur(gray, 3)
        # 阈值取「车道线亮度」和「路面亮度」的中间，先固定 110 看看
        bina = np.where(med > 110, 255, 0).astype(np.uint8)

        lines = parse_lines(txt)
        for pts in lines:
            # 只用下半幅（近处），远处点太少
            pts = pts[pts[:, 1] > 400]
            if len(pts) < 10:
                continue
            for tag, img in [('A_raw', gray), ('B_median', med), ('C_bin', bina)]:
                prof = sample_profile(img, pts)
                st = gap_stats(prof)
                if st:
                    recs[tag].append(st)

        if i % 50 == 0:
            print('  处理 %d/%d  样本数 A=%d' % (i, len(pick), len(recs['A_raw'])))

    print()
    print('=' * 74)
    print('%-10s %8s %14s %14s %12s' % ('版本', '样本', '暗段数中位', '最长暗段中位', '亮占比中位'))
    print('-' * 74)
    for tag, name in [('A_raw', 'A 原始灰度'), ('B_median', 'B +3x3中值'),
                      ('C_bin', 'C +二值化110')]:
        a = np.array(recs[tag])
        if len(a) == 0:
            continue
        print('%-10s %8d %14.1f %14.1f %12.3f' %
              (name, len(a), np.median(a[:, 1]), np.median(a[:, 2]), np.median(a[:, 3])))

    # 分布：有间隙 vs 无间隙
    print()
    print('%-12s %14s %14s' % ('版本', '零间隙占比', '间隙数>=2占比'))
    for tag, name in [('A_raw', 'A 原始灰度'), ('B_median', 'B +3x3中值'),
                      ('C_bin', 'C +二值化110')]:
        a = np.array(recs[tag])
        if len(a) == 0:
            continue
        print('%-12s %13.0f%% %14.0f%%' %
              (name, 100 * (a[:, 1] == 0).mean(), 100 * (a[:, 1] >= 2).mean()))

    # 存几个剖面图便于肉眼看
    print()
    print('提示：B 与 A 的「暗段数」差距 = 中值滤波抹掉间隙的程度')
    print('     C 的「亮占比」= 二值化后还有多少车道线像素存活')


if __name__ == '__main__':
    main()
