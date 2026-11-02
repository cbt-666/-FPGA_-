"""v13 变换: Python 参考实现 vs Verilog 位宽语义 —— 逐位一致性验证

Verilog 里为了避免有符号数，把 `0 <= G-B <= 25` 写成 `G >= B && G <= B+25`。
这个改写必须逐位验证，否则上板结果会和训练数据不一致。
"""
import numpy as np
import cv2
import os, json, sys

# ---- Verilog 侧语义 (IEEE1364 无符号位宽) ----
RED_MIN = 96


def verilog_v13(r, g, b):
    """严格按 .v 文件的表达式求值 (所有中间量扩到 16 位无符号)"""
    r16 = r.astype(np.uint16)
    g16 = g.astype(np.uint16)
    b16 = b.astype(np.uint16)

    red_in = ((r >= RED_MIN) &
              ((r16 * np.uint16(5)) > (g16 * np.uint16(8))) &
              ((r16 + np.uint16(16)) >= b16))

    rg_diff = np.where(r16 > g16, r16 - g16, g16 - r16).astype(np.uint16)
    yel_in = ((r >= 140) & (g >= 120) &
              (rg_diff <= np.uint16(70)) &
              (r16 > (b16 + np.uint16(45))) &
              (g16 > (b16 + np.uint16(45))))

    grn_in = ((g >= 140) &
              (g16 > (r16 + np.uint16(45))) &
              (g16 >= b16) &
              (g16 <= (b16 + np.uint16(25))))

    red_k = red_in
    yel_k = yel_in & ~red_in
    grn_k = grn_in & ~red_in & ~yel_in

    y = ((r16 * np.uint16(77) + g16 * np.uint16(150) + b16 * np.uint16(29)) >> np.uint16(8)).astype(np.uint8)
    out = np.dstack([y, y, y])
    out[red_k] = (0, 0, 255)
    out[yel_k] = (0, 255, 255)
    out[grn_k] = (0, 255, 0)
    return out


# ---- Python 侧参考实现 ----
def python_v13(im):
    b = im[:, :, 0].astype(np.int32)
    g = im[:, :, 1].astype(np.int32)
    r = im[:, :, 2].astype(np.int32)
    y = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
    o = np.dstack([y, y, y])
    red = (r >= 96) & (r * 5 > g * 8) & (r + 16 >= b)
    grn = (g >= 140) & (g > r + 45) & ((g - b) >= 0) & ((g - b) <= 25)
    yel = (r >= 140) & (g >= 120) & (np.abs(r - g) <= 70) & (b < r - 45) & (b < g - 45)
    grn = grn & ~red
    yel = yel & ~red & ~grn
    o[red] = (0, 0, 255)
    o[yel] = (0, 255, 255)
    o[grn] = (0, 255, 0)
    return o


def cmp(bgr, tag):
    a = verilog_v13(bgr[:, :, 2], bgr[:, :, 1], bgr[:, :, 0])
    c = python_v13(bgr)
    d = np.abs(a.astype(int) - c.astype(int))
    n = int((d.max(axis=2) > 0).sum())
    return n, int(d.max())


def main():
    # --- 1) 边界枚举: 抽取有代表性的通道值 ---
    vals = [0, 1, 15, 16, 44, 45, 46, 55, 70, 95, 96, 119, 120, 139, 140,
            141, 185, 186, 190, 195, 205, 214, 227, 254, 255]
    R, G, B = np.meshgrid(vals, vals, vals, indexing='ij')
    grid = np.dstack([B.flatten(), G.flatten(), R.flatten()]).astype(np.uint8).reshape(-1, 1, 3)
    n, mx = cmp(grid, 'boundary')
    print('边界枚举 %d 像素 -> 不一致 %d 处, 最大通道差 %d' % (grid.shape[0], n, mx))
    assert n == 0, '边界枚举不一致!'

    # --- 2) 真实图 ---
    sp = json.load(open(r'D:\ADAS_Datasets\08_中国红绿灯\_split\split.json'))
    tot_px = 0
    tot_bad = 0
    worst = 0
    for split in ['train', 'val']:
        for ip, _ in sp[split][:120]:
            im = cv2.imdecode(np.fromfile(ip, np.uint8), cv2.IMREAD_COLOR)
            if im is None:
                continue
            n, mx = cmp(im, ip)
            tot_px += im.shape[0] * im.shape[1]
            tot_bad += n
            worst = max(worst, mx)
    print('真实图 %d 像素 -> 不一致 %d 处, 最大通道差 %d' % (tot_px, tot_bad, worst))
    assert tot_bad == 0, '真实图不一致!'

    # --- 3) 官方素材 ---
    src = r'C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材'
    tot_px = tot_bad = worst = 0
    for v in ['112454', '112507', '113421']:
        p = os.path.join(src, 'video_20260929_%s.mp4' % v)
        cap = cv2.VideoCapture(p)
        n_ = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        for k in range(4):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(n_ * k / 4))
            ok, f = cap.read()
            if not ok:
                continue
            n, mx = cmp(f, v)
            tot_px += f.shape[0] * f.shape[1]
            tot_bad += n
            worst = max(worst, mx)
        cap.release()
    print('官方素材 %d 像素 -> 不一致 %d 处, 最大通道差 %d' % (tot_px, tot_bad, worst))
    assert tot_bad == 0, '官方素材不一致!'

    print()
    print('✅ Python 与 Verilog 逐位一致 (0 处差异)')


if __name__ == '__main__':
    main()
