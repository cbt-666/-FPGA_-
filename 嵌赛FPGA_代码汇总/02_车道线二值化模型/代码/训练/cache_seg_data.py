#!/usr/bin/env python3
"""把 A/B 两套数据缓存成网络输入分辨率，避免每轮重复解码 2560x1600 原图

B 套原图是 2560x1600，直接训练时 CPU 解码+缩放成为瓶颈（GPU 利用率仅 9%）。
这里一次性把「FPGA灰度(1/2: 960x540)」和「掩码(512x288)」落盘，
训练时只读小图，速度提升一个数量级。

灰度存 960x540（而非最终 512x288）是为了二值化模式：
需要先在高分辨率二值化保住细线，再 2x2 池化到 512x288。
"""
import os
import numpy as np
import cv2

ROOT = r'D:\ADAS_Datasets\06_车道线_lane\_seg_data'
CACHE = r'D:\ADAS_Datasets\06_车道线_lane\_seg_cache'
HALF = (1024, 576)    # 缓存的灰度分辨率 (W,H)
MASK = (768, 432)     # 掩码缓存在较高分辨率，训练时再按需缩放


def fpga_gray(bgr):
    b = bgr[:, :, 0].astype(np.int32)
    g = bgr[:, :, 1].astype(np.int32)
    r = bgr[:, :, 2].astype(np.int32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def main():
    total = 0
    for src in ('A', 'B'):
        for split in ('train', 'valid'):
            d = os.path.join(ROOT, src, split)
            if not os.path.isdir(os.path.join(d, 'images')):
                continue
            oi = os.path.join(CACHE, src, split, 'gray')
            om = os.path.join(CACHE, src, split, 'mask')
            os.makedirs(oi, exist_ok=True)
            os.makedirs(om, exist_ok=True)
            n = 0
            for name in sorted(os.listdir(os.path.join(d, 'images'))):
                base = os.path.splitext(name)[0]
                mp = os.path.join(d, 'masks', base + '.png')
                if not os.path.exists(mp):
                    continue
                gp = os.path.join(oi, base + '.png')
                mp2 = os.path.join(om, base + '.png')
                if os.path.exists(gp) and os.path.exists(mp2):
                    # 缓存分辨率改过 -> 尺寸不符就重建，别用旧缓存
                    chk = cv2.imdecode(np.fromfile(gp, np.uint8), cv2.IMREAD_GRAYSCALE)
                    if chk is not None and chk.shape[::-1] == HALF:
                        n += 1
                        continue
                if not os.path.exists(gp):
                    pass
                im = cv2.imdecode(np.fromfile(os.path.join(d, 'images', name), np.uint8),
                                  cv2.IMREAD_COLOR)
                mk = cv2.imdecode(np.fromfile(mp, np.uint8), cv2.IMREAD_GRAYSCALE)
                if im is None or mk is None:
                    continue
                g = cv2.resize(fpga_gray(im), HALF, interpolation=cv2.INTER_AREA)
                m = cv2.resize(mk, MASK, interpolation=cv2.INTER_NEAREST)
                cv2.imencode('.png', g)[1].tofile(gp)
                cv2.imencode('.png', np.clip(m, 0, 2).astype(np.uint8))[1].tofile(
                    os.path.join(om, base + '.png'))
                n += 1
            print('%s/%s: %d' % (src, split, n))
            total += n
    print('合计 %d  输出 %s' % (total, CACHE))


if __name__ == '__main__':
    main()
