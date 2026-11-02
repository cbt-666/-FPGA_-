#!/usr/bin/env python3
"""逐位验证：fpga_transform.py  vs  rgb888_ycbcr.v 的 Verilog 位宽语义

目的：回答「.v 转 .py 有没有转错」。
方法：严格按照 IEEE1364 的位宽规则重新实现一遍硬件，
      和 fpga_transform 的输出逐像素比对。

Verilog 里需要精确建模的点：
  1. r/g/b 是 8-bit，取值 0..255
  2. 比较式 {1'b0,g} > {1'b0,r} + {1'b0,GREEN_MARGIN}
     —— 操作数被扩成 9-bit，结果 u9。注意 9-bit 加法最大 255+255=510 会溢出 u9(511)？不会。
  3. 乘法 r * 8'd77 ：上下文宽度 = LHS(16-bit)，故按 16-bit 计算
  4. 求和 r_m0+g_m0+b_m0：16-bit。最大值 255*77+255*150+255*29 = 65280 < 65536，不溢出
  5. 移位 y_1 <= y_0[15:8]  即 >>8
"""
import numpy as np


def verilog_ref(bgr, red_min=96, green_min=56, green_margin=16):
    """按 Verilog 位宽语义逐位实现 rgb888_yrg"""
    H, W = bgr.shape[:2]
    # 8-bit 取通道
    b = bgr[:, :, 0].astype(np.uint16)   # u8
    g = bgr[:, :, 1].astype(np.uint16)   # u8
    r = bgr[:, :, 2].astype(np.uint16)   # u8

    # --- 9-bit 扩展比较（Verilog: {1'b0,x} 把 8bit 扩成 9bit）---
    r9, g9, b9 = r, g, b                       # 值域不变，只是声明为 9bit
    m9 = np.uint16(green_margin)

    red_keep = (r >= red_min) & (r9 > (g9 * 2)) & (r9 > (b9 * 2))
    #  注意 Verilog 写的是 {g,1'b0}（左移1位=乘2）和 {b,1'b0}
    green_keep = ((g >= green_min) &
                  (g9 > (r9 + m9)) &
                  (g9 > (b9 + m9)))

    # --- 乘法：16-bit ---
    r_m0 = (r * np.uint16(77)) & 0xFFFF
    g_m0 = (g * np.uint16(150)) & 0xFFFF
    b_m0 = (b * np.uint16(29)) & 0xFFFF

    # --- 求和：16-bit，按 Verilog 会自然回绕 ---
    y_0 = (r_m0 + g_m0 + b_m0) & 0xFFFF

    # --- 移位：取 [15:8] ---
    y_1 = (y_0 >> 8).astype(np.uint8)

    out = np.dstack([y_1, y_1, y_1]).astype(np.uint8)
    out[red_keep] = (0, 0, 255)     # 纯红 (B,G,R)=(0,0,255)
    out[green_keep] = (0, 255, 0)   # 纯绿
    return out, red_keep, green_keep, y_1


if __name__ == '__main__':
    import sys, os, glob
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from fpga_transform import transform_fpga, imread_u

    print('=' * 68)
    print('A. 边界枚举：全部 256^3 中抽样 + 关键边界值')
    print('=' * 68)
    rng = np.random.default_rng(0)
    # 关键边界：0,1,15,16,17,31,32,33,55,56,57,76,77,78,95,96,97,127,128,191,192,224,254,255
    edge = [0, 1, 15, 16, 17, 31, 32, 33, 47, 48, 55, 56, 57, 76, 77, 78,
            95, 96, 97, 111, 112, 127, 128, 129, 191, 192, 193, 223, 224, 254, 255]
    pts = [(a, b, c) for a in edge for b in edge for c in edge]
    pts += [tuple(int(x) for x in p) for p in rng.integers(0, 256, (200000, 3))]
    arr = np.array(pts, dtype=np.uint8).reshape(1, -1, 3)   # (1,N,3) BGR

    ref, rk_ref, gk_ref, _ = verilog_ref(arr)
    got = transform_fpga(arr, fixed=False)

    diff = (ref.astype(int) - got.astype(int))
    n_diff = int((diff != 0).any(axis=2).sum())
    print('  测试像素数        :', arr.shape[1])
    print('  输出不一致像素数  :', n_diff)
    print('  最大通道差        :', int(np.abs(diff).max()))
    if n_diff:
        idx = np.where((diff != 0).any(axis=2))[0][:10]
        for i in idx:
            print('    BGR输入=%s  参考=%s  我的=%s' % (
                tuple(arr[0, i]), tuple(ref[0, i]), tuple(got[0, i])))
    print('  结论:', '✅ 逐位一致' if n_diff == 0 else '❌ 存在差异')

    print()
    print('=' * 68)
    print('B. 真实图像：433 张全量比对')
    print('=' * 68)
    base = r'D:\AI\CLaude_workplace\临时文件\队友资料\红绿灯'
    tot_px = 0
    bad_px = 0
    bad_img = 0
    n_img = 0
    for grp in ['enable', 'unable']:
        for f in sorted(glob.glob(os.path.join(base, grp, '*.jpg'))):
            im = imread_u(f)
            if im is None:
                continue
            n_img += 1
            ref, _, _, _ = verilog_ref(im)
            got = transform_fpga(im, fixed=False)
            d = (ref.astype(int) != got.astype(int)).any(axis=2)
            tot_px += d.size
            bad_px += int(d.sum())
            if d.any():
                bad_img += 1
    print('  图像数            :', n_img)
    print('  像素总数          :', tot_px)
    print('  不一致像素        :', bad_px)
    print('  不一致图像数      :', bad_img)
    print('  结论:', '✅ 逐位一致' if bad_px == 0 else '❌ 存在差异')

    print()
    print('=' * 68)
    print('C. 关键疑点复核：269.jpg 那颗绿灯灯芯')
    print('=' * 68)
    im = imread_u(os.path.join(base, 'enable', '269.jpg'))
    for (y, x) in [(230, 186), (242, 184), (211, 128)]:
        px = im[y, x]                       # BGR
        R, G, B = int(px[2]), int(px[1]), int(px[0])
        single = np.array([[[px[0], px[1], px[2]]]], dtype=np.uint8)
        ref, _, gk, _ = verilog_ref(single)
        print('  (%3d,%3d) BGR=(%3d,%3d,%3d)  g-r=%+4d g-b=%+4d  ->  %s' % (
            y, x, B, G, R, G - R, G - B,
            '绿' if gk[0, 0] else '灰'))
    print('  说明：g>b+16 需要 g-B>16，而实测 g-b 仅 +2~+16 → 判灰')
    print('        这是 rgb888_ycbcr.v 的 GREEN_MARGIN=16 的必然结果')
