#!/usr/bin/env python3
"""合成导航卡的"数字串"字图 —— 用来预训练数字识别模型

为什么合成:
  真实素材只有 144 帧 / 93 个不同数值, 绝大多数数值只出现一次。
  直接训的模型只能死记具体数字, 学不到字形(train 也只有 7/115)。
  合成只教"0~9 长什么样", 真实卡片只用来做最后一步微调。

实测到的真实参数(144 帧统计):
  字高     19~21 px (恒定)
  左起点   x1 中位 52, p10/p90 = 42/62   ← 左对齐, 与位数无关
  上起点   y1 中位 14, p10/p90 = 8/23
  笔色     BGR 中位 (89, 53, 29), 标准差约 (22,12,8)
  背景     BGR 中位 (166, 130, 102)
  字体     DINNextLTPro-Bold (本机, 与真实字形高度吻合)
"""
import os
import json
import random
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\ADAS_Datasets\49_导航数字_合成'
FONT = r'C:\Windows\Fonts\DINNextLTPro-Bold.ttf'
CJK_FONT = r'C:\Windows\Fonts\Deng.ttf'
WIN_W, WIN_H = 200, 80           # 与真实输入窗口一致
PITCH = 16                       # 真机实测字距 16px (p10/p90 = 15/18)
CROP_X0 = 20
CROP_W = CROP_X0 + 3 * PITCH + 24   # = 92
CROP_H = 48                      # 只取数字那一行(纵向 0..48)
N_TRAIN = 24000
N_VAL = 2000


def noise_bg(w, h, rnd):
    """导航卡底色: 浅蓝灰 + 平缓渐变 + 轻微纹理"""
    base = np.array([166, 130, 102], np.float32)          # BGR
    bg = np.ones((h, w, 3), np.float32) * base
    # 平缓渐变
    gy = np.linspace(0, 1, h, dtype=np.float32)[:, None, None]
    gx = np.linspace(0, 1, w, dtype=np.float32)[None, :, None]
    amp = rnd.uniform(6, 22)
    bg += (gy - 0.5) * amp + (gx - 0.5) * amp * 0.6
    # 低频纹理(压缩/屏幕摩尔纹)
    low = np.random.randn(h // 8 + 1, w // 8 + 1, 3).astype(np.float32) * rnd.uniform(1, 5)
    low = cv2.resize(low, (w, h), interpolation=cv2.INTER_CUBIC)
    bg += low
    bg += np.random.randn(h, w, 3).astype(np.float32) * rnd.uniform(0, 3)
    return np.clip(bg, 0, 255).astype(np.uint8)


def draw_digits(v, rnd, no_arrow=True):
    """在一张窗口图上画数字串(+可选箭头), 返回 BGR uint8"""
    w, h = WIN_W, WIN_H
    bg = noise_bg(w, h, rnd)
    img = Image.fromarray(bg[:, :, ::-1])
    d = ImageDraw.Draw(img)

    hgt = int(rnd.uniform(17, 23))
    # 取字号让字形高度≈hgt (用二分搜索, 上次浮点缩放的 int() 把 20 压成了 16)
    def glyph_h(fs):
        ff = ImageFont.truetype(FONT, fs)
        bb = d.textbbox((0, 0), '8', font=ff)
        return bb[3] - bb[1], ff
    lo, hi = 5, 90
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if glyph_h(mid)[0] <= hgt:
            lo = mid
        else:
            hi = mid - 1
    fs = lo
    hh, f = glyph_h(fs)
    if hh < hgt - 1:                    # 字号只能整步, 差 1px 时用 1.5 倍分辨率画完再缩
        sc = hgt / max(hh, 1)
        fs2 = max(6, int(fs * sc * 2))
        f = ImageFont.truetype(FONT, fs2)
        _scaled = True
    else:
        _scaled = False

    txt = str(int(v))
    x0 = rnd.randint(38, 64)
    y0 = rnd.randint(3, 30)
    col = (22 + rnd.randint(-14, 20), 42 + rnd.randint(-14, 22), 72 + rnd.randint(-20, 30))
    for i, ch in enumerate(txt):
        cx = x0 + i * PITCH
        jx = rnd.randint(-2, 2)
        jy = rnd.randint(-1, 1)
        d.text((cx + jx, y0 + jy), ch, font=f, fill=col)
        wgt = rnd.random()
        if wgt < 0.55:                               # 真实笔画更粗: 重画
            d.text((cx + jx + 1, y0 + jy), ch, font=f, fill=col)
        if wgt < 0.25:                               # 更粗: 四向扩散
            for ox, oy in ((1, 1), (-1, 0), (0, 1)):
                d.text((cx + jx + ox, y0 + jy + oy), ch, font=f, fill=col)

    # 可选: "米"符号(右侧小字) —— 真实卡片上紧跟在数字后
    if rnd.random() < 0.7:
        mcjk = (29 + rnd.randint(-18, 18), 53 + rnd.randint(-12, 12),
                89 + rnd.randint(-22, 22))
        d.text((x0 + len(txt) * PITCH + 3, y0 + hgt - 11),
               '米', font=ImageFont.truetype(CJK_FONT, max(9, int(hgt * 0.62))),
               fill=mcjk)

    # 数字下方的路名(汉字, 真实卡片上紧贴数字下面) —— 是真实域的主要干扰
    if rnd.random() < 0.85:
        cjk = ImageFont.truetype(CJK_FONT, max(9, int(hgt * 0.55)))
        nm = rnd.choice(['西乡大道', '宝安大道', '深南大道', '机场路', '福永大道'])
        cc = (29 + rnd.randint(-25, 25), 53 + rnd.randint(-18, 18), 89 + rnd.randint(-30, 30))
        d.text((x0 + rnd.randint(0, 6), y0 + hgt + rnd.randint(1, 6)), nm,
               font=cjk, fill=cc)

    # 底部一排车道指示图标(短竖线+小箭头) —— 进一步的干扰
    if rnd.random() < 0.6:
        by = min(h - 6, y0 + hgt + rnd.randint(20, 34))
        for k in range(rnd.randint(2, 6)):
            bx = rnd.randint(10, 150)
            d.line([(bx, by), (bx + rnd.randint(-2, 2), by - rnd.randint(6, 12))],
                   fill=(60, 75, 105), width=2)

    # 可选: 左侧转向箭头(只画轮廓感, 不教类别)
    if not no_arrow and rnd.random() < 0.6:
        ax, ay = rnd.randint(8, 30), y0 + rnd.randint(-2, 6)
        pts = [(ax, ay + 40), (ax + rnd.randint(-4, 4), ay + 14), (ax + 16, ay + 10)]
        d.line(pts, fill=col, width=rnd.randint(5, 8), joint='curve')
        d.polygon([(ax + 12, ay), (ax + 30, ay + 6), (ax + 16, ay + 18)], fill=col)

    out = np.array(img)[:, :, ::-1].copy()
    if _scaled:                                  # 高分辨画完再缩到目标字高
        out = cv2.resize(out, (int(out.shape[1] / sc2) if (_sc2 := (fs2 / max(fs, 1))) else out.shape[1],
                               int(out.shape[0] / sc2)))
        canvas = np.zeros((WIN_H, WIN_W, 3), np.uint8)
        canvas[:, :] = 166, 130, 102
        h2 = min(WIN_H, out.shape[0]); w2 = min(WIN_W, out.shape[1])
        canvas[:h2, :w2] = out[:h2, :w2]
        out = canvas

    # 几何: 真实数字比 DIN 略宽 —— 横向拉伸 1.0~1.5 倍, 纵向 0.95~1.05
    sx_ = rnd.uniform(1.0, 1.04)
    sy_ = rnd.uniform(0.98, 1.02)
    if abs(sx_ - 1) > 0.02:
        nh, nw = int(out.shape[0] * sy_), int(out.shape[1] * sx_)
        out = cv2.resize(out, (nw, nh), interpolation=cv2.INTER_LINEAR)
        if nw <= WIN_W and nh <= WIN_H:
            canvas = np.zeros((WIN_H, WIN_W, 3), np.uint8)
            canvas[:] = (166, 130, 102)
            canvas[:nh, :nw] = out
            out = canvas
        else:
            out = cv2.resize(out, (WIN_W, WIN_H))

    # 退化: 模糊 / 压缩 / 亮度对比
    if rnd.random() < 0.75:
        k = rnd.choice([3, 3, 5])
        out = cv2.GaussianBlur(out, (k, k), rnd.uniform(0, 1.1))
    if rnd.random() < 0.5:
        ok, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, rnd.randint(28, 70)])
        out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if rnd.random() < 0.5:
        out = np.clip(out.astype(np.float32) * rnd.uniform(0.75, 1.25)
                      + rnd.uniform(-22, 22), 0, 255).astype(np.uint8)
    draw_digits.last_x0 = x0
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--n_train', type=int, default=N_TRAIN)
    ap.add_argument('--n_val', type=int, default=N_VAL)
    ap.add_argument('--arrows', action='store_true', help='同时画箭头(难度更高)')
    a = ap.parse_args()
    rnd = random.Random(7)
    np.random.seed(7)

    for split, n in (('train', a.n_train), ('val', a.n_val)):
        d = os.path.join(OUT, split)
        os.makedirs(d, exist_ok=True)
        meta = []
        for i in range(n):
            if split == 'val':
                v = rnd.randint(1, 999)                 # 验证集覆盖全部数值
            else:
                # 训练集不用 0xx(真机不会出现"0米"), 且优先 3 位
                v = rnd.randint(1, 999) if rnd.random() < 0.8 else rnd.randint(1, 99)
            if v == 0:
                v = 1
            im = draw_digits(v, rnd, no_arrow=not a.arrows)
            x0 = draw_digits.last_x0
            # 长度归一化: 窗口左边界对齐到数字串起点前 20px
            lx = max(0, min(x0 - CROP_X0, WIN_W - CROP_W))
            im = im[:CROP_H, lx:lx + CROP_W].copy()
            p = os.path.join(d, '%06d.jpg' % i)
            cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tofile(p)
            meta.append({'file': '%06d.jpg' % i, 'value': int(v)})
            if (i + 1) % 4000 == 0:
                print('  %s %d/%d' % (split, i + 1, n), flush=True)
        json.dump(meta, open(os.path.join(OUT, 'labels_%s.json' % split), 'w',
                             encoding='utf-8'), ensure_ascii=False)
    # 预览图
    tiles = []
    for v in (5, 47, 128, 306, 512, 893):
        im = draw_digits(v, rnd, no_arrow=False)
        lx = max(0, min(draw_digits.last_x0 - CROP_X0, WIN_W - CROP_W))
        tiles.append(cv2.resize(im[:CROP_H, lx:lx + CROP_W], (300, 120)))
    cv2.imencode('.jpg', np.hstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tofile(
        os.path.join(OUT, '_preview.jpg'))
    print('done ->', OUT)


if __name__ == '__main__':
    main()
