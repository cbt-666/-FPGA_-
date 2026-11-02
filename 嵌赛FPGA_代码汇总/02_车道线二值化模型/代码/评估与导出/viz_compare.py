#!/usr/bin/env python3
"""可视化：同一帧上三个模型的输出对比

输入共用一帧官方素材；三版模型各自按自己的输入契约推理，输出掩码 + 判决。
"""
import os
import sys
import csv
import json

import numpy as np
import cv2
import torch
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, r'D:\ADAS_Datasets\_code')
sys.path.insert(0, r'D:\AI\CLaude_workplace\临时文件')
from train_lane_seg import Net, fpga_gray                      # noqa: E402
from train_trunc import NetTrunc                               # noqa: E402
from lane_decision import decode                               # noqa: E402

MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(1, 3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(1, 3, 1, 1)
RUN = r'D:\ADAS_Datasets\06_车道线_lane\_runs\seg'
VID = r'C:\Users\cbt\Desktop\嵌赛FPGA\官方资料\紫光同创杯赛题三参考素材'
MAN = r'D:\ADAS_Datasets\06_车道线_lane\_label_set2\manifest.json'
OUT = r'C:\Users\cbt\Desktop\嵌赛FPGA\Rk3568\车道_A_B1_B2_C\v2_768\对比图.png'
FONT = r'C:\Windows\Fonts\msyh.ttc'


def load_models():
    a = Net(pretrained=False)
    a.load_state_dict(torch.load(os.path.join(RUN, 'lane_seg_binary_off768th190.pt'),
                                  map_location='cuda'))
    t = NetTrunc(nclass=3, pretrained=False, stop_at='m1')
    t.load_state_dict(torch.load(os.path.join(RUN, 'lane_seg_binary_truncm1_768x432.pt'),
                                  map_location='cuda'), strict=False)
    g = Net(pretrained=False)
    g.load_state_dict(torch.load(os.path.join(RUN, 'lane_seg_gray_off640gray.pt'),
                                  map_location='cuda'))
    for n in (a, t, g):
        n.to('cuda').eval()
    return a, t, g


def prep(gray, W, H, mode, th=190):
    if mode == 'gray':
        v = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
    else:
        b = np.where(gray > th, 255, 0).astype(np.uint8)
        h0, w0 = b.shape
        f = int(round(w0 / W))
        if f >= 2 and abs(w0 / W - f) < 1e-6:
            hh, ww = h0 // f * f, w0 // f * f
            vv = b[:hh, :ww].reshape(hh // f, f, ww // f, f).max(axis=(1, 3))
            v = cv2.resize(vv, (W, H), interpolation=cv2.INTER_NEAREST)
        else:
            v = cv2.resize(b, (W, H), interpolation=cv2.INTER_NEAREST)
    x = np.stack([v.astype(np.float32) / 255.0] * 3, 0)[None]
    x = (x - MEAN) / STD
    return v, x.astype(np.float32)


def infer(net, x):
    with torch.no_grad():
        return net(torch.from_numpy(x).cuda()).argmax(1)[0].cpu().numpy().astype(np.uint8)


def overlay(vis_gray, mask, box=None):
    """灰度底图 + 掩码叠加（虚线绿、实线红）"""
    v = cv2.cvtColor(vis_gray, cv2.COLOR_GRAY2BGR)
    ov = v.copy()
    ov[mask == 1] = (80, 220, 80)
    ov[mask == 2] = (60, 60, 235)
    m = mask > 0
    v[m] = (v[m] * 0.35 + ov[m] * 0.65).astype(np.uint8)
    if box is not None:
        d = decode(mask)
        H, W = mask.shape
        for k, c in (('left_x', (255, 170, 40)), ('right_x', (40, 170, 255))):
            if d[k] is not None:
                cv2.line(v, (d[k], int(H * 0.45)), (d[k], int(H * 0.92)), c, 3)
    return v


def main():
    # 找一帧左右虚实不同、且三个模型都能出结果的官方帧
    val = set(json.load(open(os.path.join(RUN, 'official_val.json'), encoding='utf-8')))
    cands = []
    with open(r'C:\Users\cbt\Desktop\嵌赛FPGA\lane_labels2.csv', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            L, R = (r.get('L') or '').strip(), (r.get('R') or '').strip()
            if {L, R} == {'dash', 'solid'} and r['img'] in val:
                cands.append((r['img'], L, R))
    print('左右虚实不同的候选帧 %d 个' % len(cands))

    man = {m['img']: m for m in json.load(open(MAN, encoding='utf-8')) if m.get('vid')}
    a_net, t_net, g_net = load_models()

    best = None
    caps = {}
    for img, L, R in cands:
        m = man.get(img)
        if not m:
            continue
        v = m['vid']
        if v not in caps:
            caps[v] = cv2.VideoCapture(os.path.join(VID, v))
        c = caps[v]
        c.set(cv2.CAP_PROP_POS_FRAMES, int(m['frame']))
        ok, fr = c.read()
        if not ok:
            continue
        raw = fr
        gray_full = fpga_gray(raw)
        res = {}
        for tag, net, W, H, mode in (('A768', a_net, 768, 432, 'binary'),
                                     ('T1H4', t_net, 768, 432, 'binary'),
                                     ('G640', g_net, 640, 360, 'gray')):
            vis_input, x = prep(gray_full, W, H, mode)
            pm = infer(net, x)
            d = decode(pm)
            res[tag] = (vis_input, pm, d)
        # 打分：三模型判决都和 GT 一致优先；至少两个对；未检出少
        score = 0
        for tag in res:
            d = res[tag][2]
            for side, gt in (('left', L), ('right', R)):
                if d[side] == gt:
                    score += 2
                elif d[side] == 'none':
                    score -= 1
        # 偏好画面清晰（车道线像素多）
        score += 0.00002 * int((res['A768'][1] > 0).sum())
        if best is None or score > best[0]:
            best = (score, img, L, R, raw, res)
    for c in caps.values():
        c.release()

    if best is None:
        raise SystemExit('没找到可用帧')

    score, img, L, R, raw, res = best
    print('选中 %s  GT: 左=%s 右=%s  (score %.1f)' % (img, L, R, score))

    # ---- 拼图 ----
    PW, PH = 768, 432
    PAD = 22
    HEAD_H = 118          # 顶部标题区
    TITLE_H = 34          # 面板标题
    SUB_H = 30            # 面板副标题
    VERDICT_H = 42        # 面板下方判决行
    LEGEND_H = 54
    CELL_H = TITLE_H + SUB_H + PH + VERDICT_H
    cols, rows = 2, 2
    W = cols * PW + PAD * (cols + 1)
    H = HEAD_H + rows * (CELL_H + PAD) + LEGEND_H

    canvas = Image.new('RGB', (W, H), (24, 26, 30))
    dr = ImageDraw.Draw(canvas)
    f_title = ImageFont.truetype(FONT, 40)
    f_sub = ImageFont.truetype(FONT, 25)
    f_panel = ImageFont.truetype(FONT, 29)
    f_body = ImageFont.truetype(FONT, 24)
    f_legend = ImageFont.truetype(FONT, 23)

    dr.text((PAD, 22), '车道模型加速方案 · 同一帧输出对比',
            font=f_title, fill=(240, 242, 246))
    dr.text((PAD, 74),
            '官方素材 %s     真值：左侧%s / 右侧%s   （绿=虚线可变道  红=实线禁变道）'
            % (img, '虚线' if L == 'dash' else '实线', '虚线' if R == 'dash' else '实线'),
            font=f_sub, fill=(150, 158, 172))

    panels = []
    # 第一格：输入（二值）
    vis_in_bin, _ = prep(fpga_gray(raw), PW, PH, 'binary')
    panels.append(('输入（二值 th190）', 'A768 / T1H4 共用此输入 · 768×432',
                   cv2.cvtColor(vis_in_bin, cv2.COLOR_GRAY2BGR), None))

    for tag, cn, mac in (('A768', '完整解码器 768×432', '3.86 GMAC · 0.36× 现役'),
                         ('T1H4', '截断 H/4 768×432', '1.88 GMAC · 0.18× 现役'),
                         ('G640', '灰度 640×360', '1.71 GMAC · 0.16× 现役')):
        vis_in, pm, d = res[tag]
        pan = overlay(vis_in, pm)
        panels.append((tag + '  ' + cn, mac + '   ' +
                       ('输入：灰度' if tag == 'G640' else '输入：二值'),
                       pan, d))

    for i, (title, sub, img_bgr, d) in enumerate(panels):
        r, c = divmod(i, cols)
        x0 = PAD + c * (PW + PAD)
        y0 = HEAD_H + r * (CELL_H + PAD)
        # 面板标题
        dr.text((x0, y0), title, font=f_panel, fill=(235, 238, 244))
        dr.text((x0, y0 + TITLE_H), sub, font=f_body, fill=(120, 195, 240))
        # 图像，统一缩放到 PW×PH
        pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        if pil.size != (PW, PH):
            pil = pil.resize((PW, PH), Image.LANCZOS)
        iy = y0 + TITLE_H + SUB_H
        canvas.paste(pil, (x0, iy))
        dr.rectangle([x0, iy, x0 + PW - 1, iy + PH - 1], outline=(70, 76, 88), width=2)
        # 判决行
        if d is not None:
            vy = iy + PH + 8
            xc = x0
            for side, gt, cn in (('left', L, '左'), ('right', R, '右')):
                got = d[side]
                name = {'dash': '虚线', 'solid': '实线', 'none': '未检出'}[got]
                ok = got == gt
                txt = '%s侧预测「%s」%s' % (cn, name, '对' if ok else '错')
                col = (105, 225, 125) if ok else (245, 105, 105)
                dr.text((xc, vy), txt, font=f_body, fill=col)
                xc += dr.textlength(txt, font=f_body) + 34

    dr.text((PAD, H - LEGEND_H + 14),
            '彩色叠加区域 = 模型判定为车道线的像素    '
            '青/橙竖线 = 解码器在参考带上选中的左/右车道线位置',
            font=f_legend, fill=(140, 148, 162))

    canvas.save(OUT)
    print('-> %s' % OUT)


if __name__ == '__main__':
    main()
