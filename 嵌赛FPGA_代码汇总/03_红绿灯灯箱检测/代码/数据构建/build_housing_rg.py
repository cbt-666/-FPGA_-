"""构建【纯红绿灰度域】灯箱检测数据集 (去掉全灰度干扰).

与 build_housing_dataset.py 的唯一区别:
    不再给每张标注图生成 to_gray() 的全灰度副本 (_g / _b2),
    也不给负样本生成灰度副本。
    训练数据 100% 是 FPGA 输出的红绿灰度图 (红=254 / 绿=255 哨兵值)。

原数据集 42_housing_1c 保持不变, 本脚本输出到 42_housing_1c_rg。

用法: python build_housing_rg.py
"""
import os
import json
import shutil
from collections import Counter

import cv2
import numpy as np

POOLS = [
    (r'D:\ADAS_Datasets\36_中国灯箱池', 'p36'),
    (r'D:\ADAS_Datasets\40_扩帧大池', 'p40'),
]
NEG_POOL = r'D:\ADAS_Datasets\43_负样本'
NEG_FRAC = 0.30
SYNC_FROM = r'D:\ADAS_Datasets\41_合并标注池'
OUT = r'D:\ADAS_Datasets\42_housing_1c_rg'
VAL_GROUPS = ('113539',)

TRAIN_W, TRAIN_H = 1280, 720


def orig_name(fn):
    return fn.split('__', 1)[1] if '__' in fn else fn


def sync_back():
    """41 合并池的标注按原文件名写回各原池 (41 只是硬链接视图)."""
    lab_p = os.path.join(SYNC_FROM, '_labels.json')
    if not os.path.isfile(lab_p):
        return
    sm_p = os.path.join(SYNC_FROM, '_source_map.json')
    sm = json.load(open(sm_p, encoding='utf-8')) if os.path.isfile(sm_p) else {}
    merged = json.load(open(lab_p, encoding='utf-8'))
    touched = {}
    n = 0
    for key, rec in merged.items():
        if not isinstance(rec, dict) or not rec.get('done'):
            continue
        info = sm.get(key)
        if not info:
            continue
        dst_p = os.path.join(info['pool'], '_labels.json')
        if dst_p not in touched:
            touched[dst_p] = (json.load(open(dst_p, encoding='utf-8'))
                               if os.path.isfile(dst_p) else {})
        touched[dst_p][info['file']] = rec
        n += 1
    for p, d in touched.items():
        json.dump(d, open(p, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('  [sync] %d 条标注写回原池' % n)


def group_of(fn):
    fn = orig_name(fn)
    if fn.startswith('s2_'):
        return 's2tld'
    for p in fn.split('_'):
        if p.isdigit() and len(p) == 6:
            return p
    return 'other'


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def imwrite(p, img):
    cv2.imencode('.png', img)[1].tofile(p)


def fit_1280x720(im):
    """等比缩放 + 居中裁切到 1280x720, 返回 (图, 缩放比, 左上偏移)."""
    H, W = im.shape[:2]
    sx, sy = TRAIN_W / float(W), TRAIN_H / float(H)
    s = max(sx, sy)
    nw, nh = int(round(W * s)), int(round(H * s))
    im2 = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_AREA)
    ox, oy = (nw - TRAIN_W) // 2, (nh - TRAIN_H) // 2
    return im2[oy:oy + TRAIN_H, ox:ox + TRAIN_W], s, ox, oy


def remap_boxes(boxes, s, ox, oy):
    lines = []
    for b in boxes:
        x1 = (float(b[1]) * s) - ox
        x2 = (float(b[3]) * s) - ox
        y1 = (float(b[2]) * s) - oy
        y2 = (float(b[4]) * s) - oy
        if x1 < 0 or y1 < 0 or x2 > TRAIN_W or y2 > TRAIN_H:
            if x1 < -2 or y1 < -2 or x2 > TRAIN_W + 2 or y2 > TRAIN_H + 2:
                continue
        x1 = max(0.0, min(TRAIN_W, x1)); x2 = max(0.0, min(TRAIN_W, x2))
        y1 = max(0.0, min(TRAIN_H, y1)); y2 = max(0.0, min(TRAIN_H, y2))
        if x2 - x1 < 4 or y2 - y1 < 4:
            continue
        lines.append('0 %.6f %.6f %.6f %.6f'
                     % (((x1 + x2) / 2) / TRAIN_W, ((y1 + y2) / 2) / TRAIN_H,
                        (x2 - x1) / TRAIN_W, (y2 - y1) / TRAIN_H))
    return lines


def main():
    sync_back()

    records = []
    for pool, tag in POOLS:
        lab_p = os.path.join(pool, '_labels.json')
        if not os.path.isfile(lab_p):
            continue
        lab = json.load(open(lab_p, encoding='utf-8'))
        imgdir = os.path.join(pool, 'images_png')
        n = 0
        for fn, r in lab.items():
            if not isinstance(r, dict) or not r.get('done'):
                continue
            boxes = r.get('boxes') or []
            if not boxes:
                continue
            src = os.path.join(imgdir, fn)
            if not os.path.isfile(src):
                continue
            records.append({'file': fn, 'pool': tag, 'src': src, 'boxes': boxes,
                            'group': group_of(fn), 'orig': orig_name(fn)})
            n += 1
        print('  %-22s %3d 帧有框' % (os.path.basename(pool), n))

    by = {}
    for rec in records:
        by[(rec['group'], rec['orig'])] = rec
    records = list(by.values())
    print('  [dedup] 合计 %d 帧' % len(records))

    for rec in records:
        rec['split'] = 'val' if rec['group'] in VAL_GROUPS else 'train'
    if not any(r['split'] == 'val' for r in records):
        raise SystemExit('验证集为空: 检查 VAL_GROUPS')

    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    for s in ('train', 'val'):
        os.makedirs(os.path.join(OUT, s, 'images'), exist_ok=True)
        os.makedirs(os.path.join(OUT, s, 'labels'), exist_ok=True)

    stat = Counter()
    for i, rec in enumerate(records):
        im = imread(rec['src'])
        if im is None:
            continue
        im2, s, ox, oy = fit_1280x720(im)
        lines = remap_boxes(rec['boxes'], s, ox, oy)
        if not lines:
            continue

        split = rec['split']
        stem = os.path.splitext(rec['orig'])[0]
        if split == 'train':
            stem = '%s_%s' % (rec['pool'], stem)

        # 唯一的输入域: 红绿灰度原图
        imwrite(os.path.join(OUT, split, 'images', stem + '.png'), im2)
        with open(os.path.join(OUT, split, 'labels', stem + '.txt'), 'w') as f:
            f.write('\n'.join(lines))
        stat[(split, 'rg', 'img')] += 1
        stat[(split, 'rg', 'box')] += len(lines)

        # 模糊增强: 只有训练集, 1/3 的图, 保持红绿不被破坏
        if split == 'train' and i % 3 == 0:
            b1 = cv2.GaussianBlur(im2, (5, 5), 1.6)
            imwrite(os.path.join(OUT, 'train', 'images', stem + '_b1.png'), b1)
            with open(os.path.join(OUT, 'train', 'labels', stem + '_b1.txt'), 'w') as f:
                f.write('\n'.join(lines))
            stat[('train', 'blur', 'img')] += 1
            stat[('train', 'blur', 'box')] += len(lines)

    with open(os.path.join(OUT, 'data.yaml'), 'w', encoding='utf-8') as f:
        f.write('path: %s\n' % OUT.replace('\\', '/'))
        f.write('train: train/images\nval: val/images\n')
        f.write('nc: 1\nnames: [housing]\n')

    # ---- 负样本: 无灯的官方帧, 空 label, 压误检 (只放红绿域) ----
    n_pos_train = stat[('train', 'rg', 'img')] + stat[('train', 'blur', 'img')]
    n_neg = int(stat[('train', 'rg', 'img')] * NEG_FRAC)
    neg_dir = os.path.join(NEG_POOL, 'images_png')
    neg_files = []
    if os.path.isdir(neg_dir):
        neg_files = sorted(f for f in os.listdir(neg_dir)
                           if f.lower().endswith(('.png', '.jpg')))
    neg_files = [f for f in neg_files if not any(g in f for g in VAL_GROUPS)]
    if len(neg_files) > n_neg > 0:
        step = len(neg_files) / float(n_neg)
        neg_files = [neg_files[int(i * step)] for i in range(n_neg)]
    n_neg_img = 0
    for fn in neg_files:
        im = imread(os.path.join(neg_dir, fn))
        if im is None:
            continue
        im = cv2.resize(im, (TRAIN_W, TRAIN_H), interpolation=cv2.INTER_AREA)
        stem = 'neg_' + os.path.splitext(fn)[0]
        imwrite(os.path.join(OUT, 'train', 'images', stem + '.png'), im)
        open(os.path.join(OUT, 'train', 'labels', stem + '.txt'), 'w').close()
        n_neg_img += 1

    print('\n==> %s' % OUT)
    for split in ('train', 'val'):
        for kind in ('rg', 'blur', 'neg'):
            n_img = stat[(split, kind, 'img')]
            if n_img:
                print('    %-5s %-5s %3d 图 / %3d 框'
                      % (split, kind, n_img, stat[(split, kind, 'box')]))
    print('    正样本训练图 %d + 负样本 %d = 合计 %d'
          % (n_pos_train, n_neg_img, n_pos_train + n_neg_img))


if __name__ == '__main__':
    main()
