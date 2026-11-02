"""构建灯箱检测数据集: 彩色版 + 灰度版混合.

为什么混灰度: 真实工况下 FPGA 输出的图里, 灯可能全灭/黄灯/画面整体低饱和,
   纯彩色样本训练的模型遇到无颜色输入会掉点. 同一张图同时给彩色和灰度两份,
   强迫模型学【灯箱的明暗结构】而不是【红绿像素】.

切分: 按视频隔离, 113539 整段作验证集.
"""
import os
import json
import shutil
from collections import Counter, defaultdict

import cv2
import numpy as np

POOLS = [
    (r'D:\ADAS_Datasets\36_中国灯箱池', 'p36'),
    (r'D:\ADAS_Datasets\40_扩帧大池', 'p40'),
]
NEG_POOL = r'D:\ADAS_Datasets\43_负样本'
NEG_FRAC = 0.30              # 负样本训练图数 = 正样本训练帧数 x 该比例 (灰度版再翻倍)
SYNC_FROM = r'D:\ADAS_Datasets\41_合并标注池'
OUT = r'D:\ADAS_Datasets\42_housing_1c'
VAL_GROUPS = ('113539',)
W_R, W_G, W_B = 77, 150, 29


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
            touched[dst_p] = json.load(open(dst_p, encoding='utf-8')) if os.path.isfile(dst_p) else {}
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


def to_gray(bgr):
    """真灰度(不保留红/绿): 只保留明暗, 灯箱轮廓仍可见但无颜色线索."""
    b = bgr[:, :, 0].astype(np.float32)
    g = bgr[:, :, 1].astype(np.float32)
    r = bgr[:, :, 2].astype(np.float32)
    y = (W_R * r + W_G * g + W_B * b) / 256.0
    y = np.clip(y, 0, 255).astype(np.uint8)
    return np.dstack([y, y, y])


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def imwrite(p, img):
    cv2.imencode('.png', img)[1].tofile(p)


TRAIN_W, TRAIN_H = 1280, 720      # 与部署输入一致的统一尺寸


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

    # 同一张原图可能在多个池里, 保留一条
    by = {}
    for rec in records:
        by[(rec['group'], rec['orig'])] = rec
    records = list(by.values())
    print('  [dedup] 合计 %d 帧' % len(records))

    for rec in records:
        im = imread(rec['src'])
        rec['H'], rec['W'] = im.shape[:2]
        rec['split'] = 'val' if rec['group'] in VAL_GROUPS else 'train'

    if not any(r['split'] == 'val' for r in records):
        raise SystemExit('验证集为空: 检查 VAL_GROUPS')

    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    for s in ('train', 'val'):
        os.makedirs(os.path.join(OUT, s, 'images'), exist_ok=True)
        os.makedirs(os.path.join(OUT, s, 'labels'), exist_ok=True)

    stat = Counter()
    for rec in records:
        im = imread(rec['src'])
        H, W = rec['H'], rec['W']
        # 统一缩放到 1280x720 (与板端输入一致); S2TLD 是 4:3, 按宽度缩放居中裁掉上下
        sx, sy = TRAIN_W / float(W), TRAIN_H / float(H)
        s = max(sx, sy)
        nw, nh = int(round(W * s)), int(round(H * s))
        im2 = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_AREA)
        ox, oy = (nw - TRAIN_W) // 2, (nh - TRAIN_H) // 2
        im2 = im2[oy:oy + TRAIN_H, ox:ox + TRAIN_W]

        lines = []
        for b in rec['boxes']:
            # 原图坐标 -> 缩放/裁剪后坐标
            x1 = (float(b[1]) * s) - ox
            x2 = (float(b[3]) * s) - ox
            y1 = (float(b[2]) * s) - oy
            y2 = (float(b[4]) * s) - oy
            # 裁掉的部分不再有完整框, 直接丢
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
        if not lines:
            continue
        split = rec['split']
        stem = os.path.splitext(rec['orig'])[0]
        stem = '%s_%s' % (rec['pool'], stem) if split == 'train' else stem

        for suffix, img in (('', im2), ('_g', to_gray(im2))):
            name = stem + suffix + '.png'
            imwrite(os.path.join(OUT, split, 'images', name), img)
            with open(os.path.join(OUT, split, 'labels', stem + suffix + '.txt'), 'w') as f:
                f.write('\n'.join(lines))
            stat[(split, 'gray' if suffix else 'color', 'img')] += 1
            stat[(split, 'gray' if suffix else 'color', 'box')] += len(lines)

        # 模糊增强: 只有训练集, 约 1/3 的图, 针对"远距/失焦小目标"这个最弱条件
        if split == 'train' and (hash(rec['orig']) % 3 == 0):
            b1 = cv2.GaussianBlur(im2, (5, 5), 1.6)
            name = stem + '_b1.png'
            imwrite(os.path.join(OUT, 'train', 'images', name), b1)
            with open(os.path.join(OUT, 'train', 'labels', stem + '_b1.txt'), 'w') as f:
                f.write('\n'.join(lines))
            stat[('train', 'blur', 'img')] += 1
            stat[('train', 'blur', 'box')] += len(lines)

            b2 = cv2.GaussianBlur(to_gray(im2), (7, 7), 2.2)
            name = stem + '_b2.png'
            imwrite(os.path.join(OUT, 'train', 'images', name), b2)
            with open(os.path.join(OUT, 'train', 'labels', stem + '_b2.txt'), 'w') as f:
                f.write('\n'.join(lines))
            stat[('train', 'blur', 'img')] += 1
            stat[('train', 'blur', 'box')] += len(lines)

    with open(os.path.join(OUT, 'data.yaml'), 'w', encoding='utf-8') as f:
        f.write('path: %s\n' % OUT.replace('\\', '/'))
        f.write('train: train/images\nval: val/images\n')
        f.write('nc: 1\nnames: [housing]\n')

    # ---- 负样本: 无灯的官方帧, 空 label, 压误检 ----
    n_train_frames = stat[('train', 'color', 'img')]
    n_neg = int(n_train_frames * NEG_FRAC)
    neg_files = []
    if os.path.isdir(os.path.join(NEG_POOL, 'images_png')):
        neg_files = sorted(os.listdir(os.path.join(NEG_POOL, 'images_png')))
    neg_files = [f for f in neg_files if f.lower().endswith(('.png', '.jpg'))]
    # ⚠️ 泄漏防线: 验证集视频(113539)的帧绝不能进训练
    neg_files = [f for f in neg_files
                 if not any(g in f for g in VAL_GROUPS)]
    # 均匀取样, 不集中在前几个视频
    if len(neg_files) > n_neg > 0:
        step = len(neg_files) / float(n_neg)
        neg_files = [neg_files[int(i * step)] for i in range(n_neg)]
    n_neg_img = 0
    for fn in neg_files:
        src = os.path.join(NEG_POOL, 'images_png', fn)
        im = imread(src)
        if im is None:
            continue
        im = cv2.resize(im, (TRAIN_W, TRAIN_H), interpolation=cv2.INTER_AREA)
        for suffix, img in (('', im), ('_g', to_gray(im))):
            name = 'neg_' + os.path.splitext(fn)[0] + suffix + '.png'
            imwrite(os.path.join(OUT, 'train', 'images', name), img)
            open(os.path.join(OUT, 'train', 'labels',
                              os.path.splitext(name)[0] + '.txt'), 'w').close()
            n_neg_img += 1
        stat[('train', 'neg', 'img')] += 2

    print('\n==> %s' % OUT)
    for split in ('train', 'val'):
        for kind in ('color', 'gray', 'blur', 'neg'):
            n_img = stat[(split, kind, 'img')]
            if n_img:
                print('    %-5s %-5s %3d 图 / %3d 框'
                      % (split, kind, n_img, stat[(split, kind, 'box')]))
    gs = Counter(r['split'] for r in records)
    print('    原始帧: train %d / val %d | 负样本 %d 张' % (gs['train'], gs['val'], n_neg_img))


if __name__ == '__main__':
    main()
