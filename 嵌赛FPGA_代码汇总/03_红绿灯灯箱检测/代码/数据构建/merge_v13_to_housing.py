"""把 v13 的【单灯框】合并/扩展成【灯箱框】, 写入 38 的 _prefill.json.
规则:
  同一行 >=2 颗灯 -> 取并集 + 少量外扩
  只有 1 颗灯     -> 按灯高 d 横向扩成 3.05d x 1.15d (中国横排三灯位)
"""
import os
import json
import numpy as np

DST = r'D:\ADAS_Datasets\38_官方扩帧'
RAW = os.path.join(DST, '_prefill_raw.json')
PREF = os.path.join(DST, '_prefill.json')
IMG = os.path.join(DST, 'images_png')

SPAN = 3.05
THICK = 1.15
CY_MAX = 0.62      # 36池人工标注 73 框 cy 最大 0.58 -> 低于 0.62 的都是车牌/电动车
AREA_MAX = 0.05


def merge_same_row(bs, y_tol=0.75, gap_k=2.6):
    """bs: [(cls,x1,y1,x2,y2)] -> [[group...]]"""
    if not bs:
        return []
    bs = sorted(bs, key=lambda b: b[1])
    used = [False] * len(bs)
    groups = []
    for i in range(len(bs)):
        if used[i]:
            continue
        g = [bs[i]]; used[i] = True
        changed = True
        while changed:
            changed = False
            gx1 = min(b[1] for b in g); gx2 = max(b[3] for b in g)
            gy1 = min(b[2] for b in g); gy2 = max(b[4] for b in g)
            gcy = (gy1 + gy2) / 2; gh = gy2 - gy1
            for j in range(len(bs)):
                if used[j]:
                    continue
                b = bs[j]
                bcy = (b[2] + b[4]) / 2; bh = b[4] - b[2]
                if abs(gcy - bcy) > max(gh, bh) * y_tol:
                    continue
                gap = max(0.0, max(b[1] - gx2, gx1 - b[3]))
                if gap > max(gh, bh) * gap_k:
                    continue
                g.append(b); used[j] = True; changed = True
        groups.append(g)
    return groups


def to_housing(g, W, H):
    x1 = min(b[1] for b in g); x2 = max(b[3] for b in g)
    y1 = min(b[2] for b in g); y2 = max(b[4] for b in g)
    cls = g[0][0]
    if len(g) >= 2:
        padx = (y2 - y1) * 0.12
        pady = (y2 - y1) * 0.10
        x1, x2 = x1 - padx, x2 + padx
        y1, y2 = y1 - pady, y2 + pady
    else:
        d = float(y2 - y1)
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        x1, x2 = cx - SPAN * d / 2, cx + SPAN * d / 2
        y1, y2 = cy - THICK * d / 2, cy + THICK * d / 2
    x1 = max(0.0, min(W, x1)); x2 = max(0.0, min(W, x2))
    y1 = max(0.0, min(H, y1)); y2 = max(0.0, min(H, y2))
    return {'cls': cls, 'x1': round(x1, 1), 'y1': round(y1, 1),
            'x2': round(x2, 1), 'y2': round(y2, 1), 'src': 'v13-merged',
            'n_lamp': len(g)}


def main():
    import cv2
    pre = json.load(open(RAW, encoding='utf-8'))
    out = {}
    n1 = n2 = n3 = 0
    n_drop_cy = n_drop_area = 0
    for fn, boxes in pre.items():
        p = os.path.join(IMG, fn)
        im = cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)
        if im is None:
            continue
        H, W = im.shape[:2]
        bs = []
        for b in boxes:
            cy = (b['y1'] + b['y2']) / 2.0
            area = (b['x2'] - b['x1']) * (b['y2'] - b['y1']) / (W * H)
            if cy > H * CY_MAX:              # 画面下部: 车牌/电动车/人身上的绿
                n_drop_cy += 1
                continue
            if area > AREA_MAX:
                n_drop_area += 1
                continue
            bs.append((b['cls'], b['x1'], b['y1'], b['x2'], b['y2']))
        hs = []
        for g in merge_same_row(bs):
            h = to_housing(g, W, H)
            if (h['x2'] - h['x1']) < 8 or (h['y2'] - h['y1']) < 8:
                continue
            if ((h['y1'] + h['y2']) / 2) > H * CY_MAX:
                continue
            hs.append(h)
            k = h['n_lamp']
            if k == 1: n1 += 1
            elif k == 2: n2 += 1
            else: n3 += 1
        if hs:
            out[fn] = hs
    json.dump(out, open(PREF, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('丢弃: 下部 %d / 过大 %d' % (n_drop_cy, n_drop_area))
    print('帧 %d | 灯箱框 %d (单灯扩展%d / 两灯%d / 三灯及以上%d)'
          % (len(out), n1 + n2 + n3, n1, n2, n3))


if __name__ == '__main__':
    main()
