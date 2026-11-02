#!/usr/bin/env python3
"""2776 帧全帧回测 —— 验收"视频内 100% 正确"

低内存版:
  · 灰度用 uint8 查表 (不建 int32 中间数组)
  · 先缩放后灰度, 峰值内存 ~10MB/帧
  · 跨类别 NMS (agnostic) —— 同一位置的 right_3 / right_2 只留一个

判据:
  ① 有箭头帧: 必须且只能 1 个框, 类别 = 该段应得
  ② 无箭头帧: 不得有任何框
"""
import os
import sys
import json
import gc
import numpy as np
import cv2

B = r'D:\ADAS_Datasets'
SRC = os.path.join(B, '47_导航连续帧', 'images')
ANN = os.path.join(B, '47_导航连续帧', 'annotations_auto.json')
NAMES = ['right_3', 'right_2', 'left_3', 'left_2', 'uturn_3', 'uturn_2']

# Y = (77R + 150G + 29B) >> 8, 按 BGR 三通道分别查表
_TB = (29 * np.arange(256) >> 8).astype(np.uint16)
_TG = (150 * np.arange(256) >> 8).astype(np.uint16)
_TR = (77 * np.arange(256) >> 8).astype(np.uint16)


def gray3(path):
    im = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    im = cv2.resize(im, (1280, 720), interpolation=cv2.INTER_AREA)
    y = (_TR[im[:, :, 2]] + _TG[im[:, :, 1]] + _TB[im[:, :, 0]]).astype(np.uint8)
    return cv2.cvtColor(y, cv2.COLOR_GRAY2RGB)


def main():
    from ultralytics import YOLO
    w = os.path.join(B, '_runs', 'nav6_1280', 'weights', 'best.pt')
    conf = 0.25
    for a in sys.argv[1:]:
        if a.endswith('.pt'):
            w = a
    if '--conf' in sys.argv:
        conf = float(sys.argv[sys.argv.index('--conf') + 1])
    print('weights:', w, ' conf:', conf, flush=True)

    m = YOLO(w)
    ann = json.load(open(ANN, encoding='utf-8'))
    print('帧数', len(ann), flush=True)

    err = []
    ok_a = ok_e = 0
    for i, r in enumerate(ann):
        im = gray3(os.path.join(SRC, r['file']))
        rr = m.predict(im, imgsz=(720, 1280), conf=conf, iou=0.5,
                       agnostic_nms=True, verbose=False, max_det=5)[0]
        got = []
        for b in rr.boxes:
            x1, y1, x2, y2 = [v * 1.5 for v in b.xyxy[0].tolist()]
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if not (0 <= cx <= 720 and 470 <= cy <= 920):
                continue          # 导航卡 ROI 外: 挡风玻璃 / 仪表台
            w, h = x2 - x1, y2 - y1
            if not (90 <= w <= 270 and 45 <= h <= 225):
                continue          # 尺寸不符: 卡片 140~220x55~80 / 3D箭头 180~195x175~200
            got.append((NAMES[int(b.cls)], round(float(b.conf), 3)))
        del im, rr
        exp = None if r['cls'] is None else '%s_%d' % (r['cls'], r['digits'])
        if exp is None:
            if got:
                err.append((r['frame'], 'FP', '应为空', got))
            else:
                ok_e += 1
        elif len(got) != 1:
            err.append((r['frame'], '数量', exp, got))
        elif got[0][0] != exp:
            err.append((r['frame'], '错类', exp, got))
        else:
            ok_a += 1
        if i % 50 == 0:
            gc.collect()
            import torch
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        if i % 200 == 0:
            print('  %d/%d' % (i, len(ann)), flush=True)

    tot = len(ann)
    print()
    print('=' * 60)
    print('有箭头帧正确 %d / 无箭头帧正确 %d / 总 %d' % (ok_a, ok_e, tot))
    print('错误 %d 帧  ->  逐帧正确率 %.2f%%  (%d/%d)'
          % (len(err), 100.0 * (tot - len(err)) / tot, tot - len(err), tot))
    for e in err[:40]:
        print('   帧 %-6d %-4s 应=%-9s 实=%s' % (e[0], e[1], e[2], e[3]))
    if len(err) > 40:
        print('   ... 共 %d 处' % len(err))
    out = os.path.join(B, '_runs', 'nav6_errors.json')
    with open(out, 'w', encoding='utf-16') as f:
        json.dump([{'frame': e[0], 'type': e[1], 'expect': e[2], 'got': e[3]}
                   for e in err], f, ensure_ascii=False, indent=1)
    print('错误明细 ->', out)


if __name__ == '__main__':
    main()
