#!/usr/bin/env python3
"""RKNN 全帧回测 —— 用 build 出的 .rknn 在同一进程内跑 2776 帧

⚠️ x86 模拟器不支持 load_rknn + init_runtime, 所以必须 build -> 立刻推理 -> 再 release。
   这里把 build 和全帧回测合在一个脚本里。

判据与 PyTorch/ONNX 版完全一致:
   ROI (x<=720, 470<=y<=920) + 尺寸 (90~270 x 45~225) + 类别 + conf 0.25 / NMS 0.5
"""
import os
import json
import time
import numpy as np
import cv2

BASE = '/mnt/d/ADAS_Datasets/_deploy_nav6_blur'
ONNX = os.path.join(BASE, 'nav_arrow_split.onnx')
CALIB = '/mnt/d/ADAS_Datasets/_deploy_nav_720/calib'
SRC = '/mnt/d/ADAS_Datasets/47_导航连续帧/images'
ANN = '/mnt/d/ADAS_Datasets/47_导航连续帧/annotations_auto.json'
NAMES = ['right_3', 'right_2', 'left_3', 'left_2', 'uturn_3', 'uturn_2']
CONF, IOU = 0.25, 0.5


def load(i):
    im = cv2.imdecode(np.fromfile(os.path.join(SRC, 'nav_%06d.jpg' % i), np.uint8),
                      cv2.IMREAD_COLOR)
    im = cv2.resize(im, (1280, 720), interpolation=cv2.INTER_AREA)
    b = im[:, :, 0].astype(np.int32)
    g = im[:, :, 1].astype(np.int32)
    r = im[:, :, 2].astype(np.int32)
    y = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
    return cv2.cvtColor(y, cv2.COLOR_GRAY2RGB)


def decode(box, cls):
    box = box.reshape(4, -1).T
    cls = cls.reshape(6, -1).T
    cf = cls.max(1)
    keys = cls.argmax(1)
    m = cf > CONF
    out = []
    for b, s, k in zip(box[m], cf[m], keys[m]):
        cx, cy, w, h = b
        out.append((NAMES[int(k)], float(s),
                    [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]))
    if not out:
        return []
    arr = np.array([o[2] for o in out], np.float32)
    sco = np.array([o[1] for o in out], np.float32)
    idx = cv2.dnn.NMSBoxes(arr.tolist(), sco.tolist(), CONF, IOU)
    return [out[i] for i in np.array(idx).reshape(-1)]


def run(kind):
    from rknn.api import RKNN
    rk = RKNN(verbose=False)
    cfg = dict(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
               target_platform='rk3568', optimization_level=3)
    if kind == 'fp16':
        cfg['float_dtype'] = 'float16'
    else:
        cfg['quantized_dtype'] = 'asymmetric_quantized-8'
    assert rk.config(**cfg) == 0
    assert rk.load_onnx(model=ONNX) == 0
    fs = sorted(f for f in os.listdir(CALIB) if f.endswith('.jpg'))
    txt = os.path.join(BASE, 'calib_list.txt')
    with open(txt, 'w') as f:
        for n in fs:
            f.write(os.path.join(CALIB, n) + '\n')
    if kind == 'int8':
        assert rk.build(do_quantization=True, dataset=txt) == 0
    else:
        assert rk.build(do_quantization=False) == 0
    assert rk.init_runtime() == 0

    ann = json.load(open(ANN, encoding='utf-8'))
    err = []
    ok_a = ok_e = 0
    t0 = time.time()
    for i, r in enumerate(ann):
        out = rk.inference(inputs=[load(r['frame'])], data_format='nhwc')
        dets = decode(out[0], out[1])
        got = []
        for nm, cf, (x1, y1, x2, y2) in dets:
            x1, y1, x2, y2 = x1 * 1.5, y1 * 1.5, x2 * 1.5, y2 * 1.5
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if not (0 <= cx <= 720 and 470 <= cy <= 920):
                continue
            w, h = x2 - x1, y2 - y1
            if not (90 <= w <= 270 and 45 <= h <= 225):
                continue
            got.append((nm, round(cf, 3)))
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
        if i % 300 == 0:
            print('   %s  %d/%d' % (kind, i, len(ann)), flush=True)

    tot = len(ann)
    print()
    print('=' * 60)
    print('RKNN %s: 有箭头 %d / 无箭头 %d / 总 %d' % (kind, ok_a, ok_e, tot))
    print('错误 %d -> 逐帧正确率 %.2f%%  (%d/%d)   %.1fs'
          % (len(err), 100.0 * (tot - len(err)) / tot, tot - len(err), tot,
             time.time() - t0))
    for e in err[:20]:
        print('   帧 %-6d %-4s 应=%-9s 实=%s' % (e[0], e[1], e[2], e[3]))
    # 顺便存盘
    rk.export_rknn(os.path.join(BASE, 'nav6_720_rk3568_%s.rknn' % kind))
    rk.release()
    print()


if __name__ == '__main__':
    for k in ('fp16', 'int8'):
        run(k)
