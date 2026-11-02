#!/usr/bin/env python3
"""ONNX 全帧回测 —— 验证 720 导出没有掉点

与 eval_nav6_full.py 同一套判据 (ROI + 尺寸 + agnostic NMS + conf0.25/iou0.5),
只是把推理后端从 PyTorch 换成 ONNX Runtime。
"""
import os
import json
import numpy as np
import cv2
import onnxruntime as ort

B = r'D:\ADAS_Datasets'
SRC = os.path.join(B, '47_导航连续帧', 'images')
ANN = os.path.join(B, '47_导航连续帧', 'annotations_auto.json')
ONNX = os.path.join(B, '_deploy_nav6_blur', 'nav_arrow_split.onnx')
NAMES = ['right_3', 'right_2', 'left_3', 'left_2', 'uturn_3', 'uturn_2']
CONF, IOU = 0.25, 0.5

_TB = (29 * np.arange(256) >> 8).astype(np.uint16)
_TG = (150 * np.arange(256) >> 8).astype(np.uint16)
_TR = (77 * np.arange(256) >> 8).astype(np.uint16)


def load(path):
    im = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    im = cv2.resize(im, (1280, 720), interpolation=cv2.INTER_AREA)
    y = (_TR[im[:, :, 2]] + _TG[im[:, :, 1]] + _TB[im[:, :, 0]]).astype(np.uint8)
    return cv2.cvtColor(y, cv2.COLOR_GRAY2RGB)


def decode(box, cls):
    """box[1,4,N] cls[1,6,N] -> [(cls_name, conf, xyxy)]"""
    box = box[0].T          # N,4  (cx,cy,w,h 在 1280x720 坐标)
    cls = cls[0].T          # N,6
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


def main():
    s = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    inp = s.get_inputs()[0].name
    ann = json.load(open(ANN, encoding='utf-8'))
    err = []
    ok_a = ok_e = 0
    for i, r in enumerate(ann):
        im = load(os.path.join(SRC, r['file']))
        x = im.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        o = s.run(None, {inp: x})
        dets = decode(o[0], o[1])
        got = []
        for nm, cf, (x1, y1, x2, y2) in dets:
            # 反算回 1920x1080 判 ROI / 尺寸 (与 PyTorch 版一致)
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
            print('  %d/%d' % (i, len(ann)), flush=True)
    tot = len(ann)
    print()
    print('=' * 60)
    print('ONNX 720: 有箭头帧正确 %d / 无箭头帧正确 %d / 总 %d' % (ok_a, ok_e, tot))
    print('错误 %d -> 逐帧正确率 %.2f%%  (%d/%d)'
          % (len(err), 100.0 * (tot - len(err)) / tot, tot - len(err), tot))
    for e in err[:20]:
        print('   帧 %-6d %-4s 应=%-9s 实=%s' % (e[0], e[1], e[2], e[3]))


if __name__ == '__main__':
    main()
