"""验证 v13 RKNN INT8 量化精度 —— 对比 FP32 ONNX

核心要查两件事：
  1) 类别分数有没有被量化压成 0（拆 Concat 前的老毛病）
  2) FP32 与 INT8 的检测结果一致率
"""
import os
import numpy as np
import cv2
import glob
import onnxruntime as ort
from rknn.api import RKNN

RKNN_MODEL = '/mnt/d/ADAS_Datasets/_runs/s2_v13/weights/traffic_light_v13_rk3568.rknn'
ONNX = '/mnt/d/ADAS_Datasets/_runs/s2_v13/weights/best_split.onnx'
VAL = '/mnt/d/ADAS_Datasets/08_中国红绿灯/S2TLD_v13/val/images'
CLASSES = ['red', 'green', 'yellow', 'off', 'wait_on']
CONF = 0.35


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def letterbox(img, new=640, color=114):
    h, w = img.shape[:2]
    r = min(new / h, new / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (h, w) != (nh, nw):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new - nh) // 2
    bot = new - nh - top
    left = (new - nw) // 2
    right = new - nw - left
    return cv2.copyMakeBorder(img, top, bot, left, right,
                              cv2.BORDER_CONSTANT, value=(color,) * 3), r, left, top


def decode(box, cls, ratio, dx, dy, conf=CONF):
    """box (4,N) cxcywh-letterbox, cls (nc,N) -> [(cls_id, score, x1,y1,x2,y2)] 原图坐标"""
    box = box.T
    cls = cls.T
    sc = cls.max(1)
    ids = cls.argmax(1)
    m = sc > conf
    box, sc, ids = box[m], sc[m], ids[m]
    out = []
    for b, s, c in zip(box, sc, ids):
        cx, cy, w, h = b
        x1 = (cx - w / 2 - dx) / ratio
        y1 = (cy - h / 2 - dy) / ratio
        x2 = (cx + w / 2 - dx) / ratio
        y2 = (cy + h / 2 - dy) / ratio
        out.append((int(c), float(s), x1, y1, x2, y2))
    return out


def iou(a, b):
    x1 = max(a[2], b[2]); y1 = max(a[3], b[3])
    x2 = min(a[4], b[4]); y2 = min(a[5], b[5])
    iw = max(0.0, x2 - x1); ih = max(0.0, y2 - y1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    aa = (a[4] - a[2]) * (a[5] - a[3])
    ab = (b[4] - b[2]) * (b[5] - b[3])
    return inter / (aa + ab - inter + 1e-9)


def main():
    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    print('ONNX outputs:', [(o.name, o.shape) for o in sess.get_outputs()])

    rk = RKNN(verbose=False)
    assert rk.load_rknn(RKNN_MODEL) == 0
    assert rk.init_runtime() == 0
    print('RKNN loaded')

    imgs = sorted(glob.glob(VAL + '/*.jpg'))[:200]
    print('val images:', len(imgs))

    # 探一下 RKNN 输出形状
    probe = imread_u(imgs[0])
    lb, ratio, dx, dy = letterbox(probe)
    rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
    rout = rk.inference(inputs=[rgb])
    print('RKNN outputs:', [np.array(o).shape for o in rout])
    print('RKNN out dtypes:', [np.array(o).dtype for o in rout])
    print('RKNN out ranges:', [(float(np.min(o)), float(np.max(o))) for o in rout])

    n_exact = 0          # 类别集合完全一致
    tot = 0
    iou_sum = 0.0
    iou_n = 0
    score_drop = []
    blank_fp = 0         # fp32 有检出但 int8 全空
    blank_rk = 0

    for p in imgs:
        img = imread_u(p)
        if img is None:
            continue
        lb, ratio, dx, dy = letterbox(img)

        x = (cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).transpose(2, 0, 1) / 255.0).astype(np.float32)[None]
        o = sess.run(None, {iname: x})
        d_fp = decode(o[0][0], o[1][0], ratio, dx, dy)

        rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
        r = rk.inference(inputs=[rgb])
        d_rk = decode(np.array(r[0]).reshape(4, -1), np.array(r[1]).reshape(len(CLASSES), -1),
                      ratio, dx, dy)

        a = sorted(CLASSES[c] for c, *_ in d_fp)
        b = sorted(CLASSES[c] for c, *_ in d_rk)
        tot += 1
        if a == b:
            n_exact += 1
        if a and not b:
            blank_rk += 1
        if not a and b:
            blank_fp += 1
        # 匹配框 IoU + 分数
        for df in d_fp:
            best, bd = 0.0, None
            for dr in d_rk:
                if dr[0] != df[0]:
                    continue
                v = iou(df, dr)
                if v > best:
                    best, bd = v, dr
            if bd is not None:
                iou_sum += best
                iou_n += 1
                score_drop.append(df[1] - bd[1])

    print()
    print('=' * 56)
    print('类别集合完全一致 : %d/%d = %.1f%%' % (n_exact, tot, 100 * n_exact / max(1, tot)))
    print('平均匹配框 IoU   : %.4f  (n=%d)' % (iou_sum / max(1, iou_n), iou_n))
    sd = np.array(score_drop) if score_drop else np.array([0.0])
    print('置信度下降       : 中位 %.4f  均值 %.4f  p95 %.4f' %
          (np.median(sd), sd.mean(), np.percentile(sd, 95)))
    print('FP32有检出/INT8空 : %d 张' % blank_rk)
    print('FP32空/INT8有检出 : %d 张' % blank_fp)
    print('=' * 56)
    rk.release()


if __name__ == '__main__':
    main()
