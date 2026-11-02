"""验证新灯箱 rknn (fp16, 双平台) 与 ONNX 一致, 并在纯红绿 val 上端到端评精度.

契约:
    onnx  : 输入 RGB float32 0~1, NHWC
    rknn  : 输入 RGB float32 0~255, NHWC (config std=255 内部除 255)
  输出  : box [1,4,20160] 像素 xywh ; cls [1,1,20160] 已 sigmoid

用法 (WSL):
    ~/rknn_convert/venv/bin/python verify_housing_rg_rknn.py [rk3568|rk3576]
"""
import os
import sys
import numpy as np
import cv2
from rknn.api import RKNN

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

VAL = '/mnt/d/ADAS_Datasets/42_housing_1c_rg/val'
# 优先用本脚本同目录下的文件 (交付包里就是这种布局), 找不到再退回开发机路径
_HERE = os.path.dirname(os.path.abspath(__file__))
ONNX = next((p for p in (os.path.join(_HERE, 'housing_split.onnx'),
                         '/mnt/d/ADAS_Datasets/_code/housing_rg_split.onnx')
             if os.path.isfile(p)), os.path.join(_HERE, 'housing_split.onnx'))
RK_DIR = _HERE if os.path.isfile(
    os.path.join(_HERE, 'housing_3568_fp16.rknn')) else '/mnt/d/ADAS_Datasets/_code/rknn_out_rg'
H, W = 768, 1280
CONF_SCAN = (0.15, 0.20, 0.25, 0.30)


def imread(p):
    return cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)


def nms(boxes, scores, thr):
    """boxes: Nx4 xyxy ; scores: N"""
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        a = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        b = ((boxes[rest, 2] - boxes[rest, 0]) *
             (boxes[rest, 3] - boxes[rest, 1]))
        iou = inter / np.maximum(a + b - inter, 1e-9)
        order = rest[iou <= thr]
    return keep


def decode(box, cls, conf, iou_thr=0.45):
    """box [4,N] 像素 xywh, cls [1,N] -> xyxy 列表"""
    s = cls.reshape(-1)
    m = s >= conf
    if not m.any():
        return np.zeros((0, 4), np.float32)
    b = box.reshape(4, -1)[:, m].T
    sc = s[m]
    xyxy = np.stack([b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2,
                     b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2], 1)
    keep = nms(xyxy, sc, iou_thr)
    return xyxy[keep]


def iou_one(a, b):
    xx1, yy1 = max(a[0], b[0]), max(a[1], b[1])
    xx2, yy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0., xx2 - xx1) * max(0., yy2 - yy1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.


def count(pred, gt, thr=0.5):
    used, tp = set(), 0
    for p in pred:
        best, bi = 0., -1
        for i, g in enumerate(gt):
            if i in used:
                continue
            v = iou_one(p, g)
            if v > best:
                best, bi = v, i
        if best >= thr and bi >= 0:
            used.add(bi); tp += 1
    return tp, len(pred) - tp, len(gt) - tp


def load_gt(lb):
    if not os.path.isfile(lb):
        return []
    out = []
    for l in open(lb):
        q = l.split()
        if len(q) < 5:
            continue
        cx, cy, bw, bh = [float(x) for x in q[1:5]]
        out.append([(cx - bw / 2) * W, (cy - bh / 2) * H,
                    (cx + bw / 2) * W, (cy + bh / 2) * H])
    return out


def main():
    plat = sys.argv[1] if len(sys.argv) > 1 else 'rk3568'
    tag = '3568' if '3568' in plat else '3576'
    rknn_p = os.path.join(RK_DIR, 'housing_%s_fp16.rknn' % tag)

    import onnxruntime as ort
    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])

    rk = RKNN(verbose=False)
    # 模拟器不支持 load_rknn, 用与正式转换逐项相同的配置重建
    assert rk.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                     target_platform=plat, float_dtype='float16',
                     optimization_level=3) == 0, 'config'
    assert rk.load_onnx(model=ONNX) == 0, 'load_onnx'
    assert rk.build(do_quantization=False) == 0, 'build'
    assert rk.init_runtime(target=None) == 0, 'init_runtime'

    files = sorted(f for f in os.listdir(os.path.join(VAL, 'images'))
                   if f.endswith('.png') and not f.endswith('_g.png'))
    print('纯红绿 val: %d 图 (平台 %s)' % (len(files), plat))

    # ---------- 1) 数值一致性 ----------
    worst_box = worst_cls = 0.0
    for f in files[:6]:
        im = imread(os.path.join(VAL, 'images', f))
        rgb01 = cv2.cvtColor(cv2.resize(im, (W, H)),
                             cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        o_rk = rk.inference(inputs=[rgb01[None] * 255.0], data_format='nhwc')
        o_on = sess.run(None, {sess.get_inputs()[0].name:
                               rgb01.transpose(2, 0, 1)[None]})
        db = float(np.abs(np.asarray(o_rk[0]) - o_on[0]).max())
        dc = float(np.abs(np.asarray(o_rk[1]) - o_on[1]).max())
        worst_box = max(worst_box, db); worst_cls = max(worst_cls, dc)
    print('  与 ONNX 对拍: 框最大误差 %.3f px | 分数最大误差 %.4f' % (worst_box, worst_cls))

    # ---------- 2) 端到端精度 ----------
    print('  %-7s %5s %5s %5s %8s %8s' % ('conf', 'TP', 'FP', 'FN', '召回', '精确'))
    cache = []
    for f in files:
        im = imread(os.path.join(VAL, 'images', f))
        # rknn 契约: RGB float32 0~255 (config std=255 内部归一化)
        rgb = cv2.cvtColor(cv2.resize(im, (W, H)),
                           cv2.COLOR_BGR2RGB).astype(np.float32)
        o = rk.inference(inputs=[rgb[None]], data_format='nhwc')
        cache.append((f, np.asarray(o[0]), np.asarray(o[1])))

    for c in CONF_SCAN:
        TP = FP = FN = 0
        for f, box, cls in cache:
            gt = load_gt(os.path.join(VAL, 'labels',
                                      os.path.splitext(f)[0] + '.txt'))
            tp, fp, fn = count(decode(box, cls, c), gt)
            TP += tp; FP += fp; FN += fn
        print('  %-7.2f %5d %5d %5d %7.1f%% %7.1f%%'
              % (c, TP, FP, FN, 100 * TP / max(TP + FN, 1),
                 100 * TP / max(TP + FP, 1)))

    rk.release()


if __name__ == '__main__':
    main()
