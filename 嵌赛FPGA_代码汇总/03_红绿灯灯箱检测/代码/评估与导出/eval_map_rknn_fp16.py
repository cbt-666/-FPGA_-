"""端到端 mAP50 评估: FP32 ONNX vs INT8 RKNN (同一评估器, 对 S2TLD val 真值)

这是量化损失的唯一可信数字 —— 集合等式对边界扰动过于脆弱, 不能用作判据。

用法: python eval_map_rknn.py <onnx> <calib.txt> <val_images_dir> <val_labels_dir>
"""
import sys, os, glob
import numpy as np
import cv2
import onnxruntime as ort
from rknn.api import RKNN

sys.path.insert(0, '/mnt/d/ADAS_Datasets/_code')
from convert_verify_v13 import imread_u, letterbox, decode, iou

ONNX, CALIB, IMG_DIR, LBL_DIR = sys.argv[1:5]
CLASSES = ['red', 'green', 'yellow', 'off', 'wait_on']
NC = len(CLASSES)


def nms(dets, thr=0.45):
    out = []
    for c in set(d[0] for d in dets):
        ds = sorted([d for d in dets if d[0] == c], key=lambda z: -z[1])
        keep = []
        while ds:
            best = ds.pop(0)
            keep.append(best)
            ds = [d for d in ds if _biou(best, d) <= thr]
        out.extend(keep)
    return out


def _iou4(a, b):
    """纯框 IoU: (x1,y1,x2,y2)"""
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ab = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + ab - inter + 1e-9)


def _biou(a, b):
    """框 IoU: a/b 为 (x1,y1,x2,y2) 在第 2,3,4,5 位 (兼容 6 元组 dets)"""
    ax1, ay1, ax2, ay2 = a[2], a[3], a[4], a[5]
    bx1, by1, bx2, by2 = b[2], b[3], b[4], b[5]
    iw = min(ax2, bx2) - max(ax1, bx1)
    ih = min(ay2, by2) - max(ay1, by1)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    aa = (ax2 - ax1) * (ay2 - ay1)
    ab = (bx2 - bx1) * (by2 - by1)
    return inter / (aa + ab - inter + 1e-9)


def load_gt(lbl_path, W, H):
    out = []
    if not os.path.exists(lbl_path):
        return out
    for ln in open(lbl_path):
        p = ln.split()
        if len(p) < 5:
            continue
        c = int(float(p[0]))
        cx, cy, w, h = (float(x) for x in p[1:5])
        out.append((c, (cx - w / 2) * W, (cy - h / 2) * H,
                    (cx + w / 2) * W, (cy + h / 2) * H))
    return out


def match(preds, gts, thr):
    """返回 (tp, fp, fn). preds: [(c,x1,y1,x2,y2)]; gts: [(c,x1,y1,x2,y2)]"""
    used = [False] * len(gts)
    tp = fp = 0
    for pc, px1, py1, px2, py2 in preds:
        best, bj = 0.0, -1
        for j, (gc, gx1, gy1, gx2, gy2) in enumerate(gts):
            if used[j] or gc != pc:
                continue
            v = _iou4((px1, py1, px2, py2), (gx1, gy1, gx2, gy2))
            if v > best:
                best, bj = v, j
        if best >= thr:
            used[bj] = True
            tp += 1
        else:
            fp += 1
    fn = sum(1 for j in range(len(gts)) if not used[j])
    return tp, fp, fn


def run(imgs, infer_fn):
    """infer_fn(path) -> [dets]; 返回 每类 (tp,fp,fn) 累加 与 分数列表用于 AP"""
    per_cls = {c: {'pred': []} for c in range(NC)}
    tp_all = np.zeros(NC)
    fp_all = np.zeros(NC)
    fn_all = np.zeros(NC)
    for p in imgs:
        img = imread_u(p)
        if img is None:
            continue
        H, W = img.shape[:2]
        lbl = os.path.join(LBL_DIR, os.path.splitext(os.path.basename(p))[0] + '.txt')
        gts = load_gt(lbl, W, H)
        preds = nms(infer_fn(img))
        pd = [(c, x1, y1, x2, y2, s) for c, s, x1, y1, x2, y2 in preds]
        for c in range(NC):
            gc = [g for g in gts if g[0] == c]
            pc = [x for x in pd if x[0] == c]
            pc.sort(key=lambda z: -z[5])
            used = [False] * len(gc)
            for x in pc:
                bx1, by1, bx2, by2 = x[1], x[2], x[3], x[4]
                best, bj = 0.0, -1
                for j, g in enumerate(gc):
                    if used[j]:
                        continue
                    v = _iou4((bx1, by1, bx2, by2), (g[1], g[2], g[3], g[4]))
                    if v > best:
                        best, bj = v, j
                is_tp = best >= 0.5
                if is_tp:
                    used[bj] = True
                    tp_all[c] += 1
                else:
                    fp_all[c] += 1
                per_cls[c]['pred'].append((x[5], 1 if is_tp else 0))
            fn_all[c] += sum(1 for j in range(len(gc)) if not used[j])
    return per_cls, tp_all, fp_all, fn_all


def ap50(preds, n_gt):
    """VOC2010+ 全点插值 AP"""
    if n_gt == 0 or not preds:
        return float('nan')
    preds = sorted(preds, key=lambda z: -z[0])
    tp = np.cumsum([p[1] for p in preds])
    fp = np.cumsum([1 - p[1] for p in preds])
    rec = tp / n_gt
    prec = tp / np.maximum(tp + fp, 1e-9)
    # 单调化
    for i in range(len(prec) - 2, -1, -1):
        prec[i] = max(prec[i], prec[i + 1])
    idx = np.where(np.diff(np.concatenate([[0], rec])) > 0)[0]
    if len(idx) == 0:
        return 0.0
    return float(np.sum(np.diff(np.concatenate([[0], rec]))[idx] * prec[idx]))


def main():
    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    rk = RKNN(verbose=False)
    rk.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
              target_platform='rk3568',
              float_dtype='float16', optimization_level=3)
    assert rk.load_onnx(model=ONNX) == 0
    assert rk.build(do_quantization=False) == 0
    assert rk.init_runtime() == 0

    imgs = sorted(glob.glob(IMG_DIR + '/*.jpg'))[:300]
    print('评估图片: %d 张' % len(imgs))

    def fp32(img):
        lb, r, dx, dy = letterbox(img)
        x = (cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).transpose(2, 0, 1) / 255.0).astype(np.float32)[None]
        o = sess.run(None, {iname: x})
        return decode(o[0][0], o[1][0], r, dx, dy)

    def int8(img):
        lb, r, dx, dy = letterbox(img)
        o = rk.inference(inputs=[cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32)[None]], data_format='nhwc')
        return decode(np.array(o[0]).reshape(4, -1), np.array(o[1]).reshape(NC, -1), r, dx, dy)

    # 先统计 GT 每类数量
    ngt = np.zeros(NC)
    for p in imgs:
        im = imread_u(p)
        if im is None:
            continue
        H, W = im.shape[:2]
        for g in load_gt(os.path.join(LBL_DIR, os.path.splitext(os.path.basename(p))[0] + '.txt'), W, H):
            ngt[g[0]] += 1

    res = {}
    for name, fn in [('FP32', fp32), ('FP16', int8)]:
        per_cls, tp, fp, fn_ = run(imgs, fn)
        aps = []
        print('\n--- %s ---' % name)
        print('%-8s %6s %6s %6s %8s' % ('类', 'TP', 'FP', 'FN', 'AP50'))
        for c in range(NC):
            a = ap50(per_cls[c]['pred'], int(ngt[c]))
            aps.append(a if not np.isnan(a) else float('nan'))
            print('%-8s %6d %6d %6d %8s' %
                  (CLASSES[c], int(tp[c]), int(fp[c]), int(fn_[c]),
                   ('%.4f' % a) if not np.isnan(a) else '  n/a'))
        valid = [a for a in aps if not np.isnan(a)]
        m = float(np.mean(valid)) if valid else float('nan')
        res[name] = m
        print('mAP50 (有效类均值) = %.4f' % m)

    print()
    print('=' * 46)
    print('FP32 mAP50 : %.4f' % res['FP32'])
    print('FP16 mAP50 : %.4f' % res['FP16'])
    print('fp16 损失  : %+.4f  (%+.2f%%)' %
          (res['FP16'] - res['FP32'],
           100 * (res['FP16'] - res['FP32']) / max(1e-9, res['FP32'])))
    print('=' * 46)
    rk.release()


if __name__ == '__main__':
    main()
