#!/usr/bin/env python3
"""导航6类 720 -> RKNN (rk3568), fp16 + int8

输入契约:
  · 1x3x720x1280, RGB, 值域 0~255
  · mean=[[0,0,0]] / std=[[255,255,255]]   (ONNX 首节点直接是 Conv)
  · 必须先做亮度灰度 Y=(77R+150G+29B)>>8 再复制成 3 通道
    CN 红绿保留变换会把整块导航屏打成黑屏(地图底色是蓝的)

⚠️ ONNX 已打 FPN Concat 补丁(720 非 32 倍数), 不能换别处的 720 模型。
⚠️ load_rknn+init_runtime 在 x86 模拟器不支持, 验证必须在 build 后同一进程做。
"""
import os
import time

BASE = '/mnt/d/ADAS_Datasets/_deploy_nav6_blur'
ONNX = os.path.join(BASE, 'nav_arrow_split.onnx')
CALIB = '/mnt/d/ADAS_Datasets/_deploy_nav_720/calib'
OUTD = '/mnt/d/ADAS_Datasets/_deploy_nav6_blur'
PLATFORM = 'rk3568'
MEAN = [[0, 0, 0]]
STD = [[255, 255, 255]]


def calib_list():
    fs = sorted(f for f in os.listdir(CALIB) if f.endswith('.jpg'))
    txt = os.path.join(OUTD, 'calib_list.txt')
    with open(txt, 'w') as f:
        for n in fs:
            f.write(os.path.join(CALIB, n) + '\n')
    return txt, len(fs)


def run_check(rk, kind, img='/mnt/d/ADAS_Datasets/47_导航连续帧/images/nav_000600.jpg'):
    import cv2
    import numpy as np
    assert rk.init_runtime() == 0, 'init_runtime'
    im = cv2.imdecode(np.fromfile(img, np.uint8), cv2.IMREAD_COLOR)
    b = im[:, :, 0].astype(np.int32)
    g = im[:, :, 1].astype(np.int32)
    r = im[:, :, 2].astype(np.int32)
    y = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
    y3 = cv2.cvtColor(cv2.resize(y, (1280, 720)), cv2.COLOR_GRAY2RGB)
    outs = rk.inference(inputs=[y3], data_format='nhwc')
    cls = np.asarray(outs[1]).reshape(6, -1)
    cf = cls.max(0)
    print('  [%s] verify: cls max %.3f   >0.25 锚点 %d'
          % (kind, cf.max(), int((cf > 0.25).sum())), flush=True)


def build(kind, calib=None):
    from rknn.api import RKNN
    t0 = time.time()
    out = os.path.join(OUTD, 'nav6_720_%s_%s.rknn' % (PLATFORM, kind))
    rk = RKNN(verbose=False)
    cfg = dict(mean_values=MEAN, std_values=STD, target_platform=PLATFORM,
               optimization_level=3)
    if kind == 'fp16':
        cfg['float_dtype'] = 'float16'
    else:
        cfg['quantized_dtype'] = 'asymmetric_quantized-8'
    assert rk.config(**cfg) == 0, 'config'
    assert rk.load_onnx(model=ONNX) == 0, 'load_onnx'
    if kind == 'int8':
        assert calib, 'int8 needs calib'
        assert rk.build(do_quantization=True, dataset=calib) == 0, 'build'
    else:
        assert rk.build(do_quantization=False) == 0, 'build'
    try:
        run_check(rk, kind)
    except Exception as e:
        print('  [%s] verify skipped: %s' % (kind, e), flush=True)
    assert rk.export_rknn(out) == 0, 'export'
    print('  [%s] %s  %.2f MB  %.0fs'
          % (kind, os.path.basename(out), os.path.getsize(out) / 1e6,
             time.time() - t0), flush=True)
    rk.release()
    return out


if __name__ == '__main__':
    calib, n = calib_list()
    print('校准图 %d 张 -> %s' % (n, calib), flush=True)
    for kind in ('fp16', 'int8'):
        print('=== %s ===' % kind, flush=True)
        build(kind, calib)
    print('DONE ->', OUTD)
