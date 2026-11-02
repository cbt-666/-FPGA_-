#!/usr/bin/env python3
"""灯箱检测模型 -> RKNN FP16 (双平台).

用法 (WSL2):
    ~/rknn_convert/venv/bin/python conv_housing_fp16.py <best_split.onnx> <outdir>

为什么 fp16 不用 int8: 小目标单类检测, int8 量化会把小框的定位打散.
见 memory: fpga-v13-rknn-fp16-beats-int8 与 fpga-rknn-int8-kills-dash-class.
"""
import os
import sys
from rknn.api import RKNN

ONNX = sys.argv[1] if len(sys.argv) > 1 else 'housing_split.onnx'
OUTD = sys.argv[2] if len(sys.argv) > 2 else '.'
ONNX = os.path.abspath(ONNX)
os.makedirs(OUTD, exist_ok=True)

for T, tag in (('rk3568', '3568'), ('rk3576', '3576')):
    out = os.path.join(OUTD, 'housing_%s_fp16.rknn' % tag)
    print('===== %s fp16 =====' % T)
    rk = RKNN(verbose=False)
    assert rk.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                     target_platform=T, float_dtype='float16',
                     optimization_level=3) == 0, 'config'
    assert rk.load_onnx(model=ONNX) == 0, 'load'
    assert rk.build(do_quantization=False) == 0, 'build'
    assert rk.export_rknn(out) == 0, 'export'
    print('  OK %s  %.2f MB' % (os.path.basename(out), os.path.getsize(out) / 1e6))
    rk.release()
