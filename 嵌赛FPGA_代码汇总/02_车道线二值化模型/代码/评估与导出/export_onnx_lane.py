#!/usr/bin/env python3
"""车道 3 类分割模型 -> ONNX（RKNN 转换的输入）

导出后用 onnxruntime 跑一遍，与 PyTorch 输出逐像素比对，
防止出现"导出成功但语义变了"的情况。
"""
import os
import sys
import argparse
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from train_lane_seg import Net                                # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--w', type=int, default=1280)
    ap.add_argument('--h', type=int, default=720)
    a = ap.parse_args()

    net = Net(pretrained=False)
    net.load_state_dict(torch.load(a.ckpt, map_location='cpu'))
    net.eval()

    dummy = torch.randn(1, 3, a.h, a.w)
    torch.onnx.export(
        net, dummy, a.out,
        input_names=['input'], output_names=['logits'],
        opset_version=12, do_constant_folding=True,
        dynamic_axes=None)

    # ⚠️ torch 2.14 的导出器默认把 >2GB 之外的权重也写成**外部数据文件**
    # （xxx.onnx + xxx.onnx.data）。只拷 .onnx 会导致
    # "External data path validation failed" —— 交付时极易漏。
    # 这里强制重新保存为**单文件**（权重内嵌），一次拷贝即可用，RKNN 也更省事。
    import onnx
    m = onnx.load(a.out)                      # 会自动把 .data 读进来
    onnx.save_model(m, a.out, save_as_external_data=False)
    side = a.out + '.data'
    if os.path.exists(side):
        os.remove(side)
    print('导出 %s  (%.2f MB, 单文件)' % (a.out, os.path.getsize(a.out) / 1e6))

    # ---- 校验：onnxruntime vs pytorch ----
    try:
        import onnxruntime as ort
    except ImportError:
        print('onnxruntime 未安装，跳过校验')
        return
    with torch.no_grad():
        pt = net(dummy).numpy()
    sess = ort.InferenceSession(a.out, providers=['CPUExecutionProvider'])
    ox = sess.run(None, {'input': dummy.numpy()})[0]
    d = np.abs(pt - ox)
    am_pt = pt.argmax(1)
    am_ox = ox.argmax(1)
    print('  最大绝对误差 %.3e | 平均 %.3e' % (d.max(), d.mean()))
    print('  argmax 不一致像素 %d / %d (%.4f%%)'
          % ((am_pt != am_ox).sum(), am_pt.size,
             100.0 * (am_pt != am_ox).sum() / am_pt.size))


if __name__ == '__main__':
    main()
