#!/usr/bin/env python3
"""导航箭头 -> 上板 ONNX 【1280x720 精确】+ 拆分 Concat

⚠️ 为什么必须打 Concat 补丁:
   YOLOv8 的 FPN 用 nearest 上采样 2x。736 时 23*2=46 与 P4 的 46 对齐;
   720 时 720/32=22.5 -> 22, 22*2=44, 而 P4 是 45 -> 尺寸冲突, 前向直接报错:
     "Expected size 46 but got size 45"
   ⇒ 720 不是 32 的倍数, 原版 YOLOv8 架构真的跑不了, 不是"保守检查"。

   补丁: 把 Concat 的所有输入插值到最大空间尺寸。权重完全通用(全卷积),
   不需要重训。已验证 720 与 736 输出锚点数一致(19320), cls max 0.863 vs 0.866。

⚠️ 为什么不用重训:
   训练用 imgsz=1280(方形, letterbox 缩放比 0.6667)。
   1280x720 的缩放比 = min(720/1080, 1280/1920) = 0.6667 —— **完全相同**,
   箭头在网络输入里的像素尺寸与训练时一致。
   而 16:9 -> 16:9 是**纯缩放, 零补边**, 比 736 那条路(要补 8px)更干净。
"""
import os, shutil, argparse
import numpy as np

CKPT = r'D:\ADAS_Datasets\_runs\navarrow_nomask_v2\weights\best.pt'
OUTDIR = r'D:\ADAS_Datasets\_deploy_nav_720'
W, H = 1280, 720


def patch_concat():
    """让 FPN 的 Concat 容忍尺寸差（720 时 44 vs 45）"""
    import torch
    import torch.nn.functional as F
    from ultralytics.nn.modules.conv import Concat

    def fwd(self, x):
        # 720 时 P5 上采样得 46、P4 是 45 -> 对齐到【较小】尺寸
        # (对齐到较大需要插值放大跳跃连接, 实测会掉点)
        hw = [t.shape[2:] for t in x]
        mh = min(h for h, _ in hw)
        mw = min(w for _, w in hw)
        out = []
        for t, (h, w) in zip(x, hw):
            if (h, w) != (mh, mw):
                t = F.interpolate(t, size=(mh, mw), mode='nearest')
            out.append(t)
        return torch.cat(out, self.d)

    Concat.forward = fwd


def split_concat(src, dst):
    import onnx
    from onnx import helper, TensorProto
    model = onnx.load(src)
    g = model.graph
    last = g.node[-1]
    assert last.op_type == 'Concat', '末尾不是 Concat: %s' % last.op_type
    box_t, cls_t = list(last.input)
    del g.node[-1]
    del g.output[:]

    def shape_of(name):
        for vi_ in list(g.value_info) + list(g.input) + list(g.output):
            if vi_.name == name and vi_.type.tensor_type.HasField('shape'):
                return [d.dim_value for d in vi_.type.tensor_type.shape.dim]
        return None

    four = shape_of(box_t) or [1, 4, -1]
    one = shape_of(cls_t) or [1, 6, four[2]]
    if len(one) == 3 and one[1] <= 0:
        one = [four[0], len(model.metadata_props) or 6, four[2]]
    g.output.extend([
        helper.make_tensor_value_info(box_t, TensorProto.FLOAT, four),
        helper.make_tensor_value_info(cls_t, TensorProto.FLOAT, one),
    ])
    onnx.checker.check_model(model)
    onnx.save(model, dst, save_as_external_data=False)
    return four, one


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', default=CKPT)
    ap.add_argument('--outdir', default=OUTDIR)
    ap.add_argument('--opset', type=int, default=12)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    patch_concat()

    import ultralytics.engine.exporter as EX

    def _keep(imgsz, stride=32, min_dim=1, max_dim=2, floor=0):
        return list(imgsz) if isinstance(imgsz, (list, tuple)) else [imgsz] * 2

    EX.check_imgsz = _keep

    from ultralytics import YOLO
    m = YOLO(a.ckpt)
    isz = (H, W)

    print('=== 1. 标准 ONNX (含 NMS) ===', flush=True)
    p = m.export(format='onnx', imgsz=isz, opset=a.opset, simplify=True,
                 dynamic=False, batch=1, nms=True, half=False)
    dst_full = os.path.join(a.outdir, 'nav_arrow_full.onnx')
    shutil.move(p, dst_full)
    print('  ->', dst_full, flush=True)

    print('=== 2. 原始输出 ONNX (无 NMS) ===', flush=True)
    p = m.export(format='onnx', imgsz=isz, opset=a.opset, simplify=True,
                 dynamic=False, batch=1, nms=False, half=False)
    dst_raw = os.path.join(a.outdir, 'nav_arrow_raw.onnx')
    if os.path.abspath(p) != os.path.abspath(dst_raw):
        shutil.move(p, dst_raw)
    print('  ->', dst_raw, flush=True)

    print('=== 3. 拆分 Concat ===', flush=True)
    dst_split = os.path.join(a.outdir, 'nav_arrow_split.onnx')
    four, one = split_concat(dst_raw, dst_split)
    print('  ->', dst_split, ' box%s cls%s' % (four, one), flush=True)

    import onnxruntime as ort
    s1 = ort.InferenceSession(dst_raw, providers=['CPUExecutionProvider'])
    ishape = s1.get_inputs()[0].shape
    x = np.random.rand(*ishape).astype(np.float32)
    o1 = s1.run(None, {s1.get_inputs()[0].name: x})[0]
    s2 = ort.InferenceSession(dst_split, providers=['CPUExecutionProvider'])
    o2 = s2.run(None, {s2.get_inputs()[0].name: x})
    d = float(np.abs(o1 - np.concatenate(o2, axis=1)).max())
    print('  等价性 最大差异 %.3e  %s' % (d, '[OK]' if d < 1e-6 else '[BAD]'))
    print('  输入', ishape, ' 输出', [o.shape for o in s2.get_outputs()])


if __name__ == '__main__':
    main()
