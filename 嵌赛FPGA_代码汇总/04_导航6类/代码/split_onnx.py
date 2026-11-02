"""拆分 YOLO ONNX 末端的 Concat(box, cls) —— RKNN INT8 量化必需

不拆的后果: 框坐标(-5.6~638)和类别分数(0~0.96)共用同一个量化 scale,
scale 被框主导, 所有类别分数被压成 0 → 模型完全失效。

用法: python split_onnx.py <src.onnx> <dst.onnx> <nc>
"""
import sys
import numpy as np
import onnx
from onnx import helper, TensorProto


def main():
    src, dst, nc = sys.argv[1], sys.argv[2], int(sys.argv[3])
    m = onnx.load(src)
    g = m.graph
    last = g.node[-1]
    assert last.op_type == 'Concat', '末尾不是 Concat，实际是 %s' % last.op_type
    box_t, cls_t = list(last.input)          # YOLO 固定顺序: 先 box 后 cls
    print('box tensor:', box_t)
    print('cls tensor:', cls_t)

    # 推断空间维 (8400 = 80*80 + 40*40 + 20*20)
    orig = g.output[0]
    dims = [d.dim_value for d in orig.type.tensor_type.shape.dim]
    assert len(dims) == 3, dims
    N = dims[2]
    print('output %s shape=%s -> 拆成 (1,4,%d) + (1,%d,%d)' % (orig.name, dims, N, nc, N))

    del g.node[-1]
    del g.output[:]

    def vi(n, c):
        return helper.make_tensor_value_info(n, TensorProto.FLOAT, [1, c, N])

    g.output.extend([vi(box_t, 4), vi(cls_t, nc)])
    onnx.checker.check_model(m)
    onnx.save(m, dst)
    print('saved:', dst)

    # --- 等价性验证: 拆分前后输出必须完全一致 ---
    import onnxruntime as ort
    rng = np.random.default_rng(0)
    x = rng.random((1, 3, 640, 640), dtype=np.float32)

    s1 = ort.InferenceSession(src, providers=['CPUExecutionProvider'])
    a = s1.run(None, {s1.get_inputs()[0].name: x})[0]      # (1, 4+nc, N)

    s2 = ort.InferenceSession(dst, providers=['CPUExecutionProvider'])
    b = s2.run(None, {s2.get_inputs()[0].name: x})
    box, cls = b[0], b[1]                                  # (1,4,N), (1,nc,N)

    cat = np.concatenate([box, cls], axis=1)
    d = np.abs(a - cat).max()
    print('拆分前后最大差异: %.3e' % d)
    assert d == 0.0, '拆分不等价!'
    print('box range   : %.2f ~ %.2f' % (box.min(), box.max()))
    print('cls range   : %.4f ~ %.4f' % (cls.min(), cls.max()))
    print('OK: 拆分等价, 量级差 %.0f 倍 (证明必须分开量化)' %
          (abs(box).max() / max(1e-6, abs(cls).max())))


if __name__ == '__main__':
    main()
