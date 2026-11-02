"""验证转换配置下 rknn 输出与 onnx 一致 (simulator: load_onnx->build->init_runtime)."""
import os
import numpy as np
import cv2
from rknn.api import RKNN

VAL = '/mnt/d/ADAS_Datasets/42_housing_1c/val/images'
ONNX = '/mnt/d/ADAS_Datasets/_code/housing_split.onnx'


def main():
    import onnxruntime as ort
    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])

    rk = RKNN(verbose=False)
    assert rk.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                     target_platform='rk3568', float_dtype='float16') == 0
    assert rk.load_onnx(model=ONNX) == 0
    assert rk.build(do_quantization=False) == 0
    assert rk.init_runtime(target=None) == 0

    files = sorted(f for f in os.listdir(VAL) if not f.endswith('_g.png'))[:5]
    worst = 0.0
    for f in files:
        im = cv2.imdecode(np.fromfile(os.path.join(VAL, f), np.uint8), cv2.IMREAD_COLOR)
        rgb = cv2.cvtColor(cv2.resize(im, (1280, 768)), cv2.COLOR_BGR2RGB).astype(np.float32)
        o_rk = rk.inference(inputs=[rgb], data_format='nhwc')[0].reshape(-1)
        o_on = sess.run(None, {sess.get_inputs()[0].name:
                               rgb.transpose(2, 0, 1)[None]})[0].reshape(-1)
        d = float(np.abs(o_rk - o_on).max())
        worst = max(worst, d)
        print('  %-28s maxdiff %.4f  maxscore %.3f'
              % (f, d, float(o_rk[4:].max())))
    rk.release()
    print('\n最大绝对误差 %.4f (fp16 正常应 < 0.02)' % worst)


if __name__ == '__main__':
    main()
