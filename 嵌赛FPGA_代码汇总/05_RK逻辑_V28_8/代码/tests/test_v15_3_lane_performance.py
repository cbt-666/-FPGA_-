"""Lane model v2 (G640 grey) adapter contract and argmax equivalence.

The v2 model takes 1 x 1 x H x W float32 in 0..255 and returns 1 x 3 x H x W
logits. Everything else — /255 and the ImageNet normalisation — is baked into
the graph, so the adapter must not touch it.

⚠️ 2026-10-31：二值版 A768 连同它的 MODE 2 采集一起删除。车道只有 G640
（灰度 640×360），预处理只有一次缩放，RK 端不允许出现任何阈值运算。
"""
import unittest
import numpy as np
import models_runtime
import lane_v2


class FakeLaneHandle:
    def __init__(self, outputs):
        self.outputs = outputs
        self.last_infer_ms = 2.5
        self.inputs = []
        self.kwargs = []
    def run(self, value, **kwargs):
        self.inputs.append(value.copy())
        self.kwargs.append(kwargs)
        return self.outputs


class LaneV2Tests(unittest.TestCase):
    def adapter(self, output, h=360, w=640):
        obj = models_runtime.LazyDetectors.__new__(models_runtime.LazyDetectors)
        fake = FakeLaneHandle([output])
        obj.loaded = {'lane': fake}
        obj.last_model_stages = {}
        obj.lane_variant = models_runtime.LANE_VARIANT
        obj._cached_yrg = None
        obj._cached_gray = None
        obj.get = lambda name: fake
        return obj, fake

    def test_gray640_input_contract(self):
        """G640 吃 YRG 灰度：亮度公式取灰后只缩放，不做阈值。"""
        gray = np.zeros((720, 1280), np.uint8)
        gray[300:480, 400:420] = 137          # grey stays grey, no threshold
        obj, fake = self.adapter(np.zeros((1, 3, 360, 640), np.float32))
        obj.lane_mask(gray)
        sample = fake.inputs[0]
        self.assertEqual(sample.shape, (1, 1, 360, 640))
        self.assertEqual(sample.dtype, np.float32)
        self.assertGreater(sample.max(), 100)
        self.assertEqual(fake.kwargs[-1], {'data_format': ['nchw'], 'data_type': 'float32'})
        self.assertEqual(obj.last_model_stages['lane']['variant'], 'gray640')
        self.assertEqual(obj.last_model_stages['lane']['source'], 'grey')

    def test_red_and_green_markers_map_to_their_brightness(self):
        """YRG 254/255 是红/绿标记，亮度公式把它们折成 76/149。"""
        x = np.zeros((720, 1280), np.uint8)
        x[100:400, 100:600] = 255          # 纯绿 -> 149
        obj, fake = self.adapter(np.zeros((1, 3, 360, 640), np.float32))
        obj.lane_mask(x)
        vals = set(np.unique(fake.inputs[0]).tolist())
        self.assertEqual(vals, {0.0, 149.0}, f'纯绿应折成 149，得到 {vals}')

    def test_no_threshold_anywhere_on_the_grey_path(self):
        """同一片灰阶从 0 到 253 铺满，输出必须逐级保留（不允许被压成 0/255）。"""
        g = np.tile(np.arange(254, dtype=np.uint8), (720, 1))[:, :1280]
        out = lane_v2.prepare(g, 'gray640')[0, 0]
        self.assertGreater(len(np.unique(out)), 100,
                           '灰度输入必须保留大量级数，说明没有做阈值')

    def test_exact_argmax_nchw_including_ties(self):
        rng = np.random.RandomState(5)
        a = rng.standard_normal((1, 3, 360, 640)).astype(np.float32)
        a[:, 1, 100:160, 100:150] = a[:, 0, 100:160, 100:150]
        a[:, 2, 120:130, 110:120] = a[:, 1, 120:130, 110:120]
        obj, _ = self.adapter(a)
        out = obj.lane_mask(np.zeros((720, 1280), np.uint8))
        self.assertTrue(np.array_equal(out, np.argmax(a[0], axis=0).astype(np.uint8)))

    def test_exact_argmax_nhwc_including_ties(self):
        rng = np.random.RandomState(6)
        a = rng.standard_normal((1, 360, 640, 3)).astype(np.float32)
        a[:, 100:150, 100:150, 1] = a[:, 100:150, 100:150, 0]
        obj, _ = self.adapter(a)
        out = obj.lane_mask(np.zeros((720, 1280), np.uint8))
        self.assertTrue(np.array_equal(out, np.argmax(a[0], axis=-1).astype(np.uint8)))

    def test_repeated_calls_do_not_leak_previous_frame(self):
        obj, fake = self.adapter(np.zeros((1, 3, 360, 640), np.float32))
        x = np.zeros((720, 1280), np.uint8); x[100:400, 100:600] = 200
        obj.lane_mask(x)
        obj.lane_mask(np.zeros_like(x))
        self.assertGreater(np.count_nonzero(fake.inputs[0]), 0)
        self.assertTrue(np.all(fake.inputs[1] == 0))

    def test_wrong_dtype_rejected(self):
        """三通道 BGR 不是 YRG 帧，适配器必须直接拒绝。"""
        obj, fake = self.adapter(np.zeros((1, 3, 360, 640), np.float32))
        with self.assertRaises(Exception):
            obj.lane_mask(np.zeros((720, 1280, 3), np.uint8))
        self.assertFalse(fake.inputs)

    def test_old_angle_backend_remains_callable(self):
        from lane_geometry import select_lanes
        mask = np.zeros((720, 1280), np.uint8)
        self.assertEqual(select_lanes(mask, ratio_threshold=.03)['left']['kind'], 'none')
        self.assertEqual(select_lanes(mask, ratio_threshold=.03)['right']['kind'], 'none')


class PreprocessTests(unittest.TestCase):
    def test_only_gray_variant_exists(self):
        """二值变体已删除：bin768 / 任何未知变体都必须抛错。"""
        g = np.zeros((720, 1280), np.uint8)
        for bad in ('bin768', 'nope', 'gray'):
            with self.assertRaises(ValueError, msg=f'{bad} 不该被接受'):
                lane_v2.prepare(g, bad)

    def test_no_binarization_helper_left(self):
        """prepare_binary 必须已经不存在 —— 它是 RK 端二值化的入口。"""
        self.assertFalse(hasattr(lane_v2, 'prepare_binary'),
                         'lane_v2.prepare_binary 必须删除（用户要求不做二值化）')
        self.assertNotIn('190', str(lane_v2.VARIANTS),
                         'VARIANTS 里不该再有二值阈值 190')

    def test_prepare_keeps_thin_lines_via_inter_area(self):
        g = np.zeros((720, 1280), np.uint8)
        g[:, 640] = 253
        out = lane_v2.prepare(g, 'gray640')[0, 0]
        self.assertGreater(int((out > 0).sum()), 0)


if __name__ == '__main__':
    unittest.main()
