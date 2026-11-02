"""V17_4：三件事的回归测试。

1. `_brightness` 不能按对象身份缓存 —— 2026-10-28 板上第二次运行从第 84 轮起
   场景模型永久检不到任何东西，形态是"图是好的、模型走的是旧图"。
2. 空场景结果不能进节流缓存 —— 同一份日志里连续 202 轮全黑。
3. `LastGood` 让 hold 分支回退到"最近一次有颜色"，而不是"上一轮"。
"""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from protocol import Display, RED, GREEN, YELLOW, BLUE
import strategy_engine
from strategy_engine import SceneThrottle, LastGood


class SceneEmptyNeverCachedTests(unittest.TestCase):
    """2026-10-28：空答案被缓存后反复复用，202 轮全黑。"""

    def test_empty_answer_is_not_reused(self):
        t = SceneThrottle(every=2, max_hold=30, nav_max_hold=4)
        t.observe([])                     # 场景模型真的什么都没检到
        # 不能复用：不然下一轮屏幕直接黑，而且这一黑就是几十轮
        self.assertTrue(t.should_run())

    def test_non_empty_answer_is_still_reused(self):
        t = SceneThrottle(every=2, max_hold=30, nav_max_hold=4)
        t.observe([{'class_name': 'lane'}])
        self.assertFalse(t.should_run())

    def test_forced_still_wins(self):
        t = SceneThrottle(every=2, max_hold=30, nav_max_hold=4)
        t.observe([{'class_name': 'lane'}])
        t.force_next()
        self.assertTrue(t.should_run())


class BrightnessNoIdentityCacheTests(unittest.TestCase):
    """`_brightness` 依赖 `yrg is self._cached_yrg`，等于依赖 id 复用。"""

    def _src(self):
        p = Path(__file__).resolve().parents[1] / 'models_runtime.py'
        return p.read_text(encoding='utf-8')

    def test_does_not_cache_by_object_identity(self):
        src = self._src()
        start = src.index('def _brightness')
        body = src[start:start + 900]
        # 只要有 `if yrg is not self._cached_yrg` 这种"命中就返回旧灰度"的写法，
        # 一旦 CPython 把新数组放在释放掉的旧地址上，就会喂上一帧的灰度。
        self.assertNotIn('if yrg is not self._cached_yrg', body)

    def test_computes_from_the_frame_it_was_given(self):
        import models_runtime
        import vision_common

        class Stub:
            _brightness = models_runtime.LazyDetectors._brightness
            _cached_yrg = None
            _cached_gray = None

        a = np.zeros((4, 4), np.uint8)
        a[0, 0] = 10
        b = np.zeros((4, 4), np.uint8)
        b[0, 0] = 200
        s = Stub()
        ga = s._brightness(a)
        gb = s._brightness(b)
        self.assertNotEqual(int(ga[0, 0]), int(gb[0, 0]),
                            '两次不同的输入必须给出不同的灰度')


class LastGoodTests(unittest.TestCase):
    """用户要求「要保证不要出现没有箭头的情况」。"""

    def test_remembers_colour_per_family(self):
        g = LastGood()
        g.remember('lane', Display(left=YELLOW, right=RED))
        g.remember('traffic', Display(left=RED, straight=RED, right=GREEN))
        self.assertEqual(g.recall('lane').left, YELLOW)
        self.assertEqual(g.recall('traffic').right, GREEN)

    def test_blank_display_is_not_remembered(self):
        g = LastGood()
        g.remember('lane', Display(left=YELLOW))
        g.remember('lane', Display())
        self.assertEqual(g.recall('lane').left, YELLOW,
                         '黑屏不能覆盖掉上一次有颜色的记忆')

    def test_expires_after_the_limit(self):
        g = LastGood()
        g.remember('traffic', Display(straight=GREEN))
        for _ in range(5):
            g.tick()
        self.assertIsNone(g.recall('traffic', limit=4))
        self.assertIsNotNone(g.recall('traffic', limit=10))

    def test_forget_clears_the_family(self):
        g = LastGood()
        g.remember('lane', Display(left=RED))
        g.forget('lane')
        self.assertIsNone(g.recall('lane'))

    def test_families_do_not_mix(self):
        g = LastGood()
        g.remember('lane', Display(left=YELLOW))
        self.assertIsNone(g.recall('traffic'))

    def run_lane_hold(self, first, second, hold):
        """车道分支连续两帧两侧都解不出线。"""
        import strategy_engine as se
        from lane_stability import LaneSolidStreaks

        class Det:
            def scene(self, g):
                return [{'class_name': 'lane', 'class_id': 2, 'score': .9,
                         'xyxy': [0, 400, 1280, 700]}]

            def navigation(self, g): return []

            def housing(self, g): return []

            def lane_mask(self, g):
                return np.zeros((432, 768), np.uint8)   # 两侧 none

        G = np.zeros((720, 1280), np.uint8)
        disp, res = se.decide_pair(
            G, Det(), previous_display=first,
            lane_streaks=LaneSolidStreaks(1), last_good=hold,
            last_good_hold=10)
        return disp, res

    def test_lane_hold_falls_back_to_the_last_coloured_round(self):
        h = LastGood()
        # 第一轮：有颜色的车道显示，会被记住
        h.remember('lane', Display(left=YELLOW, right=RED))
        disp, res = self.run_lane_hold(Display(), Display(), h)
        self.assertEqual((disp.left, disp.right), (YELLOW, RED),
                         '上一轮是黑的，但要回退到最近一次有颜色')
        self.assertEqual(res.get('last_good_fallback'), 'lane')

    def test_disabled_hold_keeps_old_behaviour(self):
        h = None
        disp, res = self.run_lane_hold(Display(), Display(), h)
        self.assertEqual((disp.left, disp.right), (None, None))
        self.assertNotIn('last_good_fallback', res)


class TimingLineTests(unittest.TestCase):
    """用户要求像调试器那样从日志看出每一段耗时。"""

    def test_main_emits_a_per_stage_timeline(self):
        p = Path(__file__).resolve().parents[1] / 'main.py'
        src = p.read_text(encoding='utf-8')
        self.assertIn('第%d轮 耗时:', src)
        for key in ('收图', '场景', '决策'):
            self.assertIn(key, src)


if __name__ == '__main__':
    unittest.main()
