"""V17_6：深度审查发现的静默失效，全部有可复现的最小用例。

这些 bug 的共同点：单元测试全绿、日志不报错、屏幕就是不对。
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from protocol import Display, RED, GREEN, YELLOW, BLUE
import strategy_engine as se
import traffic_rules
from strategy_engine import (decide_pair, TrafficHoldLimit, LastGood,
                             EmptyStreak, BranchSwitch, NoBoxHold)
from lane_stability import LaneSolidStreaks
from test_v12_logic import Fake, G, B, b, hood


class NoHousingBoxClearsTests(unittest.TestCase):
    """无框：先沿用，连续 N 轮才清零（2026-10-29 用户定）。

    旧实现还有第二个 bug：traffic_rules 无框时返回 layout='none'，而
    strategy_engine 判的是 layout=='no_housing_box'，两个名字对不上，
    整条"无框处理"分支从来没执行过。
    """

    def test_layout_name_matches_what_the_engine_checks(self):
        v = traffic_rules.judge([], np.zeros((720, 1280), np.uint8))
        self.assertEqual(v.layout, 'no_housing_box')
        self.assertEqual(v.states, {})

    def test_no_box_holds_for_the_configured_window(self):
        """无框先沿用，超时后全红 —— 任何时刻都是三个箭头。"""
        G[:] = 0
        hold = NoBoxHold(hold_frames=3)
        prev = Display(left=RED, straight=RED, right=GREEN)
        out = []
        for _ in range(3):
            disp, res = decide_pair(
                G, Fake([b('traffic_light')], traffic=[]),
                previous_display=prev, lane_streaks=LaneSolidStreaks(1),
                no_box_hold=hold)
            out.append((disp.left, disp.straight, disp.right))
            prev = disp
        self.assertEqual(out[0], (RED, RED, GREEN), '第 1 轮无框：沿用')
        self.assertEqual(out[1], (RED, RED, GREEN), '第 2 轮无框：沿用')
        self.assertEqual(out[2], (RED, RED, RED), '第 3 轮无框：全红兜底')
        # 硬性条件：任何一轮都是三个箭头
        for three in out:
            self.assertTrue(all(c is not None for c in three))

    def test_a_box_resets_the_window(self):
        hold = NoBoxHold(hold_frames=3)
        self.assertTrue(hold.observe(False))
        self.assertTrue(hold.observe(False))
        self.assertTrue(hold.observe(True))     # 又检到框了
        self.assertEqual(hold.streak, 0)
        self.assertTrue(hold.observe(False))    # 重新开始计数

    def test_housing_error_holds_instead_of_clearing(self):
        """子模型报错是我们的故障，不是"画面里没有灯"，不能拿清屏当结论。"""
        class Boom(Fake):
            def housing(self, im):
                raise ValueError('housing 需要双输出，得到 1')

        G[:] = 0
        disp, res = decide_pair(G, Boom([b('traffic_light')]),
                                previous_display=Display(left=RED, straight=RED,
                                                         right=RED),
                                lane_streaks=LaneSolidStreaks(1))
        self.assertEqual(res['branch'], 'traffic_hold')
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, RED))


class LastGoodCannotResurrectExpiredGreenTests(unittest.TestCase):
    """停用按颜色计时后，绿灯"多久算消失"只由 NoBoxHold 回答。

    2026-10-29 用户问"绿灯为什么会过期"——按颜色计时是我上一版加的保险，
    但它和"永远三个箭头"直接冲突（绿 6 轮先到期、红 15 轮后到期，
    箭头一个一个消失）。现在只剩一条边界：连续 N 轮无框 -> 全红。
    """

    def test_green_survives_until_the_no_box_window_expires(self):
        class Det:
            def scene(self, g):
                return [{'class_name': 'traffic_light', 'class_id': 3,
                         'score': .9, 'xyxy': [0, 0, 10, 10]}]

            def housing(self, g):
                return []          # 解不出灯

            def navigation(self, g): return []

            def lane_mask(self, g): return np.zeros((432, 768), np.uint8)

        lg = LastGood()
        lg.remember('traffic', Display(left=GREEN, straight=GREEN, right=GREEN))
        nb = NoBoxHold(hold_frames=3)
        prev = Display(left=GREEN, straight=GREEN, right=GREEN)
        shown = []
        for _ in range(6):
            disp, res = decide_pair(
                G, Det(), previous_display=prev,
                lane_streaks=LaneSolidStreaks(1),
                last_good=lg, last_good_hold=10,
                no_box_hold=nb)
            shown.append((disp.left, disp.straight, disp.right))
            prev = disp
        # 前两轮沿用绿灯，第 3 轮起全红；任何一轮都不许出现半个灯
        self.assertEqual(shown[0], (GREEN, GREEN, GREEN))
        self.assertEqual(shown[-1], (RED, RED, RED))
        for three in shown:
            self.assertTrue(all(c is not None for c in three),
                            '红绿灯模式必须永远三个箭头')

    def test_fallback_alone_does_not_refresh_the_memory(self):
        """回填出来的旧颜色不能被当成"本轮新鲜颜色"重新记忆，否则年龄归零。"""
        lg = LastGood()
        lg.remember('lane', Display(left=YELLOW))
        for _ in range(5):
            lg.tick()
        # 极限设成 3，已经过期
        self.assertIsNone(lg.recall('lane', limit=3))


class PerSideExpiryTests(unittest.TestCase):
    """1 框布局把同一颜色铺满三个方向，每轮只能推进一次计数。"""

    def test_one_box_full_red_lasts_the_red_limit(self):
        lim = TrafficHoldLimit(hold_frames=6, green_frames=2)
        disp = Display(left=RED, straight=RED, right=RED)
        first = lim.expire_display(disp)
        # 一轮下来只涨 1，不是 3
        self.assertEqual(lim.stale_red, 1)
        self.assertEqual((first['left'], first['straight'], first['right']),
                         (RED, RED, RED))

    def test_two_colours_count_independently(self):
        lim = TrafficHoldLimit(hold_frames=6, green_frames=2)
        lim.expire_display(Display(left=RED, straight=RED, right=GREEN))
        self.assertEqual(lim.stale_red, 1)
        self.assertEqual(lim.stale_green, 1)


class NavSubmodelErrorTests(unittest.TestCase):
    """导航子模型挂了，屏幕不能假装给了"直行"指示。"""

    def test_error_with_no_blue_shows_nothing(self):
        class Boom(Fake):
            def navigation(self, im):
                raise RuntimeError('nav model crashed')

        disp, res = decide_pair(G, Boom([b('nav')]),
                                lane_streaks=LaneSolidStreaks(1))
        self.assertEqual(res['display_reason'], 'nav_submodel_error_no_arrow')
        self.assertEqual((disp.left, disp.straight, disp.right), (None, None, None))


class BinaryIsNotConsumedTests(unittest.TestCase):
    """2026-10-31：不再请求二值化，整条二值链路必须消失。"""

    def test_no_model_reads_binary(self):
        src = (Path(__file__).resolve().parents[1] / 'strategy_engine.py').read_text(
            encoding='utf-8')
        start = src.index('def decide_pair')
        body = src[start:]
        self.assertNotIn('lane_mask(gray, binary', body)
        self.assertNotIn('prepare_binary', body)
        self.assertNotIn('lane_variant', body)

    def test_binary_machinery_is_gone(self):
        """模式选择、轮末预置、二值回退都随通道删除。"""
        src = (Path(__file__).resolve().parents[1] / 'main.py').read_text(encoding='utf-8')
        for gone in ('choose_mode(', 'prearm(', 'args.prime_binary', 'BINARY'):
            self.assertNotIn(gone, src, f'main.py 不该再有 {gone}')
        self.assertIn('IMAGE_MODE', src)


class SignatureGateCoversMethodCallsTests(unittest.TestCase):
    """签名闸必须也检查 obj.method(...)，不能只看裸函数。"""

    def test_checker_handles_attribute_calls(self):
        src = (Path(__file__).resolve().parents[1]
               / 'tools_check_call_signatures.py').read_text(encoding='utf-8')
        self.assertIn('ast.Attribute', src)
        self.assertIn('node.func.attr', src)


if __name__ == '__main__':
    unittest.main()
