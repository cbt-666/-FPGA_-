"""Regressions from the first real board run (rk3568_v17_gray.log, 2026-10-28).

Every test here pins a failure that the log actually contains, with the count
that was measured, so a future change that reintroduces it fails loudly.
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import traffic_rules
from protocol import Display, RED, GREEN, YELLOW, BLUE
from strategy_engine import (decide_pair, NavDirectionHold, TrafficHoldLimit, NoBoxHold,
                             SceneThrottle, TrafficEvidenceGuard, LayoutHold)
from lane_stability import LaneSolidStreaks
from main import default_argcheck, parser
from test_v12_logic import Fake, G, B, b, hood


def vertical_lamp(cx=700, cy=386, w=22, h=52, score=0.7, colour='red'):
    """A vertical S2TLD-style lamp: aspect 0.42, the shape the board rejected."""
    x1, y1 = cx - w // 2, cy - h // 2
    return hood(colour, x=x1, y=y1, w=w, h=h, score=score)


class TrafficNoGeometryDropTests(unittest.TestCase):
    """The board's geometry filters kept 14 of 248 rounds (5.6%).

    `dropped` accumulated 386 `tall` and only 3 `low` -- the filters were
    discarding real lamps on aspect ratio. Box COUNT decides the direction, so
    every box is now kept; 2026-10-28 replay of the same log: 14 -> 149 valid
    rounds.
    """

    def test_vertical_lamp_is_kept(self):
        v = traffic_rules.judge([vertical_lamp()], G, 1280, 720)
        self.assertEqual(len(v.boxes), 1)
        self.assertEqual(v.layout, 'single_all_three')

    def test_horizontal_housing_is_kept(self):
        box = hood('green', x=1000, y=280, w=145, h=68, score=.6)
        v = traffic_rules.judge([box], G, 1280, 720)
        self.assertEqual(len(v.boxes), 1)

    def test_thin_pole_is_kept(self):
        # Was rejected as `tall` (aspect 0.05). It is now used like any other box.
        thin = hood('red', x=700, y=200, w=12, h=220, score=.5)
        v = traffic_rules.judge([thin], G, 1280, 720)
        self.assertEqual(len(v.boxes), 1)

    def test_wide_box_is_kept(self):
        wide = hood('red', x=100, y=100, w=800, h=80, score=.5)
        v = traffic_rules.judge([wide], G, 1280, 720)
        self.assertEqual(len(v.boxes), 1)

    def test_three_vertical_lamps_in_a_row_resolve(self):
        # idx300 of the board log: cx 687/711/768, cy all 386.
        boxes = [vertical_lamp(cx=687), vertical_lamp(cx=711),
                 vertical_lamp(cx=768)]
        verdict = traffic_rules.judge(boxes, G, 1280, 720)
        self.assertEqual(verdict.layout, 'left_straight_right')
        self.assertEqual(len(verdict.boxes), 3)

    def test_low_score_box_is_not_discarded(self):
        # The old rule kept only the 0.28 horizontal box and threw away the
        # 0.7 vertical ones -- it selected the weakest evidence on the frame.
        # Score no longer gates anything; both boxes are used.
        boxes = [vertical_lamp(cx=600, score=.71),
                 hood('green', x=1000, y=290, w=145, h=68, score=.28)]
        verdict = traffic_rules.judge(boxes, G, 1280, 720)
        self.assertEqual(len(verdict.boxes), 2)
        self.assertIn(.71, [box[4] for box in verdict.boxes])


class TrafficHoldLimitTests(unittest.TestCase):
    """idx366-429 showed green for 56.9 s with only 12 confirming rounds."""

    def test_confirmed_green_restarts_the_clock(self):
        lim = TrafficHoldLimit(hold_frames=15, green_frames=6)
        for _ in range(5):
            lim.observe([GREEN])
            self.assertEqual(lim.expire(GREEN), GREEN)

    def test_unconfirmed_green_expires(self):
        # green_frames counts the confirming round too: 3 total, then gone.
        lim = TrafficHoldLimit(hold_frames=15, green_frames=3)
        lim.observe([GREEN])
        self.assertEqual([lim.expire(GREEN) for _ in range(3)],
                         [GREEN, GREEN, None])

    def test_green_is_given_less_time_than_red(self):
        # NOTE: RED == 0 is falsy, so count `is not None`, never truthiness.
        lim = TrafficHoldLimit(hold_frames=10, green_frames=2)
        lim.observe([RED])
        red_life = sum(1 for _ in range(20) if lim.expire(RED) is not None)
        lim2 = TrafficHoldLimit(hold_frames=10, green_frames=2)
        lim2.observe([GREEN])
        green_life = sum(1 for _ in range(20) if lim2.expire(GREEN) is not None)
        self.assertLess(green_life, red_life)

    def test_red_uses_the_longer_limit(self):
        lim = TrafficHoldLimit(hold_frames=4, green_frames=1)
        lim.observe([RED])
        self.assertEqual([lim.expire(RED) for _ in range(3)], [RED] * 3)
        self.assertIsNone(lim.expire(RED))

    def test_zero_disables_the_limit(self):
        lim = TrafficHoldLimit(hold_frames=0, green_frames=0)
        lim.observe([GREEN])
        self.assertEqual([lim.expire(GREEN) for _ in range(20)], [GREEN] * 20)

    def test_no_box_timeout_goes_all_red(self):
        """2026-10-29：按颜色计时停用；无框超时统一走全红（安全侧）。

        旧行为是"每个方向各自过期、熄掉"，那正是"箭头从右到直再到左
        依次消失"的原因。现在任何时刻都必须是三个箭头。
        """
        nb = NoBoxHold(hold_frames=2)
        prev = Display(left=GREEN, straight=GREEN, right=GREEN)
        for _ in range(2):
            disp, _ = decide_pair(G, Fake([b('traffic_light')], traffic=[]),
                                  previous_display=prev,
                                  lane_streaks=LaneSolidStreaks(1),
                                  no_box_hold=nb)
            prev = disp
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, RED))


class LaneDoubleConfirmationTests(unittest.TestCase):
    """V17 曾把虚线/实线都改成"连续 3 帧确认"，V17_1 又退回好版语义。

    为什么退：红绿灯分支每轮都会 `lane_streaks.reset()`，而双标签帧里车道常常
    被灯箱抢走。3 帧确认在两条分支交替时永远凑不满，箭头干脆不再出现——
    用户看到的就是"车道没了"。好版（2026-10-28 17:00）用的是虚线立即黄、
    实线才计数。这里保留的是那套语义。
    """

    def setUp(self):
        self.streaks = LaneSolidStreaks(3)

    def run_kinds(self, kinds, previous=Display()):
        out = []
        for kind in kinds:
            with patch('strategy_engine.select_lanes', return_value={
                    'left': {'kind': kind}, 'right': {'kind': 'none'}}):
                disp, _ = decide_pair(G, Fake([b('lane')]),
                                      previous_display=previous,
                                      lane_streaks=self.streaks,
                                      lane_backend='angle')
            previous = disp
            out.append(disp.left)
        return out

    def test_alternating_kinds_settle_on_yellow(self):
        # 虚线随时刷成黄；交替序列的稳态因此是黄，不是空白。
        out = self.run_kinds(['dash', 'solid'] * 6)
        self.assertIn(YELLOW, out)

    def test_dash_is_immediate(self):
        self.assertEqual(self.run_kinds(['dash'] * 2), [YELLOW, YELLOW])

    def test_solid_confirms_after_three(self):
        self.assertEqual(self.run_kinds(['solid'] * 4),
                         [None, None, RED, RED])

    def test_switch_back_to_dash_is_immediate(self):
        # 好版语义：实线确认成红之后，一帧虚线就该刷回黄。
        self.run_kinds(['solid'] * 3)
        self.assertEqual(self.run_kinds(['dash'], previous=Display(left=RED)),
                         [YELLOW])


class NavDirectionHoldTests(unittest.TestCase):
    """idx150-199: right_3 <-> right_2 alternated on the 100 m boundary."""

    def test_confident_reading_is_immediate(self):
        h = NavDirectionHold(confirm_frames=2, confident=.85)
        self.assertEqual(h.observe('right', .94), 'right')

    def test_single_weak_flip_does_not_change(self):
        h = NavDirectionHold(confirm_frames=2, confident=.85)
        h.observe('straight', .95)
        self.assertEqual(h.observe('right', .49), 'straight')

    def test_two_weak_flips_in_a_row_do_change(self):
        h = NavDirectionHold(confirm_frames=2, confident=.85)
        h.observe('straight', .95)
        h.observe('right', .60)
        self.assertEqual(h.observe('right', .62), 'right')

    def test_board_sequence_151_to_161(self):
        # straight@0.91,.78,.53 -> right@.94 (confident, switches) -> right@.93
        # -> straight@.49 (one weak frame: ignored) -> right@.90 -> ...
        h = NavDirectionHold(2, .85)
        seq = [('straight', .94), ('straight', .91), ('straight', .78),
               ('straight', .53), ('right', .94), ('right', .93),
               ('straight', .49), ('right', .90), ('right', .93),
               ('straight', .76)]
        shown = [h.observe(d, s) for d, s in seq]
        self.assertEqual(shown,
                         ['straight', 'straight', 'straight', 'straight',
                          'right', 'right', 'right', 'right', 'right', 'right'])

    def test_no_direction_keeps_what_is_shown(self):
        h = NavDirectionHold(2, .85)
        h.observe('right', .9)
        self.assertEqual(h.observe(None, 0.0), 'right')

    def test_reset_clears_the_shown_direction(self):
        h = NavDirectionHold(2, .85)
        h.observe('right', .9)
        h.reset()
        self.assertIsNone(h.shown)


class NavSceneCacheTests(unittest.TestCase):
    """The scene pass cost 468 ms on every navigation round."""

    def test_nav_answer_is_reused_twice_then_rerun(self):
        t = SceneThrottle(every=2, nav_max_hold=2)
        t.observe([{'class_name': 'nav'}])
        ran = []
        for _ in range(4):
            if t.should_run():
                ran.append(True); t.observe([{'class_name': 'nav'}])
            else:
                ran.append(False); t.reuse()
        self.assertEqual(ran, [False, False, True, False])

    def test_nav_miss_forces_the_next_scene_pass(self):
        t = SceneThrottle(every=2, nav_max_hold=5)
        t.observe([{'class_name': 'nav'}])
        self.assertFalse(t.should_run())
        t.force_next()
        self.assertTrue(t.should_run())

    def test_lane_answer_unchanged_by_the_nav_rule(self):
        t = SceneThrottle(every=2, max_hold=30, nav_max_hold=0)
        t.observe([{'class_name': 'lane'}])
        self.assertFalse(t.should_run())
        t.reuse()
        self.assertTrue(t.should_run())


if __name__ == '__main__':
    unittest.main()


class TrafficMappingTests(unittest.TestCase):
    """Box count decides the direction, left to right (fixed 2026-10-28).

    1 box = straight, 2 boxes = left + straight, 3 boxes = left + straight +
    right. Right turn is GO when it has no lamp of its own.
    """

    def test_one_box_lights_left_and_straight(self):
        # 2026-10-28 用户规则：每轮必须给满三个方向。
        verdict = traffic_rules.judge([vertical_lamp(colour='red')], G, 1280, 720)
        self.assertEqual(verdict.layout, 'single_all_three')
        self.assertEqual(verdict.states['left'], traffic_rules.STATE_STOP)
        self.assertEqual(verdict.states['straight'], traffic_rules.STATE_STOP)
        self.assertEqual(verdict.states['right'], traffic_rules.STATE_GO)

    def test_two_boxes_are_left_then_straight(self):
        boxes = [hood('red', x=687, w=22, h=52, score=.7),
                 hood('green', x=760, w=22, h=52, score=.7)]
        verdict = traffic_rules.judge(boxes, G, 1280, 720)
        self.assertEqual(verdict.layout, 'left_straight')
        self.assertEqual((verdict.states['left'], verdict.states['straight'],
                          verdict.states['right']),
                         (traffic_rules.STATE_STOP, traffic_rules.STATE_GO,
                          traffic_rules.STATE_GO))

    def test_three_boxes_give_right_its_own_lamp(self):
        boxes = [hood('green', x=670, w=22, h=52, score=.7),
                 hood('green', x=720, w=22, h=52, score=.7),
                 hood('red', x=770, w=22, h=52, score=.7)]
        verdict = traffic_rules.judge(boxes, G, 1280, 720)
        self.assertEqual(verdict.layout, 'left_straight_right')
        self.assertEqual((verdict.states['left'], verdict.states['straight'],
                          verdict.states['right']),
                         (traffic_rules.STATE_GO, traffic_rules.STATE_GO,
                          traffic_rules.STATE_STOP))


class BinaryRequestTests(unittest.TestCase):
    """取二值的时机：由**场景答案**决定，不看这一轮最终跑了哪条分支。

    历史之所以反复：

    * 每轮都取二值 → 每轮多收一帧，RSS 涨到 660MB、可用内存剩 150MB，
      第 335 轮被内核 OOM 杀掉（返回 137）；而且留在 MODE 2 上的"灰度"帧
      其实是二值图，红绿灯读不到 254/255，全部 off→全向红。
    * 完全不取 → A768 没有输入，184/184 轮 `kind: none`。
    * 按"上一轮真的跑了车道分支"取 → 双标签帧被红绿灯吃掉之后就不再取二值，
      车道又饿死。

    V17_2（2026-10-28 晚）改成：默认**只在场景缓存里有 lane 时取**，初值 False。
    要恢复"第一轮就取"用 --prime-binary，要每轮都取用 --always-request-binary。
    代价是 A768 出图机会变少；换来的是不 OOM、也不再有二值帧上的假红灯。
    """

    def _src(self):
        return (Path(__file__).resolve().parents[1] / 'main.py').read_text(encoding='utf-8')

    def test_single_grey_capture_path(self):
        """2026-10-31：二值通道整个删除。主循环只调 capture_one_frame，没有模式。"""
        src = self._src()
        self.assertIn('gray = capture_one_frame(', src)
        for gone in ('choose_mode(', 'link.prearm(', 'capture_sequential_pair',
                     'dual_capture', 'always_request_binary',
                     'BINARY', 'gray_is_binary', 'lane_variant'):
            self.assertNotIn(gone, src, f'main.py 不该再有 {gone}')
        import main as m
        import inspect
        self.assertNotIn('image_mode',
                         inspect.signature(m.capture_one_frame).parameters)

    def test_threshold_detection_renamed_and_kept(self):
        """帧塌缩成 {0,253} 时灯箱不判色 —— 这层保护必须还在，只是换了名字。

        它不是 RK 端二值化（RK 端一个阈值都没有），是防"阈值图里数不到
        254/255 → 全向红"的完整性检查（错误汇集第 7 条）。
        """
        import main as m
        import inspect
        self.assertIn('frame_is_thresholded',
                      inspect.signature(m.main_entry).parameters)
        import strategy_engine as se
        self.assertIn('frame_is_thresholded',
                      inspect.signature(se.decide_pair).parameters)
        src = self._src()
        self.assertIn('last_frame_thresholded', src)


class NetworkFaultTests(unittest.TestCase):
    """rk3568_v17_orig.log: 61 ENETUNREACH over six futile program restarts."""

    def test_wrapped_oserror_is_recognised(self):
        import errno, net_recovery
        try:
            try:
                raise OSError(errno.ENETUNREACH, 'Network is unreachable')
            except OSError as inner:
                raise TimeoutError('send failed') from inner
        except TimeoutError as mid:
            try:
                raise RuntimeError('capture gave up') from mid
            except RuntimeError as outer:
                self.assertTrue(net_recovery.is_network_fault(outer))

    def test_plain_timeout_is_not_a_network_fault(self):
        import net_recovery
        self.assertFalse(net_recovery.is_network_fault(
            RuntimeError('UDP quiet for 0.8s mode=1')))

    def test_errno_101_alone_is_enough(self):
        # The wrapper's message is localised, so the errno attribute must carry
        # the signal even when the text matches no English marker.
        import errno, net_recovery
        wrapped = TimeoutError('send failed')
        wrapped.errno = errno.ENETUNREACH
        self.assertTrue(net_recovery.is_network_fault(wrapped))

    def test_english_marker_is_recognised(self):
        import net_recovery
        self.assertTrue(net_recovery.is_network_fault(
            TimeoutError('[Errno 101] Network is unreachable')))

    def test_reconfigure_is_skipped_without_passwordless_sudo(self):
        import net_recovery
        attempted, ok, detail = net_recovery.try_reconfigure(
            Path('/nonexistent/setup_fpga_net.sh'))
        self.assertFalse(attempted)
        self.assertFalse(ok)
        self.assertIn('无法自动重配', detail)


class HighwayPhantomTrafficTests(unittest.TestCase):
    """rk3568_v17_5.log idx244-435: 192 rounds of traffic on a highway.

    The scene model called isolated crash barrels / lamp posts `traffic_light`
    at 0.86-0.93 for 192 consecutive rounds. Over half of those rounds had no
    housing box at all, and the ones that did got 10x20 boxes whose pixels were
    all `off`. `off` became RED, RED was passed as `fresh`, `last_good` stored
    it, and the arrows never went away.

    Measured geometry, same log:
        real housings   27x61 .. 63x135   (1600 .. 8500 px)
        phantom dets    10x20 .. 22x47    ( 190 .. 1000 px)
    """

    PHANTOM = [[683, 343, 693, 362, 0.4]]        # 10x19
    REAL = [[698, 380, 734, 461, 0.83]]          # 36x81

    def test_phantom_never_opens_the_traffic_branch(self):
        guard = TrafficEvidenceGuard(release_frames=3)
        for _ in range(10):
            self.assertEqual(guard.observe(self.PHANTOM), 'release')
        self.assertFalse(guard.active)

    def test_real_lamp_opens_and_phantom_then_holds_then_releases(self):
        guard = TrafficEvidenceGuard(release_frames=3)
        self.assertEqual(guard.observe(self.REAL), 'pass')
        self.assertTrue(guard.active)
        self.assertEqual(guard.observe(self.PHANTOM), 'hold')
        self.assertEqual(guard.observe(self.PHANTOM), 'hold')
        self.assertEqual(guard.observe(self.PHANTOM), 'release')
        self.assertFalse(guard.active)

    def test_scene_phantoms_fall_through_to_the_lane_branch(self):
        G[:] = 0
        mask = np.zeros((432, 768), np.uint8)
        mask[300:430, 100:110] = 2
        mask[300:430, 660:670] = 2
        fake = Fake([b('lane'), b('traffic_light')],
                    traffic=self.PHANTOM[:], mask=mask)
        disp, result = decide_pair(G, fake, lane_backend='v2',
                                   traffic_guard=TrafficEvidenceGuard(),
                                   no_box_hold=NoBoxHold(1))
        self.assertEqual(result['branch'], 'lane')
        self.assertEqual(result['scene_targets'], ['lane'])
        self.assertTrue(result['traffic_guard']['downgraded'])
        self.assertNotIn(RED, (disp.left, disp.straight, disp.right))

    def test_dark_boxes_are_not_traffic_evidence(self):
        """A box with no 254/255 pixel inside is not a lit lamp.

        V20 behaviour was to paint it three red arrows. The user reported
        exactly that symptom ("识别到一个框的三红三绿"), so the evidence gate now
        asks for a **lit** housing, not merely a box.
        """
        G[:] = 0
        fake = Fake([b('traffic_light')],
                    traffic=[[400, 150, 480, 190, 0.9]])   # 80x40, dark box
        disp, result = decide_pair(G, fake,
                                   traffic_guard=TrafficEvidenceGuard(),
                                   no_box_hold=NoBoxHold(1))
        self.assertEqual(result['branch'], 'lane_hold')   # 让给车道，不是红绿灯
        self.assertTrue(result['traffic_guard']['downgraded'])
        self.assertEqual(result['traffic_guard']['lit_lamps'], 0)
        self.assertNotIn(RED, (disp.left, disp.straight, disp.right))

    def test_lit_box_still_drives_the_traffic_branch(self):
        G[:] = 0
        box = hood('red')                      # paints 254 inside the box
        fake = Fake([b('traffic_light')], traffic=[box])
        disp, result = decide_pair(G, fake,
                                   traffic_guard=TrafficEvidenceGuard(),
                                   no_box_hold=NoBoxHold(1))
        self.assertTrue(str(result['branch']).startswith('traffic'))
        self.assertEqual(result['traffic_guard']['lit_lamps'], 1)
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, GREEN))


class TrafficLayoutHoldTests(unittest.TestCase):
    """rk3568_v17_5.log idx244-435: box count flickered, semantics flipped with it.

    Box count IS the direction semantics (1 = left+straight same colour,
    2 = left/straight, 3 = left/straight/right). A one-round flicker from 3 to
    1 does not lose one arrow -- it re-assigns all three. Measured on that
    stretch: 17 raw count changes, 8 semantic flips with no hold.
    """

    REAL_3 = [[698, 380, 734, 461, 0.83], [803, 377, 841, 457, 0.77],
              [931, 375, 969, 454, 0.70]]
    ONE = [[756, 247, 816, 373, 0.38]]

    def test_single_frame_flicker_is_rejected(self):
        hold = LayoutHold(confirm_frames=2)
        self.assertEqual(hold.observe(self.REAL_3), 3)
        self.assertEqual(hold.observe(self.ONE), 3)   # one frame is noise
        self.assertEqual(hold.observe(self.REAL_3), 3)

    def test_sustained_change_is_adopted(self):
        hold = LayoutHold(confirm_frames=2)
        hold.observe(self.REAL_3)
        self.assertEqual(hold.observe(self.ONE), 3)
        self.assertEqual(hold.observe(self.ONE), 1)

    def test_zero_boxes_does_not_reset_the_layout(self):
        # A miss is NoBoxHold's business, not the layout's: it must not
        # re-adopt a layout on the next frame.
        hold = LayoutHold(confirm_frames=2)
        hold.observe(self.REAL_3)
        self.assertEqual(hold.observe([]), 3)
        self.assertEqual(hold.observe(self.REAL_3), 3)

    def test_capped_at_three(self):
        # traffic_rules only ever uses the first three boxes.
        hold = LayoutHold(confirm_frames=1)
        self.assertEqual(hold.observe(self.REAL_3 + self.REAL_3), 3)


class TrafficAlwaysThreeArrowsTests(unittest.TestCase):
    """User's hard rule: three arrows, all or nothing."""

    def _disp(self, **kw):
        from protocol import Display
        return Display(**kw)

    def test_every_traffic_exit_has_all_three(self):
        G[:] = 0
        cases = [
            Fake([b('traffic_light')], traffic=[hood('red')]),
            Fake([b('traffic_light')], traffic=[hood('green')]),
            Fake([b('traffic_light')], traffic=[hood('off')]),
            Fake([b('traffic_light')],
                 traffic=[hood('red', x=400, w=60), hood('green', x=700, w=60)]),
            Fake([b('traffic_light')],
                 traffic=[hood('red', x=400, w=60), hood('green', x=700, w=60),
                          hood('red', x=1000, w=60)]),
            Fake([b('traffic_light')], traffic=[]),
        ]
        for fake in cases:
            disp, result = decide_pair(
                G, fake, previous_display=self._disp(),
                traffic_guard=TrafficEvidenceGuard(), no_box_hold=NoBoxHold(1),
                layout_hold=LayoutHold(1))
            if not str(result['branch']).startswith('traffic'):
                continue
            self.assertNotIn(None, (disp.left, disp.straight, disp.right),
                             msg=f"{fake.traffic} -> {result['display_reason']}")
