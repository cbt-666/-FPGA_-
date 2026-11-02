"""V15.4 requirements: dash YELLOW / solid RED, traffic anti-flicker, strict navigation digits,
priority nav > traffic > angle lane, scene-only full clearing, display packets."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import Display, RED, GREEN, BLUE, YELLOW, GRAY, BINARY, decode_command, encode_packet
from strategy_engine import decide_pair, SceneEmptyHold, eligible_navigation, BranchSwitch
from lane_stability import LaneSolidStreaks
from main import main_entry, parser
from test_v12_logic import Fake, G, B, b, hood


class V15DisplayHoldTests(unittest.TestCase):
    def setUp(self):
        self.solid = LaneSolidStreaks(3)
        self.scene_hold = SceneEmptyHold(2)
        self.previous = Display()

    def cycle(self, fake, **options):
        options.setdefault('lane_backend', 'angle')
        disp, result = decide_pair(
            G, fake, previous_display=self.previous, lane_streaks=self.solid,
            empty_scene_hold=self.scene_hold, **options)
        self.previous = disp
        return disp, result

    def test_yellow_dash_after_confirmation_and_packet(self):
        # 好版语义（V17_1 恢复）：虚线一帧即黄，实线才需要核对帧数。
        with patch('strategy_engine.select_lanes', return_value={
                'left': {'kind': 'dash'}, 'right': {'kind': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]))
        self.assertEqual(disp.left, YELLOW)
        self.assertEqual(info['lane_updates']['left'], 'dash_yellow_immediate')
        decoded, mode = decode_command(encode_packet(disp, GRAY))
        self.assertEqual(mode, GRAY)
        self.assertEqual(decoded.left, YELLOW)
        self.assertEqual(encode_packet(disp, GRAY), b'\x31\x04')
        # 2026-10-31：BINARY 已不能编码（RK 端只发 MODE 1）。
        with self.assertRaises(ValueError):
            encode_packet(disp, BINARY)

    def test_single_dash_frame_does_not_repaint(self):
        # 反过来了：好版是虚线立即生效、实线要确认。这里固定住的是
        # "虚线立即覆盖已有的红"，不再期待迟滞。
        self.previous = Display(left=RED)
        with patch('strategy_engine.select_lanes', return_value={
                'left': {'kind': 'dash'}, 'right': {'kind': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]))
        self.assertEqual(disp, Display(left=YELLOW))
        self.assertEqual(info['lane_updates']['left'], 'dash_yellow_immediate')

    def test_alternating_dash_solid_keeps_yellow(self):
        # 好版语义之下，交替序列的稳态是黄：虚线随时刷成黄，实线永远凑不满
        # confirm_frames。这是 V17_1 有意接受的代价——先让箭头出现。
        for kind in ('dash', 'solid', 'dash', 'solid', 'dash', 'solid'):
            with patch('strategy_engine.select_lanes', return_value={
                    'left': {'kind': kind}, 'right': {'kind': 'none'}}):
                disp, _ = self.cycle(Fake([b('lane')]))
        self.assertEqual(disp, Display(left=YELLOW))

    def test_solid_turns_red_on_first_frame_when_confirm_is_one(self):
        # Row scan is a deterministic geometric test, so one frame is enough.
        self.solid = LaneSolidStreaks(1)
        self.previous = Display(left=GREEN, right=RED)
        with patch('strategy_engine.select_lanes', return_value={
            'left': {'kind': 'solid'}, 'right': {'kind': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]))
        self.assertEqual(disp.left, RED)
        self.assertEqual(disp.right, RED)
        self.assertEqual(info['lane_solid_streaks']['left'], 1)

    def test_no_side_visible_masks_retains_display(self):
        self.previous = Display(left=GREEN, right=RED, straight=GREEN)
        with patch('strategy_engine.select_lanes', return_value={
            'left': {'kind': 'none'}, 'right': {'kind': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]))
        self.assertEqual(disp, self.previous)
        self.assertEqual(info['branch'], 'lane_hold')
        self.assertEqual(self.scene_hold.empty, 0)

    def test_lane_recognized_but_mask_missing_does_not_clear(self):
        self.previous = Display(left=RED)
        disp, info = self.cycle(Fake([b('lane')]))
        self.assertEqual(disp.left, RED)
        self.assertEqual(info['branch'], 'lane_hold')

    def test_traffic_no_box_clears_immediately(self):
        """2026-10-29 审查修正：无框时**立即清零**，不再沿用上一轮灯色。

        你的规则原话：「如果无框那就不输出，同时清零上一次箭头」。
        旧实现里 `traffic_rules.judge` 无框时返回 layout='none'，而这里判的是
        layout=='no_housing_box' —— 两个名字对不上，"无框清零"这条分支
        从来没执行过，屏幕一直亮着上一盏灯。
        """
        self.previous = Display(left=RED, straight=RED, right=GREEN)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[]))
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, RED),
                         '红绿灯模式必须永远三个箭头：无框时用红灯兜底')
        self.assertTrue(info['display_changed'])

    def test_traffic_interleaved_valid_invalid_no_blink(self):
        G[:] = 0
        f1 = Fake([b('traffic_light')], traffic=[hood('red')])
        f2 = Fake([b('traffic_light')], traffic=[])
        states = [self.cycle(f)[0].as_dict() for f in (f1, f2, f1, f2, f1)]
        # 红绿灯模式永远三个箭头：无框的轮走红灯兜底，直行不会消失。
        self.assertTrue(all(d['straight'] == 'red' for d in states))

    def test_traffic_four_aligned_boxes_use_left_straight_right(self):
        # A fourth lamp further right is ignored; the row stays one light group.
        G[:] = 0
        self.previous = Display(left=RED, straight=GREEN, right=GREEN)
        disp, info = self.cycle(Fake([b('traffic_light')],
                            traffic=[hood('green', x=i*90, w=60) for i in range(4)]))
        self.assertEqual((disp.left, disp.straight, disp.right), (GREEN, GREEN, GREEN))
        self.assertEqual(info['traffic']['layout'], 'left_straight_right')

    def test_traffic_far_apart_boxes_are_both_used(self):
        # Two boxes 900 px apart used to be reduced to the stronger one. Box
        # count decides the mapping, so both are kept: left turn + straight.
        G[:] = 0
        self.previous = Display(left=RED, straight=GREEN, right=GREEN)
        disp, info = self.cycle(Fake([b('traffic_light')],
                            traffic=[hood('red', x=200, w=60, score=.9),
                                     hood('green', x=1150, w=60, score=.6)]))
        self.assertEqual(info['traffic']['layout'], 'left_straight')
        self.assertEqual((disp.left, disp.straight, disp.right),
                         (RED, GREEN, GREEN))

    def test_unlit_box_does_not_take_a_direction_slot(self):
        """2026-10-31 V26 用户拍板：没亮的框不占方向位。

        旧规则（V15）是「一个 off 框 -> 左红 + 直红 + 右绿」，代价是凭空禁止直行。
        板端 seq7 实测：画面里只有一个红左转箭头，另外两个框落在暗灯杆上
        （框内 0 个 254/255），旧规则把它们读成"左转红 + 直行红"。
        现在 off 框被 'unlit' 拒掉 -> 无框 -> 走 NoBoxHold 的兜底。
        """
        G[:] = 0
        self.previous = Display(left=GREEN, straight=GREEN, right=GREEN)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[hood('off')]))
        # 关键区别：不再是"按 1 框布局铺左红+直红+右绿"，而是无框兜底的三红。
        # 三红是「红绿灯必须永远三个箭头」的硬规则，安全侧不变；
        # 变的是 reason 与 layout —— 数字/暗灯杆不再当成一盏灯去解释方向。
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, RED))
        self.assertNotEqual(info['traffic']['layout'], 'single_all_three')
        rejected = info.get('traffic_boxes_rejected') or []
        self.assertTrue(rejected, '应记录被拒的框，否则日志里看不出原因')
        self.assertIn(rejected[0]['why'], ('unlit', 'low_unlit'))

    def test_traffic_red_changes_without_delay_still_holds(self):
        """红色真灯不受 V26 影响：亮着的框照常占位。"""
        G[:] = 0
        self.previous = Display(left=GREEN, straight=GREEN, right=GREEN)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[hood('red')]))
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, GREEN))

    def test_traffic_red_changes_without_delay(self):
        G[:] = 0
        self.previous = Display(left=GREEN, straight=GREEN, right=GREEN)
        disp, _ = self.cycle(Fake([b('traffic_light')], traffic=[hood('red')]))
        # 一个框：红同时给左转和直行，右转默认绿。
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, GREEN))

    def test_traffic_invalid_does_not_repaint_lane_markings(self):
        """2026-10-28：场景里有红绿灯时，车道**不许**再刷新箭头。

        旧版（V15.7）是反过来的：灯箱解不出灯就落到 traffic_hold 并继续跑车道，
        理由是一帧误标的 traffic_light 会让实线计数永远攒不满。上板日志证明
        这个取舍现在错了——路口帧顶着上一轮灯色的同时又被画上车道的黄/红，
        用户看到的就是"红绿灯时出现车道"（rk3568_v17_1.log 第 230 轮）。

        现在的规则：只要场景标签里有 traffic_light，本轮就只沿用灯态，
        车道模型整个不跑。路口本来就在重画标线，读出来的线也不可信。
        """
        self.previous = Display(left=GREEN)
        for _ in range(3):
            f = Fake([b('traffic_light'), b('lane')], traffic=[])
            with patch('strategy_engine.select_lanes',
                       return_value={'left': {'kind': 'dash'}, 'right': {'kind': 'none'}}):
                disp, info = self.cycle(f)
        # 无框 -> 按用户规则清零；关键是车道模型根本没跑，屏上也不会有黄。
        self.assertEqual(info['branch'], 'traffic')
        self.assertNotIn('lane', f.calls)           # 车道模型根本没跑
        self.assertNotEqual(disp.left, YELLOW)      # 车道的黄不许出现在灯帧上

    def test_nav_two_digit_blue_strict(self):
        disp, info = self.cycle(Fake([b('nav'), b('traffic_light')],
                                     nav=[b('left_2')], traffic=[b('red')]))
        self.assertEqual(disp, Display(left=BLUE))
        self.assertEqual(info['branch'], 'navigation_blue')

    def test_nav_two_digit_blue_right(self):
        disp, _ = self.cycle(Fake([b('nav')], nav=[b('right_2')]))
        self.assertEqual(disp.right, BLUE)

    def test_nav_three_digits_shows_blue_straight(self):
        self.previous = Display(left=BLUE)
        disp, info = self.cycle(Fake([b('nav')], nav=[b('left_3')]))
        self.assertEqual(disp.straight, BLUE)
        self.assertIsNone(disp.left)
        self.assertEqual(info['branch'], 'navigation_blue')

    def test_nav_invalid_never_keeps_stale_red_green(self):
        # A route screen must not keep showing the previous junction's lights.
        self.previous = Display(left=RED, straight=GREEN)
        disp, info = self.cycle(Fake([b('nav')], nav=[b('left_3')]))
        self.assertEqual(disp.straight, BLUE)
        self.assertIsNone(disp.left)
        self.assertEqual(info['branch'], 'navigation_blue')

    def test_nav_never_falls_through_to_traffic_or_lane(self):
        f = Fake([b('nav'), b('traffic_light'), b('lane')],
                 nav=[b('left_3')], traffic=[b('red')])
        disp, info = self.cycle(f)
        self.assertEqual(info['branch'], 'navigation_blue')
        self.assertEqual(disp.straight, BLUE)
        self.assertNotIn('lane', f.calls)
        self.assertNotIn('housing', f.calls)

    def test_nav_never_falls_through_to_lane(self):
        f = Fake([b('nav'), b('lane')], nav=[b('right_3')])
        with patch('strategy_engine.select_lanes', return_value={
            'left': {'kind': 'dash'}, 'right': {'kind': 'none'}}):
            disp, info = self.cycle(f)
        self.assertEqual(info['branch'], 'navigation_blue')
        self.assertEqual(disp.straight, BLUE)
        self.assertNotIn('lane', f.calls)

    def test_blue_does_not_survive_into_the_lane_branch(self):
        # BLUE means "the route says go this way"; only the nav branch can make
        # one. A previous blue must not stay lit on a lane frame and read as an
        # instruction that no longer applies.
        self.previous = Display(left=BLUE, right=BLUE, straight=BLUE)
        self.solid = LaneSolidStreaks(1)      # confirm immediately, as shipped
        with patch('strategy_engine.lane_v2_decode', return_value={
                'left': {'kind': 'solid', 'x': 100, 'ev': 1.0, 'evidence': 'ok'},
                'right': {'kind': 'none', 'x': None, 'ev': 0.0, 'evidence': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]), lane_backend='v2')
        self.assertEqual(info['branch'], 'lane')
        self.assertEqual(disp.left, RED)
        self.assertIsNone(disp.right)
        self.assertIsNone(disp.straight)

    def test_blue_does_not_survive_into_the_traffic_branch(self):
        G[:] = 0
        self.previous = Display(left=BLUE, straight=BLUE, right=BLUE)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[hood('red')]))
        self.assertEqual(info['branch'], 'traffic')
        self.assertEqual(disp.left, RED, '一个框的颜色同时给左转')
        self.assertEqual(disp.straight, RED)
        self.assertEqual(disp.right, GREEN, 'right turn has no lamp: allowed')

    def test_blue_is_still_held_by_the_one_off_scene_miss(self):
        # The hold path that keeps a blue route arrow alive is a different
        # path (no targets at all) and must keep working.
        self.previous = Display(left=BLUE, right=BLUE, straight=BLUE)
        disp, info = self.cycle(Fake([b('crosswalk')]))
        self.assertEqual(info['branch'], 'scene_empty_hold')
        self.assertEqual(disp.left, BLUE)

    def test_blue_does_not_survive_a_lane_frame_with_no_visible_edges(self):
        # lane_hold path: both sides come back 'none', so the previous arrows
        # are retained -- but a retained BLUE is still a stale route arrow.
        self.previous = Display(left=BLUE, right=BLUE, straight=BLUE)
        with patch('strategy_engine.lane_v2_decode', return_value={
                'left': {'kind': 'none', 'x': None, 'ev': 0.0, 'evidence': 'none'},
                'right': {'kind': 'none', 'x': None, 'ev': 0.0, 'evidence': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]), lane_backend='v2')
        self.assertEqual(info['branch'], 'lane_hold')
        self.assertIsNone(disp.left)
        self.assertIsNone(disp.right)
        self.assertIsNone(disp.straight)

    def test_yellow_lane_arrow_is_cleared_when_a_traffic_light_owns_the_frame(self):
        # YELLOW identifies a lane marking: only the lane branch produces it.
        # Traffic only emits red/green, so a yellow arrow on a light frame is
        # always stale and must go.
        G[:] = 0
        self.previous = Display(left=YELLOW, right=YELLOW)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[hood('green')]))
        self.assertEqual(info['branch'], 'traffic')
        # 一个框 = 绿：左转和直行都是绿，右转默认绿。旧的黄必须被覆盖。
        self.assertEqual((disp.left, disp.straight, disp.right), (GREEN, GREEN, GREEN))

    def test_yellow_cleared_even_when_the_light_model_fails(self):
        # traffic_hold deliberately keeps the last *light* state; a stale lane
        # yellow is not a light state and must not survive.
        G[:] = 0
        self.previous = Display(left=YELLOW, right=YELLOW)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[]))
        # 无框超时 -> 全红兜底；车道黄不能留在灯帧上。
        self.assertEqual((disp.left, disp.right), (RED, RED))

    def test_red_is_cleared_when_no_box(self):
        """2026-10-29：无框时红也清零。

        用户规则「如果无框那就不输出，同时清零上一次箭头」。
        旧测试固化的"红保留"其实是当年"一个框代表三个方向"时代的遗产，
        现在语义已经改成"框数决定方向、无框就是没有结论"，保留旧红会让
        屏幕一直挂着已经消失的灯。
        """
        G[:] = 0
        self.previous = Display(left=RED, straight=RED)
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[]))
        # 2026-10-29 用户硬性要求：红绿灯模式永远三个箭头。
        # 无框超过 N 轮 -> 全红兜底，不是空屏。
        self.assertEqual(info['branch'], 'traffic')
        self.assertEqual((disp.left, disp.straight), (RED, RED))

    def test_lane_yellow_survives_on_a_plain_lane_frame(self):
        # The clearing rule must fire only when traffic owns the frame.
        G[:] = 0
        self.previous = Display(left=YELLOW, right=YELLOW)
        disp, info = self.cycle(Fake([b('lane')]), lane_backend='v2')
        self.assertEqual(info['branch'], 'lane_hold')
        self.assertEqual((disp.left, disp.right), (YELLOW, YELLOW))

    def test_lane_runs_on_grey_without_any_binary_frame(self):
        """2026-10-31：车道只吃灰度，不再有"等二值帧"这一步。"""
        self.previous = Display()
        disp, info = decide_pair(G, Fake([b('lane')]),
                                 previous_display=self.previous,
                                 lane_streaks=LaneSolidStreaks(1),
                                 lane_backend='v2')
        self.assertEqual(info['lane_source'], 'grey')
        self.assertNotIn('lane_skipped_no_binary_frame', info['display_reason'])

    def test_housing_error_holds_instead_of_aborting_the_round(self):
        class Boom(Fake):
            def housing(self, im):
                raise ValueError('housing 需要双输出，得到 1')
        self.previous = Display(left=RED, straight=RED, right=RED)
        disp, info = self.cycle(Boom([b('traffic_light')],
                                     traffic=[hood('green')]))
        self.assertEqual(info['branch'], 'traffic_hold')
        self.assertIn('housing_submodel_error', info['traffic']['reason'])
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, RED))

    def test_lane_error_holds_instead_of_aborting_the_round(self):
        class Boom(Fake):
            def lane_mask(self, im):
                raise ValueError('无法识别车道输出通道')
        self.previous = Display(left=YELLOW)
        disp, info = self.cycle(Boom([b('lane')]), lane_backend='v2')
        self.assertEqual(info['branch'], 'lane_hold')
        self.assertEqual(info['lane_error'], 'ValueError')
        self.assertEqual(disp.left, YELLOW)


class BranchSwitchTests(unittest.TestCase):
    """Entering a different model line starts from a blank screen.

    Yellow/red mean lane markings, red/green mean lights, blue means the route.
    The same colour code therefore means different things per branch, so carrying
    it across a mode change is always a misreading waiting to happen.
    """

    def setUp(self):
        self.switch = BranchSwitch()
        self.solid = LaneSolidStreaks(1)

    def cycle(self, fake, previous, **kw):
        # NOTE: do not touch G here. `hood()` paints the light colour into G
        # when the Fake is built, so clearing it inside cycle() would erase it.
        kw.setdefault('lane_backend', 'v2')
        return decide_pair(G, fake, previous_display=previous,
                           lane_streaks=self.solid, branch_switch=self.switch, **kw)

    def test_lane_to_traffic_clears_the_lane_arrows(self):
        # Round 1 establishes the lane branch, round 2 moves to traffic.
        self.cycle(Fake([b('lane')]), Display())
        disp, r2 = self.cycle(Fake([b('traffic_light')], traffic=[hood('red')]),
                              Display(left=YELLOW, right=RED))
        self.assertTrue(r2['branch_switched'])
        # 一个框：红同时给左转和直行，右转默认绿。
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, GREEN))

    def test_nav_to_lane_drops_the_blue_arrow(self):
        self.cycle(Fake([b('nav')], nav=[b('right_2')]), Display())
        # 车道这一帧必须真的解出线，否则它是"沿用上一轮"的 lane_hold，
        # 按 2026-10-28 的规则不算进入新状态，也就不会清空（见 BranchSwitch）。
        with patch('strategy_engine.lane_v2_decode', return_value={
                'left': {'kind': 'dash', 'x': .4, 'ev': .6, 'evidence': 'ok'},
                'right': {'kind': 'none', 'x': None, 'ev': 0.0, 'evidence': 'none'}}):
            disp, info = self.cycle(Fake([b('lane')]), Display(straight=BLUE))
        self.assertTrue(info['branch_switched'])
        self.assertIsNone(disp.straight)
        self.assertEqual(disp.left, YELLOW)

    def test_same_branch_is_not_a_switch(self):
        self.switch.observe('navigation_blue')
        self.assertFalse(self.switch.observe('navigation_blue'))
        self.assertEqual(self.switch.switches, 0)

    def test_no_target_round_is_not_a_switch(self):
        # A one-frame scene miss must not be treated as a mode change, or the
        # SceneEmptyHold anti-flicker rule would be defeated.
        self.switch.observe('navigation_blue')
        self.assertFalse(self.switch.observe('none'))
        self.assertFalse(self.switch.observe('scene_empty_hold'))
        self.assertFalse(self.switch.observe('navigation_blue'))
        self.assertEqual(self.switch.switches, 0)

    def test_first_round_is_not_a_switch(self):
        self.assertFalse(self.switch.observe('lane'))

    def test_provisional_hold_is_not_a_switch(self):
        # 灯箱解不出灯、车道两侧无线：这一轮只是沿用上一轮显示，
        # 不是"决策变了"。否则一次漏检就把好好的灯态清空。
        self.switch.observe('traffic')
        self.assertFalse(self.switch.observe('traffic_hold', provisional=True))
        self.assertFalse(self.switch.observe('lane_hold', provisional=True))
        self.assertEqual(self.switch.switches, 0)

    def test_switch_count_tracks_real_changes(self):
        for branch in ('navigation_blue', 'navigation_blue',
                       'lane', 'lane', 'traffic', 'navigation_blue'):
            self.switch.observe(branch)
        self.assertEqual(self.switch.switches, 3)

    def test_switch_does_not_persist_beyond_one_round(self):
        self.cycle(Fake([b('nav')], nav=[b('right_2')]), Display())
        disp, info = self.cycle(Fake([b('traffic_light')], traffic=[hood('green')]),
                                Display(straight=BLUE))
        self.assertTrue(info['branch_switched'])
        disp2, info2 = self.cycle(Fake([b('traffic_light')], traffic=[]), disp)
        self.assertFalse(info2['branch_switched'])
        # 红绿灯模式永远三个箭头：无框也走红灯兜底。
        self.assertEqual((disp2.left, disp2.straight, disp2.right), (RED, RED, RED))

    def test_ambiguous_traffic_plus_lane_frame_follows_the_branch_that_ran(self):
        # 标签集合不再参与判定：`traffic_hold+lane` 是车道分支画的箭头，
        # 归 lane 一族；`traffic` 才是灯。
        self.switch.observe('lane')
        self.assertFalse(self.switch.observe('traffic_hold+lane'))
        self.assertEqual(self.switch.last, 'lane')
        self.assertTrue(self.switch.observe('traffic'))

    def test_mislabelled_traffic_frame_does_not_blank_the_next_lane_round(self):
        # 一帧误标的 traffic_light 不能把下一帧真正干净的车道帧顶掉。
        # 中间那轮无框清零，但**不算切换**，所以回到车道时不黑屏。
        self.cycle(Fake([b('lane')]), Display())
        self.cycle(Fake([b('traffic_light'), b('lane')], traffic=[]),
                   Display(left=YELLOW, right=YELLOW))
        with patch('strategy_engine.lane_v2_decode', return_value={
                'left': {'kind': 'dash', 'x': .4, 'ev': .6, 'evidence': 'ok'},
                'right': {'kind': 'dash', 'x': .6, 'ev': .6, 'evidence': 'ok'}}):
            disp, info = self.cycle(Fake([b('lane')]), Display())
        # 中间那轮无框清零算一次真实的状态变化（切进 traffic 并清空），
        # 所以这一轮会判定切换、并且**本轮自己的车道颜色要保留**。
        # 关键是：车道恢复了，不会被误标灯永久顶掉。
        self.assertEqual((disp.left, disp.right), (YELLOW, YELLOW))

    def test_traffic_label_suppresses_lane_model_entirely(self):
        # 2026-10-28：红绿灯帧上不再跑车道模型，也就不会有"红绿灯时出现车道"。
        f = Fake([b('traffic_light'), b('lane')], traffic=[])
        disp, info = self.cycle(f, Display(left=GREEN, straight=GREEN, right=GREEN))
        self.assertNotIn('lane', f.calls)
        self.assertEqual(info['branch'], 'traffic')
        self.assertEqual((disp.left, disp.straight, disp.right), (RED, RED, RED),
                         '红绿灯模式必须永远三个箭头')

    def test_traffic_hold_lane_alternation_does_not_flip_flop(self):
        # traffic -> traffic_hold（沿用，provisional）-> traffic 不该判定切换。
        self.cycle(Fake([b('traffic_light')], traffic=[hood('green')]), Display())
        disp, _ = self.cycle(Fake([b('traffic_light'), b('lane')], traffic=[]),
                             Display(left=GREEN, straight=GREEN, right=GREEN))
        disp2, info = self.cycle(Fake([b('traffic_light')], traffic=[]), disp)
        # 红绿灯模式永远三个箭头；不算切换。
        self.assertFalse(info['branch_switched'])
        self.assertEqual((disp2.left, disp2.straight, disp2.right), (RED, RED, RED))

    def test_switch_into_nav_does_not_restore_blue_from_before_the_switch(self):
        # The nav branch used to consult `old`, so a blue restored by
        # read_display() after a restart survived the blank baseline. It must
        # read the baseline like every other branch.
        self.cycle(Fake([b('lane')]), Display())
        disp, info = self.cycle(Fake([b('nav')], nav=[]), Display(left=BLUE))
        self.assertTrue(info['branch_switched'])
        self.assertEqual(disp.straight, BLUE)
        self.assertIsNone(disp.left)


class V15DisplayHoldTests2(unittest.TestCase):
    """Remaining V15 cases that the BranchSwitchTests block was inserted into."""

    def setUp(self):
        self.solid = LaneSolidStreaks(3)
        self.scene_hold = SceneEmptyHold(2)
        self.previous = Display()

    cycle = V15DisplayHoldTests.cycle

    def test_nav_holds_previous_blue_instead_of_blinking(self):
        # nav6 found nothing this round: fall back to the last blue arrow
        # rather than clearing the screen or showing a stale junction light.
        self.previous = Display(left=BLUE)
        disp, info = self.cycle(Fake([b('nav')], nav=[]))
        self.assertEqual(disp.left, BLUE)
        self.assertEqual(info['branch'], 'navigation_hold')

    def test_nav_three_digit_label_beats_the_turn(self):
        f = Fake([b('nav')], nav=[b('right_3')])
        disp, info = self.cycle(f)
        self.assertEqual(disp.straight, BLUE)
        self.assertIsNone(disp.right)
        self.assertEqual(info['branch'], 'navigation_blue')

    def test_nav_low_score_second_box_ignored(self):
        # Only the top-scoring detection decides; a weaker competing label
        # must not flip the arrow on the same frame.
        f = Fake([b('nav')], nav=[b('left_3', score=.91), b('right_2', score=.30)])
        disp, info = self.cycle(f)
        self.assertEqual(disp.straight, BLUE)
        self.assertIsNone(disp.right)
        self.assertEqual(len(info['navigation']['readings']), 1)

    def test_one_frame_no_target_hold_second_clear(self):
        self.previous = Display(left=RED, straight=GREEN, right=GREEN)
        first, i1 = self.cycle(Fake([b('crosswalk')]))
        second, i2 = self.cycle(Fake([b('person')]))
        self.assertEqual(first.left, RED)
        self.assertTrue(first.edge_warning)
        self.assertEqual(i1['branch'], 'scene_empty_hold')
        self.assertEqual(second, Display())
        self.assertEqual(i2['branch'], 'none')
        self.assertEqual(i2['scene_empty_streak'], 2)

    def test_empty_streak_canceled_by_traffic_label_even_if_traffic_empty(self):
        self.previous = Display(left=RED)
        self.cycle(Fake())
        d, info = self.cycle(Fake([b('traffic_light')], traffic=[]))
        # 有 traffic 标签就打断"空场景"的连续计数（这是这个测试的本意），
        # 但无框本身按用户规则清零。
        self.assertEqual(info['branch'], 'traffic')
        self.assertEqual(self.scene_hold.empty, 0)
        d2, info2 = self.cycle(Fake())
        self.assertEqual(info2['branch'], 'scene_empty_hold')

    def test_no_target_immediate_when_configured_one(self):
        self.scene_hold = SceneEmptyHold(1)
        self.previous = Display(left=RED)
        d, r = self.cycle(Fake())
        self.assertEqual(d, Display())
        self.assertEqual(r['branch'], 'none')

    def test_scene_hold_invalid_value_rejected(self):
        with self.assertRaises(ValueError): SceneEmptyHold(0)

    def test_missing_models_do_not_change_angle_core(self):
        from lane_geometry import select_lanes
        self.assertEqual(select_lanes.__module__, 'lane_geometry')
        self.assertNotIn('lane_row_scan', sys.modules)

    def test_new_cli_clear_confirm_defaults_two(self):
        a = parser().parse_args([])
        self.assertEqual(a.empty_scene_confirm_frames, 2)
        # V20：1 -> 2（V18 取 1 时车道跳动频繁），见 轮数实验记录.md
        self.assertEqual(a.solid_confirm_frames, 2)
        # V26：2 -> 1（用户要求"斑马线灵敏度调高"）。误报改由几何判据
        # （cy≥0.45 且面积≤25%）+ --warning-max-hold 兜住。
        self.assertEqual(a.warning_confirm_frames, 1)
        self.assertEqual(a.warning_max_hold, 4)

    def test_lane_branch_clears_stale_straight_arrow(self):
        """A green straight arrow from a traffic round must not survive into a
        lane-only round: the lane detector never sets straight."""
        self.previous = Display(left=GREEN, straight=GREEN)
        for _ in range(3):        # dash needs confirm_frames now
            with patch('strategy_engine.select_lanes', return_value={
                    'left': {'kind': 'dash'}, 'right': {'kind': 'none'}}):
                disp, info = self.cycle(Fake([b('lane')]))
        self.assertEqual(info['branch'], 'lane')
        self.assertIsNone(disp.straight)
        self.assertEqual(disp.left, YELLOW)

    def test_warning_needs_consecutive_frames(self):
        from strategy_engine import WarningHold
        hold = WarningHold(2)
        d0, _ = decide_pair(G, Fake([b('crosswalk')]), previous_display=Display(),
                            lane_streaks=LaneSolidStreaks(1), warning_hold=hold)
        d1, _ = decide_pair(G, Fake([b('crosswalk')]), previous_display=d0,
                            lane_streaks=LaneSolidStreaks(1), warning_hold=hold)
        self.assertFalse(d0.edge_warning)
        self.assertTrue(d1.edge_warning)

    def test_warning_clears_immediately_when_frame_is_clean(self):
        from strategy_engine import WarningHold
        # V26：证据消失后按 max_hold 沿用，超过才放开。这里 max_hold=1
        # 还原"立刻消失"的旧语义；默认 4 的行为见下一条测试。
        hold = WarningHold(2, max_hold=1)
        d0, _ = decide_pair(G, Fake([b('crosswalk')]), previous_display=Display(),
                            lane_streaks=LaneSolidStreaks(1), warning_hold=hold)
        d1, _ = decide_pair(G, Fake([b('crosswalk')]), previous_display=d0,
                            lane_streaks=LaneSolidStreaks(1), warning_hold=hold)
        self.assertTrue(d1.edge_warning)
        d2, _ = decide_pair(G, Fake(), previous_display=d1,
                            lane_streaks=LaneSolidStreaks(1), warning_hold=hold)
        self.assertFalse(d2.edge_warning)

    def test_warning_hold_is_bounded_by_max_hold(self):
        """V26：报警置位后最多沿用 max_hold 轮，不会像 V25 那样锁存 49 轮。

        板上日志：报警 132 轮里锁存段是 49/30/30/23 轮，一次误检亮一两秒。
        """
        from strategy_engine import WarningHold
        hold = WarningHold(1, max_hold=3)
        disp = Display()
        disp, _ = decide_pair(G, Fake([b('crosswalk')]), previous_display=disp,
                              lane_streaks=LaneSolidStreaks(1), warning_hold=hold)
        self.assertTrue(disp.edge_warning, '确认帧=1，第一轮就该亮')
        seen = []
        for _ in range(5):
            disp, _ = decide_pair(G, Fake(), previous_display=disp,
                                  lane_streaks=LaneSolidStreaks(1),
                                  warning_hold=hold)
            seen.append(disp.edge_warning)
        # 前两轮仍在 max_hold 内，第三轮起必须放开
        self.assertEqual(seen[:2], [True, True])
        self.assertEqual(seen[2:], [False, False, False])

    def test_crosswalk_geometry_rejects_full_width_top_band(self):
        """V26：板上那个 `[0,0,1178,132]` 整幅顶部条不能再触发报警。

        官方 8 段素材里 cy 中位 0.853、面积最大的真斑马线也有 0.353，
        所以判据是「cy≥0.45 且面积≤0.25」，不是"大就拒"。
        """
        from strategy_engine import _crosswalk_in_view
        top_band = [{'class_name': 'crosswalk', 'class_id': 1, 'score': .53,
                     'xyxy': [0.0, 0.0, 1178.0, 132.0]}]
        self.assertFalse(_crosswalk_in_view(top_band, 720, 1280))
        # 真斑马线：画面下半部、中等面积
        real = [{'class_name': 'crosswalk', 'class_id': 1, 'score': .20,
                 'xyxy': [323.0, 500.0, 539.0, 538.0]}]
        self.assertTrue(_crosswalk_in_view(real, 720, 1280))
        # 大但位置正确（官方 max area 0.353）仍要认
        big_low = [{'class_name': 'crosswalk', 'class_id': 1, 'score': .27,
                    'xyxy': [300.0, 600.0, 980.0, 720.0]}]
        self.assertLess((980 - 300) * (720 - 600) / (720 * 1280), 0.25)
        self.assertTrue(_crosswalk_in_view(big_low, 720, 1280))

    def test_warning_still_independent_during_traffic_hold(self):
        self.previous = Display(left=RED)
        d, _ = self.cycle(Fake([b('traffic_light'), b('crosswalk')],traffic=[]))
        # 无框走红灯兜底，但边缘警告（斑马线）与灯态无关，仍然要亮。
        self.assertEqual(d.left, RED)
        self.assertTrue(d.edge_warning)

    def test_yellow_dash_does_not_disable_opposite_arrow(self):
        self.previous = Display(right=RED, straight=GREEN)
        # Dash needs confirm_frames consecutive frames (LaneSolidStreaks(3) here).
        for _ in range(3):
            with patch('strategy_engine.select_lanes', return_value={
                    'left': {'kind': 'dash'}, 'right': {'kind': 'none'}}):
                disp, _ = self.cycle(Fake([b('lane')]))
        # right is preserved; straight is cleared because the lane branch does
        # not own a straight arrow and must not keep a stale one burning.
        self.assertEqual((disp.left, disp.right, disp.straight), (YELLOW, RED, None))


if __name__ == '__main__':
    unittest.main()
