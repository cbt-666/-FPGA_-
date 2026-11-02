"""A blue navigation arrow must not blink off while the scene classifier misses.

Field evidence, rk3568_v15_10.log, 108 navigation rounds:

  * the five-class model is NOT flickering -- every nav detection scores
    0.71~0.97, and nav appears in only 2 of 114 lane-section rounds;
  * the navigation ARROW submodel finds an arrow in 7/12 rounds when real grey
    is delivered but 0/46 rounds on the thresholded (0/253) frames the FPGA
    currently sends, because blue navigator arrows (#4285F4, luminance 125)
    fall below the binary threshold;
  * with no arrow, strategy_engine holds blue (`navigation_hold`), but two
    consecutive scene misses then trip SceneEmptyHold and blank the screen.

Result before this change: 52 of 108 navigation rounds were dark. That is the
flicker the operator reports.

A blue arrow is a ROUTE indication, not a perception reading: a few rounds of
scene-classifier silence does not contradict it. Holding it costs nothing but a
route change is noticed a few rounds later; blanking it makes the arrow flash
off and on, which is what the driver actually sees.
"""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import Display, BLUE, YELLOW
from strategy_engine import (NavigationHold, SceneEmptyHold, SceneThrottle,
                             WarningHold, decide_pair)
from lane_stability import LaneSolidStreaks


class NavigationHoldUnitTests(unittest.TestCase):
    def test_holds_for_the_configured_number_of_misses(self):
        hold = NavigationHold(hold_frames=3)
        self.assertTrue(hold.observe(True))
        for _ in range(3):
            self.assertTrue(hold.observe(False))
        self.assertFalse(hold.observe(False))

    def test_a_nav_hit_resets_the_miss_counter(self):
        hold = NavigationHold(hold_frames=3)
        hold.observe(True)
        hold.observe(False)
        hold.observe(False)
        self.assertTrue(hold.observe(True))
        self.assertTrue(hold.observe(False))

    def test_zero_means_clear_immediately(self):
        hold = NavigationHold(hold_frames=0)
        hold.observe(True)
        self.assertFalse(hold.observe(False))

    def test_rejects_negative(self):
        with self.assertRaises(ValueError):
            NavigationHold(hold_frames=-1)


class _NoTargets:
    """Scene classifier sees nothing for this frame."""
    def scene(self, gray):
        return []

    def navigation(self, gray):
        return []

    def traffic_light(self, gray):
        return []

    def lane_mask(self, gray, binary=None):
        return np.zeros((720, 1280), np.uint8)


class _Session:
    """Mirrors main.py: the holds are created ONCE and threaded through rounds."""

    def __init__(self, nav_hold=None, empty_frames=2):
        self.detectors = _NoTargets()
        self.empty = SceneEmptyHold(empty_frames)
        self.nav_hold = nav_hold

    def step(self, previous):
        return decide_pair(np.zeros((720, 1280), np.uint8),
                           self.detectors, previous_display=previous,
                           lane_streaks=LaneSolidStreaks(1),
                           empty_scene_hold=self.empty,
                           scene_throttle=None, warning_hold=WarningHold(2),
                           lane_backend='scan', nav_hold=self.nav_hold)[1]


class NavBlueSurvivesSceneMissTests(unittest.TestCase):
    def test_blue_survives_past_the_empty_scene_limit(self):
        """Baseline: without a nav hold, two empty rounds blank a blue arrow."""
        s = _Session(nav_hold=None)
        first = s.step(Display(left=BLUE))
        self.assertEqual(first['display']['left'], 'blue',
                         'first miss should be held by retained()')
        second = s.step(Display(left=BLUE))
        self.assertEqual(second['display']['left'], 'hidden',
                         'baseline: second miss clears (this is the reported flicker)')

    def test_hold_keeps_blue_through_a_long_dropout(self):
        s = _Session(nav_hold=NavigationHold(hold_frames=10))
        blue = Display(left=BLUE)
        for i in range(10):
            result = s.step(blue)
            self.assertEqual(result['display']['left'], 'blue',
                             f'round {i} blanked a held blue arrow')
            self.assertEqual(result['branch'], 'navigation_hold')

    def test_hold_eventually_gives_up(self):
        """A real scene change must still be able to clear the arrow."""
        s = _Session(nav_hold=NavigationHold(hold_frames=3))
        results = [s.step(Display(left=BLUE)) for _ in range(6)]
        self.assertEqual(results[-1]['display']['left'], 'hidden')

    def test_hold_does_not_resurrect_a_red_or_yellow_lane_arrow(self):
        """Only a BLUE arrow gets the long hold; lane colours keep the 2-round rule."""
        for color, name in ((YELLOW, 'yellow'), (0, 'red')):
            s = _Session(nav_hold=NavigationHold(hold_frames=10))
            s.step(Display(left=color))
            second = s.step(Display(left=color))
            self.assertEqual(second['display']['left'], 'hidden',
                             f'{name} must still clear on the normal 2-round rule')


class SceneThrottleNavTests(unittest.TestCase):
    """--scene-every must not double the length of a navigation dropout.

    A cached scene answer is what makes the throttle save 274 ms/round on the
    lane branch, but on a navigation screen a cached MISS keeps the route arrow
    dark for an extra round, and a cached HIT hides a real junction for one.
    """

    def test_lane_scene_still_throttles(self):
        t = SceneThrottle(every=2)
        t.observe([{'class_name': 'lane'}])
        self.assertEqual([t.should_run() for _ in range(4)], [False] * 4)

    def test_navigation_scene_reuses_at_most_nav_max_hold(self):
        # 2026-10-28: a navigation answer may be reused for nav_max_hold rounds
        # (468 ms per scene pass on the board), then the scene model must run.
        # Mirror the real loop: should_run() then observe() or reuse().
        t = SceneThrottle(every=2, nav_max_hold=2)
        t.observe([{'class_name': 'nav'}])
        ran = []
        for _ in range(4):
            if t.should_run():
                ran.append(True); t.observe([{'class_name': 'nav'}])
            else:
                ran.append(False); t.reuse()
        self.assertEqual(ran, [False, False, True, False])

    def test_v283_two_round_cadence_for_traffic_and_nav(self):
        """V28.3：traffic/nav max_hold=1 -> 真跑 1 + 复用 1 = 两轮一次。"""
        for scenes in ([{'class_name': 'traffic_light'}],
                       [{'class_name': 'nav'}]):
            t = SceneThrottle(every=2, nav_max_hold=1, traffic_max_hold=1)
            t.observe(scenes)
            ran = []
            for _ in range(4):
                if t.should_run():
                    ran.append(True); t.observe(scenes)
                else:
                    ran.append(False); t.reuse()
            self.assertEqual(ran, [False, True, False, True], scenes)

    def test_v285_scene_runs_every_round(self):
        """V28.5：用户「决策模型改成每次调用」——every=1 且两个 max_hold=0。

        三条分支（红绿灯/导航/车道）都必须每轮真跑，不能再有复用轮。
        """
        for scenes in ([{'class_name': 'traffic_light'}],
                       [{'class_name': 'nav'}],
                       [{'class_name': 'lane'}]):
            t = SceneThrottle(every=1, max_hold=0, nav_max_hold=0,
                              traffic_max_hold=0)
            t.observe(scenes)
            ran = []
            for _ in range(4):
                if t.should_run():
                    ran.append(True); t.observe(scenes)
                else:
                    ran.append(False); t.reuse()
            self.assertEqual(ran, [True] * 4, scenes)

    def test_navigation_miss_forces_a_real_scene_pass(self):
        t = SceneThrottle(every=3, nav_max_hold=5)
        t.observe([{'class_name': 'nav'}])
        self.assertFalse(t.should_run())
        t.force_next()
        self.assertTrue(t.should_run())

    def test_navigation_mixed_with_lane_is_limited_too(self):
        t = SceneThrottle(every=3, nav_max_hold=1)
        t.observe([{'class_name': 'lane'}, {'class_name': 'nav'}])
        self.assertFalse(t.should_run())
        t.reuse()
        self.assertTrue(t.should_run())

    def test_max_hold_still_forces_a_rerun(self):
        t = SceneThrottle(every=2, max_hold=2)
        t.observe([{'class_name': 'lane'}])
        t.reuse()
        t.reuse()
        self.assertTrue(t.should_run())


if __name__ == '__main__':
    unittest.main()
