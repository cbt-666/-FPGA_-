"""A lane arrow must be traceable when it changes on almost no evidence.

The v2 lane model outputs a 3-class mask; `lane_v2.decode` reports how much
signal backs each side (`ev`). A cluster made of a couple of faint columns is
one bad frame away from flipping class, so the engine holds the previous
colour on `evidence='weak'` instead of repainting.

Measured on the official held-out set: non-zero evidence p10 = 0.10,
median 0.66 — i.e. the weak band is the bottom decile, exactly the readings
that used to flicker.
"""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import Display
import lane_v2
from strategy_engine import decide_pair, WarningHold
from lane_stability import LaneSolidStreaks

H, W = 432, 768          # the model's own resolution
BAND = int(H * 0.90) - int(H * 0.45)


def make_mask(weak_right=True, strong_left=True):
    """A long left line plus either a strong solid or a faint fragment right."""
    mask = np.zeros((H, W), np.uint8)
    if strong_left:
        mask[int(H * .50):int(H * .88), 150:162] = 2          # solid
    if weak_right:
        # 2 columns x 9 rows inside the band -> ev ~= 2 * 9/BAND < WEAK
        mask[int(H * .60):int(H * .60) + 9, 640:642] = 2
    else:
        mask[int(H * .50):int(H * .88), 620:634] = 1          # dash
    return mask


class EvidenceGradingTests(unittest.TestCase):
    def test_faint_fragment_is_weak(self):
        out = lane_v2.decode(make_mask(weak_right=True))
        self.assertEqual(out['right']['evidence'], 'weak')
        self.assertLess(out['right']['ev'], lane_v2.WEAK_EVIDENCE)

    def test_long_line_is_ok(self):
        out = lane_v2.decode(make_mask(weak_right=False))
        self.assertEqual(out['right']['kind'], 'dash')
        self.assertEqual(out['right']['evidence'], 'ok')

    def test_missing_side_is_none(self):
        out = lane_v2.decode(np.zeros((H, W), np.uint8))
        self.assertEqual(out['left']['kind'], 'none')
        self.assertEqual(out['left']['evidence'], 'none')

    def test_band_must_cover_the_markings(self):
        # A mark above the reference band contributes nothing.
        m = np.zeros((H, W), np.uint8)
        m[5:40, 150:162] = 2
        self.assertEqual(lane_v2.decode(m)['left']['kind'], 'none')


class _LaneScene:
    """Minimal detector stub: one lane box, no traffic, no navigation."""
    lane_variant = 'gray640'

    def __init__(self, mask):
        self._mask = mask

    def scene(self, gray):
        return [{'class_name': 'lane', 'class_id': 2, 'score': .9,
                 'xyxy': [0, 400, 1280, 700]}]

    def navigation(self, gray):
        return []

    def housing(self, gray):
        return []

    def lane_mask(self, gray, binary=None):
        return self._mask


def run(mask, previous):
    return decide_pair(np.zeros((720, 1280), np.uint8),
                       _LaneScene(mask), previous_display=previous,
                       lane_streaks=LaneSolidStreaks(1),
                       empty_scene_hold=None, scene_throttle=None,
                       warning_hold=WarningHold(2))


class WeakEvidenceChangeTests(unittest.TestCase):
    def test_weak_evidence_does_not_change_the_arrow(self):
        """A two-column fragment must not repaint an arrow decided otherwise."""
        _, result = run(make_mask(weak_right=True), Display(left=1, right=0))
        self.assertEqual(result['display']['right'], 'red')
        self.assertEqual(result['lane_updates']['right'], 'weak_hold_previous')
        self.assertNotIn('lane_weak_evidence_change', result)

    def test_strong_evidence_still_changes_the_arrow(self):
        """The weak-hold rule must not freeze a genuinely dashed line."""
        _, result = run(make_mask(weak_right=False), Display(left=1, right=1))
        self.assertEqual(result['display']['right'], 'yellow')
        self.assertEqual(result['lane_updates']['right'], 'dash_yellow_immediate')

    def test_weak_side_keeps_colour_only_when_it_would_not_change(self):
        """2026-10-29 V17_8：只有"想保持的颜色和屏幕一致"时才按住。

        旧规则是"证据弱就一律不改色"，代价是变道后要等好几轮才显示。
        现在：弱证据 + 想变成**别的**颜色 -> 那是实质变化，立刻生效；
        弱证据 + 想保持同色 -> 按住（本来就一样，没有损失）。
        这个用例是 Weak 侧想从 yellow 变 red，属于实质变化，必须立即改。
        """
        _, result = run(make_mask(weak_right=True), Display(left=1, right=3))
        self.assertEqual(result['display']['right'], 'red')
        self.assertNotEqual(result['lane_updates'].get('right'), 'weak_hold_previous')


if __name__ == '__main__':
    unittest.main()
