"""V17: the six-class navigation head (direction x digit count) and its gates.

nav6 replaced "3-class arrow detector + OpenCV template digit reader" with a
single YOLO head whose label already carries the distance digit count. These
tests pin the label->arrow mapping, the cross-class NMS that makes it work, and
the two screen-space gates the model README says the board must apply.
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import models_runtime
import vision_common
from models_runtime import nav_roi_ok, nav_size_ok, NAV_CLASSES, NAV_N
from strategy_engine import _nav_label, eligible_navigation

G = np.zeros((720, 1280), np.uint8)


def det(class_name, score=.9, xyxy=(300, 450, 400, 510)):
    return {'class_name': class_name, 'class_id': NAV_CLASSES.index(class_name),
            'score': score, 'xyxy': list(xyxy)}


class LabelTests(unittest.TestCase):
    def test_splits_every_shipped_class(self):
        expected = {'right_3': ('right', 3), 'right_2': ('right', 2),
                    'left_3': ('left', 3), 'left_2': ('left', 2),
                    'uturn_3': ('uturn', 3), 'uturn_2': ('uturn', 2)}
        for name, pair in expected.items():
            with self.subTest(name=name):
                self.assertEqual(_nav_label(name), pair)

    def test_rejects_a_label_without_a_digit_count(self):
        for bad in ('right', '', 'straight', 'right_x'):
            with self.subTest(bad=bad):
                self.assertEqual(_nav_label(bad), (None, None))


class ArrowMappingTests(unittest.TestCase):
    """2-digit turns light the turn; 3-digit and U-turn light blue straight."""

    def selected(self, class_name):
        return eligible_navigation([det(class_name)])[0]

    def test_all_six_classes(self):
        expected = {'right_2': 'right', 'left_2': 'left',
                    'right_3': 'straight', 'left_3': 'straight',
                    'uturn_2': 'straight', 'uturn_3': 'straight'}
        for name, arrow in expected.items():
            with self.subTest(name=name):
                self.assertEqual(self.selected(name), arrow)

    def test_no_detection_returns_none(self):
        arrow, evidence = eligible_navigation([])
        self.assertIsNone(arrow)
        self.assertEqual(evidence['reason'], 'no_supported_arrow_detection')

    def test_top_score_decides(self):
        arrow, evidence = eligible_navigation(
            [det('left_2', score=.40), det('right_2', score=.91)])
        self.assertEqual(arrow, 'right')
        self.assertEqual(len(evidence['readings']), 1)

    def test_unknown_label_does_not_invent_an_arrow(self):
        arrow, evidence = eligible_navigation([det('right_3') | {'class_name': 'mystery'}])
        self.assertIsNone(arrow)
        self.assertEqual(evidence['reason'], 'unknown_nav_class')

    def test_evidence_records_turn_and_digits(self):
        _, evidence = eligible_navigation([det('left_2')])
        reading = evidence['readings'][0]
        self.assertEqual((reading['turn'], reading['digits']), ('left', 2))
        self.assertEqual(reading['selected'], 'left')


class AgnosticNMSTests(unittest.TestCase):
    """One arrow frequently scores as both right_3 and right_2.

    Per-class NMS leaves one box per class, which reads as two directions;
    cross-class NMS keeps the stronger label only.
    """

    def outputs(self, c1, c2, s1=.85, s2=.60):
        box = np.zeros((1, 4, NAV_N), np.float32)
        cls = np.zeros((1, len(NAV_CLASSES), NAV_N), np.float32)
        for i, (c, s) in enumerate(((c1, s1), (c2, s2))):
            box[0, :, i] = (300, 480, 90, 50)          # identical -> IoU 1.0
            cls[0, NAV_CLASSES.index(c), i] = s
        return [box, cls]

    def decode(self, agnostic):
        return vision_common.decode_yolo(
            self.outputs('right_3', 'right_2'), NAV_N, NAV_CLASSES,
            (1.0, 0, 0), G.shape, {s: .25 for s in NAV_CLASSES},
            iou=.5, agnostic=agnostic)

    def test_agnostic_keeps_only_the_stronger_label(self):
        kept = self.decode(agnostic=True)
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]['class_name'], 'right_3')

    def test_per_class_would_keep_both(self):
        kept = self.decode(agnostic=False)
        self.assertEqual(sorted(k['class_name'] for k in kept),
                         ['right_2', 'right_3'])

    def test_agnostic_preserves_distinct_boxes(self):
        boxes = np.zeros((1, 4, NAV_N), np.float32)
        cls = np.zeros((1, len(NAV_CLASSES), NAV_N), np.float32)
        for i, (cx, k, s) in enumerate(((300, 4, .9), (900, 1, .8))):
            boxes[0, :, i] = (cx, 480, 90, 50)
            cls[0, k, i] = s
        kept = vision_common.decode_yolo(
            [boxes, cls], NAV_N, NAV_CLASSES, (1.0, 0, 0), G.shape,
            {s: .25 for s in NAV_CLASSES}, iou=.5, agnostic=True)
        self.assertEqual(sorted(k['class_name'] for k in kept),
                         ['right_2', 'uturn_3'])


class ScreenGateTests(unittest.TestCase):
    """README gates, restated in the board's 1280x720 frame (1920x1080 * 2/3)."""

    def test_roi_accepts_the_navigation_screen(self):
        self.assertTrue(nav_roi_ok(240, 460))
        self.assertTrue(nav_roi_ok(10, 320))
        self.assertTrue(nav_roi_ok(470, 600))

    def test_roi_rejects_outside_the_screen_area(self):
        self.assertFalse(nav_roi_ok(900, 460), 'right of the screen')
        self.assertFalse(nav_roi_ok(240, 100), 'sky')
        self.assertFalse(nav_roi_ok(240, 700), 'dashboard')

    def test_size_accepts_card_and_3d_arrow(self):
        self.assertTrue(nav_size_ok(100, 45))
        self.assertTrue(nav_size_ok(130, 130))

    def test_size_rejects_tiny_and_huge(self):
        self.assertFalse(nav_size_ok(40, 20))
        self.assertFalse(nav_size_ok(260, 40))
        self.assertFalse(nav_size_ok(100, 200))


class _OneShotHandle:
    """Emits a fixed detection set regardless of input."""

    def __init__(self, boxes, scores):
        self.boxes, self.scores = boxes, scores
        self.last_infer_ms = 1.0
        self.first = False

    def run(self, tensor, **kw):
        n = NAV_N
        box = np.zeros((1, 4, n), np.float32)
        cls = np.zeros((1, len(NAV_CLASSES), n), np.float32)
        for i, ((cx, cy, w, h), (k, s)) in enumerate(zip(self.boxes, self.scores)):
            box[0, :, i] = (cx, cy, w, h)
            cls[0, k, i] = s
        self.seen = np.asarray(tensor)
        return [box, cls]

    def close(self):
        pass


class NavigationAdapterTests(unittest.TestCase):
    def run_detector(self, boxes, scores):
        handle = _OneShotHandle(boxes, scores)

        class _M(models_runtime.LazyDetectors):
            def get(self, name):
                return handle

        m = _M('s.rknn', 'nav6_720.rknn', 'h.rknn', 'l.rknn')
        return m, handle, m.navigation(G)

    def test_filters_by_roi_and_size(self):
        boxes = [(240, 460, 100, 60),   # on screen, right size  -> keep
                 (900, 460, 100, 60),   # right of the screen    -> roi
                 (240, 100, 100, 60),   # sky                    -> roi
                 (240, 460, 40, 20)]    # too small              -> size
        scores = [(1, .9), (1, .9), (1, .9), (1, .9)]
        m, handle, kept = self.run_detector(boxes, scores)
        self.assertEqual(len(kept), 1)
        stages = m.last_model_stages['nav']
        self.assertEqual(stages['kept'], 1)
        self.assertEqual(stages['rejected'], {'roi': 2, 'size': 1})

    def test_fed_tensor_is_brightness_grey_three_channel(self):
        _, handle, _ = self.run_detector([(240, 460, 100, 60)], [(3, .9)])
        self.assertEqual(handle.seen.shape, (1, 720, 1280, 3))
        self.assertEqual(handle.seen.dtype, np.float32)
        self.assertEqual(handle.seen[0, :, :, 0].max(), handle.seen[0, :, :, 1].max())

    def test_yrg_colour_markers_become_luminance_not_colour(self):
        frame = G.copy()
        frame[10, 10] = 255          # pure green sentinel
        handle = _OneShotHandle([], [])

        class _M(models_runtime.LazyDetectors):
            def get(self, name):
                return handle

        _M('s.rknn', 'nav6_720.rknn', 'h.rknn', 'l.rknn').navigation(frame)
        self.assertEqual(float(handle.seen[0, 10, 10, 0]), 149.0)


class LaneClusterWidthTests(unittest.TestCase):
    """A legal lane line wider than the cap is thrown away, not clamped.

    Three held-out lines measured 144~153 px on A768 (cap used to be 0.18W=138),
    so they were dropped whole and reported as "no lane line". Raising the cap
    to 0.20 recovers two of them; this pins the value so it cannot drift back.
    """

    def band_mask(self, width, height=432, image_w=768, kind=2):
        m = np.zeros((height, image_w), np.uint8)
        a, b = int(height * 0.45), int(height * 0.90)
        x0 = image_w // 2 - width // 2 - 120
        m[a:b, x0:x0 + width] = kind
        return m

    def test_cap_is_wide_enough_for_measured_lines(self):
        import lane_v2
        self.assertGreaterEqual(lane_v2.MAX_WIDTH_FRAC, 0.20)
        self.assertEqual(int(768 * lane_v2.MAX_WIDTH_FRAC), 153)

    def test_144px_line_is_kept(self):
        import lane_v2
        d = lane_v2.decode(self.band_mask(144))
        self.assertEqual(d['left']['kind'], 'solid')
        self.assertNotEqual(d['left']['x'], None)

    def test_153px_line_is_kept(self):
        import lane_v2
        d = lane_v2.decode(self.band_mask(153))
        self.assertEqual(d['left']['kind'], 'solid')

    def test_crosswalk_sized_band_is_still_rejected(self):
        # 0.30W = 230 px: wider than any lane line, this is a crossing band.
        import lane_v2
        d = lane_v2.decode(self.band_mask(230))
        self.assertEqual(d['left']['kind'], 'none')


if __name__ == '__main__':
    unittest.main()