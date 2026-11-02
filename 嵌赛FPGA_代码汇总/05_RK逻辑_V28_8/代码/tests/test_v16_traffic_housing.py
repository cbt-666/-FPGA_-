"""Traffic: housing boxes -> colour -> left/straight/right.

The housing detector answers only "where is a light box". Everything else is
algorithm:

  * colour by counting YRG 254/255 inside the box;
  * map box count to direction, left to right (2026-10-28, fixed by the user):
    1 = straight only, 2 = left + straight, 3+ = left + straight + right.
    Right turn is GO by default when there is no right-turn lamp.

No geometric filtering. The board log (rk3568_v17_gray.log) showed the filters
kept only 14 of 248 rounds because they discarded real boxes on aspect ratio;
box COUNT decides the direction, so dropping a box turns a 3-lamp row into a
2-lamp one and shifts every direction left.
"""
import unittest
import numpy as np

import traffic_rules as tr
from traffic_rules import STATE_GO, STATE_STOP


def yrg_with(boxes, w=1280, h=720):
    """Grey YRG frame with the colour of each box painted inside it."""
    y = np.full((h, w), 60, np.uint8)
    for colour, (x1, y1, x2, y2) in boxes:
        value = {'red': 254, 'green': 255}.get(colour)
        if value is not None:
            y[int(y1):int(y2), int(x1):int(x2)] = value
    return y


def box(colour, x=400, y=150, w=80, h=40, score=.9):
    return [float(x), float(y), float(x + w), float(y + h), score]


class NoGeometricDropTests(unittest.TestCase):
    """Every box the model returns is used; geometry never removes one."""

    def test_low_box_is_still_used(self):
        # A detection at cy/H = 0.9 used to be discarded as a plate or tail
        # light. It is now kept -- wrong boxes cost one direction, but dropping
        # them shifts every direction left.
        v = tr.judge([box('red', y=620)], yrg_with([('red', (400, 620, 480, 660))]))
        self.assertEqual(len(v.boxes), 1)
        self.assertEqual(v.layout, 'single_all_three')

    def test_vertical_lamp_is_still_used(self):
        # The S2TLD-style vertical lamp (aspect 0.22) was the shape the old
        # MIN_ASPECT rejected for 96.5% of the board's real boxes.
        v = tr.judge([box('red', w=20, h=90)], yrg_with([('red', (400, 150, 420, 240))]))
        self.assertEqual(len(v.boxes), 1)

    def test_oversized_box_is_still_used(self):
        v = tr.judge([box('red', w=500, h=300)],
                     yrg_with([('red', (400, 150, 900, 450))]))
        self.assertEqual(len(v.boxes), 1)

    def test_far_apart_boxes_are_both_used(self):
        # Boxes 900 px apart used to be judged "not one lamp group" and reduced
        # to the single most confident one.
        boxes = [box('red', x=200), box('green', x=1150)]
        y = yrg_with([('red', (200, 150, 280, 190)), ('green', (1150, 150, 1230, 190))])
        v = tr.judge(boxes, y)
        self.assertEqual(v.layout, 'left_straight')
        self.assertEqual(len(v.boxes), 2)


class ColourTests(unittest.TestCase):
    def test_red_and_green_counted_from_yrg_sentinels(self):
        y = yrg_with([('red', (400, 150, 480, 190)),
                      ('green', (600, 150, 680, 190))])
        self.assertEqual(tr._colour_of([400, 150, 480, 190, .9], y), 'red')
        self.assertEqual(tr._colour_of([600, 150, 680, 190, .9], y), 'green')

    def test_no_coloured_pixels_is_off(self):
        y = yrg_with([])
        self.assertEqual(tr._colour_of([400, 150, 480, 190, .9], y), 'off')

    def test_off_maps_to_stop(self):
        v = tr.judge([box('off')], yrg_with([]))
        self.assertEqual(v.layout, 'single_all_three')
        self.assertEqual(v.states['straight'], STATE_STOP)


class DirectionMappingTests(unittest.TestCase):
    def test_one_box_lights_left_and_straight(self):
        # 2026-10-28 用户规则：必须每轮输出三个方向。
        # 一个框 = 该框颜色同时给左转和直行，右转默认绿。
        # 旧版 left=None，屏幕上左边永远是暗的，用户看到"左边不显示"。
        boxes = [box('red')]
        v = tr.judge(boxes, yrg_with([('red', (400, 150, 480, 190))]))
        self.assertEqual(v.layout, 'single_all_three')
        self.assertEqual(v.states['left'], STATE_STOP)
        self.assertEqual(v.states['straight'], STATE_STOP)
        self.assertEqual(v.states['right'], STATE_GO, 'right turns by default')

    def test_one_green_box_allows_straight(self):
        v = tr.judge([box('green')], yrg_with([('green', (400, 150, 480, 190))]))
        self.assertEqual(v.states['straight'], STATE_GO)
        self.assertEqual(v.states['right'], STATE_GO)

    def test_two_boxes_are_left_and_straight(self):
        boxes = [box('red', x=400), box('green', x=600)]
        y = yrg_with([('red', (400, 150, 480, 190)),
                      ('green', (600, 150, 680, 190))])
        v = tr.judge(boxes, y)
        self.assertEqual(v.layout, 'left_straight')
        self.assertEqual(v.states['left'], STATE_STOP)
        self.assertEqual(v.states['straight'], STATE_GO)
        self.assertEqual(v.states['right'], STATE_GO,
                         'no right-turn lamp: right turn is allowed by default')

    def test_two_boxes_red_right_is_straight_not_left(self):
        # The old mapping called the right-hand box "right turn" and blanked
        # straight, which is the dangerous direction to get wrong.
        boxes = [box('green', x=400), box('red', x=600)]
        y = yrg_with([('green', (400, 150, 480, 190)),
                      ('red', (600, 150, 680, 190))])
        v = tr.judge(boxes, y)
        self.assertEqual((v.states['left'], v.states['straight'], v.states['right']),
                         (STATE_GO, STATE_STOP, STATE_GO))

    def test_three_boxes_are_left_straight_right_in_x_order(self):
        boxes = [box('red', x=600), box('green', x=400), box('red', x=800)]
        y = yrg_with([('green', (400, 150, 480, 190)),
                      ('red', (600, 150, 680, 190)),
                      ('red', (800, 150, 880, 190))])
        v = tr.judge(boxes, y)          # unsorted on purpose
        self.assertEqual(v.layout, 'left_straight_right')
        self.assertEqual((v.states['left'], v.states['straight'], v.states['right']),
                         (STATE_GO, STATE_STOP, STATE_STOP))

    def test_four_boxes_use_the_first_three(self):
        boxes = [box('red', x=400 + i * 90, w=70) for i in range(4)]
        y = yrg_with([('red', (400 + i * 90, 150, 470 + i * 90, 190)) for i in range(4)])
        v = tr.judge(boxes, y)
        self.assertEqual(v.layout, 'left_straight_right')
        self.assertEqual(len(v.boxes), 3)

    def test_no_box_reports_no_direction(self):
        # layout 必须是 no_housing_box：strategy_engine 靠这个名字决定"清零上一次箭头"。
        v = tr.judge([], yrg_with([]))
        self.assertEqual(v.layout, 'no_housing_box')
        self.assertEqual(v.states, {})

    def test_scattered_set_keeps_all_three(self):
        # These used to be judged "not one lamp group" and reduced to the
        # single most confident box. Box count is what the mapping reads, so
        # all three are kept and ordered left to right.
        boxes = [box('red', x=200, score=.9), box('red', x=320, score=.85),
                 box('green', x=1150, score=.6)]
        y = yrg_with([('red', (200, 150, 280, 190)), ('red', (320, 150, 400, 190)),
                      ('green', (1150, 150, 1230, 190))])
        v = tr.judge(boxes, y)
        self.assertEqual(v.layout, 'left_straight_right')
        self.assertEqual(len(v.boxes), 3)
        self.assertEqual((v.states['left'], v.states['straight'], v.states['right']),
                         (STATE_STOP, STATE_STOP, STATE_GO))


if __name__ == '__main__':
    unittest.main()
