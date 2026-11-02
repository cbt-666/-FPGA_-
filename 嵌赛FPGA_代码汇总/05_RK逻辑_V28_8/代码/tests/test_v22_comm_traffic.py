"""V22 红绿灯与通信改造的回归测试。

覆盖 2026-10-31 板端 39 帧抓图实测出来的两个真问题：
  1. 框数从 3 掉到 2 时方向语义整体错位（直行绿 → 红）
  2. 灯箱误检（数字/限速牌/鬼影）占用方向位置
另外守住 V24 的"只有灰度"通信契约。
"""
import unittest
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import traffic_boxes
import traffic_rules
from strategy_engine import LayoutHold
from protocol import RED, GREEN


def yrg_with_boxes(rects, h=720, w=1280):
    """造一张 YRG：每个 rect=(x1,y1,x2,y2,value) 涂成指定值。"""
    img = np.full((h, w), 120, np.uint8)
    for x1, y1, x2, y2, v in rects:
        img[int(y1):int(y2), int(x1):int(x2)] = v
    return img


class BoxCountShrinkTests(unittest.TestCase):
    """板端实测：轮 62 三框 [红,绿,绿] → 轮 72 两框 [红,红]。

    少一个框时如果不做保护，2 框布局会把右框当直行灯 → 直行从绿翻红。
    """

    def test_layout_hold_reports_held_count_when_boxes_drop(self):
        hold = LayoutHold(confirm_frames=2)
        # 先稳定确认 3 框
        for _ in range(3):
            self.assertEqual(hold.observe([1, 2, 3]), 3)
        # 突然只剩 2 框：want 仍是 3（还没确认），调用方据此本轮沿用旧灯色
        want = hold.observe([1, 2])
        self.assertEqual(want, 3, '框数骤降时不能立刻按 2 框重新解释')
        # 连续 2 轮都是 2 框，才允许切换
        hold.observe([1, 2])
        self.assertEqual(hold.observe([1, 2]), 2)

    def test_three_then_two_does_not_flip_straight(self):
        """同样内容，3 框是绿 / 2 框会变红 —— 这就是要防的翻色。"""
        img = yrg_with_boxes([
            (100, 100, 160, 200, 254),   # 左：红
            (300, 100, 360, 200, 255),   # 中：绿
            (500, 100, 560, 200, 255),   # 右：绿
        ])
        three = traffic_rules.judge(
            [[100, 100, 160, 200, .9], [300, 100, 360, 200, .9],
             [500, 100, 560, 200, .9]], img)
        self.assertEqual(three.states['straight'], 'go')

        # 只漏掉最右那个框 -> 2 框布局里右框变直行灯，直行会翻成 stop
        two = traffic_rules.judge(
            [[100, 100, 160, 200, .9], [300, 100, 360, 200, .9]], img)
        self.assertEqual(two.states['straight'], 'go',
                         '这个用例里中框仍是绿，所以直行仍是 go')
        # 真正的危险是"漏掉中间那个绿框"：剩下左红 + 右绿
        dropped_middle = traffic_rules.judge(
            [[100, 100, 160, 200, .9], [500, 100, 560, 200, .9]], img)
        self.assertEqual(dropped_middle.layout, 'left_straight')
        self.assertEqual(dropped_middle.states['straight'], 'go',
                         '漏掉中间框时 2 框规则会把右绿当直行绿 —— 侥幸正确')
        # 而漏掉右边、且中间是红，就会让直行错成红：这才是 LayoutHold 要挡的
        img2 = yrg_with_boxes([
            (100, 100, 160, 200, 254),
            (300, 100, 360, 200, 254),
            (500, 100, 560, 200, 255),
        ])
        two_of_img2 = traffic_rules.judge(
            [[100, 100, 160, 200, .9], [300, 100, 360, 200, .9]], img2)
        self.assertEqual(two_of_img2.states['straight'], 'stop')


class FilterTests(unittest.TestCase):
    """V27 判据：长边三等分颜色集中度。真灯只亮一格，数字铺满整框。"""

    def test_digit_box_rejected(self):
        # 数字"8"：笔画铺满整个框，纵向三等分每格都有 -> 最大一格占比 ≈0.33
        img = np.full((720, 1280), 120, np.uint8)
        x1, y1, x2, y2, t = 100, 100, 140, 164, 8
        img[y1:y2, x1:x2] = 254
        img[y1 + t:y2 - t, x1 + t:x2 - t] = 120
        img[(y1 + y2) // 2 - 4:(y1 + y2) // 2 + 4, x1 + t:x2 - t] = 254
        f = traffic_boxes.box_features(img, [x1, y1, x2, y2, .9])
        self.assertLess(f['share'], traffic_boxes.SHARE_MIN,
                        f"数字笔画应摊在多格: share={f['share']}")
        kept, rejected = traffic_boxes.filter_boxes([[x1, y1, x2, y2, .9]], img)
        self.assertEqual(len(kept), 0, f'数字框应被拒: {f}')
        self.assertEqual(rejected[0][1], 'spread')

    def test_unlit_box_rejected(self):
        # 框里一个 254/255 都没有：没亮的灯不占方向位
        img = yrg_with_boxes([(100, 430, 130, 470, 90)])
        kept, rejected = traffic_boxes.filter_boxes(
            [[100, 430, 130, 470, .9]], img)
        self.assertEqual(len(kept), 0)
        self.assertEqual(rejected[0][1], 'unlit')

    def test_real_lit_box_kept(self):
        # 真灯：灯珠只占框中一格 -> share 接近 1
        img = np.full((720, 1280), 120, np.uint8)
        img[170:200, 305:325] = 254          # 框 [300,150,330,220] 的中间一格
        f = traffic_boxes.box_features(img, [300, 150, 330, 220, .9])
        self.assertGreaterEqual(f['share'], traffic_boxes.SHARE_MIN)
        kept, rejected = traffic_boxes.filter_boxes(
            [[300, 150, 330, 220, .9]], img)
        self.assertEqual(len(kept), 1, f'真灯不该被误杀: {rejected}, feats={f}')

    def test_horizontal_box_uses_columns(self):
        # 横长框按列切：颜色在右侧一列 = 集中
        img = np.full((720, 1280), 120, np.uint8)
        img[160:200, 400:430] = 255           # 框 [300,150,450,210] 的第 3 列
        f = traffic_boxes.box_features(img, [300, 150, 450, 210, .9])
        self.assertGreaterEqual(f['share'], traffic_boxes.SHARE_MIN,
                                f'横长框应按列切: share={f["share"]}')

    def test_window_slides_to_the_light_blob(self):
        """灯珠偏在框的一侧时，窗口必须能滑过去（死钉三等分边界会切成两格）。

        板端 seq1 的右灯就是这样：share 固定算只有 0.62，滑动后是 1.00。
        用户 2026-10-31 要求「70% 分布在一个小框内」，固定切法在 0.70
        门槛下会杀掉 10/38 盏真灯，滑动只杀 1 盏，且那盏是框被拉宽的。
        """
        img = np.full((720, 1280), 120, np.uint8)
        # 框 [300,150,450,210] 宽 150，灯珠只占最左 40px（跨在固定 1/3 边界上）
        img[160:200, 302:342] = 254
        f = traffic_boxes.box_features(img, [300, 150, 450, 210, .9])
        self.assertGreaterEqual(f['share'], 0.95,
                                f'窗口应滑到灯珠上: share={f["share"]}')
        kept, _ = traffic_boxes.filter_boxes([[300, 150, 450, 210, .9]], img)
        self.assertEqual(len(kept), 1, '偏在一侧的真灯不能被 0.70 门槛杀掉')

    def test_uniform_stripe_does_not_reach_70(self):
        """颜色铺满整条长边（数字笔画/反光条）时，任何 1/3 窗口最多装 1/3。"""
        img = np.full((720, 1280), 120, np.uint8)
        img[160:200, 300:450] = 254          # 横贯整框
        f = traffic_boxes.box_features(img, [300, 150, 450, 210, .9])
        self.assertLess(f['share'], traffic_boxes.SHARE_MIN,
                        f'整条铺满不该过 0.70: share={f["share"]}')
        kept, rejected = traffic_boxes.filter_boxes([[300, 150, 450, 210, .9]], img)
        self.assertEqual(kept, [])
        self.assertEqual(rejected[0][1], 'spread')

    def test_red_and_green_counted_separately(self):
        """横排灯箱左转红 + 直行绿同时亮：合并算会摊成两格误杀，分开算才稳。"""
        img = np.full((720, 1280), 120, np.uint8)
        img[160:200, 300:340] = 254            # 左 1/3 红
        img[160:200, 400:450] = 255            # 右 1/3 绿
        f = traffic_boxes.box_features(img, [300, 150, 450, 210, .9])
        self.assertGreaterEqual(f['share'], traffic_boxes.SHARE_MIN)


class RowHeightTests(unittest.TestCase):
    """第二步：同一行里明显更矮的框是陪衬（行人指示灯 / 计数牌）。"""

    def _lamp(self, img, box, colour=254):
        x1, y1, x2, y2 = box
        cy = (y1 + y2) // 2
        img[cy - 8:cy + 8, x1 + 8:(x1 + x2) // 2] = colour

    def test_small_box_in_row_rejected(self):
        img = np.full((720, 1280), 120, np.uint8)
        big = [(200, 120, 260, 230), (700, 120, 760, 230)]
        small = (430, 150, 462, 214)          # 与两个大框同一行，但矮一半
        for b in big:
            self._lamp(img, b)
        self._lamp(img, small)
        boxes = [list(b) + [.9] for b in big + [small]]
        kept, rejected = traffic_boxes.filter_boxes(boxes, img)
        self.assertEqual(len(kept), 2, f'矮框应由行内高度统计剔除: {rejected}')
        self.assertEqual(rejected[0][1], 'row_small')

    def test_row_needs_three_boxes(self):
        """行内只有 2 个框时没有参照，不得互相判矮。"""
        img = np.full((720, 1280), 120, np.uint8)
        big = (200, 120, 260, 230)
        small = (430, 150, 462, 214)
        self._lamp(img, big)
        self._lamp(img, small)
        kept, rejected = traffic_boxes.filter_boxes(
            [list(big) + [.9], list(small) + [.9]], img)
        self.assertEqual(len(kept), 2, f'不足三个框不应做高度判断: {rejected}')

    def test_far_off_row_rejected(self):
        img = np.full((720, 1280), 120, np.uint8)
        row = [(200, 120, 260, 230), (450, 120, 510, 230), (700, 120, 760, 230)]
        low = (300, 560, 360, 670)
        for b in row + [low]:
            self._lamp(img, b)
        boxes = [list(b) + [.9] for b in row + [low]]
        kept, rejected = traffic_boxes.filter_boxes(boxes, img)
        self.assertEqual(len(kept), 3, f'偏离行心的框应剔除: {rejected}')
        self.assertEqual(rejected[0][1], 'row_far')


class BinaryFrameTrafficTests(unittest.TestCase):
    """二值帧（只有 0/253）上数不到 254/255，必须走 traffic_hold 不判色。"""

    def test_binary_frame_has_no_colours(self):
        img = np.zeros((720, 1280), np.uint8)
        img[150:210, 300:360] = 253
        verdict = traffic_rules.judge([[300, 150, 360, 210, .9]], img)
        self.assertEqual(verdict.colours, ['off'],
                         '二值帧里没有 254/255，只能是 off')


class CommunicationTests(unittest.TestCase):
    """2026-10-31：二值请求通道整个删除，通信层只剩一条 MODE 1。"""

    def test_single_command_encoding(self):
        from protocol import Display, IMAGE_MODE, encode_command
        # 任何显示状态都必须落在 MODE 1（bit10~11 = 01）
        for d in (Display(), Display(left=RED, straight=GREEN, right=RED)):
            self.assertEqual((encode_command(d, IMAGE_MODE) >> 10) & 3, 1)

    def test_request_frame_takes_no_mode(self):
        import inspect
        from udp_pair import UDPLink
        params = inspect.signature(UDPLink.request_frame).parameters
        self.assertNotIn('image_mode', params)

    def test_no_binary_machinery_left(self):
        for name in ('main.py', 'udp_pair.py', 'strategy_engine.py'):
            src = (ROOT / name).read_text(encoding='utf-8')
            for gone in ('choose_mode(', 'prearm(', 'max_binary_failures',
                         'binary_mode_failures', 'capture_sequential_pair'):
                self.assertNotIn(gone, src, f'{name} 不该再有 {gone}')


if __name__ == '__main__':
    unittest.main()
