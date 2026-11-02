"""交警手势识别与状态机的单元测试。

重点不是"能不能判对"，而是四条**安全属性**：
  1. **模型说 other / 置信度不够 -> 绝不进入交警模式**（没有可用的手势证据）
  2. 弱证据绝不能进入交警模式（开放世界误触发 = FPGA 直接播报语音）
  3. 进入交警模式后只发一个箭头（FPGA 靠单箭头识别交警模式）
  4. 证据消失要能退出，不能粘住

V28 判据（见 cop_traffic_signature 模块头）：
  * 模型 cls_v2yrg3 输出 3 个 logits：0=类1(路口->左转) 1=类2(路段->直行) 2=other
  * 置信度 < 类1门槛 0.95 / 类2门槛 0.90 一律按 miss 处理
  * 输入是板上单通道 YRG -> yrg_single_to_bgr() -> 96x96 拉伸，(不重做红绿判定)
"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cop_traffic_signature as cop
from protocol import Display, GREEN, RED

# 很"确定"的 logits：one-hot 拉开 20 个 logit，softmax 之后约 1.0
SURE = 20.0


def logits(cls, conf_strength=SURE):
    """造一个主要落在 `cls` 的 logits。cls=2 表示 other。"""
    v = np.full(3, -conf_strength / 2, np.float32)
    v[cls] = conf_strength / 2
    return v


def weak(cls):
    """三类几乎打平 -> softmax ~0.34，远低于 CONF_MIN。"""
    return np.zeros(3, np.float32) if cls is None else np.array(
        [0.0, 0.0, 0.0], np.float32) + np.eye(3, dtype=np.float32)[cls] * 0.05


class TestYrgRebuild(unittest.TestCase):
    """输入契约：254->红、255->绿、其余灰度复制三通道。"""

    def test_red_and_green_become_pure(self):
        yrg = np.array([[254, 255, 100]], np.uint8)
        bgr = cop.yrg_single_to_bgr(yrg)
        self.assertEqual(bgr.shape, (1, 3, 3))
        self.assertEqual(tuple(bgr[0, 0]), (0, 0, 255))     # 254 -> BGR 纯红
        self.assertEqual(tuple(bgr[0, 1]), (0, 255, 0))     # 255 -> BGR 纯绿
        self.assertEqual(tuple(bgr[0, 2]), (100, 100, 100)) # 灰度三通道相同

    def test_tensor_is_nhwc_96_rgb(self):
        yrg = np.full((720, 1280), 120, np.uint8)
        x = cop.to_tensor(yrg)
        self.assertEqual(x.shape, (1, 96, 96, 3))
        self.assertEqual(x.dtype, np.uint8)

    def test_tensor_stretches_not_letterbox(self):
        """整幅拉伸到 96x96：左半边绿、右半边灰，缩完两半应各占一半。

        如果误做成 letterbox，两侧会出现黑边，绿块占比会变小。
        """
        yrg = np.full((720, 1280), 120, np.uint8)
        yrg[:, :640] = 255
        x = cop.to_tensor(yrg)[0]
        greenish = (x[:, :, 1] > 200) & (x[:, :, 2] < 60)
        self.assertAlmostEqual(greenish.mean(), 0.5, delta=0.06)


class TestKindFromLogits(unittest.TestCase):
    def test_class_index_mapping(self):
        """0->'1'(左转) / 1->'2'(直行) / 2->None。接错一个序号演示就反了。"""
        self.assertEqual(cop.kind_from_logits(logits(0))[0], '1')
        self.assertEqual(cop.kind_from_logits(logits(1))[0], '2')
        self.assertIsNone(cop.kind_from_logits(logits(2))[0])

    def test_confidence_is_reported(self):
        kind, idx, conf = cop.kind_from_logits(logits(0))
        self.assertEqual(idx, 0)
        self.assertGreater(conf, 0.99)

    def test_flat_logits_are_low_confidence(self):
        _, _, conf = cop.kind_from_logits(weak(None))
        self.assertAlmostEqual(conf, 1 / 3, places=3)

    def test_wrong_shape_raises(self):
        with self.assertRaises(ValueError):
            cop.kind_from_logits(np.zeros(5, np.float32))

    def test_non_finite_logits_are_not_high_confidence(self):
        """NaN / ±inf 不能变成"高置信度"。

        `NaN < conf_min` 恒为 False —— 直接写 `conf < min` 会让非有限值
        穿过门槛，状态机确认成左转绿箭头。这是安全门的第二道锁。
        """
        for v in (np.array([np.nan, 1, 2], np.float32),
                  np.array([1, np.nan, np.nan], np.float32),
                  np.array([np.inf, 0, 0], np.float32),
                  np.array([-np.inf, -np.inf, -np.inf], np.float32)):
            _, _, conf = cop.kind_from_logits(v)
            self.assertTrue(np.isfinite(conf), f'置信度必须有限，得到 {conf}')
            self.assertLess(conf, cop.CONF_MIN)


class TestSafetyGate(unittest.TestCase):
    """没有该交警 -> 任何数量的轮次都进不了交警模式。"""

    def test_other_never_locks(self):
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        for _ in range(50):
            self.assertIsNone(s.observe(logits(2)))

    def test_non_finite_logits_never_lock(self):
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        for v in (np.array([np.nan, 1, 2], np.float32),
                  np.array([np.inf, 0, 0], np.float32),
                  np.array([-np.inf, -np.inf, -np.inf], np.float32)):
            for _ in range(20):
                self.assertIsNone(s.observe(v), f'非有限 logits {v} 竟然进了交警模式')
        self.assertIsNone(s.locked)

    def test_low_confidence_never_locks(self):
        """三类打平时，即便 argmax 落在左转类也不能进入交警模式。"""
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        for _ in range(50):
            self.assertIsNone(s.observe(weak(0)))

    def test_stale_votes_cannot_lock_the_other_kind(self):
        """V28.2：旧票不能把连续的新证据锁成旧类别（方向反）。

        旧实现只看"多数票 != locked"，窗口里的旧票会让 streak 在新证据上累加：
        先零散 5 票直行（中间穿插 miss，始终未达确认轮数），随后连续 3 轮判左转，
        旧实现会在第 3 轮把 locked 锁成类2（直行），而当前证据是类1（左转）。
        """
        s = cop.TrafficSignature(window=12, lock_rounds=4, release_rounds=3)
        for x in (1, 1, None, 1, 1, None, 1):
            s.observe(None if x is None else logits(x))
        self.assertIsNone(s.locked, '零散证据不应进入交警模式')
        for i in range(3):
            self.assertIsNone(
                s.observe(logits(0)),
                f'第 {i + 1} 轮类1：旧票把新证据锁成了旧类别')
        # 继续喂类1：最终只能锁类1，绝不能锁类2
        for _ in range(12):
            s.observe(logits(0))
        self.assertEqual(s.locked, '1')

    def test_wrong_shape_is_miss_not_raise(self):
        """V28.2：模型输出形状错误只算 miss，不能抛到 run_live 触发重启。"""
        s = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3)
        self.assertIsNone(s.observe(np.zeros(5, np.float32)))
        self.assertIsNone(s.locked)
        self.assertEqual(s.snapshot()['error'], 'ValueError')

    def test_observe_none_resets_diagnostics(self):
        """模型没跑时不能再报上一轮的 cls_index/cls_conf。"""
        s = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3)
        s.observe(logits(0))
        snap = s.snapshot()
        self.assertEqual(snap['cls_index'], 0)
        s.observe(None)
        snap = s.snapshot()
        self.assertIsNone(snap['cls_kind'])
        self.assertEqual(snap['cls_index'], -1)
        self.assertEqual(snap['cls_conf'], 0.0)

    def test_conf_threshold_is_a_single_knob(self):
        """同一组 logits，门槛设在它两侧时结论必须相反。"""
        v = logits(0, conf_strength=3.0)          # softmax ≈ 0.905
        _, _, conf = cop.kind_from_logits(v)
        self.assertGreater(conf, 0.88)
        self.assertLess(conf, 0.93)

        below = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3,
                                     conf_min=0.80)
        self.assertEqual(below.observe(v), '1')

        above = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3,
                                     conf_min=0.95)
        self.assertIsNone(above.observe(v), '置信度 0.905 过不了 0.95 的门槛')

    def test_default_threshold_is_the_high_wall(self):
        """V28.4（用户直接拍板）：类1 0.95 / 类2 0.90（V28.6 未动门槛）。"""
        self.assertEqual(cop.CONF_MIN, 0.95)
        self.assertEqual(cop.CONF_MIN_BY_KIND, {'1': 0.95, '2': 0.90})
        s = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3)
        self.assertEqual(s.conf_min, 0.95)
        self.assertEqual(s.conf_by_kind, {'1': 0.95, '2': 0.90})
        # 用**算出来的**置信度当尺子，门槛架在它两侧时结论必须相反。
        # 不写死"某个 logit 恰好等于 0.99"——那个值会随 numpy 版本漂。
        v = logits(0, conf_strength=5.7)           # softmax ≈ 0.990
        _, _, c = cop.kind_from_logits(v)
        self.assertGreater(c, cop.CONF_MIN)
        self.assertEqual(s.observe(v), '1')        # 默认门槛放行

        stricter = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3,
                                        conf_min=min(1.0, c + 1e-6))
        self.assertIsNone(stricter.observe(v), '门槛只要高过实测置信度就必须拦下')
        self.assertGreater(0.9896, cop.CONF_MIN)

    def test_class2_uses_090_and_two_rounds(self):
        """V28.6：类2 0.90/2 轮；类1 0.95/2 轮（门槛仍是类1更高）。"""
        s = cop.TrafficSignature(window=12, release_rounds=3)
        v2 = logits(1, conf_strength=3.15)         # softmax ≈ 0.921
        _, _, c = cop.kind_from_logits(v2)
        self.assertGreaterEqual(c, 0.90)
        self.assertLess(c, 0.95)
        self.assertIsNone(s.observe(v2))
        self.assertEqual(s.observe(v2), '2')

        s1 = cop.TrafficSignature(window=12, release_rounds=3)
        v1 = logits(0, conf_strength=3.15)         # 同类置信度，但类1门槛 0.95
        for _ in range(8):
            self.assertIsNone(s1.observe(v1), '左转类 0.92 低于 0.95 不能进入')

    def test_class2_below_090_never_locks(self):
        """低于 0.90 的类2 帧不能把直行抬进去。"""
        s = cop.TrafficSignature(window=12, release_rounds=3)
        v = logits(1, conf_strength=2.7)           # softmax ≈ 0.881
        for _ in range(30):
            self.assertIsNone(s.observe(v))

    def test_none_logits_are_miss(self):
        """模型没跑 / 没装 -> miss，不进入交警模式。"""
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        for _ in range(20):
            self.assertIsNone(s.observe(None))

    def test_invalid_conf_min_raises(self):
        with self.assertRaises(ValueError):
            cop.TrafficSignature(conf_min=0.0)
        with self.assertRaises(ValueError):
            cop.TrafficSignature(conf_min=1.5)


class TestStateMachine(unittest.TestCase):
    def test_default_lock_rounds_matches_module_constant(self):
        """V28.6：两类都是 2 轮；单值构造仍可覆盖两类（测试兼容）。"""
        self.assertEqual(cop.LOCK_ROUNDS, 2)
        self.assertEqual(cop.LOCK_ROUNDS_BY_KIND, {'1': 2, '2': 2})
        sig = cop.TrafficSignature()
        self.assertEqual(sig.lock_rounds, 2)
        self.assertEqual(sig.lock_by_kind, {'1': 2, '2': 2})
        single = cop.TrafficSignature(lock_rounds=5)
        self.assertEqual(single.lock_by_kind, {'1': 5, '2': 5})

    def test_default_release_is_one_round(self):
        """V28.7：一帧不是交警就退出（用户要求，3 → 1）。"""
        self.assertEqual(cop.RELEASE_ROUNDS, 1)
        s = cop.TrafficSignature(window=12, lock_rounds=2)
        self.assertEqual(s.release_rounds, 1)
        s.observe(logits(1))
        self.assertEqual(s.observe(logits(1)), '2')
        self.assertIsNone(s.observe(logits(2)), '一帧 other 必须立刻退出')
        self.assertIsNone(s.locked)
        self.assertEqual(s.snapshot()['votes'], 0, '退出后票窗必须清空')

    def test_locks_only_after_streak(self):
        s = cop.TrafficSignature(window=12, lock_rounds=3, release_rounds=3)
        self.assertIsNone(s.observe(logits(0)))
        self.assertIsNone(s.observe(logits(0)))
        self.assertEqual(s.observe(logits(0)), '1')

    def test_release_after_other(self):
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        s.observe(logits(1))
        self.assertEqual(s.locked, '2')
        self.assertEqual(s.observe(logits(2)), '2')
        self.assertEqual(s.observe(logits(2)), '2')
        self.assertIsNone(s.observe(logits(2)))
        self.assertIsNone(s.locked)

    def test_release_clears_votes(self):
        """退出必须把多数票窗口一起清掉。

        V25 只清 locked 不清票，于是刚退出后又来一帧证据就以 12 票的"多数"
        重新进入 —— 屏幕上表现为退出交警模式后立刻跳回去。
        """
        s = cop.TrafficSignature(window=12, lock_rounds=3, release_rounds=3)
        for _ in range(6):
            s.observe(logits(1))
        self.assertEqual(s.locked, '2')
        for _ in range(3):
            s.observe(logits(2))
        self.assertIsNone(s.locked)
        self.assertEqual(s.snapshot()['votes'], 0, '退出后窗口必须是空的')
        # 一帧证据不足以重新进入（需要满足确认轮数）
        self.assertIsNone(s.observe(logits(1)))

    def test_evidence_resets_miss_counter(self):
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        s.observe(logits(0))
        s.observe(logits(2))
        s.observe(logits(2))
        s.observe(logits(0))            # 交警回来，计时清零
        self.assertEqual(s.observe(logits(2)), '1')
        self.assertEqual(s.observe(logits(2)), '1')
        self.assertIsNone(s.observe(logits(2)))

    def test_switch_needs_new_streak(self):
        """切换必须等多数票真的翻过去，一轮反证不够。"""
        s = cop.TrafficSignature(window=4, lock_rounds=2, release_rounds=3)
        s.observe(logits(0))
        s.observe(logits(0))
        self.assertEqual(s.locked, '1')
        self.assertEqual(s.observe(logits(1)), '1')   # [A,A,B]
        self.assertEqual(s.observe(logits(1)), '1')   # 打平
        self.assertEqual(s.observe(logits(1)), '1')   # streak=1
        self.assertEqual(s.observe(logits(1)), '2')   # streak=2

    def test_majority_guard(self):
        """窗口未满时打平不猜。"""
        s = cop.TrafficSignature(window=4, lock_rounds=1, release_rounds=3)
        s.observe(logits(0))
        self.assertEqual(s.locked, '1')
        s.observe(logits(1))
        self.assertIsNone(s._majority())

    def test_snapshot_reports_model_output(self):
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3)
        s.observe(logits(1))
        snap = s.snapshot()
        self.assertEqual(snap['kind'], '2')
        self.assertEqual(snap['cls_index'], 1)
        self.assertEqual(snap['cls_kind'], '2')
        self.assertGreater(snap['cls_conf'], 0.99)

    def test_snapshot_keeps_raw_kind_below_threshold(self):
        """低于门槛时诊断仍要看到模型原话，否则板上没法判断是不是门槛太高。"""
        s = cop.TrafficSignature(window=12, lock_rounds=1, release_rounds=3,
                                 conf_min=0.99)
        s.observe(weak(0))
        snap = s.snapshot()
        self.assertIsNone(snap['kind'])
        self.assertEqual(snap['cls_kind'], '1', '诊断字段要保留模型原始归类')
        self.assertLess(snap['cls_conf'], 0.99)


class TestDisplay(unittest.TestCase):
    def test_left_is_the_only_green_arrow(self):
        """左转语义 -> 只发左转绿箭头。"""
        d = cop.cop_display('1')
        self.assertEqual(d, Display(left=GREEN))
        self.assertIsNone(d.straight)
        self.assertIsNone(d.right)

    def test_straight_is_the_only_red_arrow(self):
        """直行语义 -> 只发直行红箭头。"""
        d = cop.cop_display('2')
        self.assertEqual(d, Display(straight=RED))
        self.assertIsNone(d.left)
        self.assertIsNone(d.right)

    def test_cop_arrow_is_still_exactly_one(self):
        """换色不能改成多发/少发箭头：FPGA 靠"只有一个箭头"识别交警模式。"""
        for kind in ('1', '2'):
            d = cop.cop_display(kind)
            self.assertEqual(
                sum(c is not None for c in (d.left, d.straight, d.right)), 1, kind)

    def test_unknown_kind_is_none(self):
        self.assertIsNone(cop.cop_display('C'))

    def test_cop_display_can_keep_edge_warning(self):
        """V28.2：交警模式可以保留斑马线/行人报警位，但绝不增加箭头。"""
        d = cop.cop_display('1', warning=True)
        self.assertEqual(d, Display(left=GREEN, edge_warning=True))
        self.assertIsNone(d.straight)
        self.assertIsNone(d.right)

    def test_colour_table_is_the_single_switch(self):
        """V28.5：颜色表独立于方向表，改演示配色只动这张表。"""
        self.assertEqual(cop.KIND_TO_COLOUR, {'1': GREEN, '2': RED})
        self.assertEqual(set(cop.KIND_TO_COLOUR), set(cop.KIND_TO_DIRECTION))

    def test_mapping_table_is_the_single_switch(self):
        """用户 2026-10-31 确认的对应关系，改演示只动这张表。"""
        self.assertEqual(set(cop.KIND_TO_DIRECTION), {'1', '2'})
        self.assertEqual(cop.KIND_TO_DIRECTION['1'], 'left')
        self.assertEqual(cop.KIND_TO_DIRECTION['2'], 'straight')

    def test_class_index_table_matches_direction_table(self):
        """CLASS_TO_KIND 与 KIND_TO_DIRECTION 必须互相闭合，不能有孤儿。"""
        for idx, kind in cop.CLASS_TO_KIND.items():
            if kind is not None:
                self.assertIn(kind, cop.KIND_TO_DIRECTION)
        self.assertIsNone(cop.CLASS_TO_KIND[2], 'other 必须是 None')


class TestRestoredCopArrow(unittest.TestCase):
    """重启后不能把上一次交警模式的单箭头当成普通显示沿用。

    V28.5 起直行手势是**红**箭头，所以"单绿"不再是唯一的交警特征；
    但"单红"只有**直行位置**才算交警——车道分支合法地只亮一侧实线红。
    """

    def setUp(self):
        import main
        self.drop = main._drop_restored_cop_arrow

    def test_single_green_is_dropped(self):
        self.assertEqual(self.drop(Display(left=GREEN)), Display())

    def test_single_red_straight_is_dropped(self):
        self.assertEqual(self.drop(Display(straight=RED)), Display())

    def test_single_red_side_is_kept(self):
        self.assertEqual(self.drop(Display(left=RED)), Display(left=RED))
        self.assertEqual(self.drop(Display(right=RED)), Display(right=RED))

    def test_three_arrows_are_kept(self):
        disp = Display(left=RED, straight=RED, right=GREEN)
        self.assertEqual(self.drop(disp), disp)


if __name__ == '__main__':
    unittest.main()
