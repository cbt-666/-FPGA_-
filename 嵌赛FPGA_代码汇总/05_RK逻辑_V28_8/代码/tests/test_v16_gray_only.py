"""全灰度（V24）：整轮只发一条 MODE 1，二值图从头到尾不存在。

2026-10-31 用户要求「把所有和二值化有关的东西都删掉、车道换成灰度模型、
不再请求二值化、RK 端坚决不要做二值化」。所以这一版不再有"全灰度版 vs
原逻辑版"之分 —— 只剩全灰度，本文件守的就是这条独木桥。

省掉的是：
  * 一趟二值采集（约 130 ms/轮）
  * 每轮一次模式翻转 —— FPGA 会 latch 最后一次模式指令，反复翻转会让
    上一模式的帧和当前请求交织，正是 rk3568_v15_10.log 第 72 轮超时的成因。
"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import main
from protocol import GRAY, BINARY, Display, encode_packet
from udp_pair import ImageFrame


class FakeLink:
    def __init__(self):
        self.requested = []
        self.pair_timing_ms = {}

    def request_frame(self, display, timeout=None, on_request=None):
        self.requested.append(GRAY)
        if on_request is not None:
            on_request(encode_packet(display, GRAY), 0)
        self.last_report = {'gray_unique_values': 201}
        self.last_drain_ms = 0.0
        return ImageFrame(1, np.zeros((720, 1280), np.uint8))


def args(**kw):
    base = dict(pair_timeout=2, max_retries=2, retry_delay=0)
    base.update(kw)
    return SimpleNamespace(**base)


class SingleModeCaptureTests(unittest.TestCase):
    def test_capture_sends_exactly_one_command(self):
        link = FakeLink()
        frame = main.capture_one_frame(link, Display(), args(), 1)
        self.assertEqual(frame.frame_id, 1)
        self.assertEqual(link.requested, [GRAY])
        self.assertEqual(link.pair_timing_ms['pair_ms'],
                         link.pair_timing_ms['gray_ms'])

    def test_repeated_rounds_keep_one_mode(self):
        link = FakeLink()
        for k in range(5):
            main.capture_one_frame(link, Display(), args(), k + 1)
        self.assertEqual(link.requested, [GRAY] * 5,
                         'no mode flip at all: the FPGA latch is never overridden')

    def test_capture_api_takes_no_mode_argument(self):
        """签名里不该再有 image_mode —— 没有第二种模式可传。"""
        import inspect
        params = inspect.signature(main.capture_one_frame).parameters
        self.assertNotIn('image_mode', params)


def run_main(argv):
    """Run main() with a given argv; validation happens before models load."""
    old = sys.argv
    sys.argv = ['main.py'] + argv
    try:
        main.main()
    except SystemExit as exc:
        return exc.code
    finally:
        sys.argv = old
    return None


class CommandLineTests(unittest.TestCase):
    def test_binary_switches_are_gone(self):
        for flag in ('--gray-only', '--lane-variant', '--request-binary',
                     '--always-request-binary', '--max-binary-failures',
                     '--binary-npy', '--capture-pair', '--lane-benchmark'):
            with self.assertRaises(SystemExit) as ctx:
                main.parser().parse_args([flag])
            self.assertEqual(ctx.exception.code, 2, f'{flag} 应已删除')

    def test_defaults_are_the_grey_lane_build(self):
        a = main.parser().parse_args([])
        self.assertIn('G640', a.lane_model.name)
        self.assertNotIn('A768', a.lane_model.name)
        self.assertIn('housing', a.traffic_model.name)
        self.assertFalse(hasattr(a, 'lane_variant'))
        self.assertFalse(hasattr(a, 'gray_only'))


class SupervisorCommandTests(unittest.TestCase):
    """守护器只把 main.py 的参数透传，前面不再有 --gray-only 这类启动器开关。"""

    def setUp(self):
        import types
        if 'fcntl' not in sys.modules:
            stub = types.ModuleType('fcntl')
            stub.flock = lambda *a, **k: None
            stub.LOCK_UN = 0
            sys.modules['fcntl'] = stub
        import live_watchdog
        self.lw = live_watchdog
        self.seen = []
        self.lw.supervise = lambda cmd, **kw: (self.seen.append(cmd), 0)[1]

    def test_plain_build(self):
        self.lw.main(['--', '--max-cycles', '2'])
        self.assertEqual(self.seen[-1],
                         ['./start_v12.sh', '--', '--max-cycles', '2'])

    def test_skip_preflight_still_works(self):
        self.lw.main(['--', '--skip-preflight', '--', '--max-cycles', '2'])
        self.assertEqual(self.seen[-1],
                         ['./start_v12.sh', '--skip-preflight', '--',
                          '--max-cycles', '2'])


if __name__ == '__main__':
    unittest.main()
