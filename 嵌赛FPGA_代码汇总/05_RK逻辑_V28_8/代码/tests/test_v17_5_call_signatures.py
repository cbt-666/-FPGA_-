"""V17_4 之后的护栏：调用点传的关键字参数必须在被调函数签名里。

2026-10-29 上板：`run_live` 给 `main_entry` 传了 `last_good_traffic_hold=`，
但 `main_entry` 的签名漏了它 → **整场 458 轮全部 TypeError**，屏幕从头黑到尾。
300 项单元测试一条都没抓到，因为它们都绕开了 `run_live` 这条真实调用链。

所以这里的检查不看逻辑，只看"名字对不对得上"，覆盖全部工程文件。
"""
import ast
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _functions():
    sig = {}
    for p in sorted(ROOT.glob('*.py')):
        try:
            tree = ast.parse(p.read_text(encoding='utf-8'))
        except Exception:
            continue
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                a = node.args
                names = {x.arg for x in a.posonlyargs + a.args + a.kwonlyargs}
                if a.vararg or a.kwarg:
                    names.add('*')
                sig.setdefault(node.name, set()).update(names)
    return sig


def _calls():
    for p in sorted(ROOT.glob('*.py')):
        try:
            tree = ast.parse(p.read_text(encoding='utf-8'))
        except Exception:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                kws = [k.arg for k in node.keywords if k.arg]
                if kws:
                    yield p.name, node.lineno, node.func.id, kws


class CallSignatureTests(unittest.TestCase):
    def test_every_keyword_exists_in_the_callee(self):
        sig = _functions()
        problems = []
        for fname, line, name, kws in _calls():
            params = sig.get(name)
            if params is None or '*' in params:
                continue
            extra = [k for k in kws if k not in params]
            if extra:
                problems.append(f'{fname}:{line} {name}() 不认识 {", ".join(extra)}')
        self.assertEqual(problems, [],
                         '调用点传了被调函数不认识的参数（上板会每轮 TypeError）：\n  '
                         + '\n  '.join(problems))

    def test_main_entry_accepts_everything_run_live_passes(self):
        """把上板踩到的那个具体组合钉住。"""
        import inspect
        import main
        params = set(inspect.signature(main.main_entry).parameters)
        for name in ('last_good', 'last_good_hold', 'last_good_traffic_hold',
                     'scene_retries', 'frame_is_thresholded', 'lane_band_top',
                     'lane_band_bot'):
            self.assertIn(name, params)
        # 2026-10-31：二值通道已删除，这些参数不该再出现。
        for gone in ('binary', 'gray_is_binary', 'lane_variant'):
            self.assertNotIn(gone, params)

    def test_decide_pair_accepts_everything_main_entry_passes(self):
        import inspect
        import strategy_engine
        params = set(inspect.signature(strategy_engine.decide_pair).parameters)
        for name in ('last_good', 'last_good_hold', 'last_good_traffic_hold',
                     'scene_retries', 'lane_band_top', 'lane_band_bot'):
            self.assertIn(name, params)


class RunLiveSmokeTests(unittest.TestCase):
    """真的走一遍 run_live 的一轮，只把网络和模型换成假的。

    上一版就是这么翻车的：单元测试全绿，真实调用链每轮 TypeError。
    """

    def test_one_round_through_run_live(self):
        from unittest.mock import patch
        import numpy as np
        import main as m

        from protocol import Display

        def _empty_display():
            return Display()

        gray = np.zeros((720, 1280), np.uint8)
        gray[:, :] = 90

        class FakeFrame:
            def __init__(self):
                self.pixels = gray
                self.frame_id = 1

        class _FakeSock:
            def getsockopt(self, *a):
                return 0

            def close(self):
                pass

        class FakeLink:
            def __init__(self, *a, **k):
                self.pair_timing_ms = {'gray_ms': 10.0, 'binary_ms': 0.0,
                                       'pair_ms': 10.0}
                self.last_report = {'gray_unique_values': 194}
                self.last_gray_is_binary = False
                self.binary_requested = False
                self.sock = _FakeSock()

            def configure_receiver(self, *a, **k):
                pass

            def close(self):
                pass

        class FakeDet:
            def scene(self, g):
                return [{'class_name': 'lane', 'class_id': 2, 'score': .9,
                         'xyxy': [0, 400, 1280, 700]}]

            def navigation(self, g):
                return []

            def housing(self, g):
                return []

            def lane_mask(self, g):
                return np.zeros((432, 768), np.uint8)

            last_model_stages = {}

            def close(self):
                pass

        args = m.parser().parse_args(['--max-cycles', '1'])
        with patch.object(m, 'UDPLink', FakeLink), \
             patch.object(m, 'build_models', lambda a: FakeDet()), \
             patch.object(m, 'read_display', lambda: _empty_display()), \
             patch.object(m, 'save_display', lambda d: None), \
             patch.object(m, 'capture_one_frame',
                          lambda *a, **k: FakeFrame()), \
             patch.object(m, 'print', create=True):
            m.run_live(args)       # 上一版在这里每轮抛 TypeError


class InferFailureCircuitBreakerTests(unittest.TestCase):
    """2026-10-29：458 轮 TypeError 全被 run_live 吞成日志，屏幕上什么都看不出来。"""

    def test_streak_raises_program_restart(self):
        from unittest.mock import patch
        import numpy as np
        import main as m
        from restart_runtime import ProgramRestartRequested
        from protocol import Display

        gray = np.full((720, 1280), 200, np.uint8)

        class Frame:
            pixels = gray
            frame_id = 1

        class Link:
            def __init__(self, *a, **k):
                self.pair_timing_ms = {'gray_ms': 1.0, 'binary_ms': 0.0,
                                       'pair_ms': 1.0}
                self.last_report = {'gray_unique_values': 194}
                self.last_gray_is_binary = False
                self.binary_requested = False
                self.sock = _Sock2()

            def configure_receiver(self, *a, **k):
                pass

            def close(self):
                pass

        class _Sock2:
            def getsockopt(self, *a):
                return 0

            def close(self):
                pass

        class Boom:
            last_model_stages = {}

            def scene(self, g):
                raise RuntimeError('boom')

            def close(self):
                pass

        args = m.parser().parse_args(['--max-cycles', '5',
                                      '--max-infer-failures', '3'])
        with patch.object(m, 'UDPLink', Link),              patch.object(m, 'build_models', lambda a: Boom()),              patch.object(m, 'read_display', lambda: Display()),              patch.object(m, 'save_display', lambda d: None),              patch.object(m, 'capture_one_frame',
                          lambda *a, **k: Frame()):
            with self.assertRaises(ProgramRestartRequested):
                m.run_live(args)

    def test_single_failure_does_not_restart(self):
        """偶发一帧出错不能触发重启，否则会比故障本身更烦。"""
        import inspect
        import main as m
        src = inspect.getsource(m.run_live)
        self.assertIn('infer_fail_streak = 0', src,
                      '成功一轮就要清零，不能累积历史错误')


if __name__ == '__main__':
    unittest.main()
