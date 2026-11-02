import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class RuntimeLauncherTests(unittest.TestCase):
    def test_shell_syntax(self):
        for name in ('start_rk3568.sh', 'start_v12.sh', 'check_rknn_versions.sh'):
            p = subprocess.run(['bash', '-n', str(ROOT / name)], capture_output=True, text=True)
            self.assertEqual(p.returncode, 0, msg=name + ': ' + p.stderr)

    def test_public_selfcheck_uses_runtime_guard(self):
        s = (ROOT / 'start_rk3568.sh').read_text(encoding='utf-8')
        self.assertIn('exec ./start_v12.sh --model-selfcheck', s)
        self.assertNotIn('exec python3 -u model_selfcheck.py', s)

    def test_public_live_uses_runtime_guard(self):
        s = (ROOT / 'start_rk3568.sh').read_text(encoding='utf-8')
        self.assertIn('exec python3 -u live_watchdog.py -- "$@"', s)
        self.assertNotIn('exec python3 -u main.py --require-all-models', s)

    def test_runtime_version_guard(self):
        s = (ROOT / 'start_v12.sh').read_text(encoding='utf-8')
        self.assertIn("version('rknn-toolkit-lite2')", s)
        self.assertIn("!= \"2.3.2\"", s)
        self.assertIn("librknnrt version: 2.3.2", s)
        self.assertIn('mount --bind "$runtime" "$target"', s)
        self.assertIn('RKNNRT_SO', s)

    def test_failure_policy_unchanged(self):
        s = (ROOT / 'start_v12.sh').read_text(encoding='utf-8')
        self.assertIn('--failure-action restart', s)
        self.assertIn('--lane-backend v2', s)
        # 2026-10-29 V20：实线确认 1 -> 2。用户反馈 V18 取 1 之后
        # "车道跳动明显比之前频率高了"——单帧误判成实线就立刻变红。
        # 注意**不要**改成 3：3 帧在"车道/红绿灯交替"时凑不满
        # （红绿灯分支每轮 reset 计数），箭头会整条消失。
        self.assertIn('--solid-confirm-frames 2', s)
        # 2026-10-31 V26：用户「决策模型可以改成两轮一触发」→ 1 改 2。
        # 2026-11-01 V28.3：板端日志实测 traffic/nav 的 max_hold=2/8 会让
        # 真实调用变成 3/9 轮一次；统一改成 1（真跑 1 + 复用 1 = 两轮一次）。
        # 2026-11-01 V28.5：用户「改成每次调用」→ 1/0/0。
        # 2026-11-01 V28.7：用户「再次改为两轮一调用」→ 2/1/1。
        # 2026-11-01 V28.8：板端报红绿灯判断全错，回滚到 1/0/0（scene 缓存
        # 复用会用旧场景标签把帧路由到错误分支）。**不要再改回 2/1/1。**
        self.assertIn('--scene-every 1', s)
        self.assertNotIn('--scene-every 2', s)
        self.assertIn('--scene-traffic-max-hold 0', s)
        self.assertIn('--scene-nav-max-hold 0', s)
        # V18：导航场景漏检保持 10 -> 3（实测黑屏都是 0，少按 4 轮）
        self.assertIn('--nav-hold-frames 3', s)

    def test_single_grey_build_only(self):
        """2026-10-31：只剩全灰度一版，车道固定 G640，二值开关全部删除。"""
        s = (ROOT / 'start_v12.sh').read_text(encoding='utf-8')
        self.assertIn('--lane-model models/G640_rk3568_fp16.rknn', s)
        for gone in ('A768', '--lane-variant', '--gray-only', 'GRAY_ONLY',
                     '--lane-benchmark', 'RK_GRAY_ONLY'):
            self.assertNotIn(gone, s, f'{gone} 应该已经从启动器删除')
        self.assertFalse((ROOT / 'start_v16_gray.sh').exists(),
                         'A768/G640 双版本启动器已合并为 start_v12.sh')
        self.assertFalse((ROOT / 'start_v16_orig.sh').exists())
        self.assertFalse((ROOT / 'lane_benchmark.py').exists(),
                         '--lane-benchmark 只吃二值帧，应随二值通道一起删除')

    def test_preflight_only_requests_grey(self):
        pf = (ROOT / 'fpga_pair_preflight.py').read_text(encoding='utf-8')
        self.assertIn('from protocol import Display, IMAGE_MODE', pf)
        for gone in ('BINARY', 'is_binary', 'gray_only', 'binary.npy'):
            self.assertNotIn(gone, pf, f'预检里不该再有 {gone}')
        s = (ROOT / 'start_v12.sh').read_text(encoding='utf-8')
        self.assertNotIn('--gray-only', s)

    def test_runtime_not_claimed_bundled(self):
        s = (ROOT / 'SOFTWARE_VERSIONS.md').read_text(encoding='utf-8')
        self.assertIn('不在工程 ZIP 内', s)
        self.assertIn('0.8.2', s)
        self.assertIn('1.5.0', s)

if __name__ == '__main__':
    unittest.main()
