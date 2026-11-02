"""V15.2 recovery tests: no FPGA/NPU/root required (Python 3.8)."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
# live_watchdog imports fcntl (Linux-only). CI/Windows has no fcntl, so stub it
# before importing; the test only exercises supervise()'s restart policy.
HAVE_REAL_FCNTL = 'fcntl' in sys.modules
if not HAVE_REAL_FCNTL:
    import types as _types
    _stub = _types.ModuleType('fcntl')
    _stub.flock = lambda *a, **k: None
    _stub.LOCK_UN = 8
    _stub.LOCK_EX = 2
    _stub.LOCK_NB = 4
    sys.modules['fcntl'] = _stub
from live_watchdog import supervise


class TestLiveWatchdog(unittest.TestCase):
    def make_worker(self, temp, fail_until):
        worker=Path(temp)/'worker.py'
        worker.write_text('''import os, pathlib, sys
p=pathlib.Path(os.environ['WATCHDOG_TEST_COUNT'])
n=int(p.read_text())+1 if p.exists() else 1
p.write_text(str(n))
sys.exit(7 if n<=int(os.environ['WATCHDOG_FAIL_UNTIL']) else 0)
''', encoding='utf-8')
        return worker

    def test_preflight_failure_relaunches_three_times_until_recovery(self):
        with tempfile.TemporaryDirectory() as d:
            count=Path(d)/'count'
            worker=self.make_worker(d,2)
            messages=[]
            with patch.dict(os.environ, {'WATCHDOG_TEST_COUNT':str(count),
                                         'WATCHDOG_FAIL_UNTIL':'2'}):
                rc=supervise([sys.executable, str(worker)], initial_delay=0.005,
                             maximum_delay=0.02,lock_path=Path(d)/'lock',log=messages.append)
            self.assertEqual(rc,0)
            self.assertEqual(count.read_text(),'3')
            self.assertTrue(any('重新启动 Python' in msg for msg in messages))
            self.assertTrue(any('正常结束' in msg for msg in messages))

    def test_persistent_failure_backoff_is_bounded_without_false_success(self):
        with tempfile.TemporaryDirectory() as d:
            count=Path(d)/'count'
            worker=self.make_worker(d,100)
            messages=[]
            with patch.dict(os.environ, {'WATCHDOG_TEST_COUNT':str(count),
                                         'WATCHDOG_FAIL_UNTIL':'100'}):
                rc=supervise([sys.executable, str(worker)], initial_delay=0.001,
                             maximum_delay=0.003,max_restarts=2,
                             lock_path=Path(d)/'lock',log=messages.append)
            self.assertEqual(rc,7)
            self.assertEqual(count.read_text(),'3')
            self.assertTrue(any('最大重启次数' in msg for msg in messages))

    def test_normal_return_does_not_restart(self):
        with tempfile.TemporaryDirectory() as d:
            count=Path(d)/'count'
            worker=self.make_worker(d,0)
            with patch.dict(os.environ, {'WATCHDOG_TEST_COUNT':str(count),
                                         'WATCHDOG_FAIL_UNTIL':'0'}):
                self.assertEqual(supervise([sys.executable, str(worker)],
                                initial_delay=0,maximum_delay=0,
                                lock_path=Path(d)/'lock',log=lambda msg:None),0)
            self.assertEqual(count.read_text(),'1')

    def test_duplicate_live_supervisors_refused(self):
        import fcntl
        if not HAVE_REAL_FCNTL:
            self.skipTest('Windows 没有真正的 fcntl.flock，独占锁无法验证')
        with tempfile.TemporaryDirectory() as d:
            lock=Path(d)/'lock'
            with open(str(lock),'a+') as f:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
                self.assertEqual(supervise([sys.executable, '-c','pass'],
                                    initial_delay=0,maximum_delay=0,lock_path=lock,
                                    log=lambda x:None),3)

    def test_main_unlimited_reexec_default_and_bounded_wait(self):
        from main import parser
        a=parser().parse_args([])
        self.assertEqual(a.restart_max_streak,0)
        self.assertEqual(a.failure_action,'restart')
        script=(ROOT/'main.py').read_text(encoding='utf-8')
        self.assertIn('args.restart_max_streak == 0 or current < args.restart_max_streak',script)
        self.assertIn('min(15.0, max(args.restart_delay, 0.5)',script)
        self.assertIn('raise ProgramRestartRequested',script)

    def test_launcher_runs_supervisor_but_preserves_private_runtime(self):
        sh=(ROOT/'start_rk3568.sh').read_text(encoding='utf-8')
        self.assertIn('exec python3 -u live_watchdog.py -- "$@"',sh)
        self.assertIn('--live-once',sh)
        runtime=(ROOT/'start_v12.sh').read_text(encoding='utf-8')
        self.assertIn('mount --bind "$runtime" "$target"',runtime)
        self.assertIn('--failure-action restart',runtime)


if __name__=='__main__':unittest.main()
