"""Actual os.execv smoke test, executed only inside an isolated child Python.

No FPGA, NPU, system reboot, or elevated privileges are used.
"""
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class TestActualPythonReexec(unittest.TestCase):
    def test_execv_restarts_script_with_same_pid_and_argv(self):
        with tempfile.TemporaryDirectory() as temp:
            demo=Path(temp)/'reexec_smoke.py'
            demo.write_text('''import os, sys
from pathlib import Path
from restart_runtime import reexec_program
if 'DEMO_FIRST_PID' not in os.environ:
    os.environ['DEMO_FIRST_PID']=str(os.getpid())
    reexec_program(Path(__file__),sys.argv[1:])
else:
    print('PID_SAME='+str(int(os.environ['DEMO_FIRST_PID'])==os.getpid()))
    print('ARGS='+','.join(sys.argv[1:]))
''',encoding='utf-8')
            env=os.environ.copy()
            env['PYTHONPATH']=str(ROOT)+os.pathsep+env.get('PYTHONPATH','')
            env.pop('DEMO_FIRST_PID',None)
            result=subprocess.run([sys.executable,str(demo),'--test-mode','1'],
                                  env=env,capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            if sys.platform.startswith('win'):
                self.skipTest('Windows 上 reexec_program 用 execv 语义无法保持 PID')
            self.assertIn('PID_SAME=True',result.stdout)
            self.assertIn('ARGS=--test-mode,1',result.stdout)


if __name__=='__main__':unittest.main()
