"""Same-process Python program restart (os.execv). NEVER reboot Linux.

A successful pair resets the restart streak. The last FPGA display is kept in
BOTH os.environ and a state file:

  * os.environ survives os.execv (the inner restart path in main.py).
  * the file survives a fully new subprocess started by live_watchdog.py,
    which does NOT inherit this process's environment. Without it, every
    watchdog restart began with all arrows hidden -- the observed
    "one side's arrow disappeared" symptom.
"""
import json
import logging
import os
import sys
from pathlib import Path
from protocol import Display

LOG = logging.getLogger('rk3576.restart')
STREAK_ENV = 'RK3576_PY_RESTART_STREAK'
DISPLAY_ENV = 'RK3576_PY_LAST_DISPLAY'
STATE_FILE = Path(__file__).resolve().parent / '.display_state.json'

class ProgramRestartRequested(RuntimeError):
    """Raise from main loop to unwind its finally (socket/model cleanup)."""


def restart_streak():
    try:
        value = int(os.environ.get(STREAK_ENV,'0'))
    except (TypeError, ValueError):
        return 0
    return max(0,min(value,1000000))


def mark_restart():
    next_count=restart_streak()+1
    os.environ[STREAK_ENV]=str(next_count)
    return next_count


def reset_restart_streak():
    os.environ.pop(STREAK_ENV,None)


def _parse_display(raw):
    obj=json.loads(raw)
    if not isinstance(obj,dict): raise ValueError('not a dictionary')
    colors=[obj.get(s) for s in ('left','right','straight')]
    if any(c is not None and (type(c) is not int or c not in (0,1,2,3)) for c in colors):
        raise ValueError('invalid arrow color')
    warning=obj.get('edge_warning',False)
    if type(warning) is not bool: raise ValueError('invalid edge-warning flag')
    return Display(colors[0],colors[1],colors[2],warning)


def save_display(display):
    payload=json.dumps({
        'left': display.left, 'right': display.right,
        'straight': display.straight,
        'edge_warning': display.edge_warning
    },separators=(',',':'))
    os.environ[DISPLAY_ENV]=payload
    try:
        STATE_FILE.write_text(payload,encoding='utf-8')
    except OSError as exc:
        # A read-only workdir must not break the live loop; the env copy still
        # covers the os.execv path.
        LOG.warning('无法写入显示状态文件 %s: %s',STATE_FILE,exc)


def read_display():
    raw=os.environ.get(DISPLAY_ENV)
    source='环境变量'
    if not raw:
        try:
            if STATE_FILE.is_file():
                raw=STATE_FILE.read_text(encoding='utf-8')
                source='状态文件'
        except OSError as exc:
            LOG.warning('读取显示状态文件失败: %s',exc)
            return Display()
    if not raw:
        return Display()
    try:
        return _parse_display(raw)
    except (ValueError,TypeError) as exc:
        LOG.warning('之前(%s)的显示状态数据损坏；忽略缓存: %s',source,exc)
        return Display()


def reexec_program(script_path, argv_tail, execv=None):
    """Reload the entire Python program in the *same process*, same arguments.

    os.execv replaces Python process image, preserving the PID and environment.
    It is NOT sudo reboot/systemctl reboot and needs no root. The caller must
    first close sockets and RKNN handles via finally.
    """
    import time
    if execv is None:execv=os.execv
    script=str(Path(script_path).resolve())
    command=[sys.executable,'-u',script,*argv_tail]
    LOG.critical('重新启动 Python 程序: %s',command)
    sys.stdout.flush()
    sys.stderr.flush()
    logging.shutdown()
    execv(sys.executable,command)
    # Only reached on a mocked execv or unexpected failure.
    raise RuntimeError('程序 execv 返回但没有重新执行（可能是测试替身）')
