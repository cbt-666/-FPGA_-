"""Heartbeat watchdog for a loop that can block forever inside the NPU.

`rknnlite.inference()` has no timeout. When the NPU hangs -- driver fault,
power glitch on the FPGA bus, or a corrupted model handle -- the call never
returns and the acquisition loop simply stops. Nothing logs, nothing raises,
and live_watchdog.py cannot help: the child process is still alive, so it has
no exit code to react to. The board looks "frozen" with arrows stuck on
whatever was last displayed.

This watchdog watches a heartbeat the loop bumps once per round. If the gap
exceeds `timeout_s` it calls `on_stall` exactly once.

Do NOT try to raise into the worker thread or unwind it: it is blocked inside
C code, so interpreter-level scheduling will not resume it. The only safe
recovery is to abandon the process. `default_on_stall` therefore dumps every
thread's stack (so the log shows WHERE it hung) and then hard-exits with
EXIT_HUNG, letting live_watchdog.py start a clean process.

Exit status is a normal negative-free code, so `subprocess.wait()` returns it
and the supervisor restarts without special handling.
"""
import logging
import os
import sys
import threading
import time
import traceback

LOG = logging.getLogger('rk3568.hang')

# Distinguishable from success (0), ordinary failure (1) and the usual
# SIGTERM-derived negatives. live_watchdog restarts on any non-zero code.
EXIT_HUNG = 70


def default_on_stall(timeout_s, where=None):
    """Dump stacks, then exit the process so the supervisor restarts it."""
    LOG.critical('看门狗：主循环 %.1f 秒没有心跳，判定 NPU/阻塞调用卡死。'
                 '导出全部线程栈后退出，由 live_watchdog 重启进程。', timeout_s)
    if where is not None:
        LOG.critical('最后一次心跳位置：%s', where)
    frames = sys._current_frames()
    for tid, frame in frames.items():
        name = 'unknown'
        for thread in threading.enumerate():
            if thread.ident == tid:
                name = thread.name
                break
        LOG.critical('--- 线程 %s (%s) ---\n%s', name, tid,
                     ''.join(traceback.format_stack(frame)))
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except Exception:
            pass
    logging.shutdown()
    os._exit(EXIT_HUNG)


class HangWatchdog:
    """Bump `beat()` from the main loop; it fires `on_stall` once if silent."""

    def __init__(self, timeout_s=45.0, on_stall=None):
        if timeout_s <= 0:
            raise ValueError('timeout_s must be positive')
        self.timeout_s = float(timeout_s)
        self.on_stall = on_stall
        self._last = time.monotonic()
        self._stopped = threading.Event()
        self._fired = threading.Event()
        self._thread = None
        self.where = None

    def start(self):
        if self._thread is not None:
            return self
        self._last = time.monotonic()
        self._thread = threading.Thread(target=self._run, name='hang-watchdog',
                                        daemon=True)
        self._thread.start()
        return self

    def beat(self, where=None):
        """Record progress. `where` is a short label shown if a stall follows."""
        self._last = time.monotonic()
        if where is not None:
            self.where = where

    def _run(self):
        # Poll faster than the timeout so the reported gap stays tight.
        interval = max(0.01, min(1.0, self.timeout_s / 4.0))
        while not self._stopped.wait(interval):
            if self._fired.is_set():
                continue
            if time.monotonic() - self._last >= self.timeout_s:
                self._fired.set()
                try:
                    if self.on_stall is not None:
                        self.on_stall()
                except SystemExit:
                    raise
                except BaseException:
                    # A broken callback must not silently disable the guard.
                    LOG.exception('看门狗 on_stall 回调自身出错')

    def stop(self):
        self._stopped.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
