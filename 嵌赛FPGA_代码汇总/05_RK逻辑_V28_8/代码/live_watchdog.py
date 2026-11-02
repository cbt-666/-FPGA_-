#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RK3568 foreground live supervisor: restart Python startup after failures.

The supervisor handles *preflight/network/model-init* failures before main.py
can start; main.py's os.execv retains responsibility for live receive failures.
No reboot of Linux, no RKNN system-library replacement, and no fake frames.
Ctrl+C/SIGTERM stops the worker and the supervisor. Python 3.8 compatible.
"""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent


def supervise(command, *, initial_delay=1.0, maximum_delay=15.0,
              stable_seconds=60.0, max_restarts=0, lock_path=None,
              log=print, on_launch=None):
    """Return 0 on normal completion / interruption, else diagnostic rc.

    max_restarts=0 means keep retrying without a fixed limit; backoff is capped.
    """
    if not command or initial_delay < 0 or maximum_delay < initial_delay:
        raise ValueError('invalid supervisor command or retry delays')
    stop = threading.Event()
    current = [None]

    def stop_worker(signum, frame):
        stop.set()
        process = current[0]
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            previous[sig] = signal.signal(sig, stop_worker)
        except ValueError:  # Tests may invoke supervise in a non-main thread.
            pass

    lock_file = None
    tries = 0
    continuous = 0
    try:
        if lock_path is not None:
            lock_file = open(str(lock_path), 'a+')
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                log('错误：已有一个 RK3568 live 守护程序运行，拒绝重复占用 UDP 1234。')
                return 3
        while not stop.is_set():
            began = time.monotonic()
            tries += 1
            log('===== RK3568 自动运行第 {} 次：网络 → FPGA 720行双图 → NPU → main.py ====='.format(tries))
            if on_launch is not None:
                on_launch(tries)
            try:
                child = subprocess.Popen(command, cwd=str(ROOT), start_new_session=True)
            except OSError as exc:
                log('启动子进程失败：{}'.format(exc))
                rc = 127
            else:
                current[0] = child
                try:
                    rc = child.wait()
                except KeyboardInterrupt:
                    stop_worker(signal.SIGINT, None)
                    rc = child.wait()
                finally:
                    current[0] = None
            if stop.is_set() or rc in (-signal.SIGINT, 130):
                log('收到停止请求，RK3568 程序停止自动重启。')
                return 0
            if rc == 0:
                log('主程序正常结束；不自动重启（可能设置了 --max-cycles 或主动结束）。')
                return 0
            ran = time.monotonic() - began
            continuous = 0 if ran >= stable_seconds else continuous + 1
            if max_restarts and tries > max_restarts:
                log('测试模式的最大重启次数已到（{}），最后状态码={}'.format(max_restarts, rc))
                return rc if rc else 1
            delay = min(maximum_delay, initial_delay * (2 ** min(continuous-1, 8)))
            log('子程序返回 {}，运行 {:.1f}s；{:.1f}s 后重新启动 Python 工程（不会重启 Linux）。'.format(rc,ran,delay))
            stop.wait(delay)
        return 0
    finally:
        if lock_file is not None:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            lock_file.close()
        for sig, old_handler in previous.items():
            signal.signal(sig, old_handler)


def main(argv=None):
    p = argparse.ArgumentParser(description='RK3568 live 自动重启守护器（不重启 Linux）')
    p.add_argument('--initial-delay', type=float, default=1.)
    p.add_argument('--max-delay', type=float, default=15.)
    p.add_argument('--stable-seconds', type=float, default=60.)
    p.add_argument('--max-restarts', type=int, default=0,
                   help='0：无限恢复；正整数：只供调试和测试')
    p.add_argument('main_args', nargs=argparse.REMAINDER)
    a = p.parse_args(argv)
    if a.stable_seconds < 0 or a.max_restarts < 0:
        p.error('稳定周期和重启次数不能小于0')
    flags = a.main_args
    if flags and flags[0] == '--':
        flags = flags[1:]
    skip_preflight = bool(flags and flags[0] == '--skip-preflight')
    if skip_preflight:
        flags = flags[1:]
        if flags and flags[0] == '--':
            flags = flags[1:]
    cmd = ['./start_v12.sh',
           *(['--skip-preflight'] if skip_preflight else []), '--', *flags]
    try:
        return supervise(cmd, initial_delay=a.initial_delay, maximum_delay=a.max_delay,
                         stable_seconds=a.stable_seconds, max_restarts=a.max_restarts,
                         lock_path=ROOT/'.rk3568_live.lock',
                         log=lambda msg: print(msg, flush=True))
    except (ValueError, OSError) as exc:
        print('守护器异常：{}'.format(exc),file=sys.stderr,flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
