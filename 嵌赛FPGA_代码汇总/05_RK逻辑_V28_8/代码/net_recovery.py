"""Detect and recover from the host's FPGA link going away mid-run.

rk3568_v17_orig.log, round 14: every send started failing with
`[Errno 101] Network is unreachable`, 61 times over six program restarts. The
restarts all failed for the same reason, because `os.execv` re-runs main.py
directly and never goes back through start_v12.sh -- so setup_fpga_net.sh, which
is what puts 192.168.1.102/24 on eth0 and installs the route, never ran again.

Restarting the process cannot fix a lost interface entry. The useful responses
are (a) do not restart, keep the last display, and retry in place, and (b) when
the host allows it without a password prompt, re-run the configuration script.

Nothing here raises: a recovery attempt that cannot run is simply reported.
"""
import errno
import logging
import subprocess
from pathlib import Path

LOG = logging.getLogger('rk3568')

NETWORK_ERRNOS = frozenset({
    errno.ENETUNREACH,      # 101 Network is unreachable
    errno.EHOSTUNREACH,     # 113 No route to host
    errno.ENETDOWN,         # 100 Network is down
    errno.ENODEV,           # 19  No such device (interface removed)
    errno.ECONNREFUSED,     # 111
})

_MARKERS = ('network is unreachable', 'no route to host',
            'network is down', 'no such device')


def is_network_fault(exc):
    """True when an exception chain is a lost link/route rather than a timeout.

    Walks __cause__/__context__ because the transport wraps OSError in
    TimeoutError and main.py wraps that in RuntimeError.
    """
    seen = set()
    node = exc
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        code = getattr(node, 'errno', None)
        if code in NETWORK_ERRNOS:
            return True
        text = str(node).lower()
        if any(marker in text for marker in _MARKERS):
            return True
        node = node.__cause__ or node.__context__
    return False


def can_configure_without_prompt(script_path):
    """True when setup_fpga_net.sh can run unattended.

    The script calls sudo for ip/sysctl. Rather than guess, ask sudo whether a
    password would be required; in a headless run nobody can answer the prompt,
    and a blocked sudo would hang the whole control loop.
    """
    try:
        if not Path(script_path).is_file():
            return False
        probe = subprocess.run(['sudo', '-n', 'true'], stdin=subprocess.DEVNULL,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=3)
        return probe.returncode == 0
    except Exception:
        return False


def try_reconfigure(script_path, timeout=15):
    """Re-run the network setup script. Returns (attempted, ok, detail)."""
    if not can_configure_without_prompt(script_path):
        return False, False, 'sudo 需要密码，无法自动重配网卡'
    try:
        result = subprocess.run(['bash', str(script_path)],
                                stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=timeout, text=True)
        detail = (result.stdout or '').strip().splitlines()
        tail = detail[-1] if detail else ''
        return True, result.returncode == 0, tail
    except subprocess.TimeoutExpired:
        return True, False, f'网卡重配超时（>{timeout}s）'
    except Exception as exc:
        return True, False, f'网卡重配异常: {exc}'
