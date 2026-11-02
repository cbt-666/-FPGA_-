"""Cheap system-health sampling for the RK3568 board.

The v17 board log had no OOM line and no restart, but the last rounds went from
950 ms to 16 s and the slowest single round spent 9.3 s just *receiving* UDP.
That shape (receive and inference both collapsing) is a host-level stall, not a
decision-logic bug, so the useful thing to record is the host: RSS, temperature,
load average and the memory the kernel thinks is free.

Everything here is best-effort and never raises: a missing /proc file or an
unreadable thermal zone must not take the main loop down.
"""
import os
from pathlib import Path

_THERMAL_ROOTS = (
    '/sys/class/thermal/thermal_zone0/temp',
    '/sys/class/thermal/thermal_zone1/temp',
)


def _read(path, default=None):
    try:
        return Path(path).read_text().strip()
    except Exception:
        return default


def _meminfo():
    """Return (mem_total_mb, mem_available_mb) or (None, None)."""
    total = avail = None
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            key, _, rest = line.partition(':')
            if not rest:
                continue
            value = rest.split()
            if not value:
                continue
            kb = int(value[0])
            if key == 'MemTotal':
                total = round(kb / 1024)
            elif key == 'MemAvailable':
                avail = round(kb / 1024)
    except Exception:
        pass
    return total, avail


def _rss_mb():
    """Resident set size of this process, in MiB."""
    try:
        for line in Path('/proc/self/status').read_text().splitlines():
            if line.startswith('VmRSS:'):
                return round(int(line.split()[1]) / 1024)
    except Exception:
        pass
    return None


def _temperature_c():
    for path in _THERMAL_ROOTS:
        raw = _read(path)
        if raw is None:
            continue
        try:
            # Values are milli-degrees C.
            return round(int(raw) / 1000.0, 1)
        except Exception:
            continue
    return None


def sample():
    """One health snapshot. Every field may be None on a platform without it."""
    load = None
    try:
        load = float(os.getloadavg()[0])
    except Exception:
        pass
    total, avail = _meminfo()
    return {
        'rss_mb': _rss_mb(),
        'mem_total_mb': total,
        'mem_available_mb': avail,
        'load1': load,
        'temp_c': _temperature_c(),
    }


def warn_lines(prev, cur, min_avail_mb=150, max_temp_c=85.0, max_rss_mb=900):
    """Human-readable warnings for a sample, given the previous one.

    Thresholds are deliberately loose: this exists to catch a board heading for
    a freeze, not to police normal operation.
    """
    out = []
    if cur.get('mem_available_mb') is not None and \
            cur['mem_available_mb'] < min_avail_mb:
        out.append(f"可用内存只剩 {cur['mem_available_mb']}MB"
                   f"（总量 {cur.get('mem_total_mb')}MB）")
    if cur.get('temp_c') is not None and cur['temp_c'] >= max_temp_c:
        out.append(f"SoC 温度 {cur['temp_c']}°C，已到降频区")
    if cur.get('rss_mb') is not None and cur['rss_mb'] >= max_rss_mb:
        out.append(f"本进程 RSS {cur['rss_mb']}MB")
    elif (cur.get('rss_mb') is not None and prev.get('rss_mb') is not None
          and cur['rss_mb'] - prev['rss_mb'] >= 50):
        out.append(f"RSS 增长 {prev['rss_mb']}->{cur['rss_mb']}MB")
    if cur.get('load1') is not None and cur['load1'] >= 8.0:
        out.append(f"1 分钟负载 {cur['load1']:.1f}")
    return out
