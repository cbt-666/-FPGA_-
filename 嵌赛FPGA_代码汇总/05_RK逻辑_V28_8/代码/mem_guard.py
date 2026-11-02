"""定期把进程占着的空闲页还给内核，避免 RK3568（2 GB）被 OOM 杀掉。

2026-10-28 第一份 V17_1 板上日志：
    第335轮后 `Killed`，子程序返回 137（=128+9，SIGKILL）
    RSS 557→660 MB，可用内存 430→150 MB（总量 1969 MB）
说明是内核 OOM killer 动的手，不是 NPU 卡死、也不是 Python 异常。

glibc 的 malloc 在 free 之后不一定会把页还给内核（尤其大量小块、
每次大小不同的分配），长期跑就表现为 RSS 只涨不跌。`malloc_trim(0)`
逐 arena 把空闲页 munmap 回去，是本进程唯一能主动做的回收。

同时跑一次 `gc.collect()`：Python 层如果有引用环（numpy 数组被异常
对象抓住），只靠引用计数永远收不回来，这一条能兜住那种情况。

⚠️ 这只能回收「本进程 glibc 堆里的空闲页」。如果 RSS 掉了但可用内存
继续掉，说明泄漏在 NPU 运行时 / 内核 socket 缓冲里，那就不是 RK 逻辑
能解决的，要换 FPGA 或 RKNN 的分配方式。
"""
import ctypes
import ctypes.util
import gc

_trim = None
try:
    _libc = ctypes.CDLL(ctypes.util.find_library('c') or 'libc.so.6')
    _trim = _libc.malloc_trim
    _trim.argtypes = [ctypes.c_size_t]
    _trim.restype = ctypes.c_int
except Exception:            # 非 glibc（musl 等）没有这个符号
    _trim = None


def _rss_mb():
    try:
        for line in open('/proc/self/status', encoding='ascii', errors='replace'):
            if line.startswith('VmRSS:'):
                return round(int(line.split()[1]) / 1024)
    except Exception:
        pass
    return None


def trim():
    """回收空闲堆 + 收引用环，返回 (回收前RSS, 回收后RSS, 是否真的调用了 malloc_trim)。"""
    before = _rss_mb()
    try:
        gc.collect()
    except Exception:
        pass
    called = False
    if _trim is not None:
        try:
            _trim(0)
            called = True
        except Exception:
            called = False
    after = _rss_mb()
    return before, after, called
