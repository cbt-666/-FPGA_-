"""Internet-path errors must never escape as OSError and kill the run.

Field evidence (rk3568_v15_7.log): three of five restarts were
`OSError: [Errno 101] Network is unreachable` raised straight out of
`sock.sendto` / `sock.recvfrom_into`. main.py only catches
(TimeoutError, ValueError) around acquisition, so an OSError unwinds the whole
process. `send()` was converted in V15.7_fixed; the two receive paths were not.

Second failure mode: the NPU call has no timeout. If `rknn.inference` never
returns, the loop blocks forever and even live_watchdog cannot see it -- the
child process is alive, so it is never restarted. A heartbeat watchdog closes
that hole, but it may only exit the process; it must not try to unwind a
thread that is stuck inside C code.
"""
import socket
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from udp_pair import UDPLink, FastProtocolMismatch
from protocol import GRAY, BINARY, Display, encode_packet

W, H, ROW_BYTES = 1280, 720, 1288
HEADER = struct.Struct('>IHH')


def rows_packet(fid, rows, value=7):
    for row in rows:
        yield HEADER.pack(fid, row, H) + bytes([value]) * W


class Link(UDPLink):
    def __init__(self):
        self.sock = None
        self.recv_buf = bytearray(2048)
        self.pixel_view = np.frombuffer(self.recv_buf, dtype=np.uint8,
                                        count=W, offset=8)
        import logging
        self.log = logging.getLogger('test')
        self.source_ip = '192.168.1.10'
        self.source_port = 1234
        self.target = ('192.168.1.10', 1234)
        self.last_report = {}
        self.last_frame_id = None
        self.last_mode = None
        self.last_drain_hit_limit = False
        self.last_drain_ms = 0.0
        self.drain_budget_ms = 15.0
        self.drain_max_packets = 4096
        self.idle_timeout = .8
        self.bad_header_limit = 64
        self.bad_length_limit = 64
        self.mode0_grace_ms = 35.0


class _DeadSocket:
    """Every operation fails the way an unplugged NIC fails."""

    def __init__(self, exc=None):
        self.exc = exc or OSError(101, 'Network is unreachable')

    def setblocking(self, flag):
        pass

    def settimeout(self, value):
        pass

    def recvfrom_into(self, buf, *a):
        raise self.exc

    def sendto(self, data, addr):
        raise self.exc


class DrainPendingTests(unittest.TestCase):
    def test_drain_survives_network_unreachable(self):
        link = Link()
        link.sock = _DeadSocket()
        # Must not raise: a drain failure is not a reason to stop the run.
        dropped = link.drain_pending()
        self.assertEqual(dropped, 0)

    def test_drain_survives_enobufs(self):
        link = Link()
        link.sock = _DeadSocket(OSError(105, 'No buffer space available'))
        self.assertEqual(link.drain_pending(), 0)


class CollectFrameErrorTests(unittest.TestCase):
    def test_receive_oserror_becomes_timeout(self):
        link = Link()
        link.sock = _DeadSocket()
        with self.assertRaises(TimeoutError):
            link.collect_frame(timeout=.05, expected_mode=GRAY)

    def test_receive_oserror_on_binary_mode_becomes_timeout(self):
        link = Link()
        link.sock = _DeadSocket(OSError(113, 'No route to host'))
        with self.assertRaises(TimeoutError):
            link.collect_frame(timeout=.05, expected_mode=BINARY)

    def test_request_frame_never_leaks_oserror(self):
        link = Link()
        link.sock = _DeadSocket()
        with self.assertRaises(TimeoutError):
            link.request_frame(Display(), timeout=.05)

    def test_good_packets_still_assemble(self):
        """Wrapping OSError must not change the success path."""
        class Good(_DeadSocket):
            def __init__(self):
                super().__init__()
                self._it = rows_packet(1, range(H))
            def recvfrom_into(self, buf, *a):
                try:
                    p = next(self._it)
                except StopIteration:
                    raise socket.timeout()
                buf[:len(p)] = p
                return len(p), ('192.168.1.10', 1234)
        link = Link()
        link.sock = Good()
        frame = link.collect_frame(timeout=2.0, expected_mode=GRAY)
        self.assertEqual(frame.frame_id, 1)
        self.assertTrue((frame.pixels == 7).all())


class HeartbeatWatchdogTests(unittest.TestCase):
    def test_heartbeat_reports_stall(self):
        from hang_watchdog import HangWatchdog
        fired = threading.Event()
        wd = HangWatchdog(timeout_s=0.05, on_stall=lambda: fired.set())
        wd.start()
        try:
            self.assertTrue(fired.wait(2.0), 'watchdog never reported a stall')
        finally:
            wd.stop()

    def test_heartbeat_is_quiet_while_beating(self):
        from hang_watchdog import HangWatchdog
        fired = threading.Event()
        wd = HangWatchdog(timeout_s=0.25, on_stall=lambda: fired.set())
        wd.start()
        try:
            for _ in range(6):
                wd.beat()
                time.sleep(0.05)
            self.assertFalse(fired.is_set())
        finally:
            wd.stop()

    def test_fires_only_once(self):
        from hang_watchdog import HangWatchdog
        calls = []
        wd = HangWatchdog(timeout_s=0.03, on_stall=lambda: calls.append(1))
        wd.start()
        try:
            time.sleep(0.4)
        finally:
            wd.stop()
        self.assertEqual(len(calls), 1)


if __name__ == '__main__':
    unittest.main()
