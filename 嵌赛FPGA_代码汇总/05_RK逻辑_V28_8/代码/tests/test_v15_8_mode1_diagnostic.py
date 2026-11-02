"""MODE 1 (YRG) frames must never be rejected on suspicion of being MODE 2.

The FPGA lane bitstream currently has no working YRG path: rgb_data always
carries the thresholded image, so a MODE 1 request returns 0/253 pixels.
Rejecting those in collect_frame() starved the receiver and killed the run.
The check is now a log-only diagnostic.
"""
import socket
import struct
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from udp_pair import UDPLink, looks_thresholded
from protocol import GRAY

W, H, ROW_BYTES = 1280, 720, 1288
HEADER = struct.Struct('>IHH')


class LooksBinaryTests(unittest.TestCase):
    def test_all_black_night_frame_is_not_suspicious(self):
        self.assertFalse(looks_thresholded(np.zeros((720, 1280), np.uint8)))

    def test_both_values_0_and_253_is_suspicious(self):
        frame = np.zeros((720, 1280), np.uint8)
        frame[:100] = 253
        self.assertTrue(looks_thresholded(frame))

    def test_real_grey_frame_is_not_suspicious(self):
        frame = np.random.RandomState(0).randint(0, 256, (720, 1280)).astype(np.uint8)
        self.assertFalse(looks_thresholded(frame))

    def test_flat_grey_frame_is_not_suspicious(self):
        self.assertFalse(looks_thresholded(np.full((720, 1280), 128, np.uint8)))


class StubSocket:
    """Yields pre-built packets, then raises timeout forever."""

    def __init__(self, packets, peer=('192.168.1.10', 1234)):
        self._packets = list(packets)
        self._peer = peer
        self._timeout = 0.1

    def settimeout(self, value):
        self._timeout = value

    def setblocking(self, flag):
        pass

    def recvfrom_into(self, buf):
        if not self._packets:
            raise socket.timeout()
        packet = self._packets.pop(0)
        buf[:len(packet)] = packet
        return len(packet), self._peer

    def sendto(self, data, target):
        return len(data)


def build_frame(frame_id, pixels):
    packets = []
    for row in range(H):
        packets.append(HEADER.pack(frame_id, row, H) + pixels[row].tobytes())
    return packets


class Mode1NeverRejectedTests(unittest.TestCase):
    def _link(self, pixels):
        link = UDPLink.__new__(UDPLink)
        link.sock = StubSocket(build_frame(7, pixels))
        link.recv_buf = bytearray(2048)
        link.pixel_view = np.frombuffer(link.recv_buf, dtype=np.uint8,
                                        count=W, offset=8)
        link.source_ip = '192.168.1.10'
        link.source_port = 1234
        link.last_report = {}
        link.last_mode = None
        link.last_frame_id = None
        link.idle_timeout = 0.5
        link.bad_header_limit = 64
        link.bad_length_limit = 64
        link.mode0_grace_ms = 35.0
        link.last_drain_ms = 0.0
        link.last_drain_hit_limit = False
        return link

    def test_mode1_frame_holding_only_0_and_253_is_accepted(self):
        pixels = np.zeros((H, W), np.uint8)
        pixels[:300] = 253
        self.assertTrue(looks_thresholded(pixels))
        frame = self._link(pixels).collect_frame(timeout=5.0, expected_mode=GRAY)
        self.assertEqual(frame.frame_id, 7)
        self.assertTrue(np.array_equal(frame.pixels, pixels))

    def test_mode1_all_black_frame_is_accepted(self):
        pixels = np.zeros((H, W), np.uint8)
        frame = self._link(pixels).collect_frame(timeout=5.0, expected_mode=GRAY)
        self.assertEqual(frame.pixels.shape, (H, W))

    def test_suspicious_frame_is_counted_in_diagnostics(self):
        pixels = np.zeros((H, W), np.uint8)
        pixels[:300] = 253
        link = self._link(pixels)
        link.collect_frame(timeout=5.0, expected_mode=GRAY)
        self.assertEqual(link.last_report['looks_thresholded'], 1)
        self.assertEqual(link.last_report['gray_unique_values'], 2)

    def test_real_grey_frame_reports_level_count(self):
        pixels = np.tile(np.arange(W) % 256, (H, 1)).astype(np.uint8)
        link = self._link(pixels)
        link.collect_frame(timeout=5.0, expected_mode=GRAY)
        self.assertEqual(link.last_report['gray_unique_values'], 256)
        self.assertNotIn('looks_thresholded', link.last_report)


if __name__ == '__main__':
    unittest.main()
