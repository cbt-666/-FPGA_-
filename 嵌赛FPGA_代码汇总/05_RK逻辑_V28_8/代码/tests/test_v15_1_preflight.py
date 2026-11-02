import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fpga_pair_preflight import capture_pair
from udp_pair import ImageFrame


class GreyPreflightTests(unittest.TestCase):
    """2026-10-31：预检只收灰度。二值预检、模式翻转全部删除。"""

    def test_retry_and_preserve_720_row_contract(self):
        calls = []
        instances = []

        class StubLink:
            def __init__(self, *args):
                self.last_report = {}
                self.mode0_grace_ms = None
                instances.append(self)

            def configure_receiver(self, **kw):
                self.mode0_grace_ms = kw['mode0_grace_ms']

            def request_frame(self, display, timeout, on_request):
                calls.append('gray')
                on_request(b'\x00\x04', 0)
                if len(calls) == 1:
                    self.last_report = {'mode0_header_like': 552, 'valid_rows': 0}
                    raise ValueError('mode0_header_burst mode=1')
                self.last_report = {'valid_rows': 720, 'udp_received': 720,
                                    'gray_unique_values': 201}
                return ImageFrame(len(calls), np.full((720, 1280), 7, np.uint8))

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as tmp, patch('fpga_pair_preflight.UDPLink', StubLink):
            r = capture_pair(output_dir=tmp, retries=3, retry_delay=0, mode0_grace_ms=180)
            self.assertEqual(calls, ['gray', 'gray'])
            self.assertEqual(r['gray']['attempts'], 2)
            self.assertEqual(instances[0].mode0_grace_ms, 180)
            self.assertEqual(int(np.load(Path(tmp) / 'gray_yrg.npy')[0, 0]), 7)
            self.assertFalse((Path(tmp) / 'binary.npy').exists(),
                             '预检不再产出二值图')

    def test_persistent_mode0_burst_fails_loudly(self):
        calls = []

        class StubLink:
            def __init__(self, *args):
                self.last_report = {}

            def configure_receiver(self, **kw):
                pass

            def request_frame(self, display, timeout, on_request):
                calls.append('gray')
                self.last_report = {'mode0_header_like': 128, 'valid_rows': 0}
                raise ValueError('mode0_header_burst')

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as tmp, patch('fpga_pair_preflight.UDPLink', StubLink):
            with self.assertRaisesRegex(ValueError, 'mode0_header_burst'):
                capture_pair(output_dir=tmp, retries=3, retry_delay=0)
            self.assertEqual(calls, ['gray', 'gray', 'gray'])
            self.assertFalse((Path(tmp) / 'gray_yrg.npy').exists())

    def test_actual_loopback_recovers_after_one_mode0_burst(self):
        service = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        service.bind(('127.0.0.1', 0))
        service.settimeout(4)
        seq = []
        errors = []

        def stream(peer, fid):
            pix = bytes([17]) * 1280
            for y in range(720):
                service.sendto(struct.pack('>IHH', fid, y, 720) + pix, peer)
                if y % 12 == 0:
                    time.sleep(.0004)

        def serve():
            try:
                for ix in range(2):   # 第一次给模式0残留，第二次给真灰度
                    raw, peer = service.recvfrom(64)
                    seq.append(raw)
                    # 现在只有一条请求，所以"第一帧就是模式0残留"必须落在 ix==0，
                    # 否则客户端第一次就成功，根本不会走到重发分支。
                    if ix == 0:
                        # RGB565 half-row packet: field 6..7 equals 0 or 1, not 720
                        # 必须持续超过 mode0_grace_ms（80ms），否则接收端会
                        # 把这段残留当成"切换中"继续等，本轮仍然能成功。
                        for y in range(240):
                            service.sendto(struct.pack('>IHH', 102, y, 0) + bytes(1280), peer)
                            service.sendto(struct.pack('>IHH', 102, y, 1) + bytes(1280), peer)
                            if y % 12 == 0:
                                time.sleep(.005)
                    else:
                        stream(peer, 103 if ix == 1 else 101)
            except Exception as exc:
                errors.append(repr(exc))

        t = threading.Thread(target=serve, daemon=True)
        t.start()
        tmp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp.bind(('127.0.0.1', 0))
        port = tmp.getsockname()[1]
        tmp.close()
        try:
            with tempfile.TemporaryDirectory() as d:
                r = capture_pair(local_ip='127.0.0.1', local_port=port,
                                 fpga_ip='127.0.0.1',
                                 fpga_port=service.getsockname()[1],
                                 fpga_source_port=service.getsockname()[1],
                                 timeout=1.8, output_dir=d, retries=3, retry_delay=0.05,
                                 mode0_grace_ms=80)
                # 每一条请求都必须是 MODE 1（bit10~11 = 01）
                for raw in seq:
                    value = int.from_bytes(raw, 'little')
                    self.assertEqual((value >> 10) & 3, 1,
                                     f'请求了非灰度模式: {raw.hex(" ")}')
                self.assertEqual(r['gray']['attempts'], 2)
                self.assertEqual(int(np.load(Path(d) / 'gray_yrg.npy')[0, 0]), 17)
        finally:
            # 先等发送线程结束再关 socket，否则 Windows 会报 WSAENOTSOCK。
            t.join(timeout=6)
            service.close()
        self.assertFalse(errors, str(errors))


if __name__ == '__main__':
    unittest.main()
