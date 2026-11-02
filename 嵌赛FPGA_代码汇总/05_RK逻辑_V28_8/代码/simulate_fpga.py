#!/usr/bin/env python3
"""LOCAL FPGA simulator: one 16-bit request -> exactly ONE MODE 1 grey frame.

2026-10-31：工程只请求 MODE 1，二值模式已删除。收到 MODE 0/2 一律拒绝并打印，
这样"又在请求二值"的回归会被本地联调直接暴露。Never run concurrently with FPGA.
"""
import argparse
import socket
import struct
import time
import numpy as np
from protocol import decode_command, IMAGE_MODE
from udp_pair import H, W


def make_frame():
    # 真灰度：每一列一个不同的级数，外加一块纯红 254、一块纯绿 255。
    gray = np.tile(np.arange(W, dtype=np.uint16) % 254, (H, 1)).astype(np.uint8)
    gray[100:120, 200:220] = 254
    gray[100:120, 300:320] = 255
    return gray


def emit(sock, target, img, fid, delay=.0002, missing_row=-1):
    for y in range(H):
        if y == missing_row:
            continue
        payload = struct.pack('>IHH', fid, y, H) + img[y].tobytes()
        sock.sendto(payload, target)
        if delay:
            time.sleep(delay)


def main():
    p = argparse.ArgumentParser(description='MODE 1 (grey) request UDP test server')
    p.add_argument('--bind', default='127.0.0.1')
    p.add_argument('--port', type=int, default=23456)
    p.add_argument('--gap', type=float, default=.0002)
    p.add_argument('--missing-row', type=int, default=-1)
    p.add_argument('--drop-first', action='store_true',
                   help='simulate one incomplete frame for the retry test')
    p.add_argument('--max-requests', type=int, default=0)
    args = p.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    gray = make_frame()
    dropped = False
    try:
        n = 0
        while not args.max_requests or n < args.max_requests:
            packet, source = sock.recvfrom(4096)
            try:
                display, mode = decode_command(packet)
            except ValueError:
                print('invalid request: ' + packet.hex(' '), flush=True)
                continue
            n += 1
            if mode != IMAGE_MODE:
                print(f'拒绝非 MODE 1 请求（RK 端不应再请求二值/RGB565）: mode={mode}', flush=True)
                continue
            img, kind = gray, 'gray'
            missing = args.missing_row
            if args.drop_first and not dropped:
                missing = 333
                dropped = True
            print(f'request {n} bytes=[{packet.hex(" ")}] mode={mode} image={kind} '
                  f'missing_row={missing} display={display.as_dict()}', flush=True)
            emit(sock, source, img, n, args.gap, missing)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == '__main__':
    main()
