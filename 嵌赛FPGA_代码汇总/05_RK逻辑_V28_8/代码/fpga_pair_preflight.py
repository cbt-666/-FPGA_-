#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Strict, bounded FPGA grey preflight (Python 3.8 compatible).

2026-10-31：全工程只请求 MODE 1。二值预检已删除 —— 预检自己再去翻一次
模式，正是旧版"启动第一轮就收到上一模式残留包"的来源。
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from protocol import Display, IMAGE_MODE
from udp_pair import UDPLink


def capture_grey(local_ip='0.0.0.0', local_port=1234,
                 fpga_ip='192.168.1.10', fpga_port=1234, fpga_source_port=1234,
                 timeout=4.0, output_dir='captures', retries=3,
                 retry_delay=0.08, mode0_grace_ms=180.0):
    if retries < 1 or timeout <= 0 or retry_delay < 0 or mode0_grace_ms < 0:
        raise ValueError('retries >= 1, timeout > 0, retry_delay >= 0, mode0_grace_ms >= 0 required')
    print('绑定 {}:{} -> FPGA {}:{}（预期来源端口 {}）'.format(
        local_ip, local_port, fpga_ip, fpga_port, fpga_source_port), flush=True)
    print('模式请求：灰度 00 04（MODE 1）；720 行；模式切换宽限 {}ms；最多 {} 次请求'.format(
        mode0_grace_ms, retries), flush=True)
    link = UDPLink(local_ip, local_port, fpga_ip, fpga_source_port, fpga_ip, fpga_port)
    link.configure_receiver(idle_timeout=min(1.5, timeout),
                            drain_budget_ms=15.0, mode0_grace_ms=mode0_grace_ms)
    result = {}
    label, filename = 'gray', 'gray_yrg.npy'
    start = time.monotonic()
    frame = None
    stats = {}
    try:
        for attempt in range(1, retries + 1):
            print('模式 {} 第 {}/{} 次请求'.format(label, attempt, retries), flush=True)
            try:
                frame = link.request_frame(Display(), timeout=timeout,
                    on_request=lambda command, drained: print(
                        '请求 {}: {}；清理旧包 {}'.format(label, command.hex(' '), drained),
                        flush=True))
                if frame.pixels.shape != (720, 1280) or frame.pixels.dtype != np.uint8:
                    raise RuntimeError('图像形状/类型错误: {} {}'.format(
                        frame.pixels.shape, frame.pixels.dtype))
                break
            except (ValueError, TimeoutError, OSError, RuntimeError) as exc:
                frame = None
                diag = dict(getattr(link, 'last_report', {}))
                print('模式 {} 第 {} 次失败：{}'.format(label, attempt, exc), flush=True)
                print('接收诊断：{}'.format(json.dumps(diag, ensure_ascii=False)), flush=True)
                if attempt == retries:
                    print('已到达最大重发次数；持续出现 mode0_header_burst 时请排查 '
                          'FPGA 图像模式状态机，不能使用模式0冒充灰度图。', flush=True)
                    raise
                if retry_delay:
                    time.sleep(retry_delay)
        elapsed = round((time.monotonic() - start) * 1000, 2)
        stats = dict(getattr(link, 'last_report', {}))
        dest = Path(output_dir)
        dest.mkdir(parents=True, exist_ok=True)
        np.save(str(dest / filename), frame.pixels, allow_pickle=False)
        result[label] = {'frame_id': int(frame.frame_id), 'lines': 720,
                         'duration_ms': elapsed, 'attempts': attempt,
                         'file': str(dest / filename),
                         'gray_unique_values': stats.get('gray_unique_values'),
                         'looks_thresholded': stats.get('looks_thresholded', 0),
                         'received_udp': stats.get('udp_received', 0),
                         'valid_rows': stats.get('valid_rows', 0)}
        print('完成 {}: frame={} lines=720 attempts={} {:.1f}ms'.format(
            label, frame.frame_id, attempt, elapsed), flush=True)
    finally:
        link.close()
    Path(output_dir, 'frame_ids.json').write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print('灰度完整收图通过：' + json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    levels = result[label].get('gray_unique_values')
    print('提示：灰度级数={}。几十~几百是真灰度；十几个说明 FPGA 的模式 1 '
          '还在发阈值图（median_data 没接出来），G640 车道精度会受影响。'.format(levels),
          flush=True)
    return result


# 旧名保留：start_rk3568.sh 与若干测试仍按旧名字导入。
capture_pair = capture_grey


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--local-ip',default='0.0.0.0')
    p.add_argument('--local-port', type=int, default=1234)
    p.add_argument('--fpga-ip', default='192.168.1.10')
    p.add_argument('--fpga-port', type=int, default=1234)
    p.add_argument('--fpga-source-port', type=int, default=1234)
    p.add_argument('--timeout', type=float, default=4.0)
    p.add_argument('--output-dir', default='captures')
    p.add_argument('--retries', type=int, default=3, help='最多发送几次请求，默认3')
    p.add_argument('--retry-delay', type=float, default=0.08, help='失败后重发间隔秒，默认0.08')
    p.add_argument('--mode0-grace-ms', type=float, default=180.0,
                   help='等待模式0残留包结束的宽限毫秒数，默认180')
    a = p.parse_args()
    try:
        capture_grey(a.local_ip, a.local_port, a.fpga_ip, a.fpga_port, a.fpga_source_port,
                     a.timeout, a.output_dir, a.retries, a.retry_delay, a.mode0_grace_ms)
    except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
        print('严格收图预检失败：' + str(exc), flush=True)
        raise SystemExit(7)
