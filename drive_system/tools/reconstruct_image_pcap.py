#!/usr/bin/env python3
"""Stream FPGA image UDP packets from a PCAPNG capture and rebuild one frame per mode."""

from __future__ import annotations

import argparse
import collections
import json
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image


WIDTH = 1280
HEIGHT = 720
UDP_PAYLOAD_LEN = 1288


@dataclass
class Frame:
    kind: str
    frame_no: int
    first_packet: int
    packets: dict[tuple[int, int], bytes] = field(default_factory=dict)
    duplicates: int = 0
    conflicts: int = 0

    def add(self, key: tuple[int, int], data: bytes) -> None:
        old = self.packets.get(key)
        if old is None:
            self.packets[key] = data
        elif old == data:
            self.duplicates += 1
        else:
            self.conflicts += 1

    @property
    def expected_packets(self) -> int:
        return HEIGHT * (2 if self.kind == "rgb565" else 1)

    @property
    def complete(self) -> bool:
        return len(self.packets) == self.expected_packets and self.conflicts == 0


def iter_pcapng_packets(path: Path):
    endian = "<"
    link_types: dict[int, int] = {}
    interface_index = 0
    with path.open("rb") as src:
        while True:
            header = src.read(8)
            if not header:
                return
            if len(header) != 8:
                raise ValueError("truncated PCAPNG block header")
            raw_type, raw_length = header[:4], header[4:]
            if raw_type == b"\x0a\x0d\x0d\x0a":
                body_prefix = src.read(4)
                if body_prefix == b"\x4d\x3c\x2b\x1a":
                    endian = "<"
                elif body_prefix == b"\x1a\x2b\x3c\x4d":
                    endian = ">"
                else:
                    raise ValueError("invalid PCAPNG byte-order magic")
                total_length = struct.unpack(endian + "I", raw_length)[0]
                src.seek(total_length - 12, 1)
                link_types.clear()
                interface_index = 0
                continue

            block_type = struct.unpack(endian + "I", raw_type)[0]
            total_length = struct.unpack(endian + "I", raw_length)[0]
            if total_length < 12 or total_length % 4:
                raise ValueError(f"invalid PCAPNG block length {total_length}")
            body = src.read(total_length - 12)
            trailer = src.read(4)
            if len(body) != total_length - 12 or len(trailer) != 4:
                raise ValueError("truncated PCAPNG block")
            if struct.unpack(endian + "I", trailer)[0] != total_length:
                raise ValueError("PCAPNG block length mismatch")

            if block_type == 1 and len(body) >= 8:  # Interface Description Block
                link_types[interface_index] = struct.unpack_from(endian + "H", body, 0)[0]
                interface_index += 1
            elif block_type == 6 and len(body) >= 20:  # Enhanced Packet Block
                interface_id, _, _, captured_len, _ = struct.unpack_from(endian + "IIIII", body, 0)
                if link_types.get(interface_id) == 1 and 20 + captured_len <= len(body):
                    yield body[20 : 20 + captured_len]


def udp_payload(frame: bytes):
    if len(frame) < 14:
        return None
    eth_type = struct.unpack_from("!H", frame, 12)[0]
    offset = 14
    while eth_type in (0x8100, 0x88A8):
        if len(frame) < offset + 4:
            return None
        eth_type = struct.unpack_from("!H", frame, offset + 2)[0]
        offset += 4
    if eth_type != 0x0800 or len(frame) < offset + 20:
        return None
    version_ihl = frame[offset]
    if version_ihl >> 4 != 4:
        return None
    ihl = (version_ihl & 0x0F) * 4
    if ihl < 20 or len(frame) < offset + ihl + 8 or frame[offset + 9] != 17:
        return None
    ip_id = struct.unpack_from("!H", frame, offset + 4)[0]
    src_ip = ".".join(str(v) for v in frame[offset + 12 : offset + 16])
    dst_ip = ".".join(str(v) for v in frame[offset + 16 : offset + 20])
    udp_offset = offset + ihl
    src_port, dst_port, udp_len = struct.unpack_from("!HHH", frame, udp_offset)
    if udp_len < 8 or udp_offset + udp_len > len(frame):
        return None
    return src_ip, dst_ip, src_port, dst_port, ip_id, frame[udp_offset + 8 : udp_offset + udp_len]


def decode_header(payload: bytes):
    if len(payload) != UDP_PAYLOAD_LEN:
        return None
    if payload[6:8] == b"\x02\xd0":
        frame_no = int.from_bytes(payload[0:4], "big")
        line = int.from_bytes(payload[4:6], "big")
        if line < HEIGHT:
            return "encoded", frame_no, line, 0, payload[8:]
    elif payload[0:2] == b"\x00\x00" and payload[6] == 0 and payload[7] <= 1:
        frame_no = int.from_bytes(payload[2:4], "big")
        line = int.from_bytes(payload[4:6], "big")
        if line < HEIGHT:
            return "rgb565", frame_no, line, payload[7], payload[8:]
    return None


def encoded_kind(frame: Frame) -> str:
    values = set()
    for data in frame.packets.values():
        values.update(data)
        if any(value not in (0, 253) for value in values):
            return "yrg"
    return "binary"


def save_rgb565(frame: Frame, path: Path) -> dict:
    raw = bytearray(WIDTH * HEIGHT * 2)
    for line in range(HEIGHT):
        for half in range(2):
            data = frame.packets[(line, half)]
            start = (line * WIDTH + half * 640) * 2
            raw[start : start + len(data)] = data
    words = np.frombuffer(raw, dtype=">u2").reshape(HEIGHT, WIDTH)
    rgb = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    rgb[:, :, 0] = ((words >> 11) & 0x1F) * 255 // 31
    rgb[:, :, 1] = ((words >> 5) & 0x3F) * 255 // 63
    rgb[:, :, 2] = (words & 0x1F) * 255 // 31
    Image.fromarray(rgb, "RGB").save(path)
    return {"min": rgb.min(axis=(0, 1)).tolist(), "max": rgb.max(axis=(0, 1)).tolist()}


def save_encoded(frame: Frame, path: Path, kind: str) -> dict:
    raw = b"".join(frame.packets[(line, 0)] for line in range(HEIGHT))
    pixels = np.frombuffer(raw, dtype=np.uint8).reshape(HEIGHT, WIDTH)
    if kind == "yrg":
        rgb = np.repeat(pixels[:, :, None], 3, axis=2)
        rgb[pixels == 254] = (255, 0, 0)
        rgb[pixels == 255] = (0, 255, 0)
        Image.fromarray(rgb, "RGB").save(path)
    else:
        Image.fromarray(np.where(pixels == 0, 0, 255).astype(np.uint8), "L").save(path)
    counts = collections.Counter(pixels.ravel().tolist())
    return {
        "unique_values": len(counts),
        "zero_pixels": counts[0],
        "value_253_pixels": counts[253],
        "red_marker_pixels": counts[254],
        "green_marker_pixels": counts[255],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    frames: dict[tuple[str, int], Frame] = {}
    udp_endpoints = collections.Counter()
    udp_lengths = collections.Counter()
    commands = []
    image_sequence = []
    total_packets = image_packets = malformed_image_packets = 0

    for packet_index, packet in enumerate(iter_pcapng_packets(args.capture), 1):
        total_packets += 1
        udp = udp_payload(packet)
        if udp is None:
            continue
        src_ip, dst_ip, src_port, dst_port, ip_id, payload = udp
        udp_endpoints[(src_ip, src_port, dst_ip, dst_port)] += 1
        udp_lengths[len(payload)] += 1
        if len(payload) == 2:
            commands.append({"packet": packet_index, "src": src_ip, "dst": dst_ip,
                             "payload_hex": payload.hex()})
        decoded = decode_header(payload)
        if decoded is None:
            if len(payload) == UDP_PAYLOAD_LEN:
                malformed_image_packets += 1
            continue
        kind, frame_no, line, half, data = decoded
        image_packets += 1
        image_sequence.append((ip_id, kind, frame_no, line, half))
        key = (kind, frame_no)
        if key not in frames:
            frames[key] = Frame(kind, frame_no, packet_index)
        frames[key].add((line, half), data)

    complete = [frame for frame in frames.values() if frame.complete]
    selected: dict[str, Frame] = {}
    for frame in sorted(complete, key=lambda item: item.first_packet):
        kind = encoded_kind(frame) if frame.kind == "encoded" else "rgb565"
        selected.setdefault(kind, frame)

    outputs = {}
    for kind in ("rgb565", "yrg", "binary"):
        frame = selected.get(kind)
        if frame is None:
            continue
        path = args.output / f"{kind}_frame_{frame.frame_no}.png"
        stats = save_rgb565(frame, path) if kind == "rgb565" else save_encoded(frame, path, kind)
        outputs[kind] = {
            "frame_no": frame.frame_no,
            "packet_count": len(frame.packets),
            "duplicates": frame.duplicates,
            "conflicts": frame.conflicts,
            "path": str(path.resolve()),
            **stats,
        }

    frame_summary = []
    for frame in sorted(frames.values(), key=lambda item: item.first_packet):
        display_kind = encoded_kind(frame) if frame.kind == "encoded" else frame.kind
        frame_summary.append({
            "kind": display_kind,
            "frame_no": frame.frame_no,
            "packets": len(frame.packets),
            "expected": frame.expected_packets,
            "complete": frame.complete,
            "duplicates": frame.duplicates,
            "conflicts": frame.conflicts,
        })

    continuity = {
        "ip_id_missing_packets": 0,
        "logical_missing_packets_within_frames": 0,
        "logical_missing_without_matching_ip_id_gap": 0,
        "out_of_order_packets": 0,
    }
    for previous, current in zip(image_sequence, image_sequence[1:]):
        prev_ip, prev_kind, prev_frame, prev_line, prev_half = previous
        curr_ip, curr_kind, curr_frame, curr_line, curr_half = current
        ip_gap = ((curr_ip - prev_ip) & 0xFFFF) - 1
        if ip_gap >= 0:
            continuity["ip_id_missing_packets"] += ip_gap
        if prev_kind == curr_kind and prev_frame == curr_frame:
            prev_pos = prev_line * (2 if prev_kind == "rgb565" else 1) + prev_half
            curr_pos = curr_line * (2 if curr_kind == "rgb565" else 1) + curr_half
            if curr_pos <= prev_pos:
                continuity["out_of_order_packets"] += 1
            elif curr_pos > prev_pos + 1:
                logical_gap = curr_pos - prev_pos - 1
                continuity["logical_missing_packets_within_frames"] += logical_gap
                continuity["logical_missing_without_matching_ip_id_gap"] += max(
                    logical_gap - max(ip_gap, 0), 0
                )

    kind_summary = {}
    for kind in ("rgb565", "yrg", "binary"):
        matching = [item for item in frame_summary if item["kind"] == kind]
        if matching:
            kind_summary[kind] = {
                "frames": len(matching),
                "complete_frames": sum(item["complete"] for item in matching),
                "incomplete_frames": sum(not item["complete"] for item in matching),
                "first_frame": matching[0]["frame_no"],
                "last_frame": matching[-1]["frame_no"],
            }

    report = {
        "capture": str(args.capture.resolve()),
        "captured_packets": total_packets,
        "image_packets": image_packets,
        "malformed_1288_byte_packets": malformed_image_packets,
        "commands": commands,
        "continuity": continuity,
        "mode_summary": kind_summary,
        "udp_payload_lengths": dict(udp_lengths.most_common()),
        "udp_endpoints": [
            {"src": f"{key[0]}:{key[1]}", "dst": f"{key[2]}:{key[3]}", "packets": value}
            for key, value in udp_endpoints.most_common()
        ],
        "frames": frame_summary,
        "outputs": outputs,
    }
    report_path = args.output / "reconstruction_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
