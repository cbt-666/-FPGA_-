from __future__ import annotations
"""FPGA 16-bit little-endian command (user screenshot)."""
from dataclasses import dataclass

RED, GREEN, BLUE, YELLOW = range(4)
RGB565, GRAY, BINARY = 0, 1, 2
# ⚠️ 2026-10-31：RK 端**只发 MODE 1**。BINARY 常量保留只为解码 FPGA/其它工具
# 可能发来的旧指令（decode_command），本工程没有任何路径会去请求它。
IMAGE_MODE = GRAY
COLOR_NAMES = ('red', 'green', 'blue', 'yellow')

@dataclass(frozen=True)
class Display:
    left: int | None = None
    right: int | None = None
    straight: int | None = None
    edge_warning: bool = False

    def as_dict(self):
        return {**{k: (COLOR_NAMES[v] if v is not None else 'hidden') for k, v in
                (('left', self.left), ('straight', self.straight), ('right', self.right))},
                'edge_warning': self.edge_warning}


def encode_command(display: Display, image_mode: int = GRAY) -> int:
    """bits 0..3 flags L,R,S,edge; 4..5 L color; 6..7 R; 8..9 S; 10..11 image."""
    if image_mode not in (RGB565, GRAY):
        raise ValueError('image_mode 必须是 0(RGB565) 或 1(gray)；'
                         'RK 端只发灰度（2026-10-31）')
    code = image_mode << 10
    for bit, shift, color in ((0, 4, display.left), (1, 6, display.right), (2, 8, display.straight)):
        if color is not None:
            if color not in (RED, GREEN, BLUE, YELLOW):
                raise ValueError('arrow color out of range')
            code |= (1 << bit) | (color << shift)
    if display.edge_warning:
        code |= (1 << 3)
    return code


def encode_packet(display: Display, image_mode: int = GRAY) -> bytes:
    return encode_command(display, image_mode).to_bytes(2, 'little', signed=False)


def decode_command(raw: bytes):
    if len(raw) != 2:
        raise ValueError('must be 2 bytes')
    code = int.from_bytes(raw, 'little')
    if code & 0xF000:
        raise ValueError('reserved bits must be 0')
    mode = (code >> 10) & 3
    if mode == 3:
        raise ValueError('image mode 3 reserved')
    disp = Display(
        *((((code >> shift) & 3) if code & (1 << bit) else None) for bit, shift in
          ((0, 4), (1, 6), (2, 8))),
        edge_warning=bool(code & 8),
    )
    # Dataclass constructor positional fields are left,right,straight.
    return disp, mode
