"""Per-side hysteresis for FPGA arrow decisions.

Dashed line -> YELLOW immediately (lane change permitted).
Solid line  -> RED after confirm_frames consecutive VALID detections (lane change forbidden).
Unknown/absent detections reset the solid streak but preserve display state.
State belongs to the current live Python process; after os.execv it resets.
"""
from dataclasses import dataclass
from protocol import RED, YELLOW


@dataclass
class LaneSolidStreaks:
    confirm_frames: int = 10
    left: int = 0
    right: int = 0

    def __post_init__(self):
        if type(self.confirm_frames) is not int or self.confirm_frames < 1:
            raise ValueError('confirm_frames must be a positive integer')

    def reset(self):
        self.left = 0
        self.right = 0

    def preview_colour(self, side, kind):
        """这一帧的 kind **最终**会变成什么颜色（不推进计数）。

        用于判断"如果要按住，按住的是不是和屏幕一样的颜色"：
        同一个颜色按住了没有损失；想变成别的颜色就不该被按住。
        - dash -> YELLOW（立即）
        - solid -> RED（只要连续够了；不够就是"还没定"，返回 None）
        - none -> None（没有新证据）
        """
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if kind == 'dash':
            return YELLOW
        if kind == 'solid':
            streak = min(self.confirm_frames, getattr(self, side) + 1)
            return RED if streak >= self.confirm_frames else None
        return None

    def update_side(self, side, kind, previous_color):
        """Return (arrow color, update label, current side streak).

        The caller MUST call once per successfully inferred image pair.
        On interrupted acquisition/inference or other branch, call reset().
        """
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if kind not in ('solid', 'dash', 'none'):
            raise ValueError('kind must be solid, dash, or none')
        if kind == 'dash':
            setattr(self, side, 0)
            return YELLOW, 'dash_yellow_immediate', 0
        if kind == 'none':
            setattr(self, side, 0)
            return previous_color, 'retained_no_line', 0
        streak = min(self.confirm_frames, getattr(self, side) + 1)
        setattr(self, side, streak)
        if streak == self.confirm_frames:
            return RED, 'solid_red_confirmed', streak
        return previous_color, f'solid_pending_{streak}/{self.confirm_frames}', streak
