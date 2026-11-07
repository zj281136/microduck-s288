"""Terminal key commands shared by S288 V0 MuJoCo and real deployment."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty
from dataclasses import dataclass, field


HELP = (
    "W/S forward/back at ±0.20 m/s; "
    "A/D left/right turn at ±2.00 rad/s; "
    "opposite key stops the axis first, then reverses on the next press; "
    "5/2 head pitch ±0.10 rad; 1/3 head yaw ±0.20 rad; "
    "Q/E head roll ±0.10 rad; X stop walking; 0 center head; "
    "Space zero all; Ctrl+C quit."
)


@dataclass
class KeyboardCommands:
    vx: float = 0.0
    wz: float = 0.0
    head: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    @property
    def vy(self) -> float:
        """This deployment intentionally never commands lateral velocity."""
        return 0.0

    def apply(self, key: str) -> bool:
        """Apply one key; head order is neck pitch, pitch, yaw, roll."""
        key = key.lower()
        if key == "w":
            self.vx = 0.0 if self.vx < 0.0 else 0.20
        elif key == "s":
            self.vx = 0.0 if self.vx > 0.0 else -0.20
        elif key == "q":
            self.head[3] = min(0.20, round(self.head[3] + 0.10, 4))
        elif key == "e":
            self.head[3] = max(-0.20, round(self.head[3] - 0.10, 4))
        elif key == "a":
            self.wz = 0.0 if self.wz < 0.0 else 2.0
        elif key == "d":
            self.wz = 0.0 if self.wz > 0.0 else -2.0
        elif key == "5":
            self.head[1] = min(0.50, round(self.head[1] + 0.10, 4))
        elif key == "2":
            self.head[1] = max(-0.50, round(self.head[1] - 0.10, 4))
        elif key == "1":
            self.head[2] = min(0.70, round(self.head[2] + 0.20, 4))
        elif key == "3":
            self.head[2] = max(-0.70, round(self.head[2] - 0.20, 4))
        elif key == "x":
            self.vx = self.wz = 0.0
        elif key == "0":
            self.head[:] = [0.0] * 4
        elif key == " ":
            self.vx = self.wz = 0.0
            self.head[:] = [0.0] * 4
        else:
            return False
        return True

    def summary(self) -> str:
        return (f"vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f} "
                f"head(pitch/yaw/roll)={self.head[1]:+.2f}/{self.head[2]:+.2f}/{self.head[3]:+.2f}")


class TerminalKeyboard:
    """Nonblocking single-key reader; cbreak preserves Ctrl+C."""

    def __init__(self, *, enable_head: bool = True) -> None:
        self.enable_head = enable_head
        self.commands = KeyboardCommands()
        self._fd: int | None = None
        self._saved = None

    def start(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard deployment needs a terminal on stdin")
        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        help_text = HELP if self.enable_head else (
            "W/S forward/back at ±0.20 m/s; A/D turn at ±2.00 rad/s; "
            "X stop walking; Space zero commands; Ctrl+C quit. "
            "All four head commands fixed at zero; head keys disabled."
        )
        print("\n" + help_text, flush=True)

    def poll(self) -> KeyboardCommands:
        if not self.enable_head:
            self.commands.head[:] = [0.0] * 4
        if self._fd is None:
            return self.commands
        changed = False
        while select.select([self._fd], [], [], 0)[0]:
            data = os.read(self._fd, 1)
            if not data:
                break
            key = data.decode(errors="ignore")
            if not self.enable_head and key.lower() in {"5", "2", "1", "3", "q", "e", "0"}:
                continue
            changed |= self.commands.apply(key)
        if changed:
            print("\r" + self.commands.summary() + "   ", flush=True)
        return self.commands

    def close(self) -> None:
        if self._fd is not None and self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        self._fd = None
        self._saved = None
