"""Decoded, immutable bus snapshots with host timestamps.

t is time.monotonic() when the read completed. An axis that did not answer is listed in `missing`; a snapshot is
"complete" only when every expected axis answered with a valid packet.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from . import control_table as ct


@dataclass(frozen=True)
class AxisFast:
    profile_acceleration: int
    profile_velocity: int
    goal_position: int
    realtime_tick: int
    moving: int
    moving_status: int
    present_pwm: int
    present_current: int
    present_velocity: int
    present_position: int
    velocity_trajectory: int
    position_trajectory: int
    present_input_voltage: int
    present_temperature: int
    status_error: int = 0

    @property
    def alert(self) -> bool:
        return bool(self.status_error & 0x80)

    @property
    def profile_ongoing(self) -> bool:
        return bool(self.moving_status & 0x02)

    @property
    def following_error(self) -> bool:
        return bool(self.moving_status & 0x08)

    @property
    def voltage_v(self) -> float:
        return self.present_input_voltage / 10.0

    @classmethod
    def from_bytes(cls, data: bytes, status_error: int) -> "AxisFast":
        v = ct.decode_block(ct.FAST[0], data)
        return cls(**{k: v[k] for k in cls.__dataclass_fields__ if k != "status_error"}, status_error=status_error)


@dataclass(frozen=True)
class FastSnapshot:
    seq: int
    t: float
    duration_s: float
    axes: dict[int, AxisFast]
    missing: tuple[int, ...]

    @property
    def complete(self) -> bool:
        return not self.missing

    def __getitem__(self, dxl_id: int) -> AxisFast:
        return self.axes[dxl_id]


@dataclass(frozen=True)
class SlowSnapshot:
    """Settings + torque + HW error + watchdog (block 10..98) per axis as a name -> int dict."""
    seq: int
    t: float
    axes: dict[int, dict[str, int]]
    status_errors: dict[int, int] = field(default_factory=dict)
    missing: tuple[int, ...] = ()

    @property
    def complete(self) -> bool:
        return not self.missing

    def value(self, dxl_id: int, name: str) -> int:
        return self.axes[dxl_id][name]

    def all(self, name: str) -> dict[int, int]:
        return {i: a[name] for i, a in self.axes.items()}

    @classmethod
    def decode(cls, seq: int, t: float, data: dict[int, bytes], errors: dict[int, int], missing) -> "SlowSnapshot":
        return cls(seq, t, {i: ct.decode_block(ct.SLOW[0], d) for i, d in data.items()}, dict(errors), tuple(missing))
