"""Unit conversions. Internally: joint angles in rad, joint speeds in rad/s, motor positions in signed counts.

One count is 1/4096 of a servo output turn; the extra gearbox (g, e.g. 16) sits after the servo, so one joint
turn is 4096*g counts. The servo's own 212.6:1 gear is inside the 4096 counts and is never multiplied in.
Counts are never wrapped with % 4096: Extended Position mode keeps the multi-turn count.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

COUNTS_PER_SERVO_REV = 4096
PV_UNIT_RPM = 0.229            # Profile Velocity / Present Velocity / Velocity Trajectory, velocity-based
PA_UNIT_REV_MIN2 = 214.577     # Profile Acceleration, velocity-based profile
GOAL_MIN, GOAL_MAX = -1_048_575, 1_048_575   # Goal Position range in Extended Position mode (+-256 rev)
INT32_MIN, INT32_MAX = -2**31, 2**31 - 1


def round_half_up(x: float) -> int:
    """Rounding used for every raw value (Python's round() rounds halves to even)."""
    return math.floor(x + 0.5)


def counts_per_rad(gear: float) -> float:
    return COUNTS_PER_SERVO_REV * gear / (2 * math.pi)


def pv_unit_rad_s(gear: float) -> float:
    """Joint speed of one Profile Velocity / Present Velocity raw unit."""
    return PV_UNIT_RPM * 2 * math.pi / 60 / gear


def pa_unit_rad_s2(gear: float) -> float:
    """Joint acceleration of one Profile Acceleration raw unit."""
    return PA_UNIT_REV_MIN2 * 2 * math.pi / 3600 / gear


def pv_counts_s(pv_raw: float) -> float:
    """Motor-side counts/s of a velocity raw value (Profile Velocity, Present Velocity, Velocity Trajectory)."""
    return pv_raw * PV_UNIT_RPM * COUNTS_PER_SERVO_REV / 60


def pa_counts_s2(pa_raw: float) -> float:
    return pa_raw * PA_UNIT_REV_MIN2 * COUNTS_PER_SERVO_REV / 3600


def pv_raw_to_rpm(pv_raw: float) -> float:
    return pv_raw * PV_UNIT_RPM


def max_pv_raw_for_rpm(rpm: float) -> int:
    """Largest raw Profile Velocity not above `rpm` (60 rpm -> 262; 306 would be 70.07 rpm)."""
    return math.floor(rpm / PV_UNIT_RPM + 1e-9)


def goal_in_range(p: int) -> bool:
    return GOAL_MIN <= p <= GOAL_MAX


@dataclass(frozen=True)
class JointMap:
    """One driven joint: URDF joint name, servo ID, extra gear ratio after the servo, sign s = +1/-1.

    s is defined against the raw Present Position as the servo reports it (whatever its Drive Mode reverse bit is):
    s = +1 when the raw count increases while the joint turns in its URDF + direction."""

    name: str
    dxl_id: int
    gear: float
    sign: int

    def __post_init__(self):
        if self.sign not in (1, -1):
            raise ValueError(f"{self.name}: sign must be +1 or -1, got {self.sign!r}")
        if not self.gear > 0:
            raise ValueError(f"{self.name}: gear ratio must be > 0")

    @property
    def k(self) -> float:
        return counts_per_rad(self.gear)

    def q_rad(self, p: int, p_ref: int, q_ref_rad: float) -> float:
        """q = q_ref + s * (P - P_ref) * 2pi / (4096 g)"""
        return q_ref_rad + self.sign * (p - p_ref) / self.k

    def p_goal(self, q_rad: float, p_ref: int, q_ref_rad: float) -> int:
        """P_goal = P_ref + s * (q - q_ref) * 4096 g / 2pi, rounded to the nearest count."""
        return p_ref + round_half_up(self.sign * (q_rad - q_ref_rad) * self.k)

    def dq_rad(self, dp_counts: float) -> float:
        return self.sign * dp_counts / self.k
