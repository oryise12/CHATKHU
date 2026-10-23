"""Frame and unit conventions (see docs/FRAMES.md).

world == base_link: the base mesh sits in the MuJoCo worldbody with identity
pose, so no transform is applied between them. Every point is in metres,
angles in radians, time in seconds. "Front" is +Y; theta is measured from +Y
towards +X:  x = r sin(theta), y = r cos(theta).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

WORLD = "world"
DOWN = np.array([0.0, 0.0, -1.0])
JOINT_AXIS_J1_OFFSET_M = np.array([0.0, 0.002984, 0.0])  # J1 axis vs base_link origin


@dataclass(frozen=True)
class Pose:
    """TCP pose in `frame`: p_m (3,), R (3,3) columns = site x/y/z axes. tool_axis = R[:, 2]."""

    p_m: np.ndarray
    R: np.ndarray
    frame: str = WORLD

    @property
    def tool_axis(self) -> np.ndarray:
        return self.R[:, 2].copy()


@dataclass(frozen=True)
class RobotState:
    """Snapshot of the live simulation: joint order Revolute_1..6."""

    t_s: float
    q_rad: np.ndarray
    qd_rad_s: np.ndarray
    tcp: Pose
    units: dict = field(default_factory=lambda: {"q": "rad", "qd": "rad/s", "t": "s", "p": "m"})


def polar_to_floor(r_m: float, theta_rad: float) -> np.ndarray:
    """Floor point (z = 0) in world for radius r around base_link origin, theta from +Y."""
    return np.array([r_m * np.sin(theta_rad), r_m * np.cos(theta_rad), 0.0])


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError("zero or non-finite vector")
    return v / n


def angle_deg(a, b) -> float:
    return float(np.degrees(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1.0, 1.0))))


def slerp_axis(a, b, u: float) -> np.ndarray:
    """Spherical interpolation between unit vectors a->b at u in [0,1]."""
    a, b = unit(a), unit(b)
    c = float(np.clip(a @ b, -1.0, 1.0))
    w = np.arccos(c)
    if w < 1e-9:
        return a
    if np.pi - w < 1e-6:
        raise ValueError("antiparallel axes have no unique interpolation")
    return unit((np.sin((1 - u) * w) * a + np.sin(u * w) * b) / np.sin(w))


def look_at_quat(pos, lookat, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """MuJoCo camera quaternion (w,x,y,z): camera looks along its -Z, +Y is image up."""
    f = unit(np.asarray(lookat, float) - np.asarray(pos, float))
    right = unit(np.cross(f, up))
    upc = np.cross(right, f)
    R = np.column_stack([right, upc, -f])
    return mat_to_quat(R)


def mat_to_quat(R) -> np.ndarray:
    import mujoco

    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def zaxis_quat(z) -> np.ndarray:
    """Quaternion of a frame whose z-axis is `z` (x chosen deterministically, right-handed)."""
    z = unit(z)
    ref = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = unit(np.cross(ref, z))
    y = np.cross(z, x)
    return mat_to_quat(np.column_stack([x, y, z]))
