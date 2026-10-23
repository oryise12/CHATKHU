"""Cartesian paths (world frame, metres) shared by initial and (future) edited paths.

A path is geometry only: points, tool axis per point, and stop waypoints. Time
is assigned later by trajectory.py, after selection, inside validation.
Min-jerk timing and arc-length bookkeeping follow the ideas of
`path 편집/src/pac_path_editor/core.py` (origin/work/webcam-path-editor@20873db);
the round-trip outbound/return pair is not reused: every path here is one-way
and starts at the robot's actual TCP.
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, field

import numpy as np

from .frames import DOWN, WORLD, Pose, angle_deg, slerp_axis, unit


class PathGenerationError(ValueError):
    pass


@dataclass(frozen=True)
class CartesianPath:
    path_id: str                   # "to_target" | "return_ready" | future edited ids
    source: str                    # "initial" | "edited"
    frame: str
    points_m: np.ndarray           # (N,3)
    tool_axis: np.ndarray          # (N,3) unit vectors
    waypoint_index: tuple          # indices where the robot stops (first and last included)
    start_q_rad: np.ndarray        # (6,) joint state the path was generated from
    meta: dict = field(default_factory=dict)

    @property
    def fingerprint(self) -> str:
        h = hashlib.sha256(f"{self.path_id}|{self.source}|{self.frame}|{self.waypoint_index}".encode())
        for a in (self.points_m, self.tool_axis, self.start_q_rad):
            h.update(np.ascontiguousarray(a, dtype="<f8").tobytes())
        return h.hexdigest()

    @property
    def length_m(self) -> float:
        return float(np.sum(np.linalg.norm(np.diff(self.points_m, axis=0), axis=1)))


def _segment(p0, p1, a0, a1, ds_max: float, rot_len: float) -> tuple[np.ndarray, np.ndarray]:
    """Samples from p0 (excluded) to p1 (included); axis slerps a0->a1 along the segment."""
    metric = np.linalg.norm(p1 - p0) + rot_len * np.radians(angle_deg(a0, a1))
    n = max(1, int(np.ceil(metric / ds_max)))
    u = np.arange(1, n + 1) / n
    pts = p0 + u[:, None] * (p1 - p0)
    axes = np.array([slerp_axis(a0, a1, x) for x in u])
    return pts, axes


def waypoint_path(path_id: str, source: str, start: Pose, start_q, waypoints: list, cfg: dict,
                  allow_unsafe: bool = False, meta: dict | None = None) -> CartesianPath:
    """start -> waypoints[(p, axis)] with straight segments and a stop at every waypoint."""
    pc = cfg["path"]
    if start.frame != WORLD:
        raise PathGenerationError(f"start pose frame {start.frame!r} is not {WORLD!r}")
    pts, axes, wp = [start.p_m.copy()], [unit(start.tool_axis)], [0]
    p_prev, a_prev = start.p_m.copy(), unit(start.tool_axis)
    for p, a in waypoints:
        p, a = np.asarray(p, float), unit(a)
        if np.linalg.norm(p - p_prev) < 1e-9 and angle_deg(a, a_prev) < 1e-6:
            continue
        sp, sa = _segment(p_prev, p, a_prev, a, pc["ds_max_m"], pc["rot_equiv_length_m_per_rad"])
        pts.extend(sp)
        axes.extend(sa)
        wp.append(len(pts) - 1)
        p_prev, a_prev = p, a
    P, A = np.array(pts), np.array(axes)
    if len(P) < 2:
        raise PathGenerationError("path has no motion (start already at goal)")
    if not np.all(np.isfinite(P)):
        raise PathGenerationError("non-finite path coordinates")
    zmin = float(P[:, 2].min())
    if zmin < pc["tcp_z_min_m"] and not allow_unsafe:
        raise PathGenerationError(f"TCP z {zmin:.4f} m below tcp_z_min_m {pc['tcp_z_min_m']} m")
    return CartesianPath(path_id, source, WORLD, P, A, tuple(wp), np.asarray(start_q, float).copy(),
                         dict(meta or {}, allow_unsafe=allow_unsafe))


def lift_then(start: Pose, goal_p, goal_axis, lift_z: float) -> list:
    """If the start is below lift_z, rise vertically first (same xy). Rule documented in docs/FRAMES.md."""
    wps = []
    if start.p_m[2] < lift_z - 1e-9:
        wps.append(([start.p_m[0], start.p_m[1], lift_z], goal_axis))
    wps.append((goal_p, goal_axis))
    return wps


def initial_path(start: Pose, start_q, target, cfg: dict, allow_unsafe: bool = False) -> CartesianPath:
    """Current TCP -> approach point (above marker) -> work point. One-way."""
    wps = lift_then(start, target.approach_m, target.tool_axis, float(target.approach_m[2]))
    wps.append((target.tcp_goal_m, target.tool_axis))
    return waypoint_path("to_target", "initial", start, start_q, wps, cfg, allow_unsafe,
                         meta={"target": target.summary()})


def return_path(start: Pose, start_q, ready: Pose, cfg: dict) -> CartesianPath:
    """Actual current TCP -> (vertical lift to approach height) -> ready TCP. One-way."""
    lift = max(cfg["target"]["approach_height_m"], 0.0)
    wps = lift_then(start, ready.p_m, ready.tool_axis, min(lift, float(ready.p_m[2])))
    return waypoint_path("return_ready", "initial", start, start_q, wps, cfg,
                         meta={"goal": ready.p_m.round(5).tolist()})


def from_points(points_m, frame: str, start_q, path_id: str = "edited", tool_axis=DOWN) -> CartesianPath:
    """Adapter for the next phase (edited paths, e.g. pac_path_editor PathData.points).

    Only world-frame points are accepted (virtual_edit_frame etc. are rejected);
    the editor's xi/times are ignored because timing is always rebuilt here.
    The first point must be the robot's current TCP (checked by validation, not here).
    """
    if frame != WORLD:
        raise PathGenerationError(f"edited path frame {frame!r} is not {WORLD!r}; transform first")
    P = np.asarray(points_m, float)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) < 2:
        raise PathGenerationError("points must be (N>=2, 3)")
    A = np.repeat(unit(tool_axis)[None], len(P), axis=0)
    return CartesianPath(path_id, "edited", WORLD, P, A, (0, len(P) - 1), np.asarray(start_q, float).copy())


def polyline_distance(points, poly) -> np.ndarray:
    """Distance (m) from each point (M,3) to the polyline poly (N,3)."""
    P, A = np.atleast_2d(points), np.asarray(poly, float)
    a, b = A[:-1], A[1:]
    ab = b - a
    L2 = np.maximum(np.sum(ab * ab, axis=1), 1e-18)
    out = np.empty(len(P))
    for i, p in enumerate(P):
        t = np.clip(np.sum((p - a) * ab, axis=1) / L2, 0.0, 1.0)
        out[i] = np.min(np.linalg.norm(a + t[:, None] * ab - p, axis=1))
    return out
