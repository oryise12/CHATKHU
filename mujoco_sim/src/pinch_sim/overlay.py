"""Decorations drawn identically into any mjvScene (passive viewer user_scn and Renderer.scene).

One `Overlay` is built from the session + live MjData at a single instant and
added to both scenes, so marker, path and TCP use the same world coordinates.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

EDITED_COLOR = (1.0, 0.55, 0.0, 1)        # (unused since 2026-10-11: edit phase shows heights instead)
HEIGHT_STATES = ("GENERATED", "SELECTED", "VALIDATING")   # edit phase: path coloured by relative height
LOW_COLOR, HIGH_COLOR = np.array([0.1, 0.3, 1.0]), np.array([1.0, 0.1, 0.1])   # blue = low, red = high
REFERENCE_COLOR = (0.75, 0.75, 0.75, 0.45)  # the generated path before edits (thin, translucent)
SELECTED_COLOR = (1.0, 1.0, 1.0, 1)        # point being dragged (white; blue now means "low")
POINTED_COLOR = (1.0, 0.85, 0.0, 1)        # clicked point for voice edits (yellow)
EDITABLE_STATES = ("GENERATED", "SELECTED", "PASSED", "FAILED")   # = session.EDITABLE (no import cycle)
PATH_COLOR = {"GENERATED": (1, 1, 1, 1), "SELECTED": (1, 0.85, 0, 1), "VALIDATING": (1, 0.85, 0, 1),
              "PASSED": (0.1, 0.9, 0.2, 1), "FAILED": (0.95, 0.1, 0.1, 1), "EXECUTING": (0.1, 0.9, 0.2, 1),
              "DONE": (0.4, 0.6, 1.0, 1), "ABORTED": (0.95, 0.4, 0.1, 1), "HOLD": (0.7, 0.7, 0.7, 1)}


@dataclass
class Overlay:
    state: str = "HOLD"
    floor_point: np.ndarray | None = None
    approach: np.ndarray | None = None
    tcp_goal: np.ndarray | None = None
    goal_axis: np.ndarray | None = None
    path: np.ndarray | None = None          # (N,3)
    tcp: np.ndarray | None = None
    failures: list = field(default_factory=list)   # [(xyz, kind)]
    reference: np.ndarray | None = None     # generated path before edits, shown only once edited
    edited: bool = False
    selected: np.ndarray | None = None      # point under drag
    pointed: np.ndarray | None = None       # clicked point that voice "here" refers to (edit phase only)
    height_from: int | None = None          # heights are coloured on path indices [height_from, height_until]
    height_until: int | None = None
    peak: np.ndarray | None = None          # highest point of the coloured part (edit phase only), world m
    stations: list = field(default_factory=list)   # demo stations: [(floor xyz, target xyz, rgba)] (stations.py)


def build_overlay(session, d: mujoco.MjData) -> Overlay:
    ov = Overlay(state=session.state, tcp=d.site_xpos[session.idx.tcp_site].copy())
    if session.target is not None:
        ov.floor_point, ov.approach = session.target.floor_point_m, session.target.approach_m
    if session.goal is not None:
        ov.tcp_goal, ov.goal_axis = session.goal.p_m, session.goal.tool_axis
    if session.path is not None:
        ov.path = session.path.points_m
        ov.edited = session.path.source == "edited"
        ov.height_from, ov.height_until = height_range(session.path.points_m, session.path.waypoint_index)
        if ov.state in HEIGHT_STATES:
            ov.peak = peak_point(session.path.points_m, ov.height_from, ov.height_until)
    if getattr(session, "stations", None):
        from .stations import RGBA
        ov.stations = [(s.floor_m, s.tcp_goal_m, RGBA.get(n, (0.7, 0.7, 0.7, 1))) for n, s in session.stations.items()
                       if s.floor_m is not None]
    k = session.pointed_index() if session.state in EDITABLE_STATES else None
    if k is not None:
        ov.pointed = session.path.points_m[k].copy()
    if session.edit is not None and session.edit.edited:
        ov.reference = session.edit.base.points_m
        ov.selected = session.edit.selected_point()
    # where the last validation failed: kept by the session (not read from the newest event, which a click or a
    # cancelled booking right after the failure would hide) while the user fixes the path
    if session.state in ("FAILED", "GENERATED", "SELECTED"):
        ov.failures = list(getattr(session, "last_failures", []))
    return ov


def _vertical(points, i0: int, i1: int, tol: float = 1e-3) -> bool:
    """Segment between two stops is a pure vertical move (lift or descent): no x, y change."""
    return float(np.linalg.norm(points[i1, :2] - points[i0, :2])) < tol


def height_range(points, waypoint_index) -> tuple[int, int]:
    """Path indices [from, until] that get height colours: the transit between the waypoints.

    A vertical lift at the start (start -> lift waypoint, e.g. after picking an object) and a
    vertical descent at the end (approach waypoint -> work point) are excluded, so they stay white.
    Height-only edits keep x, y, so the classification does not change while editing.
    """
    wp = list(waypoint_index)
    lo, hi = 0, len(wp) - 1
    if hi - lo >= 2 and _vertical(points, wp[lo], wp[lo + 1]):
        lo += 1
    if hi - lo >= 2 and _vertical(points, wp[hi - 1], wp[hi]):
        hi -= 1
    return wp[lo], wp[hi]


def peak_point(points, a0: int, a1: int) -> np.ndarray:
    """Highest vertex (world m) of the height-coloured part points[a0..a1]."""
    seg = np.asarray(points)[a0:a1 + 1]
    return seg[int(np.argmax(seg[:, 2]))].copy()


def peak_text(ov: "Overlay") -> str:
    return "" if ov.peak is None else f"peak z = {ov.peak[2]:.3f} m"


def height_until(waypoint_index) -> int:
    """(kept for callers without points) the stop before the last one, or the end."""
    wp = list(waypoint_index)
    return wp[-2] if len(wp) >= 3 else wp[-1]


def height_colors(z: np.ndarray) -> np.ndarray:
    """(N,4) RGBA, blue at the lowest and red at the highest of these heights (relative)."""
    z = np.asarray(z, float)
    span = float(z.max() - z.min())
    u = (z - z.min()) / span if span > 1e-4 else np.full(len(z), 0.5)
    rgb = LOW_COLOR[None] * (1 - u[:, None]) + HIGH_COLOR[None] * u[:, None]
    return np.column_stack([rgb, np.ones(len(z))])


def segment_colors(ov: "Overlay", idx: np.ndarray) -> list:
    """Colour for each drawn segment idx[i] -> idx[i+1]."""
    base = PATH_COLOR.get(ov.state, (1, 1, 1, 1))
    if ov.state not in HEIGHT_STATES or ov.height_until is None:
        return [base] * (len(idx) - 1)
    start = ov.height_from or 0
    end = ov.height_until
    mids = np.array([(ov.path[a, 2] + ov.path[b, 2]) / 2 for a, b in zip(idx[:-1], idx[1:])])
    inside = np.array([a >= start and b <= end for a, b in zip(idx[:-1], idx[1:])])
    out = [PATH_COLOR["GENERATED"]] * (len(idx) - 1)      # vertical lift / descent: plain white
    if inside.any():
        cols = height_colors(mids[inside])
        for k, c in zip(np.flatnonzero(inside), cols):
            out[k] = tuple(c)
    return out


def _geom(scn: mujoco.MjvScene, gtype, size, pos, rgba, mat=None) -> mujoco.MjvGeom | None:
    if scn.ngeom >= scn.maxgeom:
        return None
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, float), np.asarray(pos, float),
                        (np.eye(3) if mat is None else np.asarray(mat)).reshape(9), np.asarray(rgba, np.float32))
    g.category = mujoco.mjtCatBit.mjCAT_DECOR
    scn.ngeom += 1
    return g


def _segment(scn, a, b, width, rgba, gtype=mujoco.mjtGeom.mjGEOM_CAPSULE):
    g = _geom(scn, gtype, [0, 0, 0], [0, 0, 0], rgba)
    if g is not None:
        mujoco.mjv_connector(g, gtype, width, np.asarray(a, float), np.asarray(b, float))


def add_overlay(scn: mujoco.MjvScene, ov: Overlay, max_segments: int = 120) -> None:
    for floor, goal, rgba in ov.stations:       # demo stations: a disc on the marker and its target point above it
        _geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.025, 0.002, 0], floor + [0, 0, 0.002], rgba)
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.008, 0, 0], goal, rgba)
        _segment(scn, floor, goal, 0.0015, (*rgba[:3], 0.5))
    if ov.floor_point is not None:
        _geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.022, 0.002, 0], ov.floor_point + [0, 0, 0.002],
              (1.0, 0.1, 0.6, 1))
        _segment(scn, ov.floor_point, ov.approach, 0.002, (1.0, 0.1, 0.6, 0.5))   # marker -> approach guide
    if ov.approach is not None:
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.010, 0, 0], ov.approach, (0.1, 0.9, 0.9, 1))
    if ov.tcp_goal is not None:
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.012, 0, 0], ov.tcp_goal, (1.0, 0.9, 0.1, 1))
        if ov.goal_axis is not None:
            g = _geom(scn, mujoco.mjtGeom.mjGEOM_ARROW, [0, 0, 0], [0, 0, 0], (1.0, 0.9, 0.1, 1))
            if g is not None:
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.004, ov.tcp_goal,
                                     ov.tcp_goal + 0.04 * np.asarray(ov.goal_axis))
    if ov.reference is not None and len(ov.reference) > 1:
        idx = np.unique(np.linspace(0, len(ov.reference) - 1, min(len(ov.reference), max_segments // 2 + 1)).astype(int))
        for i0, i1 in zip(idx[:-1], idx[1:]):
            _segment(scn, ov.reference[i0], ov.reference[i1], 0.002, REFERENCE_COLOR)
    if ov.path is not None and len(ov.path) > 1:
        idx = np.linspace(0, len(ov.path) - 1, min(len(ov.path), max_segments + 1)).astype(int)
        if ov.height_until is not None:
            a0 = ov.height_from or 0
            idx = np.union1d(idx, [a0, ov.height_until])   # colour boundaries exactly at the waypoints
            if ov.state in HEIGHT_STATES:                # keep the true highest/lowest vertices as boundaries
                z = ov.path[a0:ov.height_until + 1, 2]
                idx = np.union1d(idx, [a0 + int(np.argmax(z)), a0 + int(np.argmin(z))])
        idx = np.unique(idx)
        for (i0, i1), rgba in zip(zip(idx[:-1], idx[1:]), segment_colors(ov, idx)):
            _segment(scn, ov.path[i0], ov.path[i1], 0.005, rgba)
    if ov.peak is not None:                      # highest point: small red ball with its height as a label
        g = _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.006, 0, 0], ov.peak, (1.0, 0.1, 0.1, 1))
        if g is not None:
            g.label = f"z = {ov.peak[2]:.3f} m"
    if ov.selected is not None:
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.011, 0, 0], ov.selected, SELECTED_COLOR)
    if ov.pointed is not None and ov.selected is None:
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.010, 0, 0], ov.pointed, POINTED_COLOR)
    for p, _kind in ov.failures:
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.012, 0, 0], p, (1.0, 0.0, 0.0, 0.8))
    if ov.tcp is not None:
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.008, 0, 0], ov.tcp, (1.0, 0.5, 0.0, 1))


def project(m: mujoco.MjModel, d: mujoco.MjData, cam: str, points, width: int, height: int) -> np.ndarray:
    """Pinhole projection of world points (N,3) into pixel (u right, v down) for a fixed camera.

    MuJoCo camera frame: x right, y up, looking along -z; vertical field of view fovy.
    """
    c = m.camera(cam).id
    pos, R = d.cam_xpos[c], d.cam_xmat[c].reshape(3, 3)
    f = 0.5 * height / np.tan(np.radians(m.cam_fovy[c]) / 2)
    pc = (np.atleast_2d(points) - pos) @ R          # world -> camera coordinates
    u = width / 2 + f * pc[:, 0] / -pc[:, 2]
    v = height / 2 - f * pc[:, 1] / -pc[:, 2]
    return np.column_stack([u, v])
