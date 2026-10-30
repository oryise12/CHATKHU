"""EE target gizmo in the SIM window: three arrows (move along X / Y / Z) and three rings (rotate about X / Y / Z)
around the EE target. Press on one handle and drag: only that one axis changes (one handle per drag). Empty space
keeps the normal camera mouse. The axes are the base axes or the tool's own axes (frame "base" / "tool").

Pure math and drawing; viz.py feeds it the mouse. Picking is in screen space (distance from the cursor to the
projected handle). The projection matches mjv_select (checked in tests). Pixel coordinates: origin top-left (GLFW
cursor), window size in the same units. The target point sits inside the gripper, so the gizmo is drawn pulled
halfway toward the camera and shrunk by the same factor: the same picture on screen, but in front of the arm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

HANDLES = ("x", "y", "z", "rx", "ry", "rz")
AXIS_RGB = {"x": (0.90, 0.15, 0.15), "y": (0.15, 0.75, 0.20), "z": (0.20, 0.40, 0.95)}
RING_POINTS = 48
PICK_PX = 10.0
ARROW = (0.2, 1.0)       # arrow from / to, in units of the gizmo scale
RING_R = 1.25            # rings outside the arrow tips, so an arrow and a ring never cross
PULL = 0.5               # drawn at this fraction of the camera distance (same screen image, in front of the arm)
MIN_AXIS_VIEW = 0.15     # |cos| between a ring axis and the view ray below which the ring is edge-on (not grabbed)


@dataclass(frozen=True)
class ScreenCam:
    pos: np.ndarray
    forward: np.ndarray
    up: np.ndarray
    right: np.ndarray
    center: float
    halfwidth: float
    bottom: float
    top: float
    near: float
    width: float
    height: float

    @classmethod
    def from_scene(cls, scn: mujoco.MjvScene, width: float, height: float) -> "ScreenCam":
        c0, c1 = scn.camera[0], scn.camera[1]
        fwd, up = np.array(c0.forward, float), np.array(c0.up, float)
        hw = c0.frustum_width or 0.5 * width / height * (c0.frustum_top - c0.frustum_bottom)
        return cls(0.5 * (np.array(c0.pos, float) + np.array(c1.pos, float)), fwd, up, np.cross(fwd, up),
                   float(c0.frustum_center), float(hw), float(c0.frustum_bottom), float(c0.frustum_top),
                   float(c0.frustum_near), float(width), float(height))

    def project(self, w) -> tuple[float, float, float]:
        """(px, py, depth); depth <= 0: behind the camera."""
        v = np.asarray(w, float) - self.pos
        z = float(v @ self.forward)
        if z <= 1e-9:
            return math.nan, math.nan, z
        xn, yn = (v @ self.right) * self.near / z, (v @ self.up) * self.near / z
        px = (xn - (self.center - self.halfwidth)) / (2 * self.halfwidth) * self.width
        py = (1 - (yn - self.bottom) / (self.top - self.bottom)) * self.height
        return float(px), float(py), z

    def ray(self, px: float, py: float) -> tuple[np.ndarray, np.ndarray]:
        xn = self.center - self.halfwidth + px / self.width * 2 * self.halfwidth
        yn = self.bottom + (1 - py / self.height) * (self.top - self.bottom)
        d = self.forward * self.near + self.right * xn + self.up * yn
        return self.pos.copy(), d / np.linalg.norm(d)


def frame_axes(R, frame: str) -> np.ndarray:
    """Columns = the gizmo's X, Y, Z axes in world coordinates."""
    return np.asarray(R, float).copy() if frame == "tool" else np.eye(3)


def axis_rotation(axis, angle: float) -> np.ndarray:
    q, m = np.zeros(4), np.zeros(9)
    mujoco.mju_axisAngle2Quat(q, np.asarray(axis, float), float(angle))
    mujoco.mju_quat2Mat(m, q)
    return m.reshape(3, 3)


def _seg_dist(p, a, b) -> float:
    ab, ap = b - a, p - a
    t = 0.0 if not ab.any() else min(1.0, max(0.0, float(ap @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(ap - t * ab))


class Gizmo:
    def __init__(self, scale_per_m: float = 0.07):
        self.scale_per_m = scale_per_m       # arrow length = this x camera distance to the target
        self.active: str | None = None
        self.hot: str | None = None
        self._d: dict = {}

    def scale(self, cam: ScreenCam, pos) -> float:
        return self.scale_per_m * max(0.2, float((np.asarray(pos) - cam.pos) @ cam.forward))

    def polylines(self, pos, axes: np.ndarray, scale: float) -> dict[str, np.ndarray]:
        pos = np.asarray(pos, float)
        out = {}
        for k, name in enumerate("xyz"):
            a = axes[:, k]
            out[name] = np.array([pos + ARROW[0] * scale * a, pos + ARROW[1] * scale * a])
            e1, e2 = axes[:, (k + 1) % 3], axes[:, (k + 2) % 3]
            th = np.linspace(0, 2 * math.pi, RING_POINTS + 1)
            out["r" + name] = pos + RING_R * scale * (np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2))
        return out

    def pick(self, cam: ScreenCam, px: float, py: float, pos, axes: np.ndarray, tol_px: float = PICK_PX) -> str | None:
        p = np.array([px, py])
        best, best_d = None, tol_px
        _, ray_d = cam.ray(px, py)
        for name, pts in self.polylines(pos, axes, self.scale(cam, pos)).items():
            if name[0] == "r" and abs(float(axes[:, "xyz".index(name[1])] @ ray_d)) < MIN_AXIS_VIEW:
                continue                                   # edge-on ring: its angle cannot be read from the cursor
            scr = [cam.project(w) for w in pts]
            for (x0, y0, z0), (x1, y1, z1) in zip(scr, scr[1:]):
                if z0 <= 0 or z1 <= 0:
                    continue
                dist = _seg_dist(p, np.array([x0, y0]), np.array([x1, y1]))
                if dist < best_d:
                    best, best_d = name, dist
        return best

    # ---------------------------------------------------------------- dragging one handle
    def begin(self, name: str, cam: ScreenCam, px: float, py: float, pos, R, axes: np.ndarray) -> bool:
        a = axes[:, "xyz".index(name[-1])].copy()
        pos, R = np.asarray(pos, float).copy(), np.asarray(R, float).copy()
        o, d = cam.ray(px, py)
        if name[0] == "r":
            th = self._ring_angle(o, d, pos, a)
            if th is None:
                return False
            self._d = {"a": a, "pos": pos, "R": R, "th": th, "total": 0.0}
        else:
            t = self._axis_param(o, d, pos, a)
            if t is None:
                return False
            self._d = {"a": a, "pos": pos, "R": R, "t0": t}
        self.active = name
        return True

    def drag(self, cam: ScreenCam, px: float, py: float):
        """(pos, R) of the target for the cursor now; None if this cursor position gives no reading."""
        if self.active is None:
            return None
        g = self._d
        o, d = cam.ray(px, py)
        if self.active[0] == "r":
            th = self._ring_angle(o, d, g["pos"], g["a"])
            if th is None:
                return None
            g["total"] += (th - g["th"] + math.pi) % (2 * math.pi) - math.pi
            g["th"] = th
            return g["pos"].copy(), axis_rotation(g["a"], g["total"]) @ g["R"]
        t = self._axis_param(o, d, g["pos"], g["a"])
        if t is None:
            return None
        return g["pos"] + (t - g["t0"]) * g["a"], g["R"].copy()

    def end(self) -> None:
        self.active, self._d = None, {}

    @staticmethod
    def _axis_param(o, d, p0, a) -> float | None:
        """Parameter t of the point on the line p0 + t a closest to the ray o + s d."""
        w = p0 - o
        b = float(a @ d)
        denom = 1.0 - b * b
        if denom < 0.02:                                   # axis nearly along the view ray
            return None
        return (b * float(w @ d) - float(w @ a)) / denom

    @staticmethod
    def _ring_angle(o, d, c, a) -> float | None:
        dn = float(d @ a)
        if abs(dn) < 0.05:
            return None
        q = o + float((c - o) @ a) / dn * d
        e1 = np.cross(a, [1.0, 0, 0] if abs(a[0]) < 0.9 else [0, 1.0, 0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(a, e1)
        v = q - c
        return math.atan2(float(v @ e2), float(v @ e1))

    # ---------------------------------------------------------------- drawing
    def draw(self, scn: mujoco.MjvScene, cam: ScreenCam, pos, axes: np.ndarray, ok: bool = True) -> None:
        scale = self.scale(cam, pos)
        near = lambda w: cam.pos + PULL * (np.asarray(w, float) - cam.pos)       # same pixel, nearer the camera
        for name, pts in self.polylines(pos, axes, scale).items():
            on = name in (self.active, self.hot)
            rgb = (1.0, 0.85, 0.1) if name == self.active else AXIS_RGB[name[-1]]
            rgba = np.array([*rgb, 1.0 if on or self.active is None else 0.35], np.float32)
            if name[0] != "r":
                self._add(scn, mujoco.mjtGeom.mjGEOM_ARROW, PULL * (0.16 if on else 0.11) * scale, near(pts[0]),
                          near(pts[1]), rgba)
            else:
                for p0, p1 in zip(pts, pts[1:]):
                    self._add(scn, mujoco.mjtGeom.mjGEOM_LINE, 5.0 if on else 2.5, near(p0), near(p1), rgba)
        g = self._geom(scn)
        if g is not None:
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([PULL * 0.1 * scale, 0, 0]), near(pos),
                                np.eye(3).reshape(-1), np.array([1.0, 0.85, 0.1, 0.9] if ok else [1.0, 0.15, 0.1, 0.9],
                                                                np.float32))
            scn.ngeom += 1

    @staticmethod
    def _geom(scn):
        return scn.geoms[scn.ngeom] if scn.ngeom < scn.maxgeom else None

    def _add(self, scn, gtype, width, p0, p1, rgba) -> None:
        g = self._geom(scn)
        if g is None:
            return
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        if gtype == mujoco.mjtGeom.mjGEOM_ARROW:
            p1 = p0 + 2.0 * (p1 - p0)        # MuJoCo 3.15 draws a connector arrow at half its length (measured)
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.zeros(3), np.zeros(3), np.eye(3).reshape(-1), rgba)
        mujoco.mjv_connector(g, gtype, float(width), p0, p1)
        scn.ngeom += 1
