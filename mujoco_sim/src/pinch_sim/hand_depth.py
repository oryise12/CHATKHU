"""Pinch position in 3-D from a depth camera (Astra), for moving a grabbed path point by the hand's own motion.

MediaPipe gives the hand only in the colour image (its z is relative to the wrist, not metric), so the depth comes
from the depth camera:

  1. depth of the pinching fingers: thumb IP / tip and index PIP / DIP / tip, each the low percentile of a small
     depth patch (the finger is the nearest surface there; the median of a patch on a thin finger can be the desk
     behind it), then the median of those and the ones far from it dropped;
  2. the 3-D point = the line of sight of the index fingertip at that depth (not the thumb-index midpoint: the
     thumb leaves when the pinch opens, the index tip stays where the point was put), in the
     simulator world (through the camera model: pac_vision's RGB->depth registration for a real calibration,
     a registered pinhole for the mock / tests);
  3. a filter: frames that jump more than `max_jump_m` are dropped (a stray depth), the rest go through a
     One Euro filter (strong smoothing while the hand rests - no jitter - and little lag while it moves, and
     no offset once it stops, so the motion stays 1:1).

Only the motion since the pinch closed is used (relative), so a constant depth bias cancels.

The camera model must offer depth_z(u, v, depth_mm, half, percentile) -> optical Z (m) or None, and
point_at_z(u, v, z) -> world point (PacVisionCamera, CameraModel).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

THUMB_IP, THUMB_TIP, INDEX_PIP, INDEX_DIP, INDEX_TIP = 3, 4, 6, 7, 8
FINGER_POINTS = (THUMB_IP, THUMB_TIP, INDEX_PIP, INDEX_DIP, INDEX_TIP)


@dataclass
class DepthConfig:
    half: int = 3                 # depth patch (2*half+1)^2 pixels around each finger point
    percentile: float = 25.0      # of the valid depths in the patch: the finger, not the desk behind it
    agree_m: float = 0.03         # finger points farther than this from the median depth are dropped
    min_points: int = 2
    max_jump_m: float = 0.08      # one frame moving more than this = a stray depth, dropped ...
    jump_accept_s: float = 0.3    # ... unless it stays there this long (the hand really is there)
    min_cutoff_hz: float = 0.3    # One Euro: smoothing of a resting hand (lower = calmer)
    beta: float = 6.0             # ... cutoff grows with speed (m/s) so a moving hand lags little
    d_cutoff_hz: float = 1.0
    start_m: float = 0.005        # a pinch must move this much before the path changes (a still pinch = select)
    max_move_m: float = 0.15      # one pinch moves a point at most this far (same as one voice edit)


def patch_depth(depth_mm: np.ndarray, u: float, v: float, half: int, percentile: float) -> float:
    """Low percentile of the valid (non-zero) depths in the patch around pixel (u, v), mm; 0 when none."""
    h, w = depth_mm.shape[:2]
    iu, iv = int(round(u)), int(round(v))
    if not (0 <= iu < w and 0 <= iv < h):
        return 0.0
    patch = depth_mm[max(0, iv - half):iv + half + 1, max(0, iu - half):iu + half + 1].astype(float)
    vals = patch[patch > 0]
    if vals.size < max(3, patch.size // 4):
        return 0.0
    return float(np.percentile(vals, percentile))


class HandDepth:
    """Landmarks (pixels) + depth frame -> filtered world point of the pinch."""

    def __init__(self, cam, cfg: DepthConfig | None = None):
        self.cam, self.cfg = cam, cfg or DepthConfig()
        self.reset()

    def reset(self) -> None:
        self.raw: np.ndarray | None = None        # last measurement (world), None = not measured
        self.out: np.ndarray | None = None        # filtered output
        self._dx = np.zeros(3)
        self._t: float | None = None
        self._jump_since: float | None = None
        self.z_m: float | None = None             # depth of the fingers (m) for the display
        self.reason = ""                          # why the last frame had no 3-D point

    def measure(self, lm_px: np.ndarray, depth_mm: np.ndarray | None) -> np.ndarray | None:
        """One frame: world point of the index fingertip at the fingers' depth, or None (self.reason says why)."""
        cfg = self.cfg
        if depth_mm is None:
            self.reason = "no_depth_frame"
            return None
        zs = [self.cam.depth_z(lm_px[i, 0], lm_px[i, 1], depth_mm, cfg.half, cfg.percentile) for i in FINGER_POINTS]
        zs = np.array([z for z in zs if z is not None and np.isfinite(z)])
        if zs.size < cfg.min_points:
            self.reason = "no_depth_at_fingers"
            return None
        med = float(np.median(zs))
        zs = zs[np.abs(zs - med) <= cfg.agree_m]
        if zs.size < cfg.min_points:
            self.reason = "fingers_disagree"
            return None
        z = float(np.median(zs))
        tip = lm_px[INDEX_TIP]
        p = self.cam.point_at_z(float(tip[0]), float(tip[1]), z)
        if p is None or not np.all(np.isfinite(p)):
            self.reason = "no_point"
            return None
        self.z_m, self.reason = z, ""
        return np.asarray(p, float)

    def update(self, lm_px: np.ndarray | None, depth_mm: np.ndarray | None, now: float) -> np.ndarray | None:
        """Measure and filter. None until the first good frame; afterwards a missing frame keeps the last output."""
        cfg = self.cfg
        p = None if lm_px is None else self.measure(np.asarray(lm_px, float), depth_mm)
        self.raw = p
        if p is None:
            return self.out
        if self.out is None:
            self.out, self._dx, self._t = p.copy(), np.zeros(3), now
            return self.out
        if np.linalg.norm(p - self.out) > cfg.max_jump_m:
            if self._jump_since is None:
                self._jump_since = now
            if now - self._jump_since < cfg.jump_accept_s:
                return self.out                   # stray depth: ignore this frame
            self.out, self._dx = p.copy(), np.zeros(3)   # it stayed there: the hand really moved that fast
            self._jump_since, self._t = None, now
            return self.out
        self._jump_since = None
        dt = max(1e-3, now - self._t)
        self._t = now

        def alpha(cutoff):
            return 1.0 / (1.0 + 1.0 / (2 * np.pi * cutoff * dt))
        self._dx = self._dx + alpha(cfg.d_cutoff_hz) * ((p - self.out) / dt - self._dx)
        cutoff = cfg.min_cutoff_hz + cfg.beta * float(np.linalg.norm(self._dx))
        self.out = self.out + alpha(cutoff) * (p - self.out)
        return self.out


class AstraDepthSource:
    """Astra depth frames (OpenNI2 through pac_vision) in a background thread; latest() = (depth_mm, t)."""

    def __init__(self, calib_path):
        from pathlib import Path

        from .pac_calib import _import_pac_vision
        calib_path = Path(calib_path)
        _import_pac_vision(calib_path)
        from pac_vision.astra_depth import AstraDepth
        from pac_vision.config import load_config
        cfg = load_config(calib_path.resolve().parent)            # pac_vision/config (camera.yaml + local overlays)
        self._dev = AstraDepth.from_config(cfg["camera"]).open()
        self._lock = threading.Lock()
        self._latest: tuple[np.ndarray | None, float] = (None, 0.0)
        self._stop = False
        threading.Thread(target=self._run, name="astra-depth", daemon=True).start()

    def _run(self) -> None:
        while not self._stop:
            try:
                frame = self._dev.read()
            except Exception:          # noqa: BLE001 - a dropped frame; keep the last one
                time.sleep(0.01)
                continue
            with self._lock:
                self._latest = (frame, time.monotonic())

    def latest(self) -> tuple[np.ndarray | None, float]:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop = True
        time.sleep(0.05)
        self._dev.close()
