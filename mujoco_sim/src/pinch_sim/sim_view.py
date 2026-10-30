"""Small 3-D simulator view inside the browser monitor (--ui web), in place of the MuJoCo GLFW window (2026-10-29).

The GLFW window could not be placed next to the monitor (Wayland ignores window positions), so with the web monitor
the scene is rendered offscreen (EGL) and streamed to a small panel at the bottom right of the page (webui.py GET
/sim). Dragging on that panel turns the free camera around the middle of the current path (bounding-box centre).

SimView also stands in for the passive viewer in the app's main loop: is_running / close / lock / sync / set_texts /
cam / user_scn behave like mujoco.viewer's handle, so the loop is the same for both monitors.
"""
from __future__ import annotations

import threading

import mujoco
import numpy as np

from .overlay import add_overlay

DEG_PER_PX = 0.4                 # drag: page pixels -> degrees of azimuth / elevation
ELEVATION_RANGE = (-89.0, -3.0)  # from straight above to just above the desk (never from under it)


class SimView:
    def __init__(self, m: mujoco.MjModel, width: int = 480, height: int = 360, fps: float = 15.0) -> None:
        self.renderer = mujoco.Renderer(m, height, width)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.user_scn = mujoco.MjvScene(m, maxgeom=0)    # the loop's viewer overlay goes into render() instead
        self.dt = 1.0 / fps
        self.last = 0.0
        self._lock = threading.RLock()
        self._running = True

    # ---- the passive viewer's handle, as the main loop uses it
    def __enter__(self) -> "SimView":
        return self

    def __exit__(self, *exc) -> None:
        self.renderer.close()

    def is_running(self) -> bool:
        return self._running

    def close(self) -> None:
        self._running = False

    def lock(self):
        return self._lock

    def sync(self) -> None:
        pass

    def set_texts(self, texts) -> None:
        pass

    # ---- the view
    def orbit(self, dx_px: float, dy_px: float) -> None:
        """Drag on the panel: right turns the scene to the right, down looks from higher up."""
        self.cam.azimuth = (self.cam.azimuth - DEG_PER_PX * dx_px) % 360.0
        self.cam.elevation = float(np.clip(self.cam.elevation - DEG_PER_PX * dy_px, *ELEVATION_RANGE))

    def follow(self, path) -> None:
        """Turn around the middle of the path (centre of its bounding box); keep the last centre without a path."""
        if path is not None and len(path):
            p = np.asarray(path, float)
            self.cam.lookat[:] = (p.min(axis=0) + p.max(axis=0)) / 2.0

    def due(self, now: float) -> bool:
        if now - self.last < self.dt:
            return False
        self.last = now
        return True

    def render(self, d: mujoco.MjData, ov) -> np.ndarray:
        """The scene from the orbit camera with the same 3-D overlay as the viewer (path, A, B), BGR for the stream."""
        self.renderer.update_scene(d, camera=self.cam)
        add_overlay(self.renderer.scene, ov)
        return np.ascontiguousarray(self.renderer.render()[:, :, ::-1])
