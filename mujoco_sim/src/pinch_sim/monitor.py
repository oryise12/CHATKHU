"""Monitor (side_cam window): a camera frame with the path drawn as a 2-D graph.

Nothing of the planner is drawn into the 3-D scene that produces the frame. Every overlay item
(path, height colours, reference path, target marker, approach / goal points, peak label, drag
point, failure spots, TCP) is a world point projected with a calibrated `CameraModel` and drawn
with OpenCV, so the same code works on a real camera image given its calibration.

Frame sources:
  * SimFrameSource: MuJoCo render of a fixed sim camera (robot and floor only, no overlay)
  * VideoFrameSource: cv2.VideoCapture (device index or file); the camera model is rescaled to
    the actual frame size
"""
from __future__ import annotations

from dataclasses import replace

import cv2
import mujoco
import numpy as np

from .camera_model import CameraModel
from .overlay import PATH_COLOR, REFERENCE_COLOR, Overlay, segment_colors

SHIFT = 4                       # sub-pixel drawing: coordinates * 2**SHIFT
_S = 1 << SHIFT


def _bgr(rgba) -> tuple:
    r, g, b = (np.clip(np.asarray(rgba[:3], float), 0, 1) * 255).astype(int)
    return int(b), int(g), int(r)


def _pt(uv) -> tuple:
    return int(round(uv[0] * _S)), int(round(uv[1] * _S))


class SimFrameSource:
    def __init__(self, m: mujoco.MjModel, renderer: mujoco.Renderer, camera: str):
        self.m, self.r, self.camera = m, renderer, camera

    def prepare(self, d: mujoco.MjData) -> None:
        """Copy the scene from d (call while d is not being stepped, e.g. under viewer.lock())."""
        self.r.update_scene(d, camera=self.camera)    # no user geoms: the path is NOT in the 3-D scene

    def frame(self) -> np.ndarray:
        return cv2.cvtColor(self.r.render(), cv2.COLOR_RGB2BGR)


class VideoFrameSource:
    def __init__(self, spec: str):
        self.cap = cv2.VideoCapture(int(spec) if str(spec).isdigit() else spec)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video source {spec!r}")
        self.last = None

    def prepare(self, d) -> None:
        pass

    def frame(self) -> np.ndarray:
        ok, img = self.cap.read()
        if not ok and self.cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0:      # file: loop
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, img = self.cap.read()
        if ok:
            self.last = img
        if self.last is None:
            raise RuntimeError("no frame from video source")
        return self.last.copy()


def _polyline(img, cam: CameraModel, pts, colors, thickness: int, halo=None) -> None:
    """halo: dark rim under the line (thickness + 2) so white/green paths stay visible on the light desk."""
    uv, front = cam.project(pts)
    seg = [i for i in range(len(pts) - 1) if front[i] and front[i + 1]]
    if halo is not None:
        for i in seg:
            cv2.line(img, _pt(uv[i]), _pt(uv[i + 1]), halo, thickness + 2, cv2.LINE_AA, SHIFT)
    for i in seg:
        cv2.line(img, _pt(uv[i]), _pt(uv[i + 1]), colors[i], thickness, cv2.LINE_AA, SHIFT)


def _dot(img, cam, p, radius: int, bgr, outline=(0, 0, 0)) -> tuple | None:
    uv, front = cam.project(p)
    if not front[0]:
        return None
    c = _pt(uv[0])
    cv2.circle(img, c, radius * _S, bgr, -1, cv2.LINE_AA, SHIFT)
    cv2.circle(img, c, radius * _S, outline, 1, cv2.LINE_AA, SHIFT)
    return tuple(uv[0])


def draw_overlay_2d(img: np.ndarray, cam: CameraModel, ov: Overlay, line_px: int = 3,
                    minimal: bool = False) -> np.ndarray:
    """Draw the overlay in place on a BGR frame of size cam.width x cam.height.

    minimal (real camera image): only the path and what the user edits (reference path, picked / dragged
    point, highest point, failures); no target ring, approach / goal markers or the simulated TCP.
    """
    if minimal:
        ov = replace(ov, floor_point=None, approach=None, tcp_goal=None, goal_axis=None, tcp=None)
    if ov.floor_point is not None:                           # floor marker: projected circle on z = 0
        a = np.linspace(0, 2 * np.pi, 32)
        ring = ov.floor_point + 0.022 * np.column_stack([np.cos(a), np.sin(a), np.zeros_like(a)])
        uv, front = cam.project(ring)
        if front.all():
            cv2.fillPoly(img, [np.round(uv * _S).astype(np.int32)], (153, 26, 255), cv2.LINE_AA, SHIFT)
        if ov.approach is not None:
            _polyline(img, cam, np.vstack([ov.floor_point, ov.approach]), [(153, 26, 255)], 1)
    if ov.reference is not None and len(ov.reference) > 1:
        _polyline(img, cam, ov.reference, [_bgr(REFERENCE_COLOR)] * len(ov.reference), 1, halo=(60, 60, 60))
    if ov.path is not None and len(ov.path) > 1:
        idx = np.arange(len(ov.path))
        cols = [_bgr(c) for c in segment_colors(ov, idx)]
        _polyline(img, cam, ov.path, cols, line_px, halo=(35, 35, 35))
    if ov.approach is not None:
        _dot(img, cam, ov.approach, 5, (230, 230, 25))
    if ov.tcp_goal is not None:
        _dot(img, cam, ov.tcp_goal, 6, (25, 230, 255))
        if ov.goal_axis is not None:
            uv, front = cam.project(np.vstack([ov.tcp_goal, ov.tcp_goal + 0.04 * np.asarray(ov.goal_axis)]))
            if front.all():
                cv2.arrowedLine(img, _pt(uv[0]), _pt(uv[1]), (25, 230, 255), 2, cv2.LINE_AA, SHIFT, 0.3)
    for p, _kind in ov.failures:
        _dot(img, cam, p, 9, (0, 0, 255))
    if ov.peak is not None:
        uv = _dot(img, cam, ov.peak, 5, (26, 26, 255), (255, 255, 255))
        if uv is not None:
            text, org = f"z = {ov.peak[2]:.3f} m", (int(uv[0]) + 8, int(uv[1]) - 8)
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if ov.selected is not None:
        _dot(img, cam, ov.selected, 6, (255, 255, 255))
    elif ov.pointed is not None:                 # clicked point for voice edits: yellow ring + dot
        uv = _dot(img, cam, ov.pointed, 4, (0, 215, 255))
        if uv is not None:
            cv2.circle(img, _pt(uv), 11 * _S, (0, 0, 0), 4, cv2.LINE_AA, SHIFT)
            cv2.circle(img, _pt(uv), 11 * _S, (0, 215, 255), 2, cv2.LINE_AA, SHIFT)
    if ov.tcp is not None:
        _dot(img, cam, ov.tcp, 4, (0, 128, 255))
    return img


__all__ = ["SimFrameSource", "VideoFrameSource", "draw_overlay_2d", "PATH_COLOR"]
