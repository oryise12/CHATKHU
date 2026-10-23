"""Blank canvas monitor: origin, x / y / z axes, A, B and the path on white, seen from the calibrated camera.

The camera comes from a camera calibration in pac_vision's cam2base.yaml format
(P_base = R @ P_cam + t, metres, OpenCV optical camera axes, intrinsics in camera.depth fx/fy/cx/cy and
camera.rgb width/height). The base frame of that file ("calibration frame") is REP-103 like pac_vision's
base_ref: origin = robot base centre on the desk, +X forward, +Y left, +Z up. A, B and the drawn axes use
it. Inside, the simulator keeps its own world (+Y forward), so points are converted once:

    p_cal = R_robot @ CAL_FROM_WORLD @ p_world + t_robot      (x_cal = y_world, y_cal = -x_world, z the same,
                                                               when the robot stands at the origin)

The simulator world is the robot (its base centre on the desk, +Y = robot forward). When the real robot does not
stand on the calibration origin facing +X, `set_robot_pose(x, y, yaw_deg)` gives where its base centre is and
which way it faces in the calibration frame (R_robot = yaw about +Z, t_robot = (x, y, 0)); everything drawn or
picked through these functions follows. Directions (voice axes, hand motion) use the rotation only.

Until the real calibration is done, `write_mock_cam2base` writes the same kind of file from the simulator
camera (configs/cam2base_mock.yaml). Replacing it with the real file is the only change needed later.
"""
from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from .camera_model import CameraModel

CAL_FROM_WORLD = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
WORLD_FROM_CAL = CAL_FROM_WORLD.T
_pose = {"R": CAL_FROM_WORLD.copy(), "t": np.zeros(3), "xyyaw": (0.0, 0.0, 0.0)}   # world -> calibration frame


def set_robot_pose(x: float = 0.0, y: float = 0.0, yaw_deg: float = 0.0) -> None:
    """Robot base centre (x, y on the desk, m) and heading (deg, counter-clockwise from +X seen from above) in
    the calibration frame. (0, 0, 0) = the robot stands on the calibration origin facing +X."""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    _pose["R"], _pose["t"], _pose["xyyaw"] = yaw @ CAL_FROM_WORLD, np.array([x, y, 0.0]), (x, y, yaw_deg)


def robot_pose() -> tuple[float, float, float]:
    return _pose["xyyaw"]

AXIS_LEN_M = 0.10
AXIS_COLORS = ((40, 40, 230), (40, 170, 40), (230, 90, 30))     # BGR: x red, y green, z blue


# Where the user stands around the robot (calibration frame) -> what "right" and "away" mean for them.
# right / away are unit vectors in the calibration frame; up is always +Z.
USER_SIDES = {
    "front": ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)),    # at +X, facing the robot: right = +Y, away = -X
    "behind": ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0)),   # at -X, facing like the robot: right = -Y, away = +X
    "right": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),     # at the robot's right (-Y), facing +Y: right = +X
    "left": ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),    # at the robot's left (+Y), facing -Y: right = -X
}


def user_axes(side: str) -> dict[str, np.ndarray]:
    """Voice directions fixed to the calibration axes, as the user standing at `side` sees them (world)."""
    right_c, away_c = USER_SIDES[side]
    right, away, up = cal_dir_to_world(right_c), cal_dir_to_world(away_c), cal_dir_to_world((0.0, 0.0, 1.0))
    return {"up": up, "down": -up, "right": right, "left": -right, "away": away, "toward_me": -away}


def user_view(side: str) -> tuple[np.ndarray, float, float, float]:
    """MuJoCo free-camera (lookat, distance, azimuth, elevation) from where the user stands."""
    away = user_axes(side)["away"]
    lookat = np.array([0.0, 0.18, 0.12])                          # middle of the work area, in front of the robot
    return lookat, 1.15, float(np.degrees(np.arctan2(away[1], away[0]))), -25.0


def cal_to_world(p) -> np.ndarray:
    return (np.asarray(p, float) - _pose["t"]) @ _pose["R"]


def world_to_cal(p) -> np.ndarray:
    return np.asarray(p, float) @ _pose["R"].T + _pose["t"]


def cal_dir_to_world(v) -> np.ndarray:
    """A direction / displacement (no translation)."""
    return np.asarray(v, float) @ _pose["R"]


def world_dir_to_cal(v) -> np.ndarray:
    return np.asarray(v, float) @ _pose["R"].T


def load_cam2base(path: str | Path) -> CameraModel:
    """pac_vision cam2base.yaml -> CameraModel that projects simulator WORLD points."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    R, t = np.asarray(data["R"], float), np.asarray(data["t"], float)
    cam = data.get("camera") or {}
    intr, rgb = cam.get("depth") or {}, cam.get("rgb") or {}
    w, h = int(rgb.get("width", intr.get("width", 640))), int(rgb.get("height", intr.get("height", 480)))
    K = np.array([[intr.get("fx", 570.0), 0, intr.get("cx", w / 2)], [0, intr.get("fy", 570.0), intr.get("cy", h / 2)],
                  [0, 0, 1]], float)
    # p_cam = R^T (p_cal - t),  p_cal = R_robot p_world + t_robot
    R_cw = R.T @ _pose["R"]
    t_cw = R.T @ (_pose["t"] - t)
    return CameraModel(K, np.zeros(5), w, h, R_cw, t_cw, f"cam2base:{Path(path).name}")


def write_mock_cam2base(path: str | Path, cam: CameraModel) -> Path:
    """A cam2base.yaml made from a known camera (the simulator's side_cam) - stand-in for the real calibration."""
    R = CAL_FROM_WORLD @ cam.R_cw.T                       # optical camera -> calibration frame
    t = -CAL_FROM_WORLD @ cam.R_cw.T @ cam.t_cw
    K = cam.K
    data = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "note": "MOCK - made from the simulator camera, not a real calibration. Replace with pac_vision's "
                "config/cam2base.yaml (same format). P_base = R @ P_cam + t, m; base = +X forward, +Y left, +Z up.",
        "R": [[float(v) for v in row] for row in R],
        "t": [float(v) for v in t],
        "passed": True,
        "camera": {"rgb": {"width": int(cam.width), "height": int(cam.height)},
                   "depth": {"width": int(cam.width), "height": int(cam.height), "fx": float(K[0, 0]),
                             "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2])}},
    }
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return p


def _line(img, cam, a_world, b_world, color, thick=1):
    uv, front = cam.project(np.vstack([a_world, b_world]))
    if front.all():
        cv2.line(img, tuple(int(round(v)) for v in uv[0]), tuple(int(round(v)) for v in uv[1]), color, thick,
                 cv2.LINE_AA)
        return uv
    return None


def _label(img, text, uv, color, dx=6, dy=-6, scale=0.55):
    org = (int(round(uv[0])) + dx, int(round(uv[1])) + dy)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def blank_canvas(cam: CameraModel, grid=((-0.10, 0.45), (-0.35, 0.35)), step=0.05) -> np.ndarray:
    """White image with the desk grid (z = 0, calibration frame) and the origin with x / y / z axes."""
    img = np.full((cam.height, cam.width, 3), 255, np.uint8)
    (x0, x1), (y0, y1) = grid
    for x in np.arange(x0, x1 + 1e-9, step):
        _line(img, cam, cal_to_world([x, y0, 0]), cal_to_world([x, y1, 0]), (228, 228, 228))
    for y in np.arange(y0, y1 + 1e-9, step):
        _line(img, cam, cal_to_world([x0, y, 0]), cal_to_world([x1, y, 0]), (228, 228, 228))
    return draw_axes(img, cam)


def draw_axes(img, cam: CameraModel) -> np.ndarray:
    """Origin O and the x / y / z axes of the calibration frame (on a canvas or a real camera image)."""
    o = cal_to_world([0, 0, 0])
    for k, (name, color) in enumerate(zip("xyz", AXIS_COLORS)):
        tip = np.zeros(3)
        tip[k] = AXIS_LEN_M
        uv = _line(img, cam, o, cal_to_world(tip), color, 2)
        if uv is not None:
            _label(img, name, uv[1], color, dx=4, dy=4)
    uv, front = cam.project(o)
    if front[0]:
        cv2.circle(img, tuple(int(round(v)) for v in uv[0]), 4, (30, 30, 30), -1, cv2.LINE_AA)
        _label(img, "O", uv[0], (30, 30, 30), dx=-18, dy=16)
    return img


def draw_ab(img, cam: CameraModel, a_world, b_world) -> None:
    """A (end-effector start) and B (target on the desk) labels with their calibration coordinates."""
    for name, p, color in (("A", a_world, (150, 60, 160)), ("B", b_world, (20, 120, 220))):
        if p is None:
            continue
        uv, front = cam.project(np.asarray(p, float))
        if not front[0]:
            continue
        c = tuple(int(round(v)) for v in uv[0])
        cv2.circle(img, c, 7, color, 2, cv2.LINE_AA)
        x, y, z = world_to_cal(p)
        text = f"{name} ({x:.2f}, {y:.2f}, {z:.2f})"
        if name == "A":       # left of A: the path's highest-point label is drawn to the right of the path
            (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            _label(img, text, uv[0], color, dx=-tw - 12, dy=5, scale=0.5)
        else:
            _label(img, text, uv[0], color, dx=10, dy=-10, scale=0.5)
