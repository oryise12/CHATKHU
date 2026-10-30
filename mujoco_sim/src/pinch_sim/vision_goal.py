"""Object on the desk -> target of a new initial path, from the depth frames the simulator already reads (S6).

The simulator process owns the Astra (colour + depth threads); nothing here opens a camera. pac_vision's object
detector (YOLO boxes, scripts/detect_objects.py) is not needed: on a calibrated desk (z = 0) everything standing
higher than `min_height_m` is an object. Those depth points are grouped on a 1 cm desk grid, and each group's
top-face centre comes from pac_vision's own objects.top_center (90th percentile height, median x, y of the top band).

Coordinates (base_ref = the pac_vision calibration frame, m; world = the simulator, through canvas.cal_to_world):
  desk_point  (x, y, 0)                                 where the object stands
  object_top  (x, y, z_top)                             top-face centre = what is returned here
  tcp_goal    object_top + (0, 0, target.work_height_m)  where the gripper tip stops   (Session.cmd_vision_target)
  approach    object_top + (0, 0, approach_height_m)     the point the path comes down from
Limits (measure on site, see docs): an obliquely seen tall object's top centre is biased 2.5-5 cm toward the camera
(pac_vision INTERFACE.md); the real robot standing on the desk is excluded by a radius around its base and a
height cap; the object is NOT in the MuJoCo scene, so validation does not check collisions with it.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class VisionGoalConfig:
    min_height_m: float = 0.01      # above the desk = object (pac_vision thresholds objects.min_height_m)
    max_height_m: float = 0.20      # higher points are not goal objects (the robot's own arm, a person)
    robot_clear_m: float = 0.15     # points this close to the robot base (horizontally) are the robot itself
    reach_m: float = 0.45           # farther than this from the robot base: out of reach, ignored
    grid_m: float = 0.01
    min_pixels: int = 30            # pac_vision objects.min_pixels
    top_percentile: float = 90.0
    top_band_m: float = 0.008
    step_px: int = 2                # use every 2nd depth pixel (speed)


@dataclass
class DeskObject:
    top_base: np.ndarray            # object_top in base_ref (m)
    n_points: int
    footprint_m2: float


def find_objects(conv, depth_mm, robot_base_xy, cfg: VisionGoalConfig | None = None) -> list[DeskObject]:
    """Objects standing on the calibrated desk, largest first. robot_base_xy: robot base centre in base_ref."""
    from scipy import ndimage
    from pac_vision.cam2base import roi_points
    from pac_vision.objects import top_center
    cfg = cfg or VisionGoalConfig()
    cam = conv.cam
    h, w = depth_mm.shape[:2]
    pts_cam = roi_points(depth_mm, cam, (0, 0, w, h), step=cfg.step_px)[0]
    if len(pts_cam) == 0:
        return []
    p = conv.camera_to_base(pts_cam)
    r = np.hypot(p[:, 0] - robot_base_xy[0], p[:, 1] - robot_base_xy[1])
    keep = (p[:, 2] > cfg.min_height_m) & (p[:, 2] < cfg.max_height_m) & (r > cfg.robot_clear_m) & (r < cfg.reach_m)
    p = p[keep]
    if len(p) < cfg.min_pixels:
        return []
    ij = np.floor(p[:, :2] / cfg.grid_m).astype(int)
    lo = ij.min(axis=0)
    ij -= lo
    occ = np.zeros(ij.max(axis=0) + 1, np.int32)
    np.add.at(occ, (ij[:, 0], ij[:, 1]), 1)
    labels, n = ndimage.label(occ > 0, structure=np.ones((3, 3), int))
    lab = labels[ij[:, 0], ij[:, 1]]
    out = []
    for k in range(1, n + 1):
        q = p[lab == k]
        if len(q) < cfg.min_pixels:
            continue
        tc = top_center(q, cfg.min_height_m, cfg.max_height_m, cfg.top_percentile, cfg.top_band_m, cfg.min_pixels)
        if tc.valid:
            out.append(DeskObject(tc.xyz, len(q), float(np.count_nonzero(labels == k)) * cfg.grid_m ** 2))
    return sorted(out, key=lambda o: -o.n_points)


def choose(objects: list[DeskObject], near_xy=None, near_max_m: float = 0.08) -> tuple[DeskObject | None, str]:
    """The goal object: the one the finger points at (near_xy, base_ref) or the only one. (object, reason)."""
    if not objects:
        return None, "no_object"
    if near_xy is not None:
        d = [float(np.hypot(*(o.top_base[:2] - np.asarray(near_xy, float)[:2]))) for o in objects]
        i = int(np.argmin(d))
        return (objects[i], "") if d[i] <= near_max_m else (None, "no_object_at_finger")
    if len(objects) > 1:
        return None, "several_objects"
    return objects[0], ""
