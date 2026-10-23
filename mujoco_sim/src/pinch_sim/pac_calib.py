"""Monitor camera from a real pac_vision calibration (config/cam2base.yaml), through pac_vision's own math.

pac_vision calibrates the DEPTH camera (P_base = R @ P_cam + t). A pixel of the colour image is related to
it through the RGB<->depth registration coefficients (pose.yaml copy in the file, with a distance term d/Z),
the 640x480 reference / 16:9 centre-band rule and the mirror flag. A plain pinhole would put the path a few
pixels off, so this adapter calls pac_vision itself:

  * project(world)    -> Cam2Base.base_to_pixel  (the same function pac_vision uses to draw its base axes)
  * pixel_ray(u, v)   -> RGB pixel -> depth pixel at two distances -> back-projected -> base -> world

and offers the small CameraModel interface the monitor, the mouse / hand picking and the drawings use
(width, height, project, pixel_ray, scaled_to). World is the simulator frame; base is converted with
canvas.CAL_FROM_WORLD (base_ref: +X forward, +Y left, +Z up).

pac_vision is imported from $PAC_VISION_SRC, the `src` folder next to the calibration file
(pac_vision/config/cam2base.yaml -> pac_vision/src), or a pac_vision folder next to mujoco_sim.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import yaml

from .canvas import cal_to_world, world_to_cal


def is_pac_vision_file(path: str | Path) -> bool:
    """A real calibrate.py result (not our MOCK file): has the registration coefficients of the colour camera."""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    cam = doc.get("camera") or {}
    return "MOCK" not in str(doc.get("note", "")) and "rgb_to_depth" in cam


def _import_pac_vision(calib: Path):
    candidates = [os.environ.get("PAC_VISION_SRC"), calib.resolve().parent.parent / "src",
                  Path(__file__).resolve().parents[3] / "pac_vision" / "src",          # pac_vision next to mujoco_sim
                  Path.home() / "research/manipulator/26_project_manipulator/pac_vision/src"]
    for c in candidates:
        if c and (Path(c) / "pac_vision" / "cam2base.py").exists():
            if str(c) not in sys.path:
                sys.path.insert(0, str(c))
            import pac_vision.session as pv_session
            return pv_session
    raise ImportError("pac_vision not found: put the calibration in pac_vision/config/ or set PAC_VISION_SRC")


class PacVisionCamera:
    """Monitor camera = the colour image of the calibrated Astra (screen size width x height)."""

    def __init__(self, conv, width: int, height: int, mirrored: bool = False, name: str = ""):
        self.conv, self.width, self.height, self.mirrored = conv, int(width), int(height), mirrored
        self.name = name or "pac_vision"

    @classmethod
    def load(cls, path: str | Path, width: int | None = None, height: int | None = None) -> "PacVisionCamera":
        path = Path(path)
        conv = _import_pac_vision(path).load_cam2base(path)
        return cls(conv, width or conv.cam.rgb_width, height or conv.cam.rgb_height,
                   name=f"pac_vision:{path.name}")

    def scaled_to(self, width: int, height: int) -> "PacVisionCamera":
        # pac_vision handles any screen size itself (640x480 reference, 16:9 = centre band)
        return PacVisionCamera(self.conv, width, height, self.mirrored, self.name)

    @property
    def _size(self):
        return (self.width, self.height)

    def project(self, points) -> tuple[np.ndarray, np.ndarray]:
        """World points (N,3) -> colour-image pixels (N,2) and a mask of points in front of the camera."""
        base = world_to_cal(np.atleast_2d(np.asarray(points, float)))
        uv = self.conv.base_to_pixel(base, rgb_mirrored=self.mirrored, rgb_size=self._size).reshape(-1, 2)
        front = (self.conv.base_to_camera(base)[:, 2] > 1e-6) & np.isfinite(uv).all(axis=1)
        return np.nan_to_num(uv, nan=-1e6), front

    def pixel_ray(self, u: float, v: float, z_near: float = 0.4, z_far: float = 2.0):
        """(origin, unit direction) in world of the colour pixel's line of sight.

        With a distance term in the registration a colour pixel meets the depth image at a slightly different
        place for each distance, so the line through the points seen at z_near and z_far is used.
        """
        cam, pts = self.conv.cam, []
        for z in (z_near, z_far):
            ud, vd = cam.rgb_to_depth(u, v, self._size, self.mirrored, z_m=z)
            p_cam = cam.ray(ud, vd) * z
            pts.append(cal_to_world(self.conv.camera_to_base(p_cam)))
        o, d = pts[0], pts[1] - pts[0]
        return o, d / np.linalg.norm(d)

    def depth_z(self, u: float, v: float, depth_mm, half: int = 3, percentile: float = 25.0) -> float | None:
        """Optical Z (m) of what is seen at colour pixel (u, v): the RGB->depth registration iterated on the
        depth there (pac_vision's rgb_to_depth_on_depth), then the low percentile of the depth patch.
        None outside the depth image, without valid depth, or outside the camera's range (min_m, max_m)."""
        from .hand_depth import patch_depth
        cam = self.conv.cam
        ud, vd = cam.rgb_to_depth_on_depth(u, v, depth_mm, half, self._size, self.mirrored)
        z_mm = patch_depth(depth_mm, float(np.ravel(ud)[0]), float(np.ravel(vd)[0]), half, percentile)
        if z_mm <= 0:
            return None
        z = float(cam.depth_m(z_mm))
        return z if cam.min_m <= z <= cam.max_m else None

    def point_at_z(self, u: float, v: float, z_m: float) -> np.ndarray:
        """World point on colour pixel (u, v)'s line of sight at optical Z = z_m."""
        cam = self.conv.cam
        ud, vd = cam.rgb_to_depth(u, v, self._size, self.mirrored, z_m=z_m)
        p_cam = cam.ray(float(np.ravel(ud)[0]), float(np.ravel(vd)[0])) * z_m
        return cal_to_world(np.asarray(self.conv.camera_to_base(p_cam), float).reshape(3))

    def desk_point(self, u: float, v: float) -> np.ndarray:
        """The desk point (z = 0) seen at a colour pixel, world (pac_vision's pixel_to_desk)."""
        base, _, _ = self.conv.pixel_to_desk(u, v, rgb_mirrored=self.mirrored, rgb_size=self._size)
        return cal_to_world(np.asarray(base, float).reshape(-1, 3)[0])


def load_camera(path: str | Path, fallback_loader):
    """Real pac_vision result -> PacVisionCamera; anything else (our MOCK file) -> fallback_loader(path)."""
    return PacVisionCamera.load(path) if is_pac_vision_file(path) else fallback_loader(path)
