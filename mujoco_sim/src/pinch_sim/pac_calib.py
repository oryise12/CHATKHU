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

pac_vision is imported from the pac_vision folder next to mujoco_sim (this repository), else $PAC_VISION_SRC, else
the `src` folder next to the calibration file (pac_vision/config/cam2base.yaml -> pac_vision/src).

A calibration file is not trusted just because it exists (calibration_problems): a failed result (passed: false),
the format example, settings that differ from the current pac_vision config (registration, intrinsics, flip, ...)
are refused unless the user asks for an unverified run. Even an accepted file only says the A4 points matched at
calibration time; MarkerMonitor checks fixed ArUco markers (pac_vision config/markers.yaml, measured, not used in
the calibration) on the live image, and only "verified" counts as a matching overlay / usable 3-D pinch.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import yaml

from .canvas import cal_to_world, world_to_cal

REPO_PAC_VISION = Path(__file__).resolve().parents[3] / "pac_vision"


def is_pac_vision_file(path: str | Path) -> bool:
    """A real calibrate.py result (not our MOCK file): has the registration coefficients of the colour camera."""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    cam = doc.get("camera") or {}
    return "MOCK" not in str(doc.get("note", "")) and "rgb_to_depth" in cam


def _import_pac_vision(calib: Path):
    candidates = [REPO_PAC_VISION / "src",                      # this repository's pac_vision first (same version)
                  os.environ.get("PAC_VISION_SRC"), calib.resolve().parent.parent / "src"]
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


def calibration_problems(path: str | Path) -> list[str]:
    """Why a real pac_vision cam2base.yaml must not be trusted for the live camera ([] = file is acceptable).

    Compared with the CURRENT pac_vision config of the calibration's folder (camera.yaml + local.yaml + pose.yaml),
    which is also what the depth stream is opened with."""
    path = Path(path)
    if path.name.endswith(".example.yaml"):
        return ["this is the format example, not a calibration (run pac_vision/scripts/calibrate.py)"]
    if not is_pac_vision_file(path):
        return ["not a pac_vision calibration (mock / sim camera file)"]
    pv = _import_pac_vision(path)
    from pac_vision.config import load_config
    conv = pv.load_cam2base(path)
    out = []
    if not conv.info.get("passed", False):
        out.append("calibration did not pass its checks (passed: false) - calibrate again")
    cur = load_config(path.resolve().parent, ("camera",))["camera"]
    for m in pv.calibration_mismatches(conv, cur):
        out.append(f"camera setting '{m}' differs from the current pac_vision config (pose.yaml / camera.yaml): "
                   "calibrate again with the current settings")
    return out


def identity_problems(path: str | Path, video_device: int) -> list[str]:
    """The colour camera opened now vs the one recorded in cam2base.yaml (USB port / name). [] = same or unknown."""
    pv = _import_pac_vision(Path(path))
    from pac_vision.devices import identity_problems as _problems, rgb_identity
    conv = pv.load_cam2base(path)
    return [p for p in _problems(conv.info.get("camera_identity"), rgb_identity(video_device)) if p != "no_identity"]


def find_video_device(name_match: str = "Astra", root="/sys/class/video4linux", can_open=None) -> int | None:
    """Smallest /dev/videoN whose name contains name_match and that gives a frame (pac_vision _ui.open_rgb rule).

    An Astra Pro shows two nodes with the same name; the colour stream is the one that gives frames."""
    cands = []
    for p in Path(root).glob("video*"):
        try:
            name = (p / "name").read_text(encoding="utf-8").strip()
            n = int(p.name[5:])
        except (OSError, ValueError):
            continue
        if name_match.lower() in name.lower():
            cands.append(n)
    if can_open is None:
        def can_open(n):
            import cv2
            cap = cv2.VideoCapture(n, cv2.CAP_V4L2)
            ok = cap.isOpened() and cap.read()[0]
            cap.release()
            return ok
    return next((n for n in sorted(cands) if can_open(n)), None)


class MarkerMonitor:
    """Fixed ArUco markers on the live colour image (pac_vision markers.py) -> is the overlay still calibrated?

    state: "verified" (all visible markers within tolerance, enough of them), "moved" (a visible marker is off: the
    camera moved/turned, a marker moved, or the calibration is wrong), "not_visible" (too few markers seen, for
    `lost_after_s` once verified), "no_config" (no markers.yaml next to the calibration), "checking" (not decided).
    A change needs `confirm` agreeing checks in a row (one odd frame is not a camera move). "moved" stays until the
    markers agree again for `confirm` checks. Only "verified" makes the overlay and the 3-D pinch count as calibrated.
    """

    def __init__(self, cam: "PacVisionCamera", markers_path: str | Path | None, period_s: float = 0.5,
                 confirm: int = 2, lost_after_s: float = 5.0):
        self.cam, self.period_s, self.confirm, self.lost_after_s = cam, period_s, confirm, lost_after_s
        self.mset = None
        self.path = Path(markers_path) if markers_path else None
        if self.path is not None and self.path.exists():
            from pac_vision.markers import load_markers
            self.mset = load_markers(self.path)
        self.state = "checking" if self.mset is not None else "no_config"
        self.result = None                # last pac_vision VerifyResult
        self._last_t = -1e9
        self._streak: tuple[str, int] = ("", 0)
        self._last_ok_t: float | None = None

    @property
    def verified(self) -> bool:
        return self.state == "verified"

    def text(self) -> str:
        r = self.result
        if self.state == "no_config":
            return "정합 미검증: 기준 표지 설정 없음 (markers.yaml)"
        if self.state == "checking":
            return "정합 확인 중 (기준 표지)"
        if self.state == "moved":
            return "카메라가 움직였어요 - 다시 보정하세요" + (f" ({r.summary()})" if r is not None else "")
        if self.state == "not_visible":
            return "정합 미검증: 기준 표지가 안 보여요"
        return r.summary() if r is not None else "정합 확인됨"

    def update(self, frame_bgr, now: float) -> bool:
        """Check at most every period_s. True when the state changed."""
        if self.mset is None or frame_bgr is None or now - self._last_t < self.period_s:
            return False
        self._last_t = now
        from pac_vision.markers import detect, verify
        h, w = frame_bgr.shape[:2]
        res = verify(self.cam.conv, self.mset, detect(frame_bgr, self.mset.dictionary), rgb_size=(w, h),
                     rgb_mirrored=self.cam.mirrored)
        return self.apply(res, now)

    def apply(self, res, now: float) -> bool:
        """One verification result (pac_vision VerifyResult) -> state with confirmation. True when it changed."""
        self.result = res
        kind = res.state
        if kind == "verified":
            self._last_ok_t = now
        n = self._streak[1] + 1 if self._streak[0] == kind else 1
        self._streak = (kind, n)
        old = self.state
        if kind in ("verified", "moved"):
            if n >= self.confirm:
                self.state = kind
        elif kind == "not_visible" and self.state != "moved" and n >= self.confirm:
            # a hand over a marker for a moment keeps "verified"; only a longer gap says it cannot be verified
            if self.state != "verified" or now - (self._last_ok_t or -1e9) >= self.lost_after_s:
                self.state = "not_visible"
        return self.state != old
