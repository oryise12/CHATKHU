"""Calibrated pinhole camera (OpenCV conventions) used by the monitor overlay and mouse editing.

    pixel = K * distort( R_cw * p_world + t_cw )     (cv2.projectPoints)

* camera frame = OpenCV optical frame: x right, y DOWN, z forward (into the scene)
* pixel (u, v): u right, v down, integer coordinates are pixel centres
* world = pinch_sim world = URDF base_link (m)

Two sources:
  * `CameraModel.from_mujoco(...)`: exact model of a MuJoCo camera (fx = fy = (H/2)/tan(fovy/2),
    cx = W/2 - 0.5, cy = H/2 - 0.5, no distortion; MuJoCo camera axes x right, y up, -z forward
    are turned into the optical frame by diag(1, -1, -1)). Used while no real camera is calibrated.
  * `CameraModel.load(json)`: a real calibration, see `to_dict` for the file format:
        {"camera": "...", "image_size": [W, H], "K": [[fx,0,cx],[0,fy,cy],[0,0,1]],
         "dist": [k1, k2, p1, p2, k3], "T_cam_world": 4x4  (or "T_world_cam": 4x4, inverted on load),
         "units": "m"}
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import mujoco
import numpy as np

MJ_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])     # MuJoCo camera axes -> OpenCV optical axes


@dataclass
class CameraModel:
    K: np.ndarray            # (3,3)
    dist: np.ndarray         # (5,) k1 k2 p1 p2 k3
    width: int
    height: int
    R_cw: np.ndarray         # (3,3) world -> camera optical
    t_cw: np.ndarray         # (3,)
    name: str = ""

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_mujoco(cls, m: mujoco.MjModel, d: mujoco.MjData, cam: str, width: int, height: int) -> "CameraModel":
        c = m.camera(cam).id
        f = 0.5 * height / np.tan(np.radians(m.cam_fovy[c]) / 2)
        K = np.array([[f, 0, width / 2 - 0.5], [0, f, height / 2 - 0.5], [0, 0, 1.0]])
        R_world_mj = d.cam_xmat[c].reshape(3, 3)              # columns: MuJoCo camera axes in world
        R_cw = MJ_TO_OPTICAL @ R_world_mj.T
        t_cw = -R_cw @ d.cam_xpos[c]
        return cls(K, np.zeros(5), int(width), int(height), R_cw, t_cw, f"sim:{cam}")

    @classmethod
    def load(cls, path: str | Path) -> "CameraModel":
        data = json.loads(Path(path).read_text())
        if "T_cam_world" in data:
            T = np.asarray(data["T_cam_world"], float)
        elif "T_world_cam" in data:
            T = np.linalg.inv(np.asarray(data["T_world_cam"], float))
        else:
            raise ValueError("calibration needs T_cam_world or T_world_cam (4x4, metres)")
        if data.get("units", "m") != "m":
            raise ValueError("calibration translation must be in metres")
        W, H = data["image_size"]
        dist = np.zeros(5)
        dist[:len(data.get("dist", []))] = data.get("dist", [])
        return cls(np.asarray(data["K"], float), dist, int(W), int(H), T[:3, :3], T[:3, 3],
                   data.get("camera", Path(path).stem))

    def to_dict(self) -> dict:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R_cw, self.t_cw
        return {"camera": self.name, "image_size": [self.width, self.height], "K": self.K.tolist(),
                "dist": self.dist.tolist(), "T_cam_world": T.tolist(), "units": "m",
                "_convention": "OpenCV: T_cam_world maps world (= base_link, m) to the optical frame "
                               "(x right, y down, z forward); pixel centres at integer coordinates"}

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    def scaled_to(self, width: int, height: int) -> "CameraModel":
        """Same camera for a frame resized to width x height (intrinsics scaled)."""
        sx, sy = width / self.width, height / self.height
        K = self.K.copy()
        K[0, 0] *= sx
        K[0, 2] = (K[0, 2] + 0.5) * sx - 0.5
        K[1, 1] *= sy
        K[1, 2] = (K[1, 2] + 0.5) * sy - 0.5
        return CameraModel(K, self.dist.copy(), width, height, self.R_cw.copy(), self.t_cw.copy(), self.name)

    # ------------------------------------------------------------------ geometry
    @property
    def center_world(self) -> np.ndarray:
        return -self.R_cw.T @ self.t_cw

    def project(self, points) -> tuple[np.ndarray, np.ndarray]:
        """World points (N,3) -> pixels (N,2) and a mask of points in front of the camera."""
        P = np.atleast_2d(np.asarray(points, float))
        z = (P @ self.R_cw.T + self.t_cw)[:, 2]
        rvec, _ = cv2.Rodrigues(self.R_cw)
        uv, _ = cv2.projectPoints(P.reshape(-1, 1, 3), rvec, self.t_cw.reshape(3, 1), self.K, self.dist)
        return uv.reshape(-1, 2), z > 1e-6

    def pixel_ray(self, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
        """(origin, unit direction) in world of the ray through pixel (u, v) (lens distortion removed)."""
        xy = cv2.undistortPoints(np.array([[[u, v]]], float), self.K, self.dist).reshape(2)
        d_cam = np.array([xy[0], xy[1], 1.0])
        d = self.R_cw.T @ d_cam
        return self.center_world, d / np.linalg.norm(d)

    # depth image registered to this image (same size, same pixels): the mock calibration and tests
    def depth_z(self, u: float, v: float, depth_mm, half: int = 3, percentile: float = 25.0) -> float | None:
        """Optical Z (m) at pixel (u, v) of a depth image registered to this camera (mm), or None."""
        from .hand_depth import patch_depth
        z_mm = patch_depth(depth_mm, u, v, half, percentile)
        return z_mm / 1000.0 if z_mm > 0 else None

    def point_at_z(self, u: float, v: float, z_m: float) -> np.ndarray:
        """World point on pixel (u, v)'s line of sight at optical Z = z_m."""
        xy = cv2.undistortPoints(np.array([[[u, v]]], float), self.K, self.dist).reshape(2)
        p_cam = np.array([xy[0], xy[1], 1.0]) * z_m
        return self.R_cw.T @ (p_cam - self.t_cw)


def main():
    """Export the sim camera as a calibration file (shows the format; usable with --monitor-calib)."""
    import argparse

    from .model import ROOT, load_config, load_scene, ready_q
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="side_cam")
    ap.add_argument("--out", default="configs/side_cam_calib.json")
    a = ap.parse_args()
    cfg = load_config()
    m, _, _ = load_scene(cfg)
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    cam = CameraModel.from_mujoco(m, d, a.camera, cfg["cameras"]["width_px"], cfg["cameras"]["height_px"])
    cam.save(ROOT / a.out)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
