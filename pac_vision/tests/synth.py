"""테스트용 합성 장면: 비스듬히 내려다보는 카메라, 책상면(z=0), 원기둥 물체, 벽. 카메라 없이 depth 영상을 만든다."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from pac_vision.cam2base import CameraModel
from pac_vision.config import load_config

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

# 저장소 설정과 무관하게 고정한 카메라 값 (camera.yaml 기본값과 같은 숫자)
CAMERA_CFG = {
    "depth": {"width": 640, "height": 480, "fx": 570.0, "fy": 570.0, "cx": 320.0, "cy": 240.0,
              "z_offset_m": -0.021, "flip": True, "min_m": 0.6, "max_m": 4.0},
    "rgb": {"width": 640, "height": 480},
    "rgb_to_depth": {"sx": 1.0, "sy": 1.0, "dx": 25.0, "dy": 0.0},
}


def camera(rgb_to_depth: dict | None = None) -> CameraModel:
    """CAMERA_CFG의 카메라. rgb_to_depth를 주면 그 부분만 바꾼다 (거리 식 시험용)."""
    if rgb_to_depth is None:
        return CameraModel.from_config(CAMERA_CFG)
    return CameraModel.from_config(dict(CAMERA_CFG, rgb_to_depth=rgb_to_depth))


# 거리 식 시험용 정합 (2026-10-14 설치 자세에서 맞춘 계수와 비슷한 크기)
DIST_RGB_TO_DEPTH = {"sx": 1.0, "sy": 1.0,
                     "dx": {"a": 5.7, "b": -0.0136, "c": 0.0, "d": 20.5},
                     "dy": {"a": 8.0, "b": 0.0, "c": -0.01, "d": 3.0}}


def repo_config() -> dict:
    """커밋된 config만 (이 컴퓨터의 local.yaml·pose.yaml은 테스트에 섞지 않는다)."""
    return load_config(CONFIG_DIR, overlays=False)


# 합성 장면용 기준물 배치. 현장 설정(config/a4_layout.yaml)을 바꿔도 테스트가 흔들리지 않게 따로 고정한다.
# 값: A4 두 장 맞대기, 대각선 교차점 = base, 두 장을 합친 직사각형(29.7 x 42.0 cm)의 바깥 꼭짓점
LAYOUT = {
    "base_point": [0.0, 0.0, 0.0],
    "corners": [[-0.1485, -0.105, 0.0], [0.1485, -0.105, 0.0], [0.1485, 0.315, 0.0], [-0.1485, 0.315, 0.0]],
    "recheck_corner": 4,
}


def scene_config() -> dict:
    """저장소 설정(thresholds 등) + 테스트 고정 배치 LAYOUT."""
    cfg = repo_config()
    cfg["a4_layout"] = {k: (list(map(list, v)) if k == "corners" else v) for k, v in LAYOUT.items()}
    return cfg


def look_at(position, target):
    """카메라 위치와 바라보는 점 → (R, t): P_base = R @ P_cam + t. 광학 좌표계 X 오른쪽, Y 아래, Z 정면."""
    c = np.asarray(position, float)
    f = np.asarray(target, float) - c
    f /= np.linalg.norm(f)
    right = np.cross(f, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(f, right)
    return np.column_stack([right, down, f]), c


# 기본 자세: base 뒤쪽 위에서 A4 쪽을 약 55° 기울여 내려다봄 (책상면 법선과 광축 사이 각)
DEFAULT_POSITION = (-0.90, 0.155, 0.75)
DEFAULT_TARGET = (0.15, 0.155, 0.0)


def _ray_cylinder(c, d, cyl):
    """원기둥 (xc, yc, 반지름, 높이)과 광선 c + s d의 가장 가까운 교점 거리 s (없으면 inf)."""
    xc, yc, r, h = cyl
    ox, oy = c[0] - xc, c[1] - yc
    a = d[..., 0] ** 2 + d[..., 1] ** 2
    b = 2 * (ox * d[..., 0] + oy * d[..., 1])
    cc = ox ** 2 + oy ** 2 - r ** 2
    disc = b ** 2 - 4 * a * cc
    s = np.full(d.shape[:-1], np.inf)
    with np.errstate(invalid="ignore", divide="ignore"):
        s_side = (-b - np.sqrt(disc)) / (2 * a)
        z_side = c[2] + s_side * d[..., 2]
        ok = (disc >= 0) & (s_side > 0) & (z_side >= 0) & (z_side <= h)
        s = np.where(ok, s_side, s)
        s_top = (h - c[2]) / d[..., 2]
        px, py = c[0] + s_top * d[..., 0] - xc, c[1] + s_top * d[..., 1] - yc
        ok_top = (s_top > 0) & (px ** 2 + py ** 2 <= r ** 2)
        s = np.where(ok_top & (s_top < s), s_top, s)
    return s


def render_depth(cam: CameraModel, R, t, cylinders=(), wall_rows=0, wall_m=1.2,
                 noise_mm=0.0, outlier_frac=0.0, seed=0):
    """각 depth 픽셀의 광선을 장면과 교차시켜 depth(mm, float32)를 만든다. 0 = 측정 실패."""
    rng = np.random.default_rng(seed)
    vv, uu = np.mgrid[0:cam.height, 0:cam.width]
    dirs_c = cam.ray(uu, vv)                    # Z 성분 1 → 교점 거리 s가 곧 광학 Z
    dirs_b = dirs_c @ np.asarray(R).T
    with np.errstate(divide="ignore"):
        s = np.where(dirs_b[..., 2] < 0, -t[2] / dirs_b[..., 2], np.inf)
    for cyl in cylinders:
        s = np.minimum(s, _ray_cylinder(np.asarray(t, float), dirs_b, cyl))
    depth = np.where(np.isfinite(s), (s + cam.z_offset_m) * 1000.0, 0.0)
    if wall_rows:
        depth[:wall_rows] = (wall_m + cam.z_offset_m) * 1000.0
    if noise_mm:
        depth = np.where(depth > 0, depth + rng.normal(0, noise_mm, depth.shape), 0.0)
    if outlier_frac:
        mask = rng.random(depth.shape) < outlier_frac
        depth = np.where(mask & (depth > 0), depth + rng.uniform(-300, 300, depth.shape), depth)
    return depth.astype(np.float32)


def clicks_for(cam: CameraModel, R, t, base_points, rgb_size=None, mirrored=False):
    """기준 좌표 점들이 화면 RGB에서 찍힐 픽셀 (정확한 클릭)."""
    p_cam = (np.asarray(base_points, float) - t) @ np.asarray(R)
    ud, vd = cam.project(p_cam)
    u, v = cam.depth_to_rgb(ud, vd, rgb_size, mirrored, z_m=p_cam[:, 2])
    return np.stack([u, v], axis=1)


def rotation_angle_deg(Ra, Rb) -> float:
    c = (np.trace(np.asarray(Ra) @ np.asarray(Rb).T) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))
