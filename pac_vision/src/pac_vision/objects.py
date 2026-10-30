"""물체 좌표 계산 (numpy만). 물체 좌표 = base_ref 기준 윗면 중심 (x, y, z_top).

책상면이 z = 0이므로 z > 최소 높이인 픽셀만 남기면 책상이 빠진다.
카메라가 비스듬히 보면 옆면이 같이 보여, 단순 중앙값은 윗면보다 낮고 x·y가 카메라 쪽으로 치우친다.
그래서 z_top = 물체 픽셀 높이의 상위 백분위, x·y = z_top 근처(윗면 띠) 픽셀의 중앙값.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cam2base import CameraModel, clip_roi


@dataclass
class TopCenter:
    valid: bool
    xyz: np.ndarray             # (x, y, z_top) base_ref, m. valid가 False면 nan
    n_object: int               # 책상면 위 픽셀 수
    n_top: int                  # 윗면 띠 픽셀 수
    top_mask: np.ndarray        # 입력 점 중 윗면 띠에 쓴 점 (bool)
    object_mask: np.ndarray     # 입력 점 중 책상면 위 점 (bool)
    reason: str = ""            # few_pixels


def top_center(points_base, min_height_m, max_height_m, top_percentile, top_band_m, min_pixels) -> TopCenter:
    p = np.asarray(points_base, dtype=float).reshape(-1, 3)
    z = p[:, 2]
    obj = (z > min_height_m) & (z < max_height_m)
    n_obj = int(np.count_nonzero(obj))
    if n_obj < min_pixels:
        return TopCenter(False, np.full(3, np.nan), n_obj, 0, np.zeros(len(p), bool), obj, "few_pixels")
    z_top = float(np.percentile(z[obj], top_percentile))
    top = obj & (z >= z_top - top_band_m)
    xy = np.median(p[top, :2], axis=0)
    return TopCenter(True, np.array([xy[0], xy[1], z_top]), n_obj, int(np.count_nonzero(top)), top, obj)


def box_to_depth_roi(box_xyxy, cam: CameraModel, rgb_size=None, shrink=0.1, mirrored=False, z_m=None):
    """화면 RGB 박스 (x1, y1, x2, y2) → depth 픽셀 사각형 (u0, v0, u1, v1). 테두리를 shrink 비율만큼 줄인다.

    z_m: 물체의 대략적인 광학 Z(m). 거리 식(d/Z 항)일 때만 쓰인다 (박스 아래 가운데의 책상면 Z로 충분:
    높이 10 cm 물체면 1 px 미만 차이). 거리 식인데 주지 않으면 z_ref_m (경고)."""
    x1, y1, x2, y2 = (float(a) for a in box_xyxy)
    mx, my = (x2 - x1) * shrink, (y2 - y1) * shrink
    z = None if z_m is None else np.full(2, float(z_m))
    us, vs = cam.rgb_to_depth(np.array([x1 + mx, x2 - mx]), np.array([y1 + my, y2 - my]), rgb_size, mirrored, z_m=z)
    roi = clip_roi(us.min(), vs.min(), us.max(), vs.max(), cam.width, cam.height)
    if roi[0] >= roi[2] or roi[1] >= roi[3]:
        return None
    return roi


def nearest_detection(last_xy, last_cls, candidates, max_dist_m):
    """추적 ID를 잃었을 때: 마지막 선택 위치에서 가장 가까운 같은 클래스 검출의 번호. 없으면 None.

    candidates: [(클래스, xyz 또는 None), ...]  (좌표 없는 검출은 건너뜀)
    """
    best_i, best_d = None, float(max_dist_m)
    last_xy = np.asarray(last_xy, dtype=float)[:2]
    for i, (cls, xyz) in enumerate(candidates):
        if cls != last_cls or xyz is None:
            continue
        xyz = np.asarray(xyz, dtype=float)
        if not np.all(np.isfinite(xyz)):
            continue
        d = float(np.linalg.norm(xyz[:2] - last_xy))
        if d <= best_d:
            best_i, best_d = i, d
    return best_i
