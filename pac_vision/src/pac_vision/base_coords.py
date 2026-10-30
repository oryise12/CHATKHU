"""클릭한 RGB 픽셀의 base_ref 좌표 두 가지와 책상면 격자 (scripts/show_base_coords.py용). numpy만 쓴다.

새 계산은 없다. Cam2Base의 함수를 그대로 부른다.
- P (책상면): Cam2Base.pixel_to_desk — 보정 책상면(base z = 0)과 클릭 광선의 교점. calibrate.py --check가
  "다른 점 클릭"에서 첫 단계로 부르는 함수다 (--check는 그다음 클릭 주변 depth로 평면을 다시 맞춘다).
- D (depth): Cam2Base.pixel_to_base — 클릭 → depth 픽셀 → 주변 중앙값 depth → 역투영. 높이가 있는 점용.
- 격자: 화면 네 변을 책상면에 투영한 범위 안에서 base_to_pixel로 되돌려 그린다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cam2base import Cam2Base

D_REASONS = {"no_depth": "depth 없음", "out_of_image": "depth 영상 밖",
             "too_close": "depth 너무 가까움", "too_far": "depth 너무 멂"}

_EDGE_SAMPLES = 64      # 화면 변 하나를 책상면에 투영할 때 나눌 점 수 (격자 범위 계산의 해상도일 뿐, 기준값 아님)


@dataclass
class ClickCoords:
    u: float
    v: float
    p: np.ndarray               # 책상면 교점 base (3,), 광선이 책상면과 만나지 않으면 nan
    d: np.ndarray               # depth 역투영 base (3,), 없으면 nan
    d_reason: str               # D가 없을 때 이유 (D_REASONS 키), 있으면 ""
    depth_px: tuple             # D 계산에 쓴 depth 픽셀 (ud, vd)
    valid_frames: int | None    # 그 depth 픽셀에서 0이 아니었던 프레임 수. 모르면 None (저장된 세션)

    @property
    def p_ok(self) -> bool:
        return bool(np.all(np.isfinite(self.p)))

    @property
    def d_ok(self) -> bool:
        return not self.d_reason


def click_coords(conv: Cam2Base, u, v, depth_mm, half: int, rgb_size=None, valid_frac=None,
                 n_frames: int | None = None) -> ClickCoords:
    """화면 RGB 픽셀 하나 → P, D. valid_frac: AstraDepth.read_average가 돌려준 픽셀별 유효 비율 (있으면)."""
    p = conv.pixel_to_desk(u, v, rgb_size=rgb_size)[0][0]
    r = conv.pixel_to_base(u, v, depth_mm, rgb_size=rgb_size, half=half)
    frames = None
    if valid_frac is not None and n_frames and len(r.depth_px) == 2:
        iu, iv = int(round(r.depth_px[0])), int(round(r.depth_px[1]))
        if 0 <= iv < valid_frac.shape[0] and 0 <= iu < valid_frac.shape[1]:
            frames = int(round(float(valid_frac[iv, iu]) * n_frames))
    return ClickCoords(float(u), float(v), np.asarray(p, float), np.asarray(r.xyz, float), r.reason,
                       tuple(r.depth_px), frames)


def desk_extent(conv: Cam2Base, rgb_size, far_max_m: float):
    """화면 네 변을 책상면에 투영한 범위 (x0, x1, y0, y1) base m. 없으면 None.

    광선이 책상면과 만나지 않거나 카메라 Z가 far_max_m보다 먼 점은 버린다 (= 그 모서리를 far_max에서 자름)."""
    w, h = rgb_size
    s = np.linspace(0.0, 1.0, _EDGE_SAMPLES)
    us = np.r_[s * (w - 1), np.full_like(s, w - 1), s * (w - 1), np.zeros_like(s)]
    vs = np.r_[np.zeros_like(s), s * (h - 1), np.full_like(s, h - 1), s * (h - 1)]
    pts = conv.pixel_to_desk(us, vs, rgb_size=rgb_size)[0]
    z_cam = conv.base_to_camera(pts)[:, 2]
    ok = np.all(np.isfinite(pts), axis=1) & (z_cam > 0) & (z_cam <= far_max_m)
    if not ok.any():
        return None
    q = pts[ok]
    return float(q[:, 0].min()), float(q[:, 0].max()), float(q[:, 1].min()), float(q[:, 1].max())


def grid_polylines(conv: Cam2Base, extent, step_m: float, rgb_size, far_max_m: float, per_step: int = 10):
    """책상면 격자(간격 step_m, 0을 지나는 선)를 화면 픽셀 꺾은선 목록으로. 화면 밖·far_max 너머에서 끊는다."""
    if extent is None or step_m <= 0:
        return []
    x0, x1, y0, y1 = extent
    w, h = rgb_size
    xs = np.arange(np.ceil(x0 / step_m), np.floor(x1 / step_m) + 1) * step_m
    ys = np.arange(np.ceil(y0 / step_m), np.floor(y1 / step_m) + 1) * step_m
    ds = step_m / per_step
    lines = []
    for fixed, lo, hi, along_x in [(x, y0, y1, False) for x in xs] + [(y, x0, x1, True) for y in ys]:
        t = np.arange(lo, hi + ds / 2, ds)
        if t.size < 2:
            continue
        base = np.stack([t, np.full_like(t, fixed)] if along_x else [np.full_like(t, fixed), t], axis=1)
        base = np.c_[base, np.zeros(len(t))]
        z_cam = conv.base_to_camera(base)[:, 2]
        px = conv.base_to_pixel(base, rgb_size=rgb_size)
        ok = (np.all(np.isfinite(px), axis=1) & (z_cam > 0) & (z_cam <= far_max_m)
              & (px[:, 0] >= 0) & (px[:, 0] <= w - 1) & (px[:, 1] >= 0) & (px[:, 1] <= h - 1))
        start = None
        for i in range(len(ok) + 1):            # 연속 구간마다 꺾은선 하나
            if i < len(ok) and ok[i]:
                start = i if start is None else start
            elif start is not None:
                if i - start >= 2:
                    lines.append(px[start:i])
                start = None
    return lines
