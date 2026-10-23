"""카메라 depth → base_ref 좌표 변환과 캘리브레이션 계산. numpy만 쓴다 (cv2·rclpy·openni 없음).

좌표 규약 (REP-103, 단위 m)
- 카메라 광학 좌표계: +X 오른쪽, +Y 아래, +Z 정면 (depth 좌우반전 보정 후)
- base_ref: +X 앞, +Y 왼쪽, +Z 위, 책상면 z = 0
- P_base = R @ P_cam + t  (t = base 좌표로 본 카메라 위치)

픽셀 좌표 세 가지
- 화면 RGB 픽셀: 실제로 보이는 영상 (크기 rgb_size, 좌우반전 화면일 수 있음)
- 기준 RGB 픽셀: Astra RGB 640x480(4:3 전체 화면) 기준
- depth 픽셀: 좌우반전 보정한 depth 640x480

16:9 화면(1280x720 등)은 4:3 화면의 위아래를 잘라낸 가운데 띠다 (2026-10-02 측정).
그래서 화면 → 기준 변환은 "가로 비율로 축소, 세로는 같은 비율로 축소한 뒤 남는 위쪽 여백만큼 더함"
(640x480 기준 1280x720이면 0.5배 후 +60 px).

기준 RGB → depth 픽셀 (거리 식)
  u_d = sx*u + (ax + bx*u + cx*v + dx/Z)
  v_d = sy*v + (ay + by*u + cy*v + dy/Z)
- (u, v): 기준 RGB 픽셀. Z: 그 점의 카메라 광학 Z(m) = depth_mm/1000 - z_offset_m (Z_OFFSET 적용 후).
  check_camera.py가 쌍 CSV에 남기는 Z, 책상 평면 교점의 Z, 물체 점의 Z가 모두 이 정의다 (backproject 기준).
- b = c = d = 0이면 예전의 고정 dx·dy와 같다. 설정 파일의 옛 형식(dx: 23)도 그대로 읽는다.
- d가 0이 아니면 Z가 필요하다. 책상 위 점은 책상 평면과의 교점으로 Z를 구해 고정점 반복한다 (rgb_to_depth_on_plane).
  Z를 모르면 z_ref_m(기본 1.0 m)을 쓰고 경고를 남긴다.
"""
from __future__ import annotations

import warnings
from dataclasses import dataclass, field

import numpy as np


class CalibrationError(ValueError):
    """계산을 끝낼 수 없을 때. reason은 화면 문구를 고르는 키, indices는 문제 된 클릭 번호(0=base)."""

    def __init__(self, reason: str, message: str, indices: list[int] | None = None):
        super().__init__(message)
        self.reason = reason
        self.indices = indices or []


# ---------------------------------------------------------------------------
# 카메라 모델: 픽셀 변환, 역투영, 투영
# ---------------------------------------------------------------------------
def rgb_depth_coef(value) -> tuple:
    """설정의 dx 또는 dy → (a, b, c, d). 옛 형식(숫자 하나)은 (값, 0, 0, 0)."""
    if isinstance(value, dict):
        return tuple(float(value.get(k, 0.0)) for k in ("a", "b", "c", "d"))
    return (float(value), 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int          # depth 영상 크기
    height: int
    z_offset_m: float
    min_m: float
    max_m: float
    rgb_width: int      # 기준 RGB 크기 (640x480)
    rgb_height: int
    sx: float           # 기준 RGB → depth: u_d = sx*u + (a + b*u + c*v + d/Z), v_d도 같은 꼴 (모듈 설명 참고)
    sy: float
    dx_coef: tuple      # (a, b, c, d)
    dy_coef: tuple
    z_ref_m: float = 1.0    # Z를 모를 때 쓰는 값 (쓰이면 경고)

    @classmethod
    def from_config(cls, cam: dict) -> "CameraModel":
        d, r, m = cam["depth"], cam["rgb"], cam["rgb_to_depth"]
        return cls(fx=float(d["fx"]), fy=float(d["fy"]), cx=float(d["cx"]), cy=float(d["cy"]),
                   width=int(d["width"]), height=int(d["height"]),
                   z_offset_m=float(d["z_offset_m"]), min_m=float(d["min_m"]), max_m=float(d["max_m"]),
                   rgb_width=int(r["width"]), rgb_height=int(r["height"]),
                   sx=float(m["sx"]), sy=float(m["sy"]),
                   dx_coef=rgb_depth_coef(m["dx"]), dy_coef=rgb_depth_coef(m["dy"]),
                   z_ref_m=float(m.get("z_ref_m", 1.0)))

    @property
    def has_z_term(self) -> bool:
        """거리 항(d)이 있어 Z가 필요한가."""
        return self.dx_coef[3] != 0.0 or self.dy_coef[3] != 0.0

    @property
    def is_fixed_offset(self) -> bool:
        """b = c = d = 0 (예전의 고정 dx·dy)."""
        return not any(self.dx_coef[1:]) and not any(self.dy_coef[1:])

    def rgb_depth_text(self) -> str:
        """로그용 한 줄."""
        def one(name, c):
            if not any(c[1:]):
                return f"{name} {c[0]:+.1f}"
            return f"{name} = {c[0]:+.2f} {c[1]:+.5f}·u {c[2]:+.5f}·v {c[3]:+.2f}/Z"
        return f"{one('dx', self.dx_coef)}, {one('dy', self.dy_coef)}, sx {self.sx}, sy {self.sy}"

    def _inv_z(self, z_m):
        if not self.has_z_term:
            return 0.0
        if z_m is None:
            warnings.warn(f"rgb_to_depth: 점의 Z를 몰라 z_ref_m = {self.z_ref_m} m를 썼습니다 "
                          "(거리 식의 d/Z 항이 근사됨)", RuntimeWarning, stacklevel=3)
            z_m = self.z_ref_m
        return 1.0 / np.asarray(z_m, dtype=float)

    # --- 화면 RGB ↔ 기준 RGB ↔ depth 픽셀 ---
    def screen_to_ref(self, u, v, rgb_size=None, mirrored=False):
        """화면 RGB 픽셀 → 기준 RGB(640x480) 픽셀. 16:9는 가운데 띠로 본다."""
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        w, h = rgb_size if rgb_size is not None else (self.rgb_width, self.rgb_height)
        if mirrored:
            u = (w - 1) - u
        s = self.rgb_width / float(w)
        return u * s, v * s + (self.rgb_height - h * s) / 2.0

    def ref_to_screen(self, ur, vr, rgb_size=None, mirrored=False):
        ur = np.asarray(ur, dtype=float)
        vr = np.asarray(vr, dtype=float)
        w, h = rgb_size if rgb_size is not None else (self.rgb_width, self.rgb_height)
        s = self.rgb_width / float(w)
        u = ur / s
        v = (vr - (self.rgb_height - h * s) / 2.0) / s
        if mirrored:
            u = (w - 1) - u
        return u, v

    def rgb_to_depth(self, u, v, rgb_size=None, mirrored=False, z_m=None):
        """화면 RGB 픽셀 → depth 픽셀. z_m: 그 점의 광학 Z(m). 거리 항이 있는데 없으면 z_ref_m (경고)."""
        ur, vr = self.screen_to_ref(u, v, rgb_size, mirrored)
        ax, bx, cx_, dx_ = self.dx_coef
        ay, by, cy_, dy_ = self.dy_coef
        if self.is_fixed_offset:                 # 예전 식 그대로 (옛 세션과 결과가 비트 단위로 같게)
            return self.sx * ur + ax, self.sy * vr + ay
        iz = self._inv_z(z_m)
        return (self.sx * ur + ax + bx * ur + cx_ * vr + dx_ * iz,
                self.sy * vr + ay + by * ur + cy_ * vr + dy_ * iz)

    def depth_to_rgb(self, ud, vd, rgb_size=None, mirrored=False, z_m=None):
        """depth 픽셀 → 화면 RGB 픽셀 (rgb_to_depth의 역). 거리 식이면 z_m이 필요하다."""
        ud = np.asarray(ud, dtype=float)
        vd = np.asarray(vd, dtype=float)
        ax, bx, cx_, dx_ = self.dx_coef
        ay, by, cy_, dy_ = self.dy_coef
        if self.is_fixed_offset:
            ur = (ud - ax) / self.sx
            vr = (vd - ay) / self.sy
        else:
            iz = self._inv_z(z_m)
            # [sx+bx  cx ] [ur]   [ud - ax - dx/Z]
            # [by   sy+cy] [vr] = [vd - ay - dy/Z]
            p, q = ud - ax - dx_ * iz, vd - ay - dy_ * iz
            a11, a12, a21, a22 = self.sx + bx, cx_, by, self.sy + cy_
            det = a11 * a22 - a12 * a21
            ur = (a22 * p - a12 * q) / det
            vr = (a11 * q - a21 * p) / det
        return self.ref_to_screen(ur, vr, rgb_size, mirrored)

    def rgb_to_depth_on_plane(self, u, v, plane, rgb_size=None, mirrored=False, iters=10, tol_px=0.01):
        """책상 평면 위 점: 평면 교점의 Z로 고정점 반복. 반환 (ud, vd, 카메라 좌표 (N, 3)).

        Z_ref로 depth 픽셀 → 그 광선과 평면의 교점 Z → 그 Z로 다시 depth 픽셀 … 변화가 tol_px 미만이 될 때까지.
        거리 항이 없으면 한 번에 끝난다.
        """
        u = np.atleast_1d(np.asarray(u, dtype=float))
        v = np.atleast_1d(np.asarray(v, dtype=float))
        z0 = np.full(u.shape, self.z_ref_m) if self.has_z_term else None
        ud, vd = self.rgb_to_depth(u, v, rgb_size, mirrored, z_m=z0)
        pts = intersect_ray_plane(self.ray(ud, vd), plane)
        if not self.has_z_term:
            return ud, vd, pts
        for _ in range(int(iters)):
            z = pts[:, 2]
            z = np.where(np.isfinite(z) & (z > 0), z, self.z_ref_m)
            ud2, vd2 = self.rgb_to_depth(u, v, rgb_size, mirrored, z_m=z)
            change = float(np.nanmax(np.abs(np.r_[ud2 - ud, vd2 - vd])))
            ud, vd = ud2, vd2
            pts = intersect_ray_plane(self.ray(ud, vd), plane)
            if change < tol_px:
                break
        return ud, vd, pts

    def rgb_to_depth_on_depth(self, u, v, depth_mm, half=2, rgb_size=None, mirrored=False, iters=5):
        """점마다 그 자리 depth(주변 중앙값)의 Z로 반복. depth가 없는 점은 z_ref_m 결과로 둔다 (경고 없음)."""
        u = np.atleast_1d(np.asarray(u, dtype=float))
        v = np.atleast_1d(np.asarray(v, dtype=float))
        z = np.full(u.shape, self.z_ref_m)
        ud, vd = self.rgb_to_depth(u, v, rgb_size, mirrored, z_m=z if self.has_z_term else None)
        if not self.has_z_term:
            return ud, vd
        for _ in range(int(iters)):
            zmm = np.array([patch_median(depth_mm, a, b, half)
                            if 0 <= a < self.width and 0 <= b < self.height else 0.0 for a, b in zip(ud, vd)])
            z = np.where(zmm > 0, self.depth_m(zmm), z)
            ud, vd = self.rgb_to_depth(u, v, rgb_size, mirrored, z_m=z)
        return ud, vd

    # --- 역투영·투영 ---
    def depth_m(self, depth_mm):
        """depth 원본(mm) → 광학 Z(m). Z_OFFSET 보정 포함."""
        return np.asarray(depth_mm, dtype=float) / 1000.0 - self.z_offset_m

    def ray(self, ud, vd):
        """depth 픽셀을 지나는 광선 방향 (Z 성분 = 1). 모양 (..., 3)."""
        ud = np.asarray(ud, dtype=float)
        vd = np.asarray(vd, dtype=float)
        return np.stack([(ud - self.cx) / self.fx, (vd - self.cy) / self.fy, np.ones_like(ud)], axis=-1)

    def backproject(self, ud, vd, depth_mm):
        """depth 픽셀 + depth(mm) → 카메라 좌표(m). depth 0(측정 실패)은 호출하는 쪽에서 거른다."""
        z = self.depth_m(depth_mm)
        return self.ray(ud, vd) * z[..., None]

    def project(self, p_cam):
        """카메라 좌표 → depth 픽셀 (ud, vd). 카메라 뒤(Z<=0)는 nan."""
        p = np.asarray(p_cam, dtype=float)
        z = p[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            ud = np.where(z > 0, p[..., 0] / z * self.fx + self.cx, np.nan)
            vd = np.where(z > 0, p[..., 1] / z * self.fy + self.cy, np.nan)
        return ud, vd


# ---------------------------------------------------------------------------
# depth 영상 도우미
# ---------------------------------------------------------------------------
def patch_median(depth_mm, ud, vd, half=2) -> float:
    """(ud, vd) 주변 (2*half+1)^2 픽셀에서 0(측정 실패)을 뺀 중앙값(mm). 없으면 0."""
    h, w = depth_mm.shape
    u, v = int(round(float(ud))), int(round(float(vd)))
    x0, x1 = max(0, u - half), min(w, u + half + 1)
    y0, y1 = max(0, v - half), min(h, v + half + 1)
    if x0 >= x1 or y0 >= y1:
        return 0.0
    vals = np.asarray(depth_mm[y0:y1, x0:x1], dtype=float).ravel()
    vals = vals[vals > 0]
    return float(np.median(vals)) if vals.size else 0.0


def average_depth(frames, min_valid_fraction=0.5):
    """여러 depth 프레임(mm)을 픽셀별로 평균 (0은 빼고). 유효 프레임 비율이 기준 미만이면 0.

    반환: (평균 depth float32 mm, 픽셀별 유효 비율 float32)
    """
    stack = np.asarray(frames, dtype=np.float32)
    valid = stack > 0
    count = valid.sum(axis=0)
    total = np.where(valid, stack, 0).sum(axis=0, dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        avg = np.where(count > 0, total / np.maximum(count, 1), 0.0)
    frac = count / float(stack.shape[0])
    avg = np.where(frac >= min_valid_fraction, avg, 0.0)
    return avg.astype(np.float32), frac.astype(np.float32)


def clip_roi(u0, v0, u1, v1, width, height):
    """(u0, v0, u1, v1) 정수 사각형, 오른쪽·아래는 포함하지 않음."""
    u0 = int(max(0, np.floor(u0)))
    v0 = int(max(0, np.floor(v0)))
    u1 = int(min(width, np.ceil(u1) + 1))
    v1 = int(min(height, np.ceil(v1) + 1))
    return u0, v0, u1, v1


def roi_points(depth_mm, cam: CameraModel, roi, step=1):
    """사각형 안의 유효 depth 픽셀을 카메라 좌표로. 반환: (점 (N,3), ud (N,), vd (N,), 전체 픽셀 수)"""
    u0, v0, u1, v1 = roi
    sub = np.asarray(depth_mm[v0:v1:step, u0:u1:step], dtype=float)
    vv, uu = np.mgrid[v0:v1:step, u0:u1:step]
    ok = sub > 0
    ud, vd = uu[ok].astype(float), vv[ok].astype(float)
    return cam.backproject(ud, vd, sub[ok]), ud, vd, int(sub.size)


# ---------------------------------------------------------------------------
# 평면 맞춤, 광선-평면 교차
# ---------------------------------------------------------------------------
@dataclass
class Plane:
    normal: np.ndarray      # 단위 벡터, 카메라 쪽을 향함 (d > 0)
    d: float                # normal · p + d = 0
    rms_m: float            # 인라이어의 평면 거리 RMS
    inlier_ratio: float
    n_points: int

    def distance(self, p):
        return np.asarray(p, dtype=float) @ self.normal + self.d


def fit_plane(points, iters=300, inlier_m=0.005, max_points=20000, seed=0) -> Plane:
    """RANSAC으로 가장 많은 점이 올라가는 평면을 찾고, 인라이어로 최소제곱(SVD) 정제."""
    p = np.asarray(points, dtype=float)
    n_all = len(p)
    if n_all < 3:
        raise CalibrationError("no_plane", "평면을 맞출 depth 점이 부족합니다")
    rng = np.random.default_rng(seed)
    cand = p if n_all <= max_points else p[rng.choice(n_all, max_points, replace=False)]
    best_count, best = -1, None
    for _ in range(int(iters)):
        i, j, k = rng.integers(0, len(cand), 3)
        if i == j or j == k or i == k:
            continue
        a, b, c = cand[i], cand[j], cand[k]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n = n / norm
        count = int(np.count_nonzero(np.abs(cand @ n - n @ a) < inlier_m))
        if count > best_count:
            best_count, best = count, (n, -float(n @ a))
    if best is None:
        raise CalibrationError("no_plane", "평면을 찾지 못했습니다")
    n, d = best
    for _ in range(2):                      # 인라이어로 정제 → 인라이어 다시 고르기
        mask = np.abs(p @ n + d) < inlier_m
        if np.count_nonzero(mask) < 3:
            break
        q = p[mask]
        centroid = q.mean(axis=0)
        _, _, vt = np.linalg.svd(q - centroid, full_matrices=False)
        n = vt[-1]
        d = -float(n @ centroid)
    if d < 0:                               # 법선이 카메라(원점) 쪽을 향하게
        n, d = -n, -d
    mask = np.abs(p @ n + d) < inlier_m
    res = p[mask] @ n + d
    rms = float(np.sqrt(np.mean(res ** 2))) if res.size else float("nan")
    return Plane(normal=n, d=float(d), rms_m=rms, inlier_ratio=float(mask.mean()), n_points=n_all)


def intersect_ray_plane(dirs, plane: Plane):
    """원점에서 나가는 광선과 평면의 교점. 평행하거나 카메라 뒤면 nan."""
    dirs = np.atleast_2d(np.asarray(dirs, dtype=float))
    denom = dirs @ plane.normal
    with np.errstate(divide="ignore", invalid="ignore"):
        s = -plane.d / denom
    s = np.where((np.abs(denom) > 1e-9) & (s > 0), s, np.nan)
    return dirs * s[:, None]


# ---------------------------------------------------------------------------
# Kabsch, 검사
# ---------------------------------------------------------------------------
def kabsch(a, b):
    """b ≈ R @ a + t 인 강체 변환 (SVD). det(R) < 0(반사)이면 마지막 축을 뒤집어 회전으로 만든다."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    h = (a - ca).T @ (b - cb)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt = vt.copy()
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, cb - r @ ca


def rms(errors) -> float:
    e = np.asarray(errors, dtype=float)
    return float(np.sqrt(np.mean(e ** 2)))


def handedness_ok(corners_cam, normal_to_cam, corners_base):
    """반사 감지 ①: 법선 기준 손잡이 비교.

    5점이 한 평면에 있으면 SVD 셋째 특이값이 0이라 반사가 det(R)에 드러나지 않는다.
    그래서 카메라 쪽 법선(= base의 +Z 쪽)을 셋째 축으로 써서 det[c2-c1, c4-c1, n]과
    det[b2-b1, b4-b1, +Z]의 부호를 비교한다. 번호 순서가 뒤집혔거나 좌우반전이 빠지면 부호가 다르다.
    반환: (통과 여부, 카메라 쪽 det, base 쪽 det)
    """
    c = np.asarray(corners_cam, dtype=float)
    b = np.asarray(corners_base, dtype=float)
    det_c = float(np.linalg.det(np.stack([c[1] - c[0], c[-1] - c[0], np.asarray(normal_to_cam, float)])))
    det_b = float(np.linalg.det(np.stack([b[1] - b[0], b[-1] - b[0], np.array([0.0, 0.0, 1.0])])))
    return (det_c > 0) == (det_b > 0), det_c, det_b


def camera_above_desk(t) -> bool:
    """반사 감지 ②: 카메라 위치(= t)의 base z가 책상면(z=0)보다 위인가."""
    return float(np.asarray(t, dtype=float)[2]) > 0.0


def z_distribution(z_base, depth_z, band_m=0.02, raised_m=0.01):
    """책상면 base z 분포를 가까운 쪽 / 먼 쪽(카메라 Z 중앙값 기준)으로 나눠 중앙값과 5-95 백분위.

    책상면 ±band_m 안의 점만 쓴다 (책상 너머 바닥·벽·물체가 통계에 섞이지 않게).
    raised: 그중 raised_m보다 높게 떠 보이는 점의 비율 (책상이 물체로 세질 위험). n_outside: ±band_m 밖이라 뺀 점 수.
    """
    z_base = np.asarray(z_base, dtype=float)
    depth_z = np.asarray(depth_z, dtype=float)
    out = {}
    keep = np.abs(z_base) < band_m
    if not keep.any():
        return out
    zk, dk = z_base[keep], depth_z[keep]
    split = float(np.median(dk))
    n_out_all = ~keep
    for key, sel, sel_all in (("near", dk <= split, depth_z <= split), ("far", dk > split, depth_z > split)):
        z = zk[sel]
        if z.size:
            out[key] = {"median": float(np.median(z)), "p5": float(np.percentile(z, 5)),
                        "p95": float(np.percentile(z, 95)), "n": int(z.size),
                        "raised": float(np.mean(z > raised_m)), "n_outside": int(np.count_nonzero(n_out_all & sel_all))}
    return out


def fit_desk_plane_near(points_cam, conv_R, conv_t, band_m, iters=300, inlier_m=0.005, max_points=20000, seed=0):
    """보정 책상면(base z = 0)에서 ±band_m 안의 점만으로 평면 맞춤 (책상 끝 너머 바닥을 잡지 않게). 점이 부족하면 None."""
    p = np.asarray(points_cam, dtype=float)
    z = p @ np.asarray(conv_R, dtype=float)[2] + float(np.asarray(conv_t, dtype=float)[2])
    q = p[np.abs(z) < band_m]
    if len(q) < 3:
        return None
    return fit_plane(q, iters, inlier_m, max_points, seed)


def plane_from_points(points) -> Plane:
    """적은 점(클릭 5점 등)을 지나는 최소제곱 평면, 법선은 카메라 쪽."""
    p = np.asarray(points, dtype=float)
    centroid = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - centroid, full_matrices=False)
    n = vt[-1]
    d = -float(n @ centroid)
    if d < 0:
        n, d = -n, -d
    res = p @ n + d
    return Plane(normal=n, d=d, rms_m=rms(res), inlier_ratio=1.0, n_points=len(p))


# ---------------------------------------------------------------------------
# 캘리브레이션
# ---------------------------------------------------------------------------
@dataclass
class Check:
    key: str            # base_error / a4_sides / reflection / a4_size / desk_z
    ok: bool
    value: float
    limit: float
    blocking: bool      # False면 경고만


@dataclass
class CalibResult:
    mode: str
    R: np.ndarray
    t: np.ndarray
    clicks_rgb: np.ndarray          # (5, 2) 화면 RGB 픽셀
    clicks_depth: np.ndarray        # (5, 2) depth 픽셀
    cam_points: np.ndarray          # (5, 3) 카메라 좌표
    base_points: np.ndarray         # (5, 3) 기준 좌표 (설정 값)
    residuals_m: np.ndarray         # 5점 최종 변환의 점별 오차
    holdout_m: float                # 꼭짓점 4점 변환으로 본 base 점 오차
    sides_m: np.ndarray             # A4 변 길이 (1-2, 2-3, 3-4, 4-1), 카메라 좌표에서 잼
    sides_expected_m: np.ndarray
    sides_px: np.ndarray            # 변마다 화면 길이 (px)
    sides_tol_m: np.ndarray         # 변마다 적용한 허용 오차 = max(고정값, 그 변 방향 N px 길이)
    a4_short_px: float              # 가장 짧게 보이는 변의 화면 길이
    a4_short_side: int              # 그 변 번호 (0: 1-2, 1: 2-3, 2: 3-4, 3: 4-1)
    a4_short_m_per_px: float        # 그 변 방향 1 px의 실제 길이 (m)
    roi: tuple                      # 평면 맞춤 영역 (depth 픽셀 u0, v0, u1, v1)
    roi_valid_ratio: float
    plane_rms_m: float
    plane_inlier_ratio: float
    plane_normal: np.ndarray        # 카메라 쪽 법선 (카메라 좌표)
    tilt_deg: float                 # 책상면 법선과 광축 사이 각
    handedness: bool
    camera_above: bool
    desk_z: dict
    checks: list = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.ok for c in self.checks if c.blocking)

    def check(self, key) -> Check:
        return next(c for c in self.checks if c.key == key)

    def summary(self) -> dict:
        """yaml·세션 저장과 --replay 비교용 (파이썬 기본형만)."""
        def f(x):
            return np.asarray(x, dtype=float).tolist()
        return {
            "mode": self.mode,
            "R": f(self.R), "t": f(self.t),
            "camera_points": f(self.cam_points),
            "residuals_m": f(self.residuals_m),
            "rms_m": rms(self.residuals_m),
            "holdout_m": float(self.holdout_m),
            "sides_m": f(self.sides_m),
            "sides_px": f(self.sides_px),
            "sides_tol_m": f(self.sides_tol_m),
            "a4_short_px": float(self.a4_short_px),
            "a4_short_side": int(self.a4_short_side),
            "a4_short_m_per_px": float(self.a4_short_m_per_px),
            "roi": [int(x) for x in self.roi],
            "roi_valid_ratio": float(self.roi_valid_ratio),
            "plane_rms_m": float(self.plane_rms_m),
            "tilt_deg": float(self.tilt_deg),
            "handedness": bool(self.handedness),
            "camera_above": bool(self.camera_above),
            "desk_z": self.desk_z,
            "checks": {c.key: {"ok": bool(c.ok), "value": float(c.value), "limit": float(c.limit),
                               "blocking": bool(c.blocking)} for c in self.checks},
            "passed": bool(self.passed),
        }


def layout_points(layout: dict) -> np.ndarray:
    """클릭 순서대로 기준 좌표 5점: base, 꼭짓점 1-4."""
    return np.array([layout["base_point"]] + list(layout["corners"]), dtype=float)


def solve_calibration(depth_mm, clicks_rgb, cam: CameraModel, layout: dict, thr: dict,
                      mode: str = "plane", rgb_size=None) -> CalibResult:
    """정지 화면의 평균 depth와 클릭 5점(base, 꼭짓점 1-4)으로 R, t와 검사 결과를 계산한다.

    plane: 클릭 5점을 감싸는 영역에서 책상 평면을 맞추고, 클릭 광선과 평면의 교점을 카메라 좌표로 쓴다.
    point: 클릭 점 주변 depth 중앙값으로 역투영한다.
    순서: 꼭짓점 4점 Kabsch → base 점 홀드아웃 → 5점 Kabsch(최종). 검사는 모두 계산해 checks에 담는다.
    """
    pc, cc, oc = thr["plane"], thr["calibration"], thr["objects"]
    clicks = np.asarray(clicks_rgb, dtype=float).reshape(-1, 2)
    if len(clicks) != 5:
        raise CalibrationError("clicks", f"클릭은 5점이어야 합니다 (받은 수 {len(clicks)})")
    depth_mm = np.asarray(depth_mm)
    # 거리 식(d/Z 항)이면 클릭의 Z가 필요하다: z_ref_m으로 영역을 잡고, 평면(또는 점 depth)의 Z로 고정점 반복한다.
    # 고정 dx·dy면 반복 없이 예전과 같은 계산이다.
    z0 = np.full(5, cam.z_ref_m) if cam.has_z_term else None
    ud, vd = cam.rgb_to_depth(clicks[:, 0], clicks[:, 1], rgb_size, z_m=z0)

    def check_inside(ud, vd):
        outside = [i for i in range(5) if not (0 <= ud[i] < cam.width and 0 <= vd[i] < cam.height)]
        if outside:
            raise CalibrationError("outside", "depth 영상 밖의 클릭이 있습니다", outside)

    def roi_plane(ud, vd):
        m = pc["roi_margin_px"]
        roi = clip_roi(ud.min() - m, vd.min() - m, ud.max() + m, vd.max() + m, cam.width, cam.height)
        p_roi, _, _, n_total = roi_points(depth_mm, cam, roi)
        plane = fit_plane(p_roi, pc["ransac_iters"], pc["inlier_m"], pc["max_points"], pc["seed"]) \
            if len(p_roi) >= 3 else None
        return roi, p_roi, n_total, plane

    check_inside(ud, vd)
    roi, p_roi, n_total, plane = roi_plane(ud, vd)

    if mode == "plane":
        if plane is None:
            raise CalibrationError("no_plane", "평면 맞춤 영역에 depth가 없습니다")
        if cam.has_z_term:
            ud, vd, _ = cam.rgb_to_depth_on_plane(clicks[:, 0], clicks[:, 1], plane, rgb_size)
            check_inside(ud, vd)
            roi, p_roi, n_total, plane = roi_plane(ud, vd)      # 옮겨진 클릭으로 영역을 다시 잡아 한 번 더
            if plane is None:
                raise CalibrationError("no_plane", "평면 맞춤 영역에 depth가 없습니다")
            ud, vd, cam_pts = cam.rgb_to_depth_on_plane(clicks[:, 0], clicks[:, 1], plane, rgb_size)
        else:
            cam_pts = intersect_ray_plane(cam.ray(ud, vd), plane)
        bad = [i for i in range(5) if not np.all(np.isfinite(cam_pts[i]))]
        if bad:
            raise CalibrationError("ray_plane", "클릭 광선이 책상 평면과 만나지 않습니다", bad)
    elif mode == "point":
        half = cc["point_patch_half_px"]
        if cam.has_z_term:
            ud, vd = cam.rgb_to_depth_on_depth(clicks[:, 0], clicks[:, 1], depth_mm, half, rgb_size)
            check_inside(ud, vd)
        z = np.array([patch_median(depth_mm, ud[i], vd[i], half) for i in range(5)])
        missing = [i for i in range(5) if z[i] <= 0]
        if missing:
            raise CalibrationError("no_depth", "depth가 없는 클릭이 있습니다", missing)
        cam_pts = cam.backproject(ud, vd, z)
    else:
        raise ValueError(f"알 수 없는 depth 모드: {mode}")
    valid_ratio = len(p_roi) / float(max(n_total, 1))

    base_pts = layout_points(layout)
    corners_c, corners_b = cam_pts[1:], base_pts[1:]
    nxt = [1, 2, 3, 0]
    sides = np.linalg.norm(corners_c[nxt] - corners_c, axis=1)
    sides_exp = np.linalg.norm(corners_b[nxt] - corners_b, axis=1)

    r4, t4 = kabsch(corners_c, corners_b)
    holdout = float(np.linalg.norm(r4 @ cam_pts[0] + t4 - base_pts[0]))
    r, t = kabsch(cam_pts, base_pts)
    residuals = np.linalg.norm(cam_pts @ r.T + t - base_pts, axis=1)

    normal_plane = plane if (mode == "plane" and plane is not None) else plane_from_points(cam_pts)
    hand, _, _ = handedness_ok(corners_c, normal_plane.normal, corners_b)
    above = camera_above_desk(t)

    # 변마다 화면 길이(px)와 그 방향 1 px의 실제 길이. 비스듬한 시점에서는 멀어지는 방향 변이 가장 짧게 보인다.
    px = clicks[1:]
    sides_px = np.linalg.norm(px[nxt] - px, axis=1)
    m_per_px = sides_exp / np.maximum(sides_px, 1e-9)
    sides_tol = np.maximum(cc["a4_side_tol_m"], cc["a4_side_tol_px"] * m_per_px)
    short_i = int(np.argmin(sides_px))
    a4_short_px = float(sides_px[short_i])
    side_err = np.abs(sides - sides_exp)
    worst = int(np.argmax(side_err / sides_tol))
    info_plane = plane if plane is not None else normal_plane
    tilt = float(np.degrees(np.arccos(min(1.0, abs(float(info_plane.normal[2]))))))

    desk = {}
    if len(p_roi):
        z_base = p_roi @ r[2] + t[2]
        desk = z_distribution(z_base, p_roi[:, 2], oc.get("desk_band_m", 0.02), oc["min_height_m"])

    min_h = oc["min_height_m"]
    far_p95 = desk.get("far", {}).get("p95", float("nan"))
    checks = [
        Check("base_error", holdout <= cc["base_error_max_m"], holdout, cc["base_error_max_m"], True),
        Check("a4_sides", bool(np.all(side_err <= sides_tol)), float(side_err[worst]), float(sides_tol[worst]), True),
        Check("reflection", bool(hand and above), float(t[2]), 0.0, True),
        Check("a4_size", a4_short_px >= cc["a4_short_side_min_px"], a4_short_px, cc["a4_short_side_min_px"], False),
        Check("desk_z", bool(np.isfinite(far_p95) and far_p95 < min_h), far_p95, min_h, False),
    ]
    return CalibResult(
        mode=mode, R=r, t=t, clicks_rgb=clicks, clicks_depth=np.stack([ud, vd], axis=1),
        cam_points=cam_pts, base_points=base_pts, residuals_m=residuals, holdout_m=holdout,
        sides_m=sides, sides_expected_m=sides_exp, sides_px=sides_px, sides_tol_m=sides_tol,
        a4_short_px=a4_short_px, a4_short_side=short_i, a4_short_m_per_px=float(m_per_px[short_i]), roi=roi,
        roi_valid_ratio=valid_ratio,
        plane_rms_m=plane.rms_m if plane is not None else float("nan"),
        plane_inlier_ratio=plane.inlier_ratio if plane is not None else float("nan"),
        plane_normal=info_plane.normal, tilt_deg=tilt, handedness=bool(hand), camera_above=bool(above),
        desk_z=desk, checks=checks)


# ---------------------------------------------------------------------------
# 보정 결과로 변환하기
# ---------------------------------------------------------------------------
@dataclass
class PixelResult:
    valid: bool
    xyz: np.ndarray             # base_ref 좌표 (m). valid가 False면 nan
    reason: str = ""            # out_of_image / no_depth / too_close / too_far
    depth_px: tuple = ()


class Cam2Base:
    """카메라 ↔ base_ref 변환. P_base = R @ P_cam + t."""

    def __init__(self, R, t, cam: CameraModel, info: dict | None = None):
        self.R = np.asarray(R, dtype=float)
        self.t = np.asarray(t, dtype=float)
        self.cam = cam
        self.info = info or {}

    def camera_to_base(self, p_cam):
        return np.asarray(p_cam, dtype=float) @ self.R.T + self.t

    def base_to_camera(self, p_base):
        return (np.asarray(p_base, dtype=float) - self.t) @ self.R

    def pixel_to_base(self, u, v, depth_mm, rgb_mirrored=False, rgb_size=None, half=2) -> PixelResult:
        """화면 RGB 픽셀 하나 → base_ref 좌표. depth는 주변 (2*half+1)^2 픽셀 중앙값."""
        nan = np.full(3, np.nan)
        ud, vd = self.cam.rgb_to_depth_on_depth(u, v, depth_mm, half, rgb_size, rgb_mirrored)
        ud, vd = float(ud[0]), float(vd[0])
        if not (0 <= ud < self.cam.width and 0 <= vd < self.cam.height):
            return PixelResult(False, nan, "out_of_image", (ud, vd))
        z_mm = patch_median(depth_mm, ud, vd, half)
        if z_mm <= 0:
            return PixelResult(False, nan, "no_depth", (ud, vd))
        z = float(self.cam.depth_m(z_mm))
        if z < self.cam.min_m:
            return PixelResult(False, nan, "too_close", (ud, vd))
        if z > self.cam.max_m:
            return PixelResult(False, nan, "too_far", (ud, vd))
        p = self.cam.backproject(ud, vd, z_mm)
        return PixelResult(True, self.camera_to_base(p), "", (ud, vd))

    def base_to_pixel(self, xyz, rgb_mirrored=False, rgb_size=None):
        """base_ref 좌표 → 화면 RGB 픽셀 (..., 2). 카메라 뒤는 nan."""
        pc = self.base_to_camera(xyz)
        ud, vd = self.cam.project(pc)
        u, v = self.cam.depth_to_rgb(ud, vd, rgb_size, rgb_mirrored, z_m=np.asarray(pc)[..., 2])
        return np.stack([u, v], axis=-1)

    def desk_plane(self) -> "Plane":
        """책상면(base z = 0)을 카메라 좌표의 평면으로. 법선은 카메라 쪽 (d = 카메라 높이 > 0)."""
        return Plane(normal=self.R[2].copy(), d=float(self.t[2]), rms_m=float("nan"), inlier_ratio=float("nan"),
                     n_points=0)

    def pixel_to_desk(self, u, v, rgb_mirrored=False, rgb_size=None, plane: "Plane | None" = None):
        """화면 RGB 픽셀 → 책상면(또는 주어진 평면) 위 점의 base 좌표 (N, 3)와 depth 픽셀.

        거리 식이면 평면 교점의 Z로 고정점 반복한다. plane을 주지 않으면 보정 결과의 책상면(z = 0)."""
        pl = plane if plane is not None else self.desk_plane()
        ud, vd, pts = self.cam.rgb_to_depth_on_plane(u, v, pl, rgb_size, rgb_mirrored)
        return self.camera_to_base(pts), ud, vd
