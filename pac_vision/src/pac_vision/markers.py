"""보정에 쓰지 않은 고정 표지(ArUco)로 보정이 지금도 맞는지 확인한다.

보정 결과(cam2base.yaml)의 `passed: true`는 그 순간 A4 5점이 맞았다는 뜻일 뿐, 그 뒤 카메라가 밀리거나 돌았는지는
말해 주지 않는다. 책상 위(보정 종이 밖, 로봇 작업 영역을 가리지 않는 곳)에 ArUco 표지를 붙이고 각 표지 중심의
base_ref 좌표를 줄자로 재서 config/markers.yaml에 적어 두면(보정 계산에는 쓰지 않는 독립 점) 실행 중에 다음을 잰다.

  - 이미지 오차 (px): 잰 좌표를 보정으로 화면에 투영한 점 ↔ 영상에서 찾은 표지 중심
  - 책상면 3D 오차 (mm): 영상에서 찾은 표지 중심을 보정의 책상면(z = 표지 높이)에 역투영한 점 ↔ 잰 좌표

카메라가 옆으로 밀리거나 돌면 이미지 오차가 곧바로 커진다 (책상면 높이만 보는 검사로는 놓칠 수 있다).

판정 (verify)
  verified     보이는 표지가 min_visible개 이상이고 모두 허용오차 안
  moved        보이는 표지 중 하나라도 허용오차 밖 (카메라 이동·회전, 표지 이동, 보정 오류 중 하나)
  not_visible  보이는 표지가 min_visible개보다 적다 (가림·조명·표지 없음): 검증할 수 없다
  no_config    config/markers.yaml이 없다: 검증할 수 없다
허용오차 기본값(8 px, 10 mm)은 계획의 출발값이다. 실제 카메라에서 잰 분포로 markers.yaml에서 바꾼다.

보정판(find_board): 캘리브레이션 기준점을 클릭 대신 ArUco 마커 중심으로 찾는다 (config/a4_layout_aruco.yaml).

좌표: base_ref (m, +X 앞 = A4 화살표, +Y 왼쪽, +Z 위, A4 대각선 교점 원점, 책상 z = 0).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .cam2base import Cam2Base, Plane

STATES = ("verified", "moved", "not_visible", "no_config")


@dataclass
class MarkerSet:
    dictionary: str = "DICT_4X4_50"
    size_m: float = 0.05                 # 인쇄한 검은 정사각형 한 변 (인쇄용 / 정보)
    min_visible: int = 2
    max_pixel_px: float = 8.0
    max_desk_mm: float = 10.0
    markers: dict[int, np.ndarray] = field(default_factory=dict)   # id -> base_ref (m)


def load_markers(path) -> MarkerSet:
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    ms = {}
    for k, v in (doc.get("markers") or {}).items():
        p = np.asarray(v["base_m"] if isinstance(v, dict) else v, float).reshape(-1)
        if p.size == 2:
            p = np.r_[p, 0.0]
        if p.size != 3 or not np.all(np.isfinite(p)):
            raise ValueError(f"markers.yaml: marker {k} needs base_m [x, y] or [x, y, z] (m)")
        ms[int(k)] = p
    return MarkerSet(dictionary=str(doc.get("dictionary", "DICT_4X4_50")), size_m=float(doc.get("size_m", 0.05)),
                     min_visible=int(doc.get("min_visible", 2)), max_pixel_px=float(doc.get("max_pixel_px", 8.0)),
                     max_desk_mm=float(doc.get("max_desk_mm", 10.0)), markers=ms)


def _dictionary(name: str):
    import cv2
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def square_center(corners) -> np.ndarray:
    """네 꼭짓점(순서대로)으로 정사각형 중심의 영상 위치: 두 대각선의 교점 (원근에서도 정확한 중심의 상)."""
    c = np.asarray(corners, float).reshape(4, 2)
    p, r = c[0], c[2] - c[0]
    q, s = c[1], c[3] - c[1]
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-9:
        return c.mean(axis=0)
    t = ((q - p)[0] * s[1] - (q - p)[1] * s[0]) / den
    return p + t * r


def detect_all(bgr, dictionary: str = "DICT_4X4_50", subpix: bool = False) -> list[tuple[int, np.ndarray]]:
    """영상 속 ArUco 표지 전부 [(id, (4, 2) 꼭짓점)] (화면 픽셀). 같은 번호가 여러 번 보이면 모두 담는다.
    subpix: 꼭짓점을 서브픽셀로 다듬는다 (보정판처럼 위치를 기준으로 쓸 때)."""
    import cv2
    params = cv2.aruco.DetectorParameters()
    if subpix:
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(_dictionary(dictionary), params)
    gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    return [] if ids is None else [(int(i), np.asarray(c, float).reshape(4, 2)) for c, i in zip(corners, ids.ravel())]


def detect_quads(bgr, dictionary: str = "DICT_4X4_50") -> dict[int, np.ndarray]:
    """영상 속 ArUco 표지의 네 꼭짓점 {id: (4, 2)} (화면 픽셀)."""
    return dict(detect_all(bgr, dictionary))


def detect(bgr, dictionary: str = "DICT_4X4_50") -> dict[int, np.ndarray]:
    """영상 속 ArUco 표지 중심 {id: (u, v)} (대각선 교점, 화면 픽셀)."""
    return {k: square_center(c) for k, c in detect_quads(bgr, dictionary).items()}


def shortest_side_px(quad) -> float:
    """표지가 화면에 보이는 가장 짧은 변 (px). 비스듬히 보면 짧아진다. 약 30 px보다 작으면 검출이 불안정하다."""
    q = np.asarray(quad, float).reshape(4, 2)
    return float(np.min(np.linalg.norm(q - np.roll(q, -1, axis=0), axis=1)))


@dataclass
class BoardDetection:
    """보정판(a4_layout_aruco.yaml) 마커 찾기 결과."""
    points: np.ndarray | None           # ids 순서의 마커 중심 (N, 2) 화면 픽셀. 모두 제대로 찾았을 때만
    quads: dict[int, np.ndarray]        # 한 번씩 찾은 보정판 마커의 꼭짓점
    missing: list[int]                  # 안 보이는 번호
    duplicated: list[int]               # 두 번 이상 보이는 번호 (어느 것이 보정판인지 모른다)
    small: list[int]                    # 화면에서 min_side_px보다 작게 보이는 번호


def find_board(bgr, ids, dictionary: str = "DICT_4X4_50", min_side_px: float = 30.0) -> BoardDetection:
    """보정판 마커(ids)의 중심을 ids 순서로 찾는다: 서브픽셀 꼭짓점의 대각선 교점.
    하나라도 안 보이거나, 같은 번호가 둘이거나, 너무 작게 보이면 points = None (이유는 나머지 필드)."""
    want = [int(i) for i in ids]
    by_id: dict[int, list] = {}
    for k, q in detect_all(bgr, dictionary, subpix=True):
        if k in want:
            by_id.setdefault(k, []).append(q)
    missing = [k for k in want if k not in by_id]
    duplicated = [k for k in want if len(by_id.get(k, ())) > 1]
    quads = {k: v[0] for k, v in by_id.items() if len(v) == 1}
    small = [k for k in want if k in quads and shortest_side_px(quads[k]) < min_side_px]
    points = None if (missing or duplicated or small) else np.array([square_center(quads[k]) for k in want])
    return BoardDetection(points, quads, missing, duplicated, small)


@dataclass
class MarkerCheck:
    id: int
    base_m: np.ndarray
    expected_px: np.ndarray            # 잰 좌표를 투영한 화면 점
    seen_px: np.ndarray | None         # 찾은 중심 (None = 안 보임)
    pixel_px: float | None             # 이미지 오차
    desk_mm: float | None              # 책상면 3D 오차
    ok: bool | None                    # None = 안 보임


@dataclass
class VerifyResult:
    state: str
    checks: list[MarkerCheck]
    visible: int
    max_pixel_px: float | None
    max_desk_mm: float | None
    reason: str = ""

    @property
    def verified(self) -> bool:
        return self.state == "verified"

    def summary(self) -> str:
        if self.state == "no_config":
            return "기준 표지 설정 없음 (config/markers.yaml)"
        if self.state == "not_visible":
            return f"기준 표지가 {self.visible}개만 보임 (필요 {self.reason})"
        mp = "-" if self.max_pixel_px is None else f"{self.max_pixel_px:.1f}"
        md = "-" if self.max_desk_mm is None else f"{self.max_desk_mm:.1f}"
        head = "정합 확인됨" if self.verified else "정합 어긋남"
        return f"{head}: 표지 {self.visible}개, 최대 {mp} px / {md} mm"

    def to_dict(self) -> dict:
        def f(x):
            return None if x is None else [round(float(v), 4) for v in np.ravel(x)] if np.ndim(x) else round(float(x), 4)
        return {"state": self.state, "visible": self.visible, "max_pixel_px": f(self.max_pixel_px),
                "max_desk_mm": f(self.max_desk_mm), "reason": self.reason,
                "markers": [{"id": c.id, "base_m": f(c.base_m), "expected_px": f(c.expected_px),
                             "seen_px": f(c.seen_px), "pixel_px": f(c.pixel_px), "desk_mm": f(c.desk_mm), "ok": c.ok}
                            for c in self.checks]}


def _plane_at(conv: Cam2Base, z_m: float) -> Plane:
    """base z = z_m 인 수평면을 카메라 좌표 평면으로 (n·p + d = 0, desk_plane과 같은 약속)."""
    pl = conv.desk_plane()
    return Plane(normal=pl.normal, d=pl.d - float(z_m), rms_m=float("nan"), inlier_ratio=float("nan"), n_points=0)


def verify(conv: Cam2Base, mset: MarkerSet | None, seen: dict[int, np.ndarray], rgb_size=None,
           rgb_mirrored: bool = False) -> VerifyResult:
    """잰 표지 좌표 ↔ 찾은 표지 중심. seen = detect(...)의 결과 (여러 프레임 평균이어도 된다)."""
    if mset is None or not mset.markers:
        return VerifyResult("no_config", [], 0, None, None, "no markers.yaml")
    checks = []
    for mid, base in sorted(mset.markers.items()):
        exp = np.asarray(conv.base_to_pixel(base, rgb_mirrored=rgb_mirrored, rgb_size=rgb_size), float).reshape(2)
        uv = seen.get(mid)
        if uv is None:
            checks.append(MarkerCheck(mid, base, exp, None, None, None, None))
            continue
        uv = np.asarray(uv, float).reshape(2)
        pix = float(np.linalg.norm(uv - exp)) if np.all(np.isfinite(exp)) else float("inf")
        pts, _, _ = conv.pixel_to_desk(uv[0], uv[1], rgb_mirrored=rgb_mirrored, rgb_size=rgb_size,
                                       plane=_plane_at(conv, base[2]))
        p = np.asarray(pts, float).reshape(-1, 3)[0]
        desk = float(np.linalg.norm(p - base) * 1000) if np.all(np.isfinite(p)) else float("inf")
        ok = pix <= mset.max_pixel_px and desk <= mset.max_desk_mm
        checks.append(MarkerCheck(mid, base, exp, uv, pix, desk, ok))
    vis = [c for c in checks if c.ok is not None]
    max_pix = max((c.pixel_px for c in vis), default=None)
    max_desk = max((c.desk_mm for c in vis), default=None)
    if any(not c.ok for c in vis):
        bad = [c.id for c in vis if not c.ok]
        return VerifyResult("moved", checks, len(vis), max_pix, max_desk, f"markers {bad} off")
    if len(vis) < mset.min_visible:
        return VerifyResult("not_visible", checks, len(vis), max_pix, max_desk, f"{mset.min_visible}")
    return VerifyResult("verified", checks, len(vis), max_pix, max_desk, "")


def average_detections(frames_seen: list[dict[int, np.ndarray]], min_fraction: float = 0.5) -> dict[int, np.ndarray]:
    """여러 프레임의 detect 결과를 표지별 중앙값으로 (절반 이상의 프레임에서 보인 표지만)."""
    ids: dict[int, list] = {}
    for s in frames_seen:
        for k, v in s.items():
            ids.setdefault(k, []).append(np.asarray(v, float))
    n = max(1, len(frames_seen))
    return {k: np.median(np.array(v), axis=0) for k, v in ids.items() if len(v) >= min_fraction * n}


def make_sheet(mset: MarkerSet, ids=None, dpi: int = 300, page_mm=(210.0, 297.0)):
    """인쇄용 표지 한 장 (흑백 이미지). 표지 한 변 = mset.size_m. 아래에 번호를 적는다. 100 % 크기로 인쇄한다."""
    import cv2
    ids = sorted(mset.markers) if ids is None else list(ids)
    px_per_mm = dpi / 25.4
    W, H = int(page_mm[0] * px_per_mm), int(page_mm[1] * px_per_mm)
    side = int(round(mset.size_m * 1000 * px_per_mm))
    margin = int(10 * px_per_mm)          # 흰 여백 (검출에 필요) + 번호 글씨 자리
    cell = side + 2 * margin
    page = np.full((H, W), 255, np.uint8)
    cols = max(1, W // cell)
    d = _dictionary(mset.dictionary)
    for n, mid in enumerate(ids):
        r, c = divmod(n, cols)
        x, y = c * cell + margin, r * cell + margin
        if y + side + margin > H:
            raise ValueError("too many markers for one page: use a smaller size_m or fewer ids")
        page[y:y + side, x:x + side] = cv2.aruco.generateImageMarker(d, int(mid), side)
        cv2.putText(page, f"id {mid}  {mset.size_m * 1000:.0f} mm", (x, y + side + int(8 * px_per_mm)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2, cv2.LINE_AA)
    return page
