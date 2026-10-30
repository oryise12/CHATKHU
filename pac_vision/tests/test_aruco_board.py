"""ArUco 보정판(config/a4_layout_aruco.yaml): 인쇄물 배치, 마커 중심 찾기, 찾은 점으로 캘리브레이션.

합성 장면: 카메라가 종이 너머(사용자 쪽)에서 로봇 쪽을 비스듬히 내려다본다. 보정판 그림(tools/make_aruco_board.render)을
책상면 → 화면 사영으로 붙인다. 고정 오프셋 정합이라 그 사영이 정확하다 (test_markers.py와 같은 이유).
이 테스트는 계산이 맞는지 보는 것이지, 실제 Astra에서의 오차가 아니다.
"""
import copy
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from pac_vision.cam2base import solve_calibration
from pac_vision.markers import find_board
import synth

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "scripts"))
import calibrate  # noqa: E402
import make_aruco_board as mab  # noqa: E402

LAYOUT = yaml.safe_load((synth.CONFIG_DIR / "a4_layout_aruco.yaml").read_text(encoding="utf-8"))
SINGLE = yaml.safe_load((synth.CONFIG_DIR / "a4_layout_single.yaml").read_text(encoding="utf-8"))
RGB_SIZE = (640, 480)
PX_PER_MM = 4.0
BACKGROUND = 150                       # 책상 (종이보다 어둡게)


def page_to_base(layout, uv_mm):
    sh = layout["sheet"]
    c1 = np.asarray(sh["corner1_m"], float)
    ud, vd = np.r_[sh["u_dir"], 0.0], np.r_[sh["v_dir"], 0.0]
    uv = np.asarray(uv_mm, float) / 1000
    return c1 + uv[:, :1] * ud + uv[:, 1:] * vd


def camera_view(cam, R, t, page, layout=LAYOUT):
    """보정판 그림을 책상 위 자리에 놓고 찍은 화면 (BGR)."""
    W, H = (float(x) * 1000 for x in layout["sheet"]["size_m"])
    uu, vv = np.meshgrid(np.linspace(0, W, 7), np.linspace(0, H, 5))
    uv = np.stack([uu.ravel(), vv.ravel()], axis=1)
    px = synth.clicks_for(cam, R, t, page_to_base(layout, uv), rgb_size=RGB_SIZE)
    hom, _ = cv2.findHomography(uv * PX_PER_MM, px)
    img = cv2.warpPerspective(page, hom, RGB_SIZE, flags=cv2.INTER_AREA, borderValue=BACKGROUND)
    inside = cv2.warpPerspective(np.full_like(page, 255), hom, RGB_SIZE) > 0
    img = cv2.GaussianBlur(np.where(inside, img, BACKGROUND).astype(np.uint8), (0, 0), 0.7)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


@pytest.fixture(scope="module")
def scene():
    cam = synth.camera()
    R, t = synth.look_at((0.0, 0.85, 0.70), (0.0, 0.19, 0.0))
    page = mab.render(LAYOUT, PX_PER_MM)
    return {"cam": cam, "R": R, "t": t, "page": page, "rgb": camera_view(cam, R, t, page),
            "depth": synth.render_depth(cam, R, t, noise_mm=0.5, outlier_frac=0.01),
            "truth": synth.clicks_for(cam, R, t, np.array(LAYOUT["corners"], float), rgb_size=RGB_SIZE),
            "thr": synth.repo_config()["thresholds"]}


@pytest.fixture
def calibrate_globals():
    """calibrate.use_layout은 모듈 전역을 바꾼다: 다른 테스트에 새지 않게 되돌린다."""
    names = ("CLICK_GUIDE", "POINT_LABELS", "POINT_NAMES", "SIDE_NAMES", "N_POINTS", "KEY0", "SHAPE", "KIND", "ARUCO")
    saved = {n: copy.deepcopy(getattr(calibrate, n)) for n in names}
    yield
    for n, v in saved.items():
        setattr(calibrate, n, v)


def find(img, **kw):
    a = LAYOUT["aruco"]
    return find_board(img, a["ids"], a["dictionary"], kw.pop("min_side_px", a["min_side_px"]))


def test_sheet_sits_on_the_single_a4_place_with_white_gaps():
    g = mab.board_geometry(LAYOUT)
    W, H = g["page_mm"]
    assert (W, H) == (297.0, 210.0) and g["side_mm"] == 95.0
    # 종이 네 꼭짓점 = 한 장 배치(a4_layout_single.yaml)의 꼭짓점: 같은 자리에 놓는다
    paper = page_to_base(LAYOUT, [[0, 0], [W, 0], [0, H], [W, H]])
    assert np.allclose(paper, np.array(SINGLE["corners"], float))
    # 인쇄물의 마커 중심 = 배치의 기준점 (ID 순서)
    centres = page_to_base(LAYOUT, [[u + 47.5, v + 47.5] for u, v in (g["markers"][k] for k in LAYOUT["aruco"]["ids"])])
    assert np.allclose(centres, np.array(LAYOUT["corners"], float))
    # 짧은 변 방향: 여백 5 + 95 + 사이 10 + 95 + 여백 5 / 긴 변 방향: 사이 90
    (u1, v1), (u2, _), (_, v3) = g["markers"][1], g["markers"][2], g["markers"][3]
    assert v1 == pytest.approx(5.0) and v3 - (v1 + 95) == pytest.approx(10.0) and u2 - (u1 + 95) == pytest.approx(90.0)


def test_touching_markers_are_refused():
    lay = copy.deepcopy(LAYOUT)
    for c in lay["corners"][2:]:
        c[1] -= 0.010                                          # 마커 3·4를 1·2 쪽으로 10 mm: 사이 10 mm -> 0 mm
    with pytest.raises(ValueError, match="흰 여백"):
        mab.board_geometry(lay)


def test_touching_markers_are_not_detected():
    """요청 그대로 붙여 인쇄했다면: 검은 테두리가 합쳐져 하나도 못 찾는다 (여백 검사의 근거)."""
    page = np.full((210 * 4, 297 * 4), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    s = 100 * 4
    for mid, (x, y) in {1: (14, 20), 2: (774, 20), 3: (14, 420), 4: (774, 420)}.items():
        page[y:y + s, x:x + s] = cv2.aruco.generateImageMarker(d, mid, s)
    assert find(page).missing == [1, 2, 3, 4]


def test_marker_centres_are_found_in_id_order(scene):
    det = find(scene["rgb"])
    assert det.points is not None, (det.missing, det.duplicated, det.small)
    assert np.max(np.linalg.norm(det.points - scene["truth"], axis=1)) < 0.7      # px
    assert list(det.quads) and sorted(det.quads) == [1, 2, 3, 4]


def test_calibration_from_the_found_centres(scene):
    det = find(scene["rgb"])
    res = solve_calibration(scene["depth"], det.points, scene["cam"], LAYOUT, scene["thr"], rgb_size=RGB_SIZE)
    assert res.passed, [(c.key, c.value) for c in res.checks if not c.ok]
    assert np.allclose(res.R, scene["R"], atol=3e-3) and np.allclose(res.t, scene["t"], atol=3e-3)
    assert res.holdout_m < 0.003
    assert np.allclose(res.sides_expected_m, [0.105, 0.185, 0.105, 0.185])        # cycle 1-3-4-2
    assert res.check("a4_size").limit == LAYOUT["short_side_min_px"]


def test_missing_and_duplicated_markers_are_named(scene):
    img = scene["rgb"].copy()
    q = find(img).quads[2]
    cv2.fillConvexPoly(img, np.round(q).astype(np.int32), (255, 255, 255))         # 마커 2를 가린다
    det = find(img)
    assert det.points is None and det.missing == [2]

    img = scene["rgb"].copy()                                                     # 보정판 밖에 같은 번호 1이 또 하나
    tile = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 1, 60)
    img[390:470, 10:90] = 255                                                     # 흰 여백
    img[400:460, 20:80] = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
    det = find(img)
    assert det.points is None and det.duplicated == [1]


def test_too_small_markers_are_named(scene):
    det = find(scene["rgb"], min_side_px=200)
    assert det.points is None and det.small == [1, 2, 3, 4]


def test_calibrate_finder_returns_points_like_clicks(scene, calibrate_globals):
    calibrate.use_layout(LAYOUT)
    assert calibrate.default_points(LAYOUT) == "aruco" and calibrate.default_points(SINGLE) == "click"
    assert calibrate.N_POINTS == 4 and calibrate.SIDE_NAMES == ["1-3", "3-4", "4-2", "2-1"]
    pts = calibrate.POINT_FINDERS["aruco"](None, scene["rgb"], scene["depth"], scene["cam"], scene["thr"], RGB_SIZE,
                                           "plane")
    assert len(pts) == 4 and np.allclose(pts, find(scene["rgb"]).points)
    # 결과 화면 문구가 보정판 기준 (A4가 아니라 마커 중심 사각형, r = 새 정지 화면)
    res = solve_calibration(scene["depth"], pts, scene["cam"], LAYOUT, scene["thr"], rgb_size=RGB_SIZE)
    text = "\n".join(ko for ko, _en, _c in calibrate.result_lines(res))
    assert "마커 중심 사각형 네 변" in text and "r: 새 정지 화면에서 다시 찾기" in text
    assert "A4" not in text and "꼭짓점" not in text


def test_unsolvable_board_points_go_back_to_a_new_frame(scene, calibrate_globals, monkeypatch):
    """찾은 점으로 계산이 여러 점에서 실패하면 같은 화면을 또 찾지 않고(같은 결과의 무한 반복) 새 정지 화면으로 간다."""
    calibrate.use_layout(LAYOUT)
    calls = []
    monkeypatch.setitem(calibrate.POINT_FINDERS, "aruco", lambda *a: calls.append(1) or [(5000.0, 5000.0)] * 4)
    cfg = dict(synth.repo_config(), a4_layout=LAYOUT)
    act = calibrate.calibrate_frame(None, scene["rgb"], scene["depth"], scene["cam"], cfg, RGB_SIZE, "plane", True,
                                    0.1, "aruco")
    assert act == ("back", None) and len(calls) == 1
