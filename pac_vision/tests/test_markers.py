"""고정 표지(ArUco) 검증의 판정 규칙: 합성 영상(카메라 없음)으로만 확인한다.

이 테스트는 계산과 판정이 맞는지 보는 것이지, 실제 Astra에서의 오차가 아니다. 현장 오차는 scripts/check_markers.py로 잰다.
"""
import cv2
import numpy as np
import pytest

from pac_vision.cam2base import Cam2Base
from pac_vision.markers import (MarkerSet, average_detections, detect, load_markers, make_sheet, square_center,
                                verify)
import synth

RGB_SIZE = (640, 480)
MARKERS = {0: np.array([0.30, 0.20, 0.0]), 1: np.array([0.30, -0.20, 0.0]), 2: np.array([0.12, 0.26, 0.0])}


def scene():
    # 고정 오프셋 정합: 책상면 -> 화면이 정확히 사영 변환이라 아래 render(호모그래피)가 정확하다.
    # (거리 항 d/Z가 있으면 10 cm 표지 안에서도 사영이 아니게 되어, 합성 그림 자체가 몇 mm 어긋난다)
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    return Cam2Base(R, t, cam)


def marker_set(**kw):
    # 이 합성 카메라(약 1.3 m, 비스듬히)에서 6 cm 표지는 13 x 24 px로 보여 검출이 흔들렸다 -> 12 cm
    return MarkerSet(size_m=0.12, markers=dict(MARKERS), **kw)


def render(conv, mset, ids=None, shift=(0.0, 0.0)):
    """책상 위 표지를 보정 그대로 화면에 그린 합성 영상 (shift: 카메라가 밀린 것처럼 영상 전체를 옮긴다)."""
    img = np.full((RGB_SIZE[1], RGB_SIZE[0], 3), 180, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side, pad = 120, 30
    for mid, base in mset.markers.items():
        if ids is not None and mid not in ids:
            continue
        tile = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d, mid, side), pad, pad, pad, pad,
                                  cv2.BORDER_CONSTANT, value=255)
        s, q = mset.size_m / 2, mset.size_m / 2 * (side + 2 * pad) / side
        # 표지 위쪽 = +X (카메라에서 먼 쪽), 왼쪽 = +Y
        quad = base + np.array([[q, q, 0], [q, -q, 0], [-q, -q, 0], [-q, q, 0]])
        dst = (np.asarray(conv.base_to_pixel(quad, rgb_size=RGB_SIZE), float).reshape(4, 2) + shift).astype(np.float32)
        n = side + 2 * pad
        H = cv2.getPerspectiveTransform(np.float32([[0, 0], [n, 0], [n, n], [0, n]]), dst)
        warped = cv2.warpPerspective(cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR), H, RGB_SIZE)
        mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), H, RGB_SIZE) > 0
        img[mask] = warped[mask]
        assert s < q
    return img


def test_untouched_camera_is_verified_with_small_errors():
    conv, ms = scene(), marker_set()
    res = verify(conv, ms, detect(render(conv, ms)), rgb_size=RGB_SIZE)
    assert res.state == "verified", res.summary()
    assert res.visible == 3 and res.max_pixel_px < 1.5 and res.max_desk_mm < 3.0


def test_shifted_camera_is_moved():
    conv, ms = scene(), marker_set()
    res = verify(conv, ms, detect(render(conv, ms, shift=(15, 0))), rgb_size=RGB_SIZE)
    assert res.state == "moved"
    assert res.max_pixel_px > 12 and res.max_desk_mm > ms.max_desk_mm


def test_rotated_camera_is_moved():
    """카메라가 제자리에서 2도 돌아도 (책상 높이 검사로는 안 보일 수 있는 이동) 이미지 오차로 잡힌다."""
    conv, ms = scene(), marker_set()
    cam = synth.camera(synth.DIST_RGB_TO_DEPTH)
    a = np.radians(2.0)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    moved = Cam2Base(Rz @ conv.R, conv.t, cam)          # 실제 카메라는 돌았는데 보정은 그대로
    res = verify(conv, ms, detect(render(moved, ms)), rgb_size=RGB_SIZE)
    assert res.state == "moved", res.summary()


def test_too_few_visible_markers_cannot_verify():
    conv, ms = scene(), marker_set()
    res = verify(conv, ms, detect(render(conv, ms, ids={0})), rgb_size=RGB_SIZE)
    assert res.state == "not_visible" and res.visible == 1
    assert not res.verified


def test_no_marker_config_cannot_verify():
    res = verify(scene(), None, {})
    assert res.state == "no_config" and not res.verified


def test_one_bad_marker_is_enough_to_fail():
    """두 표지가 맞아도 한 표지가 어긋나면 moved (표지가 밀렸거나 보정이 틀림 - 사람이 확인해야 한다)."""
    conv, ms = scene(), marker_set()
    seen = detect(render(conv, ms))
    seen[2] = seen[2] + [20.0, 0.0]
    assert verify(conv, ms, seen, rgb_size=RGB_SIZE).state == "moved"


def test_average_and_square_center():
    assert np.allclose(square_center([[0, 0], [2, 0], [2, 2], [0, 2]]), [1, 1])
    frames = [{0: np.array([10.0, 10.0])}, {0: np.array([12.0, 10.0]), 1: np.array([5.0, 5.0])},
              {0: np.array([11.0, 10.0])}]
    avg = average_detections(frames)
    assert set(avg) == {0} and np.allclose(avg[0], [11, 10])


def test_load_markers_and_printable_sheet(tmp_path):
    p = tmp_path / "markers.yaml"
    p.write_text("dictionary: DICT_4X4_50\nsize_m: 0.05\nmarkers:\n  3: {base_m: [0.3, 0.2]}\n  7: [0.1, -0.2, 0.0]\n",
                 encoding="utf-8")
    ms = load_markers(p)
    assert set(ms.markers) == {3, 7} and ms.markers[3][2] == 0.0
    sheet = make_sheet(ms, dpi=150)
    assert set(detect(sheet, ms.dictionary)) == {3, 7}
    with pytest.raises(ValueError):
        p.write_text("markers:\n  1: {base_m: [0.3]}\n", encoding="utf-8")
        load_markers(p)
