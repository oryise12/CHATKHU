"""화면 공용 _ui.base_axes: base 원점에서 X(빨강)·Y(초록)·Z(파랑, 위) 화살표."""
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import _ui  # noqa: E402
from pac_vision.cam2base import Cam2Base  # noqa: E402
import synth  # noqa: E402

RGB_SIZE = (640, 480)
L = 0.10                 # config 값과 같은 길이 (테스트는 값만 쓴다)
MASK_MARGIN = 3          # Z 화살표 둘레로 비교에서 빼는 폭 (px)


def conv_for(position, target):
    cam = synth.camera(synth.DIST_RGB_TO_DEPTH)
    return Cam2Base(*synth.look_at(position, target), cam)


def blank():
    return np.zeros((RGB_SIZE[1], RGB_SIZE[0], 3), np.uint8)


def old_xy(img, conv):
    """변경 전 호출부 3줄 그대로 (X·Y만)."""
    o, x, y = conv.base_to_pixel(np.array([[0, 0, 0], [L, 0, 0], [0, L, 0]], float), rgb_size=RGB_SIZE)
    _ui.arrow(img, o, x, _ui.RED, 2)
    _ui.arrow(img, o, y, _ui.GREEN, 2)


def is_color(img, color):
    return np.all(img == np.array(color, np.uint8), axis=-1)


def test_z_tip_at_projected_point_and_blue():
    conv = conv_for(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    o, z = conv.base_to_pixel(np.array([[0, 0, 0], [0, 0, L]], float), rgb_size=RGB_SIZE)
    assert _ui.finite(z) and 0 <= z[0] < RGB_SIZE[0] and 0 <= z[1] < RGB_SIZE[1]
    assert z[1] < o[1]                       # 비스듬히 내려다보는 카메라에서 +Z는 화면 위쪽
    img = blank()
    _ui.base_axes(img, conv, L, RGB_SIZE)
    u, v = _ui.ipt(z)
    patch = img[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3]
    assert is_color(patch, _ui.BLUE).any()


def test_xy_pixels_unchanged_outside_z_mask():
    conv = conv_for(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    # 끝점 계산: 점 4개로 한 번에 바꿔도 X·Y 픽셀 값은 예전(점 3개)과 같다
    p3 = conv.base_to_pixel(np.array([[0, 0, 0], [L, 0, 0], [0, L, 0]], float), rgb_size=RGB_SIZE)
    p4 = conv.base_to_pixel(np.array([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]], float), rgb_size=RGB_SIZE)
    np.testing.assert_array_equal(p3, p4[:3])

    old, new = blank(), blank()
    old_xy(old, conv)
    _ui.base_axes(new, conv, L, RGB_SIZE)
    # Z 화살표(두께 2)를 둘레 MASK_MARGIN px만큼 굵게 그린 영역은 비교에서 뺀다 (LINE_AA 섞임)
    mask = np.zeros(RGB_SIZE[::-1], np.uint8)
    cv2.arrowedLine(mask, _ui.ipt(p4[0]), _ui.ipt(p4[3]), 255, 2 + 2 * MASK_MARGIN, cv2.LINE_8, tipLength=0.15)
    out = mask == 0
    np.testing.assert_array_equal(old[out], new[out])
    diff_in_mask = int(np.any(old != new, axis=-1)[~out].sum())
    xy_in_mask = int(np.any(old != 0, axis=-1)[~out].sum())
    print(f"mask {int((~out).sum())} px (margin {MASK_MARGIN} px), X·Y px in mask {xy_in_mask}, "
          f"changed in mask {diff_in_mask}")
    assert is_color(new, _ui.RED).any() and is_color(new, _ui.GREEN).any()


def test_z_behind_camera_skipped_xy_drawn():
    # 원점 근처 6 cm 높이에서 가파르게 내려다봄 → (0, 0, L=0.10)은 카메라 뒤, 원점은 화면 안
    conv = conv_for((-0.03, 0.0, 0.06), (0.0, 0.03, 0.0))
    o, x, y, z = conv.base_to_pixel(np.array([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]], float), rgb_size=RGB_SIZE)
    assert conv.base_to_camera(np.array([0, 0, L], float))[2] <= 0
    assert not _ui.finite(z) and _ui.finite(o) and _ui.finite(x) and _ui.finite(y)
    assert 0 <= o[0] < RGB_SIZE[0] and 0 <= o[1] < RGB_SIZE[1]
    img = blank()
    _ui.base_axes(img, conv, L, RGB_SIZE)
    assert not is_color(img, _ui.BLUE).any()
    assert is_color(img, _ui.RED).any() and is_color(img, _ui.GREEN).any()
    old = blank()
    old_xy(old, conv)
    np.testing.assert_array_equal(old, img)
