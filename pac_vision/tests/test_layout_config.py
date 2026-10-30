"""저장소의 config/a4_layout.yaml(두 장)과 a4_layout_single.yaml(한 장)이 올바른 형식인지 (숫자 값이 아니라 성질만).

기준물 배치는 현장에서 바꿀 수 있으므로, 좌표 값 대신 캘리브레이션이 기대하는 성질을 검사한다.
"""
import numpy as np
import pytest
import yaml

import synth

LAYOUTS = ("a4_layout.yaml", "a4_layout_single.yaml", "a4_layout_aruco.yaml")
_current = [LAYOUTS[0]]


@pytest.fixture(autouse=True, params=LAYOUTS)
def _which(request):
    _current[0] = request.param


def layout():
    return yaml.safe_load((synth.CONFIG_DIR / _current[0]).read_text(encoding="utf-8"))


def corners():
    """꼭짓점을 종이 테두리 순서로 (cycle이 있으면 그 순서: 클릭 순서가 좌상·우상·좌하·우하인 배치)."""
    lay = layout()
    c = np.array(lay["corners"], dtype=float)
    return c[[int(i) - 1 for i in lay.get("cycle", range(1, len(c) + 1))]]


def test_single_sheet_is_one_a4_with_the_base_off_the_paper():
    lay = yaml.safe_load((synth.CONFIG_DIR / "a4_layout_single.yaml").read_text(encoding="utf-8"))
    c = np.array(lay["corners"], dtype=float)
    assert lay["base_clicked"] is False and len(c) == 4
    assert np.isclose(np.linalg.norm(c[0] - c[1]), 0.297) and np.isclose(np.linalg.norm(c[2] - c[3]), 0.297)   # 1-2, 3-4
    assert np.isclose(np.linalg.norm(c[0] - c[2]), 0.21) and np.isclose(np.linalg.norm(c[1] - c[3]), 0.21)     # 1-3, 2-4
    mid12 = (c[0] + c[1]) / 2
    assert np.isclose(np.linalg.norm(mid12[:2]), 0.09)                    # base 9 cm from the middle of edge 1-2
    centre = c.mean(axis=0)
    assert np.isclose(centre[0], 0.0) and centre[1] > 0                   # +Y = base -> paper centre


def signed_area_xy(c):
    """위(+Z)에서 본 다각형의 부호 있는 넓이 (신발끈 공식). 반시계면 양수."""
    x, y = c[:, 0], c[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def test_at_least_three_corners_not_collinear():
    c = corners()
    assert c.ndim == 2 and c.shape[1] == 3
    assert len(c) >= 3
    assert np.linalg.matrix_rank(c[1:] - c[0], tol=1e-6) == 2      # 한 직선 위에 있지 않음


def test_counter_clockwise_from_above():
    assert signed_area_xy(corners()) > 0


def test_recheck_corner_is_existing_corner_number():
    rc = layout()["recheck_corner"]
    assert isinstance(rc, int) and not isinstance(rc, bool)
    assert 1 <= rc <= len(corners())


def test_all_points_on_desk_plane():
    assert np.all(corners()[:, 2] == 0.0)
    assert float(np.asarray(layout()["base_point"], dtype=float)[2]) == 0.0
