"""저장소의 config/a4_layout.yaml이 올바른 형식인지 (숫자 값이 아니라 성질만) 확인한다.

기준물 배치는 현장에서 바꿀 수 있으므로, 좌표 값 대신 캘리브레이션이 기대하는 성질을 검사한다.
"""
import numpy as np

import synth


def layout():
    return synth.repo_config()["a4_layout"]


def corners():
    return np.array(layout()["corners"], dtype=float)


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
