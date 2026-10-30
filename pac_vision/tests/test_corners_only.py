"""A4 한 장, 꼭짓점 4개만 찍는 배치(a4_layout_single.yaml): 원점 = 로봇 베이스(종이 밖), +Y = 베이스 → 종이 가운데.

합성 장면: 카메라가 종이 너머(사용자 쪽)에서 베이스 쪽을 비스듬히 내려다본다. 화면에서 1 좌상, 2 우상, 3 좌하, 4 우하.
"""
import numpy as np
import pytest
import yaml

from pac_vision.cam2base import solve_calibration
import synth

LAYOUT = yaml.safe_load((synth.CONFIG_DIR / "a4_layout_single.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def scene():
    cfg = synth.repo_config()
    cam = synth.camera()
    R, t = synth.look_at((0.0, 1.05, 0.75), (0.0, 0.18, 0.0))
    corners = np.array(LAYOUT["corners"], float)
    depth = synth.render_depth(cam, R, t, noise_mm=0.5, outlier_frac=0.01)
    clicks = synth.clicks_for(cam, R, t, corners)
    return {"cfg": cfg, "cam": cam, "R": R, "t": t, "depth": depth, "clicks": clicks}


def solve(sc, clicks=None):
    return solve_calibration(sc["depth"], sc["clicks"] if clicks is None else clicks, sc["cam"], LAYOUT,
                             sc["cfg"]["thresholds"], rgb_size=(640, 480))


def test_corners_appear_top_left_top_right_bottom_left_bottom_right(scene):
    c = scene["clicks"]
    assert c[0][0] < c[1][0] and c[2][0] < c[3][0]            # 1 left of 2, 3 left of 4
    assert c[0][1] < c[2][1] and c[1][1] < c[3][1]            # 1, 2 above 3, 4 (the base side is the top)


def test_recovers_the_robot_frame_from_four_clicks(scene):
    res = solve(scene)
    assert res.passed, [(c.key, c.value) for c in res.checks if not c.ok]
    assert np.allclose(res.R, scene["R"], atol=2e-3) and np.allclose(res.t, scene["t"], atol=3e-3)
    assert res.holdout_m < 0.003 and res.holdout_index in range(4)
    assert np.allclose(res.sides_expected_m, [0.21, 0.297, 0.21, 0.297])      # cycle 1-3-4-2
    # +Y from the base toward the paper centre, base 9 cm from the middle of edge 1-2
    centre = np.array(LAYOUT["corners"], float).mean(axis=0)
    assert centre[0] == 0 and centre[1] > 0 and abs(np.mean(np.array(LAYOUT["corners"])[:2, 1]) - 0.09) < 1e-12


def test_a_misplaced_corner_is_named(scene):
    c = np.array(scene["clicks"], float)
    c[3] += (14.0, 0.0)                                         # corner 4 clicked off the paper corner
    res = solve(scene, c)
    assert not res.check("base_error").ok and res.holdout_index == 3


def test_clicking_in_a_wrong_order_fails(scene):
    c = np.array(scene["clicks"], float)[[0, 1, 3, 2]]          # 3 and 4 swapped (polygon order instead)
    res = solve(scene, c)
    assert not res.passed


def test_four_clicks_required(scene):
    from pac_vision.cam2base import CalibrationError
    with pytest.raises(CalibrationError):
        solve(scene, np.vstack([[[320, 240]], scene["clicks"]]))
