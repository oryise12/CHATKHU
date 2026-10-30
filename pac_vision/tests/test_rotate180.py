"""카메라를 뒤집어 단 설정(rotate_180): 돌린 좌표의 정합 계수가 돌리기 전 계수와 같은 점을 가리키는지 (카메라 없이)."""
import copy

import numpy as np

from pac_vision.cam2base import CameraModel, rotate_rgb_to_depth_180
from pac_vision.session import calibration_mismatches
import synth
from test_identity import _conv

W, H = 640, 480
PRESET = {"sx": 1.0, "sy": 1.0, "dx": {"a": 8.580472, "b": -0.012502, "c": 0.01603, "d": 14.56444},
          "dy": {"a": 8.475, "b": 0.0, "c": 0.0, "d": 0.0}}


def _cam(m):
    return CameraModel.from_config(dict(synth.CAMERA_CFG, rgb_to_depth=m))


def test_rotated_coefficients_map_the_same_points():
    rng = np.random.default_rng(0)
    for m in (PRESET, synth.DIST_RGB_TO_DEPTH, {"sx": 1.0, "sy": 1.0, "dx": 23.0, "dy": 8.0}):
        native, rot = _cam(m), _cam(rotate_rgb_to_depth_180(m, (W, H), (W, H)))
        u, v, z = rng.uniform(0, W - 1, 50), rng.uniform(0, H - 1, 50), rng.uniform(0.6, 2.0, 50)
        ud, vd = native.rgb_to_depth(u, v, z_m=z)
        ud2, vd2 = rot.rgb_to_depth(W - 1 - u, H - 1 - v, z_m=z)        # the same point seen in the rotated images
        assert np.allclose(ud2, W - 1 - ud, atol=1e-6) and np.allclose(vd2, H - 1 - vd, atol=1e-6)
        back = rotate_rgb_to_depth_180(rotate_rgb_to_depth_180(m, (W, H), (W, H)), (W, H), (W, H))
        n2 = _cam(back)
        assert np.allclose(n2.rgb_to_depth(u, v, z_m=z), native.rgb_to_depth(u, v, z_m=z), atol=1e-5)   # twice = back


def test_preset_values_after_rotation():
    r = rotate_rgb_to_depth_180(PRESET)
    assert abs(r["dx"]["a"] - (-8.580472 + 639 * 0.012502 - 479 * 0.01603)) < 1e-6 and r["dx"]["d"] == -14.56444
    assert r["dy"]["a"] == -8.475 and r["dx"]["b"] == PRESET["dx"]["b"]


def test_a_calibration_made_without_rotation_is_not_used_with_it():
    cfg = copy.deepcopy(synth.CAMERA_CFG)
    conv = _conv(cfg)
    assert calibration_mismatches(conv, cfg) == []
    assert "rotate_180" in calibration_mismatches(conv, dict(cfg, rotate_180=True))
