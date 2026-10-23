"""RGB→depth 거리 식 (dx = a + b·u + c·v + d/Z): 옛 형식 호환, 왕복, 책상 평면 고정점 반복, 캘리브레이션 전체 흐름."""
import copy
import warnings

import numpy as np
import pytest

from pac_vision.cam2base import Cam2Base, Plane, layout_points, rgb_depth_coef, solve_calibration
import synth


def test_old_scalar_config_equals_coefficients_with_zero_terms():
    """옛 형식 dx: 25 와 {a: 25, b: 0, c: 0, d: 0} 은 비트 단위로 같은 변환."""
    old = synth.camera()
    new = synth.camera({"sx": 1.0, "sy": 1.0, "dx": {"a": 25.0}, "dy": {"a": 0.0, "b": 0, "c": 0, "d": 0}})
    assert rgb_depth_coef(25) == (25.0, 0.0, 0.0, 0.0)
    u, v = np.array([0.0, 123.4, 639.0]), np.array([0.0, 240.5, 479.0])
    for size, mir in ((None, False), ((1280, 720), True)):
        assert np.array_equal(np.stack(old.rgb_to_depth(u, v, size, mir)), np.stack(new.rgb_to_depth(u, v, size, mir)))
        assert np.array_equal(np.stack(old.depth_to_rgb(u, v, size, mir)), np.stack(new.depth_to_rgb(u, v, size, mir)))
    assert not new.has_z_term and new.is_fixed_offset


def test_distance_model_round_trip_with_z():
    cam = synth.camera(synth.DIST_RGB_TO_DEPTH)
    rng = np.random.default_rng(1)
    u, v = rng.uniform(0, 639, 50), rng.uniform(0, 479, 50)
    z = rng.uniform(0.6, 2.4, 50)
    for size, mir in ((None, False), ((1280, 720), False), ((640, 480), True)):
        su, sv = cam.ref_to_screen(u, v, size, mir)
        ud, vd = cam.rgb_to_depth(su, sv, size, mir, z_m=z)
        bu, bv = cam.depth_to_rgb(ud, vd, size, mir, z_m=z)
        assert np.max(np.abs(bu - su)) < 1e-9 and np.max(np.abs(bv - sv)) < 1e-9
    # 가까울수록 dx가 크다 (d > 0)
    near = cam.rgb_to_depth(320.0, 240.0, z_m=0.7)[0]
    far = cam.rgb_to_depth(320.0, 240.0, z_m=1.4)[0]
    assert near - far == pytest.approx(20.5 * (1 / 0.7 - 1 / 1.4))


def test_missing_z_warns_only_for_distance_model():
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        synth.camera().rgb_to_depth(100.0, 100.0)          # 고정값: Z 불필요, 경고 없음
    with pytest.warns(RuntimeWarning, match="z_ref_m"):
        synth.camera(synth.DIST_RGB_TO_DEPTH).rgb_to_depth(100.0, 100.0)


def test_plane_fixed_point_matches_dense_search():
    """고정점 반복 결과 = Z를 촘촘히 바꿔 가며 '그 Z로 옮긴 광선의 평면 교점 Z = 그 Z'를 찾은 결과 (0.01 px 이내)."""
    cam = synth.camera(synth.DIST_RGB_TO_DEPTH)
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    plane = Cam2Base(R, t, cam).desk_plane()
    u, v = np.array([100.0, 320.0, 520.0]), np.array([300.0, 400.0, 260.0])
    ud, vd, pts = cam.rgb_to_depth_on_plane(u, v, plane)
    zs = np.linspace(0.5, 3.0, 250001)
    for i in range(len(u)):
        uu, vv = cam.rgb_to_depth(np.full_like(zs, u[i]), np.full_like(zs, v[i]), z_m=zs)
        zp = (-plane.d / (cam.ray(uu, vv) @ plane.normal))
        k = int(np.argmin(np.abs(zp - zs)))
        assert abs(uu[k] - ud[i]) < 0.01 and abs(vv[k] - vd[i]) < 0.01
        assert pts[i, 2] == pytest.approx(zs[k], abs=1e-4)


def test_pixel_to_desk_and_base_to_pixel_round_trip():
    cam = synth.camera(synth.DIST_RGB_TO_DEPTH)
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    conv = Cam2Base(R, t, cam)
    xyz = np.array([[0.0, 0.0, 0.0], [0.2, -0.1, 0.0], [-0.1, 0.3, 0.0]])
    for size, mir in ((None, False), ((1280, 720), True)):
        px = conv.base_to_pixel(xyz, rgb_mirrored=mir, rgb_size=size)
        back, _, _ = conv.pixel_to_desk(px[:, 0], px[:, 1], rgb_mirrored=mir, rgb_size=size)
        assert np.max(np.abs(back - xyz)) < 1e-6


def _scene(rgb_to_depth):
    cfg = synth.scene_config()
    true_cam = synth.camera(rgb_to_depth)
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    depth = synth.render_depth(true_cam, R, t, wall_rows=40, noise_mm=0.5, outlier_frac=0.01)
    clicks = synth.clicks_for(true_cam, R, t, layout_points(cfg["a4_layout"]))
    return cfg, R, t, depth, clicks


def test_calibration_with_distance_model_recovers_pose_and_fixed_offset_does_not():
    """실제 정합에 d/Z 항이 있는 장면: 거리 식 설정이면 R·t 복원, 같은 장면을 고정값(평균 근사)으로 풀면 변 길이 오차가 커진다."""
    cfg, R, t, depth, clicks = _scene(synth.DIST_RGB_TO_DEPTH)
    good = solve_calibration(depth, clicks, synth.camera(synth.DIST_RGB_TO_DEPTH), cfg["a4_layout"], cfg["thresholds"])
    assert synth.rotation_angle_deg(good.R, R) < 0.1
    assert np.linalg.norm(good.t - t) < 0.002
    assert np.max(np.abs(good.sides_m - good.sides_expected_m)) < 0.002
    assert good.check("a4_sides").ok

    # 같은 클릭의 depth 픽셀 위치에서 평균 dx·dy를 고정값으로 쓴 근사
    off = good.clicks_depth - good.clicks_rgb
    fixed = {"sx": 1.0, "sy": 1.0, "dx": float(off[:, 0].mean()), "dy": float(off[:, 1].mean())}
    approx = solve_calibration(depth, clicks, synth.camera(fixed), cfg["a4_layout"], cfg["thresholds"])
    err_good = np.max(np.abs(good.sides_m - good.sides_expected_m))
    err_fixed = np.max(np.abs(approx.sides_m - approx.sides_expected_m))
    assert err_fixed > 3 * err_good and err_fixed > 0.003


def test_point_mode_with_distance_model():
    cfg, R, t, depth, clicks = _scene(synth.DIST_RGB_TO_DEPTH)
    res = solve_calibration(depth, clicks, synth.camera(synth.DIST_RGB_TO_DEPTH), cfg["a4_layout"], cfg["thresholds"],
                            mode="point")
    assert synth.rotation_angle_deg(res.R, R) < 0.5
    assert np.linalg.norm(res.t - t) < 0.01


def test_fixed_offset_scene_unchanged_by_new_code_path():
    """고정값 설정: 예전 테스트와 같은 장면에서 결과가 그대로 (반복 없이 한 번에 계산)."""
    cfg, R, t, depth, clicks = _scene(copy.deepcopy(synth.CAMERA_CFG["rgb_to_depth"]))
    cam_old = synth.camera()
    cam_dict = synth.camera({"sx": 1.0, "sy": 1.0, "dx": {"a": 25.0}, "dy": {"a": 0.0}})
    a = solve_calibration(depth, clicks, cam_old, cfg["a4_layout"], cfg["thresholds"])
    b = solve_calibration(depth, clicks, cam_dict, cfg["a4_layout"], cfg["thresholds"])
    assert np.array_equal(a.R, b.R) and np.array_equal(a.t, b.t)
    assert a.roi == b.roi


def test_desk_plane_is_base_z0():
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    conv = Cam2Base(R, t, synth.camera())
    pl = conv.desk_plane()
    assert isinstance(pl, Plane) and pl.d > 0
    p_cam = conv.base_to_camera(np.array([[0.3, -0.2, 0.0], [0.0, 0.4, 0.0]]))
    assert np.max(np.abs(pl.distance(p_cam))) < 1e-12
