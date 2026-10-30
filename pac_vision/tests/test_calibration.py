"""캘리브레이션 전체 흐름 (합성 장면): 복원, 홀드아웃, A4 변 길이, 반사 감지, 픽셀↔base 왕복."""
import numpy as np
import pytest

from pac_vision.cam2base import (Cam2Base, CalibrationError, camera_above_desk, handedness_ok,
                                 layout_points, solve_calibration)
import synth


@pytest.fixture(scope="module")
def scene():
    cfg = synth.scene_config()
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    depth = synth.render_depth(cam, R, t, wall_rows=40, noise_mm=0.5, outlier_frac=0.01)
    base_pts = layout_points(cfg["a4_layout"])
    clicks = synth.clicks_for(cam, R, t, base_pts)
    return dict(cfg=cfg, cam=cam, R=R, t=t, depth=depth, base_pts=base_pts, clicks=clicks)


def solve(scene, clicks=None, depth=None, mode="plane"):
    return solve_calibration(scene["depth"] if depth is None else depth,
                             scene["clicks"] if clicks is None else clicks,
                             scene["cam"], scene["cfg"]["a4_layout"], scene["cfg"]["thresholds"], mode=mode)


def test_recovers_camera_pose(scene):
    res = solve(scene)
    assert synth.rotation_angle_deg(res.R, scene["R"]) < 0.1
    assert np.linalg.norm(res.t - scene["t"]) < 0.001
    assert res.passed
    assert res.roi[1] >= 40                         # 평면 맞춤 영역이 위쪽 벽을 포함하지 않음
    assert all(c.ok for c in res.checks if c.blocking)
    # 1.3 m·55° 시점에서는 멀어지는 방향의 변이 화면에서 가장 짧다 (synth.LAYOUT에서 약 95 px)
    assert res.check("a4_size").value == pytest.approx(95, abs=3)
    print(f"\n홀드아웃 {res.holdout_m * 100:.2f} cm, 변 {np.round(res.sides_m * 100, 2)} cm, "
          f"기울기 {res.tilt_deg:.1f}°, A4 짧은 변 {res.a4_short_px:.0f} px, 책상면 z {res.desk_z}")


def test_point_mode_also_works(scene):
    res = solve(scene, mode="point")
    assert np.linalg.norm(res.t - scene["t"]) < 0.01
    assert res.check("reflection").ok


def test_point_mode_reports_missing_depth(scene):
    depth = scene["depth"].copy()
    ud, vd = scene["cam"].rgb_to_depth(*scene["clicks"][2])
    depth[int(vd) - 5:int(vd) + 6, int(ud) - 5:int(ud) + 6] = 0
    with pytest.raises(CalibrationError) as e:
        solve(scene, depth=depth, mode="point")
    assert e.value.indices == [2]


def test_shifted_base_mark_fails_holdout(scene):
    """base 표시가 3 cm 밀린 자리를 클릭 → 홀드아웃 실패, A4 변 길이는 통과."""
    moved = scene["base_pts"].copy()
    moved[0] = [0.03, 0.0, 0.0]
    res = solve(scene, clicks=synth.clicks_for(scene["cam"], scene["R"], scene["t"], moved))
    assert not res.check("base_error").ok
    assert res.holdout_m == pytest.approx(0.03, abs=0.003)
    assert res.check("a4_sides").ok
    assert not res.passed


def test_misclicked_corner_fails_side_length(scene):
    moved = scene["base_pts"].copy()
    moved[2] = moved[2] + [0.02, 0.0, 0.0]          # 꼭짓점 2를 2 cm 바깥(+X)에 클릭
    res = solve(scene, clicks=synth.clicks_for(scene["cam"], scene["R"], scene["t"], moved))
    assert not res.check("a4_sides").ok


def test_reversed_corner_order_is_reflection(scene):
    """꼭짓점을 시계 방향(1, 4, 3, 2)으로 찍으면 반사로 감지 (법선 손잡이)."""
    clicks = scene["clicks"][[0, 1, 4, 3, 2]]
    res = solve(scene, clicks=clicks)
    assert not res.handedness
    assert not res.check("reflection").ok


def test_missing_depth_flip_is_reflection(scene):
    """depth 좌우반전을 빠뜨린 경우: 거울상 depth + 거울상 클릭 → 반사 감지."""
    cam = scene["cam"]
    depth = scene["depth"][:, ::-1]
    ud, vd = cam.rgb_to_depth(scene["clicks"][:, 0], scene["clicks"][:, 1])
    ud_m = (cam.width - 1) - ud
    u, v = cam.depth_to_rgb(ud_m, vd)
    res = solve(scene, clicks=np.stack([u, v], axis=1), depth=depth)
    assert not res.handedness
    assert not res.check("reflection").ok


def test_handedness_and_camera_z_are_separate_checks(scene):
    res = solve(scene)
    corners_c = res.cam_points[1:]
    mirrored = corners_c * [-1, 1, 1]
    n_mirror = res.plane_normal * [-1, 1, 1]
    ok, _, _ = handedness_ok(mirrored, n_mirror, res.base_points[1:])
    assert not ok
    assert camera_above_desk([0, 0, 0.7]) and not camera_above_desk([0, 0, -0.7])


def test_desk_z_check_uses_object_min_height(scene):
    """책상면 z 판정: 먼 쪽 95 백분위 < 물체 최소 높이 (thresholds 값 공유)."""
    res = solve(scene)
    c = res.check("desk_z")
    assert c.limit == scene["cfg"]["thresholds"]["objects"]["min_height_m"]
    assert c.ok and c.value < c.limit and not c.blocking
    assert set(res.desk_z) == {"near", "far"}


@pytest.mark.parametrize("rgb_size,mirrored", [((640, 480), False), ((1280, 720), False), ((1280, 720), True)])
def test_pixel_base_round_trip(scene, rgb_size, mirrored):
    conv = Cam2Base(scene["R"], scene["t"], scene["cam"])
    pts = np.array([[0.20, 0.20, 0.0], [0.05, 0.30, 0.0], [0.25, 0.05, 0.0]])
    uv = conv.base_to_pixel(pts, rgb_mirrored=mirrored, rgb_size=rgb_size)
    for (u, v), p in zip(uv, pts):
        r = conv.pixel_to_base(u, v, scene["depth"], rgb_mirrored=mirrored, rgb_size=rgb_size)
        assert r.valid, r.reason
        assert np.linalg.norm(r.xyz - p) < 0.005


def test_16x9_screen_is_center_band():
    """1280x720 화면 = 640x480의 가운데 띠: 가로 0.5배, 세로 0.5배 후 +60 px."""
    cam = synth.camera()
    ur, vr = cam.screen_to_ref([0, 640, 1279], [0, 360, 719], rgb_size=(1280, 720))
    assert np.allclose(ur, [0, 320, 639.5]) and np.allclose(vr, [60, 240, 419.5])
    ur, vr = cam.screen_to_ref(0, 0, rgb_size=(1280, 720), mirrored=True)
    assert np.isclose(ur, 639.5) and np.isclose(vr, 60)
    u, v = cam.ref_to_screen(*cam.screen_to_ref(123.0, 456.0, (1280, 720), True), (1280, 720), True)
    assert np.isclose(u, 123.0) and np.isclose(v, 456.0)


def _stretch_side(clicks, side, px):
    """변 side(0: 1-2 …)의 끝 꼭짓점을 그 변 방향으로 px만큼 밀어 화면 길이를 늘인다."""
    a, b = 1 + side, 1 + (side + 1) % 4
    d = clicks[b] - clicks[a]
    out = clicks.copy()
    out[b] = clicks[b] + d / np.linalg.norm(d) * px
    return out


def test_side_tolerance_follows_pixel_size(scene):
    """멀어지는 변은 1 px ≈ 4 mm. 허용 오차 = max(5 mm, 그 변 방향 2 px) → 1 px 클릭 오차는 통과, 3 px는 실패."""
    res = solve(scene)
    i = res.a4_short_side
    assert res.a4_short_m_per_px == pytest.approx(res.sides_expected_m[i] / res.sides_px[i])
    assert 0.003 < res.a4_short_m_per_px < 0.005
    assert res.sides_tol_m[i] == pytest.approx(2 * res.a4_short_m_per_px) and res.sides_tol_m[i] > 0.005
    assert np.all(res.sides_tol_m >= 0.005)

    one = solve(scene, clicks=_stretch_side(scene["clicks"], i, 1.0))
    assert one.check("a4_sides").ok
    three = solve(scene, clicks=_stretch_side(scene["clicks"], i, 3.0))
    assert not three.check("a4_sides").ok
    print(f"\n가장 짧게 보이는 변 {i + 1}: {res.a4_short_px:.0f} px (1 px ≈ {res.a4_short_m_per_px * 1000:.1f} mm), "
          f"허용 {np.round(res.sides_tol_m * 1000, 1)} mm, 1 px 오차 {abs(one.sides_m[i] - one.sides_expected_m[i]) * 1000:.1f} mm")
