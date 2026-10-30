"""책상면 근처(±2 cm) 점만 쓰기: --check 평면 맞춤이 책상 끝 너머 바닥을 잡지 않음, 책상면 z 분포에서 바닥·물체 제외."""
import numpy as np

from pac_vision.cam2base import Cam2Base, fit_plane, fit_desk_plane_near, roi_points, z_distribution
import synth

FLOOR_DROP = 0.42          # 책상 너머 바닥이 책상면보다 이만큼 낮음 (2026-10-14 현장과 비슷)
EDGE_X = 0.40              # 책상 끝 (base x)


def scene_with_floor():
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    desk = synth.render_depth(cam, R, t)
    floor = synth.render_depth(cam, R, np.asarray(t) + [0.0, 0.0, FLOOR_DROP])     # 같은 광선, 평면만 0.42 m 아래
    conv = Cam2Base(R, t, cam)
    pts, ud, vd, _ = roi_points(desk, cam, (0, 0, cam.width, cam.height))
    beyond = np.zeros(desk.shape, bool)
    beyond[vd.astype(int), ud.astype(int)] = conv.camera_to_base(pts)[:, 0] > EDGE_X
    depth = np.where(beyond, floor, desk).astype(np.float32)
    return cam, conv, depth


def region(cam, conv, depth, target, m=40):
    u, v = conv.base_to_pixel(target)[0]
    ud, vd = cam.rgb_to_depth(u, v)
    pts, _, _, _ = roi_points(depth, cam, (int(ud) - m, int(vd) - m, int(ud) + m + 1, int(vd) + m + 1))
    return u, v, pts


def test_old_plane_fit_picks_floor_at_desk_edge():
    """재현: 클릭 주변의 절반 넘게 바닥이면 예전 방식(영역 전체로 평면 맞춤)은 바닥 평면을 잡는다."""
    cam, conv, depth = scene_with_floor()
    _, _, pts = region(cam, conv, depth, np.array([[EDGE_X - 0.002, 0.0, 0.0]]))
    assert (conv.camera_to_base(pts)[:, 2] < -0.3).mean() > 0.5
    old = fit_plane(pts)
    assert abs(conv.camera_to_base(-old.d * old.normal)[2] + FLOOR_DROP) < 0.01


def test_check_plane_ignores_floor_beyond_desk_edge():
    cam, conv, depth = scene_with_floor()
    for back in (0.05, 0.02, 0.002):                          # 책상 끝에서 이만큼 안쪽 점
        target = np.array([[EDGE_X - back, 0.0, 0.0]])
        u, v, pts = region(cam, conv, depth, target)
        near = fit_desk_plane_near(pts, conv.R, conv.t, 0.02)
        p = conv.pixel_to_desk(u, v, plane=near)[0][0]
        assert np.max(np.abs(p - target[0])) < 0.002


def test_desk_z_distribution_excludes_floor_and_reports_raised():
    rng = np.random.default_rng(0)
    desk = rng.normal(0.0, 0.002, 1000)
    raised = np.full(50, 0.015)                               # 1-2 cm 떠 보이는 점 (책상이 물체로 세질 위험)
    floor = np.full(300, -FLOOR_DROP)
    z = np.r_[desk, raised, floor]
    depth_z = np.r_[np.linspace(0.8, 1.3, 1050), np.full(300, 1.4)]
    d = z_distribution(z, depth_z, band_m=0.02, raised_m=0.01)
    assert set(d) == {"near", "far"}
    assert abs(d["near"]["median"]) < 0.002 and d["far"]["p5"] > -0.02          # 바닥(−42 cm)이 섞이지 않음
    assert d["far"]["n_outside"] == 300 and d["near"]["n_outside"] == 0
    total_raised = (d["near"]["raised"] * d["near"]["n"] + d["far"]["raised"] * d["far"]["n"])
    assert round(total_raised) == 50
