"""평면 맞춤 — 기울어진 책상, 노이즈·이상치, 영역 밖의 벽."""
import numpy as np

from pac_vision.cam2base import clip_roi, fit_plane, intersect_ray_plane, roi_points
import synth


def desk_normal_in_camera(R):
    """base +Z(책상 위쪽)를 카메라 좌표로 = 카메라 쪽 법선."""
    return np.asarray(R).T @ np.array([0.0, 0.0, 1.0])


def angle_deg(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def test_tilted_desk_with_noise_outliers_and_wall():
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    tilt = angle_deg(desk_normal_in_camera(R), [0, 0, -1])     # 책상면 법선과 광축 사이 각
    assert 50 < tilt < 60                                  # 상당히 기울어진 시점
    depth = synth.render_depth(cam, R, t, wall_rows=120, noise_mm=1.0, outlier_frac=0.05)

    roi = clip_roi(150, 200, 500, 440, cam.width, cam.height)   # 벽(위쪽 120줄) 밖의 영역
    pts, _, _, _ = roi_points(depth, cam, roi)
    plane = fit_plane(pts, iters=300, inlier_m=0.005)
    assert angle_deg(plane.normal, desk_normal_in_camera(R)) < 1.0
    assert plane.d > 0 and plane.rms_m < 0.003
    print(f"\n기울기 {tilt:.1f}°, 법선 오차 {angle_deg(plane.normal, desk_normal_in_camera(R)):.3f}°, "
          f"잔차 {plane.rms_m * 1000:.2f} mm, 인라이어 {plane.inlier_ratio:.0%}")

    # 광선-평면 교점은 실제 책상면 위 (base z ≈ 0)
    uv = np.array([[200.0, 300.0], [450.0, 420.0]])
    p = intersect_ray_plane(cam.ray(uv[:, 0], uv[:, 1]), plane)
    z_base = p @ np.asarray(R).T[:, 2] + t[2]
    assert np.all(np.abs(z_base) < 0.002)


def test_whole_image_can_pick_the_wall():
    """화면 전체에서 맞추면 큰 벽을 잡을 수 있다 → 클릭 영역 안에서만 맞추는 이유."""
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    depth = synth.render_depth(cam, R, t, wall_rows=300)
    pts, _, _, _ = roi_points(depth, cam, (0, 0, cam.width, cam.height))
    plane = fit_plane(pts, iters=300, inlier_m=0.005)
    assert angle_deg(plane.normal, desk_normal_in_camera(R)) > 10
