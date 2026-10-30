"""물체 윗면 중심 — 비스듬한 시점의 원기둥, 최근접 대체 선택."""
import numpy as np

from pac_vision.cam2base import Cam2Base, roi_points
from pac_vision.objects import box_to_depth_roi, nearest_detection, top_center
import synth

CYL = (0.40, 0.0, 0.035, 0.12)      # 중심 x, y, 반지름, 높이 (컵·병 크기)


def test_top_center_of_oblique_cylinder():
    cfg = synth.repo_config()["thresholds"]["objects"]
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    depth = synth.render_depth(cam, R, t, cylinders=[CYL], noise_mm=1.0)
    conv = Cam2Base(R, t, cam)

    # 원기둥을 감싸는 RGB 박스 (YOLO 박스 대신)
    xc, yc, r, h = CYL
    corners = np.array([[xc + dx, yc + dy, z] for dx in (-r, r) for dy in (-r, r) for z in (0, h)])
    uv = conv.base_to_pixel(corners)
    box = (uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max())
    roi = box_to_depth_roi(box, cam, shrink=0.0)
    pts, _, _, _ = roi_points(depth, cam, roi)
    p_base = conv.camera_to_base(pts)

    tc = top_center(p_base, cfg["min_height_m"], cfg["max_height_m"], cfg["top_percentile"],
                    cfg["top_band_m"], cfg["min_pixels"])
    assert tc.valid
    assert abs(tc.xyz[2] - h) < 0.005
    assert np.linalg.norm(tc.xyz[:2] - [xc, yc]) < 0.01

    # 단순 중앙값은 윗면보다 낮고 카메라 쪽(-X)으로 치우친다
    naive = np.median(p_base[tc.object_mask], axis=0)
    assert h - naive[2] > 0.02
    assert np.linalg.norm(naive[:2] - [xc, yc]) > np.linalg.norm(tc.xyz[:2] - [xc, yc])
    print(f"\n윗면 중심 {np.round(tc.xyz, 4)} / 단순 중앙값 {np.round(naive, 4)} / 실제 ({xc}, {yc}, {h})")


def test_flat_object_has_too_few_pixels():
    pts = np.column_stack([np.random.default_rng(0).uniform(0, 0.1, (500, 2)), np.full(500, 0.005)])
    tc = top_center(pts, 0.01, 0.5, 90, 0.015, 30)
    assert not tc.valid and tc.reason == "few_pixels"


def test_nearest_detection_fallback():
    cands = [("cup", [0.42, 0.18, 0.09]), ("bottle", [0.40, 0.17, 0.2]), ("cup", [0.60, 0.0, 0.09]),
             ("cup", None)]
    assert nearest_detection([0.40, 0.17], "cup", cands, 0.05) == 0
    assert nearest_detection([0.40, 0.17], "bottle", cands, 0.05) == 1
    assert nearest_detection([0.10, 0.10], "cup", cands, 0.05) is None     # 거리 한계 밖
