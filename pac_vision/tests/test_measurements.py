"""README 측정 수치를 data/measurements/에서 다시 계산 (tools/recompute_measurements.py).

arithmetic: README 표를 옮겨 적은 2026-07 값으로 한 산술 검산(측정 재현 아님). 원 측정 로그가 없다.
raw: check_camera.py가 저장한 2026-10-02 CSV 원본으로 한 원자료 재계산 (2회차 8쌍만 터미널 출력을 옮긴 값).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import recompute_measurements as rm  # noqa: E402


def test_sec2_intrinsics_from_fov_arithmetic():
    """산술 검산(측정 재현 아님): 기록된 화각에서 fx, fy 계산."""
    r = rm.sec2_intrinsics()
    assert round(r["fx_from_hfov"]) == 570 and round(r["fy_from_vfov"]) == 571


def test_sec4_z_offset_arithmetic():
    """산술 검산(측정 재현 아님): README 표를 옮긴 값에서 Z_OFFSET 평균·스케일."""
    r = rm.sec4_z_offset()
    assert round(r["z_offset_m"], 3) == -0.021
    assert abs(1 - r["scale_change"]) < 0.005          # 0.4 %


def test_sec5_fx_check_ratio_arithmetic():
    """산술 검산(측정 재현 아님): ΔX/실제 = 0.96 (보정식대로면 fx ≈ 547, 사용값 570)."""
    r = rm.sec5_fx()
    assert r["delta_x_m"] == pytest.approx(0.288)
    assert r["ratio"] == pytest.approx(0.96)


def test_sec7_calibration_table_arithmetic():
    """산술 검산(측정 재현 아님): README 표를 옮긴 9점으로 RMS·R·t·검증용 점."""
    r = rm.sec7_calibration()
    assert [round(r[k]["rms_m"] * 100, 2) for k in ("all9", "excl6", "excl6_8", "excl3_6_8")] == [19.38, 4.17, 2.26, 2.42]
    assert np.allclose(r["excl6_8"]["t"], [-1.72318, -0.03662, 0.87503], atol=1e-5)
    assert round(r["holdout_3d_m"] * 100, 2) == 8.80 and round(r["holdout_xy_m"] * 100, 2) == 5.73


def test_sec11_pose_a_raw():
    """원자료로 재계산: check_camera.py CSV 원본(1회차) + 터미널 옮김(2회차 8쌍)."""
    r = rm.sec11_pose_a()
    assert r["n"] == 20
    assert round(r["dx_mean"]) == 23 and round(r["dy_mean"]) == 8
    assert round(r["dx_fixed_rms"], 1) == 3.7 and round(r["dy_fixed_rms"], 1) == 3.9
    assert round(r["dx_linear_rms"], 1) == 2.2 and round(r["dy_linear_rms"], 1) == 2.7
    assert round(r["corr_v_z"], 2) == -0.90
    assert np.allclose(r["dx_distance_model"], [-4.3, -0.0052, 28.3], atol=[0.05, 0.00005, 0.05])
    assert r["x_right_box_m"] > 0
    assert round(r["range_box_m"] - r["tape_m"], 3) == -0.013


# --- 12절: 2026-10-14 설치 자세 -------------------------------------------------------------
@pytest.fixture(scope="module")
def sec12():
    return rm.sec12_pose_2026_10_14()


def test_sec12_rgb_depth_fit_raw(sec12):
    """원자료로 재계산: check_camera.py가 저장한 쌍 CSV 원본 두 개 (24쌍)."""
    q = sec12["pairs"]
    assert (q["n_run1"], q["n_all"], q["n_fit"]) == (14, 24, 20) and q["excluded"] == [2, 3]
    assert [round(q[k], 2) for k in ("run1_loo_fixed_dx", "run1_loo_dist_dx", "run1_loo_dist_dy", "run1_rule_loo_dist_dx")] \
        == [8.98, 4.96, 1.31, 2.82]
    assert round(q["loo_dist_dx"], 2) == 2.65 and round(q["loo_dist_dy"], 2) == 1.59 and q["dy_uses_distance"]
    assert round(q["corr_c_d"], 2) == -0.81 and [round(k, 1) for k in q["direct_parallax"]] == [26.6]


def test_sec12_calibration_round1_and_fx_raw(sec12):
    """원자료로 재계산: tests/data/sample_session.npz(1회차 세션)를 fx 570 / 547로 다시 계산."""
    c = sec12["round1"]
    assert c["passed"] and round(c["holdout_m"] * 1000, 1) == 0.7 and round(c["rms_m"] * 1000, 1) == 1.6
    assert np.allclose(np.round(c["sides_err_m"] * 1000, 1), [2.6, -3.8, 0.4, -1.5])
    assert round(c["ratio570"][[0, 2]].mean() * 100, 2) == 0.50 and round(c["ratio570"][[1, 3]].mean() * 100, 2) == -0.63
    assert round(c["ratio547"][[0, 2]].mean() * 100, 2) == 4.70           # fx 547이면 가로 변만 약 5 % 길어짐


def test_sec12_round2_check_objects_arithmetic(sec12):
    """산술 검산(측정 재현 아님): 2회차는 세션에서 뽑은 CSV, --check·물체 좌표는 터미널 출력을 옮긴 CSV."""
    assert np.allclose(np.round(sec12["round2"]["sides_err_m"] * 1000, 1), [3.2, -1.2, 1.8, 6.7])
    assert round(sec12["round2"]["ratio570"][[0, 2]].mean() * 100, 2) == 0.83
    e = sec12["check_err_mm"]
    assert round(e["recheck_corner4"][0], 1) == 3.2 and round(e["S1"][0], 1) == 3.4 and round(e["ext_c"][0], 1) == 8.5
    assert sec12["check_round12_max_mm"] < 0.5
    o = sec12["objects_mm"]
    assert round(o[("setting_compare", "cup_lid", "ext_a", "current")][1], 1) == -33.6
    assert round(o[("depth_diag", "box_48mm", "ext_a", "current")][1], 1) == -46.4
