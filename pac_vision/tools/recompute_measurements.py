"""README "측정 결과" 수치를 data/measurements/ 원자료에서 다시 계산한다 (카메라 불필요, numpy만. 12절은 pac_vision 패키지도 사용).

python tools/recompute_measurements.py            # 전체
python tools/recompute_measurements.py --section 7

절 번호는 README "측정 결과와 재현 방법"의 번호와 같다. 원자료가 없는 절(1, 6, 8 일부, 10)은 계산하지 않는다.
2·4·5·7절은 README 표를 옮긴 값으로 한 산술 검산(측정 재현 아님), 3·11절은 2026-10-02 CSV 원본으로 한 원자료 재계산이다.
12절(2026-10-14 설치 자세): 정합 계수·1회차 캘리브레이션·fx 판정은 원자료(쌍 CSV 원본, tests/data/sample_session.npz)로 재계산,
2회차·재점검·A4 밖 외삽점·물체 좌표는 세션·터미널 출력에서 옮긴 CSV로 한 산술 검산이다.
13절(두 설치 자세 합친 정합 거리 식, 카메라 고유 계수 시험 중): 자세 A(2026-10-02)·B(2026-10-14) 쌍 CSV로 재계산.
자세 A 2회차 8쌍은 터미널 출력을 옮긴 값이다.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parents[1] / "data" / "measurements"
FX, CX = 570.0, 320.0          # 측정 당시 intrinsics (config/camera.yaml과 같은 값)


def read_csv(name, data_dir=DATA):
    with open(Path(data_dir) / name, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def col(rows, key):
    return np.array([float(r[key]) for r in rows])


def kabsch(a, b):
    """b ≈ R a + t (pac_vision.cam2base.kabsch와 같은 식)."""
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, cb - r @ ca


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x, float) ** 2)))


def sec2_intrinsics():
    return {"fx_from_hfov": 320 / np.tan(np.radians(58.6 / 2)), "fy_from_vfov": 240 / np.tan(np.radians(45.6 / 2))}


def sec4_z_offset(data_dir=DATA):
    r = read_csv("2026-07_z_offset.csv", data_dir)
    actual, meas = col(r, "actual_m"), col(r, "measured_z_raw_m")
    diff = meas - actual
    return {"diff_m": diff, "z_offset_m": float(diff.mean()),
            "scale_change": float((meas[-1] - meas[0]) / (actual[-1] - actual[0]))}


def sec5_fx(data_dir=DATA):
    r = read_csv("2026-07_fx_check.csv", data_dir)[0]
    d = float(r["x_after_m"]) - float(r["x_before_m"])
    actual = float(r["actual_shift_m"])
    return {"delta_x_m": d, "ratio": d / actual, "fx_if_corrected": float(r["fx_used"]) * d / actual}


def sec7_calibration(data_dir=DATA):
    r = read_csv("2026-07-25_calib_points.csv", data_dir)
    pid = col(r, "point").astype(int)
    base = np.c_[col(r, "base_x"), col(r, "base_y"), col(r, "base_z")]
    cam = np.c_[col(r, "cam_x"), col(r, "cam_y"), col(r, "cam_z")]
    out = {}
    for label, excl in (("all9", set()), ("excl6", {6}), ("excl6_8", {6, 8}), ("excl3_6_8", {3, 6, 8})):
        keep = np.array([p not in excl for p in pid])
        R, t = kabsch(cam[keep], base[keep])
        err = np.linalg.norm(cam[keep] @ R.T + t - base[keep], axis=1)
        out[label] = {"rms_m": rms(err), "det": float(np.linalg.det(R)), "R": R, "t": t}
    h = read_csv("2026-07-25_calib_holdout.csv", data_dir)
    act = np.c_[col(h, "actual_x"), col(h, "actual_y"), col(h, "actual_z")]
    outp = np.c_[col(h, "output_x"), col(h, "output_y"), col(h, "output_z")]
    out["holdout_3d_m"] = rms(np.linalg.norm(outp - act, axis=1))
    out["holdout_xy_m"] = rms(np.linalg.norm((outp - act)[:, :2], axis=1))
    return out


def sec11_pose_a(data_dir=DATA):
    p1 = read_csv("2026-10-02_poseA_pairs_run1.csv", data_dir)
    p2 = read_csv("2026-10-02_poseA_pairs_run2_transcribed.csv", data_dir)
    u = np.r_[col(p1, "u_ref"), col(p2, "u")]
    v = np.r_[col(p1, "v_ref"), col(p2, "v")]
    dx = np.r_[col(p1, "dx"), col(p2, "dx")]
    dy = np.r_[col(p1, "dy"), col(p2, "dy")]
    z = np.r_[col(p1, "Z"), col(p2, "Z")]
    A = np.c_[np.ones_like(u), u, v]
    rx = dx - A @ np.linalg.lstsq(A, dx, rcond=None)[0]
    ry = dy - A @ np.linalg.lstsq(A, dy, rcond=None)[0]
    B = np.c_[np.ones_like(u), u, 1 / z]
    cb = np.linalg.lstsq(B, dx, rcond=None)[0]
    pts = read_csv("2026-10-02_poseA_points_run1.csv", data_dir)
    tape = read_csv("2026-10-02_poseA_tape.csv", data_dir)[0]

    def point(uu, vv):
        return next(q for q in pts if int(q["u"]) == uu and int(q["v"]) == vv)
    zp = point(int(tape["u_depth"]), int(tape["v_depth"]))
    pz = np.array([float(zp["X"]), float(zp["Y"]), float(zp["Z"])])
    right = point(465, 249)
    desk = [q for q in pts if q["where"].startswith(("아래쪽", "위쪽"))]
    return {
        "n": len(dx), "dx_range": (dx.min(), dx.max()), "dy_range": (dy.min(), dy.max()),
        "dx_mean": float(dx.mean()), "dy_mean": float(dy.mean()), "z_range": (z.min(), z.max()),
        "u_range": (u.min(), u.max()), "v_range": (v.min(), v.max()),
        "dx_fixed_rms": rms(dx - dx.mean()), "dy_fixed_rms": rms(dy - dy.mean()),
        "dx_linear_rms": rms(rx), "dy_linear_rms": rms(ry),
        "corr_v_z": float(np.corrcoef(v, z)[0, 1]),
        "dx_distance_model": cb, "dx_distance_rms": rms(dx - B @ cb),
        "repeat_pairs": [(18, 17, 14, 13), (29, 30, 6, 7)],
        "x_right_box_m": float(right["X"]),
        "z_box_m": float(pz[2]), "range_box_m": float(np.linalg.norm(pz)), "tape_m": float(tape["tape_m"]),
        "desk_points": [(q["where"], float(q["Z"]), float(q["sd_single"]), int(float(q["n_single"]))) for q in desk],
    }


def sec12_pose_2026_10_14(data_dir=DATA):
    """2026-10-14 설치 자세: 정합 계수, 캘리브레이션 2회, fx 판정, --check, 물체 좌표."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import fit_rgb_to_depth as fr
    from pac_vision.session import replay_session
    root = Path(__file__).resolve().parents[1]
    pairs = [Path(data_dir) / "2026-10-14_pose_pairs.csv", Path(data_dir) / "2026-10-14_pose_pairs_run2.csv"]
    out = {}
    # 정합: 첫 측정만 (제외 규칙 전·후), 두 측정 합침 (채택)
    p1 = fr.load_pairs(pairs[0])
    rep1 = fr.fit_report(p1, p1[:, 4] <= 1.5)
    m1, _, _ = fr.select_for_fit(p1)
    rep1r = fr.fit_report(p1, m1)
    p, src = fr.load_many(pairs)
    mask, _, excluded = fr.select_many(p, src)
    cx, cy, use_dist_dy, rep = fr.choose(p, mask)
    out["pairs"] = {"n_run1": len(p1), "n_all": len(p), "n_fit": int(mask.sum()), "excluded": excluded,
                    "run1_loo_fixed_dx": rep1[("dx", "fixed")]["loo"], "run1_loo_dist_dx": rep1[("dx", "dist4")]["loo"],
                    "run1_loo_dist_dy": rep1[("dy", "dist4")]["loo"], "run1_rule_loo_dist_dx": rep1r[("dx", "dist4")]["loo"],
                    "loo_dist_dx": rep[("dx", "dist4")]["loo"], "loo_dist_dy": rep[("dy", "dist4")]["loo"],
                    "loo_fixed_dx": rep[("dx", "fixed")]["loo"], "loo_fixed_dy": rep[("dy", "fixed")]["loo"],
                    "coef_dx": cx, "coef_dy": cy, "dy_uses_distance": use_dist_dy,
                    "corr_c_d": float(fr.coef_correlation(p[mask])[2, 3]),
                    "direct_parallax": [k for _, _, k in fr.direct_parallax(p[p[:, 4] <= 1.5])]}
    # 캘리브레이션 1회차: 샘플 세션 replay (원자료), fx 570 / 547
    sample = root / "tests" / "data" / "sample_session.npz"
    expected = np.array([0.297, 0.42, 0.297, 0.42])
    r570, s = replay_session(sample)
    cam547 = {**s["configs"]["camera"], "depth": {**s["configs"]["camera"]["depth"], "fx": 547.0, "fy": 547.0}}
    r547, _ = replay_session(sample, camera_cfg=cam547)
    out["round1"] = {"holdout_m": r570.holdout_m, "sides_err_m": r570.sides_m - r570.sides_expected_m,
                     "rms_m": rms(r570.residuals_m), "passed": r570.passed, "plane_rms_m": r570.plane_rms_m,
                     "ratio570": r570.sides_m / expected - 1, "ratio547": r547.sides_m / expected - 1}
    # 2회차: 세션에서 뽑은 CSV (세션 파일은 git 제외)
    rounds = read_csv("2026-10-14_pose_calib_rounds.csv", data_dir)
    r2 = [r for r in rounds if r["round"] == "round2"][0]
    out["round2"] = {"holdout_m": float(r2["holdout_mm"]) / 1000, "rms_m": float(r2["rms5_mm"]) / 1000,
                     "sides_err_m": np.array([float(r2[k]) for k in ("side12_mm", "side23_mm", "side34_mm", "side41_mm")]) / 1000}
    out["round2"]["ratio570"] = (expected + out["round2"]["sides_err_m"]) / expected - 1
    # --check (로그에서 옮긴 클릭과 결과)
    chk = {r["point"]: r for r in read_csv("2026-10-14_pose_check.csv", data_dir)}
    out["check_err_mm"] = {k: (float(v["err_local_mm"]), float(v["err_desk_mm"])) for k, v in chk.items()}
    out["check_round12_max_mm"] = max(float(v["round12_diff_mm"]) for v in chk.values())
    # 물체 좌표 (detect·진단 터미널 출력)
    obj = read_csv("2026-10-14_pose_objects.csv", data_dir)

    def err(rows):
        a = np.array([[float(r[k]) - float(r["exp_" + k]) for k in "xyz"] for r in rows if r["x"]])
        return a.mean(axis=0) * 1000
    out["objects_mm"] = {}
    for key in (("object_vs_tape", "cup_lid", "ext_a", "current"), ("object_vs_tape", "cup_lid", "ext_c", "current"), ("object_vs_tape", "cup_open", "ext_a", "current"),
                ("setting_compare", "cup_lid", "ext_a", "current"), ("setting_compare", "cup_lid", "ext_a", "alt"),
                ("setting_compare", "cup_lid", "ext_c", "current"), ("setting_compare", "cup_lid", "ext_c", "alt"),
                ("depth_diag", "cup_lid", "ext_a", "current"), ("depth_diag", "box_48mm", "ext_a", "current"), ("depth_diag", "box_48mm", "ext_a", "alt")):
        rows = [r for r in obj if (r["test"], r["object"], r["spot"], r["setting"]) == key]
        out["objects_mm"][key] = err(rows)
    return out


def sec13_registration_two_poses(data_dir=DATA):
    """두 설치 자세(2026-10-02 자세 A, 2026-10-14 자세 B) 합친 RGB→depth 거리 식 (카메라 고유 계수 시험 중)."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import fit_rgb_to_depth as fr
    d = Path(data_dir)
    a = [d / "2026-10-02_poseA_pairs_run1.csv", d / "2026-10-02_poseA_pairs_run2_transcribed.csv"]
    b = [d / "2026-10-14_pose_pairs.csv", d / "2026-10-14_pose_pairs_run2.csv"]
    out = {}
    for key, poses in (("AB", [("A", a), ("B", b)]), ("A1B", [("A1", a[:1]), ("B", b)])):
        data = fr.load_poses(poses)
        rep = fr.analyze_poses(data, "desk")
        out[key] = {"n": rep["n"], "corr_c_d": float(rep["coef_corr"][2, 3]), "dy_auto": fr.choose_dy(rep),
                    "coef": {k: v["coef"] for k, v in rep["models"].items()},
                    "pair": {k: {n: s["rms"] for n, s in v["pair"].items()} for k, v in rep["models"].items()},
                    "corner": {k: {n: s["rms"] for n, s in v["corner"].items()} for k, v in rep["models"].items()},
                    "lopo": {k: (v["rms"], v["max"], v["extrapolate"]) for k, v in rep["lopo"].items()}}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--section", type=int, choices=[2, 4, 5, 7, 11, 12, 13])
    s = ap.parse_args().section
    if s in (None, 2):
        r = sec2_intrinsics()
        print(f"[2] fx = 320/tan(58.6°/2) = {r['fx_from_hfov']:.1f}, fy = 240/tan(45.6°/2) = {r['fy_from_vfov']:.1f}  (사용값 570)")
    if s in (None, 4):
        r = sec4_z_offset()
        print(f"[4] 차이 {np.round(r['diff_m'], 3).tolist()} m → 평균 {r['z_offset_m']:+.4f} m (사용값 -0.021), "
              f"0.8→2.0 m 측정 변화 / 실제 변화 = {r['scale_change']:.4f}")
    if s in (None, 5):
        r = sec5_fx()
        print(f"[5] ΔX {r['delta_x_m']:.3f} m / 실제 0.30 m = {r['ratio']:.3f} → 보정식을 적용하면 fx {r['fx_if_corrected']:.0f} (사용값 570 유지)")
    if s in (None, 7):
        r = sec7_calibration()
        for k, name in (("all9", "9점"), ("excl6", "점6 제외"), ("excl6_8", "점6·8 제외(채택)"), ("excl3_6_8", "점3·6·8 제외")):
            print(f"[7] {name}: RMS {r[k]['rms_m'] * 100:.2f} cm, det(R) {r[k]['det']:+.0f}")
        np.set_printoptions(precision=5, suppress=True)
        print(f"[7] 채택 R =\n{r['excl6_8']['R']}\n[7] 채택 t = {r['excl6_8']['t']}")
        print(f"[7] 검증용 점 RMS: 3D {r['holdout_3d_m'] * 100:.2f} cm, 수평 {r['holdout_xy_m'] * 100:.2f} cm")
    if s in (None, 11):
        r = sec11_pose_a()
        print(f"[11] 좌우반전: 오른쪽 상자 X = {r['x_right_box_m']:+.3f} m")
        print(f"[11] Z: 상자 Z {r['z_box_m']:.3f} m, 직선거리 {r['range_box_m']:.3f} m, 줄자 {r['tape_m']:.2f} m "
              f"(차이 {(r['range_box_m'] - r['tape_m']) * 100:+.1f} cm)")
        print(f"[11] 정합 {r['n']}쌍: dx {r['dx_range'][0]:.0f}-{r['dx_range'][1]:.0f} (평균 {r['dx_mean']:.1f}), "
              f"dy {r['dy_range'][0]:.0f}-{r['dy_range'][1]:.0f} (평균 {r['dy_mean']:.1f}), "
              f"Z {r['z_range'][0]:.2f}-{r['z_range'][1]:.2f} m, u {r['u_range'][0]:.0f}-{r['u_range'][1]:.0f}, "
              f"v {r['v_range'][0]:.0f}-{r['v_range'][1]:.0f}")
        print(f"[11] 잔차 RMS: 고정값 dx {r['dx_fixed_rms']:.1f} / dy {r['dy_fixed_rms']:.1f} px, "
              f"위치 1차식 dx {r['dx_linear_rms']:.1f} / dy {r['dy_linear_rms']:.1f} px, v·Z 상관 {r['corr_v_z']:+.2f}")
        c = r["dx_distance_model"]
        print(f"[11] 거리 식: dx ≈ {c[0]:+.1f} {c[1]:+.4f}·u {c[2]:+.1f}/Z (잔차 {r['dx_distance_rms']:.1f} px)")
        for where, zz, sd, n in r["desk_points"]:
            print(f"[11] 책상면 {where}: Z {zz:.3f} m, 픽셀 하나 30프레임 σ {sd:.1f} mm (유효 {n}/30)")

    if s in (None, 12):
        r = sec12_pose_2026_10_14()
        q = r["pairs"]
        print(f"[12] 정합 첫 측정 {q['n_run1']}쌍 (Z ≤ 1.5 m): LOO 고정 dx {q['run1_loo_fixed_dx']:.2f}, "
              f"거리 식 dx {q['run1_loo_dist_dx']:.2f} / dy {q['run1_loo_dist_dy']:.2f} px, 제외 규칙 적용 거리 식 dx {q['run1_rule_loo_dist_dx']:.2f} px")
        print(f"[12] 정합 두 측정 {q['n_all']}쌍 → 맞춤 {q['n_fit']}쌍 (제외 {[i + 1 for i in q['excluded']]}): "
              f"LOO 거리 식 dx {q['loo_dist_dx']:.2f} / dy {q['loo_dist_dy']:.2f} px, 고정 dx {q['loo_fixed_dx']:.2f} / dy {q['loo_fixed_dy']:.2f} px")
        cx, cy = q["coef_dx"], q["coef_dy"]
        print(f"[12] dx = {cx[0]:+.3f} {cx[1]:+.6f}·u {cx[2]:+.6f}·v {cx[3]:+.3f}/Z,  dy = {cy[0]:+.3f} {cy[1]:+.6f}·u {cy[2]:+.6f}·v {cy[3]:+.3f}/Z")
        print(f"[12] c–d 상관 {q['corr_c_d']:+.2f} (d를 물리적 시차로 해석하지 않음), 직접 시차 {[round(k, 1) for k in q['direct_parallax']]} px·m")
        for name in ("round1", "round2"):
            c = r[name]
            print(f"[12] 캘리브레이션 {name}: base {c['holdout_m'] * 1000:.1f} mm, 변 오차 {np.round(c['sides_err_m'] * 1000, 1).tolist()} mm, "
                  f"5점 RMS {c['rms_m'] * 1000:.1f} mm")
        a, b = r["round1"]["ratio570"], r["round1"]["ratio547"]
        print(f"[12] fx 판정 (1회차): fx 570 가로 {a[[0, 2]].mean() * 100:+.2f} % / 깊이 {a[[1, 3]].mean() * 100:+.2f} %, "
              f"fx 547 가로 {b[[0, 2]].mean() * 100:+.2f} % / 깊이 {b[[1, 3]].mean() * 100:+.2f} %")
        c2 = r["round2"]["ratio570"]
        print(f"[12] fx 판정 (2회차, fx 570): 가로 {c2[[0, 2]].mean() * 100:+.2f} % / 깊이 {c2[[1, 3]].mean() * 100:+.2f} %")
        print("[12] --check 오차 mm (주변 평면 / 책상면 교점): " + ", ".join(
            f"{k} {v[0]:.1f}/{v[1]:.1f}" for k, v in r["check_err_mm"].items()) + f"; 1·2회차 변환 차 최대 {r['check_round12_max_mm']:.1f} mm")
        for k, v in r["objects_mm"].items():
            print(f"[12] 물체 {'/'.join(k)}: 차이 x {v[0]:+.1f} y {v[1]:+.1f} z {v[2]:+.1f} mm")

    if s in (None, 13):
        r = sec13_registration_two_poses()
        for key, label in (("AB", "자세 A 20쌍 + B"), ("A1B", "자세 A 1회차 12쌍 + B")):
            q = r[key]
            print(f"[13] {label}: 맞춤 {q['n']}, 합친 자료 c–d 계수 상관 {q['corr_c_d']:+.2f}, dy auto 규칙 → {q['dy_auto']}")
            for m in (("dx", "dist4"), ("dy", "fixed"), ("dy", "dist4")):
                c = q["coef"][m]
                print(f"[13]   {m[0]} {m[1]:<6} = {c[0]:+.2f} {c[1]:+.5f}·u {c[2]:+.5f}·v {c[3]:+.2f}/Z | 자세별 LOO 쌍 / 꼭짓점: "
                      + ", ".join(f"{n} {q['pair'][m][n]:.2f} / {q['corner'][m][n]:.2f}" for n in q["pair"][m]) + " px")
                print("[13]     자세 하나 빼고 예측 RMS / 최대: " + ", ".join(
                    f"→{k[2]} {v[0]:.2f} / {v[1]:.1f}{' (외삽)' if v[2] else ''}" for k, v in q["lopo"].items() if k[:2] == m) + " px")


if __name__ == "__main__":
    main()
