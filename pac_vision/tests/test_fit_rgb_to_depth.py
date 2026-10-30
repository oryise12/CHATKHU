"""tools/fit_rgb_to_depth.py 회귀: 2026-10-14 쌍 CSV로 보고한 숫자가 그대로 나오는지 (원자료로 재계산)."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import fit_rgb_to_depth as fr  # noqa: E402

PAIRS = ROOT / "data" / "measurements" / "2026-10-14_pose_pairs.csv"


def test_reported_leave_one_out_raw():
    p = fr.load_pairs(PAIRS)
    desk = p[:, 4] <= 1.5
    rep = fr.fit_report(p, desk)                             # Z 제한만 (제외 전)
    assert round(rep[("dx", "dist4")]["loo"], 2) == 4.96
    assert round(rep[("dy", "dist4")]["loo"], 2) == 1.31
    assert round(rep[("dx", "fixed")]["loo"], 2) == 8.98
    mask, reps, excluded = fr.select_for_fit(p)
    assert sorted(excluded) == [2, 3]                        # 화면 왼쪽 끝 꼭짓점 (dx 44 / 39)
    rep2 = fr.fit_report(p, mask)
    assert round(rep2[("dx", "dist4")]["loo"], 2) == 2.82


def test_rule_choices_raw():
    p = fr.load_pairs(PAIRS)
    mask, _, _ = fr.select_for_fit(p)
    cx, cy, use_dist_dy, _ = fr.choose(p, mask)
    assert not use_dist_dy                                   # dy 거리 식 이득 < 0.3 px → 고정값
    assert cy[1:] == (0.0, 0.0, 0.0)
    assert round(cy[0], 2) == 10.0
    assert cx[3] > 0                                         # 가까울수록 dx가 큼
    dp = fr.direct_parallax(p[p[:, 4] <= 1.5])
    assert len(dp) == 1 and round(dp[0][2], 1) == 26.6       # 같은 화면 높이 6a·6b
    corr = fr.coef_correlation(p[mask])
    assert np.isclose(corr[2, 3], -0.86, atol=0.005)          # c–d 상관: d를 물리적 시차로 해석하지 않는 이유


def test_adopted_coefficients_two_runs_raw():
    """config/pose.example.yaml의 계수: 같은 자세의 두 측정(14 + 10쌍)으로 맞춘 결과."""
    import yaml
    p, src = fr.load_many([PAIRS, PAIRS.with_name("2026-10-14_pose_pairs_run2.csv")])
    mask, _, excluded = fr.select_many(p, src)
    assert int(mask.sum()) == 20 and sorted(excluded) == [2, 3]
    cx, cy, use_dist_dy, rep = fr.choose(p, mask)
    assert use_dist_dy
    assert round(rep[("dx", "dist4")]["loo"], 2) == 2.65 and round(rep[("dy", "dist4")]["loo"], 2) == 1.59
    cam = yaml.safe_load((ROOT / "config" / "pose.example.yaml").read_text(encoding="utf-8"))["camera"]["rgb_to_depth"]
    for key, c in (("dx", cx), ("dy", cy)):
        cfg = cam[key]
        assert np.allclose([cfg["a"], cfg["b"], cfg["c"], cfg["d"]], c, rtol=0, atol=[5e-4, 5e-7, 5e-7, 5e-4])


POSE_A = [ROOT / "data" / "measurements" / f for f in ("2026-10-02_poseA_pairs_run1.csv", "2026-10-02_poseA_pairs_run2_transcribed.csv")]
POSE_B = [PAIRS, PAIRS.with_name("2026-10-14_pose_pairs_run2.csv")]


def test_two_poses_pooled_fit_raw():
    """두 자세 합친 거리 식: 자세별 leave-one-out이 한 자세 모드 --other의 숫자와 같고, 정리 표의 제외 사유가 규칙대로인지."""
    d = fr.load_poses([("A", POSE_A), ("B", POSE_B)])
    assert int(d["desk"].sum()) == 40 and int(d["full"].sum()) == 42
    excl = [(d["names"][d["pose"][i]], d["files"][d["src"][i]].name, int(d["num"][i]), d["reason"][i])
            for i in np.flatnonzero(~d["desk"])]
    assert [e[:3] for e in excl] == [("B", PAIRS.name, 3), ("B", PAIRS.name, 4), ("B", PAIRS.name, 12), ("B", PAIRS.name, 13)]
    assert excl[0][3].startswith("반복 차이") and excl[2][3].startswith("배경")
    rep = fr.analyze_poses(d, "desk")
    pair = {k: {n: round(s["rms"], 2) for n, s in v["pair"].items()} for k, v in rep["models"].items()}
    assert pair[("dx", "dist4")] == {"A": 2.56, "B": 2.72}
    assert pair[("dy", "dist4")] == {"A": 3.18, "B": 2.17}
    for k, v in rep["models"].items():                      # 반복 쌍을 함께 빼도 0.3 px 안 (이 자료)
        for n in ("A", "B"):
            assert abs(v["corner"][n]["rms"] - v["pair"][n]["rms"]) <= fr.LOO_DIFF_WARN_PX
    lopo = rep["lopo"][("dx", "dist4", "A")]
    assert lopo["src"] == "B" and not lopo["extrapolate"] and round(lopo["rms"], 2) == 3.29
    assert rep["lopo"][("dx", "dist4", "B")]["extrapolate"]          # 자세 A의 Z 범위(0.86-1.11 m) 밖
    assert fr.choose_dy(rep) == "dist4"


def test_fragment_is_pose_yaml_shape(tmp_path):
    import yaml
    from pac_vision.config import POSE_KEYS, check_overlay
    d = fr.load_poses([("A", POSE_A), ("B", POSE_B)])
    rep = fr.analyze_poses(d, "desk")
    doc = yaml.safe_load(fr.make_fragment(d, rep, "fixed", "test"))
    check_overlay(doc, POSE_KEYS, "fragment")
    m = doc["camera"]["rgb_to_depth"]
    assert m["dy"]["b"] == m["dy"]["c"] == m["dy"]["d"] == 0.0
    assert np.isclose(m["dx"]["d"], rep["models"][("dx", "dist4")]["coef"][3], atol=1e-6)
