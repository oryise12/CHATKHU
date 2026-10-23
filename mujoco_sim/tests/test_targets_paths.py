"""Seeded targets stay inside the requested distribution; paths start at the live FK."""
import numpy as np
import pytest

from pinch_sim.frames import angle_deg
from pinch_sim.model import with_overrides
from pinch_sim.paths import PathGenerationError, from_points, initial_path, return_path
from pinch_sim.targets import draw_polar, make_target, sample_target
from pinch_sim.model import load_config as _load_config

_T = _load_config()["target"]
RM = round((_T["r_min_m"] + _T["r_max_m"]) / 2, 3)      # a radius inside the configured target band


def test_draws_stay_in_range_and_marker_is_on_floor(cfg):
    rng = np.random.default_rng(123)
    t = cfg["target"]
    for _ in range(5000):
        r, th = draw_polar(rng, t)
        assert t["r_min_m"] <= r <= t["r_max_m"]
        assert abs(np.degrees(th)) <= t["theta_max_deg"]
    tgt = make_target(RM, np.radians(-12), t)
    assert tgt.floor_point_m[2] == 0.0
    assert np.allclose(tgt.tcp_goal_m - tgt.floor_point_m, [0, 0, t["work_height_m"]])
    assert np.allclose(tgt.approach_m - tgt.floor_point_m, [0, 0, t["approach_height_m"]])
    assert tgt.tcp_goal_m[2] > 0


def test_fixed_target_outside_range_is_rejected(cfg):
    with pytest.raises(ValueError):
        make_target(cfg["target"]["r_max_m"] + 0.01, 0.0, cfg["target"])
    with pytest.raises(ValueError):
        make_target(RM, np.radians(31), cfg["target"])


def test_seed_sequence_is_reproducible_and_independent_of_order(cfg, kin, col, q_ready):
    a = [sample_target(7, k, cfg, kin, col, q_ready).target for k in range(3)]
    b = [sample_target(7, k, cfg, kin, col, q_ready).target for k in (2, 1, 0)][::-1]
    for x, y in zip(a, b):
        assert np.array_equal(x.floor_point_m, y.floor_point_m)
    assert not np.array_equal(a[0].floor_point_m, sample_target(8, 0, cfg, kin, col, q_ready).target.floor_point_m)


def test_redraw_limit_reports_reasons_without_widening(cfg, kin, col, q_ready):
    # an impossible work height (TCP 0.6 m above the marker) makes every draw fail precheck
    bad = with_overrides(cfg, target={"work_height_m": 0.6, "approach_height_m": 0.7, "max_draws": 5})
    res = sample_target(1, 0, bad, kin, col, q_ready)
    assert res.target is None and len(res.attempts) == 5
    assert "5회 시도" in res.message
    for _draw, r, th_deg, reason in res.attempts:
        t = cfg["target"]
        assert t["r_min_m"] <= r <= t["r_max_m"] and abs(th_deg) <= t["theta_max_deg"] and reason != "ok"


def test_initial_path_starts_at_current_fk_and_is_one_way(cfg, kin, q_ready):
    t = make_target(RM, 0.0, cfg["target"])
    start = kin.fk(q_ready)
    p = initial_path(start, q_ready, t, cfg)
    assert np.array_equal(p.points_m[0], start.p_m)
    assert np.allclose(p.points_m[-1], t.tcp_goal_m)
    assert np.allclose(p.points_m[p.waypoint_index[1]], t.approach_m)       # stop at the approach point
    assert np.max(np.linalg.norm(np.diff(p.points_m, axis=0), axis=1)) <= cfg["path"]["ds_max_m"] + 1e-12
    assert angle_deg(p.tool_axis[-1], [0, 0, -1]) < 1e-9


def test_generator_refuses_below_floor_unless_test_flag(cfg, kin, q_ready):
    low = with_overrides(cfg, target={"work_height_m": -0.02})
    t = make_target(RM, 0.0, low["target"])
    with pytest.raises(PathGenerationError):
        initial_path(kin.fk(q_ready), q_ready, t, low)
    p = initial_path(kin.fk(q_ready), q_ready, t, low, allow_unsafe=True)
    assert p.points_m[:, 2].min() < 0 and p.meta["allow_unsafe"]


def test_return_path_lifts_first_from_low_start(cfg, kin, q_ready):
    ready = kin.fk(q_ready)
    t = make_target(RM, 0.0, cfg["target"])
    q_low = kin.solve_ik(t.tcp_goal_m, [0, 0, -1], q_ready).q
    p = return_path(kin.fk(q_low), q_low, ready, cfg)
    lift = p.points_m[p.waypoint_index[1]]
    assert np.allclose(lift[:2], p.points_m[0][:2]) and np.isclose(lift[2], cfg["target"]["approach_height_m"])
    assert np.allclose(p.points_m[-1], ready.p_m)


def test_edited_path_adapter_rejects_other_frames(q_ready):
    with pytest.raises(PathGenerationError):
        from_points(np.zeros((3, 3)), "virtual_edit_frame", q_ready)
    p = from_points(np.array([[0, 0.2, 0.2], [0, 0.25, 0.1]]), "world", q_ready)
    assert p.source == "edited" and p.waypoint_index == (0, 1)


def test_fingerprint_changes_with_geometry_or_start(cfg, kin, q_ready):
    t = make_target(RM, 0.0, cfg["target"])
    p1 = initial_path(kin.fk(q_ready), q_ready, t, cfg)
    p2 = initial_path(kin.fk(q_ready), q_ready, make_target(RM + 0.01, 0.0, cfg["target"]), cfg)
    q2 = q_ready + 1e-3
    p3 = initial_path(kin.fk(q2), q2, t, cfg)
    assert len({p1.fingerprint, p2.fingerprint, p3.fingerprint}) == 3


def test_side_cam_button_bar_hit_test():
    from pinch_sim.app import BAR_H, BUTTONS, button_at
    w, h = 544, 467
    keys = [button_at(int((i + 0.5) * w / len(BUTTONS)), h - BAR_H // 2, w, h) for i in range(len(BUTTONS))]
    assert keys == [k for k, _ in BUTTONS]
    assert button_at(w // 2, h // 2, w, h) is None          # clicks on the image are not commands


def test_new_presses_alternate_theta_bands_and_start_target_is_not_counted(scene, cfg, data):
    from pinch_sim.session import Session
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, seed=7, threaded=False)
    s.cmd_new_target(data, count_as_new=False)                 # automatic start target
    assert s.new_presses == 0 and s.target_label == "start target"
    bands = [tuple(b) for b in cfg["target"]["new_theta_bands_deg"]]
    for k in range(1, 6):                                       # +, -, +, -, +
        s.handle_key("N", data)
        lo, hi = bands[(k - 1) % len(bands)]
        th = np.degrees(s.target.theta_rad)
        assert s.new_presses == k and s.target_label == f"New #{k}"
        assert lo <= th <= hi and (th > 0) == (k % 2 == 1)
        assert cfg["target"]["r_min_m"] <= s.target.r_m <= cfg["target"]["r_max_m"]
        assert s.state == "GENERATED" and s.path is not None


def test_new_band_redraws_stay_in_band(cfg, kin, col, q_ready):
    bad = with_overrides(cfg, target={"work_height_m": 0.6, "approach_height_m": 0.7, "max_draws": 6})
    res = sample_target(3, 1, bad, kin, col, q_ready, theta_band_deg=(-35.0, -25.0))
    assert res.target is None and len(res.attempts) == 6
    assert all(-35.0 <= th <= -25.0 for _d, _r, th, _reason in res.attempts)
    assert "[-35, -25]" in res.message


def test_new_band_sequence_is_seed_reproducible(scene, cfg, data):
    from pinch_sim.session import Session
    m, idx, sha = scene
    seqs = []
    for _ in range(2):
        d = __import__("mujoco").MjData(m)
        d.qpos[:] = data.qpos
        __import__("mujoco").mj_forward(m, d)
        s = Session(m, idx, sha, cfg, d, seed=11, threaded=False)
        s.cmd_new_target(d, count_as_new=False)
        seq = []
        for _k in range(3):
            s.cmd_new_target(d)
            seq.append((round(s.target.r_m, 9), round(s.target.theta_rad, 9)))
        seqs.append(seq)
    assert seqs[0] == seqs[1]
