"""HOME plan from the integer PV/PA/Goal actually sent: caps, common time, quantization warnings, refusals."""
import math

import numpy as np
import pytest

from helpers import System, fake_cfg
from manipulator_control.fake_bus import FakeServo
from manipulator_control.planner import Trapezoid, braking_target, plan_home, servo_profile
from manipulator_control.units import pa_counts_s2, pv_counts_s


def _plan(q_goal_deg, cfg=None, p_ref=None, checker=None, **motion):
    s = System(purpose="hold", cfg=cfg or fake_cfg(**motion), with_checker=False)
    s.align_all()
    assert s.until_state(s.sup.state.HOLDING, timeout_s=5)
    cal = s.sup.cal
    p0 = dict(cal.p_ref) if p_ref is None else p_ref
    return plan_home(s.cfg.joints, cal, p0, np.radians(q_goal_deg), s.cfg.motion, hold_pv=29, hold_pa=3,
                     path_checker=checker), s


def test_profile_model_matches_e_manual_formula():
    p = servo_profile(16384, 131, 3)
    assert p.kind == "trapezoid"
    assert p.t_acc * 1000 == pytest.approx(64 * 131 / 3, rel=1e-3)          # t1 = 64 * PV / PA [ms]
    assert p.T == pytest.approx(16384 / pv_counts_s(131) + p.t_acc)
    tri = servo_profile(500, 262, 1)
    assert tri.kind == "triangle" and tri.v_peak < pv_counts_s(262)
    assert float(tri.pos(tri.T)) == pytest.approx(500)
    with pytest.raises(ValueError):
        servo_profile(100, 0, 3)                                           # 0 would mean infinite


def test_fake_servo_follows_the_predicted_profile():
    s = FakeServo(1, 0, 0.0)
    s._set("torque_enable", 1)
    s.regs.update(profile_velocity=131, profile_acceleration=3, goal_position=16384)
    p = servo_profile(16384, 131, 3)
    for t in (1.0, 3.0, 6.0, 9.0, 11.0):
        s.advance(t)
        assert abs(s.prof_x - float(p.pos(t))) < 3          # internal trajectory (counts)


def test_caps_hold_after_integer_rounding_and_one_common_time():
    plan, _ = _plan([0, 36.743, 75.193, -51.55, -90, -8.008], home={"speed_deg_s": 11.25, "accel_deg_s2": 5.0})
    assert not plan.errors
    for a in plan.axes:
        assert 1 <= a.pv <= 262 and a.pa >= 1
        if a.profile.D:
            assert math.degrees(a.v_peak_rad_s) <= 11.25 + 1e-9
            assert math.degrees(a.a_rad_s2) <= 5.0 + 1e-9            # PA <= 3 (4.02 deg/s^2), never 4 (5.36)
            assert a.pa <= 3
    dom = max(plan.axes, key=lambda a: a.profile.D)
    assert dom.profile.T == pytest.approx(plan.T_common)
    assert plan.T_max >= plan.T_common                                  # recomputed from what is sent


def test_stretching_not_clipping_keeps_velocity_ratios():
    plan, _ = _plan([0, 10, 20, -40, -80, 0], home={"speed_deg_s": 22.5, "accel_deg_s2": 5.0})
    moving = [a for a in plan.axes if a.profile.D > 0]
    dom = max(moving, key=lambda a: a.profile.D)
    for a in moving:
        ratio = a.profile.D / dom.profile.D
        assert a.profile.v_peak / dom.profile.v_peak == pytest.approx(ratio, rel=0.08)   # integer PV only


def test_zero_move_never_sends_pv_zero_and_tiny_move_is_handled():
    plan, _ = _plan([0, 0, 0, 0, 0, 0.003])
    a0 = plan.axes[0]
    assert a0.profile.D == 0 and a0.pv >= 1 and a0.pa >= 1 and a0.goal == a0.p0
    tiny = plan.axes[5]
    assert tiny.profile.D >= 0 and tiny.pv >= 1 and tiny.pa >= 1
    assert not plan.errors


def test_quantization_desync_is_a_warning_not_an_error():
    # 1 deg on one joint while another moves 90 deg: PV/PA of the small joint are tiny integers
    plan, _ = _plan([0, 1.0, 0, 0, -90, 0], home={"speed_deg_s": 11.25, "accel_deg_s2": 5.0})
    small = plan.axes[1]
    assert small.pv >= 1 and small.pa >= 1
    early = plan.T_common - small.profile.T
    assert plan.arrival_spread_s == pytest.approx(max(a.profile.T for a in plan.axes if a.profile.D) -
                                                  min(a.profile.T for a in plan.axes if a.profile.D))
    assert not plan.errors
    # the review's "1 deg, 10 s -> ~4 s early" is a case, not a constant: computed here for these raw values
    assert isinstance(early, float)
    plan2, _ = _plan([0, 4.0, 0, 0, -90, 0], home={"speed_deg_s": 2.5, "accel_deg_s2": 5.0})
    assert plan2.sync_dev_rad and all(v >= 0 for v in plan2.sync_dev_rad.values())


def test_sync_warning_threshold_2_deg():
    plan, _ = _plan([0, 36.743, 75.193, -51.55, -90, -8.008], home={"speed_deg_s": 11.25, "accel_deg_s2": 5.0})
    dev = math.degrees(max(plan.sync_dev_rad.values()))
    assert (dev > 2.0) == bool(plan.warnings)
    assert not plan.errors


def test_start_just_outside_the_range_moves_back_inside():
    _, s = _plan([0, 0, 0, 0, -100, 0])                               # Revolute_5 range is [-120, 30] in the fake
    cal = s.sup.cal
    for start, ok in ((-120.3, True), (-122.0, False)):              # e.g. an IK pose at the limit, one count over
        p0 = cal.counts(np.radians([0, 0, 0, 0, start, 0]))
        plan = plan_home(s.cfg.joints, cal, p0, np.radians([0, 0, 0, 0, -100, 0]), s.cfg.motion, hold_pv=29, hold_pa=3)
        assert (not plan.errors) == ok, (start, plan.errors)


def test_path_outside_confirmed_range_is_refused():
    cfg = fake_cfg()
    plan, _ = _plan([0, 0, 0, 0, -130, 0], cfg=cfg)                    # Revolute_5 range is [-120, 30] in the fake
    assert any("가동 범위 밖" in e for e in plan.errors)


def test_model_path_check_refuses_self_contact():
    from helpers import viz
    from manipulator_control.model import PathChecker
    m, arms = viz()
    checker = PathChecker(m, arms["measured"])
    assert checker.check(np.zeros(6)) == []
    folded = np.radians([0, 0, 0, 0, 0, 0])
    folded[2] = math.radians(170)
    assert checker.check(folded), "folding the elbow onto the arm must collide in the model"
    cfg = fake_cfg()
    cfg.robot["joints"][2]["limit_deg"] = [-20, 175]
    from manipulator_control import config as cfgmod
    import yaml, tempfile, pathlib
    tmp = pathlib.Path(tempfile.mkdtemp()) / "fake.yaml"
    tmp.write_text(yaml.safe_dump(cfg.robot, allow_unicode=True))
    cfg2 = cfgmod.load(tmp)
    plan, _ = _plan([0, 0, 170, 0, 0, 0], cfg=cfg2, checker=checker.check)
    assert any("모델 경로 검사" in e for e in plan.errors)


def test_accel_cap_below_raw_minimum_is_refused():
    plan, _ = _plan([0, 10, 0, 0, 0, 0], home={"accel_deg_s2": 1.0})       # < 1.341 deg/s^2 = PA 1
    assert any("PA raw 1" in e for e in plan.errors)


def test_braking_target_never_reverses_or_passes_the_goal():
    # moving up at PV 131 (v_traj raw 131), PA 3, current goal far ahead
    st = braking_target(10000, 131, 30000, 131, 3, latency_s=0.1, pa_max=11, inside=lambda g: True)
    v, a = pv_counts_s(131), pa_counts_s2(3)
    assert st.goal >= 10000 + v * v / (2 * a)            # at least the stopping distance ahead: no reversal
    assert st.goal <= 30000
    near = braking_target(29990, 131, 30000, 131, 3, latency_s=0.1, pa_max=11, inside=lambda g: True)
    assert near.goal == 30000                            # the current goal is closer: stop there
    down = braking_target(10000, -60, -5000, 131, 3, latency_s=0.1, pa_max=11, inside=lambda g: True)
    assert down.goal < 10000
    tight = braking_target(10000, 131, 30000, 131, 3, latency_s=0.1, pa_max=11, inside=lambda g: g < 11000)
    assert tight.pa == 11 or "insufficient" in tight.note
    still = braking_target(12345, 0, 20000, 131, 3, latency_s=0.1, pa_max=11, inside=lambda g: True)
    assert still.goal == 12345 and still.pv >= 1 and still.pa >= 1


def test_trapezoid_velocity_continuous():
    p = Trapezoid(1000.0, 100.0, 50.0)
    t = np.linspace(0, p.T, 2001)
    v = p.vel(t)
    assert np.max(np.abs(np.diff(v))) < 50.0 * (t[1] - t[0]) * 1.01 + 1e-9   # |dv| <= a dt: no velocity jump
