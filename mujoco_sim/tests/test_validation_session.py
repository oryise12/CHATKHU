"""Execution gate: only a complete all-PASS report on the unchanged path/state may start motion."""
import copy

import numpy as np
import pytest

from pinch_sim.model import load_config as _load_config, with_overrides
from pinch_sim.paths import initial_path
from pinch_sim.session import Session
from pinch_sim.targets import make_target
from pinch_sim.validation import REQUIRED, Check, Goal, PlanningContext, validate

_T = _load_config()["target"]
RM = round((_T["r_min_m"] + _T["r_max_m"]) / 2, 3)      # a radius inside the configured target band


@pytest.fixture(scope="module")
def good(ctx, cfg, kin, q_ready, scene):
    import mujoco
    from pinch_sim.model import ready_q
    m, _, _ = scene
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    t = make_target(_T["r_max_m"], np.radians(30), cfg["target"])
    p = initial_path(kin.fk(q_ready), q_ready, t, cfg)
    return validate(ctx, p, d.qpos.copy(), d.qvel.copy(), d.time, Goal(t.tcp_goal_m, t.tool_axis)), p


def test_good_path_passes_all_required(good):
    rep, _ = good
    assert rep.execution_allowed, rep.blocking()
    assert sorted(c.kind for c in rep.checks) == sorted(REQUIRED)


def test_validated_quantities_come_from_the_executed_spline(good, cfg):
    rep, _ = good
    traj = rep.trajectory
    t, q, qd, qdd = traj.dense(cfg["timing"]["dt_check_s"])
    va = {c.kind: c for c in rep.checks}["joint_vel_acc"].details
    assert np.isclose(np.abs(qd).max(), va["qd_max_rad_s"]) and np.isclose(np.abs(qdd).max(), va["qdd_max_rad_s2"])
    q0, qd0, _ = traj.sample(0.0)
    assert np.allclose(qd0, 0, atol=1e-9)            # starts and ends at rest (stop waypoints)
    assert np.allclose(traj.sample(traj.duration + 1)[1], 0)


@pytest.mark.parametrize("mutation", ["fail", "not_evaluated", "missing", "duplicate", "bogus_status"])
def test_any_incomplete_report_blocks(good, mutation):
    rep, _ = good
    r = copy.copy(rep)
    r.checks = [copy.copy(c) for c in rep.checks]
    if mutation == "fail":
        r.checks[5].status = "FAIL"
    elif mutation == "not_evaluated":
        r.checks[-1].status = "NOT_EVALUATED"
    elif mutation == "missing":
        r.checks = [c for c in r.checks if c.kind != "collision"]
    elif mutation == "duplicate":
        r.checks.append(Check("collision", "PASS"))
    else:
        r.checks[0].status = "OK"
    assert not r.execution_allowed


def test_below_floor_target_fails_and_names_reasons(ctx, cfg, kin, q_ready, data):
    low = with_overrides(cfg, target={"work_height_m": -0.02})
    t = make_target(RM, 0.0, low["target"])
    p = initial_path(kin.fk(q_ready), q_ready, t, low, allow_unsafe=True)
    rep = validate(PlanningContext(ctx.m, ctx.idx, ctx.model_sha, low), p, data.qpos.copy(), data.qvel.copy(),
                   data.time, Goal(t.tcp_goal_m, t.tool_axis))
    status = {c.kind: c.status for c in rep.checks}
    assert not rep.execution_allowed
    assert status["workspace"] == "FAIL" and status["collision"] == "FAIL"
    assert status["sim_rollout"] == "NOT_EVALUATED"
    fail = next(c for c in rep.checks if c.kind == "collision")
    assert fail.details["where_m"] is not None


def test_velocity_limit_is_enforced_on_final_trajectory(ctx, cfg, kin, q_ready, data):
    slow = with_overrides(cfg, limits={"joint_speed_max_deg_s": float(np.degrees(0.2))}, timing={"max_retime_iter": 3})
    t = make_target(RM, 0.0, slow["target"])
    rep = validate(PlanningContext(ctx.m, ctx.idx, ctx.model_sha, slow), initial_path(kin.fk(q_ready), q_ready, t,
                   slow), data.qpos.copy(), data.qvel.copy(), data.time, Goal(t.tcp_goal_m, t.tool_axis))
    c = {x.kind: x for x in rep.checks}
    assert c["timing"].details["time_scale"] > 1.0          # stretched...
    va = c["joint_vel_acc"]
    assert (va.status == "PASS") == (va.details["qd_max_rad_s"] <= 0.2 * (1 + 1e-6))   # judged on the stretched spline
    ratio = slow["limits"]["joint_speed_plan_ratio"]
    assert va.status == "PASS" and np.isclose(va.details["qd_max_rad_s"], 0.2 * ratio, rtol=1e-4)  # at cap x plan ratio


def test_start_mismatch_fails(ctx, cfg, kin, q_ready, data):
    t = make_target(RM, 0.0, cfg["target"])
    q_other = q_ready + 0.05
    p = initial_path(kin.fk(q_other), q_other, t, cfg)     # generated from a different robot state
    rep = validate(ctx, p, data.qpos.copy(), data.qvel.copy(), data.time, Goal(t.tcp_goal_m, t.tool_axis))
    assert {c.kind: c.status for c in rep.checks}["start_matches_robot"] == "FAIL" and not rep.execution_allowed


# ------------------------------------------------------------------------------------------- session
def make_session(scene, cfg, data, **kw):
    m, idx, sha = scene
    return Session(m, idx, sha, cfg, data, threaded=False, **kw)


def test_session_requires_select_validate_approve(scene, cfg, data):
    s = make_session(scene, cfg, data)
    assert not s.cmd_approve(data)                      # nothing validated
    s.cmd_fixed_target(RM, 0.0, data)
    assert s.state == "GENERATED"
    assert not s.cmd_validate(data) and not s.cmd_approve(data)   # must select first
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "PASSED"
    q_before = data.qpos.copy()
    for _ in range(200):
        s.physics_step(data)
    assert np.max(np.abs(data.qpos - q_before)) < 1e-6   # PASSED alone never moves the robot
    assert s.cmd_approve(data) and s.state == "EXECUTING"
    while s.state == "EXECUTING":
        s.physics_step(data)
    assert s.state == "DONE" and s.report is None        # a report is never reused
    assert not s.cmd_approve(data)


def test_new_target_invalidates_report(scene, cfg, data):
    s = make_session(scene, cfg, data, seed=3)
    s.cmd_fixed_target(RM, 0.0, data)
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "PASSED"
    s.cmd_new_target(data)
    assert s.state == "GENERATED" and s.report is None and not s.cmd_approve(data)


def test_drift_and_config_change_block_approval(scene, cfg, data):
    s = make_session(scene, cfg, data)
    s.cmd_fixed_target(RM, 0.0, data)
    s.cmd_select()
    s.cmd_validate(data)
    s.cfg = with_overrides(cfg, limits={"joint_speed_max_deg_s": 20.0})   # config edited after validation
    assert "config changed" in s.binding_problems(data)
    s.cfg = cfg
    data.qpos[s.idx.qpos[1]] += 0.01                                 # robot pushed 0.01 rad
    s.watch(data)
    assert s.state == "GENERATED" and s.report is None
    assert np.allclose(s.path.start_q_rad, data.qpos[s.idx.qpos])    # regenerated from the actual state


def test_failed_validation_returns_to_selection_and_blocks(scene, cfg, data):
    low = with_overrides(cfg, target={"work_height_m": -0.02})
    s = make_session(scene, low, data, allow_unsafe=True)
    s.cmd_fixed_target(RM, 0.0, data)
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "FAILED" and not s.cmd_approve(data) and s.state == "FAILED"
    assert s.cmd_validate(data)                      # can request validation again from FAILED


def test_stale_threaded_result_is_discarded(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=True)
    s.cmd_fixed_target(RM, 0.0, data)
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "VALIDATING"          # stays so until poll() sees the finished future
    fut = s._future
    s.cmd_cancel(data)                      # user cancels while the worker is still running
    fut.result()
    s.poll()
    assert s.state == "SELECTED" and s.report is None and not s.cmd_approve(data)
    s.close()
