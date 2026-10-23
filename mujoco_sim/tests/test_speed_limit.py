"""Joint speed cap (limits.joint_speed_max_deg_s = 22.5 deg/s): one common time scale for every joint."""
import numpy as np
import pytest

from pinch_sim.model import joint_speed_max_rad_s, joint_speed_plan_rad_s, with_overrides
from pinch_sim.trajectory import CartesianTiming, IKTrack, build_trajectory, common_speed_scale, min_jerk

D = np.radians


def test_cap_is_single_value_in_degrees(cfg):
    assert cfg["limits"]["joint_speed_max_deg_s"] == 22.5
    assert np.isclose(joint_speed_max_rad_s(cfg), np.pi / 8)
    assert joint_speed_plan_rad_s(cfg) <= joint_speed_max_rad_s(cfg)
    assert "qd_max_rad_s" not in cfg["limits"]                     # no second, different limit


@pytest.mark.parametrize("qd_deg, expect_deg", [
    ([45, 30, -15], [22.5, 15, -7.5]),          # example from the request: the fastest joint at the cap
    ([-45, -30, 15], [-22.5, -15, 7.5]),        # reverse direction: |qd| decides
    ([0, 0, 0], [0, 0, 0]),                     # zero velocity: unchanged, no division by zero
    ([20, -10, 5], [20, -10, 5]),               # already below the cap: unchanged
    ([22.5, 1, 0], [22.5, 1, 0]),               # exactly at the cap: unchanged
])
def test_common_scale_keeps_joint_ratios(qd_deg, expect_deg):
    k = common_speed_scale(D(qd_deg), D(22.5))
    assert np.allclose(np.degrees(k * D(np.array(qd_deg, float))), expect_deg)


def _synthetic(amp_rad, T=1.0, dt=0.01):
    """Joint motion q(t) = amp * min_jerk(t/T) (all joints start and stop together)."""
    t = np.arange(0.0, T + 1e-9, dt)
    q = np.outer(min_jerk(t / T), np.asarray(amp_rad, float))
    timing = CartesianTiming(t, np.zeros((len(t), 3)), np.tile([0, 0, -1.0], (len(t), 1)), [0.0, T])
    ik = IKTrack(q, np.ones(len(t), bool), np.zeros(len(t)), np.zeros(len(t)), np.zeros(len(t)), None)
    return timing, ik


def test_multi_joint_motion_is_stretched_by_one_factor(cfg):
    cfg = with_overrides(cfg, limits={"qdd_max_rad_s2": 1e6})     # isolate the speed cap
    amp = [1.0, -0.5, 0.25, 0.0, -0.1, 0.0]                        # several joints, both directions, one idle
    timing, ik = _synthetic(amp)
    traj, info = build_trajectory(timing, ik, cfg)
    _, q, qd, _ = traj.dense(cfg["timing"]["dt_check_s"])
    peak = np.abs(qd).max(0)
    assert np.isclose(peak.max(), joint_speed_plan_rad_s(cfg), rtol=1e-4)      # fastest joint at the target
    assert peak.max() <= joint_speed_max_rad_s(cfg)
    nz = np.abs(amp) > 0
    assert np.allclose(peak[nz] / peak[nz].max(), np.abs(amp)[nz] / np.max(np.abs(amp)), rtol=1e-3)  # ratios kept
    assert peak[3] == 0.0 and peak[5] == 0.0                      # idle joints stay idle
    assert np.allclose(q[-1], amp) and np.allclose(qd[-1], 0, atol=1e-9)       # all arrive together, at rest
    unscaled = 1.875 * 1.0                                         # min-jerk peak of the 1 rad joint in 1 s
    assert np.isclose(traj.time_scale, unscaled / joint_speed_plan_rad_s(cfg), rtol=1e-3)


def test_motion_within_the_cap_is_not_slowed(cfg):
    timing, ik = _synthetic([0.05, -0.02, 0, 0, 0, 0])             # peak 5.4 deg/s
    traj, info = build_trajectory(timing, ik, cfg)
    assert traj.time_scale == 1.0 and np.isclose(traj.duration, 1.0)
    assert len(info["retime"]) == 1


def test_validated_paths_respect_the_cap(cfg, ctx, kin, q_ready, data, scene):
    """Initial path and the home return path: planned and dry-rollout speeds stay under 22.5 deg/s."""
    from pinch_sim.paths import initial_path, return_path
    from pinch_sim.targets import make_target
    from pinch_sim.validation import Goal, validate
    from pinch_sim.model import ready_q
    cap = joint_speed_max_rad_s(cfg)
    t = make_target(0.275, np.radians(10), cfg["target"])
    rep = validate(ctx, initial_path(kin.fk(q_ready), q_ready, t, cfg), data.qpos.copy(), data.qvel.copy(),
                   data.time, Goal(t.tcp_goal_m, t.tool_axis))
    assert rep.execution_allowed, rep.blocking()
    c = {x.kind: x for x in rep.checks}
    assert c["joint_vel_acc"].details["qd_max_rad_s"] <= cap
    assert max(rep.extras["rollout"]["qd_peak_rad_s"]) <= cap
    # return path from the work point back to ready
    q_low = kin.solve_ik(t.tcp_goal_m, [0, 0, -1], q_ready).q
    m, idx, _ = scene
    data.qpos[idx.qpos] = q_low
    import mujoco
    mujoco.mj_forward(m, data)
    r = return_path(kin.fk(q_low), q_low, kin.fk(ready_q(m)[idx.qpos]), cfg)
    rep = validate(ctx, r, data.qpos.copy(), data.qvel.copy(), data.time, Goal(*[kin.fk(ready_q(m)[idx.qpos]).p_m,
                   kin.fk(ready_q(m)[idx.qpos]).tool_axis], "ready"))
    assert rep.execution_allowed, rep.blocking()
    assert {x.kind: x for x in rep.checks}["joint_vel_acc"].details["qd_max_rad_s"] <= cap
    assert max(rep.extras["rollout"]["qd_peak_rad_s"]) <= cap
