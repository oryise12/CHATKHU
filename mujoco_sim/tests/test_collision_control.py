"""Collision checking honesty and torque control behaviour."""
import mujoco
import numpy as np

from pinch_sim.control import ComputedTorque, TrajectoryExecutor, controlled_step
from pinch_sim.model import ROOT, load_config

_T = load_config()["target"]
RM = round((_T["r_min_m"] + _T["r_max_m"]) / 2, 3)      # a radius inside the configured target band


def test_only_the_j1_adjacent_pair_is_excluded(scene):
    m, idx, _ = scene
    pairs = {(m.body(int(s >> 16)).name, m.body(int(s & 0xFFFF)).name) for s in m.exclude_signature}
    assert pairs == {("world", "r_a_1")}


def test_original_model_has_permanent_base_contact_generated_does_not(scene, cfg):
    orig = mujoco.MjModel.from_xml_path(str(ROOT / cfg["model"]["source_scene"]))
    d = mujoco.MjData(orig)
    mujoco.mj_forward(orig, d)
    bodies = {tuple(sorted((orig.body(orig.geom_bodyid[c.geom1]).name, orig.body(orig.geom_bodyid[c.geom2]).name)))
              for c in d.contact[:d.ncon]}
    assert bodies == {("r_a_1", "world")}
    m, _, _ = scene
    d2 = mujoco.MjData(m)
    mujoco.mj_forward(m, d2)
    assert d2.ncon == 0


def test_real_collisions_are_still_detected(kin, col, q_ready):
    # TCP driven 2 cm into the support surface (desk top at z = 0 since 2026-10-12): the housing must touch it
    q = kin.solve_ik([0, 0.25, -0.02], [0, 0, -1], q_ready).q
    r = col.check(q)
    assert r.in_contact and any(set(c[:2]) == {"desk", "housing_1"} for c in r.contacts)
    assert r.floor_clearance_m < 0 and r.floor_geom_body == "housing_1"     # clearance is measured to the desk top
    # a folded elbow: link_1 against the base / shoulder (self or base contact)
    folded = q_ready.copy()
    folded[1], folded[2] = 2.6, 2.9
    assert col.check(folded).in_contact


def test_hold_keeps_pose_and_ctrl_is_torque(scene, cfg, data, q_ready):
    m, idx, _ = scene
    ctrl = ComputedTorque(m, idx, cfg["control"])
    z = np.zeros(6)
    for _ in range(3000):
        tau, sat = controlled_step(m, data, idx, ctrl, q_ready, z, z)
        assert not sat
    assert np.max(np.abs(data.qpos[idx.qpos] - q_ready)) < cfg["control"]["hold_drift_rad"]
    # at rest, ctrl equals the gravity torque (inverse dynamics), not a joint angle
    d = mujoco.MjData(m)
    d.qpos[idx.qpos] = q_ready
    mujoco.mj_inverse(m, d)
    assert np.allclose(data.ctrl[idx.act], d.qfrc_inverse[idx.dof], atol=0.02)
    assert not np.allclose(data.ctrl[idx.act], q_ready, atol=0.1)


def test_uncontrolled_robot_falls(scene, q_ready):
    m, idx, _ = scene
    d = mujoco.MjData(m)
    d.qpos[idx.qpos] = q_ready
    for _ in range(500):
        mujoco.mj_step(m, d)
    assert np.max(np.abs(d.qpos[idx.qpos] - q_ready)) > 0.05


def test_wrist_offset_settles_with_friction_compensation(scene, cfg, q_ready):
    m, idx, _ = scene
    z = np.zeros(6)
    for comp, should_settle in ((True, True), (False, False)):
        cc = dict(cfg["control"], friction_comp=comp)
        ctrl = ComputedTorque(m, idx, cc)
        d = mujoco.MjData(m)
        d.qpos[idx.qpos] = q_ready + 0.02
        mujoco.mj_forward(m, d)
        for _ in range(1000):
            controlled_step(m, d, idx, ctrl, q_ready, z, z)
        settled = np.max(np.abs(d.qpos[idx.qpos] - q_ready)) < 1e-3
        assert settled == should_settle


def test_executor_aborts_on_push(scene, cfg, ctx, data, kin, q_ready):
    from pinch_sim.paths import initial_path
    from pinch_sim.targets import make_target
    from pinch_sim.validation import Goal, validate
    m, idx, _ = scene
    t = make_target(RM, 0.0, cfg["target"])
    rep = validate(ctx, initial_path(kin.fk(q_ready), q_ready, t, cfg), data.qpos.copy(), data.qvel.copy(),
                   data.time, Goal(t.tcp_goal_m, t.tool_axis))
    assert rep.execution_allowed
    ex = TrajectoryExecutor(m, idx, rep.trajectory, cfg["control"], t0=data.time)
    body = m.site_bodyid[idx.tcp_site]
    while ex.status == "RUNNING":
        data.xfrc_applied[body, :3] = [20.0, 0, 0] if data.time > 1.0 else 0
        ex.step(data)
    assert ex.status == "ABORTED" and ("error" in ex.reason or "saturated" in ex.reason)
