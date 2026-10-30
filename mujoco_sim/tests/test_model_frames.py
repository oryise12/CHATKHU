"""Coordinate frames, TCP definition and model facts the rest of the code relies on."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from pinch_sim.frames import angle_deg, look_at_quat, polar_to_floor
from pinch_sim.model import JOINTS, ROOT, file_sha


def _rot(axis, t):
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def _tcp_local():
    from pinch_sim import gripper
    from pinch_sim.model import load_config
    cfg = load_config()
    if cfg.get("gripper", {}).get("enabled"):
        return gripper.tcp_local(cfg["gripper"])
    return np.array(cfg["model"]["tcp_local_pos_m"]), np.array(cfg["model"]["tcp_local_zaxis"])


def independent_fk(q, tcp_local=None, tool_local=None):
    """FK straight from the ORIGINAL 6dof_control.xml body pos / joint axis (no MuJoCo).

    The TCP offset in body housing_1 comes from configs/demo.json (gripper tip, or the old flange face); the base
    stands mount.height_m above the table (z = 0) when the robot is on its plate.
    """
    if tcp_local is None:
        tcp_local, tool_local = _tcp_local()
    from pinch_sim.model import load_config
    cfg = load_config()
    src = (ROOT / cfg["model"]["source_scene"]).parent / "6dof_control.xml"
    root = ET.parse(src).getroot()
    body = root.find("worldbody")
    mount = cfg.get("mount", {})
    R, p = np.eye(3), np.array([0.0, 0.0, mount["height_m"] if mount.get("enabled") else 0.0])
    for i in range(6):
        body = body.find("body")
        j = body.find("joint")
        assert j.get("name") == JOINTS[i]
        p = p + R @ np.array(body.get("pos").split(), float)
        R = R @ _rot(np.array(j.get("axis").split(), float), q[i])
    return p + R @ np.array(tcp_local), R @ np.array(tool_local, float)


def test_original_model_copy_unchanged():
    """The model files used by the simulator must equal branch urdf@b2b4038 plus the 2026-10-24 change (every joint
    origin 2 mm along its axis toward the child: urdf/6dof.urdf, urdf/6dof.xacro, mujoco/6dof_control.xml) and the
    2026-10-27 change (links 2-3 and 3-4 with the engraved covers: meshes/*_engraved.stl, *_engraving.stl drawn black;
    the old link meshes stay unused) and the 2026-10-28 change (orange rings ID 70 / OD 80 / 6 mm at link_1's joint 2
    end and link__1__1's joint 3 and joint 4 ends: the seat is cut from *_engraved.stl, meshes/*_ring.stl visual +
    collision; gripper fingers drawn black: meshes/gripper_body.stl + gripper_fingers.stl = gripper.stl split).
    An unintended edit of these files fails here."""
    import json
    ref = json.loads((ROOT / "tests/original_model_sha256.json").read_text())
    for name, sha in ref.items():
        assert file_sha(ROOT.parent / "6dof_description" / name) == sha, name


def test_joint_order_actuators_and_integrator(scene):
    m, idx, _ = scene
    assert [m.joint(i).name for i in range(m.njnt)] == list(JOINTS)
    assert list(idx.qpos) == list(range(6))
    assert all(m.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_FIXED and m.actuator_biastype[a] == 0
               for a in idx.act), "motors must be plain torque actuators"
    assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST and m.opt.timestep == 0.001


@pytest.mark.parametrize("seed", range(5))
def test_mujoco_fk_matches_independent_fk(kin, seed):
    rng = np.random.default_rng(seed)
    for q in rng.uniform(-np.pi, np.pi, (20, 6)):
        pose = kin.fk(q)
        p, axis = independent_fk(q)
        assert np.linalg.norm(pose.p_m - p) < 1e-9
        assert np.linalg.norm(pose.tool_axis - axis) < 1e-9          # (arccos resolution is ~1e-6 deg)


def test_jacobian_matches_finite_difference(kin, q_ready):
    jacp, jacr = kin.jacobian(q_ready)
    h = 1e-6
    for j in range(6):
        dq = np.zeros(6)
        dq[j] = h
        dp = (kin.fk(q_ready + dq).p_m - kin.fk(q_ready - dq).p_m) / (2 * h)
        assert np.allclose(jacp[:, j], dp, atol=1e-7)


def test_ready_pose_is_tool_down_at_configured_point(kin, q_ready, cfg, col):
    pose = kin.fk(q_ready)
    assert np.linalg.norm(pose.p_m - cfg["ready"]["tcp_m"]) < 1e-4
    assert angle_deg(pose.tool_axis, [0, 0, -1]) < 0.05
    res = col.check(q_ready)
    assert not res.in_contact and res.floor_clearance_m > 0.01


def test_floor_target_polar_convention():
    assert np.allclose(polar_to_floor(0.25, 0.0), [0, 0.25, 0])           # front = +Y
    assert np.allclose(polar_to_floor(0.2, np.radians(30)), [0.1, 0.2 * np.cos(np.radians(30)), 0])  # +theta -> +X


def test_side_cam_looks_at_workspace(scene, data, cfg):
    m, _, _ = scene
    c = m.camera("side_cam").id
    fwd = -data.cam_xmat[c].reshape(3, 3)[:, 2]
    want = np.array(cfg["cameras"]["side_cam"]["lookat_m"]) - np.array(cfg["cameras"]["side_cam"]["pos_m"])
    assert angle_deg(fwd, want) < 1e-4
    assert np.allclose(data.cam_xpos[c], cfg["cameras"]["side_cam"]["pos_m"])
    q = look_at_quat([0, 0, 1], [0, 0, 0], up=(0, 1, 0))
    assert np.isfinite(q).all()


def test_robot_stands_on_the_plate(scene, cfg):
    """mount: base bottom at z = height (table z = 0), plate 180 x 180 x 20 mm grey around the J1 axis; the plate
    collides with the arm (and is a floor-clearance surface), not with the base."""
    from pinch_sim.collision import CollisionChecker
    m, idx, _ = scene
    mc = cfg["mount"]
    assert mc["enabled"] and mc["height_m"] == 0.02 and mc["plate_size_m"] == [0.18, 0.18]
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    g = idx.base_geom
    v = m.mesh_vert[m.mesh_vertadr[m.geom_dataid[g]]:][:m.mesh_vertnum[m.geom_dataid[g]]]
    z = (d.geom_xpos[g] + v @ d.geom_xmat[g].reshape(3, 3).T)[:, 2]
    assert abs(z.min() - 0.02) < 1e-3                                       # base bottom on the plate top
    plate = m.geom("base_plate")
    j1 = d.xanchor[m.joint("Revolute_1").id]
    assert np.allclose(plate.size, [0.09, 0.09, 0.01]) and np.allclose(d.geom_xpos[plate.id], [j1[0], j1[1], 0.01])
    assert np.allclose(plate.rgba, mc["plate_rgba"]) and m.body_weldid[plate.bodyid[0]] == 0
    assert plate.contype and plate.conaffinity and plate.id in idx.support_geoms
    assert m.geom("desk_top").id in idx.support_geoms
    assert np.isclose(m.geom("desk_top").size[2] + d.geom_xpos[m.geom("desk_top").id][2], 0.0)   # table top z = 0
    # straight up: no contact; the lowest moving part (r_a_1, on the base) is nearer the plate than the desk,
    # and the floor clearance is that plate distance (20 mm less than to the desk)
    r = CollisionChecker(m, idx).check(np.zeros(6))
    d.qpos[idx.qpos] = 0.0
    mujoco.mj_fwdPosition(m, d)
    g = [x for x in idx.robot_geoms if m.body(m.geom_bodyid[x]).name == "r_a_1"][0]
    to_plate = mujoco.mj_geomDistance(m, d, int(g), plate.id, 0.2, None)
    to_desk = mujoco.mj_geomDistance(m, d, int(g), m.geom("desk_top").id, 0.2, None)
    assert not r.contacts and r.floor_geom_body == "r_a_1"
    assert np.isclose(r.floor_clearance_m, to_plate) and np.isclose(to_desk - to_plate, 0.02, atol=1e-6)
