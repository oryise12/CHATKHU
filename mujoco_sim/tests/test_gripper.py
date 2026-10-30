"""The gripper replaces housing_1.stl on the rotating link of Revolute_6; TCP = gripper tip."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from pinch_sim import gripper
from pinch_sim.frames import angle_deg
from pinch_sim.model import ACTUATORS, JOINTS, ROOT


@pytest.fixture(scope="module")
def gcfg(cfg):
    assert cfg["gripper"]["enabled"]
    return cfg["gripper"]


def test_old_end_part_removed_and_chain_kept(scene, gcfg):
    m, idx, _ = scene
    assert [m.joint(i).name for i in range(m.njnt)] == list(JOINTS) and m.nu == 6
    assert [m.actuator(a).name for a in range(m.nu)] == list(ACTUATORS)
    j6 = m.joint("Revolute_6")
    body = m.body(gcfg["mount_body"]).id
    assert m.jnt_bodyid[j6.id] == body                                    # gripper body = J6's moving child
    assert m.body(m.body_parentid[body]).name == "r_a__2__1"
    assert np.allclose(m.jnt_axis[j6.id], [-1, 0, 0])
    assert all(m.mesh(i).name != "housing_1" for i in range(m.nmesh))       # old mesh asset gone
    names = sorted(m.geom(g).name for g in range(m.ngeom) if m.geom_bodyid[g] == body)
    assert names == ["gripper_collision", "gripper_fingers_visual", "gripper_visual"]   # only the gripper on that body
    assert np.isclose(m.body_mass[body], gcfg["mass_kg"])
    assert m.site_bodyid[m.site("tcp").id] == body


def test_gripper_and_tcp_rotate_with_j6(scene, kin, q_ready):
    m, idx, _ = scene
    d = mujoco.MjData(m)
    g = m.geom("gripper_visual").id
    poses = []
    for dq in (0.0, 0.7):
        q = q_ready.copy()
        q[5] += dq
        d.qpos[idx.qpos] = q
        mujoco.mj_kinematics(m, d)
        poses.append((d.geom_xmat[g].reshape(3, 3).copy(), kin.fk(q)))
    (Rg0, p0), (Rg1, p1) = poses
    axis = p0.tool_axis
    assert angle_deg(p0.tool_axis, p1.tool_axis) < 1e-6                     # rotation is about the tool axis
    for R0, R1 in ((Rg0, Rg1), (p0.R, p1.R)):
        dR = R1 @ R0.T
        assert np.isclose(np.degrees(np.arccos((np.trace(dR) - 1) / 2)), np.degrees(0.7), atol=1e-6)
    # the tip is 1.8 mm off the J6 axis, so the TCP position moves on a circle around that axis
    r = np.linalg.norm(gripper.tcp_local(load_g())[0][1:])
    assert np.isclose(np.linalg.norm(p1.p_m - p0.p_m), 2 * r * np.sin(0.35), atol=1e-9)


def load_g():
    from pinch_sim.model import load_config
    return load_config()["gripper"]


def _stl_triangles(path) -> np.ndarray:
    """Binary STL vertices, (n, 3, 3) float32 in file units."""
    import struct
    b = path.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    return np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"]


def test_black_fingers_are_drawing_only(scene, gcfg):
    """gripper.stl = body + fingers (same triangles); the fingers are drawn black, the collision geom is the whole
    gripper.stl, hidden (group 3) so its grey does not cover the fingers."""
    m, _, _ = scene
    whole, body, fingers = (_stl_triangles(ROOT / gcfg[k]) for k in ("mesh", "body_mesh", "fingers_mesh"))
    key = lambda t: sorted(map(bytes, t))                                   # noqa: E731
    assert key(np.concatenate([body, fingers])) == key(whole) and len(fingers) == 3060
    tip = np.array(gcfg["tip_center_mesh_mm"])
    assert np.isclose(fingers.reshape(-1, 3)[:, 2].max(), tip[2], atol=1e-3)   # the fingers end at the tip face
    vis, fin, col = (m.geom(n) for n in ("gripper_visual", "gripper_fingers_visual", "gripper_collision"))
    assert np.allclose(fin.rgba, gcfg["fingers_rgba"]) and np.allclose(vis.rgba, gcfg["rgba"])
    assert vis.group == fin.group == 1 and vis.contype == fin.contype == 0 and fin.conaffinity == 0
    assert col.group == 3 and col.contype == 1 and col.conaffinity == 1
    assert m.mesh(col.dataid[0]).name == "gripper"


def test_scale_and_mount_match_old_part(gcfg):
    import struct
    b = (ROOT / gcfg["mesh"]).read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    assert len(b) == 84 + 50 * n == 635884 and n == 12716
    v = np.frombuffer(b[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].reshape(-1, 3)
    body_pts = np.array([gripper.mesh_to_body(gcfg, p) for p in v[::50]])
    # 80 x 80 mm cross-section = old housing_1 face (y, z in [-0.04, 0.04]); mount face at x = 0, tool along -x
    assert np.allclose(body_pts[:, 1:].min(0), -0.04, atol=1e-3) and np.allclose(body_pts[:, 1:].max(0), 0.04, atol=1e-3)
    assert np.isclose(body_pts[:, 0].max(), 0.0, atol=1e-6) and np.isclose(body_pts[:, 0].min(), -0.11895, atol=1e-4)


def test_urdf_variant_matches_mujoco(scene, gcfg):
    m, _, _ = scene
    urdf = ROOT / gcfg["urdf_out_dir"] / "6dof_gripper.urdf"
    root = ET.parse(urdf).getroot()
    link = next(l for l in root.findall("link") if l.get("name") == gcfg["mount_body"])
    text = urdf.read_text()
    assert "file://" not in text and "/home/" not in text and "/mnt/" not in text   # no machine-specific paths
    assert "meshes/housing_1.stl" not in text
    org = link.find("visual/origin")
    xyz = np.array(org.get("xyz").split(), float)
    r, p, y = np.array(org.get("rpy").split(), float)
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    pos, _, R = gripper.mount(gcfg)
    assert np.allclose(xyz, pos, atol=1e-6) and np.allclose(Rz @ Ry @ Rx, R, atol=1e-6)
    assert np.isclose(float(link.find("inertial/mass").get("value")), gcfg["mass_kg"])
    joint = next(j for j in root.findall("joint") if j.get("name") == "Revolute_6")
    assert joint.get("type") == "revolute" and joint.find("child").get("link") == gcfg["mount_body"]
