"""Display model: loading, joint-name -> qpos mapping for the three arms, kinematic cross-checks, path check.

The display model (models/derived/viz_scene.xml, built by tools/build_viz_model.py) holds three copies of the arm:
  measured  (no prefix)  solid grey    - angles computed from the encoders (the real STATE)
  cmd_      (orange)     translucent   - the servo's own internal target (Position Trajectory, register 140)
  tgt_      (green)      translucent   - the final target (reference pose while waiting, HOME while moving)
Only the measured arm has collision geoms; ghosts and the decor floor never collide and are never used as
collision evidence. Joints are looked up by name; nothing relies on the order of qpos.
"""
from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from . import WS

JOINTS = tuple(f"Revolute_{i}" for i in range(1, 7))
ARMS = {"measured": "", "cmd": "cmd_", "target": "tgt_"}
DERIVED = WS / "models" / "derived"
VIZ_XML = DERIVED / "viz_scene.xml"
VIZ_MANIFEST = DERIVED / "manifest.json"
UPSTREAM_COMMIT = "849f2e1"     # reference repo commit of the model copy (tools/copy_model.py); was d654a11 until 2026-10-28 (ready 5 cm lower)
UPSTREAM = WS / "models" / f"upstream_{UPSTREAM_COMMIT}"


def file_sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class ArmIndex:
    prefix: str
    qpos: np.ndarray           # (6,) qpos addresses in Revolute_1..6 order, resolved by name
    geoms: np.ndarray          # geom ids of this arm (moving bodies only)
    tcp_site: int


def arm_index(m: mujoco.MjModel, prefix: str) -> ArmIndex:
    jid = [m.joint(prefix + n).id for n in JOINTS]
    root = m.body(prefix + "r_a_1").id
    bodies = {b for b in range(m.nbody) if _has_ancestor(m, b, root)}
    return ArmIndex(prefix, np.array([m.jnt_qposadr[j] for j in jid]),
                    np.array([g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies]),
                    m.site(prefix + "tcp").id)


def _has_ancestor(m: mujoco.MjModel, body: int, root: int) -> bool:
    while body > 0:
        if body == root:
            return True
        body = m.body_parentid[body]
    return False


def load_viz(path: Path = VIZ_XML, check_manifest: bool = True):
    """(model, {arm: ArmIndex}). Refuses a display model whose sha differs from the one its builder recorded."""
    if check_manifest:
        man = json.loads(VIZ_MANIFEST.read_text())
        if file_sha(path) != man["output_sha256"]:
            raise RuntimeError(f"{path} was changed after tools/build_viz_model.py wrote it; rebuild it")
    m = mujoco.MjModel.from_xml_path(str(path))
    return m, {arm: arm_index(m, p) for arm, p in ARMS.items()}


def set_arm(d: mujoco.MjData, idx: ArmIndex, q) -> None:
    d.qpos[idx.qpos] = q


def tcp_position(m: mujoco.MjModel, d: mujoco.MjData, idx: ArmIndex, q) -> np.ndarray:
    set_arm(d, idx, q)
    mujoco.mj_kinematics(m, d)
    return d.site_xpos[idx.tcp_site].copy()


# ---------------------------------------------------------------- kinematic cross-checks (model provenance)

def urdf_joints(path: Path) -> dict[str, dict]:
    """Joint table of a URDF or xacro file (both are plain XML for the joint elements)."""
    out = {}
    for j in ET.parse(path).getroot().iter("joint"):
        org, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        out[j.get("name")] = {
            "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": [float(v) for v in org.get("xyz", "0 0 0").split()],
            "rpy": [float(v) for v in org.get("rpy", "0 0 0").split()],
            "axis": [float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()],
            "limit": None if lim is None else {k: float(lim.get(k)) for k in ("lower", "upper", "velocity", "effort")
                                                if lim.get(k) is not None},
            "mimic": j.find("mimic") is not None,
        }
    return out


URDF_FILES = ["6dof_description/urdf/6dof.urdf", "6dof_description/urdf/6dof.xacro",
              "6dof_description/urdf/6dof_gripper.urdf", "6dof_description/urdf/6dof_gripper.xacro",
              "6dof_description/mujoco/6dof_mj.urdf"]


def compare_urdf_kinematics(upstream: Path = UPSTREAM) -> dict:
    """Joint-by-joint comparison of the five model files (origin, axis, type, limits, parent/child, mimic)."""
    tables = {f: urdf_joints(upstream / f) for f in URDF_FILES}
    base = tables[URDF_FILES[0]]
    diffs = []
    for f, t in tables.items():
        if set(t) != set(base):
            diffs.append({"file": f, "joints": sorted(set(t) ^ set(base))})
            continue
        for name, j in t.items():
            for key, v in j.items():
                b = base[name][key]
                same = np.allclose(v, b, atol=0, rtol=0) if isinstance(v, list) else v == b
                if not same:
                    diffs.append({"file": f, "joint": name, "key": key, "value": v, "base": b})
    return {"files": URDF_FILES, "joints": {n: base[n] for n in sorted(base)}, "differences": diffs}


ARM_BODIES = ("r_a_1", "link_1", "link__1__1", "r_a__1__1", "r_a__2__1", "housing_1")


def body_frames(m: mujoco.MjModel, prefix: str = "", names=ARM_BODIES) -> dict[str, tuple]:
    """body name (without prefix) -> (pos, quat, joint axis, joint pos) relative to the parent body."""
    out = {}
    for n in names:
        b = m.body(prefix + n).id
        jnt = [j for j in range(m.njnt) if m.jnt_bodyid[j] == b]
        out[n] = (m.body_pos[b].copy(), m.body_quat[b].copy(),
                  m.jnt_axis[jnt[0]].copy() if jnt else None, m.jnt_pos[jnt[0]].copy() if jnt else None)
    return out


def compare_frames(a: dict, b: dict, names, atol: float = 1e-9) -> list[str]:
    bad = []
    for n in names:
        if n not in a or n not in b:
            bad.append(f"{n}: missing")
            continue
        for k, (x, y) in enumerate(zip(a[n], b[n])):
            if (x is None) != (y is None) or (x is not None and not np.allclose(x, y, atol=atol, rtol=0)):
                bad.append(f"{n}: {('pos', 'quat', 'axis', 'jnt_pos')[k]} {x} != {y}")
    return bad


# ---------------------------------------------------------------- path check (model-based reference only)

class PathChecker:
    """Self-contact of the measured arm's collision geoms and their lowest point vs the mounting plane z = 0.

    Model-based only (STL collision meshes are convex hulls in MuJoCo): it can miss real contacts and say
    nothing about the desk, cables or people. A clean result never replaces the user's check of the real path."""

    def __init__(self, m: mujoco.MjModel, idx: ArmIndex, plane_margin_m: float = 0.0):
        self.m, self.d, self.idx, self.margin = m, mujoco.MjData(m), idx, plane_margin_m
        self.robot_geoms = {int(g) for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]}

    def check(self, q) -> list[str]:
        m, d = self.m, self.d
        set_arm(d, self.idx, q)
        mujoco.mj_forward(m, d)
        problems = []
        for c in d.contact[:d.ncon]:
            if c.geom1 in self.robot_geoms and c.geom2 in self.robot_geoms and c.dist < 0:
                b1, b2 = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
                problems.append(f"self-contact {b1}-{b2} ({c.dist * 1000:.1f} mm)")
        for g in self.idx.geoms:
            if not (m.geom_contype[g] or m.geom_conaffinity[g]) or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            z = _mesh_min_z(m, d, int(g))
            if z < -self.margin:
                problems.append(f"{m.body(m.geom_bodyid[g]).name} below mount plane ({z * 1000:.1f} mm)")
        return problems


def _mesh_min_z(m: mujoco.MjModel, d: mujoco.MjData, g: int) -> float:
    mid = m.geom_dataid[g]
    start, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    verts = m.mesh_vert[start:start + n]
    world = d.geom_xpos[g] + verts @ d.geom_xmat[g].reshape(3, 3).T
    return float(world[:, 2].min())


def deg(x) -> np.ndarray:
    return np.degrees(np.asarray(x, float))


def rad(x) -> np.ndarray:
    return np.radians(np.asarray(x, float))

