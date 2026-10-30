"""Fixed gripper that REPLACES the original end part (housing_1.stl) on body/link `housing_1`.

Kept unchanged: joint Revolute_6 (parent r_a__2__1, child housing_1, axis local -x), the body/link
`housing_1` and its frame, the 6 joints and 6 torque motors. Removed: the two housing_1.stl geoms
(visual + collision), the housing_1 mesh asset, and housing_1's explicit inertial.
Added on the SAME (rotating) body: the gripper mesh (visual + collision geom), mass from an assumed
density, and the TCP at the gripper tip. No finger joints or actuators. With `fingers_mesh` the drawing is split:
gripper.stl's body and fingers (its connected components, triangles unchanged) are two visual geoms in their own
colours, and the collision geom (still the whole gripper.stl) is hidden in group 3 so it does not cover them.

Mount transform (mesh file coordinates in mm -> housing_1 body frame in m):
    p_body = R (scale * p_mesh - scale * c_mount) + t_mount
    R = mesh_to_body_quat (configs/demo.json, Ry(-90 deg): mesh +z -> body -x = J6/tool axis, mesh +x -> body +z, mesh +y -> body +y)
    c_mount = centre of the 80x80 mount face (mesh z_min), t_mount = (0, 0, 0) = the J6 axis on the
              face where housing_1.stl met r_a__2__1 (housing_1 occupied body x in [-0.030, 0]).
"""
from __future__ import annotations

import os
import re
from pathlib import Path

import mujoco
import numpy as np

from .model import ROOT


def rot(quat) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, float))
    return R.reshape(3, 3)


def mount(gcfg: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(geom_pos, geom_quat, R): pose of the mesh file frame in the body frame."""
    R = rot(gcfg["mesh_to_body_quat"])
    c = gcfg["scale"] * np.asarray(gcfg["mount_center_mesh_mm"], float)
    pos = np.asarray(gcfg["mount_center_body_m"], float) - R @ c
    return pos, np.asarray(gcfg["mesh_to_body_quat"], float), R


def mesh_to_body(gcfg: dict, p_mesh_mm) -> np.ndarray:
    pos, _, R = mount(gcfg)
    return R @ (gcfg["scale"] * np.asarray(p_mesh_mm, float)) + pos


def tcp_local(gcfg: dict) -> tuple[np.ndarray, np.ndarray]:
    """TCP position (tip face centre) and tool axis (mount -> tip direction) in the body frame."""
    p = mesh_to_body(gcfg, gcfg["tip_center_mesh_mm"])
    axis = rot(gcfg["mesh_to_body_quat"]) @ np.array([0.0, 0.0, 1.0])
    return p, axis


def visual_parts(gcfg: dict) -> list[tuple[str, str, list, str]]:
    """(geom name, mesh path, rgba, URDF material) of the drawn parts."""
    rgba = gcfg.get("rgba", [0.35, 0.55, 0.85, 1])
    if "fingers_mesh" not in gcfg:
        return [("gripper_visual", gcfg["mesh"], rgba, "silver")]
    return [("gripper_visual", gcfg["body_mesh"], rgba, "silver"),
            ("gripper_fingers_visual", gcfg["fingers_mesh"], gcfg["fingers_rgba"], "black")]


def apply_to_spec(spec: mujoco.MjSpec, gcfg: dict, meshdir_abs: Path) -> dict:
    """Remove housing_1 geometry/inertia from the mount body and add the gripper there."""
    body = spec.body(gcfg["mount_body"])
    removed = []
    for g in list(body.geoms):
        if g.meshname == gcfg["removed_mesh"]:
            removed.append({"group": g.group, "contype": g.contype})
            spec.delete(g)
    if len(removed) != 2:
        raise RuntimeError(f"expected 2 {gcfg['removed_mesh']} geoms on {gcfg['mount_body']}, found {removed}")
    spec.delete(spec.mesh(gcfg["removed_mesh"]))

    def add_mesh(name, path):
        rel = Path(ROOT / path).resolve().relative_to(meshdir_abs.resolve(), walk_up=True)
        spec.add_mesh(name=name, file=str(rel), scale=[gcfg["scale"]] * 3)
    add_mesh("gripper", gcfg["mesh"])
    pos, quat, _ = mount(gcfg)
    rgba = gcfg.get("rgba", [0.35, 0.55, 0.85, 1])
    parts = visual_parts(gcfg)
    # same pattern as the original links: visual geoms (group 1, no contacts, no mass) and a collision geom
    for name, mesh, part_rgba, _ in parts:
        mesh_name = Path(mesh).stem
        if mesh_name != "gripper":
            add_mesh(mesh_name, mesh)
        body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh_name, pos=pos, quat=quat,
                      group=1, contype=0, conaffinity=0, density=0, rgba=part_rgba)
    body.add_geom(name="gripper_collision", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="gripper", pos=pos,
                  quat=quat, group=3 if len(parts) > 1 else 0, contype=1, conaffinity=1, mass=gcfg["mass_kg"],
                  rgba=rgba)
    body.explicitinertial = False          # inertia now computed from the gripper collision geom (uniform density)
    return {"removed_geoms": removed, "removed_mesh": gcfg["removed_mesh"], "geom_pos_m": pos.round(6).tolist(),
            "geom_quat": quat.round(6).tolist()}


# ---------------------------------------------------------------------------------------------- URDF
def _rpy(R) -> list[float]:
    """URDF rpy (R = Rz(y) Ry(p) Rx(r))."""
    p = -np.arcsin(np.clip(R[2, 0], -1, 1))
    r = np.arctan2(R[2, 1], R[2, 2])
    y = np.arctan2(R[1, 0], R[0, 0])
    return [float(r), float(p), float(y)]


def _mesh_uri(gcfg: dict, mesh: str) -> str:
    """urdf_mesh_uri's directory + the mesh file name (machine-independent), else a file:// path."""
    if gcfg.get("urdf_mesh_uri"):
        return gcfg["urdf_mesh_uri"].rsplit("/", 1)[0] + "/" + Path(mesh).name
    return "file://" + str(Path(ROOT / mesh).resolve())


def urdf_link_block(gcfg: dict, inertial: dict) -> str:
    pos, _, R = mount(gcfg)
    rpy = _rpy(R)
    s = gcfg["scale"]
    I = inertial["fullinertia"]   # ixx iyy izz ixy ixz iyz in the link frame, about the CoM

    def geom(mesh):
        return (f'    <origin xyz="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>\n'
                f'    <geometry>\n      <mesh filename="{_mesh_uri(gcfg, mesh)}" scale="{s} {s} {s}"/>\n    </geometry>\n')
    visuals = "".join(f'  <visual>\n{geom(mesh)}    <material name="{material}"/>\n  </visual>\n'
                      for _, mesh, _, material in visual_parts(gcfg))
    c = inertial["com"]
    return (f'<link name="{gcfg["mount_body"]}">\n'
            f'  <!-- housing_1.stl removed; fixed gripper ({Path(gcfg["mesh"]).name}) on the same link, '
            f'generated by pinch_sim.build_scene -->\n'
            f'  <inertial>\n    <origin xyz="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" rpy="0 0 0"/>\n'
            f'    <mass value="{inertial["mass"]:.6f}"/>\n'
            f'    <inertia ixx="{I[0]:.9f}" iyy="{I[1]:.9f}" izz="{I[2]:.9f}" ixy="{I[3]:.9f}" '
            f'ixz="{I[4]:.9f}" iyz="{I[5]:.9f}"/>\n  </inertial>\n'
            f'{visuals}'
            f'  <collision>\n{geom(gcfg["mesh"])}  </collision>\n</link>')


def write_urdf_variants(gcfg: dict, inertial: dict, out_dir: Path) -> list[str]:
    """Copy the original URDF/xacro with ONLY the housing_1 link block replaced; returns written paths."""
    src = ROOT / gcfg["urdf_src_dir"]
    block = urdf_link_block(gcfg, inertial)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for name, out in (("6dof.xacro", "6dof_gripper.xacro"), ("6dof.urdf", "6dof_gripper.urdf")):
        text = (src / name).read_text()
        pat = re.compile(r'<link name="%s">.*?</link>' % re.escape(gcfg["mount_body"]), re.S)
        if len(pat.findall(text)) != 1:
            raise RuntimeError(f"{name}: expected one {gcfg['mount_body']} link block")
        new = pat.sub(lambda _: block, text)
        if "meshes/housing_1.stl" in new:
            raise RuntimeError("housing_1.stl still referenced")
        new = new.replace('<robot name="6dof"', '<robot name="6dof_gripper"', 1)
        (out_dir / out).write_text(new)
        written.append(os.path.relpath(out_dir / out, ROOT))
    return written
