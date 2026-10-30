"""Generate models/generated/pinch_scene.xml from the unmodified original scene.

Every addition is made here, explicitly, with MjSpec:
  * gripper (cfg.gripper.enabled): housing_1.stl geoms/mesh/inertial REMOVED from body housing_1 and the
    gripper mesh added on that same body (Revolute_6 child), see gripper.py; URDF/xacro variants written to
    gripper.urdf_out_dir (../6dof_description/urdf/6dof_gripper.{urdf,xacro}) with only the housing_1 link block replaced
  * site `tcp` on housing_1: gripper tip centre (gripper enabled) or the old flange-face centre,
    z-axis = tool axis = housing_1 local -x
  * cameras `side_cam` / `side_cam_alt` aimed at a look-at point (quaternion computed below)
  * exclude world<->r_a_1 (bodies joined by Revolute_1, permanently overlapping in the CAD meshes; see docs/FRAMES.md)
  * desk (cfg.desk): wooden desk body with its top surface at z = 0 and the grey floor plane lowered under
    it (scene_env.py); no extra contact exclusion
  * mount (cfg.mount): the robot raised by mount.height_m onto a grey plate centred on the J1 axis; the table
    stays at z = 0 (scene_env.add_mount)
  * keyframe `ready`: ready.q_rad if given (checked against ready.tcp_m / tool_axis), otherwise
    solved by IK for ready.tcp_m with the tool pointing down
    (joints listed in ready.lock_joints_rad are held at the given value during that IK)

Usage: python -m pinch_sim.build_scene
"""
from __future__ import annotations

import copy
import json
import os
import time

import mujoco
import numpy as np

from . import gripper as grip
from .collision import CollisionChecker
from .frames import look_at_quat, zaxis_quat
from .kinematics import Kinematics
from .scene_env import add_desk_and_floor, add_mount, relativize_texture
from .model import JOINTS, ROOT, RobotIndex, file_sha, load_config


def make_spec(cfg: dict) -> tuple[mujoco.MjSpec, dict]:
    mc = cfg["model"]
    spec = mujoco.MjSpec.from_file(str(ROOT / mc["source_scene"]))
    info = {}
    if cfg.get("mount", {}).get("enabled"):
        info["mount"] = add_mount(spec, cfg["mount"])
    gcfg = cfg.get("gripper", {})
    if gcfg.get("enabled"):
        meshdir_abs = (ROOT / mc["source_scene"]).parent / spec.meshdir
        info["gripper"] = grip.apply_to_spec(spec, gcfg, meshdir_abs)
        tcp_pos, tcp_axis = grip.tcp_local(gcfg)
        tcp_body = gcfg["mount_body"]
        tcp_quat = np.asarray(gcfg["mesh_to_body_quat"], float)   # TCP frame = gripper mesh axes (z = tool axis)
    else:
        tcp_pos = np.asarray(mc["tcp_local_pos_m"]) + mc["tool_length_m"] * np.asarray(mc["tcp_local_zaxis"])
        tcp_axis, tcp_body = np.asarray(mc["tcp_local_zaxis"], float), mc["tcp_body"]
        tcp_quat = zaxis_quat(tcp_axis)
    info["tcp"] = (tcp_body, tcp_pos, tcp_axis)
    spec.body(tcp_body).add_site(name="tcp", pos=tcp_pos, quat=tcp_quat,
                                 size=[0.004, 0, 0], rgba=[1, 0.5, 0, 1], group=2)
    cams = cfg["cameras"]
    for name in ("side_cam", "side_cam_alt"):
        c = cams[name]
        spec.worldbody.add_camera(name=name, pos=c["pos_m"], quat=look_at_quat(c["pos_m"], c["lookat_m"]),
                                  fovy=c["fovy_deg"])
    for b1, b2 in mc["exclude_body_pairs"]:
        spec.add_exclude(bodyname1=b1, bodyname2=b2)
    if cfg.get("desk", {}).get("enabled"):
        info["desk"] = add_desk_and_floor(spec, cfg["desk"], (ROOT / mc["generated_scene"]).parent)
    spec.visual.global_.offwidth = max(1280, cams["width_px"])
    spec.visual.global_.offheight = max(960, cams["height_px"])
    return spec, info


def build(cfg: dict | None = None) -> dict:
    cfg = cfg or load_config()
    spec, info = make_spec(cfg)
    m = spec.compile()
    idx = RobotIndex.from_model(m)
    r = cfg["ready"]
    ik_idx = copy.deepcopy(idx)
    for name, value in r.get("lock_joints_rad", {}).items():   # e.g. Revolute_1 = 0 so J2's axis is world +X
        j = JOINTS.index(name)
        ik_idx.lo[j] = ik_idx.hi[j] = value
    kin = Kinematics(m, ik_idx, cfg["ik"])
    if "q_rad" in r:
        # home given in joint space (used when locked joints leave fewer DOF than the 5-D tool task)
        q = np.asarray(r["q_rad"], float)
        pose = kin.fk(q)
        ik = type("Home", (), {"q": q, "ok": True, "iters": 0,
                               "pos_err_m": float(np.linalg.norm(pose.p_m - r["tcp_m"])),
                               "axis_err_deg": float(np.degrees(np.arccos(np.clip(pose.tool_axis @ r["tool_axis"], -1, 1))))})()
        if ik.pos_err_m > 1e-5 or ik.axis_err_deg > 0.01:
            raise RuntimeError(f"ready.q_rad does not give ready.tcp_m/tool_axis: {ik.pos_err_m} m {ik.axis_err_deg} deg")
    else:
        ik = kin.solve_ik(r["tcp_m"], r["tool_axis"], r["seed_q_rad"])
        if not ik.ok:
            raise RuntimeError(f"ready IK failed: {ik}")
    col = CollisionChecker(m, idx).check(ik.q)
    q_full = np.zeros(m.nq)
    q_full[idx.qpos] = ik.q
    spec.add_key(name="ready", qpos=q_full, ctrl=np.zeros(m.nu))

    out = ROOT / cfg["model"]["generated_scene"]
    out.parent.mkdir(parents=True, exist_ok=True)
    # meshdir must be relative to the generated file, not to the original mujoco/ folder
    # (to_xml recompiles against the original location, so the attribute is rewritten in the text)
    mesh_abs = ((ROOT / cfg["model"]["source_scene"]).parent / spec.meshdir).resolve()
    meshdir = os.path.relpath(mesh_abs, out.parent.resolve()) + "/"
    xml = spec.to_xml()
    old = f'meshdir="{spec.meshdir}"'
    if xml.count(old) != 1:
        raise RuntimeError(f"expected exactly one {old} in generated XML")
    xml = relativize_texture(xml.replace(old, f'meshdir="{meshdir}"'), out.parent)
    out.write_text(xml, encoding="utf-8")
    m2 = mujoco.MjModel.from_xml_path(str(out))  # reload proves the file is self-contained
    report = {
        "generated": os.path.relpath(out, ROOT),
        "sha256": file_sha(out),
        "mujoco": mujoco.__version__,
        "built_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "ready_q_rad": ik.q.round(6).tolist(),
        "ready_ik": {"pos_err_m": ik.pos_err_m, "axis_err_deg": ik.axis_err_deg, "iters": ik.iters},
        "ready_contacts": col.contacts,
        "ready_floor_clearance_m": col.floor_clearance_m,
        "ready_self_clearance_m": col.self_clearance_m,
        "ready_self_pair": col.self_pair,
        "camera_quat": {n: m2.cam_quat[m2.camera(n).id].round(6).tolist() for n in ("side_cam", "side_cam_alt")},
        "nexclude": int(m2.nexclude),
        "tcp_body": info["tcp"][0], "tcp_local_pos_m": np.round(info["tcp"][1], 6).tolist(),
        "tcp_local_axis": np.round(info["tcp"][2], 6).tolist(),
        "nu": int(m2.nu), "njnt": int(m2.njnt),
        "desk": info.get("desk"),
        "mount": info.get("mount"),
    }
    if cfg.get("gripper", {}).get("enabled"):
        b = m2.body(cfg["gripper"]["mount_body"]).id
        Rb = grip.rot(m2.body_iquat[b])
        Ifull = Rb @ np.diag(m2.body_inertia[b]) @ Rb.T
        inertial = {"mass": float(m2.body_mass[b]), "com": m2.body_ipos[b].tolist(),
                    "fullinertia": [Ifull[0, 0], Ifull[1, 1], Ifull[2, 2], Ifull[0, 1], Ifull[0, 2], Ifull[1, 2]]}
        report["gripper"] = dict(info["gripper"], inertial=inertial,
                                 urdf=grip.write_urdf_variants(cfg["gripper"], inertial, ROOT / cfg["gripper"]["urdf_out_dir"]),
                                 mesh_sha256=file_sha(ROOT / cfg["gripper"]["mesh"]))
    (ROOT / "logs").mkdir(exist_ok=True)
    (ROOT / "logs/build_scene.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    return report


if __name__ == "__main__":
    print(json.dumps(build(), indent=2, ensure_ascii=False))
