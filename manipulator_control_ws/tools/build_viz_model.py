"""Build the display model models/derived/viz_scene.xml from the copied, committed scene and cross-check it.

Input (read-only copy at model.UPSTREAM_COMMIT): models/upstream_<commit>/mujoco_sim/models/generated/pinch_scene.xml. Only the
robot (meshes, kinematic tree, gripper mesh on housing_1, tcp site) is taken, in the robot's own frame: the sim
stands the robot on a plate (demo.json mount, world z = 0 = the table), so its mount height is subtracted here and
z = 0 stays the base bottom (= the plate top, the mount plane of PathChecker). Dropped on purpose: the desk, the
cameras, the "ready" keyframe, the torque actuators, sensors and physics options (simulation assumptions, not facts
about the real setup). Added: a measured arm (collision geoms hidden in group 3), two translucent ghost copies
(cmd_ = servo internal target, tgt_ = final target; no collision), a decor floor at z = 0 (no collision), and the
mocap body ee_target (teleop EE target marker; no collision).

Checks written to models/derived/manifest.json (the build fails if any fails):
  - the five URDF/xacro files have identical joint kinematics (origin, axis, type, limits, parent/child, no mimic)
  - measured arm frames == 6dof_control.xml == 6dof_mj.urdf (compiled by MuJoCo) == pinch_scene.xml
  - ghost arms have the same frames as the measured arm
  - FK of the sim "ready" pose matches pinch_scene.xml and demo.json ready.tcp_m (both minus the mount height)

    python tools/build_viz_model.py
"""
from __future__ import annotations

import copy
import datetime as dt
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from manipulator_control import model as mdl  # noqa: E402

SRC_SCENE = mdl.UPSTREAM / "mujoco_sim/models/generated/pinch_scene.xml"
SRC_CONTROL = mdl.UPSTREAM / "6dof_description/mujoco/6dof_control.xml"
SRC_MJ_URDF = mdl.UPSTREAM / "6dof_description/mujoco/6dof_mj.urdf"
SRC_DEMO = mdl.UPSTREAM / "mujoco_sim/configs/demo.json"
MEASURED_RGBA = "0.72 0.74 0.78 1"
GHOST = {"cmd_": "1.0 0.55 0.1 0.35", "tgt_": "0.2 0.85 0.35 0.3"}


def mount_height() -> float:
    """Height of the sim's robot plate (demo.json mount): the scene's robot stands this much above its z = 0."""
    mc = json.loads(SRC_DEMO.read_text()).get("mount", {})
    return float(mc["height_m"]) if mc.get("enabled") else 0.0


def _lowered(el: ET.Element, h: float, expect_z: float | None = None) -> ET.Element:
    """Copy of a world-level element with its z lowered by h (expect_z: the z it must have in the scene)."""
    el = copy.deepcopy(el)
    pos = np.array((el.get("pos") or "0 0 0").split(), float)
    if expect_z is not None and abs(pos[2] - expect_z) > 1e-9:
        raise ValueError(f"{el.tag} {el.get('name') or el.get('mesh')}: z {pos[2]} in the scene, expected {expect_z}")
    pos[2] -= h
    el.set("pos", " ".join(f"{v:.12g}" for v in pos))
    return el


def build() -> ET.ElementTree:
    src = ET.parse(SRC_SCENE).getroot()
    mount = mount_height()
    root = ET.Element("mujoco", model="manipulator_viz")
    meshdir = Path("..") / mdl.UPSTREAM.name / "6dof_description" / "meshes"
    ET.SubElement(root, "compiler", angle="radian", meshdir=f"{meshdir.as_posix()}/")
    ET.SubElement(root, "statistic", extent="0.8", center="0 0 0.3")
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", azimuth="120", elevation="-20", offwidth="1280", offheight="960")
    ET.SubElement(vis, "headlight", ambient="0.35 0.35 0.35", diffuse="0.6 0.6 0.6", specular="0 0 0")

    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7", rgb2="0 0 0",
                  width="512", height="3072")
    ET.SubElement(asset, "texture", type="2d", name="grid", builtin="checker", mark="edge", rgb1="0.55 0.55 0.55",
                  rgb2="0.5 0.5 0.5", markrgb="0.4 0.4 0.4", width="300", height="300")
    ET.SubElement(asset, "material", name="decor_floor", texture="grid", texrepeat="8 8", texuniform="true",
                  rgba="1 1 1 0.6")
    for mesh in src.find("asset").iter("mesh"):
        asset.append(copy.deepcopy(mesh))

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 0 1.5", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="decor_floor", type="plane", size="1 1 0.05", material="decor_floor",
                  contype="0", conaffinity="0", group="2")
    src_world = src.find("worldbody")
    base = [_lowered(g, mount, expect_z=mount) for g in src_world.findall("geom") if g.get("mesh") == "base_link"]
    for g, name in zip(base, ("base_visual", "base_collision")):
        g.set("name", name)
        _style_measured(g)
        world.append(g)
    arm = _lowered(src_world.find("body[@name='r_a_1']"), mount)
    measured = copy.deepcopy(arm)
    for g in measured.iter("geom"):
        _style_measured(g)
    world.append(measured)
    for prefix, rgba in GHOST.items():
        world.append(_ghost(arm, prefix, rgba))
    # EE target for teleop: a mocap body (drag it in the viewer: double-click, then Ctrl + right/left drag).
    # Its frame is the tcp site frame (z = tool axis). Display only: no collision, no mass.
    target = ET.SubElement(world, "body", name="ee_target", mocap="true", pos="0 0 0.6")
    ET.SubElement(target, "geom", name="ee_target_ball", type="sphere", size="0.012", rgba="1 0.85 0.1 0.7",
                  contype="0", conaffinity="0", group="2")
    for axis, rgba, end in (("x", "1 0.2 0.2 0.9", "0.05 0 0"), ("y", "0.2 0.9 0.2 0.9", "0 0.05 0"),
                            ("z", "0.3 0.4 1 0.9", "0 0 0.05")):
        ET.SubElement(target, "geom", name=f"ee_target_{axis}", type="capsule", size="0.004", fromto=f"0 0 0 {end}",
                      rgba=rgba, contype="0", conaffinity="0", group="2")

    contact = ET.SubElement(root, "contact")
    ET.SubElement(contact, "exclude", body1="world", body2="r_a_1")
    ET.indent(root, space="  ")
    return ET.ElementTree(root)


def _style_measured(g: ET.Element) -> None:
    collides = g.get("contype") != "0" or g.get("conaffinity") != "0"
    if collides:
        g.set("group", "3")          # hidden by default, still collides (path check)
    g.set("rgba", MEASURED_RGBA)


def _ghost(arm: ET.Element, prefix: str, rgba: str) -> ET.Element:
    ghost = copy.deepcopy(arm)
    for el in ghost.iter():
        if el.tag in ("body", "joint", "geom", "site") and el.get("name"):
            el.set("name", prefix + el.get("name"))
    for el in ghost.iter("body"):
        if el.get("name") is None:
            raise ValueError("unnamed body in arm")
    for g in ghost.iter("geom"):
        visual = g.get("contype") == "0" and g.get("conaffinity") == "0"
        g.set("contype", "0")
        g.set("conaffinity", "0")
        g.set("group", "2" if visual else "5")
        g.set("rgba", rgba if visual else "0 0 0 0")
    for s in ghost.iter("site"):
        s.set("rgba", rgba)
    return ghost


def check(out: Path) -> dict:
    report = {"urdf": mdl.compare_urdf_kinematics()}
    errors = [f"URDF/xacro difference: {d}" for d in report["urdf"]["differences"]]
    for name, j in report["urdf"]["joints"].items():
        if j["type"] != "revolute" or j["mimic"]:
            errors.append(f"{name}: type {j['type']}, mimic {j['mimic']}")

    viz = mujoco.MjModel.from_xml_path(str(out))
    ctrl = mujoco.MjModel.from_xml_path(str(SRC_CONTROL))
    urdf = mujoco.MjModel.from_xml_path(str(SRC_MJ_URDF))
    scene = mujoco.MjModel.from_xml_path(str(SRC_SCENE))
    measured = mdl.body_frames(viz)
    mount = mount_height()
    report["mount_height_m"] = mount
    scene_frames = mdl.body_frames(scene)
    scene_frames["r_a_1"] = (scene_frames["r_a_1"][0] - [0, 0, mount],) + scene_frames["r_a_1"][1:]
    for label, frames in (("6dof_control.xml", mdl.body_frames(ctrl)), ("6dof_mj.urdf", mdl.body_frames(urdf)),
                          ("pinch_scene.xml", scene_frames)):
        bad = mdl.compare_frames(measured, frames, mdl.ARM_BODIES)
        report[f"frames_vs_{label}"] = bad or "identical"
        errors += [f"vs {label}: {b}" for b in bad]
    for prefix in GHOST:
        bad = mdl.compare_frames(measured, mdl.body_frames(viz, prefix), mdl.ARM_BODIES, atol=0)
        report[f"frames_{prefix}ghost"] = bad or "identical"
        errors += [f"ghost {prefix}: {b}" for b in bad]

    arms = {arm: mdl.arm_index(viz, p) for arm, p in mdl.ARMS.items()}
    report["qpos_addresses"] = {arm: dict(zip(mdl.JOINTS, idx.qpos.tolist())) for arm, idx in arms.items()}
    if len({a for idx in arms.values() for a in idx.qpos}) != 18 or viz.nq != 18:
        errors.append(f"expected 18 distinct qpos addresses, nq = {viz.nq}")

    demo = json.loads(SRC_DEMO.read_text())
    q_ready = np.array(demo["ready"]["q_rad"])
    d_viz = mujoco.MjData(viz)
    tcp_viz = mdl.tcp_position(viz, d_viz, arms["measured"], q_ready)
    d_scene = mujoco.MjData(scene)
    d_scene.qpos[[scene.joint(n).qposadr[0] for n in mdl.JOINTS]] = q_ready
    mujoco.mj_kinematics(scene, d_scene)
    tcp_scene = d_scene.site_xpos[scene.site("tcp").id] - [0, 0, mount]
    tcp_demo = np.asarray(demo["ready"]["tcp_m"], float) - [0, 0, mount]
    report["tcp_ready_m"] = {"viz": tcp_viz.round(6).tolist(), "pinch_scene": tcp_scene.round(6).tolist(),
                             "demo_json": tcp_demo.round(6).tolist()}
    if not np.allclose(tcp_viz, tcp_scene, atol=1e-12, rtol=0):
        errors.append(f"tcp FK differs from pinch_scene.xml: {tcp_viz} vs {tcp_scene}")
    if not np.allclose(tcp_viz, tcp_demo, atol=5e-6, rtol=0):
        errors.append(f"tcp FK differs from demo.json ready.tcp_m: {tcp_viz}")

    checker = mdl.PathChecker(viz, arms["measured"])
    report["path_check_q0"] = checker.check(np.zeros(6)) or "clean"
    report["path_check_ready"] = checker.check(q_ready) or "clean"
    report["errors"] = errors
    return report


def main() -> int:
    mdl.DERIVED.mkdir(parents=True, exist_ok=True)
    build().write(mdl.VIZ_XML, encoding="unicode", xml_declaration=False)
    report = check(mdl.VIZ_XML)
    manifest = {
        "built": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "tool": "tools/build_viz_model.py",
        "sources": {str(p.relative_to(mdl.WS)): mdl.file_sha(p) for p in (SRC_SCENE, SRC_CONTROL, SRC_MJ_URDF, SRC_DEMO)},
        "output": str(mdl.VIZ_XML.relative_to(mdl.WS)),
        "output_sha256": mdl.file_sha(mdl.VIZ_XML),
        "mujoco": mujoco.__version__,
        "checks": report,
    }
    mdl.VIZ_MANIFEST.write_text(json.dumps(manifest, indent=1, ensure_ascii=False, default=str) + "\n")
    for k, v in report.items():
        if k not in ("urdf",):
            print(f"{k}: {v}")
    print(f"URDF/xacro differences: {report['urdf']['differences'] or 'none'}")
    if report["errors"]:
        print("BUILD CHECK FAILED", file=sys.stderr)
        return 1
    print(f"wrote {mdl.VIZ_XML} (sha {manifest['output_sha256'][:12]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
