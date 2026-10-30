"""Real-arm profile (pinch_sim.app --real-arm): plan for the real arm of <repo>/manipulator_control_ws.

That program (`python -m manipulator_control pinch`) owns the arm and reads its config/robot.yaml. This module reads
the same file (never writes it), so both use one set of joint limits and one HOME:
  - HOME (robot.yaml home.q_rad) replaces the keyframe "ready": every start pose, H and the return path use it.
    The real HOME is -1 x the demo's ready pose (the arm's mirror image), so the real arm works on model -Y.
  - The joint limits (limit_deg) narrow the model's +-pi in RobotIndex: IK, prechecks and validation use them.
  - The desk and the cameras are mirrored (y -> -y, done on the loaded spec; the generated XML is not touched) and
    targets are generated on -Y (target.front_y = -1).
  - Planning for the arm's streaming (configs/real_arm.json, merged into the config, so part of config_sha): speed
    ratio 0.95 (PV headroom), qdd 1.2 rad/s^2, braking points inside the range, 5 mm self clearance, 30 s.
The arm program refuses a simulator whose hello differs (robot.yaml sha, model sha, limits, HOME, strictness).
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .frames import look_at_quat
from .model import JOINTS, ROOT, RobotIndex, file_sha, with_overrides

# the arm program's workspace: <repo>/manipulator_control_ws next to mujoco_sim (or $MANIPULATOR_CONTROL_WS)
ARM_WS = Path(os.environ.get("MANIPULATOR_CONTROL_WS") or ROOT.parent / "manipulator_control_ws")
DEFAULT_ROBOT_YAML = ARM_WS / "config" / "robot.yaml"
PROFILE = ROOT / "configs" / "real_arm.json"


def robot_sha(doc: dict) -> str:
    """The same digest as manipulator_control.config (sha256 of the parsed YAML as sorted JSON)."""
    return hashlib.sha256(json.dumps(doc, sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class RealArm:
    path: Path
    robot_sha: str
    limits_rad: np.ndarray          # (6, 2)
    home_q_rad: np.ndarray          # (6,)

    @classmethod
    def load(cls, path: str | Path | None = None) -> "RealArm":
        import yaml
        path = Path(path) if path else DEFAULT_ROBOT_YAML
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        except OSError as e:
            raise ValueError(f"cannot read the arm's robot.yaml {path}: {e}") from None
        joints = {j.get("name"): j for j in doc.get("joints", [])}
        missing = [n for n in JOINTS if n not in joints or joints[n].get("limit_deg") is None]
        home = (doc.get("home") or {}).get("q_rad")
        if missing or home is None or len(home) != len(JOINTS):
            raise ValueError(f"{path}: joint ranges {missing or ''} or home.q_rad not set (null = unknown)")
        lim = np.radians([joints[n]["limit_deg"] for n in JOINTS]).astype(float)
        return cls(path, robot_sha(doc), lim, np.asarray(home, float))


def apply_profile(cfg: dict, arm: RealArm, profile: str | Path = PROFILE) -> dict:
    """The demo config with the real-arm overrides and cfg["real_arm"] (what the arm program compares in hello)."""
    prof = json.loads(Path(profile).read_text(encoding="utf-8"))
    out = copy.deepcopy(cfg)
    for name, values in prof.items():
        if not name.startswith("_") and isinstance(values, dict):
            out = with_overrides(out, **{name: {k: v for k, v in values.items() if not k.startswith("_")}})
    out["ready"] = dict(out["ready"], q_rad=arm.home_q_rad.tolist(), tcp_m=None)
    out["real_arm"] = dict(out.get("real_arm", {}), robot_yaml=str(arm.path), robot_sha=arm.robot_sha,
                           limits_deg=np.degrees(arm.limits_rad).round(6).tolist(),
                           home_q_rad=arm.home_q_rad.round(6).tolist())
    return out


def load_scene(cfg: dict) -> tuple[mujoco.MjModel, RobotIndex, str]:
    """The generated scene for the real arm: HOME keyframe, mirrored desk and cameras, real joint ranges.
    The returned sha is the XML file's (the robot model is unchanged; the profile is in config_sha)."""
    path = ROOT / cfg["model"]["generated_scene"]
    if not path.exists():
        raise FileNotFoundError(f"{path} missing: run `python -m pinch_sim.build_scene` first")
    real = cfg["real_arm"]
    spec = mujoco.MjSpec.from_file(str(path))
    if real.get("mirror_desk", True):
        for g in spec.body("desk").geoms:
            g.pos = [g.pos[0], -g.pos[1], g.pos[2]]
    for name, c in cfg["cameras"].items():
        if isinstance(c, dict) and "pos_m" in c and spec.camera(name) is not None:
            cam = spec.camera(name)
            cam.pos = c["pos_m"]
            cam.quat = look_at_quat(c["pos_m"], c["lookat_m"])
    m0 = spec.compile()
    jq = [m0.jnt_qposadr[m0.joint(n).id] for n in JOINTS]
    q = np.array(m0.key("ready").qpos, float)
    q[jq] = np.asarray(real["home_q_rad"], float)
    spec.key("ready").qpos = q.tolist()
    m = spec.compile()
    idx = RobotIndex.from_model(m)
    lim = np.radians(np.asarray(real["limits_deg"], float))
    idx.lo, idx.hi = np.maximum(idx.lo, lim[:, 0]), np.minimum(idx.hi, lim[:, 1])   # only ever narrower
    return m, idx, file_sha(path)


def hello(cfg: dict, model_sha: str, extra: dict | None = None) -> dict:
    """The simulator's hello to the arm program: what it loaded and how strictly it validates."""
    lim, real = cfg["limits"], cfg["real_arm"]
    return dict({"v": 1, "type": "hello", "role": "pinch_sim", "robot_sha": real["robot_sha"], "model_sha": model_sha,
                 "limits_deg": real["limits_deg"], "home_q_rad": real["home_q_rad"],
                 "validation": {"joint_speed_max_deg_s": float(lim["joint_speed_max_deg_s"]),
                                "qdd_max_deg_s2": float(np.degrees(lim["qdd_max_rad_s2"])),
                                "joint_margin_deg": float(np.degrees(lim["joint_margin_rad"])),
                                "dt_s": float(cfg["timing"]["dt_check_s"])}}, **(extra or {}))
