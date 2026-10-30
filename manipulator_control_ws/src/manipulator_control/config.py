"""Configuration loading and the two separate gates: read-only tools vs. hardware motion.

robot.yaml (hardware) and fake_robot.yaml (virtual) share one schema; motion.yaml is shared. An unknown value is
null and stays None here: nothing in this module turns a missing value into 0 or a default.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from . import WS
from .model import JOINTS
from .units import JointMap

ROBOT_YAML = WS / "config" / "robot.yaml"
FAKE_YAML = WS / "config" / "fake_robot.yaml"
MOTION_YAML = WS / "config" / "motion.yaml"


@dataclass(frozen=True)
class JointCfg:
    name: str
    dxl_id: int
    gear: float
    sign: int | None
    q_ref_rad: float | None
    limit_rad: tuple[float, float] | None

    def joint_map(self) -> JointMap:
        if self.sign is None:
            raise ValueError(f"{self.name}: sign not configured")
        return JointMap(self.name, self.dxl_id, self.gear, self.sign)


@dataclass(frozen=True)
class Config:
    robot: dict
    motion: dict
    joints: tuple[JointCfg, ...]
    home_q_rad: tuple[float, ...]
    robot_path: Path
    robot_sha: str
    motion_sha: str

    @property
    def kind(self) -> str:
        return self.robot["kind"]

    @property
    def ids(self) -> tuple[int, ...]:
        return tuple(j.dxl_id for j in self.joints)

    def expect(self, key: str):
        return self.robot["servo_expect"][key]

    def m(self, section: str, key: str):
        return self.motion[section][key]


def _sha(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


def load(robot_path: Path = ROBOT_YAML, motion_path: Path = MOTION_YAML) -> Config:
    robot = yaml.safe_load(Path(robot_path).read_text(encoding="utf-8"))
    motion = yaml.safe_load(Path(motion_path).read_text(encoding="utf-8"))
    problems = structural_problems(robot)
    if problems:
        raise ValueError(f"{robot_path}: " + "; ".join(problems))
    joints = []
    for j in robot["joints"]:
        lim = j.get("limit_deg")
        joints.append(JointCfg(
            name=j["name"], dxl_id=int(j["id"]), gear=float(j["gear_ratio"]),
            sign=None if j.get("sign") is None else int(j["sign"]),
            q_ref_rad=None if j.get("q_ref_deg") is None else math.radians(float(j["q_ref_deg"])),
            limit_rad=None if lim is None else (math.radians(float(lim[0])), math.radians(float(lim[1])))))
    return Config(robot, motion, tuple(joints), tuple(float(v) for v in robot["home"]["q_rad"]),
                  Path(robot_path), _sha(robot), _sha(motion))


def structural_problems(robot: dict) -> list[str]:
    """Errors that make the file unusable for any tool (also the read-only ones)."""
    out = []
    names = [j.get("name") for j in robot.get("joints", [])]
    if tuple(names) != JOINTS:
        out.append(f"joints must be {list(JOINTS)} (model joint names), got {names}")
    ids = [j.get("id") for j in robot.get("joints", [])]
    if len(set(ids)) != len(ids):
        out.append(f"duplicate servo IDs {ids}")
    for j in robot.get("joints", []):
        if not float(j.get("gear_ratio", 0)) > 0:
            out.append(f"{j.get('name')}: gear_ratio must be > 0")
        if j.get("sign") not in (None, 1, -1):
            out.append(f"{j.get('name')}: sign must be +1, -1 or null")
        lim = j.get("limit_deg")
        if lim is not None and (len(lim) != 2 or not float(lim[0]) < float(lim[1])):
            out.append(f"{j.get('name')}: limit_deg must be [lo, hi] with lo < hi")
    if len(robot.get("home", {}).get("q_rad", [])) != len(JOINTS):
        out.append("home.q_rad must have 6 values")
    if robot.get("kind") not in ("hardware", "fake"):
        out.append("kind must be hardware or fake")
    return out


def motion_blockers(cfg: Config, purpose: str) -> list[str]:
    """Why the calibration hold ("hold") or HOME ("home") must not start with this config. Empty list = allowed.

    "hold" (calibrate / prepare --hold-only) never moves: each joint is held where the user aligned it. It needs the
    reference pose q_ref; an unknown sign is determined in the session (the user turns the joint the way the SIM
    shows and confirms). "home" moves, so every sign, q_ref, range and the HOME confirmations must be in the file.
    The read-only tools (scan, watch) never call this."""
    if purpose not in ("hold", "home", "teleop", "pinch"):
        raise ValueError(purpose)
    moves = purpose != "hold"
    out = []
    for j in cfg.joints:
        if moves and j.sign is None:
            out.append(f"{j.name}: 회전 부호 sign 미확정 (calibrate에서 확인·저장)")
        if j.q_ref_rad is None:
            out.append(f"{j.name}: 기준 자세 각도 q_ref_deg 미확정")
        if moves and j.limit_rad is None:
            out.append(f"{j.name}: 실물 가동 범위 limit_deg 미확정")
        elif j.limit_rad is not None and j.q_ref_rad is not None and not j.limit_rad[0] <= j.q_ref_rad <= j.limit_rad[1]:
            out.append(f"{j.name}: q_ref가 limit_deg 밖")
    if moves:                     # the HOME path itself is confirmed in the session (SIM preview + y)
        for j, q in zip(cfg.joints, cfg.home_q_rad):
            if j.limit_rad is not None and not j.limit_rad[0] <= q <= j.limit_rad[1]:
                out.append(f"{j.name}: HOME {math.degrees(q):.3f} deg가 limit_deg 밖")
    return out


def save_signs(robot_path: Path, signs: dict[str, int]) -> str:
    """Write signs the user confirmed in a calibrate session into robot.yaml (only joints whose sign is null).

    Edits only the `sign: null` field of those joints (flow `{name: ..., sign: null}` or block style), so comments
    and the rest of the file stay as they are. A joint that already has a sign is not touched and is reported."""
    lines = Path(robot_path).read_text(encoding="utf-8").splitlines(keepends=True)
    done, kept = [], []
    for name, s in signs.items():
        if s not in (1, -1):
            raise ValueError(f"{name}: sign {s!r}")
        start = next((n for n, line in enumerate(lines) if re.search(rf"\bname: {name}\b", line)), None)
        if start is None:
            raise ValueError(f"{name} not found in {robot_path}")
        n = start
        while n < len(lines) and "sign:" not in lines[n]:
            n += 1
            if n < len(lines) and lines[n].lstrip().startswith("- "):
                n = len(lines)                      # next joint item reached: this one has no sign field
        if n >= len(lines):
            raise ValueError(f"{name}: no sign field in {robot_path}")
        if "sign: null" in lines[n]:
            lines[n] = lines[n].replace("sign: null", f"sign: {s}", 1)
            done.append(name)
        else:
            kept.append(name)
    Path(robot_path).write_text("".join(lines), encoding="utf-8")
    structural = structural_problems(yaml.safe_load(Path(robot_path).read_text(encoding="utf-8")))
    if structural:
        raise ValueError(f"{robot_path} after saving signs: {structural}")
    return f"saved {done}" + (f"; kept existing different sign for {kept}" if kept else "")


def calibrated_view_ready(cfg: Config) -> bool:
    """watch can show absolute angles only with every sign and q_ref known."""
    return all(j.sign is not None and j.q_ref_rad is not None for j in cfg.joints)
