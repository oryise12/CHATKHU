"""Workspace paths, config loading and model indexing."""
from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f"Revolute_{i}" for i in range(1, 7))
ACTUATORS = tuple(f"Revolute_{i}_mot" for i in range(1, 7))
TCP_SITE = "tcp"


def load_config(path: str | Path | None = None) -> dict:
    path = Path(path) if path else ROOT / "configs" / "demo.json"
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def config_sha(cfg: dict) -> str:
    return hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()


def with_overrides(cfg: dict, **sections) -> dict:
    """Deep copy cfg and update nested sections: with_overrides(cfg, target={"work_height_m": 0.08})."""
    out = copy.deepcopy(cfg)
    for name, values in sections.items():
        out.setdefault(name, {}).update(values)
    return out


def joint_speed_max_rad_s(cfg: dict) -> float:
    """The single joint-speed cap (limits.joint_speed_max_deg_s, deg/s) in rad/s. Validation checks against it."""
    return float(np.radians(cfg["limits"]["joint_speed_max_deg_s"]))


def joint_speed_plan_rad_s(cfg: dict) -> float:
    """Planning target for the fastest joint = cap x limits.joint_speed_plan_ratio (< 1 leaves room for the
    controller's tracking overshoot, measured +0.04 % when planned exactly at the cap)."""
    return joint_speed_max_rad_s(cfg) * float(cfg["limits"].get("joint_speed_plan_ratio", 1.0))


def file_sha(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass
class RobotIndex:
    """Name -> index lookups resolved once from the compiled model."""

    qpos: np.ndarray        # (6,) qpos addresses in Revolute_1..6 order
    dof: np.ndarray         # (6,) dof addresses
    act: np.ndarray         # (6,) actuator ids (torque motors, gear 1)
    lo: np.ndarray
    hi: np.ndarray
    tcp_site: int
    floor_geom: int               # robot support surface: desk_top (or the floor plane without a desk)
    robot_geoms: np.ndarray       # collision geoms (contype/conaffinity != 0) on moving bodies
    base_geom: int                # collision geom of the fixed base mesh (world body)
    ctrl_lo: np.ndarray
    ctrl_hi: np.ndarray

    @classmethod
    def from_model(cls, m: mujoco.MjModel) -> "RobotIndex":
        jid = [m.joint(n).id for n in JOINTS]
        act = np.array([m.actuator(n).id for n in ACTUATORS])
        if not np.all(m.actuator_gear[act, 0] == 1.0):
            raise ValueError("expected gear=1 torque motors")
        collide = [g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
        # the surface the robot stands on (z = 0): the desk top if the scene has a desk, else the floor plane
        names = {m.geom(g).name for g in range(m.ngeom)}
        floor = m.geom("desk_top").id if "desk_top" in names else m.geom("floor").id
        moving = lambda g: m.body_weldid[m.geom_bodyid[g]] != 0          # noqa: E731 (desk/world are static)
        robot = [g for g in collide if moving(g)]
        base = [g for g in collide if m.geom_bodyid[g] == 0 and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
        if len(base) != 1:
            raise ValueError(f"expected one base collision geom, got {base}")
        return cls(
            qpos=np.array([m.jnt_qposadr[j] for j in jid]),
            dof=np.array([m.jnt_dofadr[j] for j in jid]),
            act=act,
            lo=m.jnt_range[jid, 0].copy(),
            hi=m.jnt_range[jid, 1].copy(),
            tcp_site=m.site(TCP_SITE).id,
            floor_geom=floor,
            robot_geoms=np.array(robot),
            base_geom=base[0],
            ctrl_lo=m.actuator_ctrlrange[act, 0].copy(),
            ctrl_hi=m.actuator_ctrlrange[act, 1].copy(),
        )


def load_scene(cfg: dict) -> tuple[mujoco.MjModel, RobotIndex, str]:
    """Load the generated scene; returns (model, index, model_sha of the XML file)."""
    path = ROOT / cfg["model"]["generated_scene"]
    if not path.exists():
        raise FileNotFoundError(f"{path} missing: run `python -m pinch_sim.build_scene` first")
    m = mujoco.MjModel.from_xml_path(str(path))
    return m, RobotIndex.from_model(m), file_sha(path)


def ready_q(m: mujoco.MjModel) -> np.ndarray:
    return m.key("ready").qpos.copy()
