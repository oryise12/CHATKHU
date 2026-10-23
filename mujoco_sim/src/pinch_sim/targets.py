"""Seeded floor targets and the TCP goal/approach points derived from them.

The floor point is the marker on the floor (z = 0). The TCP never goes there:
it stops work_height_m above it with the tool pointing straight down, after
passing through approach_height_m. Redraws stay inside the same r/theta
distribution; nothing is widened or clipped to make a draw succeed.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from .collision import CollisionChecker
from .frames import DOWN, polar_to_floor
from .kinematics import Kinematics


@dataclass(frozen=True)
class TargetSpec:
    r_m: float
    theta_rad: float
    floor_point_m: np.ndarray
    tcp_goal_m: np.ndarray
    approach_m: np.ndarray
    tool_axis: np.ndarray = field(default_factory=lambda: DOWN.copy())
    seed: int | None = None
    trial: int | None = None
    draw: int | None = None

    def summary(self) -> dict:
        out = {k: (v.round(5).tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(self).items()}
        out["theta_deg"] = round(float(np.degrees(self.theta_rad)), 3)
        return out


def make_target(r_m: float, theta_rad: float, tcfg: dict, *, seed=None, trial=None, draw=None,
                allow_out_of_range: bool = False, theta_range_deg=None) -> TargetSpec:
    """theta_range_deg=(lo, hi) replaces the default +-theta_max_deg check (used for the New-key bands)."""
    if not allow_out_of_range:
        if not (tcfg["r_min_m"] - 1e-12 <= r_m <= tcfg["r_max_m"] + 1e-12):
            raise ValueError(f"r={r_m} outside [{tcfg['r_min_m']}, {tcfg['r_max_m']}] m")
        lo, hi = theta_range_deg or (-tcfg["theta_max_deg"], tcfg["theta_max_deg"])
        if not (lo - 1e-9 <= np.degrees(theta_rad) <= hi + 1e-9):
            raise ValueError(f"theta={np.degrees(theta_rad):.2f} deg outside [{lo}, {hi}] deg")
    floor = polar_to_floor(r_m, theta_rad)
    return TargetSpec(float(r_m), float(theta_rad), floor,
                      floor + [0.0, 0.0, tcfg["work_height_m"]],
                      floor + [0.0, 0.0, tcfg["approach_height_m"]], DOWN.copy(), seed, trial, draw)


def draw_polar(rng: np.random.Generator, tcfg: dict, theta_band_deg=None) -> tuple[float, float]:
    """Radius from the configured range; theta uniform in theta_band_deg (lo, hi) or +-theta_max_deg."""
    r0, r1 = tcfg["r_min_m"], tcfg["r_max_m"]
    if tcfg.get("r_dist", "area_uniform") == "area_uniform":
        r = float(np.sqrt(rng.uniform(r0 * r0, r1 * r1)))
    else:
        r = float(rng.uniform(r0, r1))
    lo, hi = theta_band_deg or (-tcfg["theta_max_deg"], tcfg["theta_max_deg"])
    th = float(np.radians(rng.uniform(lo, hi)))
    return r, th


def precheck(target: TargetSpec, kin: Kinematics, col: CollisionChecker, q_seed, cfg: dict) -> tuple[bool, str]:
    """Approach and work poses: IK converges, joints inside margin, no contact, floor clearance.

    This is a screening filter for target selection, not path validation.
    """
    margin = cfg["limits"]["joint_margin_rad"]
    q = np.asarray(q_seed, float)
    for name, p in (("approach", target.approach_m), ("work", target.tcp_goal_m)):
        ik = kin.solve_ik(p, target.tool_axis, q, q_ref=q)
        if not ik.ok:
            return False, f"ik_not_converged:{name}"
        if not kin.within_limits(ik.q, margin):
            return False, f"joint_limit:{name}"
        c = col.check(ik.q)
        if c.in_contact:
            return False, f"collision:{name}:{c.contacts[0][0]}-{c.contacts[0][1]}"
        if c.floor_clearance_m < cfg["validation"]["floor_clearance_m"]:
            return False, f"floor_clearance:{name}:{c.floor_geom_body}"
        q = ik.q
    return True, "ok"


@dataclass
class DrawResult:
    target: TargetSpec | None
    attempts: list          # [(draw, r, theta_deg, reason)]
    message: str


def sample_target(seed: int, trial: int, cfg: dict, kin: Kinematics, col: CollisionChecker, q_seed,
                  theta_band_deg=None) -> DrawResult:
    """Target number `trial` of the sequence for `seed`; reproducible independent of earlier trials.

    With theta_band_deg=(lo, hi) every draw, including redraws, is taken inside that band only.
    """
    tcfg = cfg["target"]
    rng = np.random.default_rng([int(seed), int(trial)])
    attempts = []
    for draw in range(tcfg["max_draws"]):
        r, th = draw_polar(rng, tcfg, theta_band_deg)
        t = make_target(r, th, tcfg, seed=seed, trial=trial, draw=draw, theta_range_deg=theta_band_deg)
        ok, reason = precheck(t, kin, col, q_seed, cfg)
        attempts.append((draw, round(r, 5), round(float(np.degrees(th)), 3), reason))
        if ok:
            band = f" band [{theta_band_deg[0]:+.0f},{theta_band_deg[1]:+.0f}] deg" if theta_band_deg else ""
            return DrawResult(t, attempts, f"target seed={seed} trial={trial}{band} accepted at draw {draw}")
    counts: dict = {}
    for a in attempts:
        key = a[3].split(":")[0]
        counts[key] = counts.get(key, 0) + 1
    band = f", 각도 구간 [{theta_band_deg[0]:+.0f}, {theta_band_deg[1]:+.0f}]°" if theta_band_deg else ""
    return DrawResult(None, attempts,
                      f"목표 생성 실패: seed={seed}, trial={trial}{band}, {tcfg['max_draws']}회 시도, 실패 사유 {counts}")
