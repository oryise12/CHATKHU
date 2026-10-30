"""Pinch trajectories from the simulator: parse, check (never trust the sender), and the streaming commands.

The simulator (pinch_sim --real-arm) validates a joint trajectory with its own checks and sends it here as uniform
samples (dt, q, qd; rad, rad/s, t = k*dt). Before anything moves, check() re-checks it against this program's config
and model. While it runs, the PC owns the trajectory (PLAN §7.5) and the servo's velocity-based profile only fills
in the next lead_s:

  Goal  per joint: the value of that joint furthest along its motion within [t, t + lead] (= q(t + lead) unless the
        joint turns around inside the window): always a value the checked trajectory takes.
  PV    the speed the trajectory needs now plus a small correction of the servo's internal target towards q(t),
        capped by 1.1 x the fastest speed of the window (+0.5 deg/s) and pv_raw_max; never 0 (0 = unlimited).
  PA    fixed (pinch.profile_acceleration_raw).

lead_s >= 2 / read_hz + v_cap / (2 a): the goal stays ahead by more than the braking distance plus two missed reads,
so the servo never starts braking early in normal running, and if the PC or the USB link stops (Watchdog 0) the arm
goes on to the last goals - at most lead_s of the trajectory ahead - and stops there by itself.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .units import pa_unit_rad_s2, pv_unit_rad_s

N_JOINTS = 6


@dataclass(frozen=True)
class Trajectory:
    traj_id: str
    dt: float
    q: np.ndarray                       # (N, 6) rad
    qd: np.ndarray                      # (N, 6) rad/s
    meta: dict = field(default_factory=dict)   # report_id, path_fp, robot_sha, model_sha, move (outbound / return ..)

    @property
    def n(self) -> int:
        return len(self.q)

    @property
    def duration(self) -> float:
        return (self.n - 1) * self.dt

    def at(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """(q, qd) at time t (linear between samples, clamped to [0, T])."""
        x = min(max(t, 0.0), self.duration) / self.dt
        k = min(int(x), self.n - 2) if self.n > 1 else 0
        w = x - k
        if self.n == 1:
            return self.q[0].copy(), self.qd[0].copy()
        return self.q[k] * (1 - w) + self.q[k + 1] * w, self.qd[k] * (1 - w) + self.qd[k + 1] * w

    def window(self, t0: float, t1: float) -> np.ndarray:
        """Samples of q in [t0, t1] including the interpolated ends (rows)."""
        t0, t1 = max(t0, 0.0), min(t1, self.duration)
        a, b = int(math.ceil(t0 / self.dt - 1e-9)), int(math.floor(t1 / self.dt + 1e-9))
        rows = [self.at(t0)[0]] + [self.q[k] for k in range(a, b + 1)] + [self.at(t1)[0]]
        return np.array(rows)

    def window_speed(self, t0: float, t1: float) -> np.ndarray:
        """max |qd| per joint over [t0, t1] (clamped to the trajectory; past its end: the last sample)."""
        a = min(max(int(math.floor(t0 / self.dt)), 0), self.n - 1)
        b = min(max(int(math.ceil(t1 / self.dt)), a), self.n - 1)
        return np.abs(self.qd[a:b + 1]).max(axis=0)

    def scaled(self, s: float) -> "Trajectory":
        """The same path, s times as fast (s <= 1 slows it down): t -> t / s, qd -> qd * s."""
        return Trajectory(self.traj_id, self.dt / s, self.q, self.qd * s, dict(self.meta, speed_scale=s))


def parse(msg: dict, max_samples: int) -> Trajectory:
    """The simulator's execute message -> Trajectory. ValueError: malformed (the reason is sent back)."""
    try:
        traj_id = str(msg["traj_id"])
        dt = float(msg["dt"])
        q = np.asarray(msg["q"], dtype=float)
        qd = np.asarray(msg["qd"], dtype=float)
        meta = dict(msg.get("meta") or {})
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"execute message malformed: {e}") from None
    if q.ndim != 2 or q.shape[1] != N_JOINTS or qd.shape != q.shape:
        raise ValueError(f"q/qd must be (N, {N_JOINTS}), got {q.shape} / {qd.shape}")
    if not 2 <= len(q) <= max_samples:
        raise ValueError(f"{len(q)} samples (2..{max_samples})")
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd)) and math.isfinite(dt)):
        raise ValueError("non-finite values")
    return Trajectory(traj_id, dt, q, qd, meta)


@dataclass(frozen=True)
class Acceptance:
    """What this program accepts (config/robot.yaml + motion.yaml pinch:). Also sent to the simulator in hello."""
    lo: np.ndarray                      # rad, robot.yaml limit_deg
    hi: np.ndarray
    margin_rad: float                   # every sample inside [lo + margin, hi - margin]
    v_cap_rad_s: float
    pv_raw_max: int
    pv_unit: np.ndarray                 # rad/s of one PV raw unit per joint (gear)
    qdd_max_rad_s2: float
    a_stream_rad_s2: np.ndarray         # per joint, from the fixed streaming PA
    dt_range_s: tuple[float, float]
    max_duration_s: float
    start_tol_rad: float
    check_step_rad: float
    robot_sha: str
    model_sha: str

    @classmethod
    def from_config(cls, cfg, model_sha: str) -> "Acceptance":
        p = cfg.motion["pinch"]
        lo = np.array([j.limit_rad[0] for j in cfg.joints])
        hi = np.array([j.limit_rad[1] for j in cfg.joints])
        return cls(lo, hi, math.radians(p["limit_margin_deg"]), math.radians(cfg.m("limits", "v_cap_deg_s")),
                   int(cfg.m("limits", "pv_raw_max")), np.array([pv_unit_rad_s(j.gear) for j in cfg.joints]),
                   math.radians(p["qdd_max_deg_s2"]),
                   np.array([p["profile_acceleration_raw"] * pa_unit_rad_s2(j.gear) for j in cfg.joints]),
                   tuple(p["dt_range_s"]), float(p["max_duration_s"]), math.radians(p["start_tol_deg"]),
                   math.radians(p["check_step_deg"]), cfg.robot_sha, model_sha)

    def hello(self) -> dict:
        return {"limits_deg": np.degrees(np.stack([self.lo, self.hi], axis=1)).round(6).tolist(),
                "margin_deg": round(math.degrees(self.margin_rad), 6),
                "v_cap_deg_s": round(math.degrees(self.v_cap_rad_s), 6),
                "qdd_max_deg_s2": round(math.degrees(self.qdd_max_rad_s2), 6),
                "dt_range_s": list(self.dt_range_s), "max_duration_s": self.max_duration_s,
                "start_tol_deg": round(math.degrees(self.start_tol_rad), 6),
                "robot_sha": self.robot_sha, "model_sha": self.model_sha}


def check(traj: Trajectory, acc: Acceptance, q_meas, path_checker=None) -> list[str]:
    """Every reason not to run this trajectory from the measured pose q_meas (empty: run it)."""
    out = []
    m = traj.meta
    if m.get("robot_sha") != acc.robot_sha:
        out.append(f"robot.yaml differs (sim {str(m.get('robot_sha'))[:12]} != {acc.robot_sha[:12]})")
    if m.get("model_sha") != acc.model_sha:
        out.append(f"model differs (sim {str(m.get('model_sha'))[:12]} != {acc.model_sha[:12]})")
    lo_dt, hi_dt = acc.dt_range_s
    if not lo_dt <= traj.dt <= hi_dt:
        out.append(f"sample spacing {traj.dt * 1000:.1f} ms outside [{lo_dt * 1000:.0f}, {hi_dt * 1000:.0f}] ms")
    if traj.duration > acc.max_duration_s:
        out.append(f"duration {traj.duration:.1f} s > {acc.max_duration_s:.0f} s")
    if out:
        return out
    q, qd, dt = traj.q, traj.qd, traj.dt
    d0 = np.abs(q[0] - np.asarray(q_meas, float))
    if d0.max() > acc.start_tol_rad:
        out.append(f"start differs from the real pose by {math.degrees(d0.max()):.2f} deg (joint {int(d0.argmax()) + 1}, "
                   f"limit {math.degrees(acc.start_tol_rad):.2f})")
    rest = math.radians(0.5)
    if np.abs(qd[0]).max() > rest or np.abs(qd[-1]).max() > rest:
        out.append("does not start and end at rest")
    lo, hi = acc.lo + acc.margin_rad, acc.hi - acc.margin_rad
    bad = np.where((q < lo) | (q > hi))
    if bad[0].size:
        k, j = int(bad[0][0]), int(bad[1][0])
        out.append(f"joint {j + 1} {math.degrees(q[k, j]):.2f} deg at t={k * dt:.2f} s outside "
                   f"[{math.degrees(lo[j]):.1f}, {math.degrees(hi[j]):.1f}] (range - margin)")
    brake = q + np.sign(qd) * qd * qd / (2 * acc.a_stream_rad_s2)
    bad = np.where((brake < acc.lo) | (brake > acc.hi))
    if bad[0].size:
        k, j = int(bad[0][0]), int(bad[1][0])
        out.append(f"joint {j + 1}: braking point at t={k * dt:.2f} s outside the range")
    v = np.abs(qd)
    if v.max() > acc.v_cap_rad_s:
        k, j = np.unravel_index(int(v.argmax()), v.shape)
        out.append(f"joint {j + 1} speed {math.degrees(v[k, j]):.2f} deg/s > {math.degrees(acc.v_cap_rad_s):.2f}")
    pv_need = np.ceil(v / acc.pv_unit - 1e-9)
    if pv_need.max() > acc.pv_raw_max:
        out.append(f"needs PV {int(pv_need.max())} > {acc.pv_raw_max}")
    acc_s = np.abs(np.diff(qd, axis=0)) / dt
    if acc_s.size and acc_s.max() > acc.qdd_max_rad_s2:
        k, j = np.unravel_index(int(acc_s.argmax()), acc_s.shape)
        out.append(f"joint {j + 1} acceleration {math.degrees(acc_s[k, j]):.1f} deg/s^2 > "
                   f"{math.degrees(acc.qdd_max_rad_s2):.0f}")
    step = np.abs(np.diff(q, axis=0))
    if step.size and step.max() > acc.v_cap_rad_s * dt * 1.05:
        out.append(f"jump of {math.degrees(step.max()):.3f} deg between samples")
    fd = np.diff(q, axis=0) / dt
    mid = 0.5 * (qd[1:] + qd[:-1])
    if fd.size and np.abs(fd - mid).max() > math.radians(0.2):
        out.append(f"q and qd disagree by {math.degrees(np.abs(fd - mid).max()):.2f} deg/s")
    if out or path_checker is None:
        return out
    last = q[0]
    for k in range(q.shape[0]):
        if k in (0, q.shape[0] - 1) or np.abs(q[k] - last).max() >= acc.check_step_rad:
            problems = path_checker(q[k])
            if problems:
                return [f"model check at t={k * dt:.2f} s: {problems[0]}"]
            last = q[k]
    return []


@dataclass(frozen=True)
class StreamCommand:
    q_goal: np.ndarray                  # rad, per joint
    pv: np.ndarray                      # raw, int, >= 1
    q_ref: np.ndarray                   # q(t) the command was made for


def stream_command(traj: Trajectory, t: float, q_internal, acc: Acceptance, p: dict, tick_s: float) -> StreamCommand:
    """Goal and PV for trajectory time t (already speed-scaled). q_internal: the servos' Position Trajectory (rad)."""
    lead = float(p["lead_s"])
    T = traj.duration
    win = traj.window(t, t + lead)
    q_now, _ = traj.at(t)
    q_end_win = win[-1]
    direction = np.sign(q_end_win - q_now)
    far = np.where(direction >= 0, win.max(axis=0), win.min(axis=0))   # furthest along the motion (turns: stop there)
    q_goal = np.where(direction == 0, q_end_win, far)
    v_need = traj.window_speed(t, t + tick_s)
    err = (q_now - np.asarray(q_internal, float)) * np.where(direction == 0, 1.0, direction)
    k, vmax = float(p["feedback_gain_per_s"]), math.radians(p["feedback_max_deg_s"])
    v = np.maximum(0.0, v_need + np.clip(k * err, -vmax, vmax))
    cap = 1.1 * traj.window_speed(t, t + lead) + math.radians(0.5)
    pv = np.ceil(v / acc.pv_unit - 1e-9)
    pv = np.minimum(pv, np.minimum(acc.pv_raw_max, np.ceil(cap / acc.pv_unit - 1e-9)))
    if t >= T - lead:
        pv = np.maximum(pv, int(p["end_pv_min_raw"]))
    pv = np.clip(pv, 1, acc.pv_raw_max).astype(int)
    return StreamCommand(q_goal, pv, q_now)
