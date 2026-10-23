"""Computed-torque control for the model's torque motors (ctrl = joint torque in N*m, gear 1).

    tau = M(q) [qdd_d + Kp (q_d - q) + Kd (qd_d - qd)] + qfrc_bias - qfrc_passive + tau_f,   clipped to ctrlrange
    tau_f = frictionloss * tanh(v_cmd / v_eps),   v_cmd = qd_d + (Kp / Kd)(q_d - q)

tau_f compensates the model's dry friction (dof_frictionloss, 0.01 N*m): the wrist
inertias are so small that M*Kp*error stays far below that friction and the
wrist would otherwise stick (measured in scripts/measure_hold.py).

Each physics step is mj_step1 -> control from the state at time t -> mj_step2, so
M, qfrc_bias (gravity + Coriolis) and qfrc_passive (joint damping) belong to the
same state the torque is applied at. Target angles are never written to ctrl.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .collision import robot_contacts
from .model import RobotIndex


class ComputedTorque:
    def __init__(self, m: mujoco.MjModel, idx: RobotIndex, ccfg: dict):
        self.m, self.idx = m, idx
        w = 2 * np.pi * ccfg["omega_hz"]
        self.kp, self.kd = w * w, 2 * ccfg["zeta"] * w
        self.limit = float(ccfg["torque_limit_nm"])
        self._M = np.zeros((m.nv, m.nv))
        self.f_loss = m.dof_frictionloss[idx.dof].copy() if ccfg.get("friction_comp", True) else np.zeros(6)
        self.v_eps = float(ccfg.get("friction_v_eps_rad_s", 0.005))

    def torque(self, d: mujoco.MjData, q_d, qd_d, qdd_d) -> tuple[np.ndarray, bool]:
        """Valid only after mj_step1 (or mj_forward) on d. Returns (clipped tau, saturated)."""
        i = self.idx.dof
        mujoco.mj_fullM(self.m, d, self._M)  # MuJoCo 3.15 signature (m, d, dst)
        q, qd = d.qpos[self.idx.qpos], d.qvel[i]
        acc = qdd_d + self.kp * (q_d - q) + self.kd * (qd_d - qd)
        v_cmd = qd_d + (self.kp / self.kd) * (q_d - q)
        tau = (self._M[np.ix_(i, i)] @ acc + d.qfrc_bias[i] - d.qfrc_passive[i]
               + self.f_loss * np.tanh(v_cmd / self.v_eps))
        lo, hi = self.idx.ctrl_lo, self.idx.ctrl_hi
        return np.clip(tau, lo, hi), bool(np.any(tau < lo) or np.any(tau > hi))


def controlled_step(m, d, idx: RobotIndex, ctrl: ComputedTorque, q_d, qd_d, qdd_d) -> tuple[np.ndarray, bool]:
    mujoco.mj_step1(m, d)
    tau, sat = ctrl.torque(d, q_d, qd_d, qdd_d)
    d.ctrl[idx.act] = tau
    mujoco.mj_step2(m, d)
    return tau, sat


@dataclass
class ExecMetrics:
    steps: int = 0
    max_q_err_rad: float = 0.0
    sum_sq_q_err: float = 0.0
    max_tcp_err_m: float = 0.0
    final_tcp_err_m: float = float("nan")
    final_qd_max_rad_s: float = float("nan")
    contact_steps: int = 0
    first_contact: tuple = ()
    saturation_steps: int = 0
    max_abs_tau_nm: float = 0.0
    qd_peak_rad_s: list = field(default_factory=lambda: [0.0] * 6)   # observed max |qvel| per joint (MuJoCo)

    @property
    def rms_q_err_rad(self) -> float:
        return float(np.sqrt(self.sum_sq_q_err / max(1, self.steps * 6)))

    def to_dict(self) -> dict:
        out = {k: v for k, v in self.__dict__.items() if k != "sum_sq_q_err"}
        out["rms_q_err_rad"] = self.rms_q_err_rad
        return out


@dataclass
class TrajectoryExecutor:
    """Tracks a JointTrajectory from sim time t0; same class for the live run and the dry rollout."""

    m: mujoco.MjModel
    idx: RobotIndex
    traj: object            # trajectory.JointTrajectory
    ccfg: dict
    t0: float
    status: str = "RUNNING"  # RUNNING | DONE | ABORTED
    reason: str = ""
    metrics: ExecMetrics = field(default_factory=ExecMetrics)
    trace: list = field(default_factory=list)      # executed TCP positions (m), every trace_every steps
    trace_every: int = 10

    def __post_init__(self):
        self.ctrl = ComputedTorque(self.m, self.idx, self.ccfg)
        self._sat_run = 0
        self._settle_until = self.traj.duration + self.ccfg["settle_s"]
        self._timeout = self._settle_until + 2.0

    def step(self, d: mujoco.MjData) -> None:
        if self.status != "RUNNING":
            raise RuntimeError("executor finished")
        c, met = self.ccfg, self.metrics
        mujoco.mj_step1(self.m, d)
        t = d.time - self.t0
        q_d, qd_d, qdd_d = self.traj.sample(t)
        q = d.qpos[self.idx.qpos]
        q_err = float(np.max(np.abs(q_d - q)))
        tcp_err = float(np.linalg.norm(d.site_xpos[self.idx.tcp_site] - self.traj.tcp_at(t)))
        contacts = robot_contacts(self.m, d)
        tau, sat = self.ctrl.torque(d, q_d, qd_d, qdd_d)
        d.ctrl[self.idx.act] = tau
        mujoco.mj_step2(self.m, d)

        met.steps += 1
        if met.steps % self.trace_every == 1:
            self.trace.append(d.site_xpos[self.idx.tcp_site].copy())   # TCP at time t (pre-integration)
        met.max_q_err_rad = max(met.max_q_err_rad, q_err)
        met.sum_sq_q_err += float(np.sum((q_d - q) ** 2))
        met.max_tcp_err_m = max(met.max_tcp_err_m, tcp_err)
        met.max_abs_tau_nm = max(met.max_abs_tau_nm, float(np.max(np.abs(tau))))
        met.qd_peak_rad_s = np.maximum(met.qd_peak_rad_s, np.abs(d.qvel[self.idx.dof])).tolist()
        self._sat_run = self._sat_run + 1 if sat else 0
        met.saturation_steps += int(sat)
        if contacts:
            met.contact_steps += 1
            met.first_contact = met.first_contact or (round(t, 3), contacts[0])
        dt = self.m.opt.timestep
        if contacts:
            return self._abort(f"contact {contacts[0][0]}-{contacts[0][1]} at t={t:.3f}s")
        if q_err > c["abort_q_err_rad"]:
            return self._abort(f"joint error {q_err:.4f} rad > {c['abort_q_err_rad']} at t={t:.3f}s")
        if tcp_err > c["abort_tcp_err_m"]:
            return self._abort(f"TCP error {tcp_err*1000:.1f} mm > {c['abort_tcp_err_m']*1000:.0f} mm at t={t:.3f}s")
        if self._sat_run * dt > c["abort_sat_run_s"]:
            return self._abort(f"torque saturated > {c['abort_sat_run_s']} s at t={t:.3f}s")
        if t >= self._settle_until:
            qd_max = float(np.max(np.abs(d.qvel[self.idx.dof])))
            met.final_tcp_err_m, met.final_qd_max_rad_s = tcp_err, qd_max
            if tcp_err <= c["final_tcp_err_m"] and qd_max <= c["final_qd_rad_s"]:
                self.status = "DONE"
            elif t >= self._timeout:
                self._abort(f"did not settle: TCP {tcp_err*1000:.2f} mm, |qd| {qd_max:.4f} rad/s")

    def _abort(self, reason: str) -> None:
        self.status, self.reason = "ABORTED", reason

    def progress(self, d: mujoco.MjData) -> float:
        return float(np.clip((d.time - self.t0) / self.traj.duration, 0.0, 1.0))
