"""MuJoCo FK/Jacobian on a private scratch MjData, plus a limit-aware DLS IK.

The IK task is 5-D: TCP position (3) + tool axis direction (2). Roll about the
tool axis is free and used only by a weak null-space pull towards a reference q.
Nothing here touches the live simulation data.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .frames import Pose, angle_deg, unit
from .model import RobotIndex


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    pos_err_m: float
    axis_err_deg: float
    iters: int
    reason: str = ""


class Kinematics:
    def __init__(self, m: mujoco.MjModel, idx: RobotIndex, ik_cfg: dict | None = None):
        self.m, self.idx = m, idx
        self.d = mujoco.MjData(m)
        self.ik_cfg = ik_cfg or {}
        self._jacp = np.zeros((3, m.nv))
        self._jacr = np.zeros((3, m.nv))

    def _set(self, q) -> None:
        self.d.qpos[self.idx.qpos] = q
        mujoco.mj_kinematics(self.m, self.d)

    def fk(self, q) -> Pose:
        self._set(q)
        s = self.idx.tcp_site
        return Pose(self.d.site_xpos[s].copy(), self.d.site_xmat[s].reshape(3, 3).copy())

    def jacobian(self, q) -> tuple[np.ndarray, np.ndarray]:
        """(jacp 3x6, jacr 3x6) of the TCP site; requires kinematics + com positions."""
        self._set(q)
        mujoco.mj_comPos(self.m, self.d)
        mujoco.mj_jacSite(self.m, self.d, self._jacp, self._jacr, self.idx.tcp_site)
        return self._jacp[:, self.idx.dof].copy(), self._jacr[:, self.idx.dof].copy()

    def sigma_min(self, q) -> float:
        jacp, _ = self.jacobian(q)
        return float(np.linalg.svd(jacp, compute_uv=False)[-1])

    def solve_ik(self, p_des, axis_des, q_seed, q_ref=None) -> IKResult:
        c = self.ik_cfg
        lam, clip = c.get("damping", 1e-3), c.get("step_clip_rad", 0.1)
        w_ax, k_ns = c.get("axis_weight", 0.3), c.get("nullspace_gain", 0.05)
        tol_p, tol_a = c.get("pos_tol_m", 1e-4), c.get("axis_tol_deg", 0.1)
        p_des, a_des = np.asarray(p_des, float), unit(axis_des)
        q = np.clip(np.asarray(q_seed, float).copy(), self.idx.lo, self.idx.hi)
        q_ref = q.copy() if q_ref is None else np.asarray(q_ref, float)
        for it in range(1, c.get("max_iter", 200) + 1):
            pose = self.fk(q)
            a = pose.tool_axis
            e_p = p_des - pose.p_m
            e_a = np.cross(a, a_des)  # rotation (perpendicular to a) that turns a towards a_des
            if np.linalg.norm(e_p) <= tol_p and angle_deg(a, a_des) <= tol_a:
                return IKResult(q, True, float(np.linalg.norm(e_p)), angle_deg(a, a_des), it)
            jacp, jacr = self.jacobian(q)
            P = np.eye(3) - np.outer(a, a)
            J = np.vstack([jacp, w_ax * (P @ jacr)])
            J[:, self.idx.hi - self.idx.lo < 1e-12] = 0.0   # locked joints (lo == hi) cannot help the task
            e = np.concatenate([e_p, w_ax * e_a])
            # damped inverse on the task, but an EXACT null-space projector (rank-5 task): a damped
            # projector leaks the secondary pull into the task and stalls convergence near ~1e-5
            U, s, Vt = np.linalg.svd(J)
            r = int(np.sum(s > 1e-6 * s[0]))
            # damping only far from the solution: near it, a weakly coupled direction (e.g. the gripper tip
            # 1.8 mm off the J6 axis, sigma ~ 2e-3) would otherwise converge at a rate of sigma^2 / lam per step
            lam_k = lam if np.linalg.norm(e) > c.get("damping_off_below", 1e-3) else c.get("damping_near", 1e-10)
            dq_task = Vt[:r].T @ ((s[:r] / (s[:r] ** 2 + lam_k)) * (U[:, :r].T @ e))
            N = np.eye(6) - Vt[:r].T @ Vt[:r]
            dq = dq_task + N @ (k_ns * (q_ref - q))
            q = np.clip(q + np.clip(dq, -clip, clip), self.idx.lo, self.idx.hi)
        pose = self.fk(q)
        pe, ae = float(np.linalg.norm(p_des - pose.p_m)), angle_deg(pose.tool_axis, a_des)
        return IKResult(q, False, pe, ae, c.get("max_iter", 200), "not converged")

    def within_limits(self, q, margin: float) -> bool:
        return bool(np.all(q >= self.idx.lo + margin) and np.all(q <= self.idx.hi - margin))
