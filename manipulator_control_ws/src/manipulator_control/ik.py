"""Inverse kinematics of the tcp site (position + orientation) on the display model's measured arm.

Damped least squares on MuJoCo's site Jacobian, seeded with the previous solution, joints clipped to the confirmed
range at every iteration. The result says whether the target was reached within tolerance; an unreachable target
(outside the range or the workspace) returns the closest pose found with ok = False, and the caller does not send
it to the servos.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from .model import ArmIndex


@dataclass(frozen=True)
class IKResult:
    q: np.ndarray
    ok: bool
    pos_err_m: float
    rot_err_deg: float
    iters: int


def rot_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Rotation vector (world frame) that turns `current` into `target` (both 3x3)."""
    r = target @ current.T
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, r.reshape(-1))
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


class IKSolver:
    def __init__(self, m: mujoco.MjModel, arm: ArmIndex, *, max_iters: int = 100, pos_tol_m: float = 5e-4,
                 rot_tol_deg: float = 0.3, rot_weight_m: float = 0.1, damping: float = 0.01, max_step_rad: float = 0.2):
        self.m, self.d, self.arm = m, mujoco.MjData(m), arm
        self.dofs = np.array([m.jnt_dofadr[m.jnt_qposadr.tolist().index(a)] for a in arm.qpos])
        self.max_iters, self.pos_tol, self.rot_tol = max_iters, pos_tol_m, math.radians(rot_tol_deg)
        self.w, self.lam, self.max_step = rot_weight_m, damping, max_step_rad

    def fk(self, q) -> tuple[np.ndarray, np.ndarray]:
        self.d.qpos[self.arm.qpos] = q
        mujoco.mj_kinematics(self.m, self.d)
        s = self.arm.tcp_site
        return self.d.site_xpos[s].copy(), self.d.site_xmat[s].reshape(3, 3).copy()

    def solve(self, target_pos, target_mat, q_seed, lo, hi) -> IKResult:
        m, d, s = self.m, self.d, self.arm.tcp_site
        q = np.clip(np.asarray(q_seed, float), lo, hi)
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        best = None
        for it in range(1, self.max_iters + 1):
            d.qpos[self.arm.qpos] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            e_p = np.asarray(target_pos) - d.site_xpos[s]
            e_r = rot_error(np.asarray(target_mat), d.site_xmat[s].reshape(3, 3))
            pe, re = float(np.linalg.norm(e_p)), float(np.linalg.norm(e_r))
            if best is None or pe + self.w * re < best[1] + self.w * best[2]:
                best = (q.copy(), pe, re, it)
            if pe <= self.pos_tol and re <= self.rot_tol:
                return IKResult(q.copy(), True, pe, math.degrees(re), it)
            mujoco.mj_jacSite(m, d, jacp, jacr, s)
            J = np.vstack([jacp[:, self.dofs], self.w * jacr[:, self.dofs]])
            e = np.concatenate([e_p, self.w * e_r])
            free = np.ones(len(q), bool)
            for _ in range(len(q)):                   # joints at a limit that would push past it: take them out
                Jf = J * free
                dq = Jf.T @ np.linalg.solve(Jf @ Jf.T + self.lam ** 2 * np.eye(6), e)
                blocked = free & (((q <= lo + 1e-9) & (dq < 0)) | ((q >= hi - 1e-9) & (dq > 0)))
                if not blocked.any():
                    break
                free &= ~blocked
            n = np.abs(dq).max()
            if n > self.max_step:
                dq *= self.max_step / n
            q = np.clip(q + dq, lo, hi)
        q, pe, re, it = best
        return IKResult(q, pe <= self.pos_tol and re <= self.rot_tol, pe, math.degrees(re), it)
