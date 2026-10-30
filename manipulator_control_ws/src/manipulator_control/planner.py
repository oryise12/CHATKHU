"""HOME motion owned by the servo's velocity-based profile; the PC plans, sends PV/PA/Goal once, and supervises.

Plan: one common time axis. The axis that needs the longest time at the speed/acceleration caps sets T; every other
axis gets the speed and acceleration that would make it arrive at T as well (velocity ratios kept; a limit is met
by stretching T, never by clipping one axis). The raw values are integers (PV 0.229 rpm, PA 214.577 rev/min^2), so
small motions cannot always arrive exactly together: everything below (arrival times, speeds, accelerations, the
path) is recomputed from the *integer values actually sent*, and the deviation from the ideal common trajectory is
reported as a warning. The servo profile is a trapezoid (or triangle): position and velocity are continuous,
acceleration steps between 0 and +-a at the phase boundaries - it is not jerk-limited.

Profile model (e-manual): v = PV*0.229 rpm, a = PA*214.577 rev/min^2, accel time t1 = v/a (~ 64*PV/PA ms),
triangular when the distance is shorter than v^2/a.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .units import pa_counts_s2, pa_unit_rad_s2, pv_counts_s, pv_unit_rad_s, goal_in_range

START_TOL_DEG = 1.0     # a start pose this far outside the range is accepted (moving back inside)


@dataclass(frozen=True)
class Trapezoid:
    """Displacement profile over |distance| with max speed v and acceleration a (any consistent units)."""
    distance: float          # signed
    v: float
    a: float

    @property
    def D(self) -> float:
        return abs(self.distance)

    @property
    def kind(self) -> str:
        if self.D == 0:
            return "none"
        return "trapezoid" if self.D >= self.v * self.v / self.a else "triangle"

    @property
    def t_acc(self) -> float:
        if self.D == 0:
            return 0.0
        return self.v / self.a if self.kind == "trapezoid" else math.sqrt(self.D / self.a)

    @property
    def v_peak(self) -> float:
        return self.a * self.t_acc

    @property
    def T(self) -> float:
        if self.D == 0:
            return 0.0
        return self.D / self.v + self.t_acc if self.kind == "trapezoid" else 2 * self.t_acc

    def pos(self, t):
        t = np.clip(np.asarray(t, float), 0.0, self.T)
        ta, T, a, vp = self.t_acc, self.T, self.a, self.v_peak
        s = np.where(t < ta, 0.5 * a * t ** 2,
                     np.where(t <= T - ta, 0.5 * a * ta ** 2 + vp * (t - ta), self.D - 0.5 * a * (T - t) ** 2))
        return np.sign(self.distance) * s

    def vel(self, t):
        t = np.asarray(t, float)
        ta, T, a, vp = self.t_acc, self.T, self.a, self.v_peak
        v = np.where(t < 0, 0.0, np.where(t < ta, a * t, np.where(t <= T - ta, vp, np.where(t < T, a * (T - t), 0.0))))
        return np.sign(self.distance) * v


def servo_profile(dp_counts: int, pv: int, pa: int) -> Trapezoid:
    if pv < 1 or pa < 1:
        raise ValueError("PV and PA must be >= 1 (0 means infinite)")
    return Trapezoid(float(dp_counts), pv_counts_s(pv), pa_counts_s2(pa))


@dataclass(frozen=True)
class AxisPlan:
    name: str
    dxl_id: int
    k: float                 # counts per rad (joint)
    sign: int
    q0: float
    q1: float
    p0: int
    goal: int
    pv: int
    pa: int
    profile: Trapezoid       # in counts, from the integer PV/PA actually sent

    @property
    def v_peak_rad_s(self) -> float:
        return self.profile.v_peak / self.k

    @property
    def a_rad_s2(self) -> float:
        return 0.0 if self.profile.D == 0 else self.profile.a / self.k

    def q_at(self, t) -> np.ndarray:
        return self.q0 + self.sign * self.profile.pos(t) / self.k

    def qd_at(self, t) -> np.ndarray:
        return self.sign * self.profile.vel(t) / self.k


@dataclass
class HomePlan:
    axes: list[AxisPlan]
    T_common: float                   # ideal common time (dominant axis)
    t_acc_common: float
    T_max: float                      # latest predicted arrival from the integer values
    arrival_spread_s: float
    sync_dev_rad: dict[str, float]    # max |predicted - ideal common| per joint
    progress_spread: float            # max over t of (max - min) normalized progress among moving axes
    tcp_dev_m: float | None
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    caps: dict = field(default_factory=dict)

    def predict(self, t) -> np.ndarray:
        return np.array([a.q_at(t) for a in self.axes], float)

    def summary(self) -> dict:
        return {
            "T_common_s": round(self.T_common, 3), "t_acc_s": round(self.t_acc_common, 3),
            "T_max_s": round(self.T_max, 3), "arrival_spread_s": round(self.arrival_spread_s, 3),
            "max_sync_dev_deg": round(math.degrees(max(self.sync_dev_rad.values(), default=0.0)), 4),
            "progress_spread": round(self.progress_spread, 4),
            "tcp_dev_mm": None if self.tcp_dev_m is None else round(self.tcp_dev_m * 1000, 2),
            "axes": [{"name": a.name, "id": a.dxl_id, "q0_deg": round(math.degrees(a.q0), 4),
                      "q1_deg": round(math.degrees(a.q1), 4), "p0": a.p0, "goal": a.goal, "dp": a.goal - a.p0,
                      "pv": a.pv, "pa": a.pa, "kind": a.profile.kind, "T_s": round(a.profile.T, 3),
                      "v_peak_deg_s": round(math.degrees(a.v_peak_rad_s), 4),
                      "a_deg_s2": round(math.degrees(a.a_rad_s2), 4)} for a in self.axes],
            "caps": self.caps, "errors": self.errors, "warnings": self.warnings,
        }


def _raw_caps(gear: float, v_lim: float, a_lim: float, pv_raw_max: int) -> tuple[int, int]:
    pv = min(pv_raw_max, math.floor(v_lim / pv_unit_rad_s(gear) + 1e-9))
    pa = math.floor(a_lim / pa_unit_rad_s2(gear) + 1e-9)
    return pv, pa


def plan_home(joints, cal, p0: dict[int, int], q_goal, motion: dict, *, hold_pv: int, hold_pa: int,
              path_checker=None, fk=None, grid_dt: float = 0.02, check_step_deg: float = 0.5) -> HomePlan:
    """joints: config.JointCfg (with sign and limit_rad); cal: Calibration; p0: measured counts now."""
    h, lim = motion["home"], motion["limits"]
    v_lim = math.radians(min(h["speed_deg_s"], lim["v_cap_deg_s"]))
    a_lim = math.radians(h["accel_deg_s2"])
    v_cap = math.radians(lim["v_cap_deg_s"])
    errors, warnings = [], []
    maps = [j.joint_map() for j in joints]
    q0 = cal.q_of(p0)
    q1 = np.asarray(q_goal, float)
    goals = cal.counts(q1)

    caps = {}
    t_min = {}
    for j, jm in zip(joints, maps):
        pv_cap, pa_cap = _raw_caps(jm.gear, v_lim, a_lim, lim["pv_raw_max"])
        caps[j.name] = (pv_cap, pa_cap)
        if pv_cap < 1:
            errors.append(f"{j.name}: 속도 상한이 PV raw 1보다 낮음")
        if pa_cap < 1:
            errors.append(f"{j.name}: 가속 상한 {h['accel_deg_s2']} deg/s^2가 PA raw 1({math.degrees(pa_unit_rad_s2(jm.gear)):.4f} deg/s^2)보다 낮음")
        if not goal_in_range(goals[jm.dxl_id]):
            errors.append(f"{j.name}: Goal {goals[jm.dxl_id]} count가 레지스터 범위(+-1,048,575) 밖")
        if j.limit_rad is None:
            errors.append(f"{j.name}: 실물 가동 범위 미확정")
        else:
            lo, hi = j.limit_rad
            tol = math.radians(START_TOL_DEG)          # e.g. an IK pose at the limit, one servo count beyond it
            if not lo - tol <= q0[maps.index(jm)] <= hi + tol:
                errors.append(f"{j.name}: 현재 자세 {math.degrees(q0[maps.index(jm)]):.3f} deg가 가동 범위 밖")
            if not lo <= q1[maps.index(jm)] <= hi:
                errors.append(f"{j.name}: HOME {math.degrees(q1[maps.index(jm)]):.3f} deg가 가동 범위 밖")
        dp = goals[jm.dxl_id] - p0[jm.dxl_id]
        if dp and pv_cap >= 1 and pa_cap >= 1:
            t_min[j.name] = servo_profile(dp, pv_cap, pa_cap).T
    if errors:
        return HomePlan([], 0.0, 0.0, 0.0, 0.0, {}, 0.0, None, errors, warnings, {"raw": caps})

    if t_min:
        dom = max(t_min, key=t_min.get)
        jd = maps[[j.name for j in joints].index(dom)]
        ideal = servo_profile(goals[jd.dxl_id] - p0[jd.dxl_id], *caps[dom])
        T, ta = ideal.T, ideal.t_acc
    else:
        dom, ideal, T, ta = None, None, 0.0, 0.0

    axes = []
    for j, jm in zip(joints, maps):
        dp = goals[jm.dxl_id] - p0[jm.dxl_id]
        pv_cap, pa_cap = caps[j.name]
        i = maps.index(jm)
        if dp == 0:
            prof = Trapezoid(0.0, pv_counts_s(hold_pv), pa_counts_s2(hold_pa))
            axes.append(AxisPlan(j.name, jm.dxl_id, jm.k, jm.sign, q0[i], q1[i], p0[jm.dxl_id], goals[jm.dxl_id],
                                 hold_pv, hold_pa, prof))
            continue
        v_need = abs(dp) / (T - ta) / pv_counts_s(1)      # raw PV so that this axis also takes T
        pvs = {min(pv_cap, max(1, math.floor(v_need))), min(pv_cap, max(1, math.ceil(v_need)))}
        pv, best = min(((pv_, pa) for pv_ in pvs for pa in range(1, pa_cap + 1)),
                       key=lambda c: (abs(servo_profile(dp, *c).T - T), c[1], c[0]))
        axes.append(AxisPlan(j.name, jm.dxl_id, jm.k, jm.sign, q0[i], q1[i], p0[jm.dxl_id], goals[jm.dxl_id],
                             pv, best, servo_profile(dp, pv, best)))

    moving = [a for a in axes if a.profile.D > 0]
    T_max = max((a.profile.T for a in moving), default=0.0)
    spread = (T_max - min(a.profile.T for a in moving)) if moving else 0.0
    plan = HomePlan(axes, T, ta, T_max, spread, {}, 0.0, None, errors, warnings,
                    {"v_lim_deg_s": math.degrees(v_lim), "a_lim_deg_s2": math.degrees(a_lim),
                     "v_cap_deg_s": lim["v_cap_deg_s"], "raw": caps, "dominant": dom})

    # speeds/accelerations of the integer values actually sent
    for a in moving:
        if a.v_peak_rad_s > v_cap * (1 + 1e-9) or a.v_peak_rad_s > v_lim * (1 + 1e-9):
            errors.append(f"{a.name}: 정수 PV {a.pv}의 속도 {math.degrees(a.v_peak_rad_s):.4f} deg/s가 상한 초과")
        if a.a_rad_s2 > a_lim * (1 + 1e-9):
            errors.append(f"{a.name}: 정수 PA {a.pa}의 가속 {math.degrees(a.a_rad_s2):.4f} deg/s^2가 상한 초과")

    # predicted path vs the ideal common path
    t = np.unique(np.concatenate([np.arange(0.0, T_max, grid_dt), [T_max]])) if T_max > 0 else np.array([0.0])
    q_pred = np.stack([a.q_at(t) for a in axes], axis=1)
    sigma = (np.abs(ideal.pos(t)) / ideal.D) if ideal is not None else np.ones_like(t)
    q_ideal = q0[None, :] + (q1 - q0)[None, :] * sigma[:, None]
    dev = np.abs(q_pred - q_ideal).max(axis=0)
    plan.sync_dev_rad = {a.name: float(d) for a, d in zip(axes, dev)}
    if moving:
        prog = np.stack([(a.q_at(t) - a.q0) / (a.q1 - a.q0) for a in moving if a.q1 != a.q0], axis=1)
        plan.progress_spread = float((prog.max(axis=1) - prog.min(axis=1)).max()) if prog.size else 0.0
    worst = max(plan.sync_dev_rad, key=plan.sync_dev_rad.get) if axes else None
    if worst and plan.sync_dev_rad[worst] > math.radians(h["sync_warn_deg"]):
        warnings.append(f"양자화로 공동 궤적과 최대 {math.degrees(plan.sync_dev_rad[worst]):.2f} deg 차이 "
                        f"({worst}, 도착 시각 차 {spread:.2f} s) - 경고만, 실제 추종 오류와 별개")
    if fk is not None and T_max > 0:
        coarse = slice(None, None, max(1, int(round(0.2 / grid_dt))))
        tcp_p = np.array([fk(q) for q in q_pred[coarse]])
        tcp_i = np.array([fk(q) for q in q_ideal[coarse]])
        plan.tcp_dev_m = float(np.linalg.norm(tcp_p - tcp_i, axis=1).max())

    # every predicted point and its braking point stay inside the confirmed range
    for k, a in enumerate(axes):
        j = joints[k]
        lo, hi = min(j.limit_rad[0], a.q0), max(j.limit_rad[1], a.q0)    # a start just outside moves inward only
        if a.profile.D > 0:
            v = a.qd_at(t)
            stop = q_pred[:, k] + np.sign(v) * v ** 2 / (2 * a.a_rad_s2)
            ends = np.concatenate([q_pred[:, k], np.clip(stop, min(a.q0, a.q1), max(a.q0, a.q1))])
        else:
            ends = q_pred[:, k]
        if ends.min() < lo - 1e-12 or ends.max() > hi + 1e-12:
            errors.append(f"{a.name}: 예측 경로/제동 지점이 가동 범위 밖")
    if path_checker is not None:
        # check wherever any joint has moved check_step_deg since the last check (and both ends)
        idxs, last = [0], q_pred[0]
        for idx in range(1, len(t)):
            if np.abs(q_pred[idx] - last).max() >= math.radians(check_step_deg):
                idxs.append(idx)
                last = q_pred[idx]
        for idx in idxs + [len(t) - 1]:
            problems = path_checker(q_pred[idx])
            if problems:
                errors.append(f"t={t[idx]:.2f}s 모델 경로 검사: {problems[0]}")
                break
    return plan


# ---------------------------------------------------------------- stopping

@dataclass(frozen=True)
class StopTarget:
    goal: int
    pa: int
    pv: int
    stop_time_s: float
    note: str


def braking_target(pos_traj: int, vel_traj_raw: int, goal_now: int, pv: int, pa: int, *, latency_s: float,
                   pa_max: int, inside) -> StopTarget:
    """Goal that lets the servo profile decelerate from its current internal velocity without reversing.

    pos_traj / vel_traj_raw: Position Trajectory(140) / Velocity Trajectory(136) of one fresh snapshot (the servo's
    internal target, not the measured position). The goal is put at least the stopping distance v^2/(2a) ahead,
    plus what the profile can still travel during the write latency, and never beyond the current goal.
    inside(goal) -> bool checks the confirmed joint range; if the stopping point is outside, the stronger
    deceleration pa_max is tried; if that is also outside, the goal is kept and the note says so (no clipping that
    would pretend the stop fits). Model only - no guarantee of zero overshoot on the real servo."""
    v = pv_counts_s(vel_traj_raw)
    if vel_traj_raw == 0:
        return StopTarget(pos_traj, max(pa, 1), max(pv, 1), 0.0, "already stopped (Velocity Trajectory 0)")
    direction = 1 if v > 0 else -1
    note = ""
    for a_raw in sorted({max(pa, 1), max(pa_max, 1)}):
        a = pa_counts_s2(a_raw)
        dist = v * v / (2 * a) + abs(v) * latency_s + 1.5 * a * latency_s ** 2 + 2
        goal = pos_traj + direction * math.ceil(dist)
        if (goal_now - goal) * direction < 0:
            goal = goal_now                                # the current goal is already closer: it stops there
            # ...only if the profile can still stop before it: otherwise raise PA. An overshoot would leave the path
            # the goal was planned on (a streamed trajectory keeps its goal only a little ahead of the profile)
            room = abs(goal_now - pos_traj) - abs(v) * latency_s
            need = math.ceil(v * v / (2 * max(room, 1.0)) / pa_counts_s2(1))
            if need > a_raw:
                note = (note + "; " if note else "") + f"PA {a_raw} -> {need} to stop before the current goal"
                a_raw, a = need, pa_counts_s2(need)
        if inside(goal):
            return StopTarget(int(goal), a_raw, max(pv, 1), abs(v) / a, note or "brake")
        note = f"stopping point outside the range at PA {a_raw}"
    return StopTarget(int(goal), a_raw, max(pv, 1), abs(v) / a, note + "; kept (no clipping) - range margin insufficient")
