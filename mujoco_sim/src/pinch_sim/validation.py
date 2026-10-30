"""Common validation for any selected path (initial now, edited later).

execution_allowed is True only if every REQUIRED kind is present exactly once
and is PASS. FAIL, NOT_EVALUATED, a missing kind or an exception in a check all
block execution. Joint-space checks run on the FINAL spline (re-sampled at
dt_check), i.e. the trajectory that will actually be executed, and the last
check is a dry rollout of that trajectory with the same controller on a copy of
the live MjData. pac_path_editor's ValidationReport.passed is not used anywhere.

Everything here is about this MuJoCo model; it says nothing about a real robot.
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field

import mujoco
import numpy as np

from .collision import CollisionChecker
from .control import TrajectoryExecutor
from .frames import WORLD, angle_deg
from .kinematics import Kinematics
from .model import RobotIndex, config_sha, joint_speed_max_rad_s
from .trajectory import JointTrajectory, build_trajectory, time_path, track_ik

REQUIRED = ("path_wellformed", "start_matches_robot", "goal_matches_target", "workspace", "continuous_ik",
            "joint_limits", "singularity", "collision", "timing", "joint_vel_acc", "torque", "sim_rollout")
OUT_OF_SCOPE = ("real servo limits (angle/velocity/torque)", "real mass/friction", "mesh vs real shape error",
                "obstacles other than the floor", "communication latency", "real robot execution")
STATUSES = ("PASS", "FAIL", "NOT_EVALUATED")


@dataclass
class Check:
    kind: str
    status: str
    message: str = ""
    sample_index: int | None = None
    details: dict = field(default_factory=dict)
    required: bool = True

    def to_dict(self) -> dict:
        return dict(self.__dict__)


@dataclass
class Goal:
    p_m: np.ndarray
    tool_axis: np.ndarray
    label: str = ""


@dataclass
class Binding:
    path_fp: str
    start_q_rad: np.ndarray
    model_sha: str
    config_sha: str


@dataclass
class ValidationReport:
    report_id: str
    path_id: str
    checks: list
    binding: Binding
    trajectory: JointTrajectory | None
    created_at: float
    compute_s: float
    extras: dict = field(default_factory=dict)

    @property
    def execution_allowed(self) -> bool:
        kinds = [c.kind for c in self.checks]
        if any(kinds.count(k) != 1 for k in REQUIRED):
            return False
        if any(c.status not in STATUSES for c in self.checks):
            return False
        return all(c.status == "PASS" for c in self.checks if c.kind in REQUIRED or c.required) \
            and self.trajectory is not None

    def blocking(self) -> list:
        kinds = [c.kind for c in self.checks]
        out = [f"{k}: missing" for k in REQUIRED if k not in kinds]
        out += [f"{c.kind}: {c.status} {c.message}" for c in self.checks
                if (c.required or c.kind in REQUIRED) and c.status != "PASS"]
        return out

    def summary(self) -> str:
        n_pass = sum(c.status == "PASS" for c in self.checks if c.kind in REQUIRED)
        if self.execution_allowed:
            return f"PASS {n_pass}/{len(REQUIRED)}"
        first = next((c for c in self.checks if c.status != "PASS"), None)
        where = f" @ sample {first.sample_index}" if first and first.sample_index is not None else ""
        return f"FAIL {n_pass}/{len(REQUIRED)}: {first.kind}{where}" if first else "FAIL"

    def to_dict(self) -> dict:
        return {"report_id": self.report_id, "path_id": self.path_id, "execution_allowed": self.execution_allowed,
                "summary": self.summary(), "blocking": self.blocking(),
                "checks": [c.to_dict() for c in self.checks],
                "binding": {"path_fp": self.binding.path_fp, "start_q_rad": self.binding.start_q_rad.tolist(),
                            "model_sha": self.binding.model_sha, "config_sha": self.binding.config_sha},
                "duration_s": self.trajectory.duration if self.trajectory else None,
                "compute_s": self.compute_s, "out_of_scope": list(OUT_OF_SCOPE), "extras": self.extras}


@dataclass
class PlanningContext:
    m: mujoco.MjModel
    idx: RobotIndex
    model_sha: str
    cfg: dict

    @property
    def cfg_sha(self) -> str:
        return config_sha(self.cfg)


def _json(x):
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, np.ndarray):
        return x.tolist()
    return x


def validate(ctx: PlanningContext, path, live_qpos, live_qvel, live_time: float, goal: Goal) -> ValidationReport:
    """Validate `path` from the live robot state (qpos/qvel arrays copied by the caller)."""
    t_start = time.time()
    cfg, idx = ctx.cfg, ctx.idx
    vc, lim = cfg["validation"], cfg["limits"]
    kin = Kinematics(ctx.m, idx, cfg["ik"])
    col = CollisionChecker(ctx.m, idx)
    q_now = np.asarray(live_qpos, float)[idx.qpos].copy()
    checks: list[Check] = []
    state: dict = {}

    index_space = {"start_matches_robot": "path", "goal_matches_target": "path", "workspace": "path",
                   "continuous_ik": "timing"}

    def where(c: Check):
        """World position (m) of a check's sample index, for drawing the failure location."""
        space = c.details.get("index_space", index_space.get(c.kind, "dense"))
        k = c.sample_index
        if space == "path":
            return path.points_m[min(k, len(path.points_m) - 1)].tolist()
        if space == "timing" and "timing" in state:
            return state["timing"].p_m[min(k, len(state["timing"].p_m) - 1)].tolist()
        if space == "dense" and "dense" in state:
            tt = state["dense"][0]
            return state["traj"].tcp_at(tt[min(k, len(tt) - 1)]).tolist()
        return None

    def run(kind, fn):
        try:
            c = fn()
            if c.sample_index is not None and c.status != "PASS":
                c.details["where_m"] = where(c)
            checks.append(c)
        except Exception as exc:  # a crashing check is a failing check
            checks.append(Check(kind, "FAIL", f"exception: {type(exc).__name__}: {exc}"))

    def needs(kind, *keys):
        missing = [k for k in keys if k not in state]
        if missing:
            return Check(kind, "NOT_EVALUATED", f"prerequisite missing: {', '.join(missing)}")
        return None

    # 1 ----------------------------------------------------------------------------------------
    def c_wellformed():
        P, A = path.points_m, path.tool_axis
        problems = []
        if path.frame != WORLD:
            problems.append(f"frame {path.frame} != world")
        if P.ndim != 2 or P.shape[1] != 3 or len(P) < 2:
            problems.append("points not (N>=2,3)")
        elif not (np.all(np.isfinite(P)) and np.all(np.isfinite(A))):
            problems.append("non-finite values")
        elif A.shape != P.shape or np.max(np.abs(np.linalg.norm(A, axis=1) - 1)) > 1e-6:
            problems.append("tool_axis not unit (N,3)")
        wp = path.waypoint_index
        if not problems and (wp[0] != 0 or wp[-1] != len(P) - 1 or any(np.diff(wp) <= 0)):
            problems.append("waypoint_index must increase from 0 to N-1")
        if problems:
            return Check("path_wellformed", "FAIL", "; ".join(problems))
        state["wellformed"] = True
        return Check("path_wellformed", "PASS", f"{len(P)} points, {len(wp) - 1} segments, "
                     f"{path.length_m:.3f} m", details={"n": len(P), "length_m": path.length_m})

    # 2 ----------------------------------------------------------------------------------------
    def c_start():
        if (c := needs("start_matches_robot", "wellformed")):
            return c
        pose = kin.fk(q_now)
        dp = float(np.linalg.norm(path.points_m[0] - pose.p_m))
        da = angle_deg(path.tool_axis[0], pose.tool_axis)
        ok = dp <= vc["start_pos_tol_m"] and da <= vc["start_axis_tol_deg"]
        return Check("start_matches_robot", "PASS" if ok else "FAIL",
                     f"start vs FK(q_now): {dp*1000:.3f} mm, {da:.3f} deg" + ("" if ok else " (robot moved?)"), 0,
                     {"pos_err_m": dp, "axis_err_deg": da})

    # 3 ----------------------------------------------------------------------------------------
    def c_goal():
        if (c := needs("goal_matches_target", "wellformed")):
            return c
        dp = float(np.linalg.norm(path.points_m[-1] - goal.p_m))
        da = angle_deg(path.tool_axis[-1], goal.tool_axis)
        ok = dp <= vc["goal_pos_tol_m"] and da <= vc["goal_axis_tol_deg"]
        return Check("goal_matches_target", "PASS" if ok else "FAIL",
                     f"end vs {goal.label or 'goal'}: {dp*1000:.3f} mm, {da:.3f} deg", len(path.points_m) - 1,
                     {"pos_err_m": dp, "axis_err_deg": da, "goal_m": goal.p_m.tolist()})

    # 4 ----------------------------------------------------------------------------------------
    def c_workspace():
        if (c := needs("workspace", "wellformed")):
            return c
        P = path.points_m
        z_i = int(np.argmin(P[:, 2]))
        radial = np.linalg.norm(P[:, :2], axis=1)
        r_i = int(np.argmax(radial))
        zmin, rmax = float(P[z_i, 2]), float(radial[r_i])
        if zmin < cfg["path"]["tcp_z_min_m"]:
            return Check("workspace", "FAIL", f"TCP z {zmin:.4f} m < {cfg['path']['tcp_z_min_m']} m", z_i,
                         {"z_min_m": zmin})
        if rmax > cfg["path"]["radial_max_m"]:
            return Check("workspace", "FAIL", f"radial {rmax:.3f} m > {cfg['path']['radial_max_m']} m", r_i)
        return Check("workspace", "PASS", f"z_min {zmin:.4f} m, radial_max {rmax:.3f} m", details={
            "z_min_m": zmin, "radial_max_m": rmax})

    # 5 continuous IK (+ timing of the Cartesian path and the spline that results) -----------------
    def c_ik():
        if (c := needs("continuous_ik", "wellformed")):
            return c
        timing = time_path(path, cfg)
        ik = track_ik(kin, timing, q_now)
        state["timing"] = timing
        bad_err = np.where((ik.pos_err_m > vc["ik_pos_tol_m"]) | (ik.axis_err_deg > vc["ik_axis_tol_deg"]))[0]
        bad_jump = np.where(ik.jump_rad > vc["ik_jump_rad"])[0]
        det = {"samples": len(ik.q), "max_pos_err_m": float(ik.pos_err_m.max()),
               "max_axis_err_deg": float(ik.axis_err_deg.max()), "max_jump_rad": float(ik.jump_rad.max()),
               "nonconverged": int((~ik.ok).sum())}
        if len(bad_err):
            k = int(bad_err[0])
            return Check("continuous_ik", "FAIL", f"IK error {ik.pos_err_m[k]*1000:.2f} mm / "
                         f"{ik.axis_err_deg[k]:.2f} deg at t={timing.t_s[k]:.2f}s", k, det)
        if len(bad_jump):
            k = int(bad_jump[0])
            return Check("continuous_ik", "FAIL", f"joint jump {ik.jump_rad[k]:.3f} rad (branch switch) at "
                         f"t={timing.t_s[k]:.2f}s", k, det)
        traj, info = build_trajectory(timing, ik, cfg)
        state["traj"], state["retime"] = traj, info
        tt, qq, qd, qdd = traj.dense(cfg["timing"]["dt_check_s"])
        state["dense"] = (tt, qq, qd, qdd)
        # the executed spline must still follow the Cartesian path between IK samples
        dev = np.array([np.linalg.norm(kin.fk(q).p_m - traj.tcp_at(t)) for t, q in zip(tt, qq)])
        k = int(np.argmax(dev))
        det.update({"spline_tcp_dev_max_m": float(dev[k]), "dense_samples": len(tt)})
        if dev[k] > vc["traj_tcp_dev_m"]:
            return Check("continuous_ik", "FAIL", f"final spline leaves path by {dev[k]*1000:.2f} mm at "
                         f"t={tt[k]:.2f}s", k, dict(det, index_space="dense"))
        return Check("continuous_ik", "PASS", f"{len(ik.q)} IK samples, max err {det['max_pos_err_m']*1000:.3f} mm,"
                     f" max jump {det['max_jump_rad']:.4f} rad, spline dev {dev[k]*1000:.3f} mm", details=det)

    # 6 ----------------------------------------------------------------------------------------
    def c_limits():
        if (c := needs("joint_limits", "dense")):
            return c
        _, qq, _, _ = state["dense"]
        m_ = lim["joint_margin_rad"]
        lo_v, hi_v = (idx.lo + m_) - qq, qq - (idx.hi - m_)
        viol = np.maximum(lo_v, hi_v)
        k, j = np.unravel_index(int(np.argmax(viol)), viol.shape)
        det = {"q_min_rad": qq.min(0).tolist(), "q_max_rad": qq.max(0).tolist(), "margin_rad": m_}
        if viol[k, j] > 0:
            return Check("joint_limits", "FAIL", f"Revolute_{j+1} = {qq[k, j]:.3f} rad beyond limit-margin", int(k), det)
        real = cfg.get("real_arm")
        if real:
            # the real arm streams this trajectory with a fixed profile acceleration: wherever it is stopped, its
            # braking point (q + sgn(qd) qd^2 / 2a) must still be inside the real range
            _, _, qd, _ = state["dense"]
            a = np.radians(real["a_stream_deg_s2"])
            brake = qq + np.sign(qd) * qd * qd / (2 * a)
            bviol = np.maximum(idx.lo - brake, brake - idx.hi)
            k, j = np.unravel_index(int(np.argmax(bviol)), bviol.shape)
            if bviol[k, j] > 0:
                return Check("joint_limits", "FAIL", f"Revolute_{j+1} braking point {brake[k, j]:.3f} rad beyond the "
                             "real arm's range", int(k), det)
            return Check("joint_limits", "PASS", f"within [lo+{m_}, hi-{m_}] rad of the real arm (robot.yaml), "
                         "braking points inside", details=det)
        return Check("joint_limits", "PASS", f"within [lo+{m_}, hi-{m_}] rad (model limits +-pi)", details=det)

    # 7 ----------------------------------------------------------------------------------------
    def c_singular():
        if (c := needs("singularity", "dense")):
            return c
        _, qq, _, _ = state["dense"]
        sig = np.array([kin.sigma_min(q) for q in qq[::2]])
        k = int(np.argmin(sig))
        ok = sig[k] >= lim["sigma_min_m_per_rad"]
        return Check("singularity", "PASS" if ok else "FAIL",
                     f"min sigma(Jp) {sig[k]:.4f} m/rad (threshold {lim['sigma_min_m_per_rad']})", 2 * k,
                     {"sigma_min": float(sig[k])})

    # 8 ----------------------------------------------------------------------------------------
    def c_collision():
        if (c := needs("collision", "dense")):
            return c
        _, qq, _, _ = state["dense"]
        fmin, smin, fbody, spair = np.inf, np.inf, "", ()
        per_body: dict = {}
        for k, q in enumerate(qq):
            r = col.check(q)
            for b, v in r.floor_by_body.items():
                per_body[b] = min(per_body.get(b, np.inf), v)
            if r.in_contact:
                b1, b2, dist = r.contacts[0]
                return Check("collision", "FAIL", f"contact {b1}-{b2} ({dist*1000:.1f} mm)", k,
                             {"contacts": r.contacts[:4]})
            if r.floor_clearance_m < fmin:
                fmin, fbody = r.floor_clearance_m, r.floor_geom_body
            if r.self_clearance_m < smin:
                smin, spair = r.self_clearance_m, r.self_pair
            if r.floor_clearance_m < vc["floor_clearance_m"]:
                return Check("collision", "FAIL", f"floor clearance {r.floor_clearance_m*1000:.1f} mm "
                             f"({r.floor_geom_body}) < {vc['floor_clearance_m']*1000:.0f} mm", k)
            if r.self_clearance_m < vc["self_clearance_m"]:
                return Check("collision", "FAIL", f"self clearance {r.self_clearance_m*1000:.1f} mm {r.self_pair}", k)
        distal = {b: v for b, v in per_body.items() if b != "r_a_1"}   # r_a_1 only yaws: constant height
        dbody = min(distal, key=distal.get) if distal else ""
        return Check("collision", "PASS", f"0 contacts; floor clearance {fmin*1000:.1f} mm ({fbody}), moving links "
                     f"{distal.get(dbody, np.nan)*1000:.1f} mm ({dbody}); self clearance {smin*1000:.1f} mm {spair}",
                     details={"floor_clearance_min_m": fmin, "floor_clearance_distal_m": distal.get(dbody),
                              "floor_clearance_by_body_m": per_body, "self_clearance_min_m": smin,
                              "self_pair": spair, "samples": len(qq)})

    # 9 ----------------------------------------------------------------------------------------
    def c_timing():
        if (c := needs("timing", "traj")):
            return c
        traj = state["traj"]
        tk = traj.t_knots
        dt = np.diff(tk)
        uniform = np.allclose(dt, dt[0], rtol=0, atol=1e-9) and dt[0] > 0
        ok = tk[0] == 0.0 and uniform and traj.duration <= cfg["timing"]["max_duration_s"]
        return Check("timing", "PASS" if ok else "FAIL",
                     f"duration {traj.duration:.2f} s, knot dt {dt[0]:.4f} s, time scale {traj.time_scale:.3f}",
                     details={"duration_s": traj.duration, "time_scale": traj.time_scale, **state["retime"]})

    # 10 ---------------------------------------------------------------------------------------
    def c_velacc():
        if (c := needs("joint_vel_acc", "dense")):
            return c
        _, _, qd, qdd = state["dense"]
        v, a = np.abs(qd), np.abs(qdd)
        kv, ka = np.unravel_index(int(np.argmax(v)), v.shape), np.unravel_index(int(np.argmax(a)), a.shape)
        v_lim = joint_speed_max_rad_s(cfg)
        det = {"qd_max_rad_s": float(v.max()), "qdd_max_rad_s2": float(a.max()),
               "qd_limit": v_lim, "qdd_limit": lim["qdd_max_rad_s2"],
               "qd_peak_deg_s": np.degrees(v.max(0)).round(3).tolist()}
        if v.max() > v_lim * (1 + 1e-6):
            return Check("joint_vel_acc", "FAIL", f"|qd| {v.max():.3f} rad/s at Revolute_{kv[1]+1}", int(kv[0]), det)
        if a.max() > lim["qdd_max_rad_s2"]:
            return Check("joint_vel_acc", "FAIL", f"|qdd| {a.max():.3f} rad/s2 at Revolute_{ka[1]+1}", int(ka[0]), det)
        return Check("joint_vel_acc", "PASS", f"|qd| max {v.max():.3f} rad/s, |qdd| max {a.max():.3f} rad/s2",
                     details=det)

    # 11 ---------------------------------------------------------------------------------------
    def c_torque():
        if (c := needs("torque", "dense")):
            return c
        _, qq, qd, qdd = state["dense"]
        d = mujoco.MjData(ctx.m)
        taus = np.zeros_like(qq)
        for k in range(len(qq)):
            d.qpos[idx.qpos], d.qvel[idx.dof], d.qacc[idx.dof] = qq[k], qd[k], qdd[k]
            mujoco.mj_inverse(ctx.m, d)
            taus[k] = d.qfrc_inverse[idx.dof]
        a = np.abs(taus)
        k, j = np.unravel_index(int(np.argmax(a)), a.shape)
        ok = a.max() <= lim["torque_check_nm"]
        return Check("torque", "PASS" if ok else "FAIL",
                     f"inverse dynamics |tau| max {a.max():.3f} N*m at Revolute_{j+1} (limit {lim['torque_check_nm']})",
                     None if ok else int(k), {"tau_max_nm": a.max(0).tolist()})

    # 12 ---------------------------------------------------------------------------------------
    def c_rollout():
        if (c := needs("sim_rollout", "traj")):
            return c
        cc = cfg["control"]
        d = mujoco.MjData(ctx.m)
        d.qpos[:], d.qvel[:], d.time = live_qpos, live_qvel, live_time
        mujoco.mj_forward(ctx.m, d)
        ex = TrajectoryExecutor(ctx.m, idx, state["traj"], cc, t0=live_time)
        while ex.status == "RUNNING":
            ex.step(d)
        met = ex.metrics.to_dict()
        state["rollout"] = met
        problems = []
        if ex.status != "DONE":
            problems.append(ex.reason)
        if met["max_q_err_rad"] > cc["rollout_q_err_rad"]:
            problems.append(f"joint err {met['max_q_err_rad']:.4f} rad > {cc['rollout_q_err_rad']}")
        if met["max_tcp_err_m"] > cc["rollout_tcp_err_m"]:
            problems.append(f"TCP err {met['max_tcp_err_m']*1000:.2f} mm > {cc['rollout_tcp_err_m']*1000:.1f} mm")
        if met["saturation_steps"] or met["contact_steps"]:
            problems.append(f"saturation {met['saturation_steps']} / contact {met['contact_steps']} steps")
        v_lim = joint_speed_max_rad_s(cfg)
        obs = max(met["qd_peak_rad_s"]) if met["qd_peak_rad_s"] else 0.0
        if obs > v_lim * (1 + cc["rollout_qd_over_ratio"]):
            problems.append(f"observed |qvel| {np.degrees(obs):.2f} deg/s > cap {np.degrees(v_lim):.2f} deg/s")
        msg = (f"dry run: max q err {met['max_q_err_rad']:.4f} rad, max TCP err {met['max_tcp_err_m']*1000:.2f} mm,"
               f" final {met['final_tcp_err_m']*1000:.3f} mm, observed |qvel| max {np.degrees(obs):.2f} deg/s")
        return Check("sim_rollout", "FAIL" if problems else "PASS", msg + ("; " + "; ".join(problems) if problems
                                                                           else ""), details=met)

    for kind, fn in (("path_wellformed", c_wellformed), ("start_matches_robot", c_start),
                     ("goal_matches_target", c_goal), ("workspace", c_workspace), ("continuous_ik", c_ik),
                     ("joint_limits", c_limits), ("singularity", c_singular), ("collision", c_collision),
                     ("timing", c_timing), ("joint_vel_acc", c_velacc), ("torque", c_torque)):
        run(kind, fn)
    # the dry run is only meaningful (and only performed) once every static check passed
    if all(c.status == "PASS" for c in checks):
        run("sim_rollout", c_rollout)
    else:
        checks.append(Check("sim_rollout", "NOT_EVALUATED", "skipped: an earlier required check did not pass"))
    for c in checks:
        c.details = {k: _json(v) for k, v in c.details.items()}
    return ValidationReport(uuid.uuid4().hex[:12], path.path_id, checks,
                            Binding(path.fingerprint, q_now, ctx.model_sha, ctx.cfg_sha),
                            state.get("traj"), time.time(), time.time() - t_start,
                            {"rollout": state.get("rollout"), "allow_unsafe": path.meta.get("allow_unsafe", False)})
