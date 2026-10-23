"""Path -> timed Cartesian samples -> continuous IK -> joint spline q(t) (+ retiming).

Timing: each segment between stop waypoints follows a min-jerk profile over the
combined metric (length + rot_equiv * angle), T = max(segment_min_s, 1.875 L / v_max),
rounded up to dt_plan; positions lie on a natural cubic spline through that segment's vertices. IK is solved sample by sample, seeded with the previous
solution (no re-seeding: a failure is reported, not hidden). The joint samples
are interpolated by a clamped CubicSpline. Joint speed cap (2026-10-12): if the fastest joint
exceeds its planning target (limits.joint_speed_max_deg_s x joint_speed_plan_ratio, see
model.joint_speed_plan_rad_s) or |qdd| exceeds qdd_max, the WHOLE time axis is stretched by one
common factor k, q(t) -> q(t/k): every joint's velocity is divided by the same k (relative speeds and
simultaneous arrival kept, geometry unchanged); k = 1 when the motion is already within the limits.
The validator then re-samples THIS spline densely; that is what gets executed.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline

from .frames import angle_deg, unit
from .kinematics import Kinematics
from .model import joint_speed_plan_rad_s


def min_jerk(u):
    u = np.asarray(u, float)
    return u ** 3 * (10 + u * (-15 + 6 * u))


@dataclass
class CartesianTiming:
    t_s: np.ndarray       # (M,)
    p_m: np.ndarray       # (M,3)
    axis: np.ndarray      # (M,3)
    stop_t_s: list        # times of stop waypoints


def time_path(path, cfg: dict) -> CartesianTiming:
    tc, rot = cfg["timing"], cfg["path"]["rot_equiv_length_m_per_rad"]
    P, A, wp = path.points_m, path.tool_axis, list(path.waypoint_index)
    step = np.linalg.norm(np.diff(P, axis=0), axis=1) + rot * np.radians(
        [angle_deg(A[i], A[i + 1]) for i in range(len(A) - 1)])
    sigma = np.concatenate([[0.0], np.cumsum(step)])
    ts, ps, axs, stops, t_off = [0.0], [P[0]], [A[0]], [0.0], 0.0
    for i0, i1 in zip(wp[:-1], wp[1:]):
        L = sigma[i1] - sigma[i0]
        T = max(tc["segment_min_s"], 1.875 * L / tc["v_max_m_s"])
        n = int(np.ceil(T / tc["dt_plan_s"] - 1e-9))
        T = n * tc["dt_plan_s"]
        s = sigma[i0] + min_jerk(np.arange(1, n + 1) / n) * L
        seg_sigma = sigma[i0:i1 + 1]
        # positions: natural cubic spline over the arc-length parameter between two stops. A
        # piecewise-linear interpolation turns the direction abruptly at every vertex of an edited
        # (curved) polyline, which shows up as joint-acceleration spikes; for collinear vertices the
        # spline is exactly the same straight line. Duplicate parameters are dropped first.
        keep = np.r_[True, np.diff(seg_sigma) > 1e-12]
        sig_k, P_k = seg_sigma[keep], P[i0:i1 + 1][keep]
        pos = CubicSpline(sig_k, P_k, axis=0, bc_type="natural")(s) if len(sig_k) >= 3 else \
            np.column_stack([np.interp(s, sig_k, P_k[:, k]) for k in range(3)])
        pos[-1] = P[i1]                                        # stops are hit exactly
        for sv, pv in zip(s, pos):
            ps.append(pv)
            axs.append(unit([np.interp(sv, seg_sigma, A[i0:i1 + 1, k]) for k in range(3)]))
        ts.extend(t_off + np.arange(1, n + 1) * tc["dt_plan_s"])
        t_off += T
        stops.append(t_off)
    return CartesianTiming(np.array(ts), np.array(ps), np.array(axs), stops)


@dataclass
class IKTrack:
    q: np.ndarray            # (M,6)
    ok: np.ndarray           # (M,) bool
    pos_err_m: np.ndarray
    axis_err_deg: np.ndarray
    jump_rad: np.ndarray     # (M,) max |dq| from previous sample (0 for the first)
    first_fail: int | None


def track_ik(kin: Kinematics, timing: CartesianTiming, q_start) -> IKTrack:
    M = len(timing.t_s)
    Q = np.zeros((M, 6))
    ok, pe, ae, jump = np.zeros(M, bool), np.zeros(M), np.zeros(M), np.zeros(M)
    q = np.asarray(q_start, float).copy()
    first_fail = None
    for k in range(M):
        r = kin.solve_ik(timing.p_m[k], timing.axis[k], q, q_ref=q)
        Q[k], ok[k], pe[k], ae[k] = r.q, r.ok, r.pos_err_m, r.axis_err_deg
        jump[k] = 0.0 if k == 0 else float(np.max(np.abs(r.q - Q[k - 1])))
        if not r.ok and first_fail is None:
            first_fail = k
        q = r.q
    return IKTrack(Q, ok, pe, ae, jump, first_fail)


@dataclass
class JointTrajectory:
    """Executable joint trajectory: clamped cubic spline q(t), t in [0, duration] s."""

    t_knots: np.ndarray
    q_knots: np.ndarray
    tcp_ref_t: np.ndarray        # Cartesian reference (for tracking error), same time axis
    tcp_ref_p: np.ndarray
    time_scale: float = 1.0
    spline: CubicSpline = field(init=False, repr=False)

    def __post_init__(self):
        self.spline = CubicSpline(self.t_knots, self.q_knots, axis=0, bc_type="clamped")

    @property
    def duration(self) -> float:
        return float(self.t_knots[-1])

    def sample(self, t: float):
        """(q, qd, qdd) at t; before 0 / after the end: hold the end point with zero derivatives."""
        if t >= self.duration:
            return self.q_knots[-1].copy(), np.zeros(6), np.zeros(6)
        t = max(t, 0.0)
        return self.spline(t), self.spline(t, 1), self.spline(t, 2)

    def dense(self, dt: float):
        n = int(np.ceil(self.duration / dt))
        t = np.linspace(0.0, self.duration, n + 1)
        return t, self.spline(t), self.spline(t, 1), self.spline(t, 2)

    def tcp_at(self, t: float) -> np.ndarray:
        t = min(max(t, 0.0), self.duration)
        return np.array([np.interp(t, self.tcp_ref_t, self.tcp_ref_p[:, k]) for k in range(3)])


def common_speed_scale(qd_peak, limit: float) -> float:
    """Velocity multiplier for the joint-speed cap: L / max|qd| if the fastest joint exceeds L, else 1.

    Every joint is multiplied by the SAME factor, e.g. [45, 30, -15] deg/s with L = 22.5 -> x0.5 ->
    [22.5, 15, -7.5]; all-zero or already-slow motions are returned unchanged (factor 1)."""
    m = float(np.max(np.abs(np.asarray(qd_peak, float)))) if np.size(qd_peak) else 0.0
    return 1.0 if m == 0.0 or m <= limit else limit / m


def build_trajectory(timing: CartesianTiming, ik: IKTrack, cfg: dict) -> tuple[JointTrajectory, dict]:
    """Spline the IK samples, then stretch time by one common factor until the speed cap and |qdd| hold.

    Stretching the clamped spline's knot times by k gives exactly q(t/k), so velocities scale by 1/k and
    accelerations by 1/k^2 for every joint at once."""
    lim, tc = cfg["limits"], cfg["timing"]
    v_lim, a_lim = joint_speed_plan_rad_s(cfg), lim["qdd_max_rad_s2"]   # cap x plan ratio (tracking headroom)
    scale, history = 1.0, []
    for it in range(tc["max_retime_iter"] + 2):
        traj = JointTrajectory(timing.t_s * scale, ik.q, timing.t_s * scale, timing.p_m, scale)
        _, _, qd, qdd = traj.dense(tc["dt_check_s"])
        peak = np.max(np.abs(qd), axis=0)
        v_ratio = float(peak.max() / v_lim)
        a_ratio = float(np.max(np.abs(qdd)) / a_lim)
        history.append({"scale": scale, "duration_s": traj.duration, "v_ratio": v_ratio, "a_ratio": a_ratio,
                        "qd_peak_deg_s": np.degrees(peak).round(3).tolist()})
        if (v_ratio <= 1.0 and a_ratio <= 1.0) or it == tc["max_retime_iter"] + 1:
            break
        # speed: exactly 1 / common_speed_scale; acceleration scales with k^2. A dense-sampling residue of
        # ~1e-6 can remain after the exact factor, hence the tiny bump on later iterations.
        k = max(1.0 / common_speed_scale(peak, v_lim), np.sqrt(a_ratio), 1.0) * (1.0 if it == 0 else 1.0 + 1e-6)
        scale *= k
    if traj.duration > tc["max_duration_s"]:
        history.append({"stopped": f"stretched duration {traj.duration:.2f}s exceeds max_duration_s"})
    return traj, {"retime": history}
