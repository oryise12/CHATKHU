r"""Selection -> validation -> approval -> execution state machine (UI independent).

    HOLD --N--> GENERATED --S--> SELECTED --V--> VALIDATING --> PASSED --Y--> EXECUTING --> DONE --> (HOLD)
                                     ^                      \-> FAILED (reasons) --V--> ...
    X: EXECUTING -> ABORTED (hold the current pose); VALIDATING -> SELECTED; otherwise -> GENERATED
    H: from HOLD/DONE/ABORTED, a one-way return path from the ACTUAL current TCP to ready.
    Path edit (mouse now, hand later; edit_begin/update/end/cancel, U undo, R redo, O original):
       allowed in GENERATED/SELECTED/PASSED/FAILED; any change drops the report -> GENERATED, so the
       edited path needs S, V, Y again. Refused while VALIDATING or EXECUTING (and on DONE/ABORTED/HOLD,
       whose path no longer starts at the robot).

Voice (pinch_sim.voice_gateway) calls the SAME methods from the SAME main-loop thread. Additions for it:
  * rev: +1 on every state set (_set) and edit-mode change; voice answers carry the rev they refer to.
  * listeners: called from _emit with every Event (kind, actor, request_id, rev) - the gateway forwards them.
  * edit_mode: informational "edit mode" flag (voice "start edit", any drag, FAILED). It never blocks
    the mouse; can_edit() alone decides whether editing is allowed.
  * auto-run: cmd_validate(d, auto_run_request=id) or arm_auto_run(id) books "run when THIS validation
    passes". watch(d) then calls cmd_approve(d) (same binding re-check) only if the PASSED report has the
    booked token and path fingerprint. Any _invalidate (edit, X, new path, regeneration, execution end)
    or cancel_auto_run (voice link lost) drops the booking. Button V never books it: Y is still needed.

Rules enforced here (and tested in tests/test_validation_session.py, tests/test_voice_gateway.py):
  * Y does anything only in PASSED with report.execution_allowed; right before starting it re-checks
    path fingerprint, model sha, config sha and the robot's joint state against the report binding.
  * A new/regenerated path or target drops the report; robot drift beyond tolerance in PASSED does too.
  * An edit (begin of a drag, undo, redo, reset) drops the report and any in-flight validation result.
  * Validation runs in a worker thread on copies of the state; the live MjData is only touched by
    physics_step() in the caller's thread.
"""
from __future__ import annotations

import json
import time
from contextlib import contextmanager
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .collision import CollisionChecker
from .editing import PathEdit
from .control import ComputedTorque, TrajectoryExecutor, controlled_step
from .frames import DOWN, WORLD, Pose, RobotState
from .kinematics import Kinematics
from .model import RobotIndex, config_sha, ready_q
from .paths import CartesianPath, PathGenerationError, initial_path, polyline_distance, return_path
from .targets import TargetSpec, make_target, precheck, sample_target
from .validation import Goal, PlanningContext, ValidationReport, validate

STATES = ("HOLD", "GENERATED", "SELECTED", "VALIDATING", "PASSED", "FAILED", "EXECUTING", "DONE", "ABORTED")
KOREAN = {"HOLD": "대기(자세 유지)", "GENERATED": "경로 생성됨", "SELECTED": "경로 선택됨", "VALIDATING": "검증 중",
          "PASSED": "검증 통과", "FAILED": "검증 실패", "EXECUTING": "실행 중", "DONE": "완료", "ABORTED": "중단됨"}
EDITABLE = ("GENERATED", "SELECTED", "PASSED", "FAILED")


@dataclass
class Event:
    t_wall: float
    t_sim: float
    state: str
    message: str
    data: dict = field(default_factory=dict)
    kind: str = "info"
    actor: str = "sim"            # key / mouse / script / voice / sim (async: validation result, watch)
    request_id: str = ""
    rev: int = 0


@dataclass(frozen=True)
class AutoRun:
    """Voice request "run this path once it passes": bound to one validation token and path version."""
    request_id: str
    token: int
    path_fp: str


class Session:
    def __init__(self, m: mujoco.MjModel, idx: RobotIndex, model_sha: str, cfg: dict, d: mujoco.MjData, *,
                 seed: int = 0, allow_unsafe: bool = False, threaded: bool = True, log_path: Path | None = None):
        self.m, self.idx, self.cfg = m, idx, cfg
        self.ctx = PlanningContext(m, idx, model_sha, cfg)
        self.kin = Kinematics(m, idx, cfg["ik"])
        self.col = CollisionChecker(m, idx)
        self.ctrl = ComputedTorque(m, idx, cfg["control"])
        self.q_ready = ready_q(m)[idx.qpos]
        self.ready_pose = self.kin.fk(self.q_ready)
        self.hold_q = d.qpos[idx.qpos].copy()
        self.seed, self.trial, self.allow_unsafe = seed, 0, allow_unsafe
        self.new_presses = 0          # N presses so far (the automatic start target is not counted)
        self.target_label = ""        # "start target" / "New #k" / "fixed", shown on screen
        self.state = "HOLD"
        self.target: TargetSpec | None = None
        self.goal: Goal | None = None
        self.path = None
        self.edit: PathEdit | None = None     # editing session of the current generated path
        self.report: ValidationReport | None = None
        self.executor: TrajectoryExecutor | None = None
        self.last_run: dict | None = None
        self.message = "N: new target"
        self.events: list[Event] = []
        self._pool = ThreadPoolExecutor(1) if threaded else None
        self._future: Future | None = None
        self._token = 0
        self._log_path = log_path
        self._t_sim = float(d.time)
        self.rev = 0                  # +1 on every state set / edit-mode change
        self.edit_mode = False        # informational (voice dialog, display); never blocks the mouse
        self.auto_run: AutoRun | None = None
        self.listeners: list = []     # callables(event: Event), called in this (main) thread
        self.pointed: np.ndarray | None = None   # world point the user clicked/pointed at (voice "here")
        self._pointed_hist: list[tuple[int, np.ndarray]] = []   # (edit count after a voice nudge, point before)
        self._pointed_redo: list = []                           # undone voice nudges: (count, before, after)
        # two phases: "outbound" (start -> target, editable first) and "return" (target -> start).
        self.phase = "outbound"
        self.last_exec: dict | None = None   # {"path", "goal", "phase", "status"} of the last execution
        self.outbound_exec: CartesianPath | None = None   # the full (edited) start -> target path that was run
        # what kind of move the current path is: "outbound" | "return_reverse" | "return_edit" | "return_home";
        # a resumed piece keeps the kind of the move it interrupted ("back" also returns to that move's editing)
        self.move_kind = "outbound"
        self._resume_dir: str | None = None
        self.outbound_edit: PathEdit | None = None        # editing session of the outbound path that was run
        self.outbound_goal: Goal | None = None
        self.return_exec: CartesianPath | None = None     # the full return path that was run
        self.return_edit: PathEdit | None = None          # its editing session (edited return only)
        self.return_goal: Goal | None = None
        self._saved_pointed: dict = {}                    # move kind -> (clicked point, its undo history)
        self.ab: tuple[np.ndarray, TargetSpec] | None = None   # fixed A (end effector) and B (desk target)
        self._path_builder = None            # rebuilds the current kind of path from the robot state (regenerate)
        self.screen_axes: dict[str, np.ndarray] | None = None   # voice directions, set by the app from the camera
        self._actor, self._request_id = "sim", ""

    # ------------------------------------------------------------------ bookkeeping
    @contextmanager
    def acting(self, actor: str, request_id: str = ""):
        """Tag the events emitted inside with who caused them (key, mouse, script, voice)."""
        prev = self._actor, self._request_id
        self._actor, self._request_id = actor, request_id
        try:
            yield
        finally:
            self._actor, self._request_id = prev

    def _emit(self, msg: str, kind: str = "info", **data) -> None:
        self.message = msg
        ev = Event(time.time(), self._t_sim, self.state, msg, data, kind, self._actor, self._request_id, self.rev)
        self.events.append(ev)
        who = "" if self._actor in ("sim", "key") else f"[{self._actor}] "
        print(f"[{self._t_sim:8.3f}s] {KOREAN[self.state]:<10} {who}{msg}", flush=True)
        if self._log_path:
            with open(self._log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(ev.__dict__, ensure_ascii=False, default=_json_default) + "\n")
        for listener in list(self.listeners):
            listener(ev)

    def _set(self, state: str, msg: str, kind: str = "state", **data) -> None:
        assert state in STATES
        self.state = state
        self.rev += 1
        self._emit(msg, kind, **data)

    def set_edit_mode(self, on: bool, why: str = "") -> None:
        if self.edit_mode == on:
            return
        self.edit_mode = on
        self.rev += 1
        self._emit(f"edit mode {'on' if on else 'off'}" + (f": {why}" if why else ""), "edit_mode", on=on)

    def robot_state(self, d: mujoco.MjData) -> RobotState:
        q = d.qpos[self.idx.qpos].copy()
        return RobotState(float(d.time), q, d.qvel[self.idx.dof].copy(), self.kin.fk(q))

    def _set_path(self, path) -> None:
        """A newly generated path replaces any edit history (and leaves edit mode)."""
        self.path = path
        self.edit = PathEdit(path, self.cfg["editor"]) if path is not None else None
        self.edit_mode = False
        self.pointed = None
        self._pointed_hist = []

    def _invalidate(self, why: str = "path/state changed") -> None:
        self._token += 1            # any in-flight validation result is discarded
        self.report = None
        self.cancel_auto_run(why)

    # ------------------------------------------------------------------ voice auto-run booking
    def cancel_auto_run(self, why: str) -> bool:
        a = self.auto_run
        if a is None:
            return False
        self.auto_run = None
        self._emit(f"auto-run request cancelled: {why}", "auto_run_cancelled", auto_request_id=a.request_id,
                   reason=why)
        return True

    def arm_auto_run(self, request_id: str) -> bool:
        """Book "approve when the CURRENT validation passes" (only while VALIDATING)."""
        if self.state != "VALIDATING" or self.path is None:
            return False
        self.auto_run = AutoRun(request_id, self._token, self.path.fingerprint)
        self._emit(f"auto-run booked for this validation (path {self.path.fingerprint[:10]}): runs only if it "
                   "passes for this exact path", "auto_run_armed", auto_request_id=request_id)
        return True

    def _maybe_auto_run(self, d) -> None:
        a = self.auto_run
        if a is None or self.state != "PASSED" or self.report is None:
            return
        self.auto_run = None
        if a.token != self._token or self.path is None or a.path_fp != self.report.binding.path_fp \
                or a.path_fp != self.path.fingerprint:
            self._emit("auto-run dropped: the passed report is not the booked path version", "auto_run_cancelled",
                       auto_request_id=a.request_id, reason="different path version")
            return
        with self.acting("voice", a.request_id):
            if not self.cmd_approve(d):          # same binding re-check as Y; refused -> regenerated
                self._emit(f"auto-run not started: {self.message}", "auto_run_cancelled",
                           auto_request_id=a.request_id, reason=self.message)

    # ------------------------------------------------------------------ physics (caller's thread)
    def physics_step(self, d: mujoco.MjData) -> None:
        self._t_sim = float(d.time)
        if self.state == "EXECUTING":
            ex = self.executor
            ex.step(d)
            if ex.status != "RUNNING":
                self._finish_execution(d)
            return
        z = np.zeros(6)
        controlled_step(self.m, d, self.idx, self.ctrl, self.hold_q, z, z)

    def _finish_execution(self, d) -> None:
        ex = self.executor
        met = ex.metrics.to_dict()
        # executed TCP vs the approved (possibly edited) path polyline
        dev = polyline_distance(np.array(ex.trace), self.path.points_m) if ex.trace else np.array([np.nan])
        self.last_run = {"status": ex.status, "reason": ex.reason, "path_id": self.path.path_id,
                         "path_source": self.path.source, "path_fp": self.path.fingerprint,
                         "path_dev_max_m": float(np.max(dev)),
                         "report_id": self.report.report_id if self.report else None, **met}
        self.last_exec = {"path": self.path, "goal": self.goal, "phase": self.phase, "status": ex.status,
                          "kind": self.move_kind}
        if ex.status == "DONE":
            self.hold_q = ex.traj.q_knots[-1].copy()
            # stopped and went back: back to that move's editing (outbound, edited return) - not a new question
            nxt = ""
            if self._resume_dir == "back":
                nxt = {"outbound": "edit_outbound", "return_edit": "edit_return"}.get(self.move_kind, "")
            self._set("DONE", f"done ({self.path.source} path, TCP-path dev {np.max(dev)*1000:.3f} mm): "
                      f"final TCP err {met['final_tcp_err_m']*1000:.3f} mm, max q err "
                      f"{met['max_q_err_rad']:.4f} rad, contacts {met['contact_steps']}, saturation "
                      f"{met['saturation_steps']}", "execution_end", run=self.last_run, next=nxt)
            self.executor = None
            self._invalidate("execution finished")
            if nxt:
                self._back_to_edit(nxt)
            return
        else:
            self.hold_q = d.qpos[self.idx.qpos].copy()
            self._set("ABORTED", f"aborted: {ex.reason}; holding current pose", "execution_end", run=self.last_run)
        self.executor = None
        self._invalidate("execution finished")      # a report is never reused for a second execution

    # ------------------------------------------------------------------ commands
    def new_band(self, press: int):
        """Theta band (deg) for the press-th New (1-based): bands alternate 1st, 2nd, 1st, ..."""
        bands = self.cfg["target"].get("new_theta_bands_deg")
        return tuple(bands[(press - 1) % len(bands)]) if bands else None

    def cmd_new_target(self, d, count_as_new: bool = True) -> bool:
        """N: next seeded random target (trial counter) and its initial path.

        count_as_new=False (automatic target at program start): not counted, full +-theta_max_deg.
        Otherwise press k uses band new_theta_bands_deg[(k-1) % n]; redraws stay inside it.
        """
        if self.state in ("EXECUTING", "VALIDATING"):
            self._emit(f"N ignored in {self.state}", "refused")
            return False
        if self.ab is not None:                       # fixed A/B: a "new target" is the same B again
            return self._use_target(self.ab[1], d, extra="fixed A/B")
        st = self.robot_state(d)
        band, label = None, "start target"
        if count_as_new:
            self.new_presses += 1
            band = self.new_band(self.new_presses)
            label = f"New #{self.new_presses}"
        res = sample_target(self.seed, self.trial, self.cfg, self.kin, self.col, st.q_rad, theta_band_deg=band)
        self.trial += 1
        if res.target is None:
            print(f"[{label}] no target: {res.message}", flush=True)
        if res.target is None:
            self._invalidate()
            self._set_path(None)
            self.target = None
            self._set("HOLD", res.message, "new_target", attempts=res.attempts)
            return False
        t = res.target
        self.target_label = label
        print(f"[{label}] r = {t.r_m:.3f} m, theta = {np.degrees(t.theta_rad):+.1f} deg"
              + (f" (band {band[0]:+.0f}..{band[1]:+.0f} deg)" if band else " (+-%.0f deg)" % self.cfg["target"]["theta_max_deg"]),
              flush=True)
        return self._use_target(t, d, extra=f"{label}; {res.message}; rejected draws {len(res.attempts) - 1}",
                                kind="new_target",
                                attempts=res.attempts, new_press=self.new_presses if count_as_new else None,
                                theta_band_deg=band)

    def cmd_ab(self, a_world, b_world, d) -> tuple[bool, str]:
        """Fixed A (end-effector position, world m) and B (target point on the desk, world m).

        The robot is put at A (IK, tool pointing down) and the outbound path is A -> (rise) -> above B at the
        approach height -> B + work height, the same rule as a generated target. Set before editing/running.
        """
        if self.state in ("EXECUTING", "VALIDATING"):
            return False, f"busy ({self.state})"
        tc = self.cfg["target"]
        a, b = np.asarray(a_world, float), np.asarray(b_world, float)
        ik = self.kin.solve_ik(a, DOWN, q_seed=self.q_ready)
        if not ik.ok:
            return False, (f"A {np.round(a, 3).tolist()} is not reachable with the tool pointing down "
                           f"({ik.reason or 'IK'}: {ik.pos_err_m * 1000:.1f} mm, {ik.axis_err_deg:.1f} deg)")
        d.qpos[self.idx.qpos] = ik.q
        d.qvel[self.idx.dof] = 0.0
        mujoco.mj_forward(self.m, d)
        self.hold_q = ik.q.copy()
        t = TargetSpec(float(np.hypot(b[0], b[1])), float(np.arctan2(b[0], b[1])), b.copy(),
                       b + [0.0, 0.0, tc["work_height_m"]], b + [0.0, 0.0, tc["approach_height_m"]], DOWN.copy())
        self.ab = (a.copy(), t)
        self.target_label = "A/B"
        ok = self._use_target(t, d, extra="fixed A/B")
        return ok, "" if ok else self.message

    def cmd_fixed_target(self, r_m: float, theta_deg: float, d) -> bool:
        if self.state in ("EXECUTING", "VALIDATING"):
            return False
        try:
            t = make_target(r_m, np.radians(theta_deg), self.cfg["target"], allow_out_of_range=False)
        except ValueError as exc:
            self._emit(f"fixed target rejected: {exc}", "refused")
            return False
        ok, reason = precheck(t, self.kin, self.col, self.robot_state(d).q_rad, self.cfg)
        self.target_label = "fixed"
        return self._use_target(t, d, extra=f"fixed target; precheck {reason} (informational, validation decides)")

    def _use_target(self, t: TargetSpec, d, extra: str = "", kind: str = "new_target", **data) -> bool:
        self._invalidate("new target")
        self.target = t
        self.goal = Goal(t.tcp_goal_m, t.tool_axis, "tcp_goal")
        self.phase, self.last_exec, self.outbound_exec = "outbound", None, None
        self.move_kind, self._resume_dir, self.outbound_edit, self.return_exec, self.return_edit = \
            "outbound", None, None, None, None
        self._path_builder = lambda st: initial_path(st.tcp, st.q_rad, t, self.cfg, allow_unsafe=self.allow_unsafe)
        st = self.robot_state(d)
        try:
            self._set_path(initial_path(st.tcp, st.q_rad, t, self.cfg, allow_unsafe=self.allow_unsafe))
        except PathGenerationError as exc:
            self._set_path(None)
            self._set("HOLD", f"path generation failed: {exc}", kind, target=t.summary())
            return False
        self._set("GENERATED", f"target r={t.r_m:.3f} m th={np.degrees(t.theta_rad):+.1f} deg "
                  f"floor={np.round(t.floor_point_m, 3).tolist()}; {extra}", kind, target=t.summary(), **data)
        return True

    def cmd_return(self, d) -> bool:
        """H: one-way path from the actual current TCP back to the ready TCP (needs S, V, Y like any path)."""
        if self.state not in ("HOLD", "DONE", "ABORTED"):
            self._emit(f"H ignored in {self.state}", "refused")
            return False
        self._invalidate("return path")
        st = self.robot_state(d)
        try:
            self._set_path(return_path(st.tcp, st.q_rad, self.ready_pose, self.cfg))
        except PathGenerationError as exc:
            self._set("HOLD", f"return path failed: {exc}", "return_path")
            return False
        self.target = None
        self.goal = Goal(self.ready_pose.p_m, self.ready_pose.tool_axis, "ready")
        self.phase, self.move_kind, self._resume_dir = "return", "return_home", None
        self._path_builder = lambda st: return_path(st.tcp, st.q_rad, self.ready_pose, self.cfg)
        self._set("GENERATED", f"return path to ready from current TCP {np.round(st.tcp.p_m, 3).tolist()}",
                  "return_path")
        return True

    # ------------------------------------------------------------------ return phase and resume after a stop
    @staticmethod
    def _from_tcp(path_id: str, tcp, q_rad, P, A, stops, meta: dict) -> CartesianPath:
        """Path that starts exactly at the robot's actual TCP (start check) and then follows P[1:], A[1:]."""
        P, A = np.array(P, float), np.array(A, float)
        P[0], A[0] = tcp.p_m, tcp.tool_axis
        stops = [int(i) for i in stops]
        if len(P) > 2 and np.linalg.norm(P[1] - P[0]) < 1e-3:   # stopped right at a sample: no 0-length segment
            P, A = np.delete(P, 1, axis=0), np.delete(A, 1, axis=0)
            stops = [i - 1 if i > 1 else i for i in stops if i != 1]
        wp = tuple(sorted({0, len(P) - 1, *[i for i in stops if 0 < i < len(P)]}))
        return CartesianPath(path_id, "initial", WORLD, P, A, wp, np.asarray(q_rad, float).copy(), meta)

    def _reverse_of(self, path: CartesianPath, st) -> CartesianPath:
        n = len(path.points_m)
        return self._from_tcp("return_reverse", st.tcp, st.q_rad, path.points_m[::-1], path.tool_axis[::-1],
                              [n - 1 - i for i in path.waypoint_index], {"reverse_of": path.fingerprint})

    def _start_return(self, d, edit: bool) -> tuple[bool, str]:
        """After arriving at the target (outbound DONE): the executed (edited) path backwards, to edit or to run."""
        le, out = self.last_exec, self.outbound_exec
        if self.state != "DONE" or le is None or le["phase"] != "outbound" or out is None:
            self._emit(f"return refused in {self.state} (phase {self.phase})", "refused")
            return False, ("NOT_ALLOWED" if self.state != "DONE" else "NOT_AT_TARGET")
        if np.linalg.norm(self.robot_state(d).tcp.p_m - out.points_m[-1]) > self.cfg["validation"]["start_pos_tol_m"]:
            self._emit("return refused: the robot is not at the end of the outbound path", "refused")
            return False, "NOT_AT_TARGET"
        goal = Goal(out.points_m[0].copy(), out.tool_axis[0].copy(), "start")
        self._invalidate("return path")
        self._path_builder = lambda st: self._reverse_of(out, st)
        self._set_path(self._reverse_of(out, self.robot_state(d)))
        self.goal, self.phase = goal, "return"
        self.move_kind, self._resume_dir = ("return_edit" if edit else "return_reverse"), None
        if edit:
            self.edit_mode = True
        self._set("GENERATED", "return path = executed outbound path reversed" + ("; edit it" if edit else ""),
                  "return_path", edit=edit)
        return True, ""

    def _back_to_edit(self, which: str) -> None:
        """Arrived where the interrupted move started: continue editing that move (edits and undo kept)."""
        if which == "edit_outbound" and self.outbound_edit is not None:
            ed, goal, phase, kind = self.outbound_edit, self.outbound_goal, "outbound", "outbound"
            t = self.target
            self._path_builder = (lambda st: initial_path(st.tcp, st.q_rad, t, self.cfg, allow_unsafe=self.allow_unsafe)) \
                if t is not None else None
            msg = "back at the start: edit the outbound path again"
        elif which == "edit_return" and self.return_edit is not None and self.outbound_exec is not None:
            ed, goal, phase, kind = self.return_edit, self.return_goal, "return", "return_edit"
            out = self.outbound_exec
            self._path_builder = lambda st: self._reverse_of(out, st)
            msg = "back at the target: edit the return path again"
        else:
            return
        self.edit, self.path, self.goal, self.phase, self.move_kind = ed, ed.current_path(), goal, phase, kind
        self.pointed, hist = self._saved_pointed.get(kind, (None, []))
        self._resume_dir, self._pointed_hist = None, list(hist)
        self.edit_mode = True
        self._set("GENERATED", msg, "back_to_edit", which="outbound" if phase == "outbound" else "return")

    def save_paths(self, out_dir: Path) -> Path | None:
        """Write the outbound and return paths that were run (points, tool axes, stops) to one JSON file."""
        if self.outbound_exec is None:
            return None

        def dump(p):
            return None if p is None else {"path_id": p.path_id, "source": p.source, "frame": p.frame,
                                           "edits": int(p.meta.get("edits", 0)), "fingerprint": p.fingerprint,
                                           "points_m": np.round(p.points_m, 6).tolist(),
                                           "tool_axis": np.round(p.tool_axis, 6).tolist(),
                                           "waypoint_index": list(p.waypoint_index)}
        out_dir.mkdir(parents=True, exist_ok=True)
        f = out_dir / time.strftime("paths_%Y%m%d_%H%M%S.json")
        f.write_text(json.dumps({"saved": time.strftime("%Y-%m-%dT%H:%M:%S"), "model_sha": self.ctx.model_sha
                                 if hasattr(self.ctx, "model_sha") else None,
                                 "target": None if self.target is None else self.target.summary(),
                                 "outbound": dump(self.outbound_exec), "return": dump(self.return_exec)},
                                ensure_ascii=False, indent=1, default=_json_default), encoding="utf-8")
        self._emit(f"paths saved: {f.name}", "paths_saved", file=str(f))
        return f

    def cmd_return_reverse(self, d) -> tuple[bool, str]:
        return self._start_return(d, edit=False)

    def cmd_edit_return(self, d) -> tuple[bool, str]:
        return self._start_return(d, edit=True)

    def cmd_resume(self, d, forward: bool) -> tuple[bool, str]:
        """After a stop (ABORTED): from the actual TCP, the rest of the interrupted path (forward) or back along
        the part already done to where that path started (back). Back flips the phase."""
        le = self.last_exec
        if self.state != "ABORTED" or le is None:
            self._emit(f"resume refused in {self.state}", "refused")
            return False, "NOT_STOPPED"
        st = self.robot_state(d)
        # the whole move that was interrupted (also after several stops), not only the last resumed piece
        if self.move_kind == "outbound" and self.outbound_exec is not None:
            path, end_goal = self.outbound_exec, self.outbound_goal
        elif self.move_kind in ("return_reverse", "return_edit") and self.return_exec is not None:
            path, end_goal = self.return_exec, self.return_goal
        else:
            path, end_goal = le["path"], le["goal"]
        P, A = path.points_m, path.tool_axis
        seg = int(np.argmin(polyline_distance_per_segment(st.tcp.p_m, P)))      # segment P[seg] -> P[seg+1]
        if forward:
            keep = list(range(seg, len(P)))
            goal, phase = end_goal, le["phase"]
        else:
            keep = list(range(seg + 1, -1, -1))
            goal = Goal(P[0].copy(), A[0].copy(), "start")
            phase = "return" if le["phase"] == "outbound" else "outbound"
        pos = {i: k for k, i in enumerate(keep)}
        stops = [pos[i] for i in path.waypoint_index if i in pos]
        build = lambda s_: self._from_tcp("resume_forward" if forward else "resume_back", s_.tcp, s_.q_rad,
                                          P[keep], A[keep], stops, {"resumed_from": path.fingerprint})
        self._invalidate("resume")
        self._path_builder = build
        self._set_path(build(st))
        self.goal, self.phase = goal, phase
        self._resume_dir = "forward" if forward else "back"
        self.last_exec = None
        self._set("GENERATED", f"resume {'forward' if forward else 'back'} from the stop point", "return_path",
                  resume="forward" if forward else "back")
        return True, ""

    def cmd_select(self) -> bool:
        if self.state != "GENERATED" or self.path is None:
            self._emit(f"S ignored in {self.state}", "refused")
            return False
        if self.edit is not None and self.edit.dragging:
            self._emit("S ignored: release the drag first", "refused")
            return False
        kind = "unmodified" if self.path.source == "initial" else f"edited x{self.path.meta.get('edits', 0)}"
        self._set("SELECTED", f"selected {self.path.path_id} ({kind}, {len(self.path.points_m)} points); "
                  "V: request validation", "select")
        return True

    def cmd_validate(self, d, auto_run_request: str | None = None) -> bool:
        """V. auto_run_request (voice only): book an automatic cmd_approve if THIS validation passes."""
        if self.state not in ("SELECTED", "FAILED") or self.path is None:
            self._emit(f"V ignored in {self.state}", "refused")
            return False
        self._invalidate("new validation")
        token = self._token
        args = (self.ctx, self.path, d.qpos.copy(), d.qvel.copy(), float(d.time), self.goal)
        self.edit_mode = False
        self._set("VALIDATING", "validating (12 required checks incl. dry rollout)", "validate_start")
        if auto_run_request is not None:
            self.arm_auto_run(auto_run_request)
        if self._pool is None:
            self._accept(token, validate(*args))
        else:
            self._future = self._pool.submit(validate, *args)
            self._future.token = token
        return True

    def poll(self) -> None:
        f = self._future
        if f is None or not f.done():
            return
        self._future = None
        try:
            rep = f.result()
        except Exception as exc:
            if f.token == self._token and self.state == "VALIDATING":
                self.edit_mode = True
                self._set("FAILED", f"validation crashed: {exc}", "validation")
                self.cancel_auto_run("validation crashed")
            return
        self._accept(f.token, rep)

    def _accept(self, token: int, rep: ValidationReport) -> None:
        if token != self._token or self.state != "VALIDATING":
            return  # stale (path/target changed or cancelled meanwhile)
        self.report = rep
        if rep.execution_allowed:
            self._set("PASSED", f"{rep.summary()} (duration {rep.trajectory.duration:.2f} s); Y: approve & run",
                      "validation", report=rep.to_dict())
        else:
            self.edit_mode = True        # back to editing: the path stays, failures stay visible
            self._set("FAILED", f"{rep.summary()}; " + " | ".join(rep.blocking()[:3]), "validation",
                      report=rep.to_dict())
            self.cancel_auto_run("validation failed")

    def binding_problems(self, d) -> list[str]:
        rep, out = self.report, []
        if rep is None:
            return ["no report"]
        if self.path is None or rep.binding.path_fp != self.path.fingerprint:
            out.append("path changed")
        if rep.binding.model_sha != self.ctx.model_sha:
            out.append("model changed")
        if rep.binding.config_sha != config_sha(self.cfg):
            out.append("config changed")
        drift = float(np.max(np.abs(d.qpos[self.idx.qpos] - rep.binding.start_q_rad)))
        if drift > self.cfg["validation"]["approval_q_drift_rad"]:
            out.append(f"robot moved {drift:.4f} rad since validation")
        return out

    def cmd_approve(self, d) -> bool:
        """Y: the only way to start motion."""
        if self.state != "PASSED" or self.report is None or not self.report.execution_allowed:
            self._emit(f"Y refused: not validated ({self.state})" if self.state != "FAILED"
                       else "Y refused: validation failed - execution blocked", "refused")
            return False
        problems = self.binding_problems(d)
        if problems:
            self._regenerate(d, "approval refused, report invalidated: " + "; ".join(problems))
            return False
        self.executor = TrajectoryExecutor(self.m, self.idx, self.report.trajectory, self.cfg["control"],
                                           t0=float(d.time))
        self.auto_run = None
        self.edit_mode = False
        if self.path.path_id in ("to_target", "return_reverse"):     # keep the clicked point with its edits
            self._saved_pointed[self.move_kind] = (None if self.pointed is None else self.pointed.copy(),
                                                   list(self._pointed_hist))
        if self.move_kind == "outbound" and self.path.path_id == "to_target":
            self.outbound_exec, self.outbound_edit, self.outbound_goal = self.path, self.edit, self.goal
        elif self.move_kind in ("return_reverse", "return_edit") and self.path.path_id == "return_reverse":
            self.return_exec, self.return_goal = self.path, self.goal
            self.return_edit = self.edit if self.move_kind == "return_edit" else None
        self._set("EXECUTING", f"approved report {self.report.report_id}; executing "
                  f"{self.report.trajectory.duration:.2f} s trajectory", "execution_start",
                  report_id=self.report.report_id, path_fp=self.path.fingerprint)
        return True

    def _regenerate(self, d, why: str) -> None:
        """Robot state no longer matches: rebuild the same kind of path from the actual TCP."""
        self._invalidate(why)
        st = self.robot_state(d)
        try:
            edits = " (path edits discarded)" if self.path is not None and self.path.source == "edited" else ""
            if self._path_builder is not None:
                self._set_path(self._path_builder(st))
            elif self.path is not None and self.path.path_id == "return_ready":
                self._set_path(return_path(st.tcp, st.q_rad, self.ready_pose, self.cfg))
            elif self.target is not None:
                self._set_path(initial_path(st.tcp, st.q_rad, self.target, self.cfg, allow_unsafe=self.allow_unsafe))
            self._set("GENERATED", why + "; path regenerated from current TCP" + edits + ", select again",
                      "regenerated")
        except PathGenerationError as exc:
            self._set_path(None)
            self._set("HOLD", f"{why}; regeneration failed: {exc}", "regenerated")

    def cmd_cancel(self, d) -> str:
        """X. Returns what it stopped: drag / execution / validation / selection, or "" (nothing)."""
        if self.edit is not None and self.edit.dragging:
            self.edit_cancel()
            return "drag"
        if self.state == "EXECUTING":
            self.executor._abort("user stop (X)")
            self._finish_execution(d)
            return "execution"
        if self.state == "VALIDATING":
            self._invalidate("validation cancelled (X)")
            self._set("SELECTED", "validation cancelled", "cancel")
            return "validation"
        if self.path is not None and self.state in ("SELECTED", "PASSED", "FAILED"):
            self._invalidate("selection cleared (X)")
            self._set("GENERATED", "selection cleared", "cancel")
            return "selection"
        self.cancel_auto_run("stop")
        return ""

    def watch(self, d) -> None:
        """Called every frame: deliver validation results, start a booked voice auto-run, and drop a PASSED
        report when the robot drifted (e.g. pushed in the viewer)."""
        self.poll()
        self._maybe_auto_run(d)
        if self.state == "PASSED" and self.report is not None:
            drift = float(np.max(np.abs(d.qpos[self.idx.qpos] - self.report.binding.start_q_rad)))
            if drift > self.cfg["validation"]["approval_q_drift_rad"]:
                self._regenerate(d, f"robot moved {drift:.4f} rad after validation; report invalidated")

    # ------------------------------------------------------------------ path editing (world space)
    def can_edit(self) -> tuple[bool, str]:
        if self.edit is None or self.path is None:
            return False, "no path to edit"
        if self.state not in EDITABLE:
            return False, f"editing is not allowed in {self.state}" + (
                " (cancel with X first)" if self.state == "VALIDATING" else
                " (generate a new path with N or H)" if self.state in ("DONE", "ABORTED", "HOLD") else "")
        return True, ""

    def _edited(self, msg: str, kind: str = "edit", **data) -> None:
        """Any change of the path geometry: drop report/in-flight validation, back to GENERATED."""
        self._invalidate("path edited")
        self.path = self.edit.current_path()
        self._set("GENERATED", msg + "; validation/approval cleared - S, V, Y again", kind,
                  edits=int(self.path.meta.get("edits", 0)), **data)

    def edit_begin(self, index: int, control_point, free: bool = False) -> bool:
        """free=False: the point moves on its vertical line (mouse, 2-D hand). free=True: in 3-D (pinch + depth)."""
        ok, why = self.can_edit()
        if not ok:
            self._emit(f"edit refused: {why}", "refused")
            return False
        if not self.edit.begin(index, control_point, free=free):
            self._emit(f"edit not started: {self.edit.message}", "refused")
            return False
        self._drag_prev_pointed = None if self.pointed is None else self.pointed.copy()
        self.edit_mode = True
        if free:
            self._edited(f"dragging point {index} in 3-D (hand)", "edit_begin")
        else:
            ax, ay = self.edit.anchor_xy
            self._edited(f"dragging point {index} vertically at x={ax:.3f} y={ay:.3f} m (height only)", "edit_begin")
        return True

    def edit_update(self, control_point) -> bool:
        """control_point None = invalid ray / outside workspace: the last valid preview is kept."""
        if self.edit is None or not self.edit.dragging:
            return False
        ok = self.edit.update(control_point)
        self.path = self.edit.current_path()
        return ok

    def edit_end(self) -> bool:
        if self.edit is None or not self.edit.dragging:
            return False
        p_end = self.edit.selected_point()
        kept = self.edit.end()
        if kept and p_end is not None:
            # the dragged point (mouse now, pinch later) becomes voice "here / there"; undo puts the old one back
            self._pointed_hist.append((len(self.edit.editor.candidate_records), getattr(self, "_drag_prev_pointed", None)))
            self._pointed_redo = []
            self.pointed = p_end.copy()
        self._edited(("edit kept: " if kept else "edit discarded: ") + self.edit.message, "edit_end", kept=kept)
        return kept

    def edit_cancel(self) -> bool:
        if self.edit is None or not self.edit.dragging:
            return False
        self.edit.cancel()
        self._edited("drag cancelled", "edit_cancel")
        return True

    # ------------------------------------------------------------------ pointed point + voice nudge
    def pointed_index(self) -> int | None:
        """Interior index of the current path nearest to the pointed world point (it follows edits/undo)."""
        if self.pointed is None or self.path is None:
            return None
        P = self.path.points_m
        k = int(np.argmin(np.linalg.norm(P - self.pointed, axis=1)))
        return min(max(k, 1), len(P) - 2)

    def point_select(self, index: int) -> bool:
        """A click (press + release without moving) marks the point that voice "here" refers to."""
        ok, why = self.can_edit()
        if not ok:
            self._emit(f"point refused: {why}", "refused")
            return False
        P = self.path.points_m
        if not 0 < index < len(P) - 1:
            self._emit("point refused: endpoints are fixed", "refused")
            return False
        self.pointed = P[index].copy()
        x, y, z = self.pointed
        self._emit(f"point selected at x={x:.3f} y={y:.3f} z={z:.3f} m", "point_select", index=int(index),
                   point_m=[round(float(v), 4) for v in self.pointed])
        return True

    def region_index(self, region: str) -> int | None:
        """start / middle / end = 25 / 50 / 75 % of the path length (interior points only)."""
        frac = {"start": 0.25, "middle": 0.5, "end": 0.75}.get(region)
        if frac is None or self.path is None:
            return None
        P = self.path.points_m
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
        k = int(np.argmin(np.abs(s - frac * s[-1])))
        return min(max(k, 1), len(P) - 2)

    def edit_nudge(self, index: int, direction: str, dist_m: float) -> tuple[bool, str, float]:
        """Move path point `index` by dist_m along a screen direction (voice). Returns (ok, code, moved_m).

        The same correction algorithm as a drag (begin -> one update -> end), without the vertical
        constraint: the requested 3-D delta is applied at the point, neighbours follow smoothly.
        moved_m is the measured displacement of that point (endpoint protection / limits may shorten it).
        """
        ok, why = self.can_edit()
        if not ok:
            self._emit(f"edit refused: {why}", "refused")
            return False, "NOT_EDITABLE", 0.0
        if self.edit.dragging:
            return False, "DRAG_ACTIVE", 0.0
        axes = self.screen_axes or {}
        if direction not in axes:
            return False, "BAD_DIRECTION", 0.0
        p0 = self.path.points_m[index].copy()
        delta = axes[direction] * float(dist_m)
        if not self.edit.begin(index, p0, free=True):
            self._emit(f"edit not started: {self.edit.message}", "refused")
            return False, "REFUSED", 0.0
        self.edit.update(p0 + delta)
        p1 = self.edit.selected_point()
        # near the start/end the editor's endpoint protection moves the point less than asked. The field is
        # linear in the input, so scale the input once to move the point by exactly dist_m (2 / 5 / 8 cm stay
        # 2 / 5 / 8 cm). The input is capped at the editor's drag_limit; right at an endpoint it stays short.
        got = float(np.linalg.norm(p1 - p0)) if p1 is not None else 0.0
        if 1e-6 < got < 0.995 * float(dist_m):
            scale = min(float(dist_m) / got, self.cfg["editor"]["drag_limit"] / float(dist_m))
            self.edit.update(p0 + delta * scale)
            p1 = self.edit.selected_point()
        got = float(np.linalg.norm(p1 - p0)) if p1 is not None else 0.0
        if got < 0.995 * float(dist_m):
            # cannot move it by exactly the asked distance (too close to a fixed end / over the limits):
            # change nothing rather than move it part of the way
            self.edit.cancel()
            self._emit(f"voice nudge refused: {dist_m * 100:.1f} cm {direction} reaches only {got * 100:.1f} cm here",
                       "refused")
            return False, "TOO_FAR", got
        kept = self.edit.end()
        moved = float(np.linalg.norm(p1 - p0)) if kept and p1 is not None else 0.0
        if kept:
            self._pointed_hist.append((len(self.edit.editor.candidate_records), p0.copy()))
            self._pointed_redo = []
            self.pointed = p1.copy()          # "here" now is the moved point ("a bit more" moves it again)
            self.edit_mode = True
        self._edited(f"voice nudge {direction} {dist_m * 100:.1f} cm: moved {moved * 100:.1f} cm - " + self.edit.message
                     if kept else "voice nudge discarded: " + self.edit.message, "edit_end", kept=kept,
                     nudge=direction, requested_m=round(float(dist_m), 4), moved_m=round(moved, 4))
        return kept, ("" if kept else "NO_CHANGE"), moved

    def _edit_history(self, name: str) -> bool:
        ok, why = self.can_edit()
        if not ok or self.edit.dragging:
            self._emit(f"{name} refused: {why or 'release the drag first'}", "refused")
            return False
        n_before = len(self.edit.editor.candidate_records)
        if not getattr(self.edit, name)():
            self._emit(f"{name}: {self.edit.message}", "refused")   # e.g. nothing to undo: state unchanged
            return False
        # undoing a voice nudge puts "here" back where it was (not at the nearest point of the old path)
        if name == "undo" and self._pointed_hist and self._pointed_hist[-1][0] == n_before:
            n_after, p_before = self._pointed_hist.pop()
            self._pointed_redo.append((n_after, p_before, self.pointed))
            self.pointed = p_before
        elif name == "redo" and self._pointed_redo and \
                self._pointed_redo[-1][0] == len(self.edit.editor.candidate_records):
            n_after, p_before, p_after = self._pointed_redo.pop()
            self._pointed_hist.append((n_after, p_before))
            self.pointed = p_after
        elif name == "reset" and self._pointed_hist:
            self.pointed = self._pointed_hist[0][1]
            self._pointed_hist.clear()
        self._edited(f"{name}: {self.edit.message}", name)
        return True

    # ------------------------------------------------------------------ snapshot for the voice process
    def status(self) -> dict:
        ed, rep, t = self.edit, self.report, self.target
        out = {"state": self.state, "rev": self.rev, "edit_mode": self.edit_mode, "message": self.message,
               "path_fp": self.path.fingerprint if self.path is not None else None,
               "path_source": self.path.source if self.path is not None else None,
               "path_id": self.path.path_id if self.path is not None else None,
               "edits": int(self.path.meta.get("edits", 0)) if self.path is not None else 0,
               "dragging": bool(ed is not None and ed.dragging),
               "pointed": self.pointed_index() is not None,
               "phase": self.phase,
               "last_exec": None if self.last_exec is None else
               {"phase": self.last_exec["phase"], "status": self.last_exec["status"],
                "kind": self.last_exec.get("kind")},
               "can_undo": bool(ed is not None and ed.editor.can_undo),
               "can_redo": bool(ed is not None and ed.editor.can_redo),
               "validation_token": self._token,
               "auto_run": None if self.auto_run is None else dict(self.auto_run.__dict__),
               "target": None if t is None else {"label": self.target_label, "r_m": round(float(t.r_m), 4),
                                                  "theta_deg": round(float(np.degrees(t.theta_rad)), 2)},
               "report": None, "last_run": None}
        if rep is not None:
            out["report"] = {"summary": rep.summary(), "execution_allowed": rep.execution_allowed,
                             "blocking": rep.blocking()[:3],
                             "failed_kinds": [c.kind for c in rep.checks if c.status != "PASS"]}
        if self.last_run is not None:
            out["last_run"] = {k: self.last_run.get(k) for k in ("status", "reason", "path_source", "path_fp")}
        return out

    def handle_key(self, key: str, d) -> None:
        key = key.upper()
        {"N": lambda: self.cmd_new_target(d), "S": self.cmd_select, "V": lambda: self.cmd_validate(d),
         "Y": lambda: self.cmd_approve(d), "H": lambda: self.cmd_return(d),
         "X": lambda: self.cmd_cancel(d), "U": lambda: self._edit_history("undo"),
         "R": lambda: self._edit_history("redo"), "O": lambda: self._edit_history("reset")}.get(key, lambda: None)()

    def close(self) -> None:
        if self._pool:
            self._pool.shutdown(wait=False, cancel_futures=True)


def polyline_distance_per_segment(p, P) -> np.ndarray:
    """Distance from point p to each segment P[i] -> P[i+1]."""
    a, b = P[:-1], P[1:]
    ab = b - a
    t = np.clip(np.sum((p - a) * ab, axis=1) / np.maximum(np.sum(ab * ab, axis=1), 1e-18), 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * ab - p, axis=1)


def _json_default(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return str(x)
