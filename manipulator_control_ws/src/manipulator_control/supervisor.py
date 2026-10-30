"""State machine for the preparation routine. Deterministic: step(now, fast, slow, events) -> commands.

It runs inside the bus thread (runner.py); the UI and the viewer only read view(). The state is never "stopped"
or "holding" because a command was sent: those states are entered only after the servo state read back
confirms it (velocity ~0, Moving = 0, Profile Ongoing = 0 for stop.settle_s, torque on).

States:  calibrate:        CONNECTED_CHECK -> [MODE_PREP] -> ALIGN (joint 1..6, one at a time) -> HOLDING
         prepare / teleop: CONNECTED_CHECK -> stored calibration verified (calibration.Calibration.resume) ->
                           [RESUME_HOLD: joints found with torque off are held where they are, after y] ->
                           HOME_PREVIEW (y) -> COUNTDOWN -> HOMING -> READY [-> TELEOP]
         pinch:            as prepare, then READY -> LINK_WAIT (simulator not connected) -> ARM_CONFIRM (y at the
                           terminal) -> ARMED -> TRAJ_CHECK (the received trajectory is checked again here) ->
                           TRAJ_MOVING (streamed, pinch_traj.py) -> ARMED. A terminal / panel stop disarms (HOLDING,
                           y re-arms); a simulator stop (X, voice) keeps ARMED; a lost link -> LINK_WAIT.
         STOPPING -> HOLDING;  FAULT (latched), COMM_LOST (latched, motion unknown), RELEASED (torque off by the
         user), MONITOR / CALIBRATING (watch, read-only), EXITED.

Only calibrate aligns joints. It deletes the stored calibration when it starts and stores its own once every joint
is held; prepare / teleop start from that record (no alignment) and refuse without writing anything when it does
not verify. Every confirmed hold and the exit store the evidence again (SaveCalibration); evidence of a reset
deletes it (DiscardCalibration).

ALIGN, per joint from the base up: [direction check, only while its sign is unknown: the SIM sweeps the joint's +
direction, the user turns the real joint that way a little, the encoder change gives s, the user confirms with y]
-> the user lines the joint up with the reference pose shown in the SIM and presses Enter -> still window ->
P_ref of that joint -> Goal = present (read back) -> torque on for that joint only (read back) -> 0.6 s no-jump
check. The joint then holds where it was aligned (no motion) and the next joint starts.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import mujoco
import numpy as np

from . import control_table as ct
from .calibration import Calibration, CalibrationWindow, Reject
from .config import Config, calibrated_view_ready, motion_blockers
from .pinch_traj import Acceptance, Trajectory, stream_command
from .pinch_traj import check as traj_check
from .planner import HomePlan, braking_target, plan_home
from .state import FastSnapshot, SlowSnapshot
from .units import JointMap, pa_unit_rad_s2, pv_counts_s, pv_unit_rad_s


class S(str, Enum):
    CONNECTED_CHECK = "CONNECTED_CHECK"
    MODE_PREP = "MODE_PREP"
    ALIGN = "ALIGN"
    RESUME_HOLD = "RESUME_HOLD"
    CALIBRATING = "CALIBRATING"
    COUNTDOWN = "COUNTDOWN"
    HOMING = "HOMING"
    HOME_PREVIEW = "HOME_PREVIEW"
    READY = "READY"
    TELEOP = "TELEOP"
    STOPPING = "STOPPING"
    HOLDING = "HOLDING"
    RELEASED = "RELEASED"
    MONITOR = "MONITOR"
    LINK_WAIT = "LINK_WAIT"
    ARM_CONFIRM = "ARM_CONFIRM"
    ARMED = "ARMED"
    TRAJ_CHECK = "TRAJ_CHECK"
    TRAJ_MOVING = "TRAJ_MOVING"
    FAULT = "FAULT"
    COMM_LOST = "COMM_LOST"
    EXITED = "EXITED"


PINCH_IDLE_STATES = {S.LINK_WAIT, S.ARM_CONFIRM, S.ARMED, S.TRAJ_CHECK}                          # held, not moving
TORQUE_STATES = {S.HOME_PREVIEW, S.COUNTDOWN, S.HOMING, S.READY, S.STOPPING, S.HOLDING, S.TELEOP,
                 S.TRAJ_MOVING} | PINCH_IDLE_STATES                                                # all held
MOVING_STATES = {S.HOMING, S.STOPPING, S.TELEOP, S.TRAJ_MOVING}


@dataclass(frozen=True)
class Event:
    kind: str        # START STOP QUIT TORQUE_OFF CONFIRM CANCEL NEW_SESSION RETRY | JOG STEP HOME TELEOP TARGET
    t: float         #   TARGET_RESET TORQUE_OFF_NOW (panel, already confirmed in its dialog)
    arg: object = None   # JOG "+x".."-rz" (terminal keys: base frame, session step); STEP "up"/"down";
    #                      TARGET (pos (3,), quat (4,)): the target dragged with the SIM gizmo (base frame)
    #                      link: LINK_UP (peer hello) / LINK_DOWN (reason) / EXT_STOP (reason) / TRAJ (pinch_traj.Trajectory)
    source: str = "terminal"   # terminal | ui (panel, viewer) | arm_dialog | script (--fake, tests) | link (pinch simulator)


LINK_KINDS = frozenset({"LINK_UP", "LINK_DOWN", "EXT_STOP", "TRAJ"})
OPERATOR_SOURCES = frozenset({"terminal", "script"})   # only these confirm (y) or start (Enter)
# 2026-10-29 user decision: the pinch arming (ARM_CONFIRM y) may also come from this program's own small arming
# dialog (panel.py, same process and machine as the terminal). It confirms ONLY the arming - never HOME, a torque-on
# or a START; the simulator, the browser and the panel's other buttons still cannot confirm anything.
ARM_DIALOG_SOURCE = "arm_dialog"


@dataclass(frozen=True)
class SyncWrite:
    addr: int
    length: int
    values: dict
    label: str


@dataclass(frozen=True)
class Write:
    dxl_id: int
    addr: int
    data: bytes
    label: str


@dataclass(frozen=True)
class FlushInput:
    pass


@dataclass(frozen=True)
class SlowReadNow:
    pass


@dataclass(frozen=True)
class SaveSigns:
    signs: dict          # joint name -> +1 / -1, confirmed by the user in this session


@dataclass(frozen=True)
class SaveCalibration:
    record: dict         # calibration.Calibration.record(): replaces the stored calibration
    why: str


@dataclass(frozen=True)
class DiscardCalibration:
    why: str


@dataclass(frozen=True)
class LinkSend:
    msg: dict            # to the pinch simulator (ack / result); the runner hands it to the link


@dataclass
class View:
    state: str = ""
    message: str = ""          # Korean, for the terminal
    overlay: str = ""          # ASCII, for the viewer (its font has no Hangul)
    calibrated: bool = False
    stale: bool = True
    q_meas: np.ndarray | None = None
    q_cmd: np.ndarray | None = None     # servo internal target (Position Trajectory), calibrated only
    q_target: np.ndarray | None = None  # reference pose while waiting, HOME while moving
    rel_deg: np.ndarray | None = None   # uncalibrated: joint change since start, s = +1 assumed
    axis_status: dict = field(default_factory=dict)
    joint_status: list | None = None   # ALIGN only: per joint "held" | "current" | "current_live" | "free"
    progress: str = ""                 # ALIGN / TELEOP: one Korean line the terminal repeats every second
    target_pose: tuple | None = None   # TELEOP: EE target (pos (3,), quat (4,)) for the viewer's marker
    target_ok: bool = True
    warnings: list = field(default_factory=list)
    fault: dict | None = None
    final_report: dict | None = None
    armed: bool = False                # pinch: simulator approvals move the arm
    execution: dict | None = None      # pinch: {traj_id, t_s, T_s, move} while a trajectory runs


@dataclass
class Fault:
    code: str
    message: str
    detail: dict = field(default_factory=dict)
    unknown_axes: list = field(default_factory=list)


class Supervisor:
    def __init__(self, cfg: Config, purpose: str, log, fk=None, path_checker=None, ik=None, saved: dict | None = None,
                 acceptance: Acceptance | None = None, check_submit=None, traj_path_checker=None,
                 speed_scale: float = 1.0, check_only: bool = False):
        if purpose not in ("watch", "hold", "home", "teleop", "pinch"):
            raise ValueError(purpose)
        if purpose == "pinch" and acceptance is None:
            raise ValueError("pinch needs the acceptance limits")
        # pinch: check_submit(fn) -> future (a worker thread, so a long model check never blocks the bus loop);
        # None runs the check at once (tests). traj_path_checker: a checker for that thread (its own MjData).
        self.acceptance, self.check_submit = acceptance, check_submit
        self.traj_path_checker = traj_path_checker or path_checker
        self.speed_scale, self.check_only = float(speed_scale), bool(check_only)
        self.link_up, self.peer = False, None
        self.pinch: dict | None = None             # the trajectory being moved (pinch)
        self.prev_profile: dict[int, tuple[int, int] | None] = {}   # PA/PV before the last write (streaming)
        self.cfg, self.purpose, self.log = cfg, purpose, log
        self.saved = saved                          # the stored calibration record as loaded / last saved
        self.written: dict[int, tuple[int, int, int]] = {}   # last (PA, PV, Goal) sent per servo = the evidence
        self.fk, self.path_checker, self.ik = fk, path_checker, ik
        self.teleop: dict | None = None
        self.ids = cfg.ids
        self.maps = tuple(j.joint_map() for j in cfg.joints) if all(j.sign is not None for j in cfg.joints) else None
        self.state = S.CONNECTED_CHECK
        self.entry_t = 0.0
        self.now = 0.0
        self.sub = None
        self.ctx: dict = {}
        self.fast: FastSnapshot | None = None
        self.prev_fast: FastSnapshot | None = None
        self.slow: SlowSnapshot | None = None
        self.check_slow: SlowSnapshot | None = None
        self.last_complete_t: float | None = None
        self.cal: Calibration | None = None
        self.plan: HomePlan | None = None
        self.expected_profile: dict[int, tuple[int, int]] = {}
        self.torque_ids: set[int] = set()           # joints this program turned on (and still expects on)
        self.torque_on_t: dict[int, float] = {}
        self.align: dict | None = None
        self.fault: Fault | None = None
        self.warnings: list[str] = []
        self.pending_confirm: str | None = None
        self.quit_requested = False
        self.torque_off_after_stop = False       # panel torque off while moving: stop first
        self.rel_zero: dict[int, int] | None = None
        self.retries = 0
        self.models: dict[int, int | None] = {}
        self.final_report: dict | None = None
        self._out: list = []
        self._msg = ""

    # ================================================================ public
    def on_connect(self, now: float, models: dict[int, int | None], slow: SlowSnapshot, fast: FastSnapshot) -> None:
        self.now, self.models, self.slow, self.check_slow = now, dict(models), slow, slow
        self._take_fast(fast)
        self._enter(S.CONNECTED_CHECK)

    def step(self, now: float, fast: FastSnapshot | None, slow: SlowSnapshot | None, events) -> list:
        self.now = now
        self._out = []
        if slow is not None:
            self.slow = slow
        if fast is not None:
            self._take_fast(fast)
        events = [e for e in events if self._accept_event(e)]
        if self.purpose == "pinch":
            events = self._link_bookkeeping(events)
        if self.state == S.EXITED:
            if slow is not None or fast is not None:
                self.final_report = self._final_report()     # keep the exit report on the newest reads
            return []
        self._check_comm()
        self._check_invalidation(slow)
        if any(e.kind == "TORQUE_OFF_NOW" for e in events):
            events = [e for e in events if e.kind != "TORQUE_OFF_NOW"]
            if self._torque_off_now():
                return self._out
        handler = getattr(self, f"_on_{self.state.value.lower()}")
        handler(events)
        return self._out

    @property
    def done(self) -> bool:
        return self.state == S.EXITED

    @property
    def torque_expected(self) -> bool:
        return bool(self.torque_ids)

    def view(self) -> View:
        v = View(state=self.state.value, message=self._msg, calibrated=self.cal is not None, stale=self._stale(),
                 warnings=list(self.warnings[-5:]), final_report=self.final_report)
        v.fault = None if self.fault is None else {"code": self.fault.code, "message": self.fault.message,
                                                     "unknown_axes": self.fault.unknown_axes}
        f = self.fast
        if f is not None and f.axes:
            if self.cal is not None and all(i in f.axes for i in self.ids):
                v.q_meas = self.cal.q(f)
                if self.state in TORQUE_STATES or (self.state == S.FAULT and self.torque_expected):
                    v.q_cmd = self.cal.q(f, "position_trajectory")
            elif self.rel_zero is not None and all(i in f.axes for i in self.ids):
                v.rel_deg = np.array([math.degrees((f[i].present_position - self.rel_zero[i]) / _k(j.gear))
                                      for i, j in zip(self.ids, self.cfg.joints)])
        if self.state in (S.HOMING, S.READY, S.STOPPING) and self.plan is not None:
            v.q_target = np.array([a.q1 for a in self.plan.axes])
        elif all(j.q_ref_rad is not None for j in self.cfg.joints):
            v.q_target = np.array([j.q_ref_rad for j in self.cfg.joints])
        if self.purpose == "pinch":
            self._pinch_view(v)
        if self.state == S.ALIGN:
            self._align_view(v)
        elif self.state == S.RESUME_HOLD:
            v.q_target = None
        elif self.state == S.HOME_PREVIEW and self.plan is not None:
            v.q_target = self.plan.predict((self.now - self.entry_t) % (self.plan.T_max + 1.5))
        elif self.state == S.TELEOP and self.teleop is not None:
            self._teleop_view(v)
        v.axis_status = self._axis_status()
        v.overlay = self._overlay(v)
        return v

    # ================================================================ plumbing
    def _take_fast(self, fast: FastSnapshot) -> None:
        self.prev_fast, self.fast = self.fast, fast
        if fast.complete:
            self.last_complete_t = fast.t
        if self.rel_zero is None and fast.complete:
            self.rel_zero = {i: fast[i].present_position for i in self.ids}

    def _accept_event(self, e: Event) -> bool:
        if (e.source == "link") != (e.kind in LINK_KINDS):
            self.log.event("input_discarded", kind=e.kind, source=e.source, reason="link kinds only from the link")
            return False
        if e.kind in ("START", "CONFIRM") and e.source not in OPERATOR_SOURCES and not (
                e.kind == "CONFIRM" and e.source == ARM_DIALOG_SOURCE and self.state == S.ARM_CONFIRM):
            self.log.event("input_discarded", kind=e.kind, source=e.source, reason="not the operator's terminal")
            return False
        if e.t < self.entry_t and e.kind in ("START", "CONFIRM"):
            self.log.event("input_discarded", kind=e.kind, reason="before state entry", state=self.state.value)
            return False
        return True

    def _enter(self, state: S, message: str = "", **detail) -> None:
        prev = self.state
        self.state, self.entry_t, self.sub, self.ctx = state, self.now, None, {}
        self._msg = message
        self.log.event("state", frm=prev.value, to=state.value, message=message, **detail)
        if state in (S.ALIGN, S.MODE_PREP, S.RESUME_HOLD):
            self._out.append(FlushInput())
        if state in (S.HOLDING, S.READY, S.ARMED, S.LINK_WAIT):
            self._store_hold(f"{state.value} confirmed")
        if state == S.EXITED:
            self.final_report = self._final_report()
            self.log.event("final_state", **self.final_report)
            self._store_exit()

    def _emit(self, cmd) -> None:
        self._out.append(cmd)
        if isinstance(cmd, (SyncWrite, Write)):
            vals = cmd.values if isinstance(cmd, SyncWrite) else {cmd.dxl_id: cmd.data}
            self.log.event("command_sent", label=cmd.label, addr=cmd.addr,
                           values={i: _decode_write(cmd.addr, d) for i, d in vals.items()})

    def _stale(self) -> bool:
        return self.last_complete_t is None or self.now - self.last_complete_t > self.cfg.m("loop", "stale_after_s")

    def _fresh_complete(self, max_age: float | None = None) -> FastSnapshot | None:
        f = self.fast
        max_age = max_age if max_age is not None else 2.5 / self.cfg.m("loop", "read_hz")
        if f is None or not f.complete or self.now - f.t > max_age:
            return None
        return f

    def _fault(self, code: str, message: str, unknown_axes=(), **detail) -> None:
        self._pinch_result("FAULT", message, code)
        self.fault = Fault(code, message, detail, list(unknown_axes))
        self.torque_off_after_stop = False
        self.cal = None if code in INVALIDATING else self.cal
        if code in RESETTING and self.purpose != "watch":
            self._discard(f"{code}: {message}")
        self._enter(S.FAULT, message, code=code, **detail)

    # ================================================================ stored calibration
    def _hold_record(self) -> dict | None:
        """The record to store now (PA/PV/Goal as read back), or None when a servo's PA/PV is not what this program
        wrote last (reset evidence). Goal is not compared: the servo moves it itself while torque is off."""
        f = self._fresh_complete(max_age=self.cfg.m("loop", "stale_after_s"))
        if self.cal is None or f is None or self.check_slow is None or set(self.written) != set(self.ids):
            return None
        if any((f[i].profile_acceleration, f[i].profile_velocity) != self.written[i][:2] for i in self.ids):
            return None
        return self.cal.record(self.check_slow.axes, {i: (f[i].profile_acceleration, f[i].profile_velocity,
                                                          f[i].goal_position) for i in self.ids})

    def _store_hold(self, why: str) -> None:
        if self.purpose == "watch":
            return
        rec = self._hold_record()
        if rec is not None:
            self.saved = rec
            self._out.append(SaveCalibration(rec, why))

    def _store_exit(self) -> None:
        """At exit: store the evidence if the servos hold the PA/PV this program wrote (torque on or off); after a
        lost link or without that evidence the calibration is gone (the next run needs calibrate)."""
        if self.purpose == "watch":
            return
        rec = self._hold_record() if self._comm_ok() else None
        if rec is not None:
            self.saved = rec
            self._out.append(SaveCalibration(rec, "exit"))
        else:
            self._discard("exit without a verified calibration (" + (
                "no communication" if not self._comm_ok() else "no calibration" if self.cal is None else
                "PA/PV read back != last written") + ")")

    def _discard(self, why: str) -> None:
        self.saved = None
        self._out.append(DiscardCalibration(why))

    def _warn(self, text: str) -> None:
        if text not in self.warnings:
            self.warnings.append(text)
            self.log.event("warning", text=text)

    # ================================================================ global checks
    def _check_comm(self) -> None:
        if self.state in (S.COMM_LOST, S.EXITED, S.CONNECTED_CHECK) or self.last_complete_t is None:
            return
        gap = self.now - self.last_complete_t
        if gap <= self.cfg.m("loop", "comm_fault_after_s"):
            return
        moving = self.state in MOVING_STATES or self.state == S.COUNTDOWN
        torque = bool(self.torque_ids)
        if self.state == S.TRAJ_MOVING:
            msg = (f"COMM LOST / MOTION UNKNOWN: 궤적 실행 중 통신이 끊겼습니다. 서보는 마지막 목표(궤적의 최대 "
                   f"{self.cfg.motion['pinch']['lead_s']:.2f} s 앞)까지 가서 스스로 멈춥니다. PC는 지금 정지시킬 수 없습니다. "
                   "위험하면 12V 전원 스위치를 끄세요 (팔이 자중으로 처집니다).")
        elif moving:
            msg = ("COMM LOST / MOTION UNKNOWN: 통신이 끊겼습니다. HOME 명령이 이미 전달됐다면 서보는 내부 프로파일로 "
                   "HOME까지 계속 이동한 뒤 멈출 수 있습니다. PC는 지금 정지시킬 수 없습니다. 위험하면 12V 전원 스위치를 끄세요 "
                   "(토크가 사라져 팔이 자중으로 처집니다).")
        elif torque:
            msg = ("COMM LOST: 통신이 끊겼습니다. 서보는 마지막 목표를 유지 중일 수 있으나 확인할 수 없습니다 (상태 미확인). "
                   "출력 차단은 12V 전원 스위치뿐입니다.")
        else:
            msg = "COMM LOST: 통신이 끊겼습니다 (토크 OFF 상태였음). 상태 미확인."
        was = self.state.value
        self.cal = None
        if self.purpose != "watch":
            self._discard("communication lost: calibrate again (user rule)")
            msg += " 저장된 보정은 지웠습니다: 다시 calibrate가 필요합니다."
        self.fault = Fault("COMM_LOST", msg, {"gap_s": round(gap, 3), "state_before": was}, list(self.ids))
        self._pinch_result("FAULT", "실물 통신 끊김 (COMM LOST)", "COMM_LOST", "COMM_LOST")
        self._enter(S.COMM_LOST, msg, gap_s=round(gap, 3), state_before=was, motion_possible=moving)

    def _check_invalidation(self, slow: SlowSnapshot | None) -> None:
        """Evidence that the servo state no longer matches the calibration (reboot, power reset, shutdown, ...)."""
        if self.state in (S.COMM_LOST, S.EXITED, S.CONNECTED_CHECK, S.FAULT, S.RELEASED):
            return
        evidence = []
        f, p = self.fast, self.prev_fast
        held = self.torque_ids
        if f is not None and p is not None and f.seq != p.seq:
            dt = f.t - p.t
            for i in self.ids:
                if i not in f.axes or i not in p.axes:
                    continue
                a, b = f[i], p[i]
                dtick = (a.realtime_tick - b.realtime_tick) % 32768
                if dt < 30 and abs(dtick - dt * 1000) > 150:
                    evidence.append(f"ID {i} Realtime Tick 불연속 ({dtick} ms vs {dt * 1000:.0f} ms)")
                jump_lim = max(400, 2 * 5300 * dt) if i in held else max(4000, 20000 * dt)
                if abs(a.present_position - b.present_position) > jump_lim:
                    evidence.append(f"ID {i} 위치 점프 {a.present_position - b.present_position} count")
                if a.alert and not b.alert:
                    evidence.append(f"ID {i} 하드웨어 경보(상태 패킷 alert)")
        if f is not None and self.expected_profile and held:
            for i, (pa, pv) in self.expected_profile.items():
                read = (f[i].profile_acceleration, f[i].profile_velocity) if i in f.axes else None
                if self.state == S.TRAJ_MOVING and read == self.prev_profile.get(i):
                    continue            # the newest streamed write was lost: _on_traj_moving stops (no reset evidence)
                if i in held and i in f.axes and read != (pa, pv):
                    evidence.append(f"ID {i} PV/PA 읽기값 {f[i].profile_velocity}/{f[i].profile_acceleration} != "
                                    f"보낸 값 {pv}/{pa} (재부팅 징후)")
        if slow is not None and self.check_slow is not None:
            for i in self.ids:
                if i not in slow.axes or i not in self.check_slow.axes:
                    continue
                a, ref = slow.axes[i], self.check_slow.axes[i]
                for name in ("operating_mode", "drive_mode", "homing_offset"):
                    if a[name] != ref[name] and not (name == "operating_mode" and self.ctx.get("mode_written")):
                        evidence.append(f"ID {i} {name} {ref[name]} -> {a[name]}")
                if a["hardware_error_status"] and a["hardware_error_status"] != ref["hardware_error_status"]:
                    bits = [k for k, on in ct.hardware_error_bits(a["hardware_error_status"]).items() if on]
                    evidence.append(f"ID {i} Hardware Error {bits}")
                if i in held and not a["torque_enable"] and slow.t > self.torque_on_t.get(i, math.inf):
                    evidence.append(f"ID {i} 토크가 예기치 않게 OFF (서보 보호 정지 또는 재부팅) - 이 축은 유지 중이 아님")
        if not evidence:
            return
        self.log.event("invalidation", evidence=evidence, state=self.state.value)
        msg = "보정 무효: " + "; ".join(evidence)
        if self.purpose == "watch":           # read-only tool keeps running; only the calibrated view is dropped
            self.cal = None
            for ev in evidence:
                self._warn(ev)
            if self.state == S.CALIBRATING:
                self._enter(S.MONITOR, msg)
            return
        if self.state == S.STOPPING:
            if self.ctx["then"][:2] != ("FAULT", "INVALIDATED"):
                self.ctx["then"] = ("FAULT", "INVALIDATED", msg)
            return
        if self.state in (S.HOMING, S.TELEOP, S.TRAJ_MOVING) and self._comm_ok():
            self._begin_stop("brake", then=("FAULT", "INVALIDATED", msg), reason=msg)
        else:
            self._fault("INVALIDATED", msg, evidence=evidence)

    def _comm_ok(self) -> bool:
        return self.last_complete_t is not None and self.now - self.last_complete_t <= self.cfg.m("loop", "stale_after_s")

    # ================================================================ states
    def _on_connected_check(self, events) -> None:
        if "await_slow" in self.ctx:
            if self.slow is None or self.slow.t <= self.ctx["await_slow"] or not self.slow.complete:
                if self.now - self.ctx["await_slow"] > 2.0:
                    self._fault("COMM", "설정 다시 읽기 실패 (응답 없음)")
                else:
                    self._out.append(SlowReadNow())
                return
        slow = self.slow
        self.check_slow = slow
        problems, notes = servo_setting_problems(self.cfg, slow, self.models)
        for n in notes:
            self._warn(n)
        if self.purpose == "watch":
            for pr in problems:
                self._warn("prepare 거부 사유(읽기 전용 도구는 계속 동작): " + pr)
            self._enter(S.MONITOR, self._monitor_message())
            return
        blockers = motion_blockers(self.cfg, self.purpose)
        if blockers:
            self._fault("CONFIG", "설정 미확정으로 거부 (토크 ON·이동 없음): " + "; ".join(blockers), blockers=blockers)
            return
        if problems:
            self._fault("SERVO_SETTINGS", "서보 설정 때문에 prepare 거부 (자동으로 고치지 않음): " + "; ".join(problems),
                        problems=problems)
            return
        wrong_mode = [i for i in self.ids if slow.value(i, "operating_mode") != self.cfg.expect("operating_mode")]
        torque_on = [i for i in self.ids if slow.value(i, "torque_enable")]
        if wrong_mode and self.purpose != "hold":
            self._fault("MODE", f"Operating Mode != 4: ID {wrong_mode}. 저장된 보정은 무효입니다 (모드가 바뀌면 PV/PA가 "
                                "초기화됨). calibrate에서 모드를 맞추고 다시 등록하세요", ids=wrong_mode)
        elif wrong_mode and not self.cfg.expect("allow_operating_mode_write"):
            self._fault("MODE", f"Operating Mode != 4: ID {wrong_mode}. config allow_operating_mode_write=false라 "
                                "바꾸지 않습니다 (Wizard 또는 설정 허용 후 MODE_PREP)", ids=wrong_mode)
        elif wrong_mode:
            self._enter(S.MODE_PREP, f"Operating Mode가 4가 아닌 ID {wrong_mode}. 팔을 손·지그로 받친 뒤 y: 토크 OFF 후 "
                                     "모드 4로 변경 (EEPROM 쓰기). 다른 키: 취소", ids=wrong_mode)
            self.ctx["ids"] = wrong_mode
        else:
            for i in torque_on:              # already held (e.g. an earlier calibrate ended with q): keep holding
                self.torque_ids.add(i)
                self.torque_on_t[i] = self.now
            if torque_on:
                self.log.event("torque_found_on", ids=torque_on, note="kept on" + (
                    "; re-registered at its alignment step" if self.purpose == "hold" else ""))
            if self.purpose == "hold":
                self._discard("new calibrate started: the previous calibration is dropped")
                self._start_align()
            else:
                self._start_resume()

    def _on_mode_prep(self, events) -> None:
        if self._quit_if(events):
            return
        ids = self.ctx["ids"]
        if self.sub is None:
            if any(e.kind == "CONFIRM" for e in events):
                self._emit(SyncWrite(64, 1, {i: b"\x00" for i in self.ids}, "torque_off_for_mode"))
                for i in ids:
                    self._emit(Write(i, 11, bytes([self.cfg.expect("operating_mode")]), "operating_mode_write"))
                self._out.append(SlowReadNow())
                self.ctx.update(t=self.now, mode_written=True)
                self.sub = "verify"
            elif any(e.kind == "CANCEL" for e in events):
                self._fault("MODE", "모드 변경을 취소했습니다", ids=ids)
            return
        s = self.slow
        if s is not None and s.t > self.ctx["t"] and s.complete and \
                all(s.value(i, "operating_mode") == self.cfg.expect("operating_mode") for i in self.ids):
            self.log.event("mode_changed", ids=ids, note="모드 변경으로 게인은 기본값, PV/PA는 0으로 초기화됨 - 토크 ON 전에 다시 씀")
            self.check_slow = s
            self._discard("new calibrate started (after an Operating Mode change)")
            self._start_align()
        elif self.now - self.ctx["t"] > 2.0:
            self._fault("MODE", "Operating Mode 4 변경이 확인되지 않음", ids=ids)
        else:
            self._out.append(SlowReadNow())

    def _on_calibrating(self, events) -> None:
        if self._quit_if(events):
            return
        for e in events:
            if e.kind == "START":
                self.log.event("input_ignored", kind="START", reason="already calibrating")
        win: CalibrationWindow = self.ctx["window"]
        win.add(self.fast)
        if not win.full:
            if self.now - self.entry_t > 3.0 + win.n / self.cfg.m("loop", "read_hz"):
                self._reject("완전한 샘플을 모으지 못함 (응답 누락 반복)")
            return
        problems = self._calibration_settings_problems()
        result = Reject("; ".join(problems)) if problems else win.evaluate(
            self.maps, [j.q_ref_rad for j in self.cfg.joints], self.now)
        if isinstance(result, Reject):
            self._reject(result.reason)
            return
        self.cal = result
        self.retries = 0
        self.log.event("calibration", **result.to_log())
        self.log.session(calibration=result.to_log())
        self._enter(S.MONITOR, "보정됨(읽기 전용): SIM에 실제 각도를 표시합니다. q: 종료")

    def _reject(self, reason: str) -> None:
        self.retries += 1
        self.log.event("calibration_rejected", reason=reason, retry=self.retries)
        self._enter(S.MONITOR, f"기준 등록 실패: {reason}. 정렬을 확인하고 다시 Enter "
                            f"(시도 {self.retries}/{self.cfg.m('calibration', 'max_retries')})")

    def _calibration_settings_problems(self, ids=None) -> list[str]:
        out = []
        s = self.slow
        if s is None or self.now - s.t > 2.0 or not s.complete:
            return ["서보 설정 읽기가 최신이 아님"]
        lo, hi = self.cfg.expect("voltage_v")
        for i in ids or self.ids:
            a = s.axes[i]
            if self.purpose != "watch" and a["torque_enable"] and i not in self.torque_ids:
                out.append(f"ID {i} 토크 ON")
            if a["operating_mode"] != self.cfg.expect("operating_mode") and self.purpose != "watch":
                out.append(f"ID {i} mode {a['operating_mode']}")
            if self.check_slow is not None and a["drive_mode"] != self.check_slow.value(i, "drive_mode"):
                out.append(f"ID {i} Drive Mode 변경")
        f = self.fast
        if f is not None:
            for i in self.ids:
                if i in f.axes and not lo <= f[i].voltage_v <= hi:
                    out.append(f"ID {i} 전압 {f[i].voltage_v:.1f} V")
                if i in f.axes and f[i].present_temperature > self.cfg.expect("max_temperature_c"):
                    out.append(f"ID {i} 온도 {f[i].present_temperature} C")
        return out

    # ================================================================ stored calibration (prepare / teleop)
    def _start_resume(self) -> None:
        """No alignment: P_ref from the newest calibrate, if the servos prove their count frame is unchanged."""
        f = self._fresh_complete()
        if f is None or self.slow is None or not self.slow.complete:
            return                                   # CONNECTED_CHECK runs again on the next read
        tol = round(math.radians(self.cfg.m("ready", "drift_deg")) * _k(min(j.gear for j in self.cfg.joints)))
        res = Calibration.resume(self.saved, self.cfg.joints, f, self.slow, self.now, tol)
        if isinstance(res, Reject):
            self.log.event("calibration_resume_refused", reason=res.reason)
            self._fault("NO_CALIBRATION", f"저장된 보정을 쓸 수 없습니다: {res.reason}. 아무것도 쓰지 않았습니다. "
                                          "calibrate로 관절별 원점을 다시 등록한 뒤 실행하세요. q: 종료")
            return
        self.cal, free = res
        self.maps = self.cal.joints
        self.written = {int(i): (w["pa"], w["pv"], w["goal"]) for i, w in self.saved["written"].items()}
        self.expected_profile = {i: (pa, pv) for i, (pa, pv, _) in self.written.items()}
        when = self.saved.get("calibrated_wall") or "?"
        self.log.event("calibration_resumed", calibration_id=self.cal.session_id, calibrated_wall=when,
                       torque_off_ids=free, **self.cal.to_log())
        self.log.session(calibration=self.cal.to_log(), calibration_resumed_from=when)
        if free:
            names = ", ".join(str(self.cfg.ids.index(i) + 1) for i in free)
            self._enter(S.RESUME_HOLD, f"저장된 보정({when})으로 현재 자세를 불러왔습니다 (SIM 회색 팔 = 실물). 토크가 꺼진 관절 "
                                       f"{names}: 회색 팔이 실물과 같은지 확인하고, 팔을 받친 상태에서 y = 지금 자세 그대로 토크 ON "
                                       f"(이동 없음). q: 종료", ids=free)
            self.ctx.update(ids=free, sub="ask", resumed=when)
            return
        self._resume_preview(f, when)

    def _resume_preview(self, f: FastSnapshot, when: str) -> None:
        plan = self._make_plan(f)
        if plan is not None:
            self._enter_preview(plan, {i: f[i].goal_position for i in self.ids},
                                f"저장된 보정({when})으로 현재 자세를 불러왔습니다. ① SIM 회색 팔이 실물 자세와 같은지 확인 "
                                "(다르면 q 후 calibrate). ② ")

    def _on_resume_hold(self, events) -> None:
        c, h = self.ctx, self.cfg.motion["hold"]
        ids = c["ids"]
        f = self.fast
        if c["sub"] == "ask":
            if any(e.kind == "QUIT" for e in events):
                self._enter(S.EXITED, "종료 (토크 OFF 관절은 그대로, 이동 없음)")
            elif any(e.kind == "CONFIRM" for e in events) and f is not None and f.complete:
                goals = {i: f[i].present_position for i in ids}
                self._write_profile_goal({i: (h["profile_acceleration_raw"], h["profile_velocity_raw"], g)
                                          for i, g in goals.items()}, "hold_prepare_resume")
                c.update(sub="goal", goals=goals, t=self.now)
                self._msg = "현재 위치를 목표로 설정 중 (토크 ON 전): 움직이지 마세요"
            return
        goals = c["goals"]
        if c["sub"] == "goal":
            if f is not None and f.t > c["t"] and f.complete and not any(f[i].alert for i in ids) and all(
                    (f[i].profile_acceleration, f[i].profile_velocity) ==
                    (h["profile_acceleration_raw"], h["profile_velocity_raw"])
                    and abs(f[i].goal_position - goals[i]) <= h["max_error_counts"] for i in ids):
                self._emit(SyncWrite(64, 1, {i: b"\x01" for i in ids}, "torque_on_resume"))
                self.torque_ids.update(ids)
                self.torque_on_t.update({i: self.now for i in ids})
                self._out.append(SlowReadNow())
                c.update(sub="torque", t=self.now)
                self._msg = "토크 ON 확인 중"
            elif self.now - c["t"] > 0.6:
                self._fault("HOLD_PREP", "Goal=현재 위치/PV/PA 읽기 확인 실패 - 토크를 켜지 않았습니다")
            return
        if f is not None and f.complete:
            jump = {i: f[i].present_position - goals[i] for i in ids if abs(f[i].present_position - goals[i]) > h["max_jump_counts"]}
            if jump:
                self._abort_to_present(list(jump), "HOLD_JUMP", f"토크 ON 전후 위치 점프 {jump} count")
                return
        if c["sub"] == "torque":
            s = self.slow
            if s is not None and s.t > c["t"] and s.complete and all(s.value(i, "torque_enable") for i in ids):
                c.update(sub="verify", t=self.now)
                self._msg = "고정 확인 중"
            elif self.now - c["t"] > 0.6:
                self._abort_to_present(ids, "TORQUE_PARTIAL", "토크 ON이 확인되지 않음")
            else:
                self._out.append(SlowReadNow())
            return
        if c["sub"] == "verify" and self.now - c["t"] >= h["verify_s"] and f is not None and f.complete:
            err = {i: f[i].present_position - goals[i] for i in ids}
            if any(abs(v) > h["max_error_counts"] for v in err.values()):
                self._abort_to_present(ids, "HOLD_ERROR", f"유지 오차 {err} count")
                return
            if not self._adopt_torque_on_goals(ids, f, "토크 ON 관절"):
                return
            self.log.event("resume_joints_held", goals=goals, error_counts=err)
            self._store_hold("torque-off joints held at resume")
            self._resume_preview(f, c["resumed"])

    # ================================================================ sequential alignment (calibrate)
    def _start_align(self) -> None:
        self.align = {"signs": {j.dxl_id: j.sign for j in self.cfg.joints}, "detected": {}, "parts": {},
                      "goals": {}, "saved": False}
        self.log.event("align_start", known_signs={j.name: j.sign for j in self.cfg.joints})
        self._align_joint(0)

    def _align_joint(self, k: int) -> None:
        j = self.cfg.joints[k]
        sub = "zero" if self.align["signs"][j.dxl_id] is not None else "sign"
        self._enter(S.ALIGN, self._align_message(k, sub), joint=j.name, step=sub)
        self.ctx.update(k=k, sub=sub, p_start=None, detected=None)

    def _align_sub(self, sub: str, message: str) -> None:
        """Next step of the same joint: drop keys typed before this prompt was shown."""
        self.ctx["sub"] = sub
        self._msg = message
        self.entry_t = self.now
        self._out.append(FlushInput())
        self.log.event("align_step", joint=self.cfg.joints[self.ctx["k"]].name, step=sub)

    def _align_message(self, k: int, sub: str) -> str:
        n, held = k + 1, ", ".join(str(x + 1) for x in range(k)) or "없음"
        already = self.cfg.joints[k].dxl_id in self.torque_ids
        if sub == "sign" and already:
            return (f"[관절 {n}/6 방향 확인] 관절 {n}은 토크 ON으로 고정돼 있어 손으로 돌릴 수 없습니다. x -> y: 관절 {n}의 "
                    f"토크만 끄기 (다른 관절은 계속 고정, 관절 {n} 위쪽은 받쳐 주세요)")
        if sub == "zero" and already:
            return (f"[관절 {n}/6 영점] 관절 {n}은 이미 토크 ON으로 고정돼 있습니다. SIM 기준 자세(노란 링크)와 맞으면 Enter = "
                    f"그 자리 그대로 원점 등록. 맞지 않으면 x -> y: 관절 {n}만 토크 OFF 후 손으로 맞추고 Enter. 고정된 관절: {held}")
        if sub == "sign":
            thr = self.cfg.m("calibration", "sign_detect_counts") / _k(self.cfg.joints[k].gear)
            return (f"[관절 {n}/6 방향 확인] SIM의 초록 팔이 관절 {n}의 + 방향으로 흔들립니다. 실물 관절 {n}을 그 방향으로 "
                    f"{math.degrees(thr):.0f}° 이상 천천히 돌리세요 (아래 '움직임' 숫자로 확인). 감지되면 y. 고정된 관절: {held}")
        return (f"[관절 {n}/6 영점] 관절 {n}(노란 링크)을 SIM의 기준 자세(팔이 위로 곧게 선 모양)와 같게 맞춘 뒤, 움직이지 말고 "
                f"Enter. Enter = 원점 등록 + 그 자리에서 토크 ON 고정(이동 없음). 고정된 관절: {held}. q: 종료")

    def _on_align(self, events) -> None:
        c = self.ctx
        k = c["k"]
        j = self.cfg.joints[k]
        i, n = j.dxl_id, k + 1
        if self.pending_confirm == "torque_off":
            self._torque_off_flow(events)
            return
        if self.pending_confirm == "release_joint":
            self._release_joint_flow(events, k)
            return
        if any(e.kind == "QUIT" for e in events) and c["sub"] in ("sign", "zero"):
            held = sorted(self.torque_ids)
            self._enter(S.EXITED, f"종료: 정렬 중단. 고정한 ID {held}는 토크 ON 유지" if held else "종료 (토크 OFF, 이동 없음)")
            return
        if any(e.kind == "TORQUE_OFF" for e in events) and i in self.torque_ids and c["sub"] in ("sign", "zero"):
            self.pending_confirm = "release_joint"       # only the joint being aligned now
            self._msg = (f"관절 {n}의 토크만 끕니다 (다른 관절은 계속 고정). 관절 {n} 위쪽이 처질 수 있으니 받친 뒤 y, "
                         f"취소는 다른 키")
            self._out.append(FlushInput())
            self.entry_t = self.now
            return
        if any(e.kind == "TORQUE_OFF" for e in events) and self.torque_ids and c["sub"] in ("sign", "zero"):
            self.pending_confirm = "torque_off"
            self._msg = "고정한 관절의 토크를 끄면 팔이 처집니다. 팔을 손·지그로 받친 뒤 y, 취소는 다른 키"
            self._out.append(FlushInput())
            self.entry_t = self.now
            return
        f = self.fast
        h, w = self.cfg.motion["hold"], self.cfg.motion["calibration"]
        if c["sub"] == "sign":
            if f is None or not f.complete:
                return
            p = f[i].present_position
            if c["p_start"] is None or any(e.kind == "RETRY" for e in events):
                c["p_start"], c["detected"] = p, None
                self._msg = self._align_message(k, "sign")
            d = p - c["p_start"]
            if c["detected"] is None and abs(d) >= w["sign_detect_counts"]:
                c["detected"] = 1 if d > 0 else -1
                self.log.event("sign_detected", joint=j.name, id=i, counts=d, sign=c["detected"])
                self._msg = (f"[관절 {n}/6 방향 확인] 엔코더가 {'증가' if d > 0 else '감소'}했습니다 -> s = {c['detected']:+d}. "
                             f"이제 SIM의 노란 링크가 실물을 따라 움직입니다. 같은 방향·비슷한 각도로 따라 돌면 y, "
                             f"초록 팔과 반대로 돌렸거나 크기가 크게 다르면 r(다시)")
                self.entry_t = self.now
            if c["detected"] is not None and any(e.kind == "CONFIRM" for e in events):
                self.align["signs"][i] = self.align["detected"][i] = c["detected"]
                self.log.event("sign_confirmed", joint=j.name, id=i, sign=c["detected"])
                self._align_sub("zero", self._align_message(k, "zero"))
            elif any(e.kind == "START" for e in events):
                self._msg = self._align_message(k, "sign") + " (먼저 방향 확인: 조금 돌린 뒤 y)"
            return
        if c["sub"] == "zero":
            if any(e.kind == "START" for e in events):
                c["window"] = CalibrationWindow((i,), w["samples"], w["max_spread_counts"], w["max_velocity_raw"])
                c["t_win"] = self.now
                self._align_sub("window", f"[관절 {n}/6] 원점 등록 중: 움직이지 마세요")
            return
        if c["sub"] == "window":
            win: CalibrationWindow = c["window"]
            win.add(f)
            if not win.full:
                if self.now - c["t_win"] > 3.0 + win.n / self.cfg.m("loop", "read_hz"):
                    self._align_reject(k, "완전한 샘플을 모으지 못함 (응답 누락 반복)")
                return
            problems = self._calibration_settings_problems([i])
            held_moved = {x: f[x].present_position - g for x, g in self.align["goals"].items()
                          if abs(f[x].present_position - g) > h["max_error_counts"]}
            if held_moved:
                problems.append(f"고정한 관절이 유지 목표에서 벗어남 {held_moved} count")
            jm = JointMap(j.name, i, j.gear, self.align["signs"][i])
            res = Reject("; ".join(problems)) if problems else win.evaluate((jm,), (j.q_ref_rad,), self.now)
            if isinstance(res, Reject):
                self._align_reject(k, res.reason)
                return
            self.align["parts"][i] = res
            self.log.event("joint_registered", joint=j.name, id=i, p_ref=res.p_ref[i], spread=res.spread[i])
            goal = f[i].present_position
            c.update(goal=goal, t=self.now)
            self._write_profile_goal({i: (h["profile_acceleration_raw"], h["profile_velocity_raw"], goal)},
                                     f"hold_prepare_J{n}")
            self._align_sub("hold_goal", f"[관절 {n}/6] 현재 위치를 목표로 설정 중 (토크 ON 전)")
            return
        goal = c["goal"]
        if c["sub"] == "hold_goal":
            if f is not None and f.t > c["t"] and f.complete:
                a = f[i]
                # Goal follows Present while torque is off (servo firmware): within the hold tolerance is enough
                if (a.profile_acceleration, a.profile_velocity) == (h["profile_acceleration_raw"], h["profile_velocity_raw"]) \
                        and abs(a.goal_position - goal) <= h["max_error_counts"] and not a.alert:
                    self._emit(SyncWrite(64, 1, {i: b"\x01"}, f"torque_on_J{n}"))
                    self.torque_ids.add(i)
                    self.torque_on_t[i] = self.now
                    self._out.append(SlowReadNow())
                    c["t_torque"] = self.now
                    self._align_sub("hold_torque", f"[관절 {n}/6] 토크 ON 확인 중")
                    return
            if self.now - c["t"] > 0.6:
                self.align["parts"].pop(i, None)
                self._fault("HOLD_PREP", f"관절 {n}: Goal=현재 위치/PV/PA 읽기 확인 실패 - 이 관절은 토크를 켜지 않았습니다")
            return
        if f is not None and f.complete and abs(f[i].present_position - goal) > h["max_jump_counts"]:
            self._abort_to_present([i], "HOLD_JUMP", f"관절 {n}: 토크 ON 전후 위치 점프 {f[i].present_position - goal} count")
            return
        if c["sub"] == "hold_torque":
            s = self.slow
            if s is not None and s.t > c["t_torque"] and i in s.axes and s.value(i, "torque_enable"):
                c["t_hold"] = self.now
                self._align_sub("hold_verify", f"[관절 {n}/6] 고정 확인 중")
            elif self.now - c["t_torque"] > 0.6:
                self._abort_to_present([i], "TORQUE_PARTIAL", f"관절 {n}: 토크 ON이 확인되지 않음")
            else:
                self._out.append(SlowReadNow())
            return
        if c["sub"] == "hold_verify" and self.now - c["t_hold"] >= h["verify_s"] and f is not None and f.complete:
            err = f[i].present_position - goal
            if abs(err) > h["max_error_counts"]:
                self._abort_to_present([i], "HOLD_ERROR", f"관절 {n}: 유지 오차 {err} count")
                return
            if not self._adopt_torque_on_goals([i], f, f"관절 {n}"):
                return
            self.align["goals"][i] = goal
            self.log.event("joint_held", joint=j.name, id=i, goal=goal, error_counts=err)
            if k + 1 < len(self.cfg.joints):
                self._align_joint(k + 1)
            else:
                self._finish_align(f)

    def _adopt_torque_on_goals(self, ids, f: FastSnapshot, who: str) -> bool:
        """The real servo sets Goal Position to its Present Position when torque turns on (not in the e-manual; seen
        on FW 50: written 17194, read back 17193 after torque on). The read-back Goal becomes the evidence of what
        this program set, if it stayed within the hold tolerance of the written one."""
        tol = self.cfg.m("hold", "max_error_counts")
        moved = {i: f[i].goal_position - self.written[i][2] for i in ids if abs(f[i].goal_position - self.written[i][2]) > tol}
        if moved:
            self._abort_to_present(list(moved), "GOAL_CHANGED", f"{who}: 토크 ON 뒤 Goal이 보낸 값에서 {moved} count 바뀜")
            return False
        for i in ids:
            if f[i].goal_position != self.written[i][2]:
                self.log.event("goal_set_by_torque_on", id=i, written=self.written[i][2], read=f[i].goal_position)
                self.written[i] = (*self.written[i][:2], f[i].goal_position)
        return True

    def _release_joint_flow(self, events, k: int) -> None:
        c, j = self.ctx, self.cfg.joints[k]
        i = j.dxl_id
        if c.get("release_t") is None:
            if any(e.kind == "CONFIRM" for e in events):
                self._emit(SyncWrite(64, 1, {i: b"\x00"}, f"torque_off_J{k + 1}"))
                self.torque_ids.discard(i)
                self.expected_profile.pop(i, None)
                self.align["parts"].pop(i, None)
                self.align["goals"].pop(i, None)
                self._out.append(SlowReadNow())
                c["release_t"] = self.now
            elif events:
                self.pending_confirm = None
                self._msg = self._align_message(k, c["sub"])
            return
        s = self.slow
        if s is not None and s.t > c["release_t"] and i in s.axes and not s.value(i, "torque_enable"):
            self.pending_confirm, c["release_t"] = None, None
            c["p_start"], c["detected"] = None, None
            self.log.event("joint_released", joint=j.name, id=i)
            self._align_sub(c["sub"], self._align_message(k, c["sub"]))
        elif self.now - c["release_t"] > 1.5:
            self.pending_confirm = None
            self._fault("TORQUE_OFF_UNCONFIRMED", f"관절 {k + 1} 토크 OFF 확인 실패", unknown_axes=[i])
        else:
            self._out.append(SlowReadNow())

    def _align_reject(self, k: int, reason: str) -> None:
        self.log.event("calibration_rejected", joint=self.cfg.joints[k].name, reason=reason)
        self._align_sub("zero", f"[관절 {k + 1}/6] 원점 등록 실패: {reason}. 다시 맞추고 Enter")

    def _finish_align(self, f: FastSnapshot) -> None:
        parts = [self.align["parts"][j.dxl_id] for j in self.cfg.joints]
        self.cal = Calibration.merge(parts, self.now)
        self.maps = self.cal.joints
        self.log.event("calibration", method="sequential", **self.cal.to_log())
        self.log.session(calibration=self.cal.to_log(), detected_signs=self.align["detected"])
        goals = dict(self.align["goals"])
        save = ""
        if self.align["detected"]:
            names = {j.dxl_id: j.name for j in self.cfg.joints}
            save = (" | 이번에 확인한 부호 " + ", ".join(f"{names[i]} {s:+d}" for i, s in self.align["detected"].items())
                    + ": y = config/robot.yaml에 저장")
        self._enter(S.HOLDING, "HOLDING: 6개 관절 모두 원점 등록·고정 완료 (토크 ON, 이동 없음). 이 보정이 저장되어 다음 "
                               "calibrate 전까지 prepare / teleop이 그대로 씁니다. q: 종료(토크 유지), x: 토크 OFF(받친 뒤)" + save)
        self.ctx["hold"] = goals

    def _align_view(self, v: View) -> None:
        """ALIGN display: green = reference pose (joint being checked sweeps its + direction), yellow = the joint to
        align now, grey = joints already held (encoder angles). Unaligned joints have no known angle: not drawn."""
        c, f = self.ctx, self.fast
        k = c["k"]
        q_ref = np.array([j.q_ref_rad for j in self.cfg.joints])
        target = q_ref.copy()
        if c["sub"] == "sign":
            phase = (self.now - self.entry_t) % 3.0 / 3.0
            target[k] += math.radians(30) * 0.5 * (1 - math.cos(2 * math.pi * phase))
        v.q_target = target
        q = q_ref.copy()
        status = []
        for x, j in enumerate(self.cfg.joints):
            i = j.dxl_id
            part = self.align["parts"].get(i)
            if part is not None and i in self.align["goals"] and f is not None and i in f.axes:
                q[x] = part.q(f)[0] if len(part.joints) == 1 else q[x]
                status.append("held")
            elif x == k and c["sub"] == "sign" and c.get("detected") and f is not None and i in f.axes:
                q[x] = q_ref[x] + c["detected"] * (f[i].present_position - c["p_start"]) / _k(j.gear)
                status.append("current_live")
            elif x == k:
                status.append("current")
            else:
                status.append("free")
        v.q_meas, v.joint_status, v.calibrated = q, status, False
        j = self.cfg.joints[k]
        if c["sub"] == "sign" and f is not None and j.dxl_id in f.axes and c.get("p_start") is not None:
            moved = math.degrees((f[j.dxl_id].present_position - c["p_start"]) / _k(j.gear))
            thr = math.degrees(self.cfg.m("calibration", "sign_detect_counts") / _k(j.gear))
            v.progress = (f"관절 {k + 1} 움직임 {moved:+.1f}° (감지 기준 ±{thr:.0f}°)" +
                          (f" -> 감지됨 s = {c['detected']:+d}: 노란 링크가 따라 돌면 y, 아니면 r" if c.get("detected") else ""))
        elif c["sub"] == "zero":
            v.progress = f"관절 {k + 1}: SIM 기준 자세에 맞추고 손을 멈춘 뒤 Enter"
        else:
            v.progress = f"관절 {k + 1}: 진행 중 ({c['sub']}) - 움직이지 마세요"

    def _on_countdown(self, events) -> None:
        left = self.cfg.m("home", "countdown_s") - (self.now - self.entry_t)
        self._msg = f"토크 ON 확인. 손을 떼세요 - {max(left, 0):.0f}초 후 HOME 이동 (s/space: 취소, q: 종료)"
        if any(e.kind in ("STOP", "QUIT") for e in events):
            hold = self.ctx["hold"]
            self._enter(S.HOLDING, "HOME 취소: 현재 자세 유지 중 (토크 ON)")
            self.ctx["hold"] = hold
            if any(e.kind == "QUIT" for e in events):
                self._enter(S.EXITED, "종료: 자세 유지 중")
            return
        if left > 0:
            return
        f = self._fresh_complete()
        if f is None:
            return
        plan = self.plan
        moved = {i: f[i].present_position - a.p0 for i, a in zip(self.ids, plan.axes)}
        if any(abs(v) > self.cfg.m("hold", "max_error_counts") for v in moved.values()):
            plan = self._make_plan(f)          # the hold settled somewhere else: plan again from here
            if plan is None:
                return
        self._write_profile_goal({a.dxl_id: (a.pa, a.pv, a.goal) for a in plan.axes}, "home_profile_goal")
        home_ctx = {"t0": self.now, "confirmed": False, "dwell": None,
                    "q0": np.array([a.q0 for a in plan.axes]), "q1": np.array([a.q1 for a in plan.axes])}
        self._enter(S.HOMING, f"HOME 이동 중 (예상 {plan.T_max:.1f} s). s/space: 정지, q: 정지 후 종료")
        self.ctx.update(home_ctx)

    def _make_plan(self, f: FastSnapshot) -> HomePlan | None:
        p0 = {i: f[i].present_position for i in self.ids}
        h = self.cfg.motion["hold"]
        plan = plan_home(self.cfg.joints, self.cal, p0, self.cfg.home_q_rad, self.cfg.motion,
                         hold_pv=h["profile_velocity_raw"], hold_pa=h["profile_acceleration_raw"],
                         path_checker=self.path_checker, fk=self.fk)
        self.plan = plan
        self.log.event("home_plan", **plan.summary())
        for w in plan.warnings:
            self._warn(w)
        if plan.errors:
            self._fault("PLAN_REFUSED", "HOME 계획 거부 (이동 없음, 자세 유지 중): " + "; ".join(plan.errors),
                        errors=plan.errors)
            self.ctx["hold"] = p0
            return None
        return plan

    # ================================================================ HOME preview (path confirmed in the session)
    def _enter_preview(self, plan: HomePlan, hold: dict, prefix: str = "") -> None:
        big = [a.name.replace("Revolute_", "관절 ") + f" {math.degrees(a.q1 - a.q0):+.0f}°" for a in plan.axes
               if abs(a.q1 - a.q0) > math.radians(30)]
        self._enter(S.HOME_PREVIEW, prefix + f"HOME 경로 미리보기: SIM의 초록 팔이 지금 자세 -> HOME 경로를 반복 재생합니다 "
                                    f"(예상 {plan.T_max:.0f} s, 크게 도는 관절: {', '.join(big) or '없음'}). 실물 주변(책상·케이블·"
                                    f"사람)을 확인하고 안전하면 y = 이동 시작, s = 취소(유지), q = 종료")
        self.ctx["hold"] = hold

    def _on_home_preview(self, events) -> None:
        if any(e.kind == "QUIT" for e in events):
            self._enter(S.EXITED, "종료: 자세 유지 중 (HOME 이동 안 함)")
            return
        if any(e.kind == "STOP" for e in events):
            hold = self.ctx["hold"]
            self._enter(S.HOLDING, "HOME 취소: 현재 자세 유지 중 (토크 ON). h: HOME 미리보기 다시, t: EE 조작, q: 종료")
            self.ctx["hold"] = hold
            return
        if any(e.kind == "CONFIRM" for e in events):
            hold = self.ctx["hold"]
            self.log.event("home_path_confirmed", note="user checked the SIM preview and the real surroundings")
            self._enter(S.COUNTDOWN, "")
            self.ctx["hold"] = hold

    # ================================================================ teleop: EE target -> IK -> servo goals
    def _plan_to(self, f: FastSnapshot, q_goal, speed_deg_s: float, accel_deg_s2: float, path_checker=None):
        motion = dict(self.cfg.motion)
        motion["home"] = dict(motion["home"], speed_deg_s=speed_deg_s, accel_deg_s2=accel_deg_s2)
        h = self.cfg.motion["hold"]
        return plan_home(self.cfg.joints, self.cal, {i: f[i].present_position for i in self.ids}, q_goal, motion,
                         hold_pv=h["profile_velocity_raw"], hold_pa=h["profile_acceleration_raw"],
                         path_checker=path_checker)

    def _start_teleop(self) -> None:
        f = self._fresh_complete()
        if f is None or self.cal is None or self.ik is None:
            return
        q = self.cal.q(f)
        pos, mat = self.ik.fk(q)
        tc = self.cfg.motion["teleop"]
        self.teleop = {"pos": pos, "mat": mat, "q_ik": q.copy(), "sent": q.copy(), "ok": True, "why": "",
                       "err": (0.0, 0.0), "dirty": False, "last_send": -math.inf,
                       "step_m": tc["step_m"], "step_deg": tc["step_deg"]}
        self._enter(S.TELEOP, "EE 조작(실물이 따라감): SIM 창의 기즈모에서 화살표를 클릭한 채 끌면 그 축으로 이동, 링을 끌면 그 "
                              "축으로 회전 (한 번에 하나, 축 기준 베이스/툴은 조작 패널). 빈 곳을 끌면 카메라. 터미널 키: "
                              "i/k j/l u/o = 베이스 X/Y/Z +/-, 1/2 3/4 5/6 = 베이스 X/Y/Z축 회전, [ ] = 단계, s/space = 정지(유지), "
                              "h = HOME, q = 정지 후 종료")
        self.log.event("teleop_start", tcp_m=pos.round(5).tolist())

    def _teleop_input(self, events) -> None:
        """Target updates: terminal keys (base frame, session step) and the pose dragged with the SIM gizmo."""
        T = self.teleop
        axes = {"x": 0, "y": 1, "z": 2}
        for e in events:
            if e.kind == "JOG" and isinstance(e.arg, str):
                sign = 1.0 if e.arg[0] == "+" else -1.0
                if e.arg[1] == "r":
                    ax = np.zeros(3)
                    ax[axes[e.arg[2]]] = 1.0
                    rq = np.zeros(4)
                    mujoco.mju_axisAngle2Quat(rq, ax, sign * math.radians(T["step_deg"]))
                    rm = np.zeros(9)
                    mujoco.mju_quat2Mat(rm, rq)
                    T["mat"] = rm.reshape(3, 3) @ T["mat"]
                else:
                    T["pos"] = T["pos"].copy()
                    T["pos"][axes[e.arg[1]]] += sign * T["step_m"]
                T["dirty"] = True
            elif e.kind == "STEP":
                k = 2.0 if e.arg == "up" else 0.5
                T["step_m"] = min(0.05, max(0.001, T["step_m"] * k))
                T["step_deg"] = min(20.0, max(1.0, T["step_deg"] * k))
            elif e.kind == "TARGET" and e.arg is not None:
                pos, quat = e.arg
                rm = np.zeros(9)
                mujoco.mju_quat2Mat(rm, np.asarray(quat, float))
                T["pos"], T["mat"] = np.asarray(pos, float).copy(), rm.reshape(3, 3)
                T["dirty"] = True
            elif e.kind == "TARGET_RESET":
                f = self._fresh_complete()
                if f is not None:
                    q = self.cal.q(f)
                    T["pos"], T["mat"] = self.ik.fk(q)
                    T["q_ik"], T["dirty"] = q, True

    def _on_teleop(self, events) -> None:
        if self.pending_confirm == "torque_off":
            self._torque_off_flow(events)
            return
        T, tc, mh = self.teleop, self.cfg.motion["teleop"], self.cfg.motion["home"]
        if any(e.kind in ("STOP", "QUIT") for e in events):
            self.quit_requested |= any(e.kind == "QUIT" for e in events)
            self._begin_stop("brake", then=("HOLDING",), reason="사용자 정지 (EE 조작)")
            return
        if any(e.kind == "HOME" for e in events):
            self._teleop_home()
            return
        if any(e.kind == "TORQUE_OFF" for e in events):
            self._begin_stop("brake", then=("HOLDING",), reason="토크 OFF 전 정지")
            return
        self._teleop_input(events)
        f = self._fresh_complete()
        if f is None:
            return
        q = self.cal.q(f)
        q_traj = self.cal.q(f, "position_trajectory")
        lo = np.array([j.limit_rad[0] for j in self.cfg.joints])
        hi = np.array([j.limit_rad[1] for j in self.cfg.joints])
        names = [j.name for j in self.cfg.joints]
        bad = [n for k, n in enumerate(names) if abs(q[k] - q_traj[k]) > math.radians(mh["tracking_error_deg"])
               or f[self.ids[k]].following_error]
        if bad:
            self._abort_axes(bad, "TRACKING", f"EE 조작 중 추종 오류: {bad}")
            return
        out = [n for k, n in enumerate(names) if not lo[k] - math.radians(1) <= q[k] <= hi[k] + math.radians(1)]
        if out:
            self._begin_stop("brake", then=("FAULT", "LIMIT", f"가동 범위 이탈: {out}"), reason=f"가동 범위 이탈 {out}")
            return
        v_lim = math.radians(1.15 * tc["speed_deg_s"] + 0.5)
        fast = [n for k, n in enumerate(names)
                if abs(f[self.ids[k]].present_velocity) * pv_unit_rad_s(self.maps[k].gear) > v_lim]
        if fast:
            self._begin_stop("brake", then=("FAULT", "OVERSPEED", f"과속: {fast}"), reason=f"과속 {fast}")
            return
        if T["dirty"]:
            T["dirty"] = False
            inset = math.radians(0.2)                  # IK keeps off the limits (servo quantization: one count over)
            r = self.ik.solve(T["pos"], T["mat"], np.clip(T["q_ik"], lo + inset, hi - inset), lo + inset, hi - inset)
            T["err"] = (r.pos_err_m, r.rot_err_deg)
            if not r.ok:
                T["ok"], T["why"] = False, "도달 불가(가동 범위 또는 작업 공간 밖)"
            elif problems := self._line_problems(q, r.q):
                T["ok"], T["why"] = False, f"모델 충돌: {problems[0]}"
            else:
                T["ok"], T["why"], T["q_ik"] = True, "", r.q
        if not T["ok"] or self.now - T["last_send"] < 1.0 / tc["send_hz"]:
            return
        # chase: never command more than max_jump_deg ahead of the measured pose, along the checked joint line
        lead = float(np.abs(T["q_ik"] - q).max())
        frac = min(1.0, math.radians(tc["max_jump_deg"]) / lead) if lead > 0 else 1.0
        goal = q + frac * (T["q_ik"] - q)
        T["chasing"] = frac < 1.0
        if np.abs(goal - T["sent"]).max() < math.radians(tc["deadband_deg"]):
            return
        plan = self._plan_to(f, goal, tc["speed_deg_s"], tc["accel_deg_s2"])
        if plan.errors:
            T["ok"], T["why"] = False, "계획 거부: " + plan.errors[0]
            return
        self.plan = plan
        self._write_profile_goal({a.dxl_id: (a.pa, a.pv, a.goal) for a in plan.axes}, "teleop_goal")
        T["sent"], T["last_send"] = goal, self.now

    def _line_problems(self, q0, q1) -> list[str]:
        """Model check of the joint-space line q0 -> q1 (the servo profiles share one time scale: close to a line)."""
        if self.path_checker is None:
            return []
        step = math.radians(self.cfg.m("teleop", "path_check_step_deg"))
        n = max(1, math.ceil(float(np.abs(q1 - q0).max()) / step))
        for k in range(1, n + 1):
            problems = self.path_checker(q0 + (q1 - q0) * k / n)
            if problems:
                return problems
        return []

    def _teleop_home(self) -> None:
        f = self._fresh_complete()
        if f is None:
            return
        plan = self._make_plan(f)
        if plan is not None:
            self._enter_preview(plan, {i: f[i].goal_position for i in self.ids})

    def _teleop_view(self, v: View) -> None:
        T = self.teleop
        v.q_target = T["q_ik"]
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, T["mat"].reshape(-1))
        v.target_pose, v.target_ok = (T["pos"].copy(), q), T["ok"]
        p = T["pos"] * 1000
        v.progress = (f"EE 목표 [{p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f}] mm | IK "
                      f"{('OK' + (' (따라가는 중)' if T.get('chasing') else '')) if T['ok'] else '불가: ' + T['why']}"
                      f" (오차 {T['err'][0] * 1000:.1f} mm, {T['err'][1]:.1f}°) | 단계 {T['step_m'] * 100:.1f} cm / "
                      f"{T['step_deg']:.0f}°")

    def command_done(self, cmd, t_sent: float) -> None:
        """Runner: when a write actually left the PC. HOME prediction and stop timing start there."""
        if getattr(cmd, "label", "") == "home_profile_goal" and self.state == S.HOMING:
            self.ctx["t0"] = t_sent
        elif getattr(cmd, "label", "").startswith("stop_") and self.state == S.STOPPING:
            self.ctx["t"] = t_sent
        elif getattr(cmd, "label", "") == "pinch_stream_first":
            self.command_sent_pinch(t_sent)

    def _on_homing(self, events) -> None:
        c, plan, mh = self.ctx, self.plan, self.cfg.motion["home"]
        for e in events:
            if e.kind == "START":
                self.log.event("input_ignored", kind="START", reason="HOME 중 재보정·재시작 거부")
                self._warn("HOME 이동 중에는 재보정·재시작을 받지 않습니다")
        if any(e.kind in ("STOP", "QUIT") for e in events):
            self.quit_requested |= any(e.kind == "QUIT" for e in events)
            self._begin_stop("brake", then=("HOLDING",), reason="사용자 정지")
            return
        f = self.fast
        t = self.now - c["t0"]
        if not c["confirmed"]:
            # nothing read back yet: keep waiting; no complete read for comm_fault_after_s raises COMM_LOST
            if f is None or f.t <= c["t0"] or not f.complete:
                return
            exp = {a.dxl_id: (a.pa, a.pv, a.goal) for a in plan.axes}
            bad = [i for i in self.ids if (f[i].profile_acceleration, f[i].profile_velocity, f[i].goal_position) != exp[i]]
            if bad:
                self._abort_to_present(list(self.ids), "HOME_CMD_MISMATCH", f"HOME 명령 읽기값 불일치 ID {bad}")
                return
            c["confirmed"] = True
            self.log.event("home_command_confirmed")
        if f is None or not f.complete or self.now - f.t > self.cfg.m("loop", "stale_after_s"):
            c["dwell"] = None
            return
        q = self.cal.q(f)
        q_traj = self.cal.q(f, "position_trajectory")
        q_pred = plan.predict(f.t - c["t0"])
        q0, q1 = c["q0"], c["q1"]
        lo_env = np.minimum(q0, q1) - math.radians(mh["envelope_margin_deg"])
        hi_env = np.maximum(q0, q1) + math.radians(mh["envelope_margin_deg"])
        track_bad = [a.name for k, a in enumerate(plan.axes)
                     if abs(q[k] - q_traj[k]) > math.radians(mh["tracking_error_deg"]) or f[a.dxl_id].following_error]
        if track_bad:
            self._abort_axes(track_bad, "TRACKING", f"추종 오류(Present vs Position Trajectory / Following Error): {track_bad}")
            return
        pred_bad = [a.name for k, a in enumerate(plan.axes) if abs(q[k] - q_pred[k]) > math.radians(mh["predicted_error_deg"])]
        if pred_bad:
            msg = f"전송 프로파일 예측과 차이 > {mh['predicted_error_deg']} deg: {pred_bad}"
            self._begin_stop("brake", then=("FAULT", "PREDICTION", msg), reason=msg)
            return
        env_bad = [a.name for k, a in enumerate(plan.axes) if not lo_env[k] <= q[k] <= hi_env[k]]
        lim_bad = [j.name for k, j in enumerate(self.cfg.joints) if not j.limit_rad[0] <= q[k] <= j.limit_rad[1]]
        wrong = [a.name for k, a in enumerate(plan.axes)
                 if (q[k] - q0[k]) * np.sign(q1[k] - q0[k]) < -math.radians(mh["wrong_direction_deg"])]
        speed_bad = []
        for k, a in enumerate(plan.axes):
            v = abs(f[a.dxl_id].present_velocity) * pv_unit_rad_s(self.maps[k].gear)
            limit = mh["overspeed_factor"] * a.v_peak_rad_s + math.radians(0.5)
            if v > limit or v > 1.1 * math.radians(self.cfg.m("limits", "v_cap_deg_s")):
                speed_bad.append(a.name)
        for code, bad, text in (("LIMIT", lim_bad, "가동 범위 이탈"), ("ENVELOPE", env_bad, "이동 구간 이탈"),
                                ("WRONG_DIRECTION", wrong, "반대 방향 이동 (부호·ID 확인)"), ("OVERSPEED", speed_bad, "과속")):
            if bad:
                self._begin_stop("brake", then=("FAULT", code, f"{text}: {bad}"), reason=f"{text}: {bad}")
                return
        timeout = mh["timeout_factor"] * plan.T_max + mh["timeout_margin_s"]
        arrived = self._arrived(f, q, q1)
        if t > timeout and not arrived:
            late = [a.name for k, a in enumerate(plan.axes) if abs(q[k] - q1[k]) > math.radians(self.cfg.m("ready", "tolerance_deg"))]
            self._abort_axes(late or [a.name for a in plan.axes], "TIMEOUT", f"HOME 시간 초과 ({timeout:.1f} s): {late}")
            return
        if arrived:
            if c["dwell"] is None:
                c["dwell"] = {"t": self.now, "pos": {i: [f[i].present_position] for i in self.ids}}
            else:
                for i in self.ids:
                    c["dwell"]["pos"][i].append(f[i].present_position)
            spread = {i: max(v) - min(v) for i, v in c["dwell"]["pos"].items()}
            if max(spread.values()) > self.cfg.m("ready", "still_spread_counts"):
                c["dwell"] = None
            elif self.now - c["dwell"]["t"] >= self.cfg.m("ready", "dwell_s"):
                err = (q - q1)
                report = {"t_s": round(t, 3), "predicted_T_s": round(plan.T_max, 3),
                          "error_deg": dict(zip([a.name for a in plan.axes], np.degrees(err).round(4).tolist())),
                          "dwell_spread_counts": spread, "plan": plan.summary(),
                          "note": "엔코더 기준 도착. 마킹 오차·백래시는 이 판정에 들어 있지 않음"}
                self.log.home_report(report)
                hold = {i: f[i].goal_position for i in self.ids}
                self._enter(S.READY, "READY: HOME 도착 확인 (오차·정지 유지 시간 충족). 자세 유지 중. q: 종료(토크 유지)",
                            **{k: v for k, v in report.items() if k != "plan"})
                self.ctx["hold"] = hold
        else:
            c["dwell"] = None

    def _arrived(self, f: FastSnapshot, q, q1) -> bool:
        tol = math.radians(self.cfg.m("ready", "tolerance_deg"))
        return bool(np.all(np.abs(q - q1) <= tol)) and all(
            f[i].moving == 0 and not f[i].profile_ongoing for i in self.ids)

    def _on_ready(self, events) -> None:
        if self.purpose == "teleop" and self.teleop is None:
            self._start_teleop()
            return
        if self.purpose == "pinch" and self.cal is not None and self.pending_confirm is None:
            self._pinch_idle(S.LINK_WAIT)
            return
        self._hold_monitor(events, "READY")

    def _on_holding(self, events) -> None:
        if self.purpose == "pinch" and self.cal is not None and self.pending_confirm is None and \
                any(e.kind == "CONFIRM" for e in events):
            if self.link_up:
                self._pinch_idle(S.ARMED)
                return
            self._msg = "시뮬이 연결돼 있지 않아 무장할 수 없습니다. 시뮬을 --real-arm으로 실행한 뒤 y. q: 종료"
        self._hold_monitor(events, "HOLDING")

    def _hold_monitor(self, events, label: str) -> None:
        if self.pending_confirm == "torque_off":
            self._torque_off_flow(events)
            return
        for e in events:
            if e.kind == "START":
                self.log.event("input_ignored", kind="START", reason=f"{label}: 재시작은 새 실행에서")
        if any(e.kind == "QUIT" for e in events) or self.quit_requested:
            self._enter(S.EXITED, f"종료: {label} - 정지 확인 후 자세 유지 중 (토크 ON)")
            return
        if any(e.kind == "TORQUE_OFF" for e in events):
            self.pending_confirm = "torque_off"
            self._msg = "토크를 끄면 팔이 자중으로 처집니다. 팔을 손·지그로 받친 뒤 y, 취소는 다른 키"
            self._out.append(FlushInput())
            self.entry_t = self.now
            return
        if self.purpose == "teleop" and self.cal is not None:
            if any(e.kind == "TELEOP" for e in events):
                self._start_teleop()
                return
            if any(e.kind == "HOME" for e in events):
                self._teleop_home()
                return
        if any(e.kind == "CONFIRM" for e in events) and self.align and self.align["detected"] and not self.align["saved"]:
            names = {j.dxl_id: j.name for j in self.cfg.joints}
            self._out.append(SaveSigns({names[i]: s for i, s in self.align["detected"].items()}))
            self.align["saved"] = True
            self.log.event("signs_save_requested", signs={names[i]: s for i, s in self.align["detected"].items()})
            self._msg = "부호 저장 요청 완료 (config/robot.yaml). " + self._msg.split(" | ")[0]
        self._drift_check(label)

    def _drift_check(self, label: str) -> bool:
        """READY / HOLDING / pinch idle: the arm must stay where it is held. True: a DRIFT fault was raised."""
        f = self.fast
        hold = self.ctx.get("hold")
        if f is not None and f.complete and hold:
            drift = {i: abs(f[i].present_position - hold[i]) for i in self.ids}
            k_min = min(_k(j.gear) for j in self.cfg.joints)
            if max(drift.values()) > math.radians(self.cfg.m("ready", "drift_deg")) * k_min:
                self._fault("DRIFT", f"{label} 중 위치가 유지 목표에서 벗어남 {drift} count (외력·걸림?)", drift=drift)
                return True
        return False

    def _torque_off_now(self) -> bool:
        """Control panel "torque off" (its dialog already asked whether the arm is supported). Moving: braking stop
        first, torque off once the stop is confirmed; holding / waiting: torque off now. True if taken."""
        if self.state.value not in TORQUE_OFF_NOW_STATES or not self._comm_ok():
            self.log.event("input_ignored", kind="TORQUE_OFF_NOW", state=self.state.value)
            return False
        self.log.event("torque_off_requested", source="panel", state=self.state.value)
        if self.state == S.STOPPING:
            self.torque_off_after_stop = True
        elif self.state in MOVING_STATES:
            self._begin_stop("brake", then=("HOLDING",), reason="토크 OFF 전 정지 (조작 패널)")
            self.torque_off_after_stop = self.state == S.STOPPING
        else:
            if self.state == S.TRAJ_CHECK:
                self._traj_refuse(self.ctx["traj"], ["토크 OFF (조작 패널)"])
            if self.state in (S.HOME_PREVIEW, S.COUNTDOWN, S.RESUME_HOLD, S.READY) or self.state in PINCH_IDLE_STATES:
                self._enter(S.HOLDING, "토크 OFF 중 (조작 패널)")
            self.pending_confirm = "torque_off"
            self._torque_off_write()
        return True

    def _torque_off_write(self) -> None:
        self._emit(SyncWrite(64, 1, {i: b"\x00" for i in self.ids}, "torque_off_user"))
        self.torque_ids.clear()              # requested off: from here on "off" is expected, not evidence
        self._out.append(SlowReadNow())
        self.sub, self.ctx["t_off"] = "torque_off_wait", self.now

    def _torque_off_flow(self, events) -> None:
        if self.sub != "torque_off_wait":
            if any(e.kind == "CONFIRM" for e in events):
                self._torque_off_write()
            elif any(e.kind in ("CANCEL", "STOP", "START", "TORQUE_OFF") for e in events):
                self.pending_confirm = None
                self._msg = "토크 OFF 취소: 자세 유지 계속"
            return
        s = self.slow
        if s is not None and s.t > self.ctx["t_off"] and s.complete:
            on = [i for i in self.ids if s.value(i, "torque_enable")]
            if not on:
                self.pending_confirm = None
                self.torque_ids.clear()
                if self.state == S.ALIGN:
                    self.log.event("align_restart", reason="torque off by the user")
                    self._start_align()
                    self._msg = "토크 OFF 확인: 관절 1부터 다시 맞춥니다. " + self._msg
                    return
                kept = (" 보정은 유지됩니다(토크 OFF는 엔코더 기준을 바꾸지 않음): 다음 prepare / teleop이 지금 자세에서 다시 "
                        "고정합니다." if self.cal is not None else "")
                self._enter(S.RELEASED, "토크 OFF 확인. 팔이 처질 수 있습니다." + kept + " q: 종료, n: 새 세션")
                return
        if self.now - self.ctx["t_off"] > 1.5:
            self.pending_confirm = None
            self._fault("TORQUE_OFF_UNCONFIRMED", "토크 OFF 확인 실패", unknown_axes=list(self.ids))
        else:
            self._out.append(SlowReadNow())

    def _on_released(self, events) -> None:
        if any(e.kind == "QUIT" for e in events):
            self._enter(S.EXITED, "종료: 토크 OFF")
        elif any(e.kind == "NEW_SESSION" for e in events):
            self._new_session()

    def _on_monitor(self, events) -> None:
        if any(e.kind == "QUIT" for e in events):
            self._enter(S.EXITED, "종료 (읽기 전용)")
            return
        if any(e.kind == "START" for e in events):
            if self.maps is not None and calibrated_view_ready(self.cfg):
                w = self.cfg.motion["calibration"]
                self._enter(S.CALIBRATING, "기준 등록 중(읽기 전용): 움직이지 마세요")
                self.ctx["window"] = CalibrationWindow(self.ids, w["samples"], w["max_spread_counts"], w["max_velocity_raw"])
            else:
                self.rel_zero = {i: self.fast[i].present_position for i in self.ids} if self.fast and self.fast.complete else None
                self._msg = self._monitor_message() + " | 상대 변화량 0점을 지금 위치로 다시 잡았습니다"

    def _on_stopping(self, events) -> None:
        c = self.ctx
        if any(e.kind == "QUIT" for e in events):
            self.quit_requested = True
        f = self.fast
        if not c["confirmed"]:
            if f is not None and f.t > c["t"] and f.complete:
                bad = [i for i, g in c["goals"].items() if f[i].goal_position != g]
                if bad:
                    self._stop_unconfirmed(bad, "정지 명령 읽기값 불일치")
                    return
                c["confirmed"] = True
                self.log.event("stop_command_confirmed", goals=c["goals"])
            elif self.now - c["t"] > 0.6:
                self._stop_unconfirmed(list(c["goals"]), "정지 명령 확인 응답 없음")
                return
            else:
                return
        if f is None or not f.complete or self.now - f.t > self.cfg.m("loop", "stale_after_s"):
            c["settle"] = None
            return
        ids = list(c["goals"])
        still = all(abs(f[i].present_velocity) <= 1 and f[i].moving == 0 and not f[i].profile_ongoing for i in ids)
        if still:
            c["settle"] = c["settle"] or self.now
            if self.now - c["settle"] >= self.cfg.m("stop", "settle_s"):
                self._finish_stop(f)
        else:
            c["settle"] = None
            if self.now - c["t"] > c["timeout"]:
                moving = [i for i in ids if not (abs(f[i].present_velocity) <= 1 and f[i].moving == 0)]
                self._stop_unconfirmed(moving, f"정지 확인 시간 초과 ({c['timeout']:.1f} s)")

    def _finish_stop(self, f: FastSnapshot) -> None:
        c = self.ctx
        off, self.torque_off_after_stop = self.torque_off_after_stop, False
        final = {i: f[i].present_position for i in c["goals"]}
        residual = {i: f[i].present_position - f[i].goal_position for i in c["goals"]}
        self.log.event("stop_confirmed", kind=c["kind"], final_counts=final, residual_counts=residual,
                       reversal_counts=c.get("reversal", {}))
        then = c["then"]
        hold = {i: f[i].goal_position for i in c["goals"]}
        if then[0] in ("ARMED", "LINK_WAIT") and self.purpose == "pinch":
            after = S.ARMED if then[0] == "ARMED" and self.link_up else S.LINK_WAIT
            self._pinch_result("STOPPED", c.get("reason", "정지"), after=after.value)
            self._pinch_idle(after, hold=hold, prefix="정지 확인됨. ")
        elif then[0] in ("HOLDING", "ARMED", "LINK_WAIT"):
            self._pinch_result("STOPPED", c.get("reason", "정지"), after="HOLDING")
            self._enter(S.HOLDING, "HOLDING: 정지 확인됨, 최종 위치 유지 중 (토크 ON). q: 종료(토크 유지), x: 토크 OFF(받친 뒤)" +
                        (", y: 다시 무장 (시뮬 승인으로 다시 움직임)" if self.purpose == "pinch" else ""),
                        residual_counts=residual)
            self.ctx["hold"] = hold
        else:
            self._fault(then[1], then[2] + " | 정지 확인됨, 자세 유지 중 (토크 ON). x: 토크 OFF(받친 뒤), q: 종료",
                        residual_counts=residual)
            self.ctx["hold"] = hold
        if off:
            self.pending_confirm = "torque_off"
            self._torque_off_write()

    def _stop_unconfirmed(self, axes, why: str) -> None:
        self._fault("STOP_UNCONFIRMED", f"FAULT / 상태 미확인: {why}. 미확인 축 ID {axes}. 위험하면 12V 전원 스위치",
                    unknown_axes=list(axes))

    def _on_fault(self, events) -> None:
        if self.pending_confirm == "torque_off":
            self._torque_off_flow(events)
            return
        if any(e.kind == "QUIT" for e in events):
            self._enter(S.EXITED, "종료: FAULT 상태 그대로 (" + (self.fault.code if self.fault else "") + ")")
        elif any(e.kind == "TORQUE_OFF" for e in events) and self._comm_ok():
            self.pending_confirm = "torque_off"
            self._msg = "토크를 끄면 팔이 자중으로 처집니다. 팔을 손·지그로 받친 뒤 y, 취소는 다른 키"
            self._out.append(FlushInput())
            self.entry_t = self.now
        elif any(e.kind == "NEW_SESSION" for e in events):
            self._new_session()

    def _on_comm_lost(self, events) -> None:
        if self._comm_ok():
            self._msg = (self.fault.message + " | 통신 복구됨: 이 세션의 보정은 무효입니다. 자동으로 재보정·HOME을 재개하지 "
                         "않습니다. n: 새 세션(설정 확인부터" + (", 재정렬)" if self.purpose == "hold" else
                                                         ", 저장된 보정을 서보 읽기값으로 다시 확인)") + ", q: 종료")
        if any(e.kind == "QUIT" for e in events):
            self._enter(S.EXITED, "종료: 통신 단절 후 상태 미확인")
        elif any(e.kind == "NEW_SESSION" for e in events) and self._comm_ok():
            self._new_session()

    def _new_session(self) -> None:
        if self.saved is not None and self.written:     # this run's own last writes are evidence too
            self.saved = dict(self.saved, written={**self.saved["written"], **{
                str(i): {"pa": pa, "pv": pv, "goal": g} for i, (pa, pv, g) in self.written.items()}})
        self.cal, self.plan, self.fault, self.expected_profile = None, None, None, {}
        self.torque_ids.clear()          # torque state is re-read; joints still on are kept and re-registered
        self.torque_on_t.clear()
        self.align = None
        self.retries = 0
        self.log.event("new_session", note="설정 확인부터 다시" + (": 새 정렬" if self.purpose == "hold" else
                                                                ": 저장된 보정을 서보 읽기값으로 다시 확인"))
        self._out.append(SlowReadNow())
        self._enter(S.CONNECTED_CHECK, "새 세션: 설정 다시 확인 중")
        self.ctx["await_slow"] = self.now

    # ================================================================ pinch: trajectories from the simulator
    def _link_bookkeeping(self, events) -> list:
        """LINK_UP / LINK_DOWN only update the link flags here (the states read them); a trajectory that arrives
        while not ARMED is refused at once."""
        out = []
        for e in events:
            if e.kind == "LINK_UP":
                self.link_up, self.peer = True, e.arg
                self.log.event("link_up", peer=e.arg)
            elif e.kind == "LINK_DOWN":
                self.link_up = False
                self.log.event("link_down", reason=e.arg)
                out.append(e)
            elif e.kind == "TRAJ" and (self.state != S.ARMED or self.pending_confirm is not None):
                why = ("실물이 무장 상태가 아님" if self.state not in (S.TRAJ_CHECK, S.TRAJ_MOVING) else "이전 궤적 처리 중") \
                    + f" ({self.state.value})"
                self._traj_refuse(e.arg, [why])
            else:
                out.append(e)
        return out

    def _link_send(self, msg: dict) -> None:
        self._out.append(LinkSend(msg))
        self.log.event("link_send", **{k: v for k, v in msg.items() if k != "metrics"})

    def _pinch_idle(self, state: S, hold=None, prefix: str = "") -> None:
        """LINK_WAIT / ARM_CONFIRM / ARMED: held where the arm is (drift monitor), waiting for the simulator."""
        hold = hold or self.ctx.get("hold") or (
            {i: self.fast[i].goal_position for i in self.ids} if self.fast is not None and self.fast.complete else None)
        lead = self.cfg.motion["pinch"]["lead_s"]
        scale = f", 속도 배율 {self.speed_scale:g}" if self.speed_scale != 1 else ""
        msg = {
            S.LINK_WAIT: "시뮬 연결 대기 중: 다른 터미널에서 python -m pinch_sim.app --real-arm ... 을 실행하세요. 자세 유지 중. "
                         "q: 종료",
            S.ARM_CONFIRM: f"시뮬 연결됨. y = 무장: 이제부터 시뮬 승인(Y 키·음성 '이 경로로 할게')으로 실물이 움직입니다{scale}"
                           f"{' (검사만: 실제로는 움직이지 않음)' if self.check_only else ''}. 실물 주변을 확인하세요. 정지는 "
                           "s/space·패널 ■·시뮬 X·'멈춰'·12V 스위치. q: 종료",
            S.ARMED: f"무장됨: 시뮬 승인으로 실물이 움직입니다{scale}"
                     f"{' (검사만: 움직이지 않음)' if self.check_only else ''}. s/space: 무장 해제(유지), q: 종료",
        }[state]
        self._enter(state, prefix + msg, lead_s=lead)
        self.ctx["hold"] = hold
        if state == S.ARM_CONFIRM:
            self._out.append(FlushInput())         # a y typed before this prompt is not the confirmation
        if state == S.ARMED:
            self.log.event("armed", speed_scale=self.speed_scale, check_only=self.check_only, peer=self.peer)

    def _on_link_wait(self, events) -> None:
        if self.link_up and self.pending_confirm is None:
            self._pinch_idle(S.ARM_CONFIRM)
            return
        self._hold_monitor(events, "LINK_WAIT")

    def _on_arm_confirm(self, events) -> None:
        if self.pending_confirm is None:
            if not self.link_up:
                self._pinch_idle(S.LINK_WAIT, prefix="시뮬 연결 끊김. ")
                return
            if any(e.kind == "CONFIRM" for e in events):
                self._pinch_idle(S.ARMED)
                return
        self._hold_monitor(events, "ARM_CONFIRM")

    def _on_armed(self, events) -> None:
        if self.pending_confirm is None:
            if not self.link_up:
                self._pinch_idle(S.LINK_WAIT, prefix="시뮬 연결 끊김: 무장 해제. ")
                return
            if any(e.kind == "STOP" for e in events):
                hold = self.ctx.get("hold")
                self._enter(S.HOLDING, "무장 해제 (터미널·패널 정지): 자세 유지 중. y: 다시 무장, q: 종료")
                self.ctx["hold"] = hold
                return
            traj = next((e.arg for e in events if e.kind == "TRAJ"), None)
            if traj is not None and not any(e.kind in ("QUIT", "TORQUE_OFF") for e in events):
                self._start_traj_check(traj)
                return
        self._hold_monitor(events, "ARMED")

    def _start_traj_check(self, traj: Trajectory) -> None:
        f = self._fresh_complete()
        if f is None or self.cal is None:
            self._traj_refuse(traj, ["실물 상태가 최신이 아님"])
            return
        q = self.cal.q(f)
        self.log.event("traj_received", traj_id=traj.traj_id, samples=traj.n, dt=traj.dt, duration_s=traj.duration,
                       meta=traj.meta)
        acc, checker = self.acceptance, self.traj_path_checker

        def run():
            return traj_check(traj, acc, q, checker)
        fut = self.check_submit(run) if self.check_submit is not None else _Done(run)
        hold = self.ctx.get("hold")
        self._enter(S.TRAJ_CHECK, f"시뮬 궤적 검사 중 ({traj.duration:.1f} s, {traj.meta.get('move', '')}). s/space: 취소")
        self.ctx.update(hold=hold, traj=traj, future=fut, t=self.now)

    def _traj_refuse(self, traj, reasons: list[str]) -> None:
        self._link_send({"type": "ack", "traj_id": getattr(traj, "traj_id", None), "accepted": False,
                         "reasons": list(reasons)})
        self.log.event("traj_refused", traj_id=getattr(traj, "traj_id", None), reasons=reasons)

    def _pinch_stop_kind(self, events) -> tuple[str, str, bool] | None:
        """(state after the stop, reason, quit) for the stop-class events of this cycle, or None."""
        if any(e.kind == "QUIT" for e in events):
            return "HOLDING", "사용자 종료", True
        if any(e.kind in ("STOP", "TORQUE_OFF") for e in events):
            return "HOLDING", "사용자 정지 (터미널·패널): 무장 해제", False
        if any(e.kind == "LINK_DOWN" for e in events) or not self.link_up:
            return "LINK_WAIT", "시뮬 연결 끊김", False
        if any(e.kind == "EXT_STOP" for e in events):
            reason = next(e.arg for e in events if e.kind == "EXT_STOP") or "시뮬 정지"
            return "ARMED", f"시뮬 정지 ({reason})", False
        return None

    def _on_traj_check(self, events) -> None:
        c = self.ctx
        traj: Trajectory = c["traj"]
        stop = self._pinch_stop_kind(events)
        if stop is not None:
            then, reason, quit_ = stop
            self._traj_refuse(traj, [f"검사 중 정지: {reason}"])
            self.quit_requested |= quit_
            hold = c.get("hold")
            if then == "HOLDING":
                self._enter(S.HOLDING, f"{reason}: 자세 유지 중. y: 다시 무장, q: 종료")
                self.ctx["hold"] = hold
            else:
                self._pinch_idle(S.ARMED if then == "ARMED" else S.LINK_WAIT, hold=hold)
            return
        if self._drift_check("TRAJ_CHECK"):
            return
        fut = c["future"]
        if not fut.done():
            if self.now - c["t"] > self.cfg.motion["pinch"]["check_timeout_s"]:
                self._traj_refuse(traj, ["검사 시간 초과"])
                self._pinch_idle(S.ARMED, hold=c.get("hold"))
            return
        try:
            problems = fut.result()
        except Exception as e:        # noqa: BLE001 - a broken check never lets the trajectory through
            problems = [f"검사 중 예외: {e!r}"]
        f = self._fresh_complete()
        if not problems:
            if f is None:
                problems = ["실물 상태가 최신이 아님"]
            else:
                d = np.abs(traj.q[0] - self.cal.q(f)).max()
                if d > self.acceptance.start_tol_rad:
                    problems = [f"검사 중 실물이 움직임: 시작점과 {math.degrees(d):.2f} deg 차이"]
        if problems:
            self._traj_refuse(traj, problems)
            self._pinch_idle(S.ARMED, hold=c.get("hold"), prefix="궤적 거부 (이동 없음): " + problems[0] + ". ")
            return
        if self.check_only:
            self._link_send({"type": "ack", "traj_id": traj.traj_id, "accepted": True, "check_only": True})
            self._link_send({"type": "result", "traj_id": traj.traj_id, "status": "CHECKED", "reason": "검사만 (이동 없음)",
                             "q_final": self.cal.q(f).tolist(), "metrics": {}, "state_after": "ARMED"})
            self._pinch_idle(S.ARMED, hold=c.get("hold"), prefix="검사 통과 (검사만 모드: 움직이지 않음). ")
            return
        run = traj if self.speed_scale == 1 else traj.scaled(self.speed_scale)
        self.plan = None
        self.pinch = {"traj": run, "t0": None, "last": {}, "prev": {}, "dwell": None, "move": traj.meta.get("move", ""),
                      "metrics": {"ticks": 0, "pv_capped": 0, "max_following_deg": 0.0, "max_tracking_deg": 0.0}}
        self._link_send({"type": "ack", "traj_id": traj.traj_id, "accepted": True, "duration_s": run.duration,
                         "speed_scale": self.speed_scale})
        self.log.event("traj_start", traj_id=traj.traj_id, duration_s=run.duration, speed_scale=self.speed_scale)
        self._enter(S.TRAJ_MOVING, f"궤적 실행 중 ({self.pinch['move']}, {run.duration:.1f} s"
                                   f"{', 배율 ' + format(self.speed_scale, 'g') if self.speed_scale != 1 else ''}). "
                                   "s/space: 정지(무장 해제), 시뮬 X·'멈춰': 정지(무장 유지), q: 정지 후 종료")
        self._pinch_write(f, 0.0, "pinch_stream_first")

    def _pinch_write(self, f: FastSnapshot, t: float, label: str = "pinch_stream") -> None:
        P, p = self.pinch, self.cfg.motion["pinch"]
        cmd = stream_command(P["traj"], t, self.cal.q(f, "position_trajectory"), self.acceptance, p,
                             1.0 / self.cfg.m("loop", "read_hz"))
        goals = self.cal.counts(cmd.q_goal)
        pa = int(p["profile_acceleration_raw"])
        values = {i: (pa, int(pv), goals[i]) for i, pv in zip(self.ids, cmd.pv)}
        P["metrics"]["pv_capped"] += int(np.sum(cmd.pv >= self.acceptance.pv_raw_max))
        P["prev"], P["last"] = P["last"], values
        self._write_profile_goal(values, label)

    def command_sent_pinch(self, t_sent: float) -> None:
        if self.pinch is not None and self.pinch["t0"] is None:
            self.pinch["t0"] = t_sent

    def _on_traj_moving(self, events) -> None:
        P, p = self.pinch, self.cfg.motion["pinch"]
        run: Trajectory = P["traj"]
        stop = self._pinch_stop_kind(events)
        if stop is not None:
            then, reason, quit_ = stop
            self.quit_requested |= quit_
            self._begin_stop("brake", then=(then,), reason=reason)
            return
        if P["t0"] is None:
            P["t0"] = self.now
        f = self._fresh_complete(max_age=self.cfg.m("loop", "stale_after_s"))
        if f is None or f.t <= P["t0"]:
            return                                   # no read since the first write yet (COMM_LOST is global)
        names = [j.name for j in self.cfg.joints]
        lost = [i for i in self.ids if P["last"] and
                (f[i].profile_acceleration, f[i].profile_velocity, f[i].goal_position) != P["last"][i]]
        if lost:
            msg = f"스트리밍 명령 읽기값 불일치 ID {lost} (명령 유실)"
            self._begin_stop("brake", then=("FAULT", "STREAM_WRITE_LOST", msg), reason=msg)
            return
        t_read = f.t - P["t0"]
        q = self.cal.q(f)
        q_int = self.cal.q(f, "position_trajectory")
        q_ref, _ = run.at(t_read)
        m = P["metrics"]
        m["ticks"] += 1
        follow = np.abs(q - q_int)
        m["max_following_deg"] = max(m["max_following_deg"], float(np.degrees(follow.max())))
        bad = [n for k, n in enumerate(names) if follow[k] > math.radians(p["following_error_deg"])
               or f[self.ids[k]].following_error]
        if bad:
            self._abort_axes(bad, "TRACKING", f"궤적 실행 중 추종 오류(측정 vs 서보 내부 목표): {bad}")
            return
        out = [n for k, n in enumerate(names) if not self.acceptance.lo[k] <= q[k] <= self.acceptance.hi[k]]
        if out:
            self._begin_stop("brake", then=("FAULT", "LIMIT", f"가동 범위 이탈: {out}"), reason=f"가동 범위 이탈 {out}")
            return
        track = np.abs(q - q_ref)
        m["max_tracking_deg"] = max(m["max_tracking_deg"], float(np.degrees(track.max())))
        if track.max() > math.radians(p["tracking_error_deg"]):
            bad = [n for k, n in enumerate(names) if track[k] > math.radians(p["tracking_error_deg"])]
            msg = f"궤적과 차이 > {p['tracking_error_deg']} deg: {bad}"
            self._begin_stop("brake", then=("FAULT", "TRACKING", msg), reason=msg)
            return
        win = run.window(t_read - p["envelope_back_s"], t_read + p["lead_s"])
        margin = math.radians(p["envelope_margin_deg"])
        env = [n for k, n in enumerate(names) if not win[:, k].min() - margin <= q[k] <= win[:, k].max() + margin]
        if env:
            self._begin_stop("brake", then=("FAULT", "ENVELOPE", f"궤적 구간 이탈: {env}"), reason=f"궤적 구간 이탈 {env}")
            return
        vwin = run.window_speed(t_read - 0.2, t_read + p["lead_s"])
        v_cap = math.radians(self.cfg.m("limits", "v_cap_deg_s"))
        fast = [n for k, n in enumerate(names)
                if (v := abs(f[self.ids[k]].present_velocity) * pv_unit_rad_s(self.maps[k].gear))
                > 1.15 * vwin[k] + math.radians(0.5) or v > 1.1 * v_cap]
        if fast:
            self._begin_stop("brake", then=("FAULT", "OVERSPEED", f"과속: {fast}"), reason=f"과속 {fast}")
            return
        t_now = self.now - P["t0"]
        q_end = run.q[-1]
        if t_now >= run.duration:
            arrived = bool(np.all(np.abs(q - q_end) <= math.radians(p["arrive_tolerance_deg"]))) and all(
                f[i].moving == 0 and not f[i].profile_ongoing for i in self.ids)
            if arrived:
                P["dwell"] = P["dwell"] if P["dwell"] is not None else self.now
                if self.now - P["dwell"] >= p["settle_s"]:
                    self._pinch_done(f, q, q_end, t_now)
                return
            P["dwell"] = None
            if t_now > run.duration + p["end_timeout_s"]:
                late = [n for k, n in enumerate(names) if abs(q[k] - q_end[k]) > math.radians(p["arrive_tolerance_deg"])]
                self._abort_axes(late or names, "TIMEOUT", f"궤적 끝 도착 시간 초과: {late}")
                return
            if P["last"] and all(v[2] == g for v, g in zip(P["last"].values(), self.cal.counts(q_end).values())):
                return                               # the final goal is sent and read back: let the servos finish
        self._pinch_write(f, t_now)

    def _pinch_done(self, f: FastSnapshot, q, q_end, t_now: float) -> None:
        P = self.pinch
        err = np.degrees(q - q_end)
        self.log.event("traj_done", traj_id=P["traj"].traj_id, t_s=round(t_now, 3), error_deg=err.round(4).tolist(),
                       metrics=P["metrics"])
        after = S.ARMED if self.link_up else S.LINK_WAIT
        self._pinch_result("DONE", "궤적 끝 도착", after=after.value)
        self._pinch_idle(after, hold={i: f[i].goal_position for i in self.ids},
                         prefix=f"궤적 완료 ({t_now:.1f} s, 끝 오차 최대 {np.abs(err).max():.2f} deg). ")

    def _pinch_result(self, status: str, reason: str, code: str | None = None, after: str | None = None) -> None:
        """The simulator's answer for the trajectory that was moving (once). after: the state this program goes to
        now (the state stream follows a little later; the simulator must not see a stale TRAJ_MOVING)."""
        P = self.pinch
        if P is None:
            return
        self.pinch = None
        f = self.fast
        q = self.cal.q(f).tolist() if self.cal is not None and f is not None and f.complete else None
        t = None if P["t0"] is None else round(self.now - P["t0"], 3)
        self._link_send({"type": "result", "traj_id": P["traj"].traj_id, "status": status, "reason": reason, "code": code,
                         "q_final": q, "t_s": t, "metrics": P["metrics"], "state_after": after or "FAULT"})

    def _pinch_view(self, v: View) -> None:
        v.armed = self.state in (S.ARMED, S.TRAJ_CHECK, S.TRAJ_MOVING) or (
            self.state == S.STOPPING and self.ctx.get("then", ("",))[0] == "ARMED")
        if self.state in PINCH_IDLE_STATES or self.state in (S.HOLDING, S.READY):
            v.q_target = None
        P = self.pinch
        if P is not None:
            run = P["traj"]
            t = 0.0 if P["t0"] is None else max(0.0, self.now - P["t0"])
            v.execution = {"traj_id": run.traj_id, "t_s": round(t, 3), "T_s": round(run.duration, 3), "move": P["move"]}
            if self.state == S.TRAJ_MOVING:
                v.q_target = run.at(t)[0]
                v.progress = f"궤적 실행 {min(t, run.duration):.1f}/{run.duration:.1f} s ({P['move']})"

    # ================================================================ commands
    def _write_profile_goal(self, values: dict[int, tuple[int, int, int]], label: str) -> None:
        enc = {i: ct.encode(pa, 4, False) + ct.encode(pv, 4, False) + ct.encode(goal, 4, True)
               for i, (pa, pv, goal) in values.items()}
        self._emit(SyncWrite(*ct.PROFILE_GOAL, enc, label))
        self.prev_profile = {i: self.expected_profile.get(i) for i in values}
        self.expected_profile.update({i: (pa, pv) for i, (pa, pv, _) in values.items()})
        self.written.update(values)

    def _begin_stop(self, kind: str, then: tuple, reason: str, abort_ids=()) -> None:
        """kind 'brake': decelerate along the profile (stopping point ahead, never past the current goal);
        abort_ids: those axes get Goal = latest Present Position (cancel the move: stuck / timeout / tracking)."""
        f = self.fast
        if f is None or not f.complete or not self._comm_ok():
            self._fault("STOP_UNCONFIRMED", f"정지 명령을 보낼 최신 상태가 없음 ({reason}). 상태 미확인. 위험하면 12V 전원 스위치",
                        unknown_axes=list(self.ids))
            return
        pa_max = math.floor(math.radians(self.cfg.m("stop", "accel_max_deg_s2")) /
                            pa_unit_rad_s2(min(j.gear for j in self.cfg.joints)) + 1e-9)
        values, notes, stop_time = {}, {}, 0.0
        for k, i in enumerate(self.ids):
            if self.torque_ids and i not in self.torque_ids:
                continue                      # a free joint (torque off) cannot be stopped by a goal
            a = f[i]
            jc = self.cfg.joints[k]
            if i in abort_ids:
                values[i] = (a.profile_acceleration, a.profile_velocity, a.present_position)
                notes[i] = "abort: Goal = Present Position"
                v, acc = abs(pv_counts_s(a.velocity_trajectory)), max(1.0, _pa_c(a.profile_acceleration))
                back = abs(a.position_trajectory - a.present_position) + v * v / (2 * acc)
                stop_time = max(stop_time, v / acc + 2 * math.sqrt(back / acc))
                continue
            inside = (lambda g, k=k, jc=jc: jc.limit_rad is None or self.cal is None or
                      jc.limit_rad[0] <= self.cal.q_of({**{x: 0 for x in self.ids}, i: g})[k] <= jc.limit_rad[1]) \
                if self.cal is not None else (lambda g: True)
            st = braking_target(a.position_trajectory, a.velocity_trajectory, a.goal_position,
                                a.profile_velocity, a.profile_acceleration,
                                latency_s=self.cfg.m("loop", "latency_allowance_s"), pa_max=pa_max, inside=inside)
            values[i] = (st.pa, st.pv, st.goal)
            notes[i] = st.note
            stop_time = max(stop_time, st.stop_time_s)
            if "insufficient" in st.note:
                self._warn(f"ID {i}: 제동 지점이 가동 범위 안에 들어오지 않음 - {st.note}")
        self.log.event("stop_requested", kind=kind, reason=reason, notes=notes, abort_ids=list(abort_ids),
                       predicted_stop_s=round(stop_time, 3))
        self._write_profile_goal(values, f"stop_{kind}")
        self._enter(S.STOPPING, f"STOPPING: 정지 중 ({reason}). 정지 확인 전에는 HOLDING으로 표시하지 않습니다")
        self.ctx.update(kind=kind, then=then, t=self.now, confirmed=False, settle=None, reason=reason,
                        goals={i: v[2] for i, v in values.items()},
                        timeout=stop_time + self.cfg.m("stop", "timeout_margin_s"))

    def _abort_axes(self, names, code: str, message: str) -> None:
        ids = [j.dxl_id for j in self.cfg.joints if j.name in names]
        self._begin_stop("abort", then=("FAULT", code, message), reason=message, abort_ids=ids)

    def _abort_to_present(self, ids, code: str, message: str) -> None:
        if ids and self._comm_ok():
            self._begin_stop("abort", then=("FAULT", code, message), reason=message, abort_ids=list(ids))
        else:
            self._fault(code, message)

    def _quit_if(self, events) -> bool:
        if any(e.kind == "QUIT" for e in events):
            self._enter(S.EXITED, "종료 (토크 OFF 상태, 이동 없음)")
            return True
        return False

    # ================================================================ texts
    def _monitor_message(self) -> str:
        if self.maps is not None and calibrated_view_ready(self.cfg):
            return "읽기 전용 감시: Enter = 마킹 정렬 상태에서 기준 등록(쓰기 없음), q: 종료"
        return ("읽기 전용 감시 (미보정: SIM 절대 자세는 실제가 아님, 상대 변화량만 표시, 부호 s=+1 가정). "
                "Enter: 상대 0점 다시 잡기, q: 종료")

    def _overlay(self, v: View) -> str:
        lines = [f"STATE {v.state}" + ("  [STALE]" if v.stale else "")]
        if self.state == S.ALIGN:
            k, sub = self.ctx["k"], self.ctx["sub"]
            lines.append(f"ALIGN joint {k + 1}/6 (Revolute_{k + 1}) step: {sub}")
            lines.append("green = reference pose" + ("; it sweeps the + direction" if sub == "sign" else ""))
            lines.append("yellow = joint to align now, grey = joints already held (torque on)")
            lines.append("keys in TERMINAL: Enter register+hold, y confirm, r retry, x torque off, q quit")
            return "\n".join(lines)
        if self.state == S.HOME_PREVIEW:
            lines.append("PREVIEW: green replays the planned path to HOME. Check the real surroundings, then y")
        if self.state == S.RESUME_HOLD:
            lines.append("STORED CALIBRATION: grey = real pose from the encoders. Check it, then y in the terminal")
        if self.purpose == "pinch" and self.state in PINCH_IDLE_STATES | {S.TRAJ_MOVING, S.HOLDING}:
            lines.append({S.LINK_WAIT: "PINCH: waiting for the simulator (--real-arm)",
                          S.ARM_CONFIRM: "PINCH: simulator connected - press y in the TERMINAL (or the arming dialog) to arm",
                          S.HOLDING: "PINCH: disarmed - y in the TERMINAL re-arms"}.get(
                self.state, "PINCH ARMED: simulator approvals move the REAL arm (s = stop + disarm)"))
            if self.state == S.TRAJ_MOVING:
                lines.append("green = trajectory now, orange = servo target, grey = measured")
        if self.state == S.TELEOP:
            lines.append("EE TARGET = gizmo (red/green/blue = X/Y/Z)" +
                         ("" if v.target_ok else "   [TARGET NOT REACHABLE: holding last pose]"))
            lines.append("green = IK solution, orange = servo target, grey = measured")
        if v.calibrated:
            lines.append("grey = MEASURED (encoders)  orange = SERVO TARGET (pos. trajectory)  green = GOAL")
        else:
            lines.append("UNCALIBRATED: pose shown is NOT the real pose (relative motion only, s=+1 assumed)")
        if self.state == S.COMM_LOST:
            lines.append("COMM LOST / MOTION UNKNOWN - power switch is the only hard stop")
        if self.fault is not None and self.state == S.FAULT:
            lines.append(f"FAULT {self.fault.code}" + (f" unknown axes {self.fault.unknown_axes}" if self.fault.unknown_axes else ""))
        if v.q_meas is not None:
            lines.append("q meas deg: " + " ".join(f"{x:7.2f}" for x in np.degrees(v.q_meas)))
        elif v.rel_deg is not None:
            lines.append("rel deg:    " + " ".join(f"{x:7.2f}" for x in v.rel_deg))
        lines.append("keys in TERMINAL: Enter start, s stop, q quit")
        return "\n".join(lines)

    def _axis_status(self) -> dict:
        out = {}
        s, f = self.slow, self.fast
        for i in self.ids:
            st = {}
            if s is not None and i in s.axes:
                st["torque"] = bool(s.value(i, "torque_enable"))
                st["hw_error"] = s.value(i, "hardware_error_status")
            if f is not None and i in f.axes:
                st["moving"] = f[i].moving
                st["present"] = f[i].present_position
            else:
                st["present"] = None
            out[i] = st
        return out

    def _final_report(self) -> dict:
        s = self.slow
        comm = self._comm_ok()
        axes = {}
        for i in self.ids:
            if not comm or s is None or i not in s.axes:
                axes[i] = "상태 미확인 (통신 없음)"
            else:
                tq = s.value(i, "torque_enable")
                mv = self.fast[i].moving if self.fast and i in self.fast.axes else None
                axes[i] = ("토크 ON" if tq else "토크 OFF") + (", 이동 중" if mv else ", 정지")
        torque_on = comm and s is not None and any(s.value(i, "torque_enable") for i in self.ids if i in s.axes)
        note = ("토크 ON 상태로 종료: 서보가 마지막 목표 위치를 유지합니다(PC 감시 없음, Watchdog 미사용). "
                "출력 차단은 12V 전원 스위치이며 끄면 팔이 처집니다." if torque_on else
                "통신이 없어 최종 상태를 확인할 수 없습니다." if not comm else "모든 축 토크 OFF.")
        return {"state_before_exit": self.state.value, "axes": axes, "note": note,
                "fault": None if self.fault is None else self.fault.code}


class _Done:
    """A finished future: the trajectory check run at once (no worker thread)."""

    def __init__(self, fn):
        try:
            self._r, self._e = fn(), None
        except Exception as e:      # noqa: BLE001
            self._r, self._e = None, e

    def done(self) -> bool:
        return True

    def result(self):
        if self._e is not None:
            raise self._e
        return self._r


INVALIDATING = {"INVALIDATED", "COMM_LOST", "TORQUE_ON", "MODE"}
RESETTING = {"INVALIDATED", "MODE"}          # evidence the count frame changed: the stored calibration is deleted
TORQUE_OFF_NOW_STATES = {"ALIGN", "RESUME_HOLD", "HOME_PREVIEW", "COUNTDOWN", "HOMING", "READY", "TELEOP", "STOPPING",
                         "HOLDING", "FAULT", "LINK_WAIT", "ARM_CONFIRM", "ARMED", "TRAJ_CHECK", "TRAJ_MOVING"}


def _k(gear: float) -> float:
    return 4096 * gear / (2 * math.pi)


def _pa_c(pa_raw: int) -> float:
    from .units import pa_counts_s2
    return pa_counts_s2(pa_raw) if pa_raw > 0 else math.inf


def _decode_write(addr: int, data: bytes) -> dict | int:
    if addr == ct.PROFILE_GOAL[0] and len(data) == 12:
        return {"pa": int.from_bytes(data[0:4], "little"), "pv": int.from_bytes(data[4:8], "little"),
                "goal": int.from_bytes(data[8:12], "little", signed=True)}
    return int.from_bytes(data, "little")


def servo_setting_problems(cfg: Config, slow: SlowSnapshot, models: dict) -> tuple[list[str], list[str]]:
    """(reasons to refuse prepare, informational notes). Nothing here is ever "fixed" automatically."""
    problems, notes = [], []
    e = cfg.robot["servo_expect"]
    for i in cfg.ids:
        if models.get(i) != cfg.robot["bus"]["model_number"]:
            problems.append(f"ID {i}: 모델 번호 {models.get(i)} != {cfg.robot['bus']['model_number']} (또는 무응답)")
        if i not in slow.axes:
            problems.append(f"ID {i}: 설정 읽기 실패")
            continue
        a = slow.axes[i]
        dm = ct.drive_mode_bits(a["drive_mode"])
        if dm["time_based_profile"]:
            problems.append(f"ID {i}: Drive Mode bit2=1 (시간 기반 프로파일) - 이 실행기의 PV/PA 단위(rpm)와 다름")
        if dm["torque_on_by_goal_update"]:
            problems.append(f"ID {i}: Drive Mode bit3=1 (Goal 갱신으로 토크 자동 ON) - 현재 위치 준비 후 토크 ON 순서를 보장 못함")
        if a["drive_mode"] != e["drive_mode"]:
            problems.append(f"ID {i}: Drive Mode {a['drive_mode']} != 설정 기대값 {e['drive_mode']} (부호 정의 확인 필요)")
        sc = a["startup_configuration"]
        if sc != e["startup_configuration"]:
            bits = [k for k, on in ct.startup_configuration_bits(sc).items() if on]
            problems.append(f"ID {i}: Startup Configuration {sc} {bits} != 0 (이번 구현은 Startup Torque On=0, RAM Restore=0 전제)")
        if a["status_return_level"] != e["status_return_level"]:
            problems.append(f"ID {i}: Status Return Level {a['status_return_level']} != 2")
        if a["bus_watchdog"] != e["bus_watchdog"]:
            problems.append(f"ID {i}: Bus Watchdog {a['bus_watchdog']} != 0 (이번 구현은 Watchdog을 바꾸지 않으며 켜진 상태로는 실행하지 않음)")
        if a["protocol_type"] != 2:
            problems.append(f"ID {i}: Protocol {a['protocol_type']}")
        if a["hardware_error_status"]:
            bits = [k for k, on in ct.hardware_error_bits(a["hardware_error_status"]).items() if on]
            problems.append(f"ID {i}: Hardware Error {bits} (자동으로 지우지 않음)")
        if a["velocity_limit"] < cfg.m("limits", "pv_raw_max"):
            notes.append(f"ID {i}: Velocity Limit {a['velocity_limit']} < PV 상한 {cfg.m('limits', 'pv_raw_max')}")
        if a["secondary_id"] != 255:
            notes.append(f"ID {i}: Secondary ID {a['secondary_id']} 설정됨")
        if a["operating_mode"] != e["operating_mode"]:
            notes.append(f"ID {i}: Operating Mode {a['operating_mode']} ({ct.OPERATING_MODES.get(a['operating_mode'])})")
    notes.append("mode 4에서는 Current Limit/Goal Current가 전류 상한으로 적용되지 않음 (PWM Limit만 출력 제한). "
                 "저속이 최대 토크를 제한하지 않음")
    return problems, notes
