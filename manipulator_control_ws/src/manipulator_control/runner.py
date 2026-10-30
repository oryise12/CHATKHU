"""The bus loop: the only code that talks to the bus. One cycle = fast Sync Read (+ slow read when due or asked)
-> supervisor.step -> execute its commands. The viewer and the terminal never block it: they read the latest
View under a lock and put key events on a queue.
"""
from __future__ import annotations

import threading
import time
import traceback

import numpy as np

from . import control_table as ct
from .bus import Bus, BusError, WriteBlocked
from .state import AxisFast, FastSnapshot, SlowSnapshot
from .supervisor import (MOVING_STATES, DiscardCalibration, FlushInput, LinkSend, S, SaveCalibration, SaveSigns,
                         SlowReadNow, Supervisor, SyncWrite, View, Write)


class Runner:
    def __init__(self, bus: Bus, sup: Supervisor, log, inputs, clock=time.monotonic, hooks=None, save_signs=None,
                 store=None, link=None):
        self.bus, self.sup, self.log, self.inputs, self.clock = bus, sup, log, inputs, clock
        self.link = link                    # pinch: link.ArmLink (drain() -> events, send(msg)); never flushed
        self.save_signs = save_signs        # callable(dict name -> sign) -> str (what was written)
        self.store = store                  # calibration.CalibrationStore (None: nothing is stored)
        self.cfg = sup.cfg
        self.ids = sup.ids
        self.seq = 0
        self.slow_due = 0.0
        self.slow_now = False
        self.hooks = hooks or []           # callables(now, runner) for --fake scenarios
        self._view = View(state="CONNECTING", message="연결 중")
        self._lock = threading.Lock()
        self.errors = 0
        self.stop = threading.Event()

    # ---------------------------------------------------------------- reads
    def read_fast(self) -> FastSnapshot:
        r = self.bus.sync_read(*ct.FAST, self.ids)
        self.seq += 1
        axes = {i: AxisFast.from_bytes(d, r.errors.get(i, 0)) for i, d in r.data.items()}
        return FastSnapshot(self.seq, r.t_done, r.t_done - r.t_start, axes, r.missing)

    def read_slow(self) -> SlowSnapshot:
        r = self.bus.sync_read(*ct.SLOW, self.ids)
        self.seq += 1
        return SlowSnapshot.decode(self.seq, r.t_done, r.data, r.errors, r.missing)

    def connect(self) -> bool:
        models = {}
        for i in self.ids:
            model, _ = self.bus.ping(i)
            models[i] = model
        missing = [i for i, m in models.items() if m is None]
        slow, fast = self.read_slow(), self.read_fast()
        self.log.event("connected", models=models, missing=missing, bus=self.bus.info,
                       settings={i: a for i, a in slow.axes.items()})
        self.log.session(bus=self.bus.info, ping_models=models, servo_settings=slow.axes,
                         config_sha=self.cfg.robot_sha, motion_sha=self.cfg.motion_sha,
                         config_path=str(self.cfg.robot_path))
        if missing:
            self._set_view(View(state="DISCONNECTED", message=f"응답 없는 ID {missing}: 전원·배선·ID·baud 확인. "
                                f"다른 ID로 대신하지 않습니다"))
            self.log.event("connect_failed", missing=missing)
            return False
        self.sup.on_connect(self.clock(), models, slow, fast)
        self.slow_due = self.clock() + 1.0 / self.cfg.m("loop", "slow_read_hz")
        self._set_view(self.sup.view())
        return True

    # ---------------------------------------------------------------- loop
    def cycle(self) -> None:
        now = self.clock()
        for hook in self.hooks:
            hook(now, self)
        fast = self.read_fast()
        slow = None
        if self.slow_now or now >= self.slow_due:
            slow = self.read_slow()
            self.slow_now = False
            self.slow_due = now + 1.0 / self.cfg.m("loop", "slow_read_hz")
        events = self.inputs.drain() + (self.link.drain(now) if self.link is not None else [])
        for e in events:
            self.log.event("input", kind=e.kind, t_event=e.t, source=e.source)
        cmds = self.sup.step(self.clock(), fast, slow, events)
        for c in cmds:
            self.execute(c)
        view = self.sup.view()
        self._set_view(view)
        self._sample(now, fast, view)

    def execute(self, c) -> None:
        try:
            if isinstance(c, SyncWrite):
                self.sup.command_done(c, self.bus.sync_write(c.addr, c.length, c.values))
            elif isinstance(c, Write):
                err = self.bus.write(c.dxl_id, c.addr, c.data)
                if err:
                    self.log.event("write_status_error", label=c.label, id=c.dxl_id, error=ct.status_error(err))
            elif isinstance(c, FlushInput):
                self.inputs.flush()
            elif isinstance(c, SlowReadNow):
                self.slow_now = True
            elif isinstance(c, LinkSend):
                result = self.link.send(c.msg) if self.link is not None else "no link"
                if result:
                    self.log.event("link_send_failed", type=c.msg.get("type"), error=result)
            elif isinstance(c, SaveSigns):
                result = self.save_signs(c.signs) if self.save_signs else "no writer (not saved)"
                self.log.event("signs_saved", signs=c.signs, result=result)
            elif isinstance(c, SaveCalibration):
                result = self.store.save(c.record, getattr(self.log, "dir", None) and str(self.log.dir)) \
                    if self.store else "no store"
                self.log.event("calibration_stored", why=c.why, written=c.record["written"], result=result)
            elif isinstance(c, DiscardCalibration):
                old = self.store.load() if self.store else None
                result = self.store.discard() if self.store else "no store"
                self.log.event("calibration_discarded", why=c.why, result=result, record=old)
        except WriteBlocked as e:
            self.log.event("write_blocked", label=getattr(c, "label", ""), error=str(e))
            raise
        except BusError as e:
            self.log.event("write_failed", label=getattr(c, "label", ""), error=str(e))

    def run(self) -> None:
        period = 1.0 / self.cfg.m("loop", "read_hz")
        while not self.stop.is_set() and not self.sup.done:
            t0 = self.clock()
            try:
                self.cycle()
                self.errors = 0
            except Exception as e:          # noqa: BLE001 - keep supervising; stop motion if possible
                self.handle_exception(e)
                if self.errors >= 5:
                    self.log.event("runner_stopped", reason="repeated exceptions")
                    break
            time.sleep(max(0.0, period - (self.clock() - t0)))
        self.finish()

    def finish(self) -> None:
        """One last fresh read after the session ended, so the exit report shows the servos as they are now."""
        if not self.sup.done:
            return
        try:
            self.sup.step(self.clock(), self.read_fast(), self.read_slow(), [])
            self._set_view(self.sup.view())
        except Exception as e:      # noqa: BLE001 - the report then stays as it was (and says so if comm is gone)
            self.log.event("final_read_failed", error=repr(e))

    def handle_exception(self, e: Exception) -> None:
        """A cycle raised: log it and, if a HOME is moving, start the normal braking stop (FAULT EXCEPTION)."""
        self.errors += 1
        self.log.event("exception", error=repr(e), trace=traceback.format_exc(limit=5))
        if self.sup.state in (S.HOMING, S.TELEOP, S.TRAJ_MOVING):
            try:
                self.sup._out = []
                self.sup._begin_stop("brake", then=("FAULT", "EXCEPTION", f"프로그램 예외: {e!r}"), reason=f"exception {e!r}")
                for c in self.sup._out:
                    self.execute(c)
            except Exception as e2:     # noqa: BLE001
                self.log.event("exception_stop_failed", error=repr(e2))

    def request_quit(self) -> None:
        self.inputs.put("QUIT")

    # ---------------------------------------------------------------- view & samples
    def _set_view(self, v: View) -> None:
        with self._lock:
            self._view = v

    def view(self) -> View:
        with self._lock:
            return self._view

    def _sample(self, now: float, fast: FastSnapshot, view: View) -> None:
        rec = {"state": view.state, "complete": fast.complete, "missing": list(fast.missing),
               "read_ms": round(fast.duration_s * 1000, 2),
               "present": {i: a.present_position for i, a in fast.axes.items()},
               "pos_traj": {i: a.position_trajectory for i, a in fast.axes.items()},
               "velocity_raw": {i: a.present_velocity for i, a in fast.axes.items()},
               "current_raw": {i: a.present_current for i, a in fast.axes.items()}}
        if view.q_meas is not None:
            rec["q_meas_deg"] = np.degrees(view.q_meas).round(4)
        if view.q_cmd is not None:
            rec["q_cmd_deg"] = np.degrees(view.q_cmd).round(4)
        if view.q_target is not None and view.q_meas is not None:
            rec["err_deg"] = np.degrees(view.q_meas - view.q_target).round(4)
        moving = self.sup.state in MOVING_STATES or self.sup.state in (S.ALIGN, S.CALIBRATING)
        self.log.sample(now, moving, rec)
