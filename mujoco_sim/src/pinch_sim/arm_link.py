"""The simulator's side of the real-arm link (pinch_sim.app --real-arm). The arm's program
(`python -m manipulator_control pinch`, <repo>/manipulator_control_ws) is the server and owns the arm.

Protocol (version 1, JSON over multiprocessing.connection with the HMAC key file next to the socket): the server
sends hello; we answer with ours (real_arm.hello: robot.yaml sha, model sha, limits, HOME, how strictly we validate);
a mismatch is "refused". Then the server streams "state" (measured q, state, armed, execution) and answers each
"execute" with "ack" and later "result" (DONE / STOPPED / FAULT / CHECKED). We send "ping" from the main loop (the
server stops a moving arm when they stop) and "stop" for X / voice "멈춰".

Socket threads only move bytes; drain() runs in the main loop next to the voice gateway's.

RealArmExecutor has the TrajectoryExecutor interface (step / status / reason / metrics / trace / traj) but the motion
is the real arm's: the simulator shows the measured pose (digital twin), and the run ends only with the arm's result.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from multiprocessing.connection import Client
from pathlib import Path

import numpy as np

from .control import ExecMetrics

PROTOCOL = 1
MAX_MSG = 1 << 20
ARMED_STATES = ("ARMED",)


def default_address() -> Path:
    base = os.environ.get("XDG_RUNTIME_DIR") or f"/tmp/mc-{os.getuid()}"
    return Path(base) / "manipulator_control" / "arm.sock"


class LinkRefused(ConnectionError):
    pass


class ArmLink:
    def __init__(self, hello: dict, address: str | Path | None = None, clock=time.monotonic,
                 state_timeout_s: float = 0.5, ping_hz: float = 5.0):
        self.hello = hello
        self.address = str(address or default_address())
        self.clock = clock
        self.state_timeout_s, self.ping_period = state_timeout_s, 1.0 / ping_hz
        self.inbox: queue.Queue = queue.Queue()
        self.out: queue.Queue = queue.Queue()
        self.conn = None
        self.up = False
        self.peer: dict = {}
        self.state: dict = {}            # the newest "state" message
        self.t_state = -1e9
        self._t_ping = -1e9
        self.down_reason = ""
        self._finished: set = set()      # traj_ids whose result arrived
        self._main_loop = False          # until the main loop drains, a keepalive thread pings (start-up)

    # ---------------------------------------------------------------- connect (blocking, at start)
    def connect(self, timeout_s: float = 3.0) -> "ArmLink":
        key_path = Path(self.address).with_suffix(".key")
        try:
            key = key_path.read_bytes()
            self.conn = Client(self.address, family="AF_UNIX", authkey=key)
        except (OSError, EOFError) as e:
            raise ConnectionError(f"실물 제어 프로그램에 연결할 수 없습니다 ({self.address}: {e}). 다른 터미널에서 "
                                  "python -m manipulator_control pinch 를 먼저 실행하고 HOME까지 진행하세요") from None
        if not self.conn.poll(timeout_s):
            raise ConnectionError("실물 제어 프로그램이 hello를 보내지 않습니다")
        self.peer = json.loads(self.conn.recv_bytes(MAX_MSG).decode("utf-8"))
        if self.peer.get("type") == "refused":
            raise LinkRefused("; ".join(self.peer.get("reasons") or [str(self.peer)]))
        self.conn.send_bytes(json.dumps(dict(self.hello, pid=os.getpid())).encode("utf-8"))
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:            # the first state (or a refusal) proves the handshake
            if not self.conn.poll(0.1):
                continue
            msg = json.loads(self.conn.recv_bytes(MAX_MSG).decode("utf-8"))
            if msg.get("type") == "refused":
                raise LinkRefused("; ".join(msg.get("reasons") or []))
            if msg.get("type") == "state":
                self.state, self.t_state, self.up = msg, self.clock(), True
                break
        if not self.up:
            raise ConnectionError("실물 제어 프로그램의 상태가 오지 않습니다")
        threading.Thread(target=self._reader, name="arm-link-read", daemon=True).start()
        threading.Thread(target=self._writer, name="arm-link-write", daemon=True).start()
        threading.Thread(target=self._keepalive, name="arm-link-startup-ping", daemon=True).start()
        return self

    def _keepalive(self) -> None:
        """Pings while the simulator is still starting (windows, the first path): nothing can be executed or stopped
        from here yet. The main loop's drain() takes over, so a stalled main loop still loses the link."""
        while self.up and not self._main_loop:
            self.send({"type": "ping"})
            time.sleep(self.ping_period)

    def close(self) -> None:
        self.out.put(None)
        try:
            if self.conn is not None:
                self.conn.close()
        except OSError:
            pass

    # ---------------------------------------------------------------- threads (bytes only)
    def _reader(self) -> None:
        try:
            while True:
                raw = self.conn.recv_bytes(MAX_MSG)
                try:
                    self.inbox.put(json.loads(raw.decode("utf-8")))
                except ValueError:
                    pass
        except (EOFError, OSError, TypeError, ValueError):     # TypeError / ValueError: closed under the read
            self.inbox.put({"type": "_down", "reason": "실물 제어 프로그램과 연결이 끊김"})

    def _writer(self) -> None:
        while True:
            item = self.out.get()
            if item is None:
                return
            try:
                self.conn.send_bytes(item)
            except (OSError, ValueError, TypeError):
                return

    def send(self, msg: dict) -> None:
        if self.up:
            self.out.put(json.dumps(dict(msg, v=PROTOCOL), default=_json_default).encode("utf-8"))

    # ---------------------------------------------------------------- main loop
    def drain(self) -> list[dict]:
        """Ping, then every message since the last call (state / ack / result / refused) plus a "_down" message
        when the link is lost (end of stream, or no state for state_timeout_s)."""
        now = self.clock()
        self._main_loop = True
        if self.up and now - self._t_ping >= self.ping_period:
            self._t_ping = now
            self.send({"type": "ping"})
        msgs = []
        while True:
            try:
                msg = self.inbox.get_nowait()
            except queue.Empty:
                break
            if msg.get("type") == "state":
                ex = msg.get("execution") or {}
                if ex.get("traj_id") in self._finished:   # made before that run's result, delivered after it
                    self.t_state = now
                else:
                    self.state, self.t_state = msg, now
            elif msg.get("type") == "result" and msg.get("state_after"):
                self._finished.add(msg.get("traj_id"))
                # the arm's state right after the result (its state stream may still say TRAJ_MOVING for a moment)
                after = msg["state_after"]
                self.state = dict(self.state, state=after, armed=after == "ARMED", execution=None,
                                  q_meas=msg.get("q_final") or self.state.get("q_meas"))
            if msg.get("type") in ("_down", "refused"):
                if self.up:
                    msgs.append(self._went_down(msg.get("reason") or "; ".join(msg.get("reasons") or [])))
                continue
            msgs.append(msg)
        if self.up and now - self.t_state > self.state_timeout_s:
            msgs.append(self._went_down(f"실물 상태가 {self.state_timeout_s:.1f} s 넘게 오지 않음"))
        return msgs

    def _went_down(self, why: str) -> dict:
        self.up, self.down_reason = False, why
        try:
            self.send({"type": "stop", "reason": "link lost"})
        finally:
            self.close()
        return {"type": "_down", "reason": why}

    # ---------------------------------------------------------------- views
    def fresh(self) -> bool:
        return self.up and self.clock() - self.t_state <= self.state_timeout_s and not self.state.get("stale", True)

    @property
    def armed(self) -> bool:
        return self.fresh() and bool(self.state.get("armed")) and self.state.get("state") in ARMED_STATES

    def q_meas(self) -> np.ndarray | None:
        q = self.state.get("q_meas")
        return None if q is None else np.asarray(q, float)

    def gate(self) -> str:
        """Why V / Y must not start now ("" = they may): the arm must be connected, armed and reporting."""
        if not self.up:
            return f"실물 연결 끊김 ({self.down_reason or '연결 안 됨'})"
        if not self.fresh():
            return "실물 상태가 최신이 아님"
        if self.state.get("state") != "ARMED":
            st = self.state.get("state")
            hint = {"ARM_CONFIRM": "실물 터미널에서 y로 무장", "HOLDING": "실물 터미널에서 y로 다시 무장",
                    "LINK_WAIT": "실물이 연결을 확인하는 중"}.get(st, "")
            return f"실물이 무장 상태가 아님 ({st}{': ' + hint if hint else ''})"
        return ""

    def badge(self) -> str:
        if not self.up:
            return f"실물: 연결 끊김 - {self.down_reason}"
        st = self.state.get("state", "?")
        ex = self.state.get("execution") or {}
        if ex:
            return f"실물: {st} {ex.get('t_s', 0):.1f}/{ex.get('T_s', 0):.1f} s"
        return f"실물: {st}" + (" (무장: 승인하면 실물이 움직임)" if self.armed else "")


class RealArmExecutor:
    """The approved trajectory on the real arm. Interface of control.TrajectoryExecutor; finished only by the arm's
    result (or a lost link / a timeout). The simulator's pose is the measured one (session twin mode)."""

    ACK_TIMEOUT_S = 6.0          # the arm checks the trajectory first (<= 3 s)
    STOP_TIMEOUT_S = 3.0

    def __init__(self, link: ArmLink, m, idx, traj, report, kin, t0: float, dt: float, move: str = ""):
        self.link, self.m, self.idx, self.traj, self.kin, self.t0 = link, m, idx, traj, kin, t0
        self.status, self.reason = "RUNNING", ""
        self.metrics = ExecMetrics()
        self.trace: list = []
        self.refused = False
        self.check_only = False
        self.traj_id = f"{report.report_id}"
        self._t_sent = link.clock()
        self._t_ack = None
        self._t_stop = None
        tt, qq, qd, _ = traj.dense(dt)
        self.duration = float(tt[-1])
        link.send({"type": "execute", "traj_id": self.traj_id, "dt": float(dt), "q": qq, "qd": qd,
                   "meta": {"report_id": report.report_id, "path_fp": report.binding.path_fp,
                            "robot_sha": link.hello["robot_sha"], "model_sha": link.hello["model_sha"],
                            "move": move}})

    # ---------------------------------------------------------------- from the session
    def step(self, d) -> None:
        if self.status != "RUNNING":
            raise RuntimeError("executor finished")
        now = self.link.clock()
        if self._t_ack is None and now - self._t_sent > self.ACK_TIMEOUT_S:
            self._end("ABORTED", "실물이 궤적에 답하지 않음 (확인 시간 초과)")
        elif self._t_stop is not None and now - self._t_stop > self.STOP_TIMEOUT_S:
            self._end("ABORTED", "정지를 보냈지만 실물의 정지 확인이 오지 않음 (상태 미확인: 실물을 직접 확인하세요)")

    def request_stop(self, reason: str) -> None:
        """X / voice: ask the arm to stop; the run stays RUNNING until the arm confirms (result)."""
        if self._t_stop is None:
            self._t_stop = self.link.clock()
            self.link.send({"type": "stop", "reason": reason})

    def _abort(self, reason: str) -> None:          # TrajectoryExecutor interface (used by stop paths)
        self.request_stop(reason)

    def lost(self, reason: str) -> None:
        self._end("ABORTED", f"실물 상태 모름: {reason}")

    def on_twin(self, q_meas, exec_t: float | None, d) -> None:
        """Every new measured pose: path trace and errors against the trajectory at the arm's execution time."""
        p = self.kin.fk(q_meas).p_m
        self.trace.append(p.copy())
        met = self.metrics
        met.steps += 1
        if exec_t is not None:
            q_d, _, _ = self.traj.sample(min(max(exec_t, 0.0), self.traj.duration))
            err = float(np.max(np.abs(q_d - q_meas)))
            met.max_q_err_rad = max(met.max_q_err_rad, err)
            met.sum_sq_q_err += float(np.sum((q_d - q_meas) ** 2))
            met.max_tcp_err_m = max(met.max_tcp_err_m, float(np.linalg.norm(p - self.traj.tcp_at(exec_t))))

    def on_message(self, msg: dict, q_meas=None) -> None:
        if msg.get("traj_id") != self.traj_id or self.status != "RUNNING":
            return
        if msg["type"] == "ack":
            self._t_ack = self.link.clock()
            if not msg.get("accepted"):
                self.refused = True
                self._end("ABORTED", "실물이 궤적을 거부: " + "; ".join(msg.get("reasons") or []))
            self.check_only = bool(msg.get("check_only"))
        elif msg["type"] == "result":
            q = np.asarray(msg["q_final"], float) if msg.get("q_final") is not None else q_meas
            if q is not None:
                self.metrics.final_tcp_err_m = float(np.linalg.norm(self.kin.fk(q).p_m - self.traj.tcp_at(self.duration)))
                self.metrics.final_qd_max_rad_s = 0.0
            st = msg.get("status")
            if st == "DONE":
                self._end("DONE", "")
            elif st == "CHECKED":
                self._end("ABORTED", "검사만 모드: 실물은 움직이지 않음")
            else:
                self._end("ABORTED", f"실물 {st}: {msg.get('reason') or msg.get('code') or ''}")

    def _end(self, status: str, reason: str) -> None:
        if self.status == "RUNNING":
            self.status, self.reason = status, reason

    def progress(self, d) -> float:
        ex = self.link.state.get("execution") or {}
        return float(np.clip(ex.get("t_s", 0.0) / max(ex.get("T_s", self.duration), 1e-6), 0.0, 1.0))


def _json_default(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return str(x)
