"""Link to the pinch simulator (pinch_sim --real-arm). This program is the server: it owns the hardware, starts
first and decides who may connect.

AF_UNIX socket $XDG_RUNTIME_DIR/manipulator_control/arm.sock (directory 0700), multiprocessing.connection with an
HMAC key in arm.key (0600, new each run), JSON messages (send_bytes / recv_bytes, never pickle), version 1, one
simulator at a time. Socket threads only move bytes (and parse execute messages); the bus loop calls drain(now),
which turns them into supervisor events (source "link") and runs the handshake and the ping watchdog. Sends go
through a writer thread, so the bus loop never blocks on the socket.

controller -> sim  hello   {role, kind, robot_sha, model_sha, joints, limits_deg, home_q_rad, speed_scale, check_only,
                            accept}   (accept: what check() accepts; the simulator must validate at least as strictly)
                   state   {seq, state, armed, q_meas, q_cmd, stale, fault, execution, message}   (state_hz)
                   ack     {traj_id, accepted, reasons | duration_s, speed_scale | check_only}
                   result  {traj_id, status DONE | STOPPED | FAULT | CHECKED, reason, code, q_final, t_s, metrics}
                   refused {reasons}   (the connection is closed after it)
sim -> controller  hello   {role: "pinch_sim", robot_sha, model_sha, limits_deg, home_q_rad, validation{...}}
                   ping    (the simulator's main loop, >= 5 Hz: proves its stop path is alive)
                   stop    {reason}    -> EXT_STOP (never deadline-checked)
                   execute {traj_id, dt, q, qd, meta}   -> TRAJ (pinch_traj.parse; malformed -> ack refused here)
"""
from __future__ import annotations

import json
import math
import os
import queue
import secrets
import threading
import time
from multiprocessing.connection import Listener
from pathlib import Path

import numpy as np

from .pinch_traj import parse
from .supervisor import Event

PROTOCOL = 1
MAX_MSG = 8 << 20


def default_address() -> Path:
    base = os.environ.get("XDG_RUNTIME_DIR") or f"/tmp/mc-{os.getuid()}"
    return Path(base) / "manipulator_control" / "arm.sock"


def hello_problems(mine: dict, peer: dict) -> list[str]:
    """Why the simulator's hello does not match this program (empty: link up). Same robot.yaml and model, the same
    limits and HOME, and a validation at least as strict as the checks here, so a simulator PASS is not refused here
    for configuration reasons."""
    out = []
    if peer.get("v") != PROTOCOL or peer.get("role") != "pinch_sim":
        return [f"not a pinch simulator hello (v {peer.get('v')}, role {peer.get('role')})"]
    for key in ("robot_sha", "model_sha"):
        if peer.get(key) != mine[key]:
            out.append(f"{key} differs: sim {str(peer.get(key))[:12]} != {mine[key][:12]}")
    for key in ("limits_deg", "home_q_rad"):
        a, b = np.asarray(peer.get(key, []), float), np.asarray(mine[key], float)
        if a.shape != b.shape or not np.allclose(a, b, atol=1e-6):
            out.append(f"{key} differs: sim {peer.get(key)} != {mine[key]}")
    val, acc = peer.get("validation") or {}, mine["accept"]
    checks = (("joint_speed_max_deg_s", "<=", acc["v_cap_deg_s"]), ("qdd_max_deg_s2", "<=", acc["qdd_max_deg_s2"]),
              ("joint_margin_deg", ">=", acc["margin_deg"]))
    for key, op, lim in checks:
        x = val.get(key)
        if x is None or not (x <= lim + 1e-9 if op == "<=" else x >= lim - 1e-9):
            out.append(f"simulator {key} {x} must be {op} {lim}")
    dt = val.get("dt_s")
    if dt is None or not acc["dt_range_s"][0] <= dt <= acc["dt_range_s"][1]:
        out.append(f"simulator sample spacing {dt} outside {acc['dt_range_s']}")
    return out


class ArmLink:
    def __init__(self, hello: dict, view_fn, *, ping_timeout_s: float, state_hz: float, max_samples: int,
                 address: str | Path | None = None, log=None, clock=time.monotonic):
        self.hello = dict(hello, v=PROTOCOL, type="hello", role="manipulator_control")
        self.view_fn, self.log, self.clock = view_fn, log, clock
        self.ping_timeout_s, self.state_hz, self.max_samples = ping_timeout_s, state_hz, max_samples
        self.address = str(address or default_address())
        self.key_path = Path(self.address).with_suffix(".key")
        self.inbox: queue.Queue = queue.Queue()
        self.linked = False
        self._conn = self._out = None
        self._cid = 0
        self._hello_due: float | None = None
        self._last_ping = 0.0
        self._listener = None
        self._closing = False
        self._seq = 0

    # ---------------------------------------------------------------- setup
    def start(self) -> "ArmLink":
        if len(os.fsencode(self.address)) > 100:
            raise ValueError(f"socket path too long for AF_UNIX: {self.address}")
        d = Path(self.address).parent
        d.mkdir(parents=True, exist_ok=True)
        os.chmod(d, 0o700)
        if os.path.exists(self.address):
            os.unlink(self.address)
        key = secrets.token_bytes(32)
        fd = os.open(self.key_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "wb") as f:
            f.write(key)
        self._listener = Listener(self.address, family="AF_UNIX", authkey=key)
        threading.Thread(target=self._accept_loop, name="link-accept", daemon=True).start()
        threading.Thread(target=self._state_loop, name="link-state", daemon=True).start()
        return self

    def close(self) -> None:
        self._closing = True
        self._drop_conn()
        try:
            if self._listener is not None:
                self._listener.close()
        except OSError:
            pass
        for p in (self.address, self.key_path):
            try:
                os.unlink(p)
            except OSError:
                pass

    # ---------------------------------------------------------------- socket threads (bytes only)
    def _accept_loop(self) -> None:
        while not self._closing:
            try:
                conn = self._listener.accept()
            except Exception:              # noqa: BLE001 - wrong key, closed listener
                if self._closing:
                    return
                continue
            self.inbox.put(("connect", conn, None))

    def _reader(self, conn, cid: int) -> None:
        try:
            while True:
                raw = conn.recv_bytes(MAX_MSG)
                try:
                    msg = json.loads(raw.decode("utf-8"))
                except ValueError:
                    msg = {"type": "invalid"}
                if msg.get("type") == "execute":
                    try:
                        msg = {"type": "execute", "traj": parse(msg, self.max_samples)}
                    except ValueError as e:
                        msg = {"type": "execute_bad", "traj_id": msg.get("traj_id"), "error": str(e)}
                self.inbox.put(("msg", cid, msg))
        except (EOFError, OSError, TypeError, ValueError):     # TypeError / ValueError: closed under the read
            self.inbox.put(("disconnect", cid, None))

    @staticmethod
    def _writer(conn, out: queue.Queue) -> None:
        while True:
            item = out.get()
            if item is None:
                return
            try:
                conn.send_bytes(item)
            except (OSError, ValueError, TypeError):
                return

    def _state_loop(self) -> None:
        while not self._closing:
            time.sleep(1.0 / self.state_hz)
            if not self.linked:
                continue
            v = self.view_fn()
            self._seq += 1
            self.send({"type": "state", "seq": self._seq, "state": v.state, "armed": v.armed, "stale": v.stale,
                       "q_meas": None if v.q_meas is None else np.asarray(v.q_meas).tolist(),
                       "q_cmd": None if v.q_cmd is None else np.asarray(v.q_cmd).tolist(),
                       "fault": v.fault, "execution": v.execution, "message": v.message})

    def send(self, msg: dict) -> str | None:
        out = self._out
        if out is None:
            return "not connected"
        out.put(json.dumps(dict(msg, v=PROTOCOL), ensure_ascii=False, default=_json_default).encode("utf-8"))
        return None

    # ---------------------------------------------------------------- bus loop
    def drain(self, now: float) -> list[Event]:
        events = []
        while True:
            try:
                kind, a, msg = self.inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "connect":
                if self._conn is not None:
                    _refuse(a, ["another simulator is already connected"])
                    self._event("link_refused", reason="second client")
                    continue
                self._cid += 1
                self._conn, self._out = a, queue.Queue()
                threading.Thread(target=self._reader, args=(a, self._cid), daemon=True).start()
                threading.Thread(target=self._writer, args=(a, self._out), daemon=True).start()
                self.send(self.hello)
                self._hello_due = now + 3.0
                self._event("link_connect")
            elif kind == "disconnect":
                if a == self._cid:
                    events += self._down(now, "시뮬 연결 끊김")
            elif a == self._cid:
                events += self._message(now, msg)
        if self._conn is not None and not self.linked and self._hello_due is not None and now > self._hello_due:
            self._refuse_current(["no hello within 3 s"])
        if self.linked:
            moving = getattr(self.view_fn(), "state", "") in ("TRAJ_CHECK", "TRAJ_MOVING")
            limit = self.ping_timeout_s * (1 if moving else 5)       # idle: a busy simulator frame is not a loss
            if now - self._last_ping > limit:
                events += self._down(now, f"시뮬 응답 없음 ({limit:.1f} s 넘게 ping 없음)")
        return events

    def _message(self, now: float, msg: dict) -> list[Event]:
        typ = msg.get("type")
        if not self.linked:
            if typ != "hello":
                return []
            problems = hello_problems(self.hello, msg)
            if problems:
                self._refuse_current(problems)
                return []
            self.linked, self._last_ping, self._hello_due = True, now, None
            peer = {k: msg.get(k) for k in ("git", "config_sha", "model_sha", "robot_sha", "pid")}
            self._event("link_up", peer=peer)
            return [Event("LINK_UP", now, peer, "link")]
        if typ == "ping":
            self._last_ping = now
        elif typ == "stop":
            self._last_ping = now
            return [Event("EXT_STOP", now, str(msg.get("reason") or "시뮬 정지"), "link")]
        elif typ == "execute":
            self._last_ping = now
            return [Event("TRAJ", now, msg["traj"], "link")]
        elif typ == "execute_bad":
            self.send({"type": "ack", "traj_id": msg.get("traj_id"), "accepted": False, "reasons": [msg["error"]]})
            self._event("traj_malformed", error=msg["error"])
        return []

    def _down(self, now: float, why: str) -> list[Event]:
        was = self.linked
        self._drop_conn()
        self._event("link_down", reason=why)
        return [Event("LINK_DOWN", now, why, "link")] if was else []

    def _refuse_current(self, reasons: list[str]) -> None:
        self._event("link_refused", reasons=reasons)
        if self._out is not None:
            self.send({"type": "refused", "reasons": reasons})
            time.sleep(0.05)                 # let the writer flush before the close
        self._drop_conn()

    def _drop_conn(self) -> None:
        self.linked = False
        self._hello_due = None
        if self._out is not None:
            self._out.put(None)
        try:
            if self._conn is not None:
                self._conn.close()
        except OSError:
            pass
        self._conn = self._out = None

    def _event(self, kind: str, **kw) -> None:
        if self.log is not None:
            self.log.event(kind, **kw)


def _refuse(conn, reasons: list[str]) -> None:
    try:
        conn.send_bytes(json.dumps({"v": PROTOCOL, "type": "refused", "reasons": reasons}).encode("utf-8"))
        conn.close()
    except OSError:
        pass


def _json_default(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return str(x)
