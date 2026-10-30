"""Local command gateway for the voice process (쿠옹이): JSON over an AF_UNIX socket.

Threads: an accept thread and one reader/writer pair per connection only move bytes between the socket
and two queues. Every request is executed by drain(session, d), which the app calls from its MAIN
loop next to the key handling - so voice, buttons, keys and the mouse all go through the same Session
methods in the same thread, and MjData / UI objects are never touched from another thread.

Protocol (one JSON object per message, multiprocessing.connection framing + HMAC authkey, no pickle):
  request : {"v":1, "type":"command", "request_id":str, "command":str, "expected_rev":int|null,
             "deadline_unix":float|null, ...}   a request still queued after its deadline (the voice process
             has already told the user it was NOT done) is refused as REQUEST_EXPIRED, never executed late
  result  : {"v":1, "type":"result", "request_id", "command", "accepted", "reason_code", "message",
             "previous_state", "state", "rev_before", "rev", "duplicate", "detail", "status"}
  event   : {"v":1, "type":"event", "kind", "message", "state", "rev", "actor", "request_id", "data", "status"}
  hello   : {"v":1, "type":"hello", "status"}                      (sent on connect)
  refused : {"v":1, "type":"refused", "reason"}    (sent to a second client while one is connected, then closed;
            one voice client at a time - the first one keeps the link until it disconnects)
  client  : {"v":1, "type":"client_state", "text":str}            (voice -> sim: one short hint to draw)
  chat    : {"v":1, "type":"chat", "role":"user"|"kuong", "text":str}   (voice -> sim: conversation bubbles;
            what the user said/typed and what 쿠옹이 says - never function names, JSON or settings)
The socket directory is created 0700, the per-run authkey file 0600 (both under $XDG_RUNTIME_DIR).
"""
from __future__ import annotations

import json
import os
import queue
import secrets
import threading
import time
from collections import OrderedDict, deque
from multiprocessing.connection import Listener
from pathlib import Path

PROTOCOL = 1
# commands whose request must name the state revision it was decided on (stale answers are refused)
REV_CHECKED = ("start_edit", "continue_edit", "undo_last_edit", "reset_to_original", "use_current_path",
               "new_target", "return_home", "move_point", "redo_last_edit", "return_reverse", "edit_return", "resume",
               "vision_target", "station_target")
# no revision check: always safe / read-only / link bookkeeping
UNCHECKED = ("stop", "status", "cancel_auto_run", "save_paths")
COMMANDS = REV_CHECKED + UNCHECKED
DIRECTIONS = ("up", "down", "left", "right", "toward_me", "away")
MAX_MOVE_CM = 15.0
SAVE_DIR = Path(__file__).resolve().parents[2] / "saved_paths"   # mujoco_sim/saved_paths
# Session event kinds forwarded to the voice process
FORWARDED = ("state", "point_select", "back_to_edit", "paths_saved", "new_target", "return_path", "select", "validate_start", "validation", "execution_start",
             "execution_end", "cancel", "regenerated", "edit_begin", "edit_end", "edit_cancel", "undo", "redo",
             "reset", "edit_mode", "auto_run_armed", "auto_run_cancelled", "calib", "hand_refused", "stations_ready")


def _clean(text, limit: int) -> str:
    """Printable single-line text for the screen."""
    s = " ".join(str(text).split())
    return "".join(ch for ch in s if ch.isprintable())[:limit]


def default_dir() -> Path:
    base = os.environ.get("XDG_RUNTIME_DIR") or f"/tmp/pinch_sim-{os.getuid()}"
    return Path(base) / "pinch_sim"


class Dispatcher:
    """Request dict -> Session calls -> result dict. Main thread only. No sockets (unit-testable)."""

    def __init__(self, cache_size: int = 256):
        self._done: OrderedDict[str, dict] = OrderedDict()
        self._cache_size = cache_size

    def handle(self, session, d, req: dict, actor: str = "voice") -> dict:
        """actor: who the session events are tagged with (the browser monitor's buttons use "ui")."""
        rid = str(req.get("request_id") or "")
        cmd = str(req.get("command") or "")
        if rid and rid in self._done:                       # duplicated function call: answer, do nothing
            return dict(self._done[rid], duplicate=True)
        before_state, before_rev = session.state, session.rev
        accepted, code, detail = self._run(session, d, cmd, rid, req, actor)
        res = {"v": PROTOCOL, "type": "result", "request_id": rid, "command": cmd, "accepted": bool(accepted),
               "reason_code": "" if accepted else code, "message": session.message,
               "previous_state": before_state, "state": session.state, "rev_before": before_rev,
               "rev": session.rev, "duplicate": False, "detail": detail, "status": session.status()}
        if rid and cmd in COMMANDS and cmd != "status":
            self._done[rid] = res
            while len(self._done) > self._cache_size:
                self._done.popitem(last=False)
        return res

    # ------------------------------------------------------------------ command contract
    def _run(self, s, d, cmd, rid, req, actor="voice"):
        if req.get("v") != PROTOCOL or req.get("type") != "command":
            return False, "BAD_REQUEST", "unsupported message"
        if cmd not in COMMANDS:
            return False, "UNKNOWN_COMMAND", cmd
        if cmd == "status":
            return True, "", ""
        deadline = req.get("deadline_unix")
        # a stop is never too late: stopping later is still better than not stopping (and it has no rev check)
        if cmd != "stop" and isinstance(deadline, (int, float)) and time.time() > deadline:
            return False, "REQUEST_EXPIRED", f"arrived/processed {time.time() - deadline:.1f} s after its deadline"
        if cmd in REV_CHECKED:
            exp = req.get("expected_rev")
            if not isinstance(exp, int):
                return False, "MISSING_REVISION", "expected_rev is required"
            if exp != s.rev:
                return False, "STALE_REVISION", f"decided on rev {exp}, sim is at rev {s.rev}"
        dragging = s.edit is not None and s.edit.dragging
        with s.acting(actor, rid):
            if cmd == "save_paths":
                f = s.save_paths(SAVE_DIR)
                return f is not None, "NOTHING_TO_SAVE", {"file": str(f) if f else ""}
            if cmd == "cancel_auto_run":
                return s.cancel_auto_run(str(req.get("reason") or "voice link")), "NOTHING_BOOKED", ""
            if cmd == "stop":
                what = s.cmd_cancel(d)
                return bool(what), "NOTHING_TO_STOP", what
            if dragging:
                return False, "DRAG_ACTIVE", "release the drag first"
            if cmd in ("start_edit", "continue_edit"):
                return self._start_edit(s, d)
            if cmd == "redo_last_edit":
                ok, why = s.can_edit()
                if not ok:
                    return False, "NOT_EDITABLE", why
                return s._edit_history("redo"), "NOTHING_TO_REDO", s.message
            if cmd in ("undo_last_edit", "reset_to_original"):
                ok, why = s.can_edit()
                if not ok:
                    return False, "NOT_EDITABLE", why
                if cmd == "undo_last_edit" and not s.edit.editor.can_undo:
                    s._emit("undo refused: no edit to undo", "refused")
                    return False, "NOTHING_TO_UNDO", ""
                done = s._edit_history("undo" if cmd == "undo_last_edit" else "reset")
                return done, ("ALREADY_ORIGINAL" if cmd == "reset_to_original" else "REFUSED"), s.message
            if cmd == "new_target":
                if s.state in ("EXECUTING", "VALIDATING"):
                    s._emit(f"N ignored in {s.state}", "refused")
                    return False, "BUSY", s.state
                ok = s.cmd_new_target(d)
                return ok, "NO_TARGET", ""
            if cmd == "station_target":
                # demo stations: "B 국그릇 / C 밥그릇" from S - a straight path there, edit mode on
                name = str(req.get("station", "")).upper()
                if name not in ("B", "C"):
                    return False, "BAD_STATION", name
                ok, code = s.cmd_station(name, d)
                if not ok:
                    return False, code, s.message
                return True, "", {"station": name, "label": s.stations[name].label, "edit_mode": s.edit_mode}
            if cmd == "vision_target":
                ok, code = s.cmd_vision_target(d)
                return ok, code, {"target": None if s.target is None else s.target.summary()} if ok else s.message
            if cmd == "return_home":
                if s.state not in ("HOLD", "DONE", "ABORTED"):
                    s._emit(f"H ignored in {s.state}", "refused")
                    return False, "NOT_ALLOWED", s.state
                return s.cmd_return(d), "NO_PATH", ""
            if cmd == "use_current_path":
                # auto_run false ("수정 완료"): validate only - the user is asked before it moves (approve = again
                # with auto_run true). auto_run true: run as soon as this exact path passes.
                return self._use_current_path(s, d, rid, auto_run=req.get("auto_run", True) is not False)
            if cmd == "move_point":
                return self._move_point(s, req)
            if cmd in ("return_reverse", "edit_return"):
                # at the target: the executed outbound path backwards - run it (validate, auto-run) or edit it
                ok, code = s.cmd_return_reverse(d) if cmd == "return_reverse" else s.cmd_edit_return(d)
                if not ok or cmd == "edit_return":
                    return ok, code, "edit the return path" if ok else s.message
                return self._use_current_path(s, d, rid)
            if cmd == "resume":
                # after a stop: continue to where the interrupted path went, or go back to where it started
                direction = req.get("direction")
                if direction not in ("forward", "back"):
                    return False, "BAD_DIRECTION", str(direction)
                ok, code = s.cmd_resume(d, forward=direction == "forward")
                if not ok:
                    return False, code, s.message
                return self._use_current_path(s, d, rid)
        return False, "UNKNOWN_COMMAND", cmd

    @staticmethod
    def _start_edit(s, d):
        """"경로 수정할래": edit mode ON in the simulator (the hand works only then), for the path there is now.

        VALIDATING: the running validation (and its auto-run booking) is cancelled first - the user wants to change
        the path, not run it. DONE at the target after the outbound move: the return path is what can be edited.
        The detail tells the voice side what the user can use now (hand 3d / 2d / none, calibration state)."""
        if s.state == "VALIDATING":
            s.cmd_cancel(d)                                    # -> SELECTED, booking dropped
        elif s.state == "DONE" and (s.last_exec or {}).get("phase") == "outbound":
            ok, code = s.cmd_edit_return(d)                     # return path, edit mode on
            if not ok:
                return False, code, s.message
        ok, why = s.can_edit()
        if not ok:
            return False, ("NO_PATH" if s.path is None else "NOT_EDITABLE"), why
        s.set_edit_mode(True, "voice")
        return True, "", {"edit_mode": True, "hand": s.hand_mode, "calib": s.calib_state, "phase": s.phase}

    @staticmethod
    def _move_point(s, req):
        """{"target": "here"|"start"|"middle"|"end", "direction": up|down|left|right|toward_me|away,
        "distance_cm": 0 < x <= MAX_MOVE_CM}. "here" = the point the user clicked on the monitor."""
        target, direction, dist = req.get("target"), req.get("direction"), req.get("distance_cm")
        if direction not in DIRECTIONS:
            return False, "BAD_DIRECTION", str(direction)
        if isinstance(dist, bool) or not isinstance(dist, (int, float)) or not 0 < dist <= MAX_MOVE_CM:
            return False, "BAD_DISTANCE", str(dist)
        ok, why = s.can_edit()
        if not ok:
            return False, ("NO_PATH" if s.path is None else "NOT_EDITABLE"), why
        if target == "here":
            k = s.pointed_index()
            if k is None:
                s._emit("voice edit refused: click a path point first", "refused")
                return False, "NO_POINT", "no clicked point"
        elif target in ("start", "middle", "end"):
            k = s.region_index(target)
        else:
            return False, "BAD_TARGET", str(target)
        done, code, moved = s.edit_nudge(k, direction, dist / 100.0)
        return done, code or "REFUSED", {"moved_cm": round(moved * 100, 1), "index": int(k)}

    @staticmethod
    def _use_current_path(s, d, rid, auto_run=True):
        st = s.state
        arm = getattr(s, "real_arm", None)
        if st in ("GENERATED", "SELECTED", "PASSED") and arm is not None and (why := arm.gate()):
            # V / Y would refuse it anyway: say why (not armed yet is not a failed validation)
            s._emit(f"use path refused: {why}", "refused")
            return False, ("ARM_NOT_ARMED" if arm.up and arm.fresh() else "ARM_DISCONNECTED"), why
        if st == "GENERATED":
            if not s.cmd_select():
                return False, "REFUSED", s.message
            st = s.state
        if st == "SELECTED":
            if not auto_run:
                return s.cmd_validate(d), "REFUSED", "validating; asks before moving"
            ok = s.cmd_validate(d, auto_run_request=rid)
            return ok, "REFUSED", "validating; runs automatically only if this exact path passes"
        if not auto_run and st in ("VALIDATING", "PASSED"):
            return True, "", "validating" if st == "VALIDATING" else "passed; waiting for approval"
        if st == "VALIDATING":
            if s.auto_run is not None:
                return False, "ALREADY_REQUESTED", "already booked for this validation"
            return s.arm_auto_run(rid), "REFUSED", "booked for the running validation"
        if st == "PASSED":
            ok = s.cmd_approve(d)            # the same re-check as Y (path, model, config, robot state)
            return ok, "APPROVAL_REFUSED", s.message
        if st == "FAILED":
            s._emit("use path refused: this path failed validation - edit it or reset it first", "refused")
            return False, "PATH_FAILED", ""
        if st == "EXECUTING":
            return False, "BUSY_EXECUTING", ""
        s._emit(f"use path refused in {st}: generate a new target or a return path first", "refused")
        return False, "NOT_ALLOWED", st


def event_message(session, ev) -> dict:
    return {"v": PROTOCOL, "type": "event", "kind": ev.kind, "message": ev.message, "state": ev.state,
            "rev": ev.rev, "actor": ev.actor, "request_id": ev.request_id,
            "data": {k: v for k, v in ev.data.items() if k in ("kept", "edits", "on", "auto_request_id", "reason", "next", "which", "file", "calib")},
            "status": session.status()}


class VoiceGateway:
    """Socket server. start() in any thread; drain()/on_event() from the main loop only."""

    def __init__(self, address: str | None = None, log_path: str | Path | None = None):
        self.log_path = Path(log_path) if log_path else None   # request/result timing (JSONL) for diagnosis
        self.dir = Path(address).parent if address else default_dir()
        self.address = str(address or self.dir / "voice.sock")
        self.key_path = Path(self.address).with_suffix(".key")
        self.inbox: queue.Queue = queue.Queue()
        self.dispatcher = Dispatcher()
        self.client_line = ""             # what the voice process wants shown on side_cam
        self.chat: deque = deque(maxlen=20)   # (t_monotonic, role, text) for the conversation bubbles
        self.connected = False
        self._conn = None
        self._conn_id = 0
        self._out: queue.Queue | None = None
        self._listener = None
        self._session = None
        self._closing = False

    # ------------------------------------------------------------------ setup
    def start(self, session) -> "VoiceGateway":
        if len(os.fsencode(self.address)) > 100:          # AF_UNIX sun_path limit (~107 bytes on Linux)
            raise ValueError(f"voice socket path too long for AF_UNIX ({len(os.fsencode(self.address))} bytes): "
                             f"{self.address}; use a shorter --voice-gateway path")
        self._session = session
        self.dir.mkdir(parents=True, exist_ok=True)
        os.chmod(self.dir, 0o700)
        if os.path.exists(self.address):
            os.unlink(self.address)
        key = secrets.token_bytes(32)
        fd = os.open(self.key_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "wb") as f:
            f.write(key)
        self._listener = Listener(self.address, family="AF_UNIX", authkey=key)
        session.listeners.append(self.on_event)
        threading.Thread(target=self._accept_loop, name="voice-accept", daemon=True).start()
        print(f"voice gateway: {self.address} (key {self.key_path})", flush=True)
        return self

    def close(self) -> None:
        self._closing = True
        try:
            if self._conn is not None:
                self._conn.close()
            if self._listener is not None:
                self._listener.close()
        except OSError:
            pass
        for p in (self.address, self.key_path):
            try:
                os.unlink(p)
            except OSError:
                pass

    # ------------------------------------------------------------------ socket threads (bytes only)
    def _accept_loop(self):
        while not self._closing:
            try:
                conn = self._listener.accept()
            except Exception:              # wrong authkey, closed listener, ...
                if self._closing:
                    return
                continue
            self.inbox.put(("connect", conn, None))

    def _reader(self, conn, cid):
        try:
            while True:
                raw = conn.recv_bytes(1 << 20)
                try:
                    msg = json.loads(raw.decode("utf-8"))
                except ValueError:
                    msg = {"type": "invalid"}
                msg["_t_recv"] = time.time()
                self.inbox.put(("msg", cid, msg))
        except (EOFError, OSError):
            self.inbox.put(("disconnect", cid, None))

    def _writer(self, conn, out: queue.Queue):
        while True:
            item = out.get()
            if item is None:
                return
            try:
                conn.send_bytes(item)
            except (OSError, ValueError):
                return

    def _send(self, msg: dict) -> None:
        if self._out is not None:
            self._out.put(json.dumps(msg, ensure_ascii=False, default=str).encode("utf-8"))

    # ------------------------------------------------------------------ main loop
    def on_event(self, ev) -> None:
        if self.connected and ev.kind in FORWARDED:
            self._send(event_message(self._session, ev))

    def drain(self, session, d) -> int:
        n = 0
        while True:
            try:
                kind, a, msg = self.inbox.get_nowait()
            except queue.Empty:
                return n
            n += 1
            if kind == "connect":
                if self.connected:
                    # one voice client at a time: a second one (e.g. the old ROS kuong_sim, which uses the same
                    # socket and reconnects every second) is refused instead of silently replacing the first
                    self._refuse(a, "another voice client is already connected")
                    session._emit("second voice client refused (one is connected)", "voice_link")
                    continue
                self._conn_id += 1
                self._conn, self._out = a, queue.Queue()
                self.connected = True
                threading.Thread(target=self._reader, args=(a, self._conn_id), daemon=True).start()
                threading.Thread(target=self._writer, args=(a, self._out), daemon=True).start()
                session._emit("voice process connected", "voice_link")
                self._send({"v": PROTOCOL, "type": "hello", "status": session.status()})
            elif kind == "disconnect":
                if a == self._conn_id:
                    self._drop(session, "voice link lost")
            elif a == self._conn_id:
                if msg.get("type") == "client_state":
                    self.client_line = _clean(msg.get("text", ""), 80)
                elif msg.get("type") == "chat" and msg.get("role") in ("user", "kuong"):
                    text = _clean(msg.get("text", ""), 200)
                    if text and not (self.chat and self.chat[-1][1:] == (msg["role"], text)):   # no duplicates
                        self.chat.append((time.monotonic(), msg["role"], text))
                elif msg.get("type") == "command":
                    t0 = time.time()
                    res = self.dispatcher.handle(session, d, msg)
                    self._send(res)
                    self._log({"t_recv": msg.get("_t_recv"), "t_start": t0, "t_done": time.time(),
                               "command": res["command"], "request_id": res["request_id"],
                               "accepted": res["accepted"], "reason": res["reason_code"],
                               "state": res["state"], "rev": res["rev"], "duplicate": res["duplicate"]})

    @staticmethod
    def _refuse(conn, why: str) -> None:
        try:
            conn.send_bytes(json.dumps({"v": PROTOCOL, "type": "refused", "reason": why}).encode("utf-8"))
            conn.close()
        except OSError:
            pass

    def _log(self, rec: dict) -> None:
        if self.log_path is None:
            return
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except OSError:
            pass

    def _drop(self, session, why: str) -> None:
        if not self.connected:
            return
        self.connected = False
        self.client_line = ""
        if self._out is not None:
            self._out.put(None)
        try:
            if self._conn is not None:
                self._conn.close()
        except OSError:
            pass
        self._conn = self._out = None
        session._emit(why, "voice_link")
        session.cancel_auto_run(why)       # a booked auto-run never survives a lost voice link


def connect(address: str | None = None, timeout_s: float = 2.0):
    """Client side helper (stdlib only) for tools/tests: returns a multiprocessing Connection."""
    from multiprocessing.connection import Client
    addr = str(address or default_dir() / "voice.sock")
    key = Path(addr).with_suffix(".key").read_bytes()
    return Client(addr, family="AF_UNIX", authkey=key)
