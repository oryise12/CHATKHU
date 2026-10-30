"""Browser monitor (2026-10-28, Figma "아이로봇 / 재민님 전달용", white theme).

The camera image with the projected path (overlay, hand cursor) is still drawn here and streamed; the screen around it
- side rail / sidebar, the Chat KHU panel, the 4-step menu (경로설정 → 수정 → 검증 → 이동) with its status chip,
the height graph, alert dialogs and cards, the run / stop buttons - is plain HTML/CSS/JS in web/ (no build step).

Server: ThreadingHTTPServer on 127.0.0.1, a free port. A random token per run guards everything that carries data or
acts on the session: GET /stream (MJPEG of the latest frame), GET /sim (MJPEG of the small 3-D simulator view,
sim_view.py), GET /events (server-sent events with the UI state),
POST /api/input (key / mouse / action). Static files (/, /static/...) carry no data.
The Session is only touched by the main loop: inputs are queued here (drain()), the state is built there (ui_state()).
What the screen shows is decided here (menu_state, dialogs, toasts), so it is tested without a browser.
"""
from __future__ import annotations

import json
import mimetypes
import queue
import secrets
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

STATIC_DIR = Path(__file__).resolve().parent / "web"
KEYS = set("NSVYHXUROEGQ")
MOUSE_KINDS = ("press", "move", "release", "cancel")
ACTIONS = ("step", "run", "stop", "confirm_run", "answer", "undo", "redo", "reset", "viewport", "orbit")
UI_COMMANDS = ("use_current_path", "resume", "edit_return", "return_reverse", "new_target", "save_paths",
               "station_target")
REFUSAL = {"STALE_REVISION": "방금 경로가 바뀌었어요. 다시 확인해 주세요.", "BUSY": "검증이나 이동 중이에요.",
           "BUSY_EXECUTING": "이동 중이에요.", "NOT_ALLOWED": "지금 상태에서는 할 수 없어요.",
           "NO_PATH": "경로가 없어요.", "PATH_FAILED": "검증에 실패한 경로예요. 먼저 수정해 주세요.",
           "ARM_NOT_ARMED": "실물 로봇팔이 무장 전이에요. 실물 제어 창에서 y를 눌러 주세요.",
           "ARM_DISCONNECTED": "실물 로봇팔과 연결되어 있지 않아요."}
MAX_BODY = 64 * 1024
JPEG_QUALITY = 82

# ------------------------------------------------------------------------------------------------ what the screen says
STEPS = ("경로설정", "수정", "검증", "이동")
FAIL_DIALOG = {   # validation failure kind -> (title, line 1, line 2)   (design: 검증 오류 / 초기경로 오류 dialogs)
    "collision": ("충돌 위험 감지", "충돌 위험이 감지되었어요.", "경로를 수정하고 다시 검증해주세요."),
    "joint_limits": ("관절 범위 초과", "로봇팔의 관절 허용 범위를 벗어났어요.", "경로를 수정해주세요."),
    "workspace": ("작업 범위 초과", "설정한 위치로 이동할 수 없어요.", "경로를 수정해주세요."),
    "continuous_ik": ("작업 범위 초과", "설정한 위치로 이동할 수 없어요.", "경로를 수정해주세요."),
}
FAIL_OTHER = ("경로 검증 실패", "경로의 안전성을 확인하지 못했어요.", "경로를 수정하고 다시 검증해주세요.")
TOAST_TITLE = {"refused": "요청을 처리하지 않았어요", "hand_refused": "손 입력 확인", "calib": "카메라 정합",
               "regenerated": "경로를 다시 만들었어요", "auto_run_cancelled": "자동 이동 취소", "cancel": "취소했어요",
               "edit_end": "수정하지 않았어요", "voice_link": "쿠옹이"}
WARN_KINDS = ("refused", "hand_refused", "regenerated", "auto_run_cancelled")
# app questions (voice/sim_link.py) -> answer chips: (label, dispatcher command, extra fields)
ANSWERS = (
    ("검증 후 이동할까요?", (("검증 후 이동", "use_current_path", {"auto_run": True}),)),
    ("계속 갈까요, 돌아갈까요?", (("계속 가기", "resume", {"direction": "forward"}),
                                ("돌아가기", "resume", {"direction": "back"}))),
    ("돌아오는 길을 수정할까요", (("돌아오는 길 수정", "edit_return", {}), ("반대로 돌아가기", "return_reverse", {}))),
    ("새 목표로 다시 해 볼까요?", (("새 목표", "new_target", {}), ("경로 저장", "save_paths", {}))),
)


def menu_state(session) -> dict:
    """The 4-step menu: current step (4 = all done), whether it is an error, and the chip under it.
    chip icon: None | loading | check | pause | warning."""
    st, path = session.state, session.path
    back = getattr(session, "phase", "outbound") == "return"

    def out(step, text, icon=None, error=False):
        return {"step": step, "error": error, "chip": {"text": ("돌아오는 길 · " if back and step < 4 else "") + text,
                                                        "icon": icon}}
    real = getattr(session, "real_arm", None)
    if real is not None and not real.up and st in ("PASSED", "EXECUTING", "ABORTED"):
        return out(3, "로봇 연결 끊김", "warning", True)
    gate = getattr(real, "gate", None)
    if gate is not None and st in ("SELECTED", "PASSED") and gate():     # V / Y are refused until the arm is armed
        return out(2 if st == "SELECTED" else 3, "실물 무장 대기" if real.up else "로봇 연결 끊김", "warning")
    if path is None or st == "HOLD":
        return out(0, "경로 설정 전")
    if st in ("GENERATED", "SELECTED"):
        dragging = session.edit is not None and session.edit.dragging
        if dragging or session.edit_mode:
            return out(1, "경로 수정 중", "loading")
        if st == "SELECTED":
            return out(2, "검증 대기")
        return out(1, "경로 수정 완료", "check") if path.source == "edited" else out(1, "수정 대기")
    if st == "VALIDATING":
        return out(2, "경로 검증 중", "loading")
    if st == "FAILED":
        return out(2, "경로 검증 실패", "warning", True)
    if st == "PASSED":
        return out(3, "이동 대기")
    if st == "EXECUTING":
        return out(3, "로봇팔 이동 중", "loading")
    if st == "DONE":
        return out(4, "이동 완료", "check")
    return out(3, "이동 일시정지", "pause")                     # ABORTED


def _last_kuong(chat: list) -> str:
    return next((t for _, role, t in reversed(chat) if role == "kuong"), "")


def _station_chips(session, last: str) -> list | None:
    """Demo stations (stations.py): answers to 어디로 갈까요? / 남은 ~으로 갈까요? / 다시 이동할까요? /
    설거지 바구니로 이동할까요? - (label, command, fields). None = not a station question."""
    st = getattr(session, "stations", None)
    if not st:
        return None
    go = [(f"{n} {st[n].label}", "station_target", {"station": n}) for n in ("B", "C")]
    if "어디로 갈까요" in last or "다시 이동할까요" in last:
        return go
    rest = [c for c in go if c[2]["station"] not in getattr(session, "visited", [])]
    if "남은 " in last and "갈까요" in last:
        return rest or go
    if last.rstrip().endswith(("로 이동할까요?", "으로 이동할까요?")):
        return [("이동", "use_current_path", {"auto_run": True})]
    return None


def answer_chips(chat: list, rev: int, session=None) -> list:
    """Chips under 쿠옹이's last bubble when it is one of the app's questions (one click = that answer)."""
    last = _last_kuong(chat)
    station = _station_chips(session, last) if session is not None else None
    if station is not None:
        return [{"label": label, "command": cmd, "fields": fields, "rev": rev} for label, cmd, fields in station]
    for key, chips in ANSWERS:
        if key in last:
            return [{"label": label, "command": cmd, "fields": fields, "rev": rev} for label, cmd, fields in chips]
    return []


def suggestions(session, chat: list) -> list[str]:
    """What the user could say now (shown when the chat panel is folded): from the state and 쿠옹이's last question."""
    st, last = session.state, _last_kuong(chat)
    if st in ("EXECUTING", "VALIDATING"):
        return ["멈춰"]
    if st == "ABORTED" or "계속 갈까요, 돌아갈까요" in last:
        return ["계속 가", "돌아가"]
    stations = getattr(session, "stations", None)
    if stations and ("어디로 갈까요" in last or "다시 이동할까요" in last):
        return [f"{stations['B'].label}으로 가자", f"{stations['C'].label}으로 가자"]
    if stations and "남은 " in last and "갈까요" in last:
        return ["응, 가자", "아니"]
    if last.rstrip().endswith(("로 이동할까요?", "으로 이동할까요?")) and "검증 후" not in last:
        return ["응, 이동해", "아니"]
    if "검증 후 이동할까요" in last:
        return ["응", "아니, 더 고칠래"]
    if "돌아오는 길을 수정할까요" in last:
        return ["돌아오는 길 수정할래", "반대로 돌아가"]
    if session.edit_mode or st == "FAILED":
        return ["여기 위로 조금", "앞쪽으로 2센티", "이전으로", "수정 완료"]
    return []


def failure_dialog(report_kinds: list) -> dict:
    title, l1, l2 = next((FAIL_DIALOG[k] for k in report_kinds if k in FAIL_DIALOG), FAIL_OTHER)
    return {"title": title, "lines": [l1, l2], "cancel": "취소", "ok": "경로 수정", "action": "edit"}


def heights(session, tcp) -> dict | None:
    """Height graph: TCP height (cm, table = 0) along the current path, its min / max and where the TCP is now."""
    if session.path is None:
        return None
    z = np.asarray(session.path.points_m, float)
    if len(z) < 2:
        return None
    idx = np.linspace(0, len(z) - 1, min(120, len(z))).round().astype(int)
    series = (z[idx, 2] * 100).round(1)
    now = None if tcp is None else float(np.asarray(tcp)[2] * 100)
    at = None
    if tcp is not None:
        at = float(np.argmin(np.linalg.norm(z[idx] - np.asarray(tcp), axis=1)) / max(1, len(idx) - 1))
    return {"series": series.tolist(), "min": float(series.min()), "max": float(series.max()),
            "min_at": float(np.argmin(series) / (len(series) - 1)), "max_at": float(np.argmax(series) / (len(series) - 1)),
            "now": None if now is None else round(now, 1), "now_at": at,
            "route": "돌아오는 길" if getattr(session, "phase", "outbound") == "return" else "가는 길"}


class Alerts:
    """Session events -> toasts (cards, a few seconds) and dialogs (stay until closed). Listener on the session."""

    def __init__(self, notice_text, keep: int = 4):
        self.notice_text, self.keep = notice_text, keep
        self.toasts: list[dict] = []
        self.dialog: dict | None = None
        self._id = 0

    def _next(self) -> int:
        self._id += 1
        return self._id

    def on_event(self, ev, session) -> None:
        if ev.kind == "validation" and ev.state == "FAILED":
            rep = session.report
            kinds = [c.kind for c in rep.checks if c.status != "PASS"] if rep is not None else []
            self.dialog = dict(failure_dialog(kinds), id=self._next())
            return
        if ev.kind == "new_target" and ev.state == "HOLD":
            self.dialog = {"id": self._next(), "title": "작업 범위 초과", "lines": ["설정한 위치로 이동할 수 없어요.",
                           "새 목표를 다시 만들어 주세요."], "cancel": "취소", "ok": "새 목표", "action": "new_target"}
            return
        if ev.kind == "arm_link" and "lost" in ev.message:
            self.dialog = {"id": self._next(), "title": "로봇 연결 끊김", "lines": ["로봇과의 연결이 끊어졌어요.",
                           "연결 상태를 확인해주세요."], "cancel": "취소", "ok": "확인", "action": "close"}
            return
        text = self.notice_text(ev, session)
        if not text or ev.kind not in TOAST_TITLE:
            return
        warn = ev.kind in WARN_KINDS or (ev.kind == "calib" and ev.data.get("calib") != "verified")
        last = self.toasts[-1] if self.toasts else None
        if last is not None and last["text"] == text and time.monotonic() - last["t"] < 3.0:
            return                                              # the same refusal again right away: one card
        self.toasts.append({"id": self._next(), "title": TOAST_TITLE[ev.kind], "text": text, "warn": warn,
                            "t": time.monotonic()})
        del self.toasts[:-self.keep]

    def toast(self, title: str, text: str, warn: bool = True) -> None:
        self.toasts.append({"id": self._next(), "title": title, "text": text, "warn": warn, "t": time.monotonic()})
        del self.toasts[:-self.keep]

    def current(self, hold_s: float = 6.0) -> list:
        now = time.monotonic()
        return [{k: v for k, v in t.items() if k != "t"} for t in self.toasts if now - t["t"] < hold_s]


def apply_action(session, d, act: dict, dispatcher) -> tuple[list[str], str]:
    """A browser action -> keys for the app's existing key path, or a command of the voice protocol (same Dispatcher,
    same checks: expected_rev is the state the user saw). Returns (keys, why it was refused or "")."""
    name = act["name"]
    if name in ("viewport", "orbit"):                          # page size (logged) / 3-D view: handled by the app
        return [], ""
    if name == "step":                                         # the 4-step menu: start that step
        i = int(act.get("index", -1)) if str(act.get("index", "")).lstrip("-").isdigit() else -1
        if i == 2:
            return (["S", "V"] if session.state == "GENERATED" else ["V"]), ""
        return {0: ["N"], 1: ["E"], 3: ["Y"]}.get(i, []), ""
    if name in ("run", "stop", "undo", "redo", "reset"):
        return [{"run": "Y", "stop": "X", "undo": "U", "redo": "R", "reset": "O"}[name]], ""
    cmd = "use_current_path" if name == "confirm_run" else str(act.get("command", ""))
    if cmd not in UI_COMMANDS:
        return [], "알 수 없는 요청이에요."
    fields = {"auto_run": True} if name == "confirm_run" else dict(act.get("fields") or {})
    rev = act.get("rev", session.rev)
    req = {"v": 1, "type": "command", "request_id": f"ui-{secrets.token_hex(6)}", "command": cmd,
           "expected_rev": int(rev) if str(rev).isdigit() else session.rev, "deadline_unix": None, **fields}
    res = dispatcher.handle(session, d, req, actor="ui")
    if res.get("accepted"):
        return [], ""
    code = str(res.get("reason_code", ""))
    return [], REFUSAL.get(code, f"지금은 할 수 없어요. ({code.lower()})")


def ui_state(session, *, alerts: Alerts, chat: list, hint: str, voice_connected: bool, calib: dict | None,
             arm_badge: str, tcp, frame_wh: tuple[int, int], enabled: dict) -> dict:
    t = session.target
    return {
        "state": session.state, "rev": session.rev, "phase": getattr(session, "phase", "outbound"),
        "menu": menu_state(session), "edit_mode": bool(session.edit_mode),
        "can_undo": bool(session.edit is not None and session.edit.editor.can_undo),
        "can_redo": bool(session.edit is not None and session.edit.editor.can_redo),
        "auto_run": session.auto_run is not None,
        "target": None if t is None else {"r_cm": round(float(t.r_m) * 100, 1),
                                          "theta_deg": round(float(np.degrees(t.theta_rad)), 1)},
        "heights": heights(session, tcp),
        "chat": [{"id": f"{tm:.3f}", "role": role, "text": text} for tm, role, text in chat],
        "chips": answer_chips(chat, session.rev, session),
        "suggest": suggestions(session, chat),
        "hint": hint, "voice": voice_connected, "calib": calib, "arm": arm_badge,
        "toasts": alerts.current(), "dialog": alerts.dialog,
        "enabled": enabled, "frame": list(frame_wh),
    }


# ------------------------------------------------------------------------------------------------ server
class WebUI:
    def __init__(self, static_dir: Path = STATIC_DIR, host: str = "127.0.0.1", port: int = 0):
        self.static_dir = Path(static_dir).resolve()
        self.token = secrets.token_urlsafe(18)
        self.inbox: queue.Queue = queue.Queue()
        self._frame = None
        self._frame_seq = 0
        self._sim = None                 # the small 3-D simulator view (sim_view.py)
        self._sim_seq = 0
        self._state = "{}"
        self._state_seq = 0
        self._cv = threading.Condition()
        self.closing = False
        self.clients = 0
        self.server = ThreadingHTTPServer((host, port), _handler(self))
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}/?t={self.token}"

    def start(self) -> "WebUI":
        threading.Thread(target=self.server.serve_forever, name="webui", daemon=True).start()
        return self

    def push_frame(self, bgr: np.ndarray) -> None:
        with self._cv:
            self._frame, self._frame_seq = bgr, self._frame_seq + 1
            self._cv.notify_all()

    def push_sim(self, bgr: np.ndarray) -> None:
        with self._cv:
            self._sim, self._sim_seq = bgr, self._sim_seq + 1
            self._cv.notify_all()

    def push_state(self, state: dict) -> None:
        text = json.dumps(state, ensure_ascii=False, separators=(",", ":"))
        with self._cv:
            if text != self._state:
                self._state, self._state_seq = text, self._state_seq + 1
                self._cv.notify_all()

    def drain(self) -> list[dict]:
        out = []
        while True:
            try:
                out.append(self.inbox.get_nowait())
            except queue.Empty:
                return out

    def close(self) -> None:
        self.closing = True
        with self._cv:
            self._cv.notify_all()
        self.server.shutdown()
        self.server.server_close()

    # ---- used by the handler threads
    def wait_frame(self, seq: int, timeout: float = 1.0):
        with self._cv:
            self._cv.wait_for(lambda: self._frame_seq != seq or self.closing, timeout)
            return self._frame_seq, self._frame

    def wait_sim(self, seq: int, timeout: float = 1.0):
        with self._cv:
            self._cv.wait_for(lambda: self._sim_seq != seq or self.closing, timeout)
            return self._sim_seq, self._sim

    def wait_state(self, seq: int, timeout: float = 15.0):
        with self._cv:
            self._cv.wait_for(lambda: self._state_seq != seq or self.closing, timeout)
            return self._state_seq, self._state


def parse_input(msg) -> dict | None:
    """A browser input message -> the queued form, or None if it is not one we accept."""
    if not isinstance(msg, dict):
        return None
    kind = msg.get("type")
    if kind == "key":
        k = str(msg.get("key", "")).upper()
        return {"type": "key", "key": k} if len(k) == 1 and k in KEYS else None
    if kind == "mouse":
        try:
            u, v = float(msg["u"]), float(msg["v"])
        except (KeyError, TypeError, ValueError):
            return None
        if msg.get("kind") not in MOUSE_KINDS or not (-0.05 <= u <= 1.05 and -0.05 <= v <= 1.05):
            return None
        return {"type": "mouse", "kind": msg["kind"], "u": u, "v": v}
    if kind == "action" and msg.get("name") in ACTIONS:
        out = {"type": "action", "name": msg["name"]}
        for k in ("index", "command", "rev", "w", "h"):
            if isinstance(msg.get(k), (int, str)) and not isinstance(msg.get(k), bool):
                out[k] = msg[k]
        for k in ("dx", "dy"):                  # orbit: drag on the 3-D view, page pixels
            v = msg.get(k)
            if isinstance(v, (int, float)) and not isinstance(v, bool) and abs(v) <= 5000:
                out[k] = float(v)
        if isinstance(msg.get("fields"), dict):
            out["fields"] = {k: v for k, v in msg["fields"].items() if k in ("auto_run", "direction", "station")}
        return out
    return None


def _handler(ui: WebUI):
    class Handler(BaseHTTPRequestHandler):
        server_version = "pinch_sim"

        def log_message(self, *args):          # quiet: the app prints what matters
            pass

        def _authorized(self, q) -> bool:
            tok = self.headers.get("X-Token") or (q.get("t") or [""])[0]
            return secrets.compare_digest(tok, ui.token)

        def _send(self, code: int, body: bytes = b"", ctype: str = "text/plain; charset=utf-8") -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if body:
                self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path in ("/", "/index.html"):
                return self._static("index.html")
            if u.path.startswith("/static/"):
                return self._static(u.path[len("/static/"):])
            if u.path == "/stream":
                return self._stream(ui.wait_frame) if self._authorized(q) else self._send(403)
            if u.path == "/sim":
                return self._stream(ui.wait_sim) if self._authorized(q) else self._send(403)
            if u.path == "/events":
                return self._events() if self._authorized(q) else self._send(403)
            self._send(404)

        def do_POST(self):
            u = urlparse(self.path)
            if u.path != "/api/input":
                return self._send(404)
            if not self._authorized(parse_qs(u.query)):
                return self._send(403)
            n = int(self.headers.get("Content-Length") or 0)
            if not 0 < n <= MAX_BODY:
                return self._send(413)
            try:
                msg = parse_input(json.loads(self.rfile.read(n).decode("utf-8")))
            except (ValueError, UnicodeDecodeError):
                msg = None
            if msg is None:
                return self._send(400)
            ui.inbox.put(msg)
            self._send(204)

        def _static(self, rel: str) -> None:
            path = (ui.static_dir / rel).resolve()
            if ui.static_dir not in path.parents or not path.is_file():
                return self._send(404)
            ctype = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            if path.suffix == ".woff2":
                ctype = "font/woff2"
            if ctype.startswith("text/") or ctype in ("application/javascript", "image/svg+xml"):
                ctype += "; charset=utf-8"
            self._send(200, path.read_bytes(), ctype)

        def _stream(self, wait) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            ui.clients += 1
            try:
                while not ui.closing:
                    seq, frame = wait(seq)
                    if frame is None:
                        continue
                    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if not ok:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     str(len(jpg)).encode() + b"\r\n\r\n" + jpg.tobytes() + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                ui.clients -= 1

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while not ui.closing:
                    new, state = ui.wait_state(seq)
                    if new == seq:
                        self.wfile.write(b": keep-alive\n\n")            # no change for a while
                    else:
                        seq = new
                        self.wfile.write(b"data: " + state.encode("utf-8") + b"\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
    return Handler


# ------------------------------------------------------------------------------------------------ browser
def _forget_window_size(profile: Path) -> None:
    """Chrome restores an app window's last size from its profile and then ignores the requested one (2026-10-28:
    a 151 x 1060 window after a resize). Drop the remembered placement before each start."""
    prefs = profile / "Default" / "Preferences"
    try:
        doc = json.loads(prefs.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    br = doc.get("browser", {})
    if br.pop("app_window_placement", None) is not None or br.pop("window_placement", None) is not None:
        prefs.write_text(json.dumps(doc), encoding="utf-8")


def open_browser(url: str, kind: str = "auto", screen: tuple | None = None, mode: str = "window",
                 size: tuple[int, int] = (1600, 1000)) -> subprocess.Popen | None:
    """A browser window of its own (own profile, so it is a new process the app can watch: closing it ends the app).
    mode window: a normal resizable window of `size`, centred on `screen` (x, y, w, h of the monitor);
    mode kiosk: full screen, no resizing (Alt+F4 / Q to end). kind: auto | chrome | firefox | none."""
    if kind == "none":
        return None
    profile = Path.home() / ".cache" / "pinch_sim" / "browser"
    sx, sy, sw, sh = screen or (0, 0, 1920, 1080)
    if mode == "kiosk":
        place = ["--kiosk", f"--window-position={sx},{sy}", f"--window-size={sw},{sh}"]
    else:
        w, h = min(size[0], sw), min(size[1], sh)
        place = [f"--window-position={sx + (sw - w) // 2},{sy + (sh - h) // 2}", f"--window-size={w},{h}"]
    chrome = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")
    if kind in ("auto", "chrome") and chrome:
        prof = Path(f"{profile}-chrome")
        prof.mkdir(parents=True, exist_ok=True)
        _forget_window_size(prof)
        return subprocess.Popen([chrome, f"--app={url}", *place, f"--user-data-dir={prof}",
                                 "--no-first-run", "--no-default-browser-check", "--disable-translate",
                                 "--password-store=basic", "--disable-session-crashed-bubble"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    firefox = shutil.which("firefox")
    if kind in ("auto", "firefox") and firefox:
        (Path(f"{profile}-firefox")).mkdir(parents=True, exist_ok=True)
        size_args = ["--kiosk"] if mode == "kiosk" else ["--width", str(min(size[0], sw)), "--height", str(min(size[1], sh))]
        return subprocess.Popen([firefox, "--new-instance", "--profile", f"{profile}-firefox", *size_args, url],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return None
