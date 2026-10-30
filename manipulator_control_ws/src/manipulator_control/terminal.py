"""Terminal keys (Korean prompts live here, the viewer overlay is ASCII).

Enter = START, s/space = STOP, q = QUIT, x = TORQUE_OFF (asks for y), y = CONFIRM, n = NEW_SESSION, r = RETRY.
EE teleop: i/k = +X/-X, j/l = +Y/-Y, u/o = +Z/-Z, 1/2 3/4 5/6 = rotate about X/Y/Z (base frame), [ ] = step
down/up, h = HOME, t = start/resume EE teleop. The control panel (panel.py) puts its events here too (put()).
Events carry their time; flush() drops typed-ahead input (tcflush) so a WAIT_REFERENCE / confirmation only
accepts keys pressed after it was shown. The terminal mode is restored on exit.
"""
from __future__ import annotations

import os
import queue
import select
import sys
import termios
import threading
import time
import tty

import numpy as np

from .supervisor import Event

KEEP_ON_FLUSH = frozenset({"STOP", "QUIT", "TORQUE_OFF_NOW"})   # a flush never drops a stop

KEYS = {"\n": "START", "\r": "START", " ": "STOP", "s": "STOP", "q": "QUIT", "x": "TORQUE_OFF", "y": "CONFIRM",
        "n": "NEW_SESSION", "r": "RETRY", "h": "HOME", "t": "TELEOP"}
JOG_KEYS = {"i": "+x", "k": "-x", "j": "+y", "l": "-y", "u": "+z", "o": "-z",
            "1": "+rx", "2": "-rx", "3": "+ry", "4": "-ry", "5": "+rz", "6": "-rz"}
STEP_KEYS = {"[": "down", "]": "up"}


class TerminalInput:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.q: queue.Queue = queue.Queue()
        self.tty = self.stream.isatty()
        self._old = None
        self._stop = threading.Event()
        self.flushed_at = 0.0

    def __enter__(self):
        if self.tty:
            fd = self.stream.fileno()
            self._old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        threading.Thread(target=self._run, name="terminal-input", daemon=True).start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._old is not None:
            termios.tcsetattr(self.stream.fileno(), termios.TCSADRAIN, self._old)

    def _run(self) -> None:
        fd = self.stream.fileno()
        while not self._stop.is_set():
            r, _, _ = select.select([fd], [], [], 0.1)
            if not r:
                continue
            ch = os.read(fd, 1).decode(errors="ignore")
            if not ch:
                return
            c = ch.lower()
            if c in JOG_KEYS:
                self.q.put(Event("JOG", time.monotonic(), JOG_KEYS[c]))
                continue
            if c in STEP_KEYS:
                self.q.put(Event("STEP", time.monotonic(), STEP_KEYS[c]))
                continue
            kind = KEYS.get(c)
            if kind is None and ch.isprintable():
                kind = "CANCEL"
            if kind:
                self.q.put(Event(kind, time.monotonic()))

    def put(self, kind: str, arg=None, source: str = "ui") -> None:
        """Panel / viewer events (not the operator's terminal: they cannot confirm or start)."""
        self.q.put(Event(kind, time.monotonic(), arg, source))

    def drain(self) -> list[Event]:
        out = []
        while True:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                return out

    def flush(self) -> None:
        if self.tty:
            termios.tcflush(self.stream.fileno(), termios.TCIFLUSH)
        for e in self.drain():
            if e.kind in KEEP_ON_FLUSH:
                self.q.put(e)
        self.flushed_at = time.monotonic()


class ScriptedInput:
    """Timed events for --fake runs and tests: [(seconds after start, kind), ...]."""

    def __init__(self, script, clock):
        self.script, self.clock = sorted(script), clock
        self.t0 = clock()
        self.extra: list[Event] = []
        self._lock = threading.Lock()        # put() also comes from the viewer / panel threads

    def put(self, kind: str, arg=None, source: str = "script") -> None:
        with self._lock:
            self.extra.append(Event(kind, self.clock(), arg, source))

    def drain(self) -> list[Event]:
        now = self.clock()
        out = [Event(k, now, None, "script") for t, k in self.script if t <= now - self.t0]
        self.script = [(t, k) for t, k in self.script if t > now - self.t0]
        with self._lock:
            out, self.extra = out + self.extra, []
        return out

    def flush(self) -> None:
        pass


def print_status(view, last: dict, stream=sys.stdout) -> None:
    """Print the Korean message when it changes, plus a compact angle line at most once a second."""
    now = time.monotonic()
    if view.message and view.message != last.get("message"):
        stream.write(f"\n[{view.state}] {view.message}\n")
        last["message"] = view.message
    for w in view.warnings:
        if w not in last.setdefault("warned", set()):
            stream.write(f"  ! {w}\n")
            last["warned"].add(w)
    if now - last.get("t", 0.0) >= 1.0:
        last["t"] = now
        if view.progress:
            stream.write(f"  > {view.progress}{'  (STALE)' if view.stale else ''}\n")
        elif view.q_meas is not None:
            line = " ".join(f"{x:8.3f}" for x in np.degrees(view.q_meas))
            stream.write(f"  q[deg] {line}{'  (STALE)' if view.stale else ''}\n")
        elif view.rel_deg is not None:
            line = " ".join(f"{x:8.3f}" for x in view.rel_deg)
            stream.write(f"  상대 변화[deg, s=+1 가정] {line}{'  (STALE)' if view.stale else ''}\n")
    stream.flush()
