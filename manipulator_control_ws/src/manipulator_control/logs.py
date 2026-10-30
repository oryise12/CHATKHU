"""Session logs: logs/<timestamp>_<purpose>[_fake]/ events.jsonl, samples.jsonl, session.json, home_report.json.

Every state change, command, rejection, warning and decision is an event. Samples are rate-limited (fast while
moving, slow while idle) so a long session does not write every bus cycle. Nothing here is read back by the
program: the session's P_ref is a record, never a restore source.
"""
from __future__ import annotations

import datetime as dt
import json
import threading
import time
from pathlib import Path

import numpy as np

from . import WS


def _default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.integer, np.floating)):
        return o.item()
    return str(o)


class SessionLog:
    def __init__(self, purpose: str, fake: bool, root: Path | None = None, echo=None):
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        root = root or WS / "logs" / ("fake" if fake else "sessions")
        self.dir = root / f"{stamp}_{purpose}"
        n = 1
        while self.dir.exists():
            n += 1
            self.dir = root / f"{stamp}_{purpose}_{n}"
        self.dir.mkdir(parents=True)
        self._lock = threading.Lock()
        self._session: dict = {"purpose": purpose, "fake": fake, "started": dt.datetime.now().astimezone().isoformat()}
        self._last_sample = 0.0
        self.echo = echo
        self.events: list[dict] = []       # kept in memory for tests and the final report

    def _append(self, name: str, record: dict) -> None:
        with self._lock, open(self.dir / name, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False, default=_default) + "\n")

    def event(self, name: str, /, **fields) -> None:
        rec = {"t": time.monotonic(), "wall": dt.datetime.now().isoformat(timespec="milliseconds"), "event": name, **fields}
        self.events.append(rec)
        self._append("events.jsonl", rec)
        if self.echo is not None:
            self.echo(rec)

    def sample(self, now: float, moving: bool, record: dict, hz_moving: float = 10.0, hz_idle: float = 1.0) -> None:
        if now - self._last_sample < 1.0 / (hz_moving if moving else hz_idle):
            return
        self._last_sample = now
        self._append("samples.jsonl", {"t": now, **record})

    def session(self, **fields) -> None:
        with self._lock:
            self._session.update(fields)
            (self.dir / "session.json").write_text(json.dumps(self._session, ensure_ascii=False, indent=1,
                                                              default=_default), encoding="utf-8")

    def home_report(self, report: dict) -> None:
        (self.dir / "home_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=_default),
                                                   encoding="utf-8")
        self.event("home_report_written", path=str(self.dir / "home_report.json"))


class NullLog(SessionLog):
    """In-memory only (tests)."""

    def __init__(self):
        self.events, self._session, self._last_sample, self.echo = [], {}, 0.0, None
        self.samples, self.reports = [], []

    def _append(self, name, record):
        if name == "samples.jsonl":
            self.samples.append(record)

    def session(self, **fields):
        self._session.update(fields)

    def home_report(self, report):
        self.reports.append(report)
        self.event("home_report_written")
