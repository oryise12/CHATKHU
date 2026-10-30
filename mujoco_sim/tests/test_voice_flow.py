"""The voice side (voice/sim_link.SimEditor) against the real gateway and Session, without the AI.

What 쿠옹이's functions do in the simulator and what it is told to say: "경로 수정할래" really turns edit mode on,
hand edits and voice undo share one history, "수정 완료" asks "검증 후 이동할까요?" and the yes (validate_and_move)
validates and - only if that path passes - runs it, a failure stays visible and returns to edit mode, a booking dies
with an edit, a second voice client is refused, a stale yes is not re-sent. The Session is only touched from one "main loop" thread.
"""
import os
import queue
import shutil
import sys
import tempfile
import threading
import time
from concurrent.futures import Future
from pathlib import Path

import numpy as np
import pytest

from pinch_sim.overlay import build_overlay
from pinch_sim.session import Session
from pinch_sim.voice_gateway import Dispatcher, VoiceGateway

from test_voice_gateway import RM

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))          # the repository root: the voice package
from voice import sim_link  # noqa: E402
from voice.sim_link import SimEditor, SimLinkError  # noqa: E402

AMOUNT = {"small": 2.0, "medium": 5.0, "large": 8.0}


class MainLoop:
    """The app's main loop in a thread: gateway drain, watch (auto-run), physics; `call` runs a function there."""

    def __init__(self, s, d, gw):
        self.s, self.d, self.gw, self._calls, self._stop = s, d, gw, queue.Queue(), False
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while not self._stop:
            while not self._calls.empty():
                fn, fut = self._calls.get()
                try:
                    fut.set_result(fn())
                except Exception as exc:          # noqa: BLE001 - re-raised in the test thread
                    fut.set_exception(exc)
            self.gw.drain(self.s, self.d)
            self.s.watch(self.d)
            for _ in range(50 if self.s.state == "EXECUTING" else 1):
                self.s.physics_step(self.d)
            time.sleep(0.001)

    def call(self, fn):
        fut = Future()
        self._calls.put((fn, fut))
        return fut.result(timeout=30)

    def wait(self, cond, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.call(cond):
                return True
            time.sleep(0.02)
        return False

    def close(self):
        self._stop = True
        self.t.join(2)


@pytest.fixture
def link(scene, cfg, data, request):
    threaded = getattr(request, "param", False)
    short = Path(tempfile.mkdtemp(prefix="pv", dir=os.environ.get("XDG_RUNTIME_DIR") or "/tmp"))  # AF_UNIX length
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=threaded)
    s.cmd_fixed_target(RM, 10.0, data)
    gw = VoiceGateway(str(short / "p" / "voice.sock")).start(s)
    loop = MainLoop(s, data, gw)
    ed = SimEditor(AMOUNT, gw.address)
    yield s, data, gw, loop, ed
    ed.close()
    loop.close()
    gw.close()
    s.close()
    shutil.rmtree(short, ignore_errors=True)


def _hand_drag(s, dz, k=None):
    """A pinch edit as the hand does it (actor hand, free 3-D), run in the main loop."""
    def run():
        kk = s.path.waypoint_index[1] // 2 if k is None else k
        p0 = s.path.points_m[kk].copy()
        with s.acting("hand"):
            assert s.edit_begin(kk, p0, free=True)
            s.edit_update(p0 + [0.0, 0.0, dz])
            assert s.edit_end()
        return s.path.fingerprint
    return run


def _synced(loop, s, ed):
    """The voice side has seen the newest state (the user speaks after the hand edit, not during it)."""
    assert loop.wait(lambda: ed.rev == s.rev, 5.0)


def _said(ed, timeout=5.0):
    out, t0 = [], time.time()
    while time.time() - t0 < timeout:
        try:
            out.append(ed.announcements.get(timeout=0.1))
        except queue.Empty:
            pass
    return out


def test_edit_mode_hand_edit_voice_undo_finish_asks_and_yes_runs(link):
    s, d, gw, loop, ed = link
    loop.call(lambda: (setattr(s, "hand_mode", "3d"), setattr(s, "calib_state", "verified")))
    fp0 = loop.call(lambda: s.path.fingerprint)
    assert not loop.call(lambda: s.edit_mode)
    r = ed.handle("start_edit", {})                                         # "쿠옹아, 경로 수정할래"
    assert r["status"] == "applied" and r["say"] == sim_link.EDIT_GUIDE
    assert loop.call(lambda: s.edit_mode)                                    # the simulator really is in edit mode
    loop.call(_hand_drag(s, 0.03))                                           # a pinch edit (same Session)
    assert sim_link.HAND_EDITED in _said(ed, 2.0)
    _synced(loop, s, ed)
    assert ed.handle("undo_edit", {})["status"] == "applied"                 # "이전으로" undoes the HAND edit
    assert loop.call(lambda: s.path.fingerprint) == fp0
    loop.call(_hand_drag(s, 0.03))
    _synced(loop, s, ed)
    r = ed.handle("finish_edit", {})                                         # "수정 완료"
    assert r == {"status": "asking"}
    assert sim_link.ASK_RUN in _said(ed, 1.0)                                 # "검증 후 이동할까요?"
    assert loop.call(lambda: s.state not in ("VALIDATING", "EXECUTING"))      # nothing runs before the answer
    r = ed.handle("validate_and_move", {})                                   # "어"
    assert r == {"status": "validating", "next": "moves_if_passed"}
    assert loop.wait(lambda: s.state == "DONE")
    said = _said(ed, 1.5)
    assert sim_link.PASSED_MOVE in said and sim_link.ASK_ARRIVED in said     # no second question after the yes
    assert not any("이동할까요" in t for t in said)
    run = loop.call(lambda: s.last_run)
    assert run["status"] == "DONE" and run["path_source"] == "edited"


def test_a_failed_path_stays_visible_and_returns_to_edit_mode(link):
    s, d, gw, loop, ed = link
    ed.handle("start_edit", {})
    k = loop.call(lambda: s.path.waypoint_index[1] - 3)                      # near the target, low already
    loop.call(_hand_drag(s, -0.15, k=k))                                     # pushed below the desk
    _synced(loop, s, ed)
    assert ed.handle("finish_edit", {})["status"] == "asking"
    r = ed.handle("validate_and_move", {})
    assert r["status"] == "validating"
    assert loop.wait(lambda: s.state == "FAILED")
    said = " ".join(_said(ed, 1.5))
    assert "검증을 통과하지 못했어요" in said and "% 지점" in said and "수정 모드로 돌아왔어요" in said
    assert loop.call(lambda: s.edit_mode) and loop.call(lambda: s.state) == "FAILED"
    time.sleep(0.5)
    assert loop.call(lambda: s.state) == "FAILED"                            # never moved
    loop.call(lambda: s.point_select(s.path.waypoint_index[1] // 3))         # a later event (a click) ...
    ov = loop.call(lambda: build_overlay(s, d))
    assert ov.failures                                                       # ... does not hide the red points
    _synced(loop, s, ed)
    assert ed.handle("finish_edit", {}) == {"status": "failed", "reason": "path_failed"}   # not asked again
    assert ed.handle("validate_and_move", {})["reason"] == "path_failed"    # the same path is not run


@pytest.mark.parametrize("link", [True], indirect=True)
def test_saying_edit_during_validation_drops_the_booking(link):
    """Validation in its worker thread (as in the app); "경로 수정할래" while it runs: never moves."""
    s, d, gw, loop, ed = link
    loop.call(_hand_drag(s, 0.03))
    _synced(loop, s, ed)
    assert ed.handle("validate_and_move", {})["status"] == "validating"
    assert loop.call(lambda: s.state == "VALIDATING" and s.auto_run is not None)
    _synced(loop, s, ed)
    r = ed.handle("start_edit", {})
    assert r["status"] == "applied"
    assert loop.call(lambda: s.auto_run is None and s.edit_mode and s.state == "SELECTED")
    time.sleep(12.0)                                                         # longer than the validation
    assert loop.call(lambda: s.state) == "SELECTED"


def test_a_stale_finish_is_not_resent(link):
    s, d, gw, loop, ed = link
    ed.rev -= 1                                                              # decided before a change it did not see
    assert ed.handle("validate_and_move", {}) == {"status": "failed", "reason": "path_changed"}
    assert loop.call(lambda: s.state) == "GENERATED"


def test_a_second_voice_client_is_refused(link):
    s, d, gw, loop, ed = link
    with pytest.raises(SimLinkError, match="another voice client"):
        SimEditor(AMOUNT, gw.address)
    assert ed.handle("start_edit", {})["status"] == "applied"                # the first one still works


def test_stop_is_never_refused_as_too_late(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    s.cmd_select()
    s.cmd_validate(data)
    assert s.cmd_approve(data) and s.state == "EXECUTING"
    r = Dispatcher().handle(s, data, {"v": 1, "type": "command", "command": "stop", "request_id": "late",
                                      "expected_rev": None, "deadline_unix": time.time() - 5})
    assert r["accepted"] and s.state == "ABORTED"
