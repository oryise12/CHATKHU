"""Voice gateway: command contract on the real Session, and the auto-run boundaries.

Auto-run = a voice "use this path" books ONE validation; it starts (through cmd_approve and its binding
re-check) only if that validation passes for the same path version. Buttons keep their old behavior.
"""
import threading
import time

import numpy as np
import pytest

from pinch_sim.model import load_config
from pinch_sim.session import Session
from pinch_sim.voice_gateway import Dispatcher, VoiceGateway, connect

_T = load_config()["target"]
RM = round((_T["r_min_m"] + _T["r_max_m"]) / 2, 3)


def _session(scene, cfg, data, threaded=False):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=threaded)
    s.cmd_fixed_target(RM, 10.0, data)
    return s


class Voice:
    """Requests as the voice process sends them (expected_rev = the rev it decided on)."""

    def __init__(self, s, d):
        self.s, self.d, self.disp, self.n = s, d, Dispatcher(), 0

    def __call__(self, cmd, rev=None, rid=None):
        self.n += 1
        return self.disp.handle(self.s, self.d, {"v": 1, "type": "command", "command": cmd,
                                                 "request_id": rid or f"call-{self.n}",
                                                 "expected_rev": self.s.rev if rev is None else rev})


def _drag(s, dz, k=None):
    k = s.path.waypoint_index[1] // 2 if k is None else k
    p0 = s.path.points_m[k].copy()
    assert s.edit_begin(k, p0)
    s.edit_update(p0 + [0.0, 0.0, dz])
    assert s.edit_end()


def _run_to_end(s, d):
    while s.state == "EXECUTING":
        s.physics_step(d)


def test_contract_refusals_change_nothing(scene, cfg, data):
    s = _session(scene, cfg, data)
    v = Voice(s, data)
    r = v("undo_last_edit")
    assert not r["accepted"] and r["reason_code"] == "NOTHING_TO_UNDO" and r["rev"] == r["rev_before"]
    r = v("stop")
    assert not r["accepted"] and r["reason_code"] == "NOTHING_TO_STOP" and s.state == "GENERATED"
    r = v("start_edit")
    assert r["accepted"] and s.edit_mode and r["rev"] == r["rev_before"] + 1
    stale = s.rev
    with s.acting("key"):
        s.handle_key("S", data)                                # a button moves the state on
    r = v("use_current_path", rev=stale)                       # answer decided before the button
    assert not r["accepted"] and r["reason_code"] == "STALE_REVISION" and s.state == "SELECTED"
    assert v("return_home")["reason_code"] == "NOT_ALLOWED"
    assert v("new_target", rev=None)["accepted"] is True       # N's generator (band +25..+35 first)
    assert s.target_label == "New #1" and 25 <= np.degrees(s.target.theta_rad) <= 35


def test_duplicate_call_and_drag_in_progress(scene, cfg, data):
    s = _session(scene, cfg, data)
    v = Voice(s, data)
    r1 = v("start_edit", rid="same")
    r2 = v("start_edit", rid="same", rev=s.rev)
    assert r1["accepted"] and r2["duplicate"] and r2["rev"] == r1["rev"]
    k = s.path.waypoint_index[1] // 2
    assert s.edit_begin(k, s.path.points_m[k])
    r = v("use_current_path")
    assert not r["accepted"] and r["reason_code"] == "DRAG_ACTIVE" and s.state == "GENERATED"
    assert v("stop")["detail"] == "drag" and not s.edit.dragging       # X priority: the drag first


def test_voice_path_choice_validates_then_runs_the_edited_path(scene, cfg, data):
    s = _session(scene, cfg, data)
    v = Voice(s, data)
    _drag(s, 0.04)
    _drag(s, 0.03, k=s.path.waypoint_index[1] // 3)
    assert v("undo_last_edit")["accepted"] and s.path.meta["edits"] == 1      # exactly one edit undone
    edited = s.path.fingerprint
    r = v("use_current_path")
    assert r["accepted"] and s.state == "PASSED" and s.auto_run is not None      # sync validation
    s.watch(data)                                              # the main loop's per-frame call
    assert s.state == "EXECUTING"
    started = [e for e in s.events if e.kind == "execution_start"][-1]
    assert started.actor == "voice" and started.request_id == r["request_id"]
    _run_to_end(s, data)
    assert s.state == "DONE" and s.last_run["path_fp"] == edited and s.last_run["path_source"] == "edited"


def test_button_v_never_auto_runs(scene, cfg, data):
    s = _session(scene, cfg, data)
    s.handle_key("S", data)
    s.handle_key("V", data)
    assert s.state == "PASSED"
    for _ in range(5):
        s.watch(data)
    assert s.state == "PASSED" and s.auto_run is None            # Y is still required


def test_failed_validation_does_not_move_and_returns_to_editing(scene, cfg, data):
    s = _session(scene, cfg, data)
    v = Voice(s, data)
    _drag(s, -0.15, k=s.path.waypoint_index[1] - 3)             # pushed below the floor
    r = v("use_current_path")
    assert r["accepted"] and s.state == "FAILED" and s.auto_run is None and s.edit_mode
    s.watch(data)
    assert s.state == "FAILED"
    assert v("use_current_path")["reason_code"] == "PATH_FAILED"
    assert v("undo_last_edit")["accepted"] and s.state == "GENERATED"


def test_binding_recheck_still_applies_to_auto_run(scene, cfg, data):
    s = _session(scene, cfg, data)
    v = Voice(s, data)
    assert v("use_current_path")["accepted"] and s.state == "PASSED"
    m, idx, _ = scene
    data.qpos[idx.qpos[0]] += 0.05                              # robot moved after validation
    s.watch(data)
    assert s.state == "GENERATED" and s.executor is None        # regenerated, nothing executed
    assert any(e.kind == "auto_run_cancelled" for e in s.events) or s.auto_run is None


def _wait(s, timeout=30.0):
    t0 = time.time()
    while s._future is not None and time.time() - t0 < timeout:
        s._future.result()
        s.poll()


def test_threaded_x_cancels_booking_n_and_edit_are_refused(scene, cfg, data):
    s = _session(scene, cfg, data, threaded=True)
    v = Voice(s, data)
    assert v("use_current_path")["accepted"] and s.state == "VALIDATING" and s.auto_run is not None
    # existing rules: N and a new mouse edit are refused while validating (booking untouched)
    assert not s.cmd_new_target(data) and s.state == "VALIDATING" and s.auto_run is not None
    assert not s.edit_begin(10, s.path.points_m[10]) and s.auto_run is not None
    assert v("new_target")["reason_code"] == "BUSY"
    # X cancels the validation and the booking; the late result is discarded
    assert v("stop")["detail"] == "validation" and s.state == "SELECTED" and s.auto_run is None
    _wait(s)
    s.watch(data)
    assert s.state == "SELECTED" and s.executor is None
    s.close()


def test_threaded_result_after_refused_n_still_runs_booked_path(scene, cfg, data):
    s = _session(scene, cfg, data, threaded=True)
    v = Voice(s, data)
    fp = s.path.fingerprint
    v("use_current_path")
    s.handle_key("N", data)                                     # refused in VALIDATING (unchanged rule)
    _wait(s)
    s.watch(data)
    assert s.state == "EXECUTING" and s.path.fingerprint == fp
    s.cmd_cancel(data)
    assert s.state == "ABORTED"                                 # X still stops a voice-started run
    s.close()


def test_lost_voice_link_cancels_booking_but_y_still_works(scene, cfg, data, tmp_path):
    s = _session(scene, cfg, data, threaded=True)
    gw = VoiceGateway(str(tmp_path / "voice.sock"))
    gw.connected = True                                         # as if a client had connected
    gw._session = s
    Voice(s, data)("use_current_path")
    gw._drop(s, "voice link lost")
    assert s.auto_run is None
    _wait(s)
    s.watch(data)
    assert s.state == "PASSED"                                  # not executed ...
    assert s.cmd_approve(data) and s.state == "EXECUTING"       # ... but the button path is unchanged
    s.close()


def test_socket_roundtrip_runs_in_drain(scene, cfg, data):
    import os
    import shutil
    import tempfile
    from pathlib import Path
    short = Path(tempfile.mkdtemp(prefix="pv", dir=os.environ.get("XDG_RUNTIME_DIR") or "/tmp"))  # AF_UNIX length
    tmp_path = short
    s = _session(scene, cfg, data)
    gw = VoiceGateway(str(tmp_path / "pinch" / "voice.sock")).start(s)
    got = []

    def client():
        conn = connect(gw.address)
        import json
        got.append(json.loads(conn.recv_bytes()))                # hello
        conn.send_bytes(json.dumps({"v": 1, "type": "command", "command": "start_edit", "request_id": "a",
                                    "expected_rev": got[0]["status"]["rev"]}).encode())
        while len(got) < 3:                                      # result + forwarded edit_mode event
            got.append(json.loads(conn.recv_bytes()))
        conn.close()

    t = threading.Thread(target=client)
    t.start()
    t0 = time.time()
    while t.is_alive() and time.time() - t0 < 10:
        gw.drain(s, data)                                        # the app calls this from its main loop
        time.sleep(0.01)
    t.join(1)
    gw.close()
    kinds = [g["type"] for g in got]
    assert kinds[0] == "hello" and "result" in kinds and "event" in kinds
    res = next(g for g in got if g["type"] == "result")
    assert res["accepted"] and res["status"]["edit_mode"] is True
    assert (tmp_path / "pinch").stat().st_mode & 0o777 == 0o700
    shutil.rmtree(short, ignore_errors=True)
    with pytest.raises(ValueError):
        VoiceGateway("/" + "x" * 120 + "/voice.sock").start(s)


def test_request_past_its_deadline_is_never_executed(scene, cfg, data):
    """The voice side gives up after its timeout and tells the user 'not done'; a late delivery must not run."""
    s = _session(scene, cfg, data)
    r = Dispatcher().handle(s, data, {"v": 1, "type": "command", "command": "new_target", "request_id": "late",
                                      "expected_rev": s.rev, "deadline_unix": time.time() - 0.1})
    assert not r["accepted"] and r["reason_code"] == "REQUEST_EXPIRED"
    assert s.state == "GENERATED" and s.target_label == "fixed" and r["rev"] == r["rev_before"]
