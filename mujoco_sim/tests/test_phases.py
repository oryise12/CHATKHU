"""Two phases (outbound, then return) and resume after a stop, through the voice gateway.

Outbound: the generated start -> target path (editable) is validated and run. At the target the
return is the executed (edited) outbound path backwards - run as is (return_reverse) or edited first
(edit_return). A stop (ABORTED) can resume forward (rest of the interrupted path) or back (to where it started).
"""
import numpy as np

from pinch_sim.session import Session
from pinch_sim.voice_gateway import Dispatcher

from test_voice_gateway import RM, _drag, _run_to_end


def _setup(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    return s


def _cmd(s, d, cmd, rid, **kw):
    return Dispatcher().handle(s, d, {"v": 1, "type": "command", "command": cmd, "request_id": rid,
                                      "expected_rev": s.rev, **kw})


def _run_for(s, d, seconds):
    t0 = d.time
    while s.state == "EXECUTING" and d.time - t0 < seconds:
        s.physics_step(d)


def _settle(s, d, steps=300):
    for _ in range(steps):
        s.physics_step(d)


def test_return_is_refused_before_arriving(scene, cfg, data):
    s = _setup(scene, cfg, data)
    for cmd in ("return_reverse", "edit_return"):
        r = _cmd(s, data, cmd, cmd)
        assert not r["accepted"] and s.phase == "outbound" and s.state == "GENERATED"
    r = _cmd(s, data, "resume", "res", direction="forward")
    assert not r["accepted"] and r["reason_code"] == "NOT_STOPPED"


def test_outbound_then_reverse_return_ends_at_start(scene, cfg, data):
    s = _setup(scene, cfg, data)
    start = s.path.points_m[0].copy()
    _drag(s, 0.03)
    edited = s.path.points_m.copy()
    assert _cmd(s, data, "use_current_path", "go")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    assert s.state == "DONE" and s.status()["last_exec"]["phase"] == "outbound"
    r = _cmd(s, data, "return_reverse", "back")
    assert r["accepted"] and s.phase == "return" and s.path.path_id == "return_reverse"
    assert np.allclose(s.path.points_m[1:], edited[::-1][1:])      # the edited outbound path, backwards
    s.watch(data)
    assert s.state == "EXECUTING"
    _run_to_end(s, data)
    assert s.state == "DONE" and np.linalg.norm(s.robot_state(data).tcp.p_m - start) < 1e-3


def test_edit_return_then_done_runs_the_edited_return(scene, cfg, data):
    s = _setup(scene, cfg, data)
    assert _cmd(s, data, "use_current_path", "go")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    r = _cmd(s, data, "edit_return", "edit")
    assert r["accepted"] and s.state == "GENERATED" and s.phase == "return" and s.edit_mode
    _drag(s, 0.02)
    fp = s.path.fingerprint
    assert _cmd(s, data, "use_current_path", "done")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    assert s.state == "DONE" and s.last_run["path_fp"] == fp and s.last_run["path_source"] == "edited"


def test_stop_then_resume_forward_and_back(scene, cfg, data):
    s = _setup(scene, cfg, data)
    start, goal = s.path.points_m[0].copy(), s.goal.p_m.copy()
    assert _cmd(s, data, "use_current_path", "go")["accepted"]
    s.watch(data)
    _run_for(s, data, 3.0)
    r = _cmd(s, data, "stop", "stop1")
    assert r["accepted"] and r["detail"] == "execution" and s.state == "ABORTED"
    _settle(s, data)
    r = _cmd(s, data, "resume", "fwd", direction="forward")
    assert r["accepted"] and s.phase == "outbound"
    s.watch(data)
    _run_to_end(s, data)
    assert s.state == "DONE" and np.linalg.norm(s.robot_state(data).tcp.p_m - goal) < 1e-3
    # on the way back, stop and go back to the target again: the phase flips to outbound
    assert _cmd(s, data, "return_reverse", "back")["accepted"]
    s.watch(data)
    _run_for(s, data, 3.0)
    assert _cmd(s, data, "stop", "stop2")["accepted"]
    _settle(s, data)
    assert _cmd(s, data, "resume", "bk", direction="back")["accepted"] and s.phase == "outbound"
    s.watch(data)
    _run_to_end(s, data)
    assert s.state == "DONE" and np.linalg.norm(s.robot_state(data).tcp.p_m - goal) < 1e-3
    # the return is still the whole outbound path reversed (not the short piece just run)
    assert _cmd(s, data, "return_reverse", "back2")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    assert s.state == "DONE" and np.linalg.norm(s.robot_state(data).tcp.p_m - start) < 1e-3


def _edit(s, dz):
    k = len(s.path.points_m) // 2
    p0 = s.path.points_m[k].copy()
    assert s.edit_begin(k, p0)
    s.edit_update(p0 + [0.0, 0.0, dz])
    assert s.edit_end()


def test_stop_twice_then_back_returns_to_outbound_editing(scene, cfg, data):
    s = _setup(scene, cfg, data)
    _edit(s, 0.03)
    edited = s.path.fingerprint
    assert _cmd(s, data, "use_current_path", "go")["accepted"]
    s.watch(data)
    _run_for(s, data, 2.0)
    assert _cmd(s, data, "stop", "s1")["accepted"]
    _settle(s, data)
    assert _cmd(s, data, "resume", "f1", direction="forward")["accepted"]
    s.watch(data)
    _run_for(s, data, 1.5)
    assert _cmd(s, data, "stop", "s2")["accepted"]
    _settle(s, data)
    assert _cmd(s, data, "resume", "b1", direction="back")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    # back at the very start of the outbound move (not at the first stop), editing it again with its history
    ends = [e for e in s.events if e.kind == "execution_end"]
    assert ends[-1].state == "DONE" and ends[-1].data["next"] == "edit_outbound"
    assert s.events[-1].kind == "back_to_edit" and s.state == "GENERATED" and s.phase == "outbound"
    assert s.path.fingerprint == edited and s.edit.editor.can_undo
    assert np.linalg.norm(s.robot_state(data).tcp.p_m - s.path.points_m[0]) < 1e-3
    assert _cmd(s, data, "use_current_path", "again")["accepted"]       # and it can be run again
    s.watch(data)
    assert s.state == "EXECUTING"


def test_edited_return_stop_back_returns_to_return_editing_and_save(scene, cfg, data, tmp_path, monkeypatch):
    import pinch_sim.voice_gateway as vg
    monkeypatch.setattr(vg, "SAVE_DIR", tmp_path)
    s = _setup(scene, cfg, data)
    r = _cmd(s, data, "save_paths", "early")
    assert not r["accepted"] and r["reason_code"] == "NOTHING_TO_SAVE"
    assert _cmd(s, data, "use_current_path", "go")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    assert _cmd(s, data, "edit_return", "edit")["accepted"]
    _edit(s, 0.02)
    edited = s.path.fingerprint
    assert _cmd(s, data, "use_current_path", "done")["accepted"]
    s.watch(data)
    _run_for(s, data, 2.5)
    assert _cmd(s, data, "stop", "s")["accepted"]
    _settle(s, data)
    assert _cmd(s, data, "resume", "b", direction="back")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    assert s.events[-1].kind == "back_to_edit" and s.events[-1].data["which"] == "return"
    assert s.phase == "return" and s.path.fingerprint == edited and s.edit_mode
    assert _cmd(s, data, "use_current_path", "done2")["accepted"]
    s.watch(data)
    _run_to_end(s, data)
    assert s.state == "DONE" and s.status()["last_exec"]["phase"] == "return"
    r = _cmd(s, data, "save_paths", "save")
    assert r["accepted"]
    import json
    saved = json.loads(open(r["detail"]["file"]).read())
    assert saved["outbound"]["path_id"] == "to_target" and saved["return"]["edits"] == 1


def test_finish_edit_validates_only_and_waits_for_approval(scene, cfg, data):
    """수정 완료 = validate only (auto_run false); it moves only after the approval (auto_run true)."""
    s = _setup(scene, cfg, data)
    _drag(s, 0.02)
    r = _cmd(s, data, "use_current_path", "done", auto_run=False)
    assert r["accepted"] and s.auto_run is None
    s.watch(data)
    assert s.state == "PASSED"
    for _ in range(5):                                          # nothing booked: it keeps waiting
        s.watch(data)
    assert s.state == "PASSED"
    r = _cmd(s, data, "use_current_path", "done-again", auto_run=False)
    assert r["accepted"] and s.state == "PASSED"                # 수정 완료 twice is not an approval
    r = _cmd(s, data, "use_current_path", "approve")
    assert r["accepted"] and s.state == "EXECUTING"
    _run_to_end(s, data)
    assert s.state == "DONE" and s.last_run["path_source"] == "edited"


def test_more_editing_after_passed_needs_a_new_validation(scene, cfg, data):
    s = _setup(scene, cfg, data)
    assert _cmd(s, data, "use_current_path", "done", auto_run=False)["accepted"]
    s.watch(data)
    assert s.state == "PASSED"
    _drag(s, 0.02)                                              # 더 고칠래 -> edit again
    assert s.state == "GENERATED" and s.report is None
