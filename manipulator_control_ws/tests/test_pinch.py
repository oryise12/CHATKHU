"""pinch: trajectories from the simulator, checked again here and streamed on fake_bus (helpers.System, no threads).

Covers: the checker (accepts a fast valid trajectory, refuses each fault), arming only from the operator's
terminal, streaming to the end, every stop source (terminal, simulator, lost link), comm loss, a stuck joint, a lost
streamed write (calibration kept), a reboot (calibration deleted), check-only, speed scale, the socket link.
"""
import io
import math
import os
import tempfile
import time
from multiprocessing.connection import Client

import numpy as np
import pytest

from helpers import FAKE_SCENE_SHA, System, fake_cfg
from manipulator_control.link import ArmLink, hello_problems
from manipulator_control.pinch_traj import Acceptance, check, parse
from manipulator_control.planner import braking_target
from manipulator_control.supervisor import S, SyncWrite, View
from manipulator_control.terminal import TerminalInput
from manipulator_control.units import pa_counts_s2, pv_counts_s

R = np.radians


def min_jerk(q0, q1, T, dt=0.01):
    t = np.arange(0.0, T + 1e-9, dt)
    s = t / T
    pos = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    vel = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T
    d = np.asarray(q1, float) - np.asarray(q0, float)
    return np.asarray(q0, float) + np.outer(pos, d), np.outer(vel, d)


def msg(q, qd, dt=0.01, traj_id="t1", **meta):
    cfg = fake_cfg()
    return {"type": "execute", "traj_id": traj_id, "dt": dt, "q": np.asarray(q).tolist(), "qd": np.asarray(qd).tolist(),
            "meta": dict({"robot_sha": cfg.robot_sha, "model_sha": FAKE_SCENE_SHA, "move": "outbound"}, **meta)}


def traj(q0, dq_deg, peak_deg_s=21.0, dt=0.01, **meta):
    """Min-jerk move by dq_deg whose fastest joint peaks at peak_deg_s."""
    dq = R(dq_deg)
    T = 1.875 * np.abs(dq).max() / R(peak_deg_s)
    q, qd = min_jerk(q0, np.asarray(q0) + dq, T, dt)
    return parse(msg(q, qd, dt, **meta), 10_000)


def acceptance():
    return Acceptance.from_config(fake_cfg(), FAKE_SCENE_SHA)


def q_meas(s):
    return s.sup.cal.q(s.sup.fast)


def armed(**pinch_kw):
    s = System(purpose="pinch", cfg=fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5}), pinch_kw=pinch_kw)
    s.confirm_preview()
    assert s.until_state(S.LINK_WAIT, timeout_s=40), (s.state, s.sup.fault)
    s.link.put("LINK_UP", {"git": "test"})
    assert s.until_state(S.ARM_CONFIRM, timeout_s=2)
    s.cycle(1)
    s.press("CONFIRM")
    assert s.until_state(S.ARMED, timeout_s=2), (s.state, s.sup.fault)
    return s


def stream_goals(s):
    out = []
    for e in s.events("command_sent"):
        if e["label"].startswith("pinch_stream"):
            out.append(e["values"])
    return out


def run_move(s, tr, until=(S.ARMED,), timeout_s=None, on_cycle=None):
    s.link.put("TRAJ", tr)
    s.cycle(1)
    assert s.state in (S.TRAJ_CHECK, S.TRAJ_MOVING), (s.state, s.link.sent[-1:])
    t_end = s.clock.t + (timeout_s or tr.duration + 6)
    while s.clock.t < t_end and s.state not in until:
        if on_cycle:
            on_cycle(s)
        s.cycle(1)
    return s.state in until


# ---------------------------------------------------------------- checker
def test_checker_accepts_a_fast_valid_trajectory():
    s_cfg = fake_cfg()
    home = np.array(s_cfg.home_q_rad)
    tr = traj(home, [20, 10, 0, -15, 0, 0], peak_deg_s=21.2)
    assert check(tr, acceptance(), home) == []


CASES = {          # case -> the reason the checker must give
    "limit": "outside", "brake": "braking point", "speed": "speed", "accel": "acceleration", "start": "start differs",
    "sha": "robot.yaml differs", "long": "duration", "dt": "sample spacing", "rest": "at rest", "model": "model check",
    "qd_mismatch": "disagree",
}


@pytest.mark.parametrize("case", sorted(CASES))
def test_checker_refuses(case):
    import dataclasses
    cfg = fake_cfg()
    home = np.array(cfg.home_q_rad)
    acc = acceptance()
    checker = None
    if case == "limit":                                   # J4 limit -100: the end -98.6 is inside the margin
        tr = traj(home, [0, 0, 0, -47, 0, 0])
    elif case == "brake":                                 # fast close to the limit; with a weak stream PA it overshoots
        acc = dataclasses.replace(acc, margin_rad=0.0, a_stream_rad_s2=np.full(6, R(5.0)))
        tr = traj(home, [0, 0, 0, -47, 0, 0])
    elif case == "speed":
        tr = traj(home, [20, 0, 0, 0, 0, 0], peak_deg_s=22.6)
    elif case == "accel":
        tr = traj(home, [3, 0, 0, 0, 0, 0], peak_deg_s=21)  # short and fast: high acceleration
    elif case == "start":
        tr = traj(home + R([0.5, 0, 0, 0, 0, 0]), [10, 0, 0, 0, 0, 0], peak_deg_s=10)
    elif case == "sha":
        tr = traj(home, [10, 0, 0, 0, 0, 0], peak_deg_s=10, robot_sha="0" * 64)
    elif case == "long":
        q, qd = min_jerk(home, home + R([10, 0, 0, 0, 0, 0]), 31.0)
        tr = parse(msg(q, qd), 10_000)
    elif case == "dt":
        q, qd = min_jerk(home, home + R([10, 0, 0, 0, 0, 0]), 3.0, dt=0.05)
        tr = parse(msg(q, qd, dt=0.05), 10_000)
    elif case == "rest":
        q, qd = min_jerk(home, home + R([10, 0, 0, 0, 0, 0]), 3.0)
        q, qd = q[: len(q) // 2], qd[: len(q) // 2]
        tr = parse(msg(q, qd), 10_000)
    elif case == "model":
        tr = traj(home, [10, 0, 0, 0, 0, 0], peak_deg_s=10)
        checker = (lambda qq: ["below mount plane"] if qq[0] > home[0] + R(5) else [])
    else:                                                 # qd_mismatch
        tr = traj(home, [10, 0, 0, 0, 0, 0], peak_deg_s=10)
        tr = parse(msg(tr.q, tr.qd * 1.05), 10_000)
    problems = check(tr, acc, home, checker)
    assert problems and CASES[case] in problems[0], (case, problems)


def test_parse_refuses_bad_shapes_and_values():
    with pytest.raises(ValueError):
        parse({"traj_id": "x", "dt": 0.01, "q": [[0.0] * 5] * 3, "qd": [[0.0] * 5] * 3}, 100)
    with pytest.raises(ValueError):
        parse({"traj_id": "x", "dt": 0.01, "q": [[float("nan")] * 6] * 3, "qd": [[0.0] * 6] * 3}, 100)
    with pytest.raises(ValueError):
        parse({"traj_id": "x", "dt": 0.01, "q": [[0.0] * 6] * 300, "qd": [[0.0] * 6] * 300}, 100)


# ---------------------------------------------------------------- arming
def test_only_the_operator_terminal_arms():
    s = System(purpose="pinch", cfg=fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5}))
    s.confirm_preview()
    assert s.until_state(S.LINK_WAIT, timeout_s=40)
    s.link.put("TRAJ", traj(q_meas(s), [5, 0, 0, 0, 0, 0]))       # not armed: refused at once
    s.cycle(2)
    assert s.link.of("ack")[-1]["accepted"] is False
    s.link.put("LINK_UP", {})
    assert s.until_state(S.ARM_CONFIRM, timeout_s=2)
    s.cycle(1)
    s.link.put("CONFIRM")                                         # the simulator cannot confirm
    s.inputs.put("CONFIRM", source="ui")                          # nor can the panel / viewer
    s.cycle(3)
    assert s.state == S.ARM_CONFIRM
    assert any(e["kind"] == "CONFIRM" for e in s.events("input_discarded"))
    s.press("CONFIRM")
    assert s.until_state(S.ARMED, timeout_s=1)
    assert s.sup.view().armed


def test_arm_dialog_confirms_only_the_arming():
    """2026-10-29 user decision: this program's own arming dialog (panel.py) may arm, and nothing else."""
    s = System(purpose="pinch", cfg=fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5}))
    assert s.until_state(S.HOME_PREVIEW, timeout_s=40)
    s.cycle(1)
    s.inputs.put("CONFIRM", source="arm_dialog")                  # not the HOME preview
    s.cycle(3)
    assert s.state == S.HOME_PREVIEW
    assert any(e["kind"] == "CONFIRM" and e.get("source") == "arm_dialog" for e in s.events("input_discarded"))
    s.confirm_preview()
    assert s.until_state(S.LINK_WAIT, timeout_s=40)
    s.link.put("LINK_UP", {})
    assert s.until_state(S.ARM_CONFIRM, timeout_s=2)
    s.cycle(1)
    s.inputs.put("CONFIRM", source="arm_dialog")                  # the arming dialog's y
    assert s.until_state(S.ARMED, timeout_s=1)
    assert s.sup.view().armed


def test_flush_keeps_stop_events():
    t = TerminalInput(stream=io.StringIO())
    t.put("CONFIRM")
    t.put("STOP")
    t.put("TORQUE_OFF_NOW")
    t.flush()
    assert [e.kind for e in t.drain()] == ["STOP", "TORQUE_OFF_NOW"]


# ---------------------------------------------------------------- streaming
def test_streams_the_trajectory_to_its_end():
    s = armed()
    q0 = q_meas(s)
    tr = traj(q0, [20, 10, 0, -15, 0, 0], peak_deg_s=21.2)
    errs = []

    def watch(s):
        if s.state == S.TRAJ_MOVING and s.sup.pinch is not None and s.sup.pinch["t0"] is not None:
            t = s.sup.fast.t - s.sup.pinch["t0"]
            errs.append(np.degrees(np.abs(q_meas(s) - tr.at(t)[0])).max())
    assert run_move(s, tr, on_cycle=watch), (s.state, s.sup.fault, s.link.sent[-2:])
    ack, res = s.link.of("ack")[-1], s.link.of("result")[-1]
    assert ack["accepted"] and res["status"] == "DONE", res
    assert np.degrees(np.abs(q_meas(s) - tr.q[-1])).max() < 0.2
    assert max(errs) < 1.5, max(errs)                   # fake servo: first-order lag 30 ms behind its profile
    lo, hi = tr.q.min(axis=0) - R(0.01), tr.q.max(axis=0) + R(0.01)
    goals = stream_goals(s)
    assert len(goals) > tr.duration * 15
    for v in goals:
        counts = {int(i): x["goal"] for i, x in v.items()}
        q = s.sup.cal.q_of(counts)
        assert np.all(q >= lo) and np.all(q <= hi)
        assert all(1 <= x["pv"] <= 262 and x["pa"] == 75 for x in v.values())
    assert res["metrics"]["max_tracking_deg"] < 1.5


def test_check_only_moves_nothing():
    s = armed(check_only=True)
    q0 = q_meas(s)
    s.link.put("TRAJ", traj(q0, [20, 0, 0, 0, 0, 0]))
    s.cycle(3)
    assert s.state == S.ARMED
    assert s.link.of("ack")[-1].get("check_only") and s.link.of("result")[-1]["status"] == "CHECKED"
    assert not stream_goals(s)
    assert np.degrees(np.abs(q_meas(s) - q0)).max() < 0.05


def test_speed_scale_slows_the_same_path():
    s = armed(speed_scale=0.5)
    tr = traj(q_meas(s), [15, 0, 0, 0, 0, 0])
    assert run_move(s, tr, timeout_s=2 * tr.duration + 6)
    ack = s.link.of("ack")[-1]
    assert ack["duration_s"] == pytest.approx(2 * tr.duration, rel=1e-6)
    assert s.link.of("result")[-1]["status"] == "DONE"


def test_refused_trajectory_does_not_move():
    s = armed()
    q0 = q_meas(s)
    s.link.put("TRAJ", traj(q0 + R([1, 0, 0, 0, 0, 0]), [10, 0, 0, 0, 0, 0]))   # does not start at the real pose
    s.cycle(3)
    assert s.state == S.ARMED
    ack = s.link.of("ack")[-1]
    assert ack["accepted"] is False and "start" in ack["reasons"][0]
    assert not stream_goals(s)


# ---------------------------------------------------------------- stops
def _stop_midway(source):
    s = armed()
    tr = traj(q_meas(s), [25, 0, 0, -20, 0, 0], peak_deg_s=21)
    hit = {}

    def go(s):
        if s.state == S.TRAJ_MOVING and "t" not in hit and s.sup.pinch["t0"] is not None and \
                s.clock.t - s.sup.pinch["t0"] > 0.5 * tr.duration:
            hit["t"] = s.clock.t - s.sup.pinch["t0"]
            if source == "terminal":
                s.press("STOP")
            elif source == "sim":
                s.link.put("EXT_STOP", "user stop (X)")
            else:
                s.link.put("LINK_DOWN", "gone")
    done = run_move(s, tr, until=(S.HOLDING, S.ARMED, S.LINK_WAIT, S.FAULT), on_cycle=go)
    return s, tr, hit["t"], done


@pytest.mark.parametrize("source,after", [("terminal", S.HOLDING), ("sim", S.ARMED), ("link", S.LINK_WAIT)])
def test_stops_stay_on_the_path(source, after):
    s, tr, t_stop, done = _stop_midway(source)
    assert done and s.state == after, (s.state, s.sup.fault)
    res = s.link.of("result")[-1]
    assert res["status"] == "STOPPED", res
    lead = s.cfg.motion["pinch"]["lead_s"]
    win = tr.window(t_stop - 0.2, t_stop + lead + 0.2)
    q = q_meas(s)
    assert np.all(q >= win.min(axis=0) - R(0.3)) and np.all(q <= win.max(axis=0) + R(0.3)), np.degrees(q - tr.at(t_stop)[0])
    if source == "terminal":                     # disarmed: y re-arms (link still up)
        s.cycle(1)
        s.press("CONFIRM")
        assert s.until_state(S.ARMED, timeout_s=1)


def test_comm_loss_leaves_the_arm_at_most_a_lead_ahead():
    s = armed()
    tr = traj(q_meas(s), [25, 0, 0, -20, 0, 0], peak_deg_s=21)
    state = {}

    def cut(s):
        if s.state == S.TRAJ_MOVING and "t" not in state and s.sup.pinch["t0"] is not None and \
                s.clock.t - s.sup.pinch["t0"] > 0.4 * tr.duration:
            state["t"], state["cal"] = s.clock.t - s.sup.pinch["t0"], s.sup.cal
            s.bus.link_down = True
    run_move(s, tr, until=(S.COMM_LOST,), on_cycle=cut)
    assert s.state == S.COMM_LOST
    assert s.link.of("result")[-1]["status"] == "FAULT"
    s.cycle(60)                                   # the servos go on by themselves (Watchdog 0)
    last = {int(i): x["goal"] for i, x in stream_goals(s)[-1].items()}
    for i, sv in s.bus.servos.items():
        assert abs(sv.pos - last[i]) < 2          # stopped at the last goal, not beyond
    lead = s.cfg.motion["pinch"]["lead_s"]
    win = tr.window(state["t"] - 0.1, state["t"] + lead + 0.1)        # the last goal is at most a lead ahead
    q_last = state["cal"].q_of(last)                 # the calibration as it was before the link went down
    assert np.all(q_last >= win.min(axis=0) - R(0.01)) and np.all(q_last <= win.max(axis=0) + R(0.01))
    assert s.store.load() is None                 # comm loss drops the stored calibration (user rule)


def test_stuck_joint_faults_and_reports():
    s = armed()
    tr = traj(q_meas(s), [25, 0, 0, -20, 0, 0], peak_deg_s=21)
    hit = {}

    def stick(s):
        if s.state == S.TRAJ_MOVING and not hit and s.sup.pinch["t0"] is not None and \
                s.clock.t - s.sup.pinch["t0"] > 0.3 * tr.duration:
            hit["x"] = True
            s.bus.servos[1].stuck = True
    run_move(s, tr, until=(S.FAULT,), on_cycle=stick)
    assert s.state == S.FAULT and s.sup.fault.code in ("TRACKING", "ENVELOPE"), s.sup.fault
    assert s.link.of("result")[-1]["status"] == "FAULT"


def test_a_lost_streamed_write_stops_but_keeps_the_calibration():
    s = armed()
    tr = traj(q_meas(s), [25, 0, 0, -20, 0, 0], peak_deg_s=21)
    real_execute = s.runner.execute
    dropped = {}

    def execute(c):
        if isinstance(c, SyncWrite) and c.label == "pinch_stream" and not dropped and \
                s.clock.t - s.sup.pinch["t0"] > 0.3 * tr.duration:
            dropped["x"] = True                  # the write never reaches the servos
            return
        real_execute(c)
    s.runner.execute = execute
    run_move(s, tr, until=(S.FAULT,))
    assert s.sup.fault.code == "STREAM_WRITE_LOST", s.sup.fault
    assert s.store.load() is not None             # not reset evidence: the calibration stays


def test_a_reboot_mid_move_invalidates():
    s = armed()
    tr = traj(q_meas(s), [25, 0, 0, -20, 0, 0], peak_deg_s=21)
    hit = {}

    def boot(s):
        if s.state == S.TRAJ_MOVING and not hit and s.sup.pinch["t0"] is not None and \
                s.clock.t - s.sup.pinch["t0"] > 0.3 * tr.duration:
            hit["x"] = True
            s.bus.servos[4].reboot()
    run_move(s, tr, until=(S.FAULT,), on_cycle=boot)
    assert s.state == S.FAULT and s.sup.fault.code in ("INVALIDATED", "STOP_UNCONFIRMED"), s.sup.fault
    assert s.store.load() is None


# ---------------------------------------------------------------- planner fix
def test_braking_target_raises_pa_when_the_current_goal_is_close():
    v_raw = 250                                         # ~21.5 deg/s at the joint (16:1)
    v = pv_counts_s(v_raw)
    room = 600                                          # counts to the current goal: less than v^2 / 2a at PA 11
    st = braking_target(0, v_raw, room, 262, 11, latency_s=0.1, pa_max=11, inside=lambda g: True)
    assert st.goal == room
    a = pa_counts_s2(st.pa)
    assert v * v / (2 * a) <= room - v * 0.1 + 1e-6, st
    assert "to stop before the current goal" in st.note


# ---------------------------------------------------------------- link
def _hello(acc):
    return {"kind": "fake", "robot_sha": acc.robot_sha, "model_sha": acc.model_sha,
            "joints": [f"Revolute_{k}" for k in range(1, 7)], "limits_deg": acc.hello()["limits_deg"],
            "home_q_rad": list(fake_cfg().home_q_rad), "speed_scale": 1.0, "check_only": False, "accept": acc.hello()}


def _sim_hello(acc, **over):
    h = {"v": 1, "type": "hello", "role": "pinch_sim", "robot_sha": acc.robot_sha, "model_sha": acc.model_sha,
         "limits_deg": acc.hello()["limits_deg"], "home_q_rad": list(fake_cfg().home_q_rad),
         "validation": {"joint_speed_max_deg_s": 21.16, "qdd_max_deg_s2": 68.75, "joint_margin_deg": 2.865,
                        "dt_s": 0.01}}
    h.update(over)
    return h


def test_hello_problems():
    acc = acceptance()
    mine = _hello(acc)
    assert hello_problems(mine, _sim_hello(acc)) == []
    assert hello_problems(mine, _sim_hello(acc, robot_sha="x"))
    bad = _sim_hello(acc)
    bad["validation"]["joint_margin_deg"] = 1.0
    assert hello_problems(mine, bad)
    bad = _sim_hello(acc, limits_deg=[[-1, 1]] * 6)
    assert hello_problems(mine, bad)


def test_socket_link_round_trip():
    import json
    acc = acceptance()
    addr = os.path.join(tempfile.mkdtemp(prefix="mcl", dir="/tmp"), "arm.sock")
    view = View(state="ARMED", armed=True, q_meas=np.zeros(6))
    link = ArmLink(_hello(acc), lambda: view, ping_timeout_s=1.0, state_hz=50, max_samples=1000, address=addr).start()
    try:
        c = Client(addr, family="AF_UNIX", authkey=open(link.key_path, "rb").read())

        def pump(until, timeout=2.0):
            got, t_end = [], time.monotonic() + timeout
            while time.monotonic() < t_end:
                got += link.drain(time.monotonic())
                if until(got):
                    return got
                time.sleep(0.01)
            return got
        pump(lambda g: link._conn is not None, 1.0)
        first = json.loads(c.recv_bytes())
        assert first["type"] == "hello" and first["robot_sha"] == acc.robot_sha
        c.send_bytes(json.dumps(_sim_hello(acc)).encode())
        got = pump(lambda g: any(e.kind == "LINK_UP" for e in g))
        assert any(e.kind == "LINK_UP" and e.source == "link" for e in got)
        home = np.array(fake_cfg().home_q_rad)
        tr = traj(home, [5, 0, 0, 0, 0, 0])
        c.send_bytes(json.dumps(msg(tr.q, tr.qd)).encode())
        got = pump(lambda g: any(e.kind == "TRAJ" for e in g))
        t = next(e.arg for e in got if e.kind == "TRAJ")
        assert t.n == tr.n and np.allclose(t.q, tr.q)
        c.send_bytes(json.dumps({"type": "execute", "traj_id": "bad", "dt": 0.01, "q": [[0] * 5], "qd": [[0] * 5]}).encode())
        pump(lambda g: False, 0.3)
        msgs, t_end = [], time.monotonic() + 0.5        # the state stream never stops: read for a while
        while time.monotonic() < t_end and c.poll(0.1):
            msgs.append(json.loads(c.recv_bytes()))
        assert any(m["type"] == "ack" and m["traj_id"] == "bad" and not m["accepted"] for m in msgs)
        assert any(m["type"] == "state" and m["state"] == "ARMED" for m in msgs)
        got = pump(lambda g: any(e.kind == "LINK_DOWN" for e in g), 6.5)   # no ping: idle timeout 5 s
        assert any(e.kind == "LINK_DOWN" for e in got)
        c.close()
        c2 = Client(addr, family="AF_UNIX", authkey=open(link.key_path, "rb").read())
        pump(lambda g: link._conn is not None, 1.0)
        json.loads(c2.recv_bytes())
        c2.send_bytes(json.dumps(_sim_hello(acc, model_sha="0" * 64)).encode())
        pump(lambda g: False, 0.3)
        reply = json.loads(c2.recv_bytes())
        assert reply["type"] == "refused" and "model_sha" in reply["reasons"][0]
    finally:
        link.close()
