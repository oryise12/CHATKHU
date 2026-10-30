"""End to end: the arm's real program (manipulator_control pinch --fake: its own state machine, checks and streaming,
on fake servos) and a headless simulator session, over the real socket.

The arm program runs in its own venv as a subprocess with a temporary config: robot.yaml's joints, ranges and HOME
on the fake bus. Its fake hooks press y at the HOME preview and at ARM_CONFIRM (a person does that on the real arm).
Scenario: outbound -> DONE -> reverse return -> DONE; a run stopped with X and resumed; the simulator vanishing
mid-move (the arm stops by itself). Slow (real time, ~1 min).
"""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from pinch_sim.arm_link import ArmLink
from pinch_sim.model import config_sha, load_config, load_scene, ready_q
from pinch_sim.real_arm import ARM_WS, RealArm, apply_profile, hello
from pinch_sim.session import Session

WS = ARM_WS
PY = WS / ".venv" / "bin" / "python"
pytestmark = pytest.mark.skipif(not (PY.exists() and (WS / "config" / "robot.yaml").exists()),
                                reason="no manipulator_control_ws with its venv (setup.sh)")


def e2e_config(tmp: Path) -> Path:
    real = yaml.safe_load((WS / "config" / "robot.yaml").read_text(encoding="utf-8"))
    fake = yaml.safe_load((WS / "config" / "fake_robot.yaml").read_text(encoding="utf-8"))
    doc = dict(fake, joints=real["joints"], home=real["home"], reference=real.get("reference", {}))
    path = tmp / "robot_e2e.yaml"
    path.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


@pytest.fixture(scope="module")
def arm_program():
    tmp = Path(tempfile.mkdtemp(prefix="e2e", dir="/tmp"))
    cfg_path = e2e_config(tmp)
    sock = tmp / "arm.sock"
    env = dict({k: v for k, v in os.environ.items() if k != "PYTHONPATH"}, PYTHONPATH=str(WS / "src"))   # as env.sh
    log = open(tmp / "arm.log", "w")
    proc = subprocess.Popen(
        [str(PY), "-m", "manipulator_control", "pinch", "--fake", "--fake-calibrated", "--config", str(cfg_path),
         "--no-viewer", "--no-panel", "--home-speed-deg-s", "22.5", "--link", str(sock),
         "--fake-event", "confirm_in@0:HOME_PREVIEW", "--fake-event", "confirm_in@0:ARM_CONFIRM"],
        cwd=WS, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
    yield {"proc": proc, "cfg": cfg_path, "sock": sock, "log": tmp / "arm.log"}
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(15)
    except subprocess.TimeoutExpired:
        proc.kill()
    log.close()


def connect(arm_program, cfg, sha, timeout=40.0) -> ArmLink:
    t_end = time.monotonic() + timeout
    last = None
    while time.monotonic() < t_end:
        assert arm_program["proc"].poll() is None, arm_program["log"].read_text()[-3000:]
        try:
            return ArmLink(hello(cfg, sha, {"config_sha": config_sha(cfg)}), arm_program["sock"]).connect()
        except ConnectionError as e:
            last = e
            time.sleep(0.3)
    raise AssertionError(f"no link: {last}\n{arm_program['log'].read_text()[-3000:]}")


class Sim:
    def __init__(self, arm_program):
        self.cfg = apply_profile(load_config(), RealArm.load(arm_program["cfg"]))
        self.m, self.idx, self.sha = load_scene(self.cfg)
        self.arm = connect(arm_program, self.cfg, self.sha)
        assert self.pump(lambda: self.arm.state.get("state") == "ARMED", 40), self.arm.state
        self.d = mujoco.MjData(self.m)
        self.d.qpos[:] = ready_q(self.m)
        self.d.qpos[self.idx.qpos] = self.arm.q_meas()
        mujoco.mj_forward(self.m, self.d)
        self.s = Session(self.m, self.idx, self.sha, self.cfg, self.d, threaded=False)
        self.s.real_arm = self.arm

    def pump(self, until, timeout):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            msgs = self.arm.drain()
            if hasattr(self, "s"):
                self.s.on_arm(self.d, msgs)
                for _ in range(10):
                    self.s.physics_step(self.d)
                self.s.watch(self.d)
            if until():
                return True
            time.sleep(0.02)
        return until()

    def run_selected(self, stop_at=None):
        s, d = self.s, self.d
        assert s.cmd_select() and s.cmd_validate(d) and s.state == "PASSED", (s.state, s.message)
        # this test validates in its own thread (threaded=False) and read nothing meanwhile; the app validates in a
        # worker and reads the link every frame
        assert self.pump(self.arm.fresh, 2)
        assert s.cmd_approve(d), s.message
        T = s.executor.duration
        if stop_at is not None:
            assert self.pump(lambda: s.executor is not None and s.executor.progress(d) >= stop_at, T + 8), s.message
            assert s.cmd_cancel(d) == "execution"
        assert self.pump(lambda: s.state != "EXECUTING", T + 10), (s.state, s.message, self.arm.state)
        return T


def deg(x):
    return np.degrees(np.abs(x)).max()


def test_pinch_scenario_on_the_arm_program(arm_program):
    sim = Sim(arm_program)
    s, d = sim.s, sim.d
    home = np.asarray(sim.cfg["real_arm"]["home_q_rad"])
    assert deg(sim.arm.q_meas() - home) < 0.3                       # the arm program went to HOME first

    # outbound to a target on the arm's side, then the same path backwards
    assert s.cmd_fixed_target(0.275, 20.0, d)
    sim.run_selected()
    assert s.state == "DONE", s.message
    end = s.last_exec["path"]
    assert np.linalg.norm(s.robot_state(d).tcp.p_m - end.points_m[-1]) < 0.002
    ok, why = s.cmd_return_reverse(d)
    assert ok, why
    sim.run_selected()
    assert s.state == "DONE", s.message
    assert deg(sim.arm.q_meas() - home) < 0.3

    # X at half way: the arm stops on the path and stays armed; resume forward finishes the move
    assert s.cmd_new_target(d)
    sim.run_selected(stop_at=0.5)
    assert s.state == "ABORTED" and "STOPPED" in s.message, s.message
    assert sim.pump(lambda: sim.arm.state.get("state") == "ARMED", 5), sim.arm.state
    ok, why = s.cmd_resume(d, forward=True)
    assert ok, why
    sim.run_selected()
    assert s.state == "DONE", s.message

    # the simulator vanishes mid-move: the arm program stops the arm by itself (lost link) and waits
    ok, why = s.cmd_return_reverse(d)
    assert ok, why
    assert s.cmd_select() and s.cmd_validate(d) and sim.pump(sim.arm.fresh, 2) and s.cmd_approve(d)
    T = s.executor.duration
    assert sim.pump(lambda: s.executor is not None and s.executor.progress(d) >= 0.3, T + 8)
    sim.arm.close()                                                  # no stop message: just gone
    time.sleep(3.0)
    again = connect(arm_program, sim.cfg, sim.sha)
    try:
        t_end = time.monotonic() + 5
        seen = []
        while time.monotonic() < t_end and len(seen) < 10:
            again.drain()
            seen.append((again.state.get("state"), again.q_meas()))
            time.sleep(0.1)
        states = {st for st, _ in seen}
        assert states <= {"LINK_WAIT", "ARM_CONFIRM", "ARMED"}, states
        assert deg(seen[-1][1] - seen[0][1]) < 0.01                  # standing still
        assert deg(seen[-1][1] - home) > 1.0                         # stopped part way, not at the end
    finally:
        again.close()
    log = arm_program["log"].read_text()
    assert "Traceback" not in log, log[-3000:]
