"""--real-arm: the real arm's profile (real_arm.py) and the run on the arm through its link (arm_link.py).

The arm program is replaced by FakeArm (no socket, no servos): it records what the simulator sends and plays the
arm's answers. The full round trip with the real program (fake servos) is test_arm_link_e2e.py.
"""
import copy

import mujoco
import numpy as np
import pytest

from pinch_sim.arm_link import ArmLink
from pinch_sim.model import ROOT, config_sha, file_sha, load_config, load_scene, ready_q
from pinch_sim.real_arm import DEFAULT_ROBOT_YAML, RealArm, apply_profile, hello
from pinch_sim.session import Session
from pinch_sim.targets import make_target, precheck, sample_target
from pinch_sim.ui import notice_text
from pinch_sim.voice_gateway import Dispatcher
from pinch_sim.webui import menu_state

pytestmark = pytest.mark.skipif(not DEFAULT_ROBOT_YAML.exists(), reason="no manipulator_control_ws robot.yaml")


@pytest.fixture(scope="module")
def real():
    arm = RealArm.load()
    cfg = apply_profile(load_config(), arm)
    m, idx, sha = load_scene(cfg)
    return arm, cfg, m, idx, sha


class FakeArm(ArmLink):
    """ArmLink without a socket: state set by the test, sends recorded."""

    def __init__(self, cfg, sha, q, state="ARMED"):
        super().__init__(hello(cfg, sha), "/nonexistent")
        self.t = 0.0
        self.clock = lambda: self.t
        self.up = True
        self.sent = []
        self.set(state, q)

    def set(self, state, q, execution=None, armed=None):
        self.state = {"type": "state", "state": state, "armed": state == "ARMED" if armed is None else armed,
                      "stale": False, "q_meas": np.asarray(q, float).tolist(), "execution": execution}
        self.t_state = self.t

    def send(self, msg):
        self.sent.append(msg)

    def drain(self):
        return []

    def of(self, typ):
        return [m for m in self.sent if m.get("type") == typ]


def session_on(real, state="ARMED"):
    arm, cfg, m, idx, sha = real
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    fake = FakeArm(cfg, sha, d.qpos[idx.qpos].copy(), state)
    s = Session(m, idx, sha, cfg, d, threaded=False)
    s.real_arm = fake
    return s, d, fake


def to_passed(s, d, r=0.275, deg=20.0):
    assert s.cmd_fixed_target(r, deg, d)
    assert s.cmd_select()
    assert s.cmd_validate(d)
    assert s.state == "PASSED", (s.state, s.message)


# ---------------------------------------------------------------- profile
def test_default_demo_is_unchanged():
    cfg = load_config()
    before = config_sha(cfg)
    m, idx, sha = load_scene(cfg)
    assert np.allclose(idx.lo, -np.pi) and np.allclose(idx.hi, np.pi)
    assert np.allclose(ready_q(m)[idx.qpos], cfg["ready"]["q_rad"])
    assert sha == file_sha(ROOT / cfg["model"]["generated_scene"])
    if DEFAULT_ROBOT_YAML.exists():
        apply_profile(cfg, RealArm.load())
    assert config_sha(cfg) == before                      # apply_profile never changes its input


def test_profile_uses_the_arm_file(real):
    arm, cfg, m, idx, sha = real
    assert np.allclose(idx.lo, arm.limits_rad[:, 0]) and np.allclose(idx.hi, arm.limits_rad[:, 1])
    assert np.allclose(ready_q(m)[idx.qpos], arm.home_q_rad)
    assert m.geom_pos[m.geom("desk_top").id][1] < 0               # the desk is on the arm's side (-Y)
    assert sha == file_sha(ROOT / cfg["model"]["generated_scene"])  # the file the arm program's copy must match
    h = hello(cfg, sha)
    assert h["robot_sha"] == arm.robot_sha and h["limits_deg"] == np.degrees(arm.limits_rad).round(6).tolist()
    assert h["validation"]["joint_speed_max_deg_s"] <= 22.5 and h["validation"]["qdd_max_deg_s2"] <= 70.0
    assert h["validation"]["joint_margin_deg"] >= 2.5 and h["validation"]["dt_s"] == 0.005


def test_targets_are_on_minus_y_and_inside_the_real_ranges(real, tmp_path):
    arm, cfg, m, idx, sha = real
    from pinch_sim.collision import CollisionChecker
    from pinch_sim.kinematics import Kinematics
    kin, col = Kinematics(m, idx, cfg["ik"]), CollisionChecker(m, idx)
    home = ready_q(m)[idx.qpos]
    t = make_target(0.275, np.radians(20.0), cfg["target"])
    assert t.floor_point_m[1] < -0.2
    for trial in range(4):
        res = sample_target(3, trial, cfg, kin, col, home)
        assert res.target is not None and res.target.floor_point_m[1] < 0, res.message
    # the arm's ranges narrow the simulator's (load_scene): an arm with J1 +-45 refuses a target that needs J1 ~ -49
    # (a copy of robot.yaml with J1 narrowed; the real file is whatever the user set, 2026-10-28 all +-180)
    narrow = tmp_path / "robot.yaml"
    text = DEFAULT_ROBOT_YAML.read_text()
    j1 = next(line for line in text.splitlines() if "name: Revolute_1" in line)
    narrow.write_text(text.replace(j1, j1[:j1.index("limit_deg")] + "limit_deg: [-45, 45]}"))
    cfg45 = apply_profile(load_config(), RealArm.load(narrow))
    m45, idx45, _ = load_scene(cfg45)
    assert np.isclose(np.degrees(idx45.hi[0]), 45.0) and np.isclose(np.degrees(idx45.lo[0]), -45.0)
    kin45, col45 = Kinematics(m45, idx45, cfg45["ik"]), CollisionChecker(m45, idx45)
    beyond = make_target(0.275, np.radians(-30.0), cfg45["target"], allow_out_of_range=True)   # J1 would be ~ -49
    ok, why = precheck(beyond, kin45, col45, ready_q(m45)[idx45.qpos], cfg45)
    assert not ok and why.startswith(("joint_limit", "ik_not_converged")), why


def test_real_profile_path_validates_within_the_arm_limits(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    tt, qq, qd, qdd = s.report.trajectory.dense(0.005)
    arm = real[0]
    assert np.all(qq >= arm.limits_rad[:, 0]) and np.all(qq <= arm.limits_rad[:, 1])
    assert np.degrees(np.abs(qd).max()) <= 22.5 * 0.95 + 1e-6 and np.degrees(np.abs(qdd).max()) <= 68.76
    assert "real arm" in next(c.message for c in s.report.checks if c.kind == "joint_limits")


# ---------------------------------------------------------------- run on the (fake) arm
def _accept_and_play(s, d, fake, final="DONE", frac=1.0):
    ex = s.executor
    execute = fake.of("execute")[-1]
    assert execute["dt"] == 0.005 and len(execute["q"]) == len(execute["qd"]) > 100
    assert execute["meta"]["robot_sha"] == real_sha(s) and execute["meta"]["model_sha"] == s.ctx.model_sha
    s.on_arm(d, [{"type": "ack", "traj_id": ex.traj_id, "accepted": True, "duration_s": ex.duration}])
    tr = s.report.trajectory if s.report is not None else ex.traj
    for t in np.linspace(0, ex.duration * frac, 25):              # the arm moves; the simulator shows it
        q, _, _ = tr.sample(t)
        fake.t += ex.duration / 25
        fake.set("TRAJ_MOVING", q, {"traj_id": ex.traj_id, "t_s": float(t), "T_s": ex.duration}, armed=True)
        s.on_arm(d, [])
        assert s.state == "EXECUTING"
        assert np.allclose(d.qpos[s.idx.qpos], q)                 # twin
    q_end = tr.sample(ex.duration * frac)[0]
    fake.set("ARMED", q_end)
    s.on_arm(d, [{"type": "result", "traj_id": ex.traj_id, "status": final, "reason": "", "q_final": q_end.tolist()}])
    return q_end


def real_sha(s):
    return s.cfg["real_arm"]["robot_sha"]


def test_approval_runs_on_the_arm_and_the_twin_follows(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    assert s.cmd_approve(d) and s.state == "EXECUTING"
    q_end = _accept_and_play(s, d, fake)
    assert s.state == "DONE", s.message
    assert np.allclose(s.hold_q, q_end) and s.last_run["status"] == "DONE"
    assert s.last_run["max_q_err_rad"] < 1e-9


def test_stop_waits_for_the_arm(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    s.cmd_approve(d)
    ex = s.executor
    s.on_arm(d, [{"type": "ack", "traj_id": ex.traj_id, "accepted": True}])
    assert s.cmd_cancel(d) == "execution"
    assert fake.of("stop") and s.state == "EXECUTING"           # not finished until the arm confirms
    q = d.qpos[s.idx.qpos].copy()
    s.on_arm(d, [{"type": "result", "traj_id": ex.traj_id, "status": "STOPPED", "reason": "시뮬 정지", "q_final": q.tolist()}])
    assert s.state == "ABORTED" and "STOPPED" in s.message


def test_stop_without_confirmation_times_out(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    s.cmd_approve(d)
    s.on_arm(d, [{"type": "ack", "traj_id": s.executor.traj_id, "accepted": True}])
    s.cmd_cancel(d)
    fake.t += 3.5
    fake.set("TRAJ_MOVING", d.qpos[s.idx.qpos], armed=True)
    s.physics_step(d)
    assert s.state == "ABORTED" and "정지 확인" in s.message


def test_refusal_regenerates_the_path(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    s.cmd_approve(d)
    s.on_arm(d, [{"type": "ack", "traj_id": s.executor.traj_id, "accepted": False, "reasons": ["start differs"]}])
    assert s.state == "GENERATED" and "거부" in s.message


def test_lost_link_cancels_a_booked_auto_run(real):
    s, d, fake = session_on(real)
    assert s.cmd_fixed_target(0.275, 20.0, d) and s.cmd_select()
    assert s.cmd_validate(d, auto_run_request="voice-1")          # "수정 완료": run it if it passes
    assert s.state == "PASSED" and s.auto_run is not None
    fake.up, fake.down_reason = False, "gone"
    s.on_arm(d, [{"type": "_down", "reason": "gone"}])
    assert s.auto_run is None and s.state == "SELECTED"
    s.watch(d)
    assert s.state != "EXECUTING" and not fake.of("execute")


def test_lost_link_aborts_and_blocks(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    s.cmd_approve(d)
    fake.up, fake.down_reason = False, "gone"
    s.on_arm(d, [{"type": "_down", "reason": "gone"}])
    assert s.state == "ABORTED" and "실물 상태 모름" in s.message
    s.cmd_new_target(d)
    s.cmd_select()
    assert not s.cmd_validate(d) and "연결 끊김" in s.message


def test_v_and_y_need_the_armed_arm(real):
    s, d, fake = session_on(real, state="ARM_CONFIRM")
    assert s.cmd_fixed_target(0.275, 20.0, d) and s.cmd_select()
    assert not s.cmd_validate(d) and "무장" in s.message
    fake.set("ARMED", d.qpos[s.idx.qpos])
    assert s.cmd_validate(d) and s.state == "PASSED"
    fake.set("HOLDING", d.qpos[s.idx.qpos])
    assert not s.cmd_approve(d) and "무장" in s.message
    assert s.status()["real_arm"]["gate"]


def test_not_armed_is_told_as_not_armed_not_as_a_failed_validation(real):
    s, d, fake = session_on(real, state="ARM_CONFIRM")
    events = []
    s.listeners.append(events.append)
    assert s.cmd_fixed_target(0.275, 20.0, d)
    req = {"v": 1, "type": "command", "request_id": "r1", "command": "use_current_path", "expected_rev": s.rev,
           "deadline_unix": None, "auto_run": True}
    res = Dispatcher().handle(s, d, req)
    assert not res["accepted"] and res["reason_code"] == "ARM_NOT_ARMED" and s.state == "GENERATED"
    assert "y를 눌러" in notice_text(events[-1], s)                  # the screen says what to do, not "failed"
    assert s.cmd_select() and menu_state(s)["chip"] == {"text": "실물 무장 대기", "icon": "warning"}
    assert not s.cmd_validate(d) and "y를 눌러" in notice_text(events[-1], s)
    fake.up, fake.down_reason = False, "gone"
    res = Dispatcher().handle(s, d, dict(req, request_id="r2", expected_rev=s.rev))
    assert res["reason_code"] == "ARM_DISCONNECTED" and menu_state(s)["chip"]["text"] == "로봇 연결 끊김"
    fake.up = True
    fake.set("ARMED", d.qpos[s.idx.qpos])
    assert menu_state(s)["chip"]["text"] == "검증 대기"


def test_moved_arm_invalidates_the_approval(real):
    s, d, fake = session_on(real)
    to_passed(s, d)
    q = d.qpos[s.idx.qpos].copy()
    q[0] += np.radians(1.0)                                     # the real arm is not where the path starts
    fake.set("ARMED", q)
    s.on_arm(d, [])
    assert not s.cmd_approve(d) and s.state == "GENERATED"
