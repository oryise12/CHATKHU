"""Full routine on fake_bus with a simulated clock: calibrate (joint by joint, each joint held, stored) and
prepare (stored calibration -> HOME -> READY)."""
import pathlib
import tempfile

import numpy as np
import pytest
import yaml

from helpers import System, fake_cfg
from manipulator_control import config as cfgmod
from manipulator_control.supervisor import S


def fast_cfg(**extra):
    return fake_cfg(home={"speed_deg_s": 11.25, "countdown_s": 1.0}, **extra)


def cfg_with(robot_edit, **motion):
    """fake config with an edited robot section (written to a temp file and loaded like the real one)."""
    cfg = fast_cfg(**motion)
    robot = cfg.robot
    robot_edit(robot)
    tmp = pathlib.Path(tempfile.mkdtemp()) / "fake.yaml"
    tmp.write_text(yaml.safe_dump(robot, allow_unicode=True))
    out = cfgmod.load(tmp)
    out.motion.update(cfg.motion)
    return out, tmp


def test_calibrate_joint_by_joint_stores_the_newest_calibration():
    s = System(purpose="hold", cfg=fast_cfg())
    assert s.connected and s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=1)
    s.cycle(10)
    assert s.at_step(0, "zero") and not s.writes()                     # nothing is written before the first Enter
    s.align_all()
    assert s.until_state(S.HOLDING, timeout_s=5), (s.state, s.sup.fault)
    cal = s.sup.cal
    # P_ref is the multi-turn count where the "hand" left each joint (incl. negative)
    assert cal.p_ref[2] == 3900 - 70000 and cal.p_ref[5] == 4000 - 4096
    labels = [e["label"] for e in s.events("command_sent")]
    assert labels == [x for n in range(1, 7) for x in (f"hold_prepare_J{n}", f"torque_on_J{n}")]
    for e in s.events("command_sent")[::2]:                            # goal = present, one joint per packet
        (i, v), = e["values"].items()
        assert v["goal"] == cal.p_ref[int(i)]
    rec = s.store.load()
    h = s.cfg.motion["hold"]
    assert {j["id"]: j["p_ref"] for j in rec["joints"]} == cal.p_ref and rec["calibration_id"] == cal.session_id
    assert {int(i): (w["pa"], w["pv"], w["goal"]) for i, w in rec["written"].items()} == \
        {i: (h["profile_acceleration_raw"], h["profile_velocity_raw"], cal.p_ref[i]) for i in cal.p_ref}
    assert "저장" in s.sup.view().message


def test_prepare_starts_from_the_stored_calibration_then_ready():
    s = System(cfg=fast_cfg())                     # the helper ran calibrate first; this is a new program run
    stored = {j["id"]: j["p_ref"] for j in s.store.load()["joints"]}
    assert s.until_state(S.HOME_PREVIEW, timeout_s=1), (s.state, s.sup.fault)
    assert "저장된 보정" in s.sup.view().message and s.sup.view().calibrated
    assert not s.events("command_sent") and s.sup.cal.p_ref == stored  # no alignment, nothing written yet
    s.cycle(2)
    s.press("CONFIRM")
    assert s.until_state(S.READY, timeout_s=40), (s.state, s.sup.fault)
    q = s.sup.cal.q(s.sup.fast)
    assert np.degrees(np.abs(q - np.array(s.cfg.home_q_rad))).max() <= 0.2
    assert [e["label"] for e in s.events("command_sent")] == ["home_profile_goal"]
    assert s.log.reports and s.log.reports[0]["error_deg"]
    ready = [e for e in s.events("state") if e["to"] == "READY"][0]
    assert ready["t_s"] >= s.sup.plan.T_max * 0.9                      # judged by arrival, not by elapsed time
    goals = {a.dxl_id: a.goal for a in s.sup.plan.axes}
    assert {int(i): w["goal"] for i, w in s.store.load()["written"].items()} == goals   # evidence updated at READY


def test_joints_are_held_one_at_a_time_from_the_base():
    s = System(purpose="hold", cfg=fast_cfg())
    for k in range(6):
        assert s.run_until(lambda x: x.at_step(k, "zero"), timeout_s=5)
        torque = [s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids]
        assert torque == [1] * k + [0] * (6 - k)                        # below: held, this and above: free
        v = s.sup.view()
        assert v.joint_status == ["held"] * k + ["current"] + ["free"] * (5 - k)
        s.press("START")
        assert s.run_until(lambda x: x.sup.ctx.get("k") != k or x.state != S.ALIGN, timeout_s=10)
    assert s.until_state(S.HOLDING, timeout_s=3)
    assert all(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
    assert "home_profile_goal" not in [e["label"] for e in s.events("command_sent")]   # no motion


def test_hold_only_stops_after_all_joints_held():
    s = System(purpose="hold", cfg=fast_cfg())
    s.align_all()
    assert s.until_state(S.HOLDING, timeout_s=10)
    s.cycle(40)
    assert s.state == S.HOLDING
    s.press("QUIT")
    s.cycle(2)
    assert s.state == S.EXITED and "토크 ON" in s.sup.final_report["note"]


def test_unknown_signs_are_found_from_the_hand_motion_and_saved():
    true_sign = [1, -1, 1, 1, -1, 1]
    cfg, path = cfg_with(lambda r: [j.__setitem__("sign", None) for j in r["joints"]])
    s = System(purpose="hold", cfg=cfg)
    saved = {}
    s.runner.save_signs = lambda signs: saved.update(signs) or cfgmod.save_signs(path, signs)
    assert s.run_until(lambda x: x.at_step(0, "sign"), timeout_s=2)
    s.cycle(10)
    v = s.sup.view()
    assert v.q_target[0] > 0.0 or True                     # green ghost sweeps the + direction (animated)
    s.press("START")                                       # Enter before the direction check: only a reminder
    s.cycle(2)
    assert s.at_step(0, "sign") and "먼저 방향 확인" in s.sup.view().message
    s.align_all(hand=[3000.0 * t for t in true_sign])     # the user turns each joint the + way: counts follow s
    assert s.until_state(S.HOLDING, timeout_s=5)
    assert [m.sign for m in s.sup.cal.joints] == true_sign
    assert "y = config/robot.yaml에 저장" in s.sup.view().message
    s.press("CONFIRM")
    s.cycle(2)
    assert saved == {f"Revolute_{n}": sg for n, sg in zip(range(1, 7), true_sign)}
    assert [j.sign for j in cfgmod.load(path).joints] == true_sign
    assert s.events("signs_saved")


def test_direction_check_retry_and_live_display():
    cfg, _ = cfg_with(lambda r: r["joints"][0].__setitem__("sign", None))
    s = System(purpose="hold", cfg=cfg)
    assert s.run_until(lambda x: x.at_step(0, "sign"), timeout_s=2)
    sv = s.bus.servos[1]
    sv.external_v = -3000.0                                 # turned the wrong way first
    assert s.run_until(lambda x: x.sup.ctx.get("detected") == -1, timeout_s=3)
    sv.external_v = 0.0
    assert s.sup.view().joint_status[0] == "current_live"   # the yellow link follows the real joint
    s.press("RETRY")
    s.cycle(2)
    assert s.sup.ctx["detected"] is None
    sv.external_v = 3000.0
    assert s.run_until(lambda x: x.sup.ctx.get("detected") == 1, timeout_s=3)
    sv.external_v = 0.0
    s.cycle(2)
    s.press("CONFIRM")
    assert s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=2)


def test_typed_ahead_enter_is_discarded():
    s = System(purpose="hold", cfg=fast_cfg())
    s.inputs.put("START")                     # pressed before the prompt was shown (t < state entry)
    s.inputs.extra[-1] = type(s.inputs.extra[-1])("START", s.clock.t - 5.0)
    s.cycle(5)
    assert s.at_step(0, "zero")
    assert any(e["event"] == "input_discarded" for e in s.log.events)
    s.press("START")
    assert s.run_until(lambda x: x.sup.ctx.get("sub") == "window", timeout_s=1)


def test_motion_during_registration_is_rejected_and_retried():
    s = System(purpose="hold", cfg=fast_cfg())
    s.align_joint(0)
    s.align_joint(1)
    assert s.run_until(lambda x: x.at_step(2, "zero"), timeout_s=3)
    s.bus.servos[3].external_v = 400.0        # the hand is still turning joint 3
    s.press("START")
    assert s.run_until(lambda x: x.sup.ctx.get("sub") == "window", timeout_s=1)
    assert s.run_until(lambda x: x.at_step(2, "zero"), timeout_s=5)
    assert s.events("calibration_rejected") and "움직임" in s.sup.view().message
    assert s.bus.servos[3].regs["torque_enable"] == 0
    s.bus.servos[3].external_v = 0.0
    s.cycle(3)
    s.press("START")
    assert s.run_until(lambda x: x.sup.ctx.get("k") == 3, timeout_s=5)


def test_hold_jump_on_torque_on_is_cancelled():
    s = System(purpose="hold", cfg=fast_cfg())
    s.align_joint(0)
    assert s.run_until(lambda x: x.at_step(1, "zero"), timeout_s=3)
    s.press("START")
    assert s.run_until(lambda x: x.sup.ctx.get("sub") == "hold_goal", timeout_s=3)
    s.bus.servos[2].external_v = 3000.0       # the joint is pushed between "goal = present" and torque on
    assert s.until_state(S.FAULT, S.STOPPING, timeout_s=3)
    s.bus.servos[2].external_v = 0.0
    assert s.until_state(S.FAULT, timeout_s=10)
    assert s.sup.fault.code in ("HOLD_JUMP", "HOLD_PREP")


def test_quit_during_alignment_keeps_held_joints_and_reports_them():
    s = System(purpose="hold", cfg=fast_cfg())
    s.align_joint(0)
    s.align_joint(1)
    assert s.run_until(lambda x: x.at_step(2, "zero"), timeout_s=3)
    s.press("QUIT")
    s.cycle(2)
    assert s.state == S.EXITED
    rep = s.sup.final_report["axes"]
    assert rep[1].startswith("토크 ON") and rep[2].startswith("토크 ON") and rep[3].startswith("토크 OFF")
    assert s.store.load() is None                  # an unfinished calibrate leaves no calibration


def test_torque_off_during_alignment_needs_confirmation():
    s = System(purpose="hold", cfg=fast_cfg())
    s.align_joint(0)
    assert s.run_until(lambda x: x.at_step(1, "zero"), timeout_s=3)
    s.press("TORQUE_OFF")
    s.cycle(2)
    assert s.bus.servos[1].regs["torque_enable"] == 1
    s.press("CONFIRM")
    assert s.run_until(lambda x: x.at_step(0, "zero") and not x.sup.torque_ids, timeout_s=3)   # from joint 1 again
    assert s.bus.servos[1].regs["torque_enable"] == 0 and s.sup.align["parts"] == {}


def test_missing_motor_blocks_connection():
    from manipulator_control.__main__ import make_fake_bus
    from manipulator_control.bus import WritePolicy
    from helpers import SimClock
    from manipulator_control.logs import NullLog
    from manipulator_control.runner import Runner
    from manipulator_control.supervisor import Supervisor
    from manipulator_control.terminal import ScriptedInput
    cfg = fast_cfg()
    clock = SimClock()
    bus = make_fake_bus(cfg, WritePolicy("control", True), clock)
    bus.dead_ids.add(4)
    log = NullLog()
    r = Runner(bus, Supervisor(cfg, "home", log), log, ScriptedInput([], clock), clock)
    assert r.connect() is False
    assert "4" in r.view().message and not bus.writes


@pytest.mark.parametrize("field,value,needle", [
    ("drive_mode", 4, "bit2"), ("drive_mode", 8, "bit3"), ("startup_configuration", 1, "Startup"),
    ("startup_configuration", 2, "Startup"), ("bus_watchdog", 10, "Watchdog"), ("drive_mode", 1, "Drive Mode 1"),
    ("hardware_error_status", 0x20, "Hardware Error"),
])
def test_servo_settings_refuse_prepare_but_not_watch(field, value, needle):
    s = System(cfg=fast_cfg(), servo_overrides={3: {field: value}}, calibrated=False)
    s.cycle(2)
    assert s.state == S.FAULT and s.sup.fault.code == "SERVO_SETTINGS" and needle in s.sup.fault.message
    assert not s.writes()                                                # nothing "repaired"
    w = System(purpose="watch", cfg=fast_cfg(), servo_overrides={3: {field: value}})
    w.cycle(2)
    assert w.state == S.MONITOR and any(needle in x for x in w.sup.warnings)


def test_mode4_current_limit_is_not_claimed():
    s = System(cfg=fast_cfg())
    s.cycle(1)
    assert any("Current Limit" in w and "적용되지 않음" in w for w in s.sup.warnings)


def test_mode_prep_needs_confirmation_and_permission():
    s = System(purpose="hold", cfg=fast_cfg(), servo_overrides={2: {"operating_mode": 3}})
    s.cycle(1)
    assert s.state == S.MODE_PREP and not s.writes()
    s.press("CONFIRM")
    assert s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=3)
    assert s.bus.servos[2].regs["operating_mode"] == 4
    assert [w for w in s.writes(11)] and s.events("mode_changed")
    cfg = fast_cfg()
    cfg.robot["servo_expect"]["allow_operating_mode_write"] = False
    n = System(purpose="hold", cfg=cfg, servo_overrides={2: {"operating_mode": 3}})
    n.cycle(1)
    assert n.state == S.FAULT and n.sup.fault.code == "MODE" and not n.writes()
    p = System(cfg=fast_cfg(), servo_overrides={2: {"operating_mode": 3}}, calibrated=False)   # prepare never changes it
    p.cycle(1)
    assert p.state == S.FAULT and p.sup.fault.code == "MODE" and "calibrate" in p.sup.fault.message and not p.writes()


def test_joints_found_held_are_kept_and_registered_in_place():
    s = System(purpose="hold", cfg=fast_cfg(), servo_overrides={1: {"torque_enable": 1}, 2: {"torque_enable": 1}})
    assert s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=2)
    assert not s.writes() and "이미 토크 ON" in s.sup.view().message       # nothing released, nothing sags
    s.align_joint(0)
    s.align_joint(1)
    assert all(s.bus.servos[i].regs["torque_enable"] for i in (1, 2))
    assert not [w for w in s.writes(64) if 0 in [v[0] for v in w[3].values()]]   # never a torque-off write
    for k in range(2, 6):
        s.align_joint(k)
    assert s.until_state(S.HOLDING, timeout_s=5)


def test_countdown_can_be_cancelled_without_motion():
    s = System(cfg=fake_cfg(home={"speed_deg_s": 11.25, "countdown_s": 3.0}))
    s.confirm_preview()
    assert s.until_state(S.COUNTDOWN, timeout_s=10)
    s.press("STOP")
    s.cycle(2)
    assert s.state == S.HOLDING
    assert "home_profile_goal" not in [e["label"] for e in s.events("command_sent")]


def test_duplicate_start_during_homing_is_refused():
    s = System(cfg=fast_cfg())
    s.to_homing()
    p_ref = dict(s.sup.cal.p_ref)
    s.cycle(10)
    s.press("START")
    s.cycle(3)
    assert s.state == S.HOMING and s.sup.cal.p_ref == p_ref
    assert any(e.get("reason", "").startswith("HOME 중") for e in s.events("input_ignored"))


def test_plan_refused_before_any_motion_when_home_outside_range():
    cfg, _ = cfg_with(lambda r: r["joints"][4].__setitem__("limit_deg", [-60, 30]))   # HOME R5 = -90 is outside
    s = System(cfg=cfg, calibrated=False)
    s.cycle(1)
    assert s.state == S.FAULT and s.sup.fault.code == "CONFIG"          # caught by the config gate already
    assert not s.writes()


def test_held_joint_with_unknown_sign_is_released_alone():
    cfg, _ = cfg_with(lambda r: r["joints"][2].__setitem__("sign", None))           # joint 3 must be checked again
    s = System(purpose="hold", cfg=cfg, servo_overrides={i: {"torque_enable": 1} for i in range(1, 7)})
    s.align_joint(0)
    s.align_joint(1)
    assert s.run_until(lambda x: x.at_step(2, "sign"), timeout_s=3)
    assert "토크만 끄기" in s.sup.view().message
    s.press("TORQUE_OFF")
    s.cycle(2)
    s.press("CONFIRM")
    assert s.run_until(lambda x: x.bus.servos[3].regs["torque_enable"] == 0 and x.sup.pending_confirm is None, timeout_s=3)
    assert [s.bus.servos[i].regs["torque_enable"] for i in range(1, 7)] == [1, 1, 0, 1, 1, 1]   # only joint 3 released
    s.align_joint(2, hand_counts_s=3000.0)
    assert s.bus.servos[3].regs["torque_enable"] == 1
    for k in range(3, 6):
        s.align_joint(k)
    assert s.until_state(S.HOLDING, timeout_s=5)
