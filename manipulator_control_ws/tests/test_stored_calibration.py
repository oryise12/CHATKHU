"""The stored calibration: only the newest calibrate is kept, prepare / teleop reuse it while the servos prove that
their count frame was not reset, and refuse (writing nothing) otherwise."""
from helpers import System, fake_cfg
from manipulator_control.calibration import CalibrationStore
from manipulator_control.supervisor import S


def cfg():
    return fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5})


def _refused(s, needle):
    assert s.until_state(S.FAULT, timeout_s=1), s.state
    assert s.sup.fault.code == "NO_CALIBRATION" and needle in s.sup.fault.message, s.sup.fault.message
    assert not s.events("command_sent")                       # nothing written in this run


def test_power_cycle_between_runs_refuses_and_drops_the_record():
    s = System(purpose="hold", cfg=cfg())
    s.align_all()
    assert s.until_state(S.HOLDING, timeout_s=5)
    s.press("QUIT")
    assert s.until_state(S.EXITED, timeout_s=1) and s.store.load()
    for sv in s.bus.servos.values():
        sv.reboot()                                           # 12 V off/on: counts back inside one turn, PV/PA 0
    s.start("teleop")
    _refused(s, "재부팅")
    s.press("QUIT")
    s.cycle(2)
    assert s.store.load() is None                             # unusable: gone, the next run needs calibrate


def test_a_new_calibrate_drops_the_old_one_at_its_start():
    s = System(cfg=cfg())                                     # calibrated once, prepare run now
    old = s.store.load()["calibration_id"]
    s.press("QUIT")
    s.cycle(2)
    s.start("hold")
    assert s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=1)
    assert s.store.load() is None and s.events("calibration_discarded")
    s.align_all()
    assert s.until_state(S.HOLDING, timeout_s=5)
    assert s.store.load()["calibration_id"] not in (None, old)


def test_keeps_using_the_calibration_across_runs_until_the_next_calibrate():
    s = System(purpose="teleop", cfg=cfg())
    first = s.store.load()
    for run in range(3):
        if run:
            s.start("teleop")
        s.confirm_preview()                                   # every run goes to HOME first
        assert s.until_state(S.TELEOP, timeout_s=40), (run, s.state, s.sup.fault)
        for _ in range(2):
            s.press("JOG", "+x" if run % 2 == 0 else "-x")
        s.cycle(60)
        s.press("QUIT")
        assert s.until_state(S.EXITED, timeout_s=15)
        rec = s.store.load()
        assert rec["calibration_id"] == first["calibration_id"] and rec["joints"] == first["joints"]
        assert rec["calibrated_wall"] == first["calibrated_wall"]
        f = s.sup.fast
        assert {int(i): w["goal"] for i, w in rec["written"].items()} == {i: f[i].goal_position for i in s.sup.ids}


def test_profile_changed_by_another_program_is_refused_but_a_goal_change_is_not():
    s = System(cfg=cfg())
    s.press("QUIT")
    s.cycle(2)
    s.bus.servos[4].regs["goal_position"] += 5                # Goal is not evidence (the servo moves it itself)
    s.start("home")
    assert s.until_state(S.HOME_PREVIEW, timeout_s=1), (s.state, s.sup.fault)
    s.press("QUIT")
    s.cycle(2)
    s.bus.servos[4].regs["profile_velocity"] += 1             # e.g. the Wizard wrote a profile: refused
    s.start("home")
    _refused(s, "ID 4")


def test_torque_off_exit_keeps_the_calibration_although_goal_follows_present():
    s = System(cfg=cfg())
    assert s.until_state(S.HOME_PREVIEW, timeout_s=1)
    s.press("TORQUE_OFF_NOW")
    assert s.until_state(S.RELEASED, timeout_s=3)
    s.bus.servos[2].pos += 400                                # sags while free: Goal follows Present
    s.cycle(3)
    assert s.bus.servos[2].regs["goal_position"] == round(s.bus.servos[2].pos)
    s.press("QUIT")
    s.cycle(2)
    assert s.store.load() is not None
    s.start("teleop")
    assert s.until_state(S.RESUME_HOLD, timeout_s=1), (s.state, s.sup.fault)


def test_changed_reference_angle_in_the_config_is_refused():
    s = System(cfg=cfg())
    s.press("QUIT")
    s.cycle(2)
    edited = cfg()
    j = edited.joints[2]
    import dataclasses
    edited = dataclasses.replace(edited, joints=edited.joints[:2] + (dataclasses.replace(j, q_ref_rad=j.q_ref_rad + 0.1),)
                                 + edited.joints[3:])
    s.cfg = edited
    s.start("home")
    _refused(s, "Revolute_3")


def test_joints_found_with_torque_off_are_held_in_place_after_y():
    s = System(cfg=cfg())
    assert s.until_state(S.HOME_PREVIEW, timeout_s=1)
    s.press("STOP")                                           # cancel the HOME preview: holding
    assert s.until_state(S.HOLDING, timeout_s=1)
    s.press("TORQUE_OFF")
    s.cycle(1)
    s.press("CONFIRM")
    assert s.until_state(S.RELEASED, timeout_s=3)
    assert s.sup.cal is not None                              # torque off does not reset the counts
    s.press("QUIT")
    s.cycle(2)
    assert s.store.load() is not None
    s.bus.servos[3].pos += 300                                # sagged a little while free
    s.start("teleop")
    assert s.until_state(S.RESUME_HOLD, timeout_s=1)
    assert s.sup.view().calibrated and not s.events("command_sent")
    s.cycle(10)
    assert all(not s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)   # nothing before y
    s.press("CONFIRM")
    assert s.until_state(S.HOME_PREVIEW, timeout_s=3), (s.state, s.sup.fault)
    assert all(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
    sent = s.events("command_sent")
    assert [e["label"] for e in sent] == ["hold_prepare_resume", "torque_on_resume"]
    assert {int(i): v["goal"] for i, v in sent[0]["values"].items()}[3] == int(s.bus.servos[3].pos)   # held where it is


def test_watch_never_touches_the_store():
    s = System(cfg=cfg())
    s.press("QUIT")
    s.cycle(2)
    rec = s.store.load()
    s.start("watch")
    s.cycle(5)
    s.press("QUIT")
    s.cycle(2)
    assert s.store.load() == rec


def test_comm_lost_drops_the_calibration_calibrate_again():
    s = System(cfg=cfg())
    assert s.store.load() is not None
    s.bus.link_down = True
    assert s.until_state(S.COMM_LOST, timeout_s=2)
    assert s.store.load() is None and "다시 calibrate" in s.sup.view().message   # user rule: lost link -> calibrate
    s.press("QUIT")
    s.cycle(2)
    s.bus.link_down = False
    s.start("home")
    _refused(s, "저장된 보정이 없습니다")


def test_store_file_roundtrip(tmp_path):
    st = CalibrationStore(tmp_path / "x" / "calibration_state.json")
    assert st.load() is None and st.discard() == "nothing stored"
    rec = {"schema": 1, "calibration_id": "a", "joints": [], "servo": {}, "written": {}}
    st.save(rec, "s1")
    first = st.load()
    assert first["calibrated_session"] == "s1" and first["calibrated_wall"]
    st.save(dict(rec, written={"1": {"pa": 3, "pv": 29, "goal": 5}}), "s2")
    assert st.load()["calibrated_session"] == "s1" and st.load()["written"]["1"]["goal"] == 5   # same calibration
    st.save(dict(rec, calibration_id="b"), "s3")
    assert st.load()["calibrated_session"] == "s3"                                           # newest only
    assert st.discard().startswith("deleted") and st.load() is None
    assert not list((tmp_path / "x").glob("*.tmp"))


def test_goal_set_by_the_servo_at_torque_on_becomes_the_evidence():
    s = System(purpose="hold", cfg=cfg())
    s.align_joint(0)
    assert s.run_until(lambda x: x.at_step(1, "zero"), timeout_s=3)
    s.press("START")
    assert s.run_until(lambda x: x.sup.ctx.get("sub") == "hold_goal", timeout_s=3)
    s.bus.servos[2].pos += 0.6                      # the reading flickers by a count right at torque on
    for k in range(2, 6):
        s.align_joint(k)
    assert s.until_state(S.HOLDING, timeout_s=5), (s.state, s.sup.fault)
    ev = s.events("goal_set_by_torque_on")
    assert [e["id"] for e in ev] == [2] and ev[0]["read"] == ev[0]["written"] + 1
    assert s.store.load()["written"]["2"]["goal"] == s.bus.servos[2].regs["goal_position"] == ev[0]["read"]
    s.press("QUIT")
    s.cycle(2)
    s.start("teleop")
    assert s.until_state(S.HOME_PREVIEW, timeout_s=1), (s.state, s.sup.fault)
