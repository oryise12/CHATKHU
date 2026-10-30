"""Stopping a servo-profile HOME, faults, comm loss (incl. the watchdog-disabled "motion continues" case), exit."""
import math

import numpy as np

from helpers import System, fake_cfg
from manipulator_control.supervisor import S


def cfg_fast(**kw):
    c = fake_cfg(home={"speed_deg_s": 11.25, "countdown_s": 1.0})
    for k, v in kw.items():
        c.motion[k].update(v)
    return c


def _q(s):
    return np.degrees(s.sup.cal.q(s.sup.fast))


def test_stop_during_home_brakes_confirms_and_holds_without_finishing_home():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(60)                                    # 3 s in: cruising
    q_stop = _q(s)
    s.press("STOP")
    s.cycle(1)
    assert s.state == S.STOPPING                   # not HOLDING just because a command was sent
    assert s.until_state(S.HOLDING, timeout_s=10)
    stop = s.events("stop_confirmed")[0]
    assert stop["kind"] == "brake"
    s.cycle(60)
    q_end = _q(s)
    home = np.degrees(s.cfg.home_q_rad)
    moving = np.abs(home - q_stop) > 1.0
    # stopped between the stop point and HOME: did not continue to HOME, did not reverse, and travelled at most
    # the braking distance v^2/(2a) (+ what the profile moves during the write latency)
    plan = s.sup.plan
    t_stop = 3.0
    for k, a in enumerate(plan.axes):
        if not moving[k]:
            continue
        v, acc = abs(math.degrees(float(a.qd_at(t_stop)))), math.degrees(a.a_rad_s2)
        # + latency allowance 0.1 s + one read cycle (STOP arrives between reads) + the servo's tracking lag
        brake = v * v / (2 * acc) + v * (0.1 + 0.05 + 0.03) + 1.5 * acc * 0.1 ** 2 + 0.5
        assert abs(q_end[k] - home[k]) > 0.5, a.name
        assert 0 <= (q_end[k] - q_stop[k]) * np.sign(home[k] - q_stop[k]) <= brake, a.name
    v = [abs(s.sup.fast[i].present_velocity) for i in s.sup.ids]
    assert max(v) <= 1 and s.state == S.HOLDING


def test_quit_during_home_stops_first_then_exits_holding():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(40)
    s.press("QUIT")                                 # Ctrl+C and closing the viewer put the same QUIT event
    assert s.until_state(S.EXITED, timeout_s=15)
    order = [e["to"] for e in s.events("state")][-3:]
    assert order == ["STOPPING", "HOLDING", "EXITED"]
    rep = s.sup.final_report
    assert all("토크 ON" in v and "정지" in v for v in rep["axes"].values())


def test_exception_in_the_loop_during_home_triggers_a_braking_stop():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(30)
    real_step = s.sup.step
    calls = {"n": 0}

    def boom(*a, **k):
        if calls["n"] == 0:
            calls["n"] += 1
            raise RuntimeError("injected")
        return real_step(*a, **k)
    s.sup.step = boom
    try:                                            # one cycle of Runner.run: the step raises
        s.clock.advance(s.dt)
        s.runner.cycle()
    except RuntimeError as e:
        s.runner.handle_exception(e)
    s.sup.step = real_step
    assert s.state == S.STOPPING and s.events("exception")
    assert s.until_state(S.FAULT, timeout_s=10)
    assert s.sup.fault.code == "EXCEPTION" and s.events("stop_confirmed")


def test_stuck_joint_is_cancelled_with_goal_equal_present_not_extended():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(20)
    s.bus.servos[3].stuck = True                    # joint 3 blocked
    assert s.until_state(S.STOPPING, timeout_s=10)
    req = s.events("stop_requested")[0]
    assert 3 in req["abort_ids"] and req["kind"] == "abort"
    sent = s.events("command_sent")[-1]["values"][3]
    assert sent["goal"] == s.bus.servos[3].regs["goal_position"]
    assert abs(sent["goal"] - s.bus.servos[3].regs["present_position"]) < 50
    assert s.until_state(S.FAULT, timeout_s=15)
    assert s.sup.fault.code in ("TRACKING", "TIMEOUT")
    assert all(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)   # no automatic torque off
    assert "residual_counts" in [e for e in s.log.events if e["event"] == "state" and e["to"] == "FAULT"][-1]


def test_torque_off_only_after_explicit_confirmation():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(20)
    s.bus.servos[3].stuck = True
    assert s.until_state(S.FAULT, timeout_s=20)
    s.press("TORQUE_OFF")
    s.cycle(3)
    assert all(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
    s.press("CONFIRM")
    assert s.until_state(S.RELEASED, timeout_s=3)
    assert not any(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)


def test_comm_lost_during_home_watchdog_off_servo_keeps_going_and_pc_says_unknown():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(20)
    s.bus.link_down = True                          # cable / U2D2 gone: nothing reaches the servos
    assert s.until_state(S.COMM_LOST, timeout_s=2)
    assert "MOTION UNKNOWN" in s.sup.view().message and "12V" in s.sup.view().message
    assert "COMM LOST / MOTION UNKNOWN" in s.sup.view().overlay
    assert s.sup.cal is None
    # meanwhile the servo follows its internal profile to HOME and stops there (watchdog disabled = default)
    s.clock.advance(20.0)
    s.bus.advance()
    servo5 = s.bus.servos[5]
    assert abs(servo5.pos - servo5.regs["goal_position"]) < 2
    # link back: no auto re-calibration, no auto HOME, still latched
    s.bus.link_down = False
    s.cycle(20)
    assert s.state == S.COMM_LOST and s.sup.cal is None
    assert "자동으로 재보정·HOME을 재개하지 않습니다" in s.sup.view().message
    n_writes = len(s.writes())
    s.press("NEW_SESSION")
    # the link was lost: the stored calibration is gone (user rule), so a new session refuses without writing
    assert s.until_state(S.FAULT, timeout_s=3), (s.state, s.sup.fault)
    assert s.sup.fault.code == "NO_CALIBRATION" and len(s.writes()) == n_writes
    assert all(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)


def test_partial_reply_loss_marks_unknown_not_holding():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(10)
    s.bus.silent_ids.add(4)                         # ID 4 receives and moves, its replies are lost
    assert s.until_state(S.COMM_LOST, timeout_s=2)
    assert s.sup.fault.unknown_axes and s.sup.state != S.HOLDING
    rep = s.sup._final_report()
    assert all("미확인" in v for v in rep["axes"].values())


def test_stale_state_is_flagged():
    s = System(cfg=cfg_fast())
    s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=1)
    s.bus.rx_loss = True
    s.cycle(6)                                      # 0.3 s > stale_after 0.25 s
    assert s.sup.view().stale and "[STALE]" in s.sup.view().overlay


def test_reboot_during_ready_invalidates():
    s = System(cfg=cfg_fast())
    s.confirm_preview()
    assert s.until_state(S.READY, timeout_s=40)
    s.bus.servos[2].reboot()
    assert s.until_state(S.FAULT, timeout_s=2)
    assert s.sup.fault.code == "INVALIDATED" and s.sup.cal is None
    assert s.sup.fault.message.count("ID 2") >= 1
    assert s.store.load() is None and s.events("calibration_discarded")   # the count frame was reset


def test_servo_self_shutdown_is_not_reported_as_holding():
    s = System(cfg=cfg_fast())
    s.confirm_preview()
    assert s.until_state(S.READY, timeout_s=40)
    s.bus.servos[6].hardware_error(0x20)            # overload -> servo turns its torque off
    assert s.until_state(S.FAULT, timeout_s=2)
    msg = s.sup.fault.message
    assert "ID 6" in msg and ("유지 중이 아님" in msg or "경보" in msg or "Hardware Error" in msg)
    s.press("QUIT")
    s.cycle(2)
    s.runner.finish()                               # the final fresh read the bus loop does on exit
    assert "토크 OFF" in s.sup.final_report["axes"][6]
    assert s.bus.servos[6].regs["torque_enable"] == 0 and s.bus.servos[6].regs["hardware_error_status"]  # not cleared


def test_reboot_during_home_brakes_others_then_faults():
    s = System(cfg=cfg_fast())
    s.to_homing()
    s.cycle(30)
    s.bus.servos[3].reboot()
    assert s.until_state(S.FAULT, timeout_s=10)
    assert s.sup.fault.code == "INVALIDATED"


def test_watchdog_semantics_in_the_fake_servo():
    """Only relevant if a watchdog were enabled (prepare refuses that): reads keep it fed, a dead link does not."""
    from helpers import SimClock
    from manipulator_control.fake_bus import FakeBus
    from manipulator_control import control_table as ct
    clock = SimClock()
    bus = FakeBus(clock, ids=(1,))
    sv = bus.servos[1]
    sv.regs.update(bus_watchdog=5, profile_velocity=50, profile_acceleration=5)     # 100 ms
    sv._set("torque_enable", 1)
    sv.regs["goal_position"] = 50000
    bus.rx_loss = True                              # replies lost, but every read still reaches the servo
    for _ in range(40):
        clock.advance(0.05)
        bus.sync_read(*ct.FAST, (1,))
    assert sv.regs["bus_watchdog"] == 5             # fed by the read requests: no timeout
    bus.rx_loss, bus.link_down = False, True
    clock.advance(0.3)
    bus.advance()
    assert sv.regs["bus_watchdog"] == -1            # nothing received for 300 ms > 100 ms
    x = sv.prof_x
    clock.advance(0.5)
    bus.advance()
    assert sv.prof_x == x                           # stopped
    bus.link_down = False
    assert sv.write(116, ct.encode(0, 4, True)) == 4    # goal writes refused (data range error) until cleared
    assert sv.write(98, b"\x00") == 0 and sv.regs["bus_watchdog"] == 0


def test_exit_while_waiting_reports_torque_off():
    s = System(purpose="hold", cfg=cfg_fast())
    s.run_until(lambda x: x.at_step(0, "zero"), timeout_s=1)
    s.press("QUIT")
    s.cycle(1)
    assert s.state == S.EXITED and s.sup.final_report["note"] == "모든 축 토크 OFF." and not s.writes()


def test_torque_off_from_holding_after_confirmation():
    s = System(purpose="hold", cfg=cfg_fast())
    s.align_all()
    assert s.until_state(S.HOLDING, timeout_s=5)
    s.press("TORQUE_OFF")
    s.cycle(2)
    assert all(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
    s.press("CONFIRM")
    assert s.until_state(S.RELEASED, timeout_s=3), (s.state, s.sup.fault)
    assert not any(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
