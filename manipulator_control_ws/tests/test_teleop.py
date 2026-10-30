"""EE teleop: IK on the tcp site, live following on fake_bus, unreachable targets, stop / resume, HOME preview."""
import math

import mujoco
import numpy as np
import pytest

from helpers import System, fake_cfg, viz
from manipulator_control import panel
from manipulator_control.ik import IKSolver
from manipulator_control.supervisor import S

HOME = np.array([0.0, 0.641283, 1.312362, -0.899717, -1.570796, -0.139769])


def test_ik_reaches_pose_targets_and_respects_limits():
    m, arms = viz()
    ik = IKSolver(m, arms["measured"])
    lo, hi = np.full(6, -math.pi), np.full(6, math.pi)
    p0, R0 = ik.fk(HOME)
    for dp in ([0.03, 0, 0], [0, 0.05, 0], [0, 0, 0.03], [0, 0, -0.05]):
        r = ik.solve(p0 + np.array(dp), R0, HOME, lo, hi)
        assert r.ok and r.pos_err_m < 5e-4 and r.rot_err_deg < 0.3, dp
        p, R = ik.fk(r.q)
        assert np.linalg.norm(p - (p0 + dp)) < 6e-4
    tight_lo, tight_hi = HOME - math.radians(2), HOME + math.radians(2)
    r = ik.solve(p0 + np.array([0, 0, -0.05]), R0, HOME, tight_lo, tight_hi)
    assert not r.ok and np.all(r.q >= tight_lo - 1e-12) and np.all(r.q <= tight_hi + 1e-12)
    far = ik.solve(p0 + np.array([0, 0, 0.6]), R0, HOME, lo, hi)
    assert not far.ok


def _teleop_system():
    s = System(purpose="teleop", cfg=fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5}))
    s.confirm_preview()
    assert s.until_state(S.TELEOP, timeout_s=40), (s.state, s.sup.fault)
    return s


def _tcp(s):
    return s.ik.fk(s.sup.cal.q(s.sup.fast))


def test_teleop_starts_at_home_with_the_target_on_the_tcp():
    s = _teleop_system()
    p, _ = _tcp(s)
    assert np.allclose(s.sup.teleop["pos"], p, atol=1e-3)
    v = s.sup.view()
    assert v.target_pose is not None and v.target_ok and "EE 목표" in v.progress and v.overlay.isascii()


def test_jog_key_moves_the_real_arm_to_the_ik_pose():
    s = _teleop_system()
    p0, R0 = _tcp(s)
    for _ in range(3):
        s.press("JOG", "+x")                                 # 3 x 1 cm along +X (robot base frame)
    s.cycle(80)
    p, R = _tcp(s)
    assert np.linalg.norm(p - (p0 + [0.03, 0, 0])) < 2e-3, (p - p0)
    from manipulator_control.ik import rot_error
    assert math.degrees(np.linalg.norm(rot_error(R0, R))) < 1.0   # orientation kept
    assert "teleop_goal" in [e["label"] for e in s.events("command_sent")]
    q = s.sup.cal.q(s.sup.fast)
    lo = np.array([j.limit_rad[0] for j in s.cfg.joints])
    hi = np.array([j.limit_rad[1] for j in s.cfg.joints])
    assert np.all(q >= lo) and np.all(q <= hi)


def test_rotation_and_step_keys():
    s = _teleop_system()
    _, R0 = _tcp(s)
    s.press("STEP", "up")                                    # 2 cm / 10 deg
    s.cycle(1)
    assert s.sup.teleop["step_deg"] == 10.0 and s.sup.teleop["step_m"] == pytest.approx(0.02)
    s.press("JOG", "+rz")
    s.cycle(80)
    _, R = _tcp(s)
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(R0.T @ R) - 1) / 2))))
    assert abs(ang - 10.0) < 1.0


def test_unreachable_target_is_not_sent_and_marked():
    s = _teleop_system()
    s.cycle(5)
    n_before = len([e for e in s.events("command_sent") if e["label"] == "teleop_goal"])
    for _ in range(30):
        s.press("JOG", "+z")                                 # 30 cm up: out of reach
    s.cycle(20)
    v = s.sup.view()
    assert not v.target_ok and "불가" in v.progress and "NOT REACHABLE" in v.overlay
    sent = [e for e in s.events("command_sent") if e["label"] == "teleop_goal"]
    assert len(sent) == n_before                               # nothing new for the unreachable target
    for _ in range(30):
        s.press("JOG", "-z")                                 # back into reach: following resumes
    s.cycle(10)
    assert s.sup.view().target_ok


def _viewer_at(view, frame="base"):
    from manipulator_control.viz import StateViewer
    sv = StateViewer(settings={"frame": frame})
    sent = []
    sv.on_target = lambda pos, quat: sent.append((pos, quat))
    sv.frame(view, 1280, 860)
    return sv, sent


def _px(sv, w):
    x, y, _ = sv.screen.project(w)
    return x, y


def _drag(sv, a, b):
    sv.mouse_button("left", True, False, *a)
    name = sv.gizmo.active
    sv.mouse_move(*b)
    sv.mouse_button("left", False, False, *b)
    return name


def _reset(sv, view):
    sv.drag_target, sv._drag_until = None, 0.0
    sv.frame(view, 1280, 860)


def test_gizmo_projection_matches_mujoco_selection():
    from manipulator_control.gizmo import ScreenCam
    sv, _ = _viewer_at(_teleop_view_stub())
    m, d = sv.m, sv.d
    for relx, rely in ((0.5, 0.5), (0.47, 0.4), (0.45, 0.3)):
        sel, gid, fid, sid = np.zeros(3), np.zeros(1, np.int32), np.zeros(1, np.int32), np.zeros(1, np.int32)
        if mujoco.mjv_select(m, d, sv.opt, 1280 / 860, relx, rely, sv.scn, sel, gid, fid, sid) < 0:
            continue
        x, y, _ = ScreenCam.from_scene(sv.scn, 1280, 860).project(sel)
        assert abs(x - relx * 1280) < 1e-3 and abs(y - (1 - rely) * 860) < 1e-3          # float32 camera


def _teleop_view_stub():
    from manipulator_control.supervisor import View
    m, arms = viz()
    ik = IKSolver(m, arms["measured"])
    p, R = ik.fk(HOME)
    return View(state="TELEOP", calibrated=True, stale=False, q_meas=HOME, q_target=HOME,
                target_pose=(p, panel.mat_to_quat(R)))


@pytest.mark.parametrize("frame", ["base", "tool"])
def test_gizmo_arrow_drag_moves_along_that_axis_only(frame):
    from manipulator_control.gizmo import ARROW, frame_axes
    v = _teleop_view_stub()
    sv, sent = _viewer_at(v, frame)
    pos, R = sv._shown
    axes = frame_axes(R, frame)
    for k, name in enumerate("xyz"):
        _reset(sv, v)
        a = axes[:, k]
        grab = pos + 0.5 * sum(ARROW) * sv.gizmo.scale(sv.screen, pos) * a
        assert sv.gizmo.pick(sv.screen, *_px(sv, grab), pos, axes) == name
        assert _drag(sv, _px(sv, grab), _px(sv, grab + 0.03 * a)) == name
        p_new, q_new = sent[-1]
        dp = p_new - pos
        assert abs(dp @ a - 0.03) < 1e-4 and np.linalg.norm(dp - (dp @ a) * a) < 1e-9     # only along this axis
        assert np.allclose(panel.quat_to_mat(q_new), R, atol=1e-9)                       # orientation untouched
        assert sv.gizmo.active is None


@pytest.mark.parametrize("frame", ["base", "tool"])
def test_gizmo_ring_drag_rotates_about_that_axis_only(frame):
    from manipulator_control.gizmo import RING_R, axis_rotation, frame_axes
    v = _teleop_view_stub()
    sv, sent = _viewer_at(v, frame)
    pos, R = sv._shown
    axes = frame_axes(R, frame)
    done = []
    for k, name in enumerate(("rx", "ry", "rz")):
        _reset(sv, v)
        a, e1, e2 = axes[:, k], axes[:, (k + 1) % 3], axes[:, (k + 2) % 3]
        if abs(a @ sv.screen.ray(*_px(sv, pos))[1]) < 0.3:
            continue                                     # nearly edge-on from this camera
        r = RING_R * sv.gizmo.scale(sv.screen, pos)
        on_ring = lambda deg: pos + r * (math.cos(math.radians(deg)) * e1 + math.sin(math.radians(deg)) * e2)
        ang = math.radians(20)                       # grab at 45 deg: away from where two rings cross
        assert _drag(sv, _px(sv, on_ring(45)), _px(sv, on_ring(65))) == name
        p_new, q_new = sent[-1]
        assert np.allclose(p_new, pos, atol=1e-12)                                        # rotation only
        assert np.allclose(panel.quat_to_mat(q_new), axis_rotation(a, ang) @ R, atol=1e-6)
        done.append(name)
    assert len(done) >= 2


def test_empty_space_drag_moves_the_camera_and_sends_nothing():
    sv, sent = _viewer_at(_teleop_view_stub())
    az = sv.cam.azimuth
    sv.mouse_button("left", True, False, 40, 40)
    sv.mouse_move(140, 40)
    sv.mouse_button("left", False, False, 140, 40)
    assert sv.cam.azimuth != az and not sent and sv.gizmo.active is None


def test_gizmo_drag_is_followed_by_the_real_arm():
    from manipulator_control.gizmo import ARROW
    s = _teleop_system()
    p0, R0 = _tcp(s)
    sv, _ = _viewer_at(s.sup.view(), "base")
    sv.on_target = lambda pos, quat: s.press("TARGET", (pos, quat))
    pos, _ = sv._shown
    grab = pos + 0.5 * sum(ARROW) * sv.gizmo.scale(sv.screen, pos) * np.array([0.0, 1.0, 0.0])
    _drag(sv, _px(sv, grab), _px(sv, grab + [0.0, 0.03, 0.0]))
    s.cycle(80)
    p, R = _tcp(s)
    assert np.linalg.norm(p - (p0 + [0.0, 0.03, 0.0])) < 2e-3
    from manipulator_control.ik import rot_error
    assert math.degrees(np.linalg.norm(rot_error(R0, R))) < 1.0


def test_far_target_is_chased_with_a_bounded_lead_and_reset_works():
    s = _teleop_system()
    p0, R0 = _tcp(s)
    from manipulator_control.gizmo import axis_rotation
    target_R = axis_rotation(R0[:, 2], math.radians(60)) @ R0           # about 60 deg of joint motion at once
    s.press("TARGET", (p0, panel.mat_to_quat(target_R)))
    s.cycle(3)
    assert s.sup.view().target_ok and "따라가는 중" in s.sup.view().progress
    lead_max = 0.0
    for _ in range(200):
        s.cycle(1)
        q = s.sup.cal.q(s.sup.fast)
        lead_max = max(lead_max, math.degrees(np.abs(s.sup.teleop["sent"] - q).max()))
    assert lead_max <= s.cfg.m("teleop", "max_jump_deg") + 3.0            # commanded at most ~30 deg ahead
    _, R = _tcp(s)
    from manipulator_control.ik import rot_error
    assert math.degrees(np.linalg.norm(rot_error(target_R, R))) < 1.0       # and it got there
    s.press("TARGET_RESET")
    s.cycle(3)
    assert s.sup.view().target_ok and np.allclose(s.sup.teleop["pos"], _tcp(s)[0], atol=1e-6)


def test_panel_torque_off_while_moving_stops_first_then_releases():
    s = _teleop_system()
    for _ in range(3):
        s.press("JOG", "+y")
    s.cycle(3)
    s.press("TORQUE_OFF_NOW")
    assert s.until_state(S.RELEASED, timeout_s=15), (s.state, s.sup.fault)
    names = [e["event"] if e["event"] != "command_sent" else e["label"] for e in s.log.events]
    assert names.index("stop_confirmed") < names.index("torque_off_user")              # stopped, then off
    assert not any(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
    assert s.sup.cal is not None and s.store.load() is not None                         # counts kept: still usable


def test_panel_torque_off_from_the_home_preview_and_panel_states():
    s = System(purpose="teleop", cfg=fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5}))
    assert s.until_state(S.HOME_PREVIEW, timeout_s=2)
    s.press("TORQUE_OFF_NOW")
    assert s.until_state(S.RELEASED, timeout_s=3)
    assert "home_profile_goal" not in [e["label"] for e in s.events("command_sent")]
    assert not any(s.bus.servos[i].regs["torque_enable"] for i in s.sup.ids)
    on = panel.enabled
    assert on("TELEOP")["torque_off"] and on("HOLDING")["torque_off"] and not on("RELEASED")["torque_off"]
    assert on("HOLDING")["teleop"] and not on("TELEOP")["teleop"] and on("HOMING")["stop"]


def test_panel_helpers():
    for rpy in ([10, -20, 30], [0, 89.9, 0], [-170, 5, 120]):
        R = panel.rpy_to_mat(*np.radians(rpy))
        assert np.allclose(panel.rpy_to_mat(*panel.mat_to_rpy(R)), R, atol=1e-9)
        assert np.allclose(panel.quat_to_mat(panel.mat_to_quat(R)), R, atol=1e-9)


def test_stop_holds_and_t_resumes_h_previews_home():
    s = _teleop_system()
    for _ in range(4):
        s.press("JOG", "+y")
    s.cycle(3)
    s.press("STOP")
    assert s.until_state(S.HOLDING, timeout_s=10)
    s.press("TELEOP")
    assert s.until_state(S.TELEOP, timeout_s=2)
    s.press("HOME")
    assert s.until_state(S.HOME_PREVIEW, timeout_s=2)
    s.press("STOP")                                           # cancel the preview: stays, no motion
    assert s.until_state(S.HOLDING, timeout_s=2)


def test_quit_during_teleop_stops_then_exits_holding():
    s = _teleop_system()
    s.press("JOG", "+x")
    s.cycle(3)
    s.press("QUIT")
    assert s.until_state(S.EXITED, timeout_s=15)
    assert [e["to"] for e in s.events("state")][-3:] == ["STOPPING", "HOLDING", "EXITED"]


def test_home_preview_needs_y_and_shows_the_path():
    s = System(purpose="teleop", cfg=fake_cfg(home={"speed_deg_s": 22.5, "countdown_s": 0.5}))
    assert s.until_state(S.HOME_PREVIEW, timeout_s=5)
    s.cycle(30)
    assert s.state == S.HOME_PREVIEW and "home_profile_goal" not in [e["label"] for e in s.events("command_sent")]
    v1 = s.sup.view().q_target
    s.cycle(10)
    assert not np.allclose(v1, s.sup.view().q_target)          # the green ghost replays the path
    s.press("CONFIRM")
    assert s.until_state(S.HOMING, timeout_s=5)
