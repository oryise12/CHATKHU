"""Pinch with depth (no camera): synthetic depth images + synthetic MediaPipe hands.

The pinch grabs the path point nearest to it on the screen and moves it by the hand's 3-D motion, 1:1.
Also: the robot pose in the calibration frame, and the depth filter (jitter, stray frames).
"""
import numpy as np
import pytest

from pinch_sim import canvas
from pinch_sim.app import MouseEditor
from pinch_sim.camera_model import CameraModel
from pinch_sim.canvas import cal_dir_to_world, cal_to_world, set_robot_pose, user_axes, world_to_cal
from pinch_sim.hand_depth import DepthConfig, HandDepth
from pinch_sim.hand_input import HandConfig, HandController
from pinch_sim.session import Session

from test_hand_input import hand
from test_voice_gateway import RM


@pytest.fixture(autouse=True)
def _robot_at_origin():
    set_robot_pose(0.0, 0.0, 0.0)
    yield
    set_robot_pose(0.0, 0.0, 0.0)


# ---------------------------------------------------------------- robot pose in the calibration frame
def test_robot_pose_moves_and_turns_the_robot_in_the_calibration_frame():
    set_robot_pose(0.10, -0.20, 90.0)
    assert np.allclose(world_to_cal([0, 0, 0]), [0.10, -0.20, 0.0])          # robot base centre
    assert np.allclose(world_to_cal([0, 0.1, 0]), [0.10, -0.10, 0.0])        # robot forward (+Y world) = +Y cal
    p = np.array([0.31, -0.07, 0.12])
    assert np.allclose(world_to_cal(cal_to_world(p)), p)
    assert np.allclose(cal_dir_to_world([1, 0, 0]) @ cal_dir_to_world([1, 0, 0]), 1.0)
    ax = user_axes("right")                                                   # voice axes stay the calibration axes
    assert np.allclose(world_to_cal(ax["right"]) - world_to_cal([0, 0, 0]), [1, 0, 0])
    assert canvas.robot_pose() == (0.10, -0.20, 90.0)


# ---------------------------------------------------------------- depth filter
class _FlatCam:
    """Camera stand-in: pixel (u, v) at depth z -> (u/1000, v/1000, z); depth read straight from the image."""

    def depth_z(self, u, v, depth_mm, half=3, percentile=25.0):
        from pinch_sim.hand_depth import patch_depth
        z = patch_depth(depth_mm, u, v, half, percentile)
        return z / 1000 if z > 0 else None

    def point_at_z(self, u, v, z):
        return np.array([u / 1000, v / 1000, z])


def _fingers(z_mm, shape=(120, 160), uv=(80, 60)):
    lm = np.zeros((21, 2))
    lm[:] = uv
    depth = np.zeros(shape, np.uint16)
    depth[uv[1] - 10:uv[1] + 10, uv[0] - 10:uv[0] + 10] = z_mm
    return lm, depth


def test_filter_keeps_a_resting_hand_still_and_drops_a_stray_frame():
    hd = HandDepth(_FlatCam(), DepthConfig())
    rng = np.random.default_rng(0)
    t, outs = 0.0, []
    for i in range(90):                                       # 3 s at rest with +-2 mm depth noise
        t += 1 / 30
        lm, depth = _fingers(700 + int(rng.integers(-2, 3)))
        outs.append(hd.update(lm, depth, t)[2])
    assert np.ptp(outs[30:]) < 0.0015                         # far less than the raw 4 mm spread
    lm, depth = _fingers(1300)                                # one stray frame 60 cm away
    t += 1 / 30
    assert abs(hd.update(lm, depth, t)[2] - outs[-1]) < 1e-9
    for _ in range(45):                                       # a real 3 cm move is followed fully (no offset)
        t += 1 / 30
        lm, depth = _fingers(730)
        out = hd.update(lm, depth, t)
    assert abs(out[2] - 0.730) < 0.0005


def test_no_depth_at_the_fingers_gives_no_point():
    hd = HandDepth(_FlatCam(), DepthConfig())
    lm, _ = _fingers(700)
    assert hd.update(lm, np.zeros((120, 160), np.uint16), 0.1) is None and hd.reason == "no_depth_at_fingers"


# ---------------------------------------------------------------- pinch moves the grabbed point in 3-D
@pytest.fixture
def rig(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    cam = CameraModel.from_mujoco(m, data, "side_cam", W, H)
    hcfg = HandConfig()
    me = MouseEditor(cam, cfg, 1.0)
    me.radius = hcfg.select_radius_px
    ctrl = HandController(hcfg, depth=HandDepth(cam))
    return s, data, cam, me, ctrl, W, H


def _depth_for(lm, W, H, z_mm):
    """Depth image registered to the camera: the hand's fingers at z_mm, nothing else measured."""
    depth = np.zeros((H, W), np.uint16)
    for x, y in (lm[:, :2] * [W, H]).astype(int):
        depth[max(0, y - 9):y + 10, max(0, x - 9):x + 10] = z_mm
    return depth


def _run(ctrl, s, d, me, W, H, frames, t0=0.0):
    t = t0
    for lm, z_mm in frames:
        t += 1 / 30
        with s.acting("hand"):
            ctrl.step(lm, t, W, H, W, H, me, s, d, depth_mm=None if lm is None else _depth_for(lm, W, H, z_mm))
    return t


def test_pinch_moves_the_grabbed_point_by_the_hands_3d_motion(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    p0 = s.path.points_m[k].copy()
    u, v = cam.project(p0)[0][0]
    z0 = (cam.R_cw @ p0 + cam.t_cw)[2]                      # the hand at the point's own depth (any depth works)
    z0_mm, z1_mm = int(round(z0 * 1000)), int(round(z0 * 1000)) - 40      # then 4 cm toward the camera
    du = 25                                                  # and 25 px to the right on the screen
    lm0, lm1 = hand(u / W, v / H, "pinch"), hand((u + du) / W, v / H, "pinch")
    frames = [(hand(u / W, v / H, "open"), z0_mm)] * 6 + [(lm0, z0_mm)] * 20
    frames += [(hand((u + du * i / 20) / W, v / H, "pinch"), z0_mm - 2 * i) for i in range(1, 21)]
    frames += [(lm1, z1_mm)] * 60
    t = _run(ctrl, s, d, me, W, H, frames)
    assert ctrl.grab3d is not None and ctrl.grab3d["started"] and ctrl.grab3d["k"] == k
    tip = lambda lm: lm[8, :2] * [W, H]                      # the index fingertip carries the point
    expected = cam.point_at_z(*tip(lm1), z1_mm / 1000) - cam.point_at_z(*tip(lm0), z0_mm / 1000)
    assert np.linalg.norm(ctrl.grab3d["delta"] - expected) < 0.001          # 1:1, within 1 mm
    _run(ctrl, s, d, me, W, H, [(hand((u + du) / W, v / H, "open"), z1_mm)] * 6, t0=t)
    assert not s.edit.dragging and s.edit.editor.can_undo
    assert s.events[-1].kind == "edit_end" and s.events[-1].actor == "hand"
    moved = s.path.points_m[s.pointed_index()] - p0
    assert np.linalg.norm(moved - expected) < 0.004                          # the path point went there


def test_a_still_pinch_selects_without_editing(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    u, v = cam.project(s.path.points_m[k])[0][0]
    z_mm = 900
    fp = s.path.fingerprint
    frames = [(hand(u / W, v / H, "open"), z_mm)] * 6 + [(hand(u / W, v / H, "pinch"), z_mm)] * 30
    frames += [(hand(u / W, v / H, "open"), z_mm)] * 6
    _run(ctrl, s, d, me, W, H, frames)
    assert s.path.fingerprint == fp and s.pointed is not None
    assert np.linalg.norm(s.path.points_m[s.pointed_index()] - s.path.points_m[k]) < 1e-9


def test_pinch_without_depth_falls_back_to_the_height_drag(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    u, v = cam.project(s.path.points_m[k])[0][0]
    z0 = s.path.points_m[k, 2]
    frames = [(hand(u / W, v / H, "open"), 0)] * 6 + [(hand(u / W, v / H, "pinch"), 0)] * 6
    frames += [(hand(u / W, (v - i * 3) / H, "pinch"), 0) for i in range(1, 16)]
    frames += [(hand(u / W, (v - 45) / H, "open"), 0)] * 6
    _run(ctrl, s, d, me, W, H, frames)
    assert ctrl.grab3d is None and s.edit.editor.can_undo
    assert s.path.points_m[s.pointed_index()][2] > z0 + 0.01


def test_pointing_never_grabs(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    u, v = cam.project(s.path.points_m[k])[0][0]
    fp = s.path.fingerprint
    frames = [(hand((u + i) / W, v / H, "point"), 800) for i in range(30)]   # a moving pointing finger
    _run(ctrl, s, d, me, W, H, frames)
    assert not ctrl.grabbing and s.path.fingerprint == fp
