"""Object on the desk -> new target (S6), on pac_vision's synthetic depth scene (no camera).

These check the geometry and the rules (one object / pointed object / robot excluded / tcp goal above the top);
the error on the real Astra (oblique view bias) is measured on site.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

PV = Path(__file__).resolve().parents[2] / "pac_vision"
sys.path.insert(0, str(PV / "tests"))
synth = pytest.importorskip("synth")

from pac_vision.cam2base import Cam2Base  # noqa: E402

from pinch_sim.canvas import cal_to_world, set_robot_pose  # noqa: E402
from pinch_sim.session import Session  # noqa: E402
from pinch_sim.vision_goal import VisionGoalConfig, choose, find_objects  # noqa: E402

from test_voice_gateway import RM  # noqa: E402

CUP = (0.30, 0.05, 0.035, 0.08)          # cylinder (x, y, radius, height) in base_ref, m
BOX = (0.25, -0.20, 0.03, 0.05)
ROBOT = (0.0, 0.0, 0.06, 0.15)           # the real robot's base standing on the desk


def scene(*cyls):
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    return Cam2Base(R, t, cam), synth.render_depth(cam, R, t, cylinders=cyls)


def test_one_object_top_centre_and_the_robot_is_not_an_object():
    conv, depth = scene(CUP, ROBOT)
    objs = find_objects(conv, depth, (0.0, 0.0), VisionGoalConfig(step_px=1))
    assert len(objs) == 1                                        # the robot base (within 15 cm) is excluded
    top = objs[0].top_base
    assert abs(top[2] - CUP[3]) < 0.005                          # z_top = the object's height
    assert np.hypot(top[0] - CUP[0], top[1] - CUP[1]) < 0.02     # oblique view: a small bias toward the camera
    obj, why = choose(objs)
    assert obj is objs[0] and why == ""


def test_several_objects_need_a_pointing_finger():
    conv, depth = scene(CUP, BOX)
    objs = find_objects(conv, depth, (0.0, 0.0), VisionGoalConfig(step_px=1))
    assert len(objs) == 2
    assert choose(objs) == (None, "several_objects")
    obj, _ = choose(objs, near_xy=(BOX[0] - 0.03, BOX[1]))       # the finger's desk point, a bit off
    assert abs(obj.top_base[2] - BOX[3]) < 0.005
    assert choose(objs, near_xy=(0.10, 0.30))[1] == "no_object_at_finger"
    assert choose([]) == (None, "no_object")


@pytest.fixture
def sess(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    set_robot_pose(0.0, 0.0, 0.0)
    yield s
    set_robot_pose(0.0, 0.0, 0.0)


def test_g_sets_the_object_as_target_with_the_tcp_above_its_top(sess, cfg, data):
    s = sess
    assert s.cmd_vision_target(data) == (False, "NO_CAMERA_TARGET")       # started without --depth
    conv, depth = scene(CUP)
    obj, _ = choose(find_objects(conv, depth, (0.0, 0.0), VisionGoalConfig(step_px=1)))
    s.vision_target = lambda: (cal_to_world(obj.top_base), "")
    s.set_edit_mode(True, "test")
    s.handle_key("G", data)
    assert s.state == "GENERATED" and s.target_label == "camera" and not s.edit_mode   # a new path, not edited
    top_w = cal_to_world(obj.top_base)
    assert np.allclose(s.target.tcp_goal_m, top_w + [0, 0, cfg["target"]["work_height_m"]])
    assert np.allclose(s.target.approach_m, top_w + [0, 0, cfg["target"]["approach_height_m"]])
    assert np.linalg.norm(s.path.points_m[-1] - s.target.tcp_goal_m) < 1e-9
    s.vision_target = lambda: (None, "several_objects")
    fp = s.path.fingerprint
    assert s.cmd_vision_target(data) == (False, "SEVERAL_OBJECTS") and s.path.fingerprint == fp
