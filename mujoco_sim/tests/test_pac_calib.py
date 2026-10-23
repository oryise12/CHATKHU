"""Real pac_vision calibration (replayed from pac_vision's recorded session) drives the monitor camera."""
import sys
from pathlib import Path

import numpy as np
import pytest

PV = Path(__file__).resolve().parents[2] / "pac_vision"                     # pac_vision next to mujoco_sim
if not PV.exists():
    PV = Path.home() / "research/manipulator/26_project_manipulator/pac_vision"
pytestmark = pytest.mark.skipif(not (PV / "tests/data/sample_session.npz").exists(), reason="pac_vision not here")


@pytest.fixture(scope="module")
def calib(tmp_path_factory):
    sys.path.insert(0, str(PV / "src"))
    from pac_vision.session import replay_session, write_cam2base
    res, sess = replay_session(PV / "tests/data/sample_session.npz")
    assert res.passed
    f = write_cam2base(tmp_path_factory.mktemp("pv") / "cam2base.yaml", res, sess["configs"]["camera"])
    return f, tuple(sess["rgb_size"])


def test_projection_and_rays_are_pac_vision_s(calib):
    from pac_vision.session import load_cam2base
    from pinch_sim.canvas import cal_to_world
    from pinch_sim.pac_calib import PacVisionCamera, is_pac_vision_file
    f, size = calib
    assert is_pac_vision_file(f)
    cam, conv = PacVisionCamera.load(f, *size), load_cam2base(f)
    B = np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1], [0.15, -0.1, 0.05]])
    uv, front = cam.project(cal_to_world(B))
    assert front.all() and np.allclose(uv, conv.base_to_pixel(B, rgb_size=size), atol=1e-9)
    for u, v in [(320, 240), (400, 300), (250, 330)]:
        o, r = cam.pixel_ray(u, v)
        p = cam.desk_point(u, v)                              # pac_vision's own pixel -> desk point
        assert np.linalg.norm(np.cross(p - o, r)) < 1e-6      # lies on our ray
    assert cam.scaled_to(1280, 720).project(cal_to_world(B))[0].shape == (5, 2)


def test_mock_file_is_not_taken_for_a_real_one(scene, data, tmp_path):
    from pinch_sim.camera_model import CameraModel
    from pinch_sim.canvas import load_cam2base, write_mock_cam2base
    from pinch_sim.pac_calib import is_pac_vision_file, load_camera
    m, _, _ = scene
    f = write_mock_cam2base(tmp_path / "mock.yaml", CameraModel.from_mujoco(m, data, "side_cam", 768, 480))
    assert not is_pac_vision_file(f) and isinstance(load_camera(f, load_cam2base), CameraModel)


def test_depth_point_is_pac_vision_s_pixel_to_base(calib):
    """On the recorded Astra depth, our depth_z + point_at_z (used for the 3-D pinch) give pac_vision's own
    pixel_to_base point (same median, same registration), and a moved robot pose moves nothing on the screen."""
    from pac_vision.session import load_cam2base, load_session
    from pinch_sim.canvas import set_robot_pose, world_to_cal
    from pinch_sim.pac_calib import PacVisionCamera
    f, size = calib
    cam, conv = PacVisionCamera.load(f, *size), load_cam2base(f)
    depth = np.asarray(load_session(PV / "tests/data/sample_session.npz")["depth"])
    checked = 0
    for u, v in [(320, 240), (400, 300), (250, 330), (300, 200)]:
        r = conv.pixel_to_base(u, v, depth, rgb_size=size, half=2)
        z = cam.depth_z(u, v, depth, half=2, percentile=50.0)
        if not r.valid:
            continue
        assert z is not None
        assert np.linalg.norm(world_to_cal(cam.point_at_z(u, v, z)) - r.xyz) < 0.002
        checked, good = checked + 1, (u, v)
    assert checked >= 2
    try:
        set_robot_pose(0.2, -0.1, 30.0)                       # the robot elsewhere: the desk point stays the same
        r = conv.pixel_to_base(*good, depth, rgb_size=size, half=2)
        z = cam.depth_z(*good, depth, half=2, percentile=50.0)
        assert np.linalg.norm(world_to_cal(cam.point_at_z(*good, z)) - r.xyz) < 0.002
    finally:
        set_robot_pose(0.0, 0.0, 0.0)
