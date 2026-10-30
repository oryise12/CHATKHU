"""Real pac_vision calibration (replayed from pac_vision's recorded session) drives the monitor camera."""
import sys
from pathlib import Path

import numpy as np
import pytest

PV = Path(__file__).resolve().parents[2] / "pac_vision"                     # pac_vision next to mujoco_sim
pytestmark = pytest.mark.skipif(not (PV / "tests/data/sample_session.npz").exists(), reason="pac_vision not here")


def _write(folder, res, camera_cfg, **kw):
    """cam2base.yaml + the "current" pac_vision camera.yaml in the same folder (what the sim compares with)."""
    import yaml
    from pac_vision.session import write_cam2base
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "camera.yaml").write_text(yaml.safe_dump(camera_cfg), encoding="utf-8")
    return write_cam2base(folder / "cam2base.yaml", res, camera_cfg, **kw)


@pytest.fixture(scope="module")
def session_result():
    sys.path.insert(0, str(PV / "src"))
    from pac_vision.session import replay_session
    res, sess = replay_session(PV / "tests/data/sample_session.npz")
    assert res.passed
    return res, sess


@pytest.fixture(scope="module")
def calib(tmp_path_factory, session_result):
    res, sess = session_result
    f = _write(tmp_path_factory.mktemp("pv"), res, sess["configs"]["camera"])
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


# ------------------------------------------------------------------ which calibration files the live camera may use
def test_a_passed_file_with_current_settings_is_accepted(calib):
    from pinch_sim.pac_calib import calibration_problems
    assert calibration_problems(calib[0]) == []


def test_failed_example_mock_and_changed_settings_are_refused(tmp_path, session_result, scene, data):
    import copy
    import dataclasses
    from pinch_sim.camera_model import CameraModel
    from pinch_sim.canvas import write_mock_cam2base
    from pinch_sim.pac_calib import calibration_problems
    res, sess = session_result
    cam_cfg = sess["configs"]["camera"]
    failed = dataclasses.replace(res, checks=[dataclasses.replace(c, ok=False) for c in res.checks])
    assert any("passed: false" in p for p in calibration_problems(_write(tmp_path / "failed", failed, cam_cfg)))
    assert calibration_problems(PV / "config" / "cam2base.example.yaml")
    m, _, _ = scene
    mock = write_mock_cam2base(tmp_path / "mock.yaml", CameraModel.from_mujoco(m, data, "side_cam", 768, 480))
    assert calibration_problems(mock)
    # registration re-measured (pose.yaml) after the calibration: the file no longer matches what is opened now
    f = _write(tmp_path / "changed", res, cam_cfg)
    import yaml
    cur = copy.deepcopy(cam_cfg)
    cur["depth"]["flip"] = not cur["depth"].get("flip", False)
    (f.parent / "camera.yaml").write_text(yaml.safe_dump(cur), encoding="utf-8")
    assert any("depth.flip" in p for p in calibration_problems(f))


def test_find_video_device_by_name(tmp_path):
    from pinch_sim.pac_calib import find_video_device
    for n, name in ((0, "Integrated Camera"), (3, "Astra Pro HD Camera"), (2, "Astra Pro HD Camera")):
        (tmp_path / f"video{n}").mkdir()
        (tmp_path / f"video{n}" / "name").write_text(name + "\n")
    assert find_video_device("Astra", tmp_path, can_open=lambda n: True) == 2
    assert find_video_device("Astra", tmp_path, can_open=lambda n: n == 3) == 3     # the node that gives frames
    assert find_video_device("Orbbec", tmp_path, can_open=lambda n: True) is None


def test_marker_monitor_needs_agreeing_checks(calib):
    """One odd check is not a camera move; a hand over the markers for a moment keeps 'verified'."""
    from types import SimpleNamespace as NS
    from pinch_sim.pac_calib import MarkerMonitor, PacVisionCamera
    mon = MarkerMonitor(PacVisionCamera.load(calib[0]), None)
    assert mon.state == "no_config" and not mon.verified
    mon.mset, mon.state = object(), "checking"
    r = lambda s: NS(state=s, summary=lambda: s)
    assert not mon.apply(r("verified"), 0.0) and mon.state == "checking"
    assert mon.apply(r("verified"), 0.5) and mon.verified
    assert not mon.apply(r("moved"), 1.0) and mon.verified                # one odd frame
    mon.apply(r("verified"), 1.5)
    mon.apply(r("not_visible"), 2.0)
    mon.apply(r("not_visible"), 2.5)
    assert mon.verified                                                    # hand over the markers for a moment
    mon.apply(r("not_visible"), 7.0)
    assert mon.state == "not_visible"
    mon.apply(r("moved"), 7.5)
    assert mon.apply(r("moved"), 8.0) and mon.state == "moved"
    mon.apply(r("not_visible"), 8.5)
    mon.apply(r("not_visible"), 9.0)
    assert mon.state == "moved"                                            # stays until the markers agree again
