"""Blank canvas: camera from a cam2base.yaml (pac_vision format), calibration frame axes, fixed A and B."""
import numpy as np
import yaml

from pinch_sim.camera_model import CameraModel
from pinch_sim.canvas import blank_canvas, cal_to_world, load_cam2base, world_to_cal, write_mock_cam2base
from pinch_sim.session import Session


def test_mock_calibration_projects_like_the_sim_camera(scene, cfg, data, tmp_path):
    m, idx, sha = scene
    sim_cam = CameraModel.from_mujoco(m, data, "side_cam", 768, 480)
    f = write_mock_cam2base(tmp_path / "cam2base.yaml", sim_cam)
    raw = yaml.safe_load(f.read_text())
    assert set(raw) >= {"R", "t", "camera"} and "MOCK" in raw["note"]
    cam = load_cam2base(f)
    P = np.array([[0.0, 0.2, 0.0], [0.1, 0.3, 0.15], [-0.05, 0.25, 0.3]])
    assert np.abs(cam.project(P)[0] - sim_cam.project(P)[0]).max() < 1e-6
    # P_base = R @ P_cam + t holds for the file, with the base in the calibration frame
    p_cam = P @ sim_cam.R_cw.T + sim_cam.t_cw
    assert np.allclose(p_cam @ np.asarray(raw["R"]).T + raw["t"], world_to_cal(P), atol=1e-9)
    img = blank_canvas(cam)
    assert img.shape == (480, 768, 3) and (img < 250).any()          # grid and axes were drawn


def test_calibration_frame_axes():
    # +X forward (= sim +Y), +Y left (= sim -X), +Z up; and back again
    assert np.allclose(cal_to_world([1, 0, 0]), [0, 1, 0])
    assert np.allclose(cal_to_world([0, 1, 0]), [-1, 0, 0])
    assert np.allclose(cal_to_world([0, 0, 1]), [0, 0, 1])
    p = np.array([0.12, -0.07, 0.2])
    assert np.allclose(world_to_cal(cal_to_world(p)), p)


def test_fixed_a_and_b(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    a, b = cal_to_world([0.10, -0.05, 0.22]), cal_to_world([0.27, 0.06, 0.0])
    ok, why = s.cmd_ab(a, b, data)
    assert ok, why
    assert np.linalg.norm(s.robot_state(data).tcp.p_m - a) < 1e-3            # the robot stands at A
    assert np.allclose(s.path.points_m[0], s.robot_state(data).tcp.p_m, atol=1e-6)
    assert np.allclose(s.path.points_m[-1], b + [0, 0, cfg["target"]["work_height_m"]])   # B + work height
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "PASSED", s.message
    s.cmd_new_target(data)                                                     # "new target" keeps the same B
    assert np.allclose(s.target.floor_point_m, b)
    ok, why = s.cmd_ab(cal_to_world([0.9, 0.0, 0.2]), b, data)                 # out of reach
    assert not ok and "not reachable" in why


def test_user_directions_are_fixed_to_the_calibration_axes():
    from pinch_sim.canvas import user_axes
    ax = user_axes("right")                    # user at the robot's right (-Y), facing +Y
    assert np.allclose(world_to_cal(ax["right"]), [1, 0, 0]) and np.allclose(world_to_cal(ax["away"]), [0, 1, 0])
    assert np.allclose(world_to_cal(ax["up"]), [0, 0, 1]) and np.allclose(ax["left"], -ax["right"])
    ax = user_axes("front")                    # facing the robot: right = +Y, away = -X
    assert np.allclose(world_to_cal(ax["right"]), [0, 1, 0]) and np.allclose(world_to_cal(ax["away"]), [-1, 0, 0])
    for side in ("right", "front", "behind", "left"):   # right x away = up for every side (no mirrored sets)
        a = user_axes(side)
        assert np.allclose(np.cross(a["right"], a["away"]), a["up"])
