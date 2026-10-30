"""Click a path point on the monitor, then move it by voice (move_point), then "done" validates and runs.

A click (press + release without moving) only marks the point; a drag still edits the height.
Voice directions are screen directions: up/down = world Z, right/left and away/toward_me = the
monitor camera's right / viewing direction flattened onto the desk.
"""
import numpy as np
import pytest

from pinch_sim.app import MouseEditor, screen_axes
from pinch_sim.camera_model import CameraModel
from pinch_sim.session import Session
from pinch_sim.voice_gateway import Dispatcher

from test_voice_gateway import RM, _run_to_end


@pytest.fixture
def setup(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    cam = CameraModel.from_mujoco(m, data, "side_cam", cfg["cameras"]["width_px"], cfg["cameras"]["height_px"])
    s.screen_axes = screen_axes(cam)
    return s, cam, data


def _move(s, d, rid, **kw):
    return Dispatcher().handle(s, d, {"v": 1, "type": "command", "command": "move_point", "request_id": rid,
                                      "expected_rev": s.rev, **kw})


def _display_xy(cam, p, scale=1.0):
    u, v = cam.project(p)[0][0]
    return u * scale, v * scale


def test_screen_axes_are_unit_and_match_the_camera(setup):
    s, cam, _ = setup
    ax = s.screen_axes
    for k in ("right", "away"):
        assert abs(np.linalg.norm(ax[k]) - 1) < 1e-12 and abs(ax[k][2]) < 1e-12
    assert abs(ax["right"] @ ax["away"]) < 1e-6        # camera quaternion precision
    # "right" moves a point to the right on the monitor, "away" moves it into the picture (deeper)
    p = s.path.points_m[len(s.path.points_m) // 2]
    u0 = cam.project(p)[0][0][0]
    assert cam.project(p + 0.02 * ax["right"])[0][0][0] > u0
    depth = lambda q: (cam.R_cw @ q + cam.t_cw)[2]
    assert depth(p + 0.02 * ax["away"]) > depth(p)


def test_click_selects_without_editing_and_drag_still_edits(setup, cfg):
    s, cam, d = setup
    me = MouseEditor(cam, cfg, 1.0)
    k = s.region_index("middle")
    x, y = _display_xy(cam, s.path.points_m[k])
    fp = s.path.fingerprint
    assert me.press(s, d, x, y) and me.release(s, d)
    assert s.pointed_index() is not None and s.path.fingerprint == fp and not s.edit.editor.can_undo
    # small jitter below the click threshold is still a click
    assert me.press(s, d, x, y)
    me.move(s, d, x + 2, y + 1)
    assert me.release(s, d) and s.path.fingerprint == fp
    # a real drag edits the height only (as before)
    assert me.press(s, d, x, y)
    me.move(s, d, x, y - 30)
    assert s.edit.dragging
    me.release(s, d)
    assert s.edit.editor.can_undo and s.path.fingerprint != fp


def test_voice_moves_the_clicked_point_by_the_requested_distance(setup):
    s, cam, d = setup
    r = _move(s, d, "a", target="here", direction="up", distance_cm=5)
    assert not r["accepted"] and r["reason_code"] == "NO_POINT"
    P0 = s.path.points_m.copy()
    assert s.point_select(s.region_index("middle"))
    for i, (direction, cm) in enumerate([("up", 5), ("right", 2), ("away", 8), ("toward_me", 2), ("left", 5)]):
        before = s.path.points_m[s.pointed_index()].copy()
        r = _move(s, d, f"m{i}", target="here", direction=direction, distance_cm=cm)
        assert r["accepted"], r
        after = s.path.points_m[s.pointed_index()]
        assert np.allclose(after - before, s.screen_axes[direction] * cm / 100, atol=2e-3)
        assert r["detail"]["moved_cm"] == pytest.approx(cm, abs=0.2)
    assert np.allclose(s.path.points_m[0], P0[0]) and np.allclose(s.path.points_m[-1], P0[-1])
    assert s.state == "GENERATED" and s.report is None


def test_region_targets_bad_values_and_new_path_clears_the_point(setup):
    s, cam, d = setup
    k_end = s.region_index("end")                  # an edit re-samples the path, so take it before
    r = _move(s, d, "r1", target="end", direction="up", distance_cm=2)
    assert r["accepted"] and r["detail"]["index"] == k_end
    for bad, code in [(dict(target="whole", direction="up", distance_cm=2), "BAD_TARGET"),
                      (dict(target="middle", direction="sideways", distance_cm=2), "BAD_DIRECTION"),
                      (dict(target="middle", direction="up", distance_cm=40), "BAD_DISTANCE"),
                      (dict(target="middle", direction="up", distance_cm=0), "BAD_DISTANCE")]:
        r = _move(s, d, f"bad-{code}-{bad['distance_cm']}", **bad)
        assert not r["accepted"] and r["reason_code"] == code
    assert s.point_select(s.region_index("middle"))
    s.cmd_new_target(d)
    assert s.pointed is None


def test_click_voice_edit_then_done_validates_and_runs(setup):
    s, cam, d = setup
    assert s.point_select(s.region_index("middle"))
    assert _move(s, d, "u1", target="here", direction="up", distance_cm=2)["accepted"]
    edited_fp = s.path.fingerprint
    r = Dispatcher().handle(s, d, {"v": 1, "type": "command", "command": "use_current_path",
                                   "request_id": "done", "expected_rev": s.rev})
    assert r["accepted"] and s.state == "PASSED" and s.auto_run is not None   # sync validation
    s.watch(d)                                           # the main loop's per-frame call starts it
    assert s.state == "EXECUTING", s.message
    _run_to_end(s, d)
    assert s.state == "DONE" and s.last_run["path_fp"] == edited_fp and s.last_run["path_source"] == "edited"


def test_dragged_point_becomes_voice_here_and_undo_restores(setup):
    s, cam, d = setup
    k = len(s.path.points_m) // 3
    p0 = s.path.points_m[k].copy()
    assert s.edit_begin(k, p0)
    s.edit_update(p0 + [0.0, 0.0, 0.04])
    assert s.edit_end()
    dragged = s.path.points_m[s.pointed_index()].copy()
    assert abs(dragged[2] - p0[2] - 0.04) < 2e-3                      # "here" = the point the hand moved
    r = _move(s, d, "after-drag", target="here", direction="left", distance_cm=2)
    assert r["accepted"]
    moved = s.path.points_m[s.pointed_index()] - dragged
    assert np.allclose(moved, s.screen_axes["left"] * 0.02, atol=2e-3)
    s._edit_history("undo")
    assert np.allclose(s.path.points_m[s.pointed_index()], dragged, atol=1e-6)
    s._edit_history("undo")
    assert s.pointed is None                                           # as before the drag: nothing picked
