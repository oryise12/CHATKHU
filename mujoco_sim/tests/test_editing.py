"""Mouse/hand path editing: geometry rules, screen back-projection, and the validation/approval gate."""
import numpy as np
import pytest

from pinch_sim.editing import PathEdit
from pinch_sim.model import load_config
from pinch_sim.overlay import project
from pinch_sim.paths import initial_path, polyline_distance
from pinch_sim.camera_model import CameraModel
from pinch_sim.screen import pick_index, pixel_to_plane, pixel_to_vertical, ray_plane_z
from pinch_sim.session import Session
from pinch_sim.targets import make_target

_T = load_config()["target"]
RM = round((_T["r_min_m"] + _T["r_max_m"]) / 2, 3)


@pytest.fixture
def base(cfg, kin, q_ready):
    return initial_path(kin.fk(q_ready), q_ready, make_target(RM, np.radians(10), cfg["target"]), cfg)


def drag(edit: PathEdit, k: int, dz: float):
    p0 = edit.current_path().points_m[k].copy()
    assert edit.begin(k, p0)
    assert edit.update(p0 + [0.0, 0.0, dz])
    return edit.end()


def _on_base(e, base, xi_new, cols):
    return np.column_stack([np.interp(xi_new, e.xi, base.points_m[:, i]) for i in cols])


def test_drag_changes_only_z_and_keeps_endpoints(base, cfg):
    e = PathEdit(base, cfg["editor"])
    k = base.waypoint_index[1] // 2
    assert drag(e, k, 0.04)
    new = e.current_path()
    xi_new = e.editor.paths["outbound"].xi
    assert np.max(np.abs(new.points_m[:, :2] - _on_base(e, base, xi_new, (0, 1)))) < 1e-12   # every x, y kept
    assert np.array_equal(new.points_m[0], base.points_m[0]) and np.array_equal(new.points_m[-1], base.points_m[-1])
    dz = new.points_m[:, 2] - _on_base(e, base, xi_new, (2,))[:, 0]
    assert 0.03 < dz.max() <= 0.04 + 1e-9 and dz.min() > -1e-12                 # local raise, no overshoot
    assert new.source == "edited" and new.fingerprint != base.fingerprint


def test_control_point_xy_is_forced_to_picked_vertical(base, cfg):
    e = PathEdit(base, cfg["editor"])
    k = 10
    p0 = base.points_m[k].copy()
    e.begin(k, p0 + [0.05, -0.05, 0.0])            # control points with a different x, y ...
    e.update(p0 + [0.08, 0.03, 0.03])
    e.end()
    xi_new = e.editor.paths["outbound"].xi
    assert np.allclose(e.current_path().points_m[:, :2], _on_base(e, base, xi_new, (0, 1)), atol=1e-12)
    assert e.current_path().points_m[:, 2].max() > base.points_m[:, 2].max() - 1e-9


def test_undo_redo_reset_restore_exact_geometry(base, cfg):
    e = PathEdit(base, cfg["editor"])
    drag(e, 15, 0.02)
    after = e.current_path().fingerprint
    assert e.undo() and e.current_path().fingerprint == base.fingerprint
    assert e.redo() and e.current_path().fingerprint == after
    assert e.reset() and e.current_path().fingerprint == base.fingerprint


def test_screen_ray_plane_roundtrip_and_invalid_rays(scene, cfg, data, base):
    m, _, _ = scene
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    cam = CameraModel.from_mujoco(m, data, "side_cam", W, H)
    for p in base.points_m[5:-5:7]:
        u, v = cam.project(p)[0][0]
        q = pixel_to_plane(cam, u, v, p[2])
        assert np.linalg.norm(q - p) < 1e-9
        assert pick_index(base.points_m, cam, u, v, 3.0) is not None
        # the same pixel on the point's vertical line gives back its height
        z = pixel_to_vertical(cam, u, v, p[:2], (0.0, 0.8))
        assert np.linalg.norm(z - p) < 1e-9
    assert pixel_to_vertical(cam, u, v, p[:2], (0.5, 0.8)) is None             # outside z range
    o, r = cam.pixel_ray(W / 2, H / 2)
    assert ray_plane_z(o, r, o[2] + 0.5) is None                              # plane behind the camera
    assert ray_plane_z(o, np.array([1.0, 0, 0]), 0.1) is None                 # parallel ray
    assert pixel_to_plane(cam, W / 2, H / 2, 0.1, radial_max=0.01) is None
    u, v = cam.project(base.points_m[0])[0][0]
    assert pick_index(base.points_m, cam, u, v, 2.0) != 0                     # endpoints not pickable


def _session(scene, cfg, data, threaded=False):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=threaded)
    s.cmd_fixed_target(RM, 10.0, data)
    return s


def _validate_run(s, d):
    s.cmd_select()
    s.cmd_validate(d)
    if s.state != "PASSED":
        return False
    assert s.cmd_approve(d)
    while s.state == "EXECUTING":
        s.physics_step(d)
    return s.state == "DONE"


def test_edit_invalidates_pass_and_edited_path_is_what_runs(scene, cfg, data):
    s = _session(scene, cfg, data)
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "PASSED"
    base = s.path
    k = base.waypoint_index[1] // 2
    p0 = base.points_m[k]
    assert s.edit_begin(k, p0)
    assert s.state == "GENERATED" and s.report is None and not s.cmd_approve(data)   # old PASS is gone
    s.edit_update(p0 + [0.0, 0.0, 0.04])
    assert s.edit_end() and s.path.source == "edited"
    assert not s.cmd_approve(data)
    edited = s.path
    assert _validate_run(s, data)
    run = s.last_run
    assert run["path_source"] == "edited" and run["path_fp"] == edited.fingerprint
    assert run["path_dev_max_m"] < 1e-3                                         # executed TCP follows the edit
    # and clearly not the original path near the edited point
    assert polyline_distance(edited.points_m[k:k + 1], base.points_m)[0] > 0.02


def test_unreachable_edit_fails_then_reset_runs(scene, cfg, data):
    s = _session(scene, cfg, data)
    base = s.path
    k = base.waypoint_index[1] - 3                       # near the approach point (z ~ 0.12)
    p0 = base.points_m[k]
    s.edit_begin(k, p0)
    s.edit_update(p0 + [0.0, 0.0, -0.15])                # pushed below the floor
    s.edit_end()
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "FAILED" and not s.cmd_approve(data)
    assert any("workspace" in b or "collision" in b for b in s.report.blocking())
    s._edit_history("reset")
    assert s.state == "GENERATED" and s.path.fingerprint == base.fingerprint
    assert _validate_run(s, data) and s.last_run["path_source"] == "initial"


def test_edits_blocked_while_validating_or_executing(scene, cfg, data):
    s = _session(scene, cfg, data, threaded=True)
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "VALIDATING"
    assert not s.edit_begin(10, s.path.points_m[10])
    s._future.result()
    s.poll()
    assert s.state == "PASSED"
    s.cmd_approve(data)
    assert s.state == "EXECUTING"
    assert not s.edit_begin(10, s.path.points_m[10]) and not s._edit_history("undo")
    s.cmd_cancel(data)
    assert s.state == "ABORTED"
    assert not s.edit_begin(10, s.path.points_m[10])          # stale path: generate a new one first
    s.close()


def test_edit_phase_colours_relative_height_up_to_waypoint(base):
    from pinch_sim.overlay import PATH_COLOR, Overlay, height_range, segment_colors
    idx = np.arange(len(base.points_m))
    a0, a1 = height_range(base.points_m, base.waypoint_index)
    assert (a0, a1) == (0, base.waypoint_index[-2])
    ov = Overlay("GENERATED", path=base.points_m, height_from=a0, height_until=a1)
    cols = np.array(segment_colors(ov, idx))
    end = base.waypoint_index[-2]                        # approach point
    mids = (base.points_m[:-1, 2] + base.points_m[1:, 2]) / 2
    seg = slice(0, end)
    hi, lo = int(np.argmax(mids[seg])), int(np.argmin(mids[seg]))
    assert cols[hi][0] > 0.9 and cols[hi][2] < 0.2          # highest -> red
    assert cols[lo][2] > 0.9 and cols[lo][0] < 0.2          # lowest -> blue
    assert all(tuple(c) == PATH_COLOR["GENERATED"] for c in cols[end:])   # after the waypoint: unchanged
    for state in ("PASSED", "FAILED", "EXECUTING"):       # results keep their single colour
        ov.state = state
        assert set(segment_colors(ov, idx)) == {PATH_COLOR[state]}


def test_after_pick_the_vertical_lift_is_also_excluded(scene, cfg, data, kin):
    """Start at a work point (object picked): lift -> transit -> descent; only the transit is coloured."""
    from pinch_sim.overlay import PATH_COLOR, Overlay, height_range, segment_colors
    q_low = kin.solve_ik(make_target(RM, 0.0, cfg["target"]).tcp_goal_m, [0, 0, -1], data.qpos[:6].copy()).q
    nxt = make_target(RM, np.radians(25), cfg["target"])
    p = initial_path(kin.fk(q_low), q_low, nxt, cfg)
    wp = p.waypoint_index
    assert len(wp) == 4                                       # start, lift, approach, work
    a0, a1 = height_range(p.points_m, wp)
    assert (a0, a1) == (wp[1], wp[2])
    idx = np.arange(len(p.points_m))
    cols = segment_colors(Overlay("SELECTED", path=p.points_m, height_from=a0, height_until=a1), idx)
    white = PATH_COLOR["GENERATED"]
    assert all(c == white for c in cols[:a0]) and all(c == white for c in cols[a1:])   # lift and descent
    assert all(c != white for c in cols[a0:a1])
    # the return path from a low point: lift excluded, transit to ready coloured
    from pinch_sim.paths import return_path
    from pinch_sim.model import ready_q
    m, idx_, _ = scene
    r = return_path(kin.fk(q_low), q_low, kin.fk(ready_q(m)[idx_.qpos]), cfg)
    assert height_range(r.points_m, r.waypoint_index) == (r.waypoint_index[1], r.waypoint_index[2])


def test_peak_marker_tracks_highest_point_in_edit_phase(scene, cfg, data):
    from pinch_sim.overlay import build_overlay, peak_text
    s = _session(scene, cfg, data)
    ov = build_overlay(s, data)
    a0, a1 = ov.height_from, ov.height_until
    assert np.isclose(ov.peak[2], s.path.points_m[a0:a1 + 1, 2].max())
    k = s.path.waypoint_index[1] // 2
    p0 = s.path.points_m[k]
    s.edit_begin(k, p0)
    s.edit_update(p0 + [0.0, 0.0, 0.08])                 # preview during the drag
    ov = build_overlay(s, data)
    assert np.isclose(ov.peak[2], s.path.points_m[ov.height_from:ov.height_until + 1, 2].max())
    assert ov.peak[2] > p0[2] + 0.07 and peak_text(ov) == f"peak z = {ov.peak[2]:.3f} m"
    s.edit_end()
    s.cmd_select()
    s.cmd_validate(data)
    assert s.state == "PASSED" and build_overlay(s, data).peak is None   # results: no height display
