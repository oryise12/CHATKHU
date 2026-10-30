"""GUI-free transaction tests; metre values come from the requested scenarios."""

from __future__ import annotations

from copy import deepcopy

import numpy as np
import pytest

from pac_path_editor.core import CameraConfig, ColorConfig, Config, PathData, apply_corrections, generate_paths, interpolate_at_xi
from pac_path_editor.editor import Editor
from pac_path_editor.validation import CheckResult, ValidationReport


def make_editor(demo: str = "straight", **config_kwargs: object) -> Editor:
    config = Config(**config_kwargs)
    return Editor(config, generate_paths(config, demo))


def record_dicts(editor: Editor) -> list[dict]:
    return [record.to_dict() for record in editor.candidate_records]


def gesture(editor: Editor, delta: tuple[float, float, float], *, xi: float = 0.5, path: str = "outbound") -> None:
    hand = np.array([1.0, -2.0, 3.0])
    assert editor.begin_drag(path, xi, hand)
    assert editor.update_drag(hand + delta)
    assert editor.end_drag(), editor.last_message


def assert_same_paths(first: dict, second: dict) -> None:
    for path_id in ("outbound", "return"):
        np.testing.assert_allclose(first[path_id].points, second[path_id].points, atol=1e-12, rtol=0)
        np.testing.assert_array_equal(first[path_id].xi, second[path_id].xi)


def test_event_count_does_not_accumulate_total_displacement() -> None:
    single, many = make_editor(), make_editor()
    start = np.array([0.8, -0.5, 1.2])
    for editor in (single, many):
        assert editor.start_edit()
        assert editor.begin_drag("outbound", 0.5, start)
    assert single.update_drag(start + [0, 0, 0.08])
    for height in np.linspace(0, 0.08, 101):
        assert many.update_drag(start + [0, 0, height])
    assert_same_paths(single.paths, many.paths)
    assert single.end_drag() and many.end_drag()
    assert_same_paths(single.paths, many.paths)
    assert len(many.candidate_records) == 2


def test_same_center_eight_then_three_centimetres_is_eleven() -> None:
    editor = make_editor()
    assert editor.start_edit()
    gesture(editor, (0, 0, 0.08))
    gesture(editor, (0, 0, 0.03))
    for path_id in ("outbound", "return"):
        before = interpolate_at_xi(editor.originals[path_id], 0.5)
        after = interpolate_at_xi(editor.paths[path_id], 0.5)
        np.testing.assert_allclose(after - before, [0, 0, 0.11], atol=1e-12)


def test_input_limit_is_vector_norm_and_total_limit_is_separate() -> None:
    editor = make_editor(drag_limit=0.1, total_offset_limit=0.18)
    editor.start_edit()
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    assert editor.update_drag(np.array([0.12, 0.16, 0]))
    np.testing.assert_allclose(editor.drag.applied_delta, [0.06, 0.08, 0], atol=1e-12)
    assert editor.end_drag()
    for _ in range(5):
        assert editor.begin_drag("outbound", 0.5, np.zeros(3))
        assert editor.update_drag(np.array([0.12, 0.16, 0]))
        editor.end_drag()  # A fully saturated no-op must be rejected.
    for name, path in editor.paths.items():
        offsets = np.linalg.norm(path.points - interpolate_at_xi(editor.originals[name], path.xi), axis=1)
        assert offsets.max() <= 0.18 + 1e-10


def test_outbound_mode_never_moves_return() -> None:
    editor = make_editor()
    editor.start_edit()
    editor.set_edit_mode("outbound")
    before_return = editor.paths["return"].points.copy()
    gesture(editor, (0, 0.03, 0.08))
    np.testing.assert_array_equal(editor.paths["return"].points, before_return)
    assert {(record.applies_to, record.path_id) for record in editor.candidate_records} == {("outbound", "outbound")}


def test_drag_selection_and_mode_are_fixed_until_release() -> None:
    editor = make_editor("bump")
    editor.start_edit()
    assert editor.begin_drag("outbound", 0.31, np.array([8, 9, 10]))
    initial_centers = deepcopy(editor.drag.centers)
    assert not editor.set_edit_mode("return")
    assert not editor.begin_drag("return", 0.9, np.zeros(3))
    assert not editor.undo()
    assert not editor.reset()
    editor.update_drag(np.array([8.03, 9.01, 9.97]))
    assert editor.drag.selected_path == "outbound"
    assert editor.drag.center_xi == 0.31
    assert editor.mode == "both"
    for name in initial_centers:
        np.testing.assert_array_equal(editor.drag.centers[name], initial_centers[name])


def test_both_uses_original_xi_and_independent_current_centers() -> None:
    editor = make_editor("bump")
    editor.start_edit()
    editor.set_edit_mode("outbound")
    gesture(editor, (0, 0, 0.04), xi=0.31)
    editor.set_edit_mode("both")
    before = editor.paths
    assert editor.begin_drag("outbound", 0.31, np.zeros(3))
    assert editor.drag.centers["outbound"][2] > editor.drag.centers["return"][2]
    for name in before:
        np.testing.assert_allclose(editor.drag.centers[name], interpolate_at_xi(before[name], 0.31), atol=1e-12)
    assert editor.update_drag(np.array([0, 0, -0.025]))
    assert editor.end_drag()
    group = editor.candidate_records[-1].group_id
    new_records = [record for record in editor.candidate_records if record.group_id == group]
    assert len(new_records) == 2
    assert {record.center_xi for record in new_records} == {0.31}
    assert {record.path_id for record in new_records} == {"outbound", "return"}
    for name in before:
        movement = interpolate_at_xi(editor.paths[name], 0.31) - interpolate_at_xi(before[name], 0.31)
        np.testing.assert_allclose(movement, [0, 0, -0.025], atol=1e-12)


def test_both_bump_and_straight_receive_same_delta_keep_distinct_shapes() -> None:
    editor = make_editor("bump")
    editor.start_edit()
    before = editor.paths
    gesture(editor, (0, 0, -0.04))
    for name in before:
        np.testing.assert_allclose(
            interpolate_at_xi(editor.paths[name], 0.5) - interpolate_at_xi(before[name], 0.5),
            [0, 0, -0.04], atol=1e-12,
        )
    assert interpolate_at_xi(editor.paths["outbound"], 0.5)[2] > interpolate_at_xi(editor.paths["return"], 0.5)[2]


def test_both_undo_redo_is_atomic_and_preserves_metadata() -> None:
    editor = make_editor("bump")
    editor.start_edit()
    original = editor.paths
    gesture(editor, (0, 0.01, 0.04))
    result, memory = editor.paths, record_dicts(editor)
    assert len(memory) == 2
    assert memory[0]["group_id"] == memory[1]["group_id"]
    assert editor.undo()
    assert editor.candidate_records == []
    assert_same_paths(editor.paths, original)
    assert editor.redo()
    assert_same_paths(editor.paths, result)
    assert record_dicts(editor) == memory


def test_reset_is_one_undoable_operation_and_preserves_committed() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.08))
    assert editor.finish_edit().passed
    committed = editor.committed_paths
    editor.start_edit()
    gesture(editor, (0, 0.02, 0))
    candidate, memory = editor.paths, record_dicts(editor)
    assert editor.reset()
    assert editor.candidate_records == []
    assert_same_paths(editor.paths, editor.originals)
    assert_same_paths(editor.committed_paths, committed)
    assert editor.undo()
    assert_same_paths(editor.paths, candidate)
    assert record_dicts(editor) == memory
    assert editor.redo()
    assert_same_paths(editor.paths, editor.originals)


def test_new_gesture_after_undo_drops_redo_branch() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.04))
    gesture(editor, (0, 0, 0.03))
    assert editor.undo() and editor.can_redo
    gesture(editor, (0, 0.02, 0))
    assert not editor.can_redo
    assert not editor.redo()


def test_rejected_gesture_after_undo_preserves_redo_branch() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.04))
    assert editor.undo() and editor.can_redo
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0, 0, 0.0001]))
    assert not editor.end_drag()
    assert editor.can_redo


def test_candidate_and_preview_never_change_committed_memory() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.08))
    assert editor.finish_edit().passed
    committed = editor.committed_paths
    committed_memory = [record.to_dict() for record in editor.committed_records]
    editor.start_edit()
    for committed_record, candidate_record in zip(editor.committed_records, editor.candidate_records):
        for name in ("center", "delta", "weight_xi", "weight_values"):
            assert not np.shares_memory(getattr(committed_record, name), getattr(candidate_record, name))
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0, 0, 0.03]))
    assert_same_paths(editor.committed_paths, committed)
    assert [record.to_dict() for record in editor.committed_records] == committed_memory
    assert record_dicts(editor) == committed_memory
    assert editor.end_drag()
    assert_same_paths(editor.committed_paths, committed)


def test_original_and_returned_path_arrays_are_independent() -> None:
    config = Config()
    originals = generate_paths(config)
    editor = Editor(config, originals)
    saved = editor.originals["outbound"].points.copy()
    originals["outbound"].points[:] += 99
    np.testing.assert_array_equal(editor.originals["outbound"].points, saved)
    returned = editor.paths
    returned["outbound"].points[:] += 1
    returned["outbound"].xi[:] = 0
    np.testing.assert_array_equal(editor.originals["outbound"].points, saved)
    assert not np.shares_memory(editor.paths["outbound"].points, editor.originals["outbound"].points)


@pytest.mark.parametrize("bad_hand", [None, [0, 0], [0, 0, float("nan")], [0, 0, float("inf")], "missing depth"])
def test_missing_or_invalid_input_freezes_last_valid_preview(bad_hand: object) -> None:
    editor = make_editor()
    editor.start_edit()
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    assert editor.update_drag(np.array([0, 0, 0.05]))
    valid, revision = editor.paths, editor.revision
    assert not editor.update_drag(bad_hand)
    assert_same_paths(editor.paths, valid)
    assert editor.revision == revision
    assert editor.end_drag()
    assert_same_paths(editor.paths, valid)


def test_cancel_drag_restores_exact_pre_gesture_memory() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.05))
    before, memory = editor.paths, record_dicts(editor)
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0.02, 0, 0.03]))
    assert editor.cancel_drag()
    assert_same_paths(editor.paths, before)
    assert record_dicts(editor) == memory
    assert editor.undo()
    assert_same_paths(editor.paths, editor.originals)


def test_return_to_start_does_not_absorb_or_create_history() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.05))
    before, memory = editor.paths, record_dicts(editor)
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0, 0, 0.03]))
    editor.update_drag(np.zeros(3))
    assert_same_paths(editor.paths, before)
    assert not editor.end_drag()
    assert record_dicts(editor) == memory
    assert editor.undo()
    assert not editor.can_undo


def test_small_new_effect_rejected_despite_large_accumulated_offset() -> None:
    editor = make_editor(min_effective_change=0.001)
    editor.start_edit()
    gesture(editor, (0, 0, 0.08))
    before, memory = editor.paths, record_dicts(editor)
    # Keep the large both correction intact: another mode cannot absorb it.
    # Thus this gesture has a genuinely tiny effect everywhere, rather than
    # an absorption-induced change to neighboring curve shape.
    editor.set_edit_mode("outbound")
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0, 0, 0.0002]))
    assert not editor.end_drag()
    assert "rejected" in editor.last_message.lower()
    assert record_dicts(editor) == memory
    assert_same_paths(editor.paths, before)
    assert editor.undo()
    assert not editor.can_undo


def test_endpoint_selection_cannot_create_history_or_move_endpoints() -> None:
    editor = make_editor()
    editor.start_edit()
    before = editor.paths
    memory = record_dicts(editor)
    assert not editor.begin_drag("outbound", 0, np.zeros(3))
    assert editor.drag is None
    assert record_dicts(editor) == memory
    assert_same_paths(editor.paths, before)
    assert not editor.can_undo


def test_preview_end_finish_and_memory_reconstruction_agree() -> None:
    editor = make_editor("bump")
    editor.start_edit()
    gesture(editor, (0, 0, 0.06), xi=0.5)
    assert editor.begin_drag("outbound", 0.44, np.zeros(3))
    editor.update_drag(np.array([0.01, 0.02, 0.02]))
    preview = editor.paths
    preview_records = [record.to_dict() for record in editor.drag.candidate_records]
    assert editor.end_drag()
    assert_same_paths(editor.paths, preview)
    assert record_dicts(editor) == preview_records
    report = editor.finish_edit()
    assert report.passed
    assert_same_paths(editor.committed_paths, preview)
    reconstructed = apply_corrections(editor.originals, editor.committed_records, editor.config)
    assert_same_paths(reconstructed, preview)


def test_new_preview_invalidates_previous_validation_revision() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.05))
    report = editor.finish_edit()
    assert report.passed and report.revision == editor.revision
    editor.start_edit()
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0, 0.03, 0]))
    assert editor.revision > report.revision
    assert editor.validation_report is None
    assert editor.finish_edit() is None
    assert editor.editing and editor.drag is not None


def test_failed_finish_preserves_candidate_and_committed(monkeypatch: pytest.MonkeyPatch) -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.04))
    before = editor.paths
    fail = ValidationReport(editor.revision, [CheckResult("endpoints", "FAIL", "outbound", 0.0, "Test failure", {})], {})
    monkeypatch.setattr("pac_path_editor.editor.validate_paths", lambda *args, **kwargs: fail)
    assert editor.finish_edit() is fail
    assert editor.editing
    assert editor.committed_records == []
    assert_same_paths(editor.paths, before)
    assert editor.validation_report is fail


def test_cancel_edit_discards_all_uncommitted_gestures_and_active_drag() -> None:
    editor = make_editor()
    editor.start_edit()
    gesture(editor, (0, 0, 0.05))
    assert editor.finish_edit().passed
    committed = editor.committed_paths
    editor.start_edit()
    gesture(editor, (0, 0, 0.03))
    assert editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.update_drag(np.array([0, 0.02, 0]))
    assert editor.cancel_edit()
    assert not editor.editing and editor.drag is None
    assert not editor.can_undo and not editor.can_redo
    assert_same_paths(editor.paths, committed)


def test_shared_commands_have_same_state_transitions_and_validate_names() -> None:
    editor = make_editor()
    assert editor.execute_command("start_edit")
    assert editor.execute_command("set_edit_mode", mode="return")
    assert editor.execute_command("begin_drag", path_id="return", xi=0.5, hand_position=np.zeros(3))
    assert editor.execute_command("update_drag", hand_position=np.array([0, 0, 0.04]))
    assert editor.execute_command("end_drag")
    assert editor.execute_command("undo")
    assert editor.execute_command("redo")
    assert editor.execute_command("reset")
    assert editor.execute_command("undo")
    assert editor.execute_command("finish_edit").passed
    with pytest.raises(ValueError, match="Unknown"):
        editor.execute_command("run_robot")


def test_invalid_begin_and_mode_leave_state_unchanged() -> None:
    editor = make_editor()
    assert not editor.begin_drag("outbound", 0.5, np.zeros(3))
    editor.start_edit()
    editor.set_edit_mode("outbound")
    for path, xi, hand in [("return", 0.5, np.zeros(3)), ("outbound", -0.1, np.zeros(3)), ("outbound", 0.5, np.ones(2))]:
        assert not editor.begin_drag(path, xi, hand)
        assert editor.drag is None
    with pytest.raises(ValueError):
        editor.set_edit_mode("invalid")


def test_camera_roundtrip_and_depth_scaled_pixel_motion() -> None:
    from pac_path_editor.viewer import Camera

    camera = Camera(CameraConfig())
    pixels = np.array([[120.0, 80.0], [500.0, 350.0], [830.0, 640.0]])
    depths = np.array([0.25, 1.5, 3.0])
    points = np.stack([camera.unproject(pixel, depth) for pixel, depth in zip(pixels, depths)])
    projected, returned_depth = camera.project(points)
    np.testing.assert_allclose(projected, pixels, atol=1e-10)
    np.testing.assert_allclose(returned_depth, depths, atol=1e-12)
    for depth in (0.5, 2.0):
        p0 = camera.unproject(np.array([500, 350]), depth)
        p1 = camera.unproject(np.array([510, 350]), depth)
        assert np.linalg.norm(p1 - p0) == pytest.approx(10 * depth / camera.config.fx)


def test_camera_points_behind_near_plane_cannot_be_picked() -> None:
    from pac_path_editor.viewer import Camera, pick_point

    camera = Camera(CameraConfig(position=(0, 0, 0), look_at=(0, 0, 1), up=(0, 1, 0)))
    points = np.array([[0, 0, -1.0], [0, 0, 0.005]])
    pixels, _ = camera.project(points)
    assert np.isnan(pixels).all()
    path = PathData("outbound", points, np.array([0.0, 1.0]), np.array([0.0, 1.0]))
    assert pick_point({"outbound": path}, camera, np.array([500, 350]), "both", 18) is None
    with pytest.raises(ValueError):
        camera.unproject(np.array([500, 350]), 0.005)


def test_picker_uses_pixels_even_when_selected_point_is_far_in_3d() -> None:
    from pac_path_editor.viewer import Camera, pick_point

    camera = Camera(CameraConfig(position=(0, 0, 0), look_at=(0, 0, 1), up=(0, 1, 0)))
    cursor = np.array([500.0, 350.0])
    paths = {
        "outbound": PathData("outbound", np.stack([camera.unproject(cursor + [1, 0], 100), camera.unproject(cursor + [100, 0], 100)]), np.array([0.0, 1.0]), np.array([0.0, 1.0])),
        "return": PathData("return", np.stack([camera.unproject(cursor + [3, 0], 0.2), camera.unproject(cursor + [100, 0], 0.2)]), np.array([1.0, 0.0]), np.array([0.0, 1.0])),
    }
    selected = pick_point(paths, camera, cursor, "both", 18)
    assert selected.path_id == "outbound" and selected.index == 0
    assert selected.pixel_distance == pytest.approx(1)
    assert pick_point(paths, camera, cursor, "return", 2) is None
    assert pick_point(paths, camera, cursor, "return", 4).path_id == "return"


def test_picker_deterministic_tie_and_radius_boundary() -> None:
    from pac_path_editor.viewer import Camera, pick_point

    camera = Camera(CameraConfig(position=(0, 0, 0), look_at=(0, 0, 1), up=(0, 1, 0)))
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    paths = {
        "return": PathData("return", points.copy(), np.array([1.0, 0.0]), np.array([0.0, 1.0])),
        "outbound": PathData("outbound", points.copy(), np.array([0.0, 1.0]), np.array([0.0, 1.0])),
    }
    selected = pick_point(paths, camera, np.array([500, 350]), "both", 0)
    assert (selected.path_id, selected.index) == ("outbound", 0)
    assert pick_point(paths, camera, np.array([518, 350]), "both", 18) is not None
    assert pick_point(paths, camera, np.array([518.001, 350]), "both", 18) is None


def test_both_starts_with_only_one_path_inside_selection_radius() -> None:
    from pac_path_editor.viewer import Camera, pick_point

    editor = make_editor("bump")
    editor.start_edit()
    camera = Camera(editor.config.camera)
    paths = editor.paths
    index = int(np.argmin(np.abs(paths["outbound"].xi - 0.5)))
    pixels, depths = camera.project(paths["outbound"].points)
    return_pixels, _ = camera.project(paths["return"].points)
    cursor = pixels[index]
    return_distance = np.min(np.linalg.norm(return_pixels - cursor, axis=1))
    assert return_distance > 1
    radius = return_distance / 3
    assert pick_point(paths, camera, cursor, "return", radius) is None
    selected = pick_point(paths, camera, cursor, "both", radius)
    assert selected.path_id == "outbound"
    assert editor.begin_drag(selected.path_id, selected.xi, camera.unproject(cursor, depths[index]))
    assert set(editor.drag.centers) == {"outbound", "return"}
    for name in paths:
        np.testing.assert_allclose(editor.drag.centers[name], interpolate_at_xi(paths[name], selected.xi), atol=1e-12)


@pytest.mark.parametrize("mode", ["fixed", "path"])
def test_height_colors_share_normalization_across_arrays(mode: str) -> None:
    from pac_path_editor.viewer import height_to_rgba

    normalization = ColorConfig(mode=mode, z_min=-0.15, z_max=0.65)
    before = np.array([-0.1, 0.2, 0.5])
    after = np.array([0.2, 0.3, 0.6])
    bounds = (-0.1, 0.6)
    before_rgba = height_to_rgba(before, normalization, bounds)
    after_rgba = height_to_rgba(after, normalization, bounds)
    assert before_rgba.shape == (3, 4)
    np.testing.assert_array_equal(before_rgba[1], after_rgba[0])
    assert np.isfinite(before_rgba).all()
    assert np.all((before_rgba >= 0) & (before_rgba <= 1))
    if mode == "fixed":
        np.testing.assert_array_equal(before_rgba, height_to_rgba(before, normalization, (-99, 99)))


def test_path_color_bounds_can_stay_frozen_during_large_preview() -> None:
    from pac_path_editor.viewer import height_to_rgba

    config = ColorConfig(mode="path")
    bounds = (0.0, 0.2)
    original_color = height_to_rgba(np.array([0.1]), config, bounds)[0]
    preview_colors = height_to_rgba(np.array([0.1, 0.5]), config, bounds)
    np.testing.assert_array_equal(preview_colors[0], original_color)
    assert not np.array_equal(preview_colors[0], preview_colors[1])


@pytest.fixture
def camera_view(tmp_path):
    import matplotlib
    matplotlib.use('Agg', force=True)
    import matplotlib.pyplot as plt
    from pac_path_editor.viewer import Viewer
    editor = make_editor('bump')
    viewer = Viewer(editor, tmp_path / 'session.json', language='en')
    viewer.figure.canvas.draw()
    yield viewer
    plt.close(viewer.figure)


def _key(viewer, key):
    from matplotlib.backend_bases import KeyEvent
    viewer.figure.canvas.callbacks.process('key_press_event', KeyEvent('key_press_event', viewer.figure.canvas, key=key))


def _mouse(viewer, name, pixel, button=1, step=0):
    from matplotlib.backend_bases import MouseEvent
    x, y = viewer.axes.transData.transform(pixel)
    event = MouseEvent(name, viewer.figure.canvas, x, y, button=button, step=step)
    viewer.figure.canvas.callbacks.process(name, event)


def _central_pixel(viewer):
    points = viewer.editor.paths['outbound']
    index = np.argmin(abs(points.xi - .5))
    return viewer.camera.project(points.points)[0][index]


def test_camera_callbacks_drag_depth_history_save_and_reload(camera_view):
    from pac_path_editor.storage import load_session
    from pac_path_editor.core import apply_corrections
    view = camera_view
    _key(view, 'e')
    pixel = _central_pixel(view)
    _mouse(view, 'button_press_event', pixel)
    assert view.editor.drag is not None
    start_depth = view._depth
    _mouse(view, 'motion_notify_event', pixel + [28, 25])
    _mouse(view, 'scroll_event', pixel + [28, 25], button='up', step=2)
    assert view._depth == pytest.approx(start_depth - 2 * view.editor.config.mouse_depth_scale)
    expected = view.camera.unproject(view._last_pixel, view._depth) - view.editor.drag.start_hand
    np.testing.assert_allclose(view.editor.drag.requested_delta, expected, atol=1e-12)
    preview = view.editor.paths
    _mouse(view, 'button_release_event', pixel + [28, 25])
    assert view.editor.drag is None
    for name in preview:
        np.testing.assert_array_equal(view.editor.paths[name].points, preview[name].points)
    assert not view.save_path.exists()
    _key(view, 'ctrl+z')
    assert view.editor.candidate_records == []
    _key(view, 'ctrl+y')
    _key(view, 'r')
    assert view.editor.candidate_records == []
    _key(view, 'ctrl+z')
    _key(view, 'f')
    assert view.save_path.exists()
    assert view.editor.validation_report.passed
    loaded = load_session(view.save_path, view.editor.config)
    restored = apply_corrections(loaded.originals, loaded.records, loaded.config)
    for name in preview:
        np.testing.assert_array_equal(restored[name].points, preview[name].points)


def test_camera_interruption_cancel_and_locked_mode(camera_view):
    from matplotlib.backend_bases import Event, MouseEvent
    view = camera_view
    _key(view, 'e')
    pixel = _central_pixel(view)
    _mouse(view, 'button_press_event', pixel)
    selected = (view.editor.drag.selected_path, view.editor.drag.center_xi)
    _mouse(view, 'motion_notify_event', pixel + [200, 100])
    _key(view, '2')
    assert view.editor.mode == 'both'
    assert (view.editor.drag.selected_path, view.editor.drag.center_xi) == selected
    preview = view.editor.paths
    event = MouseEvent('motion_notify_event', view.figure.canvas, 0, 0)
    view.figure.canvas.callbacks.process('motion_notify_event', event)
    for name in preview:
        np.testing.assert_array_equal(view.editor.paths[name].points, preview[name].points)
    _key(view, 'f')
    assert view.editor.editing and view.editor.drag is not None
    assert not view.save_path.exists()
    view.figure.canvas.callbacks.process('figure_leave_event', Event('figure_leave_event', view.figure.canvas))
    assert view.editor.drag is None and view.editor.candidate_records == []


def test_failed_mock_report_renders_problem_and_keeps_candidate(camera_view, monkeypatch):
    import pac_path_editor.editor as editor_module
    from pac_path_editor.validation import MockRobotValidator, validate_paths
    view = camera_view
    _key(view, 'e')
    pixel = _central_pixel(view)
    _mouse(view, 'button_press_event', pixel)
    _mouse(view, 'motion_notify_event', pixel + [25, 20])
    _mouse(view, 'button_release_event', pixel + [25, 20])
    candidate = view.editor.paths
    def mock_validation(*args, **kwargs):
        return validate_paths(*args, **kwargs, robot_validator=MockRobotValidator())
    monkeypatch.setattr(editor_module, 'validate_paths', mock_validation)
    _key(view, 'f')
    assert view.editor.editing and not view.editor.validation_report.passed
    assert view.editor.committed_records == []
    assert view.editor.candidate_records and not view.save_path.exists()
    assert any('Simulated check failure' in text.get_text() for text in view.panel.texts)
    for name in candidate:
        np.testing.assert_array_equal(candidate[name].points, view.editor.paths[name].points)
    view.figure.canvas.draw()
