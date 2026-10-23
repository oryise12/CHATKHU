"""Device-free hand/input/overlay and command-authority checks."""
import threading
import time

import numpy as np

from pac_path_editor.core import Config, generate_paths, max_corresponding_displacement
from pac_path_editor.editor import Editor
from pac_path_editor.ros_bridge import EditorCommands
from pac_path_editor.webcam import HandInput, WebcamConfig, hand_to_editor, prepare_frame, synthetic_hand


def setup_input(**updates):
    config = WebcamConfig(debounce_s=0, filter_tau_s=0, **updates)
    editor = Editor(Config(frame='virtual_edit_frame'), generate_paths(Config(frame='virtual_edit_frame')))
    commands = EditorCommands(editor)
    inputs = HandInput(commands, config)
    return editor, commands, inputs, config


def test_mirror_happens_once_and_landmark_cursor_uses_display_direction():
    frame = np.arange(2 * 4 * 3, dtype=np.uint8).reshape(2, 4, 3)
    mirrored = prepare_frame(frame, True)
    np.testing.assert_array_equal(mirrored[:, 0], frame[:, -1])
    np.testing.assert_array_equal(prepare_frame(frame, False), frame)
    # Detection runs after prepare_frame: normalized x grows toward display right.
    a = synthetic_hand((.2, .5))
    b = synthetic_hand((.8, .5))
    from pac_path_editor.webcam import hand_sample
    assert hand_sample(b, 1000, 500)[1][0] > hand_sample(a, 1000, 500)[1][0]


def test_pinch_start_has_zero_preview_jump_and_xyz_axes_map_independently():
    editor, commands, inputs, config = setup_input()
    commands.execute('SELECT_MODE', 'outbound')
    cursor = ((config.start_x + config.end_x) / 2, config.path_y)
    open_hand = synthetic_hand(cursor)
    closed = synthetic_hand(cursor, .01)
    inputs.step(open_hand, 0, config.width, config.height)
    inputs.step(closed, .1, config.width, config.height)
    assert editor.drag is not None
    np.testing.assert_array_equal(editor.drag.requested_delta, np.zeros(3))
    baseline = editor.paths
    moved = closed.copy()
    moved[:, 0] += .02
    moved[:, 1] -= .03
    moved[:, 2] += .04
    inputs.step(moved, .2, config.width, config.height)
    np.testing.assert_allclose(editor.drag.requested_delta,
                               hand_to_editor(np.array([.02, -.03, .04]), config), atol=1e-14)
    assert max_corresponding_displacement(baseline, editor.paths) > 0
    assert np.all(np.abs(editor.drag.requested_delta) > 0)


def test_each_landmark_axis_changes_its_virtual_path_axis():
    from pac_path_editor.core import interpolate_at_xi
    for axis in range(3):
        editor, commands, inputs, config = setup_input()
        commands.execute('SELECT_MODE', 'outbound')
        cursor = ((config.start_x + config.end_x) / 2, config.path_y)
        inputs.step(synthetic_hand(cursor), 0, config.width, config.height)
        closed = synthetic_hand(cursor, .01)
        inputs.step(closed, .1, config.width, config.height)
        moved = closed.copy()
        moved[:, axis] += .03
        before = editor.drag.before_paths['outbound']
        inputs.step(moved, .2, config.width, config.height)
        after = editor.paths['outbound']
        xi = editor.drag.center_xi
        delta = interpolate_at_xi(after, xi) - interpolate_at_xi(before, xi)
        assert abs(delta[axis]) > 0
        np.testing.assert_allclose(np.delete(delta, axis), 0, atol=2e-14)


def test_closed_on_edit_entry_and_outside_pinch_never_auto_select():
    editor, commands, inputs, config = setup_input()
    closed_on_path = synthetic_hand(((config.start_x + config.end_x) / 2, config.path_y), .01)
    inputs.step(closed_on_path, 0, config.width, config.height)
    commands.execute('SELECT_MODE', 'outbound')
    inputs.step(closed_on_path, .1, config.width, config.height)
    assert editor.drag is None
    # Release arms the next pinch. A pinch far away consumes that arm and cannot
    # begin later merely because the still-closed hand approaches the path.
    inputs.step(synthetic_hand((.5, .1)), .2, config.width, config.height)
    inputs.step(synthetic_hand((.5, .1), .01), .3, config.width, config.height)
    assert editor.drag is None
    inputs.step(closed_on_path, .4, config.width, config.height)
    assert editor.drag is None


def test_tracking_loss_freezes_then_cancels_without_commit_or_resume():
    editor, commands, inputs, config = setup_input(loss_timeout_s=.5)
    commands.execute('SELECT_MODE', 'both')
    cursor = ((config.start_x + config.end_x) / 2, config.path_y)
    inputs.step(synthetic_hand(cursor), 0, config.width, config.height)
    inputs.step(synthetic_hand(cursor, .01), .1, config.width, config.height)
    moved = synthetic_hand((cursor[0] + .04, cursor[1]), .01)
    inputs.step(moved, .2, config.width, config.height)
    preview = editor.paths
    inputs.step(None, .3, config.width, config.height)
    assert editor.drag is not None
    assert max_corresponding_displacement(preview, editor.paths) == 0
    inputs.step(moved, .4, config.width, config.height)
    assert editor.drag is not None  # reacquisition alone cannot resume/commit
    inputs.step(None, .9, config.width, config.height)
    assert editor.drag is None
    assert not editor.can_undo


def test_both_uses_fixed_xi_and_one_undo():
    editor, commands, inputs, config = setup_input()
    commands.execute('SELECT_MODE', 'both')
    cursor = ((config.start_x + config.end_x) / 2, config.path_y)
    inputs.step(synthetic_hand(cursor), 0, config.width, config.height)
    inputs.step(synthetic_hand(cursor, .01), .1, config.width, config.height)
    xi = editor.drag.center_xi
    inputs.step(synthetic_hand((cursor[0] + .03, cursor[1] - .04), .01, -.03), .2,
                config.width, config.height)
    inputs.step(synthetic_hand(cursor), .3, config.width, config.height)
    assert editor.drag is None and editor.can_undo
    groups = {record.group_id for record in editor.candidate_records}
    assert len(groups) == 1
    assert {record.path_id for record in editor.candidate_records} == {'outbound', 'return'}
    assert {record.center_xi for record in editor.candidate_records} == {xi}
    assert commands.execute('UNDO').accepted
    assert not editor.candidate_records


def test_command_mode_duplicate_timeout_cancel_and_drag_rejection():
    editor, commands, _, _ = setup_input()
    selected = commands.execute('SELECT_MODE', 'return', 'voice', 'mode-1')
    assert selected.accepted and editor.mode == 'return' and editor.editing
    assert commands.execute('RESET', source='voice', request_id='same').accepted is False
    # Add one gesture directly, then duplicate undo must apply once.
    assert editor.begin_drag('return', .5, np.zeros(3))
    editor.update_drag([0, .03, 0])
    assert editor.end_drag()
    first = commands.execute('UNDO', source='voice', request_id='undo-1')
    revision = editor.revision
    duplicate = commands.execute('UNDO', source='voice', request_id='undo-1')
    assert first.accepted and duplicate == first and editor.revision == revision
    assert editor.redo()
    assert editor.begin_drag('return', .5, np.zeros(3))
    rejected = commands.execute('SELECT_MODE', 'both', 'voice', 'during-drag')
    assert not rejected.accepted and rejected.reason_code == 'DRAG_ACTIVE'
    editor.cancel_drag()
    before = editor.revision
    pending = commands.submit('UNDO', source='voice', request_id='late', deadline_ns=time.time_ns()-1)
    commands.drain()
    assert commands.wait(pending).reason_code == 'TIMEOUT'
    assert editor.revision == before
    # Whole-session cancel is distinct from undo and leaves editing.
    assert commands.execute('CANCEL', source='voice').accepted
    assert not editor.editing


def test_ros_callback_thread_only_queues_until_owner_drains():
    editor, commands, _, _ = setup_input()
    holder = []
    def submit():
        holder.append(commands.submit('SELECT_MODE', 'both', 'voice', 'queued'))
    thread = threading.Thread(target=submit)
    thread.start(); thread.join()
    assert not editor.editing
    commands.drain()
    assert commands.wait(holder[0]).accepted and editor.editing
