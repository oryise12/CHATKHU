"""Persistence, geometry rejection and independent execution speed checks."""

import json

import numpy as np
import pytest

from pac_path_editor.core import Config, PathData, apply_corrections, build_drag_candidate, generate_paths, interpolate_at_xi
from pac_path_editor.storage import load_config, load_session, migrate_v1_session, save_session
from pac_path_editor.validation import MockRobotValidator, ROBOT_KINDS, ValidationReport, retime_path, validate_paths


@pytest.fixture
def edited():
    config = Config()
    originals = generate_paths(config, "bump")
    records = build_drag_candidate(originals, [], config, "outbound", 0.5, np.array([0.0, 0.02, -0.08]), "both", "test-gesture")
    paths = apply_corrections(originals, records, config)
    report = validate_paths(originals, paths, records, config, revision=7, last_change=0.08)
    assert report.passed
    return config, originals, records, paths, report


def test_retime_uses_uniform_time_and_verifies_actual_speed(edited):
    config, originals, records, paths, report = edited
    for path_id, execution in report.executions.items():
        np.testing.assert_allclose(np.diff(execution.times), config.sample_dt, atol=1e-12)
        assert execution.times[-1] >= paths[path_id].times[-1]
        speeds = np.linalg.norm(np.diff(execution.points, axis=0), axis=1) / np.diff(execution.times)
        assert max(speeds) <= config.max_ee_speed
        np.testing.assert_allclose(execution.points, interpolate_at_xi(paths[path_id], execution.xi), atol=1e-10)
        np.testing.assert_array_equal(execution.xi[[0, -1]], paths[path_id].xi[[0, -1]])
        assert (np.diff(execution.xi) > 0).all() if path_id == "outbound" else (np.diff(execution.xi) < 0).all()
        assert not np.shares_memory(execution.points, paths[path_id].points)


def test_retime_noninteger_initial_duration_is_rounded_up():
    config = Config(initial_duration=3.011, sample_dt=0.07, max_ee_speed=0.1)
    original = generate_paths(config)["outbound"]
    execution = retime_path(original, config)
    assert execution.times[-1] >= 1.875 * original.length / config.max_ee_speed
    np.testing.assert_allclose(np.diff(execution.times), 0.07, atol=1e-12)
    assert np.max(np.linalg.norm(np.diff(execution.points, axis=0), axis=1) / 0.07) <= config.max_ee_speed


def test_never_shorten_and_opt_in_shortening():
    config = Config(initial_duration=20)
    path = generate_paths(config)["outbound"]
    assert retime_path(path, config).times[-1] >= path.times[-1]
    config.never_shorten = False
    assert retime_path(path, config).times[-1] < 20


def test_never_shorten_scales_from_immutable_original_length_without_compounding():
    config = Config(sample_dt=.01, max_ee_speed=100.)
    original = generate_paths(config)['outbound']
    longer = original.copy()
    longer.points[len(longer.points)//2, 1] += .3
    longer.timing_length = longer.length
    expected = config.initial_duration * max(1., longer.length / original.length)
    first = retime_path(longer, config)
    second = retime_path(first, config)
    assert first.times[-1] >= expected
    assert first.times[-1] < expected + config.sample_dt + 1e-12
    assert second.times[-1] == first.times[-1]


def test_retime_duplicate_geometry_segments_remains_finite():
    config = Config()
    path = PathData("outbound", np.array([[0, 0, 0], [0, 0, 0], [0.3, 0, 0], [0.6, 0, 0]]), np.array([0, 0.1, 0.5, 1]), np.array([0, 1, 2, 3]))
    result = retime_path(path, config)
    assert np.isfinite(result.points).all()
    assert (np.diff(result.xi) > 0).all()
    np.testing.assert_array_equal(result.xi[[0, -1]], [0, 1])


def test_robot_checks_are_not_pass_and_mock_is_explicit(edited):
    config, originals, records, paths, report = edited
    robot = [check for check in report.checks if check.kind in ROBOT_KINDS]
    assert len(robot) == 6
    assert {check.status for check in robot} == {"NOT_EVALUATED"}
    mocked = validate_paths(originals, paths, records, config, 7, robot_validator=MockRobotValidator())
    assert not mocked.passed
    failure = next(check for check in mocked.checks if check.kind == "self_collision")
    assert failure.details["mock"] is True
    assert "MOCK" in failure.message
    assert failure.path_id == "outbound" and failure.xi == 0.5


def test_empty_or_incomplete_report_is_not_pass():
    assert not ValidationReport(0, [], {}).passed


@pytest.mark.parametrize("damage,kind", [("nan", "finite_geometry"), ("shape", "finite_geometry"), ("endpoint", "endpoints"), ("offset", "total_offset"), ("time", "timing"), ("xi", "correspondence")])
def test_geometry_failures_are_explained_without_crashing(edited, damage, kind):
    config, originals, records, paths, _ = edited
    if damage == "nan":
        paths["outbound"].points[10, 1] = np.nan
    elif damage == "shape":
        paths["outbound"].points = paths["outbound"].points[:, :2]
    elif damage == "endpoint":
        paths["outbound"].points[0, 0] += 0.01
    elif damage == "offset":
        paths["outbound"].points[len(paths["outbound"].points) // 2, 1] += 2
    elif damage == "time":
        paths["outbound"].times[4] = paths["outbound"].times[3]
    else:
        paths["outbound"].xi[4] = paths["outbound"].xi[3]
    report = validate_paths(originals, paths, records, config, 8)
    assert not report.passed
    failure = next(check for check in report.checks if check.kind == kind and check.path_id == "outbound")
    assert failure.status == "FAIL" and failure.message


def test_minimum_change_uses_new_gesture_not_accumulated_offset(edited):
    config, originals, records, paths, _ = edited
    report = validate_paths(originals, paths, records, config, 8, last_change=0.0001)
    assert not report.passed
    failure = next(check for check in report.checks if check.kind == "minimum_change")
    assert failure.status == "FAIL"
    assert failure.details["maximum_change_m"] == 0.0001


def test_session_round_trip_preserves_geometry_and_has_no_double_application(edited, tmp_path):
    config, originals, records, paths, report = edited
    destination = tmp_path / "nested" / "session.json"
    save_session(destination, config, originals, records, report, 7)
    loaded = load_session(destination, expected_config=config)
    reproduced = apply_corrections(loaded.originals, loaded.records, loaded.config)
    for key in paths:
        np.testing.assert_array_equal(reproduced[key].points, paths[key].points)
        np.testing.assert_array_equal(loaded.originals[key].points, originals[key].points)
        np.testing.assert_array_equal(loaded.report.executions[key].points, report.executions[key].points)
        assert not np.shares_memory(loaded.originals[key].points, originals[key].points)
    assert loaded.revision == 7
    assert loaded.report.passed
    data = json.loads(destination.read_text())
    assert data["prototype_only"] is True
    assert data["frame"] == "robot_base"
    assert data["units"] == {"length": "m", "time": "s"}


def test_virtual_frame_round_trip_is_explicit_and_robot_frame_conflict_is_rejected(edited, tmp_path):
    config, originals, records, _, _ = edited
    config.frame = 'virtual_edit_frame'
    paths = apply_corrections(originals, records, config)
    report = validate_paths(originals, paths, records, config, revision=9, last_change=.08)
    destination = tmp_path / 'virtual.json'
    save_session(destination, config, originals, records, report, 9)
    loaded = load_session(destination)
    assert loaded.config.frame == 'virtual_edit_frame'
    with pytest.raises(ValueError, match='conflicts'):
        load_session(destination, expected_config=Config(frame='robot_base'))


def test_save_rejects_stale_revision_or_mismatched_geometry(edited, tmp_path):
    config, originals, records, _, report = edited
    destination = tmp_path / "session.json"
    with pytest.raises(ValueError, match="stale"):
        save_session(destination, config, originals, records, report, 8)
    with pytest.raises(ValueError, match="does not match"):
        save_session(destination, config, originals, [], report, 7)
    assert not destination.exists()


def test_failed_atomic_replace_preserves_previous_file(edited, tmp_path, monkeypatch):
    config, originals, records, _, report = edited
    destination = tmp_path / "session.json"
    save_session(destination, config, originals, records, report, 7)
    previous = destination.read_bytes()

    def fail_replace(*args):
        raise OSError("simulated replace failure")

    monkeypatch.setattr("pac_path_editor.storage.os.replace", fail_replace)
    with pytest.raises(OSError, match="simulated"):
        save_session(destination, config, originals, records, report, 7)
    assert destination.read_bytes() == previous
    assert list(tmp_path.iterdir()) == [destination]


@pytest.mark.parametrize("damage", ["version", "frame", "units", "config", "missing_config", "revision", "record", "original_xi", "report", "robot_status", "unknown"])
def test_load_rejects_invalid_schema_and_invariants(edited, tmp_path, damage):
    config, originals, records, _, report = edited
    destination = tmp_path / "session.json"
    save_session(destination, config, originals, records, report, 7)
    data = json.loads(destination.read_text())
    if damage == "version":
        data["schema_version"] = 1
    elif damage == "frame":
        data["frame"] = "camera"
    elif damage == "units":
        data["units"]["length"] = "mm"
    elif damage == "config":
        data["config"]["bogus"] = 42
    elif damage == "missing_config":
        del data["config"]["influence_length"]
    elif damage == "revision":
        data["revision"] = 8
    elif damage == "record":
        data["records"][0]["weight_values"][0] = -1
    elif damage == "original_xi":
        data["originals"]["outbound"]["xi"][10] += 0.0001
    elif damage == "report":
        data["report"]["executions"]["outbound"]["points"][10][1] += 0.001
    elif damage == "robot_status":
        data["report"]["checks"] = [check for check in data["report"]["checks"] if check["kind"] != "torque"]
    else:
        data["extra"] = 42
    destination.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_session(destination)


def test_explicit_v1_migration_preserves_geometry(edited, tmp_path):
    config, originals, records, paths, report = edited
    current, legacy, migrated = (tmp_path / name for name in ('current.json', 'legacy.json', 'migrated.json'))
    save_session(current, config, originals, records, report, 7)
    data = json.loads(current.read_text())
    data["schema_version"] = 1
    data["config"].pop("frame")
    for path in list(data["originals"].values()) + list(data["report"]["executions"].values()):
        path.pop("reference_length")
        path.pop("reference_duration")
        path.pop("timing_length")
    for record in data["records"]:
        record.pop("profile_residual")
    legacy.write_text(json.dumps(data))
    migrate_v1_session(legacy, migrated)
    loaded = load_session(migrated)
    reproduced = apply_corrections(loaded.originals, loaded.records, loaded.config)
    for name in paths:
        np.testing.assert_allclose(reproduced[name].points, paths[name].points)


def test_load_config_rejects_nonfinite_duplicate_or_unknown_values(tmp_path):
    source = tmp_path / "config.json"
    for text in ('{"sample_dt": NaN}', '{"sample_dt": 0.03, "sample_dt": 0.04}', '{"unknown": 2}'):
        source.write_text(text)
        with pytest.raises(ValueError):
            load_config(source)


def test_config_conflict_is_not_silently_ignored(edited, tmp_path):
    config, originals, records, _, report = edited
    destination = tmp_path / "session.json"
    save_session(destination, config, originals, records, report, 7)
    incompatible = Config(influence_length=0.3)
    with pytest.raises(ValueError, match="conflicts"):
        load_session(destination, expected_config=incompatible)


def test_failed_report_cannot_be_saved(edited, tmp_path):
    config, originals, records, paths, _ = edited
    report = validate_paths(originals, paths, records, config, 7, last_change=0.0001)
    with pytest.raises(ValueError, match="passing"):
        save_session(tmp_path / "failure.json", config, originals, records, report, 7)
