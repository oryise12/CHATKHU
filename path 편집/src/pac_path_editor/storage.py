"""Strict atomic persistence with explicit robot_base/virtual frame metadata."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np

from .core import Config, Correction, PATH_IDS, PathData, apply_corrections, arc_lengths
from .validation import ROBOT_KINDS, ValidationReport, _finite_json, _strict_keys, path_from_dict, path_to_dict, validate_paths


SCHEMA_VERSION = 2
SESSION_KEYS = {"schema_version", "prototype_only", "frame", "units", "config", "originals", "records", "report", "revision"}
CORRECTION_KEYS = {"id", "group_id", "center", "delta", "applies_to", "path_id", "center_xi", "source_fingerprint", "weight_xi", "weight_values", "profile_residual"}


@dataclass
class StoredSession:
    """Loaded original paths and memory; apply records exactly once for display."""

    config: Config
    originals: dict[str, PathData]
    records: list[Correction]
    report: ValidationReport
    revision: int


def _reject_constant(value: str) -> None:
    raise ValueError(f"Non-finite JSON constant: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _read_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = json.load(handle, parse_constant=_reject_constant, object_pairs_hook=_unique_object)
    if not isinstance(data, dict):
        raise ValueError("JSON root must be an object")
    _finite_json(data)
    return data


def load_config(path: str | Path) -> Config:
    """Load configuration with metres, seconds and pixel selection radius."""
    config = Config.from_dict(_read_json(path))
    config.validate()
    return config


def _numeric_array(value: Any, shape: tuple[int, ...] | None = None) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "fiu":
        raise ValueError("Coordinate arrays must contain numbers")
    array = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(array)) or (shape is not None and array.shape != shape):
        raise ValueError("Coordinate array has invalid shape or non-finite values")
    return array


def _read_records(data: Any) -> list[Correction]:
    if not isinstance(data, list):
        raise ValueError("records must be a list")
    result: list[Correction] = []
    ids: set[str] = set()
    for record in data:
        _strict_keys(record, CORRECTION_KEYS, "correction")
        _finite_json(record)
        for key in ("id", "group_id", "source_fingerprint"):
            if not isinstance(record[key], str) or not record[key]:
                raise ValueError(f"Correction {key} must be a nonempty string")
        if record["id"] in ids:
            raise ValueError("Duplicate correction id")
        ids.add(record["id"])
        if record["path_id"] not in PATH_IDS or record["applies_to"] not in ("both", *PATH_IDS):
            raise ValueError("Invalid correction path/mode")
        if record["applies_to"] != "both" and record["path_id"] != record["applies_to"]:
            raise ValueError("Correction path conflicts with its edit mode")
        _numeric_array(record["center"], (3,))
        _numeric_array(record["delta"], (3,))
        center_xi = record["center_xi"]
        if isinstance(center_xi, bool) or not isinstance(center_xi, (float, int)) or not 0 <= center_xi <= 1:
            raise ValueError("Correction center_xi must be within [0, 1]")
        xi = _numeric_array(record["weight_xi"])
        weights = _numeric_array(record["weight_values"])
        if xi.ndim != 1 or len(xi) < 2 or weights.shape != xi.shape or np.any(np.diff(xi) <= 0):
            raise ValueError("Correction weights must use matching, increasing xi arrays")
        if xi[0] != 0 or xi[-1] != 1 or np.any((weights < 0) | (weights > 1)):
            raise ValueError("Correction weights must cover xi [0, 1] with values in [0, 1]")
        if not np.isclose(np.interp(center_xi, xi, weights), 1.0, atol=1e-8, rtol=0):
            raise ValueError("Correction local weight must be one at its center")
        result.append(Correction.from_dict(record))
    return result


def _read_originals(data: Any, config: Config) -> dict[str, PathData]:
    _strict_keys(data, set(PATH_IDS), "originals")
    originals: dict[str, PathData] = {}
    for key in PATH_IDS:
        for name in ("points", "xi", "times"):
            if not isinstance(data[key], dict) or name not in data[key]:
                raise ValueError("Original path is missing an array")
            _numeric_array(data[key][name])
        path = path_from_dict(data[key])
        if path.path_id != key:
            raise ValueError("Original dictionary key differs from path_id")
        expected_endpoints = np.asarray((config.base, config.target) if key == "outbound" else (config.target, config.base), dtype=float)
        if not np.allclose(path.points[[0, -1]], expected_endpoints, rtol=0, atol=config.numeric_tolerance):
            raise ValueError("Original endpoints conflict with configuration")
        distance = arc_lengths(path.points)
        if distance[-1] <= 0:
            raise ValueError("Original path must have positive length")
        if not np.isclose(path.reference_length, path.length, rtol=0, atol=config.numeric_tolerance) or not np.isclose(path.reference_duration, path.times[-1] - path.times[0], rtol=0, atol=config.numeric_tolerance):
            raise ValueError("Original timing references must match original geometry/time")
        if not np.isclose(path.timing_length, path.length, rtol=0, atol=config.numeric_tolerance):
            raise ValueError("Original timing length must match original geometry")
        expected_xi = distance / distance[-1]
        if key == "return":
            expected_xi = 1 - expected_xi
        if not np.allclose(path.xi, expected_xi, rtol=0, atol=config.numeric_tolerance):
            raise ValueError("Original xi must be normalized original geometric arclength")
        originals[key] = path
    return originals


def _check_report(report: ValidationReport, revision: int, originals: dict[str, PathData], records: list[Correction], config: Config) -> None:
    if type(revision) is not int or revision < 0 or report.revision != revision:
        raise ValueError("Validation report is stale or revision is invalid")
    if not report.passed:
        raise ValueError("Only a passing prototype geometry report can be saved or loaded")
    robot_kinds = {check.kind for check in report.checks if check.kind in ROBOT_KINDS}
    if robot_kinds != set(ROBOT_KINDS):
        raise ValueError("All robot validation statuses must be explicitly stored")
    reconstructed = apply_corrections(originals, records, config)
    fresh = validate_paths(originals, reconstructed, records, config, revision)
    if not fresh.passed:
        raise ValueError("Stored geometry fails current validation")
    if set(report.executions) != set(PATH_IDS):
        raise ValueError("Report must include both execution paths")
    for key in PATH_IDS:
        expected, saved = fresh.executions[key], report.executions[key]
        for attribute in ("points", "xi", "times"):
            a, b = getattr(expected, attribute), getattr(saved, attribute)
            if a.shape != b.shape or not np.allclose(a, b, rtol=0, atol=config.numeric_tolerance):
                raise ValueError(f"Report execution {key}.{attribute} does not match the saved correction memory")


def save_session(path: str | Path, config: Config, originals: dict[str, PathData], records: list[Correction], report: ValidationReport, revision: int) -> None:
    """Atomically replace a JSON session after validating its exact revision.

    Original tagged-frame positions plus correction memory are persisted
    as editable geometry. Execution samples (m/s) live inside the report.
    """
    config.validate()
    checked_config = Config.from_dict(config.to_dict())
    checked_originals = _read_originals({key: path_to_dict(value) for key, value in originals.items()}, checked_config)
    checked_records = _read_records([record.to_dict() for record in records])
    checked_report = ValidationReport.from_dict(report.to_dict())
    _check_report(checked_report, revision, checked_originals, checked_records, checked_config)
    data = {"schema_version": SCHEMA_VERSION, "prototype_only": True, "frame": checked_config.frame, "units": {"length": "m", "time": "s"}, "config": checked_config.to_dict(), "originals": {key: path_to_dict(value) for key, value in checked_originals.items()}, "records": [record.to_dict() for record in checked_records], "report": checked_report.to_dict(), "revision": revision}
    payload = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=destination.parent, prefix=f".{destination.name}.", suffix=".tmp", delete=False) as handle:
            temporary = handle.name
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
        temporary = None
    finally:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)


def load_session(path: str | Path, expected_config: Config | None = None) -> StoredSession:
    """Load strictly, rejecting schema, unit, configuration and revision conflicts."""
    data = _read_json(path)
    _strict_keys(data, SESSION_KEYS, "session")
    version = data["schema_version"]
    if type(version) is not int or version != SCHEMA_VERSION:
        if version == 1:
            raise ValueError("Schema 1 requires explicit migrate_v1_session(source, destination)")
        raise ValueError(f"Unsupported session schema_version: {version!r}")
    if data["prototype_only"] is not True or data["frame"] not in ("robot_base", "virtual_edit_frame") or data["units"] != {"length": "m", "time": "s"}:
        raise ValueError("Unsupported prototype status, coordinate frame or units")
    template = Config().to_dict()
    _strict_keys(data["config"], set(template), "stored configuration")
    for section in ("gate", "camera", "colors"):
        _strict_keys(data["config"][section], set(template[section]), f"stored {section} configuration")
    config = Config.from_dict(data["config"])
    if data["frame"] != config.frame:
        raise ValueError("Stored frame conflicts with configuration")
    config.validate()
    if expected_config is not None:
        expected_config.validate()
        if json.dumps(config.to_dict(), sort_keys=True) != json.dumps(expected_config.to_dict(), sort_keys=True):
            raise ValueError("Stored configuration conflicts with the requested configuration")
    originals = _read_originals(data["originals"], config)
    records = _read_records(data["records"])
    report = ValidationReport.from_dict(data["report"])
    revision = data["revision"]
    _check_report(report, revision, originals, records, config)
    return StoredSession(config, originals, records, report, revision)


def migrate_v1_session(source: str | Path, destination: str | Path) -> None:
    """Explicitly migrate schema 1 robot_base data without changing geometry."""
    data = _read_json(source)
    if data.get("schema_version") != 1:
        raise ValueError("Migration source must use schema_version 1")
    _strict_keys(data, SESSION_KEYS, "schema 1 session")
    if data.get("frame") != "robot_base":
        raise ValueError("Schema 1 migration only supports its robot_base frame")
    data["schema_version"] = SCHEMA_VERSION
    data["config"]["frame"] = "robot_base"
    references = {}
    for path_id, path in data["originals"].items():
        points = np.asarray(path["points"], dtype=float)
        times = np.asarray(path["times"], dtype=float)
        references[path_id] = (float(arc_lengths(points)[-1]), float(times[-1] - times[0]))
        path["reference_length"], path["reference_duration"] = references[path_id]
        path["timing_length"] = references[path_id][0]
    for record in data["records"]:
        record["profile_residual"] = [[0.0, 0.0, 0.0] for _ in record["weight_xi"]]
    for path_id, path in data["report"]["executions"].items():
        path["reference_length"], path["reference_duration"] = references[path_id]
        path["timing_length"] = float(arc_lengths(np.asarray(path["points"], dtype=float))[-1])
    payload = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=destination.parent,
                                     prefix=f".{destination.name}.", suffix=".tmp",
                                     delete=False) as handle:
        temporary = Path(handle.name)
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    try:
        load_session(temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
