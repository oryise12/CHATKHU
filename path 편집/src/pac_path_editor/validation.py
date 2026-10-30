"""Geometry checks and time sampling in the explicitly configured edit frame."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from .core import Config, Correction, PATH_IDS, PathData, arc_lengths, interpolate_at_xi, min_jerk


STATUSES = frozenset({"PASS", "FAIL", "NOT_EVALUATED"})
GEOMETRY_KINDS = frozenset({"finite_geometry", "correspondence", "endpoints", "total_offset", "timing", "ee_speed"})
ROBOT_KINDS = ("continuous_ik", "joint_limits", "singularity", "self_collision", "joint_dynamics", "torque")


def _strict_keys(data: Any, names: set[str], label: str) -> None:
    if not isinstance(data, dict) or set(data) != names:
        raise ValueError(f"{label} must contain exactly {sorted(names)}")


def _finite_json(value: Any) -> None:
    """Reject non-JSON values, including NaN/Inf at any nesting depth."""
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not np.isfinite(value):
            raise ValueError("JSON numbers must be finite")
        return
    if isinstance(value, list):
        for item in value:
            _finite_json(item)
        return
    if isinstance(value, dict) and all(isinstance(key, str) for key in value):
        for item in value.values():
            _finite_json(item)
        return
    raise ValueError("Expected JSON-compatible data")


def path_to_dict(path: PathData) -> dict[str, Any]:
    """Serialize a tagged-frame path: points (N,3), xi (N,), times (N,) s."""
    return {"path_id": path.path_id, "points": path.points.tolist(), "xi": path.xi.tolist(), "times": path.times.tolist(),
            "reference_length": path.reference_length, "reference_duration": path.reference_duration,
            "timing_length": path.timing_length}


def path_from_dict(data: dict[str, Any]) -> PathData:
    """Read a strictly shaped finite path while preserving original xi."""
    _strict_keys(data, {"path_id", "points", "xi", "times", "reference_length", "reference_duration", "timing_length"}, "path")
    _finite_json(data)
    if data["path_id"] not in PATH_IDS:
        raise ValueError("Invalid path_id")
    try:
        path = PathData(data["path_id"], np.asarray(data["points"], dtype=float), np.asarray(data["xi"], dtype=float), np.asarray(data["times"], dtype=float), data["reference_length"], data["reference_duration"], data["timing_length"])
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid path arrays") from exc
    error = _path_error(path)
    if error:
        raise ValueError(error)
    if any(isinstance(data[k], bool) or not isinstance(data[k], (float, int)) or data[k] <= 0
           for k in ("reference_length", "reference_duration", "timing_length")):
        raise ValueError("Positive original reference length/duration required")
    if not _xi_valid(path, 1e-8):
        raise ValueError("Path xi must run monotonically between the correct endpoints")
    if np.any(np.diff(path.times) <= 0) or abs(float(path.times[0])) > 1e-8:
        raise ValueError("Path times must start at zero and strictly increase")
    return path


@dataclass
class CheckResult:
    """One check; xi identifies an original correspondence, details use SI units."""

    kind: str
    status: str
    path_id: str | None = None
    xi: float | None = None
    message: str = ""
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"kind": self.kind, "status": self.status, "path_id": self.path_id, "xi": self.xi, "message": self.message, "details": self.details.copy()}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CheckResult:
        _strict_keys(data, {"kind", "status", "path_id", "xi", "message", "details"}, "check")
        _finite_json(data)
        if not isinstance(data["kind"], str) or not data["kind"] or data["status"] not in STATUSES:
            raise ValueError("Invalid validation kind/status")
        if data["path_id"] is not None and data["path_id"] not in PATH_IDS:
            raise ValueError("Invalid check path_id")
        if data["xi"] is not None and (isinstance(data["xi"], bool) or not isinstance(data["xi"], (int, float)) or not 0 <= data["xi"] <= 1):
            raise ValueError("Check xi must be within [0, 1]")
        if not isinstance(data["message"], str) or not isinstance(data["details"], dict):
            raise ValueError("Invalid check message/details")
        return cls(**data)


@dataclass
class ValidationReport:
    """A report applies only to its revision; executions have uniform seconds."""

    revision: int
    checks: list[CheckResult]
    executions: dict[str, PathData]

    @property
    def passed(self) -> bool:
        """Prototype geometry passed; this never implies robot execution safety."""
        if any(check.status not in STATUSES or check.status == "FAIL" for check in self.checks):
            return False
        return all(any(check.kind == kind and check.path_id == path_id and check.status == "PASS" for check in self.checks) for path_id in PATH_IDS for kind in GEOMETRY_KINDS)

    def to_dict(self) -> dict[str, Any]:
        return {"revision": self.revision, "checks": [check.to_dict() for check in self.checks], "executions": {key: path_to_dict(path) for key, path in self.executions.items()}}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ValidationReport:
        _strict_keys(data, {"revision", "checks", "executions"}, "report")
        _finite_json(data)
        if type(data["revision"]) is not int or data["revision"] < 0:
            raise ValueError("Report revision must be a nonnegative integer")
        if not isinstance(data["checks"], list) or not isinstance(data["executions"], dict) or not set(data["executions"]).issubset(PATH_IDS):
            raise ValueError("Invalid report collections")
        checks = [CheckResult.from_dict(check) for check in data["checks"]]
        executions = {key: path_from_dict(path) for key, path in data["executions"].items()}
        if any(key != path.path_id for key, path in executions.items()):
            raise ValueError("Execution dictionary key differs from path_id")
        return cls(data["revision"], checks, executions)


class RobotValidator(Protocol):
    """Future adapter consumes sampled paths; never commands hardware."""

    is_mock: bool

    def validate(self, executions: dict[str, PathData], config: Config) -> list[CheckResult]: ...


@dataclass
class MockRobotValidator:
    """Inject a labelled failure for UI tests, without claiming robot validation."""

    failure_kind: str = "self_collision"
    path_id: str = "outbound"
    xi: float = 0.5
    is_mock: bool = field(default=True, init=False)

    def validate(self, executions: dict[str, PathData], config: Config) -> list[CheckResult]:
        if self.failure_kind not in ROBOT_KINDS:
            raise ValueError("Unknown mock robot check")
        return [CheckResult(self.failure_kind, "FAIL", self.path_id, self.xi, "MOCK check: simulated robot validation failure.", {"mock": True})]


def _path_error(path: PathData) -> str | None:
    try:
        if path.path_id not in PATH_IDS:
            return "Unknown path identifier."
        if not isinstance(path.points, np.ndarray) or path.points.ndim != 2 or path.points.shape[1] != 3 or len(path.points) < 2:
            return "Path positions must have shape (N, 3), N >= 2."
        if path.xi.shape != (len(path.points),) or path.times.shape != (len(path.points),):
            return "Path xi and times must have shape (N,)."
        if not all(np.all(np.isfinite(array)) for array in (path.points, path.xi, path.times)):
            return "Path contains missing or non-finite coordinates/times."
    except (AttributeError, TypeError, ValueError):
        return "Path arrays are malformed."
    return None


def _xi_valid(path: PathData, tolerance: float) -> bool:
    increasing = path.path_id == "outbound"
    endpoints = (0.0, 1.0) if increasing else (1.0, 0.0)
    return bool(np.all(path.xi >= -tolerance) and np.all(path.xi <= 1 + tolerance) and np.all(np.diff(path.xi) * (1 if increasing else -1) > 0) and np.allclose(path.xi[[0, -1]], endpoints, atol=tolerance, rtol=0))


def retime_path(path: PathData, config: Config) -> PathData:
    """Resample geometry (N,3) m by min-jerk arclength at sample_dt seconds.

    The analytic peak of ds/du is 1.875. Duration is rounded upward to a
    whole number of intervals; xi is interpolated with the same arclength
    weights as positions. The original geometry is never modified.
    """
    config.validate()
    error = _path_error(path)
    if error:
        raise ValueError(error)
    if not _xi_valid(path, config.numeric_tolerance):
        raise ValueError("Invalid original correspondence coordinates")
    if np.any(np.diff(path.times) <= 0):
        raise ValueError("Input times must strictly increase")
    distances = arc_lengths(path.points)
    geometry_length = float(distances[-1])
    length = float(path.timing_length)
    if path.reference_length <= 0 or path.reference_duration <= 0:
        raise ValueError("Positive immutable timing references required")
    baseline = path.reference_duration * max(1., length / path.reference_length) if config.never_shorten else config.sample_dt
    duration = max(baseline, 1.875 * length * config.speed_safety / config.max_ee_speed)
    count = max(1, int(np.ceil(duration / config.sample_dt)))
    duration = count * config.sample_dt
    times = np.arange(count + 1, dtype=float) * config.sample_dt
    fractions = min_jerk(times / duration)
    if geometry_length == 0:
        points = np.repeat(path.points[:1], count + 1, axis=0)
        xi = path.xi[0] + fractions * (path.xi[-1] - path.xi[0])
    else:
        # Remove zero-length segments only for interpolation; retain final xi.
        keep = np.r_[np.diff(distances) > 0, True]
        interpolation_distances = distances[keep]
        sample_distances = fractions * geometry_length
        points = np.column_stack([np.interp(sample_distances, interpolation_distances, path.points[keep, axis]) for axis in range(3)])
        xi = np.interp(sample_distances, interpolation_distances, path.xi[keep])
    points[[0, -1]] = path.points[[0, -1]]
    xi[[0, -1]] = path.xi[[0, -1]]
    return PathData(path.path_id, points, xi, times, path.reference_length,
                    path.reference_duration, path.timing_length)


def validate_paths(originals: dict[str, PathData], paths: dict[str, PathData], records: list[Correction], config: Config, revision: int, last_change: float | None = None, robot_validator: RobotValidator | None = None) -> ValidationReport:
    """Check candidate paths and construct uniformly timed execution samples.

    last_change is the maximum corresponding-point displacement of the last
    completed gesture, in metres; it is not the cumulative original offset.
    Robot checks remain NOT_EVALUATED unless an explicit adapter supplies them.
    """
    config.validate()
    if type(revision) is not int or revision < 0:
        raise ValueError("revision must be a nonnegative integer")
    checks: list[CheckResult] = []
    executions: dict[str, PathData] = {}
    tolerance = config.numeric_tolerance
    if set(paths) != set(PATH_IDS) or set(originals) != set(PATH_IDS):
        checks.append(CheckResult("path_set", "FAIL", message="Both outbound and return paths are required."))
    for path_id in PATH_IDS:
        path, original = paths.get(path_id), originals.get(path_id)
        error = _path_error(path) or _path_error(original)
        if path is not None and path.path_id != path_id:
            error = "Path identifier does not match its dictionary key."
        checks.append(CheckResult("finite_geometry", "FAIL" if error else "PASS", path_id, message=error or "Coordinates, correspondence and times are finite."))
        if error:
            continue
        correspondence_ok = _xi_valid(path, tolerance) and _xi_valid(original, tolerance)
        checks.append(CheckResult("correspondence", "PASS" if correspondence_ok else "FAIL", path_id, message="Original path correspondence is preserved." if correspondence_ok else "Path correspondence must increase outbound and decrease on return."))
        if not correspondence_ok:
            continue
        endpoint_errors = np.linalg.norm(path.points[[0, -1]] - original.points[[0, -1]], axis=1)
        index = int(np.argmax(endpoint_errors))
        endpoints_ok = float(endpoint_errors[index]) <= tolerance
        checks.append(CheckResult("endpoints", "PASS" if endpoints_ok else "FAIL", path_id, float(path.xi[0 if index == 0 else -1]), "Both path endpoints remain fixed." if endpoints_ok else "A protected endpoint has moved.", {"max_error_m": float(endpoint_errors[index]), "tolerance_m": tolerance}))
        original_points = interpolate_at_xi(original, path.xi)
        offsets = np.linalg.norm(path.points - original_points, axis=1)
        index = int(np.argmax(offsets))
        maximum = float(offsets[index])
        offsets_ok = maximum <= config.total_offset_limit + tolerance
        checks.append(CheckResult("total_offset", "PASS" if offsets_ok else "FAIL", path_id, float(path.xi[index]), "Total displacement is within its limit." if offsets_ok else "The path moved too far from the original.", {"maximum_m": maximum, "limit_m": config.total_offset_limit}))
        if np.any(np.diff(path.times) <= 0) or abs(float(path.times[0])) > tolerance:
            checks.append(CheckResult("timing", "FAIL", path_id, message="Input times must start at zero and strictly increase."))
            continue
        execution = retime_path(path, config)
        executions[path_id] = execution
        intervals = np.diff(execution.times)
        uniform = bool(np.all(intervals > 0) and np.allclose(intervals, config.sample_dt, atol=tolerance, rtol=0))
        checks.append(CheckResult("timing", "PASS" if uniform else "FAIL", path_id, message="Execution uses uniformly spaced, increasing times." if uniform else "Execution timing is invalid.", details={"duration_s": float(execution.times[-1]), "sample_dt_s": config.sample_dt, "geometry_length_m": float(arc_lengths(path.points)[-1]), "sample_count": len(execution.times)}))
        speeds = np.linalg.norm(np.diff(execution.points, axis=0), axis=1) / intervals
        index = int(np.argmax(speeds))
        speed = float(speeds[index])
        speed_ok = speed <= config.max_ee_speed + tolerance
        checks.append(CheckResult("ee_speed", "PASS" if speed_ok else "FAIL", path_id, float((execution.xi[index] + execution.xi[index + 1]) / 2), "Tool motion is within the speed limit." if speed_ok else "The tool moves too quickly.", {"maximum_m_s": speed, "limit_m_s": config.max_ee_speed, "xi_interval": [float(execution.xi[index]), float(execution.xi[index + 1])]}))
    if last_change is not None:
        finite = bool(np.isfinite(last_change))
        minimum_ok = finite and last_change + tolerance >= config.min_effective_change
        checks.append(CheckResult("minimum_change", "PASS" if minimum_ok else "FAIL", message="The last gesture made a visible change." if minimum_ok else "The last gesture changed the path by less than the minimum amount.", details={"maximum_change_m": float(last_change) if finite else None, "minimum_m": config.min_effective_change}))
    robot_checks: list[CheckResult] = []
    if robot_validator is not None:
        mock = bool(getattr(robot_validator, "is_mock", False))
        try:
            supplied = robot_validator.validate({key: path.copy() for key, path in executions.items()}, config)
            for check in supplied:
                check = CheckResult.from_dict(check.to_dict())
                if check.kind not in ROBOT_KINDS:
                    raise ValueError("Robot adapter returned an unknown check kind")
                if mock:
                    check.details["mock"] = True
                    if not check.message.startswith("MOCK"):
                        check.message = "MOCK check: " + check.message
                    if check.status == "PASS":
                        check.status = "NOT_EVALUATED"
                robot_checks.append(check)
        except Exception as exc:
            checks.append(CheckResult("robot_validator", "FAIL", message=f"{'MOCK ' if mock else ''}Robot check adapter failed: {exc}", details={"mock": mock}))
    present = {check.kind for check in robot_checks}
    robot_checks.extend(CheckResult(kind, "NOT_EVALUATED", message="Robot validation has not been performed.", details={"prototype_only": True}) for kind in ROBOT_KINDS if kind not in present)
    return ValidationReport(revision, checks + robot_checks, executions)
