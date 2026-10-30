"""Geometry and correction memory, independent of UI and robot libraries.

All points/vectors use Config.frame length units; time is in seconds.
Path arrays are (N, 3), correspondence/time arrays are (N,), vectors are (3,).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
import hashlib
from typing import Literal

import numpy as np

PATH_IDS = ("outbound", "return")
PathId = Literal["outbound", "return"]
EditMode = Literal["both", "outbound", "return"]


@dataclass
class GateConfig:
    """Spatial reuse radii in configured frame length units; alpha is dimensionless."""

    mode: str = "anisotropic"
    radius: float = 0.18
    radius_xy: float = 0.22
    radius_z: float = 0.16
    alpha: float = 0.5


@dataclass
class CameraConfig:
    """Matplotlib pinhole camera; pose uses configured frame length units."""

    width: int = 1000
    height: int = 700
    fx: float = 900.0
    fy: float = 900.0
    position: tuple[float, float, float] = (0.75, -1.1, 0.9)
    look_at: tuple[float, float, float] = (0.3, 0.0, 0.16)
    up: tuple[float, float, float] = (0.0, 0.0, 1.0)
    near: float = 0.01


@dataclass
class ColorConfig:
    """Height normalization in metres: fixed workspace or common path range."""

    mode: str = "fixed"
    z_min: float = -0.15
    z_max: float = 0.65
    cmap: str = "viridis"


@dataclass
class Config:
    """Prototype settings; distances in metres except selection radius in pixels."""

    base: tuple[float, float, float] = (0.0, 0.0, 0.12)
    target: tuple[float, float, float] = (0.6, 0.0, 0.12)
    initial_duration: float = 3.0
    sample_dt: float = 0.03
    selection_radius_px: float = 18.0
    influence_length: float = 0.22
    drag_limit: float = 0.15
    total_offset_limit: float = 0.30
    min_effective_change: float = 0.001
    max_ee_speed: float = 0.35
    endpoint_protection: float = 0.08
    numeric_tolerance: float = 1e-8
    mouse_depth_scale: float = 0.015
    bump_height: float = 0.10
    never_shorten: bool = True
    speed_safety: float = 1.01
    gate: GateConfig = field(default_factory=GateConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    colors: ColorConfig = field(default_factory=ColorConfig)
    frame: str = "robot_base"

    def validate(self) -> None:
        """Reject invalid/ambiguous settings before creating metre-valued geometry."""
        if self.frame not in ("robot_base", "virtual_edit_frame"):
            raise ValueError("frame must be robot_base or virtual_edit_frame")
        positive = ("initial_duration", "sample_dt", "selection_radius_px", "influence_length",
                    "drag_limit", "total_offset_limit", "min_effective_change", "max_ee_speed",
                    "endpoint_protection", "numeric_tolerance", "mouse_depth_scale")
        for name in positive:
            _positive(getattr(self, name), name)
        if self.sample_dt > self.initial_duration:
            raise ValueError("sample_dt must not exceed initial_duration")
        if not isinstance(self.never_shorten, bool):
            raise ValueError("never_shorten must be boolean")
        if not _finite_scalar(self.bump_height) or self.bump_height < 0:
            raise ValueError("bump_height must be finite and nonnegative")
        if not _finite_scalar(self.speed_safety) or self.speed_safety < 1:
            raise ValueError("speed_safety must be finite and >= 1")
        for name in ("base", "target"):
            _vector(getattr(self, name), name)
        if np.linalg.norm(np.asarray(self.target) - np.asarray(self.base)) <= self.numeric_tolerance:
            raise ValueError("base and target must differ")
        if self.gate.mode not in ("isotropic", "anisotropic"):
            raise ValueError("gate.mode must be isotropic or anisotropic")
        for name in ("radius", "radius_xy", "radius_z"):
            _positive(getattr(self.gate, name), "gate." + name)
        if not _finite_scalar(self.gate.alpha) or not 0 <= self.gate.alpha < 1:
            raise ValueError("gate.alpha must be in [0, 1)")
        for name in ("width", "height"):
            value = getattr(self.camera, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError("camera dimensions must be positive integers")
        for name in ("fx", "fy", "near"):
            _positive(getattr(self.camera, name), "camera." + name)
        for name in ("position", "look_at", "up"):
            _vector(getattr(self.camera, name), "camera." + name)
        view = np.asarray(self.camera.look_at) - np.asarray(self.camera.position)
        if np.linalg.norm(np.cross(view, self.camera.up)) <= self.numeric_tolerance:
            raise ValueError("camera view and up must form a nondegenerate frame")
        if self.colors.mode not in ("fixed", "path"):
            raise ValueError("colors.mode must be fixed or path")
        if not (_finite_scalar(self.colors.z_min) and _finite_scalar(self.colors.z_max)
                and self.colors.z_min < self.colors.z_max):
            raise ValueError("colors.z_min must be smaller than finite z_max")
        if not isinstance(self.colors.cmap, str) or not self.colors.cmap:
            raise ValueError("colors.cmap must be a nonempty colormap name")

    def to_dict(self) -> dict:
        """Return JSON-compatible settings with explicit metre/second parameters."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> Config:
        """Load strict settings; unknown fields are errors rather than ignored typos."""
        if not isinstance(data, dict):
            raise ValueError("configuration must be a JSON object")
        values = dict(data)
        _check_keys(values, cls)
        for key, nested in (("gate", GateConfig), ("camera", CameraConfig), ("colors", ColorConfig)):
            if key in values:
                _check_keys(values[key], nested)
                values[key] = nested(**values[key])
        config = cls(**values)
        config.validate()
        return config


def _finite_scalar(value: object) -> bool:
    return isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, bool) and bool(np.isfinite(value))


def _positive(value: float, name: str) -> None:
    if not _finite_scalar(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")


def _check_keys(data: dict, cls: type) -> None:
    if not isinstance(data, dict):
        raise ValueError(f"{cls.__name__} must be an object")
    unknown = set(data) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"Unknown {cls.__name__} keys: {sorted(unknown)}")


def _vector(value: object, name: str = "vector") -> np.ndarray:
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite 3D vector") from exc
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite 3D vector")
    return result


@dataclass
class PathData:
    """Execution-order geometry: frame points (N,3), xi (N,), times (N,) s."""

    path_id: PathId
    points: np.ndarray
    xi: np.ndarray
    times: np.ndarray
    reference_length: float | None = None
    reference_duration: float | None = None
    timing_length: float | None = None

    def __post_init__(self) -> None:
        self.points = np.array(self.points, dtype=float, copy=True)
        self.xi = np.array(self.xi, dtype=float, copy=True)
        self.times = np.array(self.times, dtype=float, copy=True)
        if self.reference_length is None:
            self.reference_length = self.length
        if self.reference_duration is None:
            self.reference_duration = float(self.times[-1] - self.times[0])
        if self.timing_length is None:
            self.timing_length = self.length

    def copy(self) -> PathData:
        """Copy all mutable arrays; no aliases to the source path."""
        return PathData(self.path_id, self.points, self.xi, self.times,
                        self.reference_length, self.reference_duration, self.timing_length)

    @property
    def length(self) -> float:
        """Polyline length in metres, independent of execution timing."""
        return float(arc_lengths(self.points)[-1])

    @property
    def fingerprint(self) -> str:
        """Stable source identity of geometry and xi; excludes retimed execution time."""
        digest = hashlib.sha256(self.path_id.encode("utf-8"))
        for values in (self.points, self.xi):
            digest.update(np.asarray(values, dtype="<f8").tobytes())
        return digest.hexdigest()


@dataclass
class Correction:
    """Frozen local raw field. Frame vectors, weights indexed by original xi.

    delta is a residual field coefficient, NOT a gesture displacement limit.
    A record can exceed drag_limit to cancel retained unsaturated contributions.
    """

    id: str
    group_id: str
    center: np.ndarray
    delta: np.ndarray
    applies_to: EditMode
    path_id: PathId
    center_xi: float
    source_fingerprint: str
    weight_xi: np.ndarray
    weight_values: np.ndarray
    # A vector-valued remainder preserves absorbed shapes/directions exactly.
    # Boundary attenuation is frozen at weight_xi before linear interpolation.
    profile_residual: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.center = np.array(self.center, dtype=float, copy=True)
        self.delta = np.array(self.delta, dtype=float, copy=True)
        self.weight_xi = np.array(self.weight_xi, dtype=float, copy=True)
        self.weight_values = np.array(self.weight_values, dtype=float, copy=True)
        self.profile_residual = (np.zeros((len(self.weight_xi), 3))
                                 if self.profile_residual is None else
                                 np.array(self.profile_residual, dtype=float, copy=True))

    def copy(self) -> Correction:
        """Return independent record arrays in configured frame units."""
        return Correction(**{**self.__dict__})

    def to_dict(self) -> dict:
        """Serialize metre-valued raw correction with frozen dimensionless weights."""
        return {name: value.tolist() if isinstance(value, np.ndarray) else value
                for name, value in self.__dict__.items()}

    @classmethod
    def from_dict(cls, data: dict) -> Correction:
        """Parse a record; storage additionally validates its source and full invariants."""
        _check_keys(data, cls)
        try:
            record = cls(**data)
        except (TypeError, ValueError) as exc:
            raise ValueError("Malformed correction record") from exc
        if not isinstance(record.id, str) or not record.id or not isinstance(record.group_id, str) or not record.group_id:
            raise ValueError("Correction id and group_id must be nonempty strings")
        _vector(record.center, "correction center")
        _vector(record.delta, "correction delta")
        if record.path_id not in PATH_IDS or record.applies_to not in ("both", *PATH_IDS):
            raise ValueError("Invalid correction mode/path_id")
        if record.applies_to != "both" and record.applies_to != record.path_id:
            raise ValueError("Single-path correction mode must match path_id")
        if not _finite_scalar(record.center_xi) or not 0 <= record.center_xi <= 1:
            raise ValueError("Correction center_xi must be in [0,1]")
        if not isinstance(record.source_fingerprint, str) or not record.source_fingerprint:
            raise ValueError("Correction source fingerprint is required")
        wx, wv = record.weight_xi, record.weight_values
        if (wx.ndim != 1 or len(wx) < 2 or wv.shape != wx.shape or
                not np.isfinite(wx).all() or not np.isfinite(wv).all() or
                np.any(np.diff(wx) <= 0) or wx[0] != 0 or wx[-1] != 1 or
                np.any(wv < 0) or np.any(wv > 1)):
            raise ValueError("Invalid correction weight profile")
        if not np.isclose(np.interp(record.center_xi, wx, wv), 1., rtol=0, atol=1e-12):
            raise ValueError("Correction weight at center must be one")
        if record.profile_residual.shape != (len(wx), 3) or not np.isfinite(record.profile_residual).all():
            raise ValueError("Invalid vector profile_residual")
        return record


def arc_lengths(points: np.ndarray) -> np.ndarray:
    """Return cumulative polyline arclength (N,) from configured-frame points."""
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 1 or not np.isfinite(p).all():
        raise ValueError("points must be a finite (N,3) array")
    return np.concatenate(([0.], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))))


def min_jerk(u: np.ndarray | float) -> np.ndarray:
    """Dimensionless fifth-order progress for normalized time u, shape preserved."""
    u = np.asarray(u, dtype=float)
    return u ** 3 * (10 + u * (-15 + 6 * u))


def generate_paths(config: Config, demo: str = "straight") -> dict[str, PathData]:
    """Create independent round-trip paths in Config.frame, uniformly timed in s.

    'bump' raises only outbound by bump_height m. xi is initial normalized
    arclength in base->target direction, NOT the min-jerk time progress.
    Initial duration rounds upward to a whole sample interval when necessary.
    """
    config.validate()
    if demo not in ("straight", "bump"):
        raise ValueError("demo must be straight or bump")
    steps = max(2, int(np.ceil(config.initial_duration / config.sample_dt)))
    times = np.arange(steps + 1, dtype=float) * config.sample_dt
    s = min_jerk(times / times[-1])
    base, target = np.asarray(config.base), np.asarray(config.target)
    outbound = base + s[:, None] * (target - base)
    if demo == "bump":
        outbound[:, 2] += config.bump_height * 16 * s ** 2 * (1 - s) ** 2
    returning = target + s[:, None] * (base - target)
    result = {}
    for path_id, points in zip(PATH_IDS, (outbound, returning)):
        length = arc_lengths(points)
        xi = length / length[-1]
        if path_id == "return":
            xi = 1 - xi
        result[path_id] = PathData(path_id, points, xi, times)
    return result


def _interp(values: np.ndarray, xi: np.ndarray, query: np.ndarray | float) -> np.ndarray:
    if xi[0] > xi[-1]:
        xi, values = xi[::-1], values[::-1]
    if values.ndim == 1:
        return np.interp(query, xi, values)
    return np.stack([np.interp(query, xi, values[:, axis]) for axis in range(values.shape[1])], axis=-1)


def interpolate_at_xi(path: PathData, xi: np.ndarray | float) -> np.ndarray:
    """Interpolate configured-frame coordinates at immutable xi."""
    query = np.asarray(xi, dtype=float)
    if not np.isfinite(query).all() or np.any(query < 0) or np.any(query > 1):
        raise ValueError("xi must be finite and in [0,1]")
    return _interp(path.points, path.xi, query)


def clip_norm(vector: np.ndarray, limit: float) -> np.ndarray:
    """Norm saturation in m, preserving direction for vector arrays (...,3)."""
    v = np.asarray(vector, dtype=float)
    _positive(limit, "norm limit")
    if v.ndim < 1 or v.shape[-1] != 3 or not np.isfinite(v).all():
        raise ValueError("vector must be finite with final dimension 3")
    norm = np.linalg.norm(v, axis=-1, keepdims=True)
    return v * np.minimum(1., np.divide(limit, norm, out=np.ones_like(norm), where=norm > 0))


def local_weights(distances: np.ndarray | float, influence_length: float) -> np.ndarray:
    """Finite-support cosine of along-path distance (m), returning weights in [0,1]."""
    _positive(influence_length, "influence_length")
    r = np.abs(np.asarray(distances, dtype=float))
    return np.where(r < influence_length, .5 * (1 + np.cos(np.pi * np.minimum(r / influence_length, 1.))), 0.)


def boundary_weights(xi: np.ndarray | float, original_length: float, protection_length: float) -> np.ndarray:
    """Smoothstep endpoint protection using immutable original arclength (metres)."""
    _positive(protection_length, "protection_length")
    q = np.asarray(xi, dtype=float)
    distance = np.minimum(q, 1 - q) * original_length
    u = np.clip(distance / protection_length, 0., 1.)
    return u * u * (3 - 2 * u)


def _gate_radii(gate: GateConfig) -> np.ndarray:
    if gate.mode == "isotropic":
        return np.full(3, gate.radius)
    if gate.mode == "anisotropic":
        return np.array([gate.radius_xy, gate.radius_xy, gate.radius_z])
    raise ValueError("Invalid gate mode")


def spatial_gate(points: np.ndarray, center: np.ndarray, gate: GateConfig) -> float:
    """Cosine spatial reuse gate from minimum point/segment distance in scaled XYZ.

    Input is a configured-frame (N,3) polyline and (3,) centre. Anisotropic
    distance uses ellipsoid radii; every segment is minimized analytically.
    """
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 1 or not np.isfinite(p).all():
        raise ValueError("gate path must have finite shape (N,3)")
    c = _vector(center, "gate center")
    radii = _gate_radii(gate)
    if not np.isfinite(radii).all() or np.any(radii <= 0) or not 0 <= gate.alpha < 1:
        raise ValueError("Invalid gate radii or alpha")
    q = (p - c) / radii
    if len(q) == 1:
        distance = float(np.linalg.norm(q[0]))
    else:
        segments = q[1:] - q[:-1]
        squared = np.sum(segments * segments, axis=1)
        fraction = np.clip(np.divide(-np.sum(q[:-1] * segments, axis=1), squared,
                                     out=np.zeros_like(squared), where=squared > 0), 0, 1)
        nearest = q[:-1] + fraction[:, None] * segments
        distance = float(np.min(np.linalg.norm(nearest, axis=1)))
    if distance < gate.alpha:
        return 1.
    if distance >= 1:
        return 0.
    return float(.5 * (1 + np.cos(np.pi * (distance - gate.alpha) / (1 - gate.alpha))))


def record_gain(record: Correction, original: PathData, config: Config) -> float:
    """A source-local gesture has unit gain; reuse on other originals is gated."""
    if record.path_id != original.path_id:
        return 0.
    if record.source_fingerprint == original.fingerprint:
        return 1.
    return spatial_gate(original.points, record.center, config.gate)


def raw_field(original: PathData, records: list[Correction], config: Config,
              xi: np.ndarray | float | None = None) -> np.ndarray:
    """Unsaturated sum of frame-local correction contributions, shape (...,3)."""
    query = original.xi if xi is None else np.asarray(xi, dtype=float)
    result = np.zeros(np.shape(query) + (3,), dtype=float)
    for record in records:
        if record.path_id != original.path_id:
            continue
        gamma = record_gain(record, original, config)
        weights = np.interp(query, record.weight_xi, record.weight_values)
        result += gamma * (np.asarray(weights)[..., None] * record.delta +
                           _interp(record.profile_residual, record.weight_xi, query))
    return result


def visible_field(original: PathData, records: list[Correction], config: Config,
                  xi: np.ndarray | float) -> np.ndarray:
    """Sum frozen, boundary-weighted vector polylines before total saturation.

    Freezing each component's knots avoids changing a displayed polyline merely
    by adding another correspondence knot. The new gesture is attenuated with
    the same smooth endpoint weight as before; stored shapes are not refitted.
    """
    result = np.zeros(np.shape(xi) + (3,))
    for record in records:
        if record.path_id != original.path_id:
            continue
        values = record.weight_values[:, None] * record.delta + record.profile_residual
        values = values * boundary_weights(record.weight_xi, original.length,
                                           config.endpoint_protection)[:, None]
        result += record_gain(record, original, config) * _interp(values, record.weight_xi, xi)
    return result


def evaluate_path(original: PathData, records: list[Correction], config: Config,
                  xi: np.ndarray | float | None = None) -> np.ndarray:
    """Reconstruct configured-frame points from original plus correction field."""
    query = original.xi if xi is None else np.asarray(xi, dtype=float)
    base = original.points.copy() if xi is None else interpolate_at_xi(original, query)
    return base + clip_norm(visible_field(original, records, config, query), config.total_offset_limit)


def apply_corrections(originals: dict[str, PathData], records: list[Correction],
                      config: Config) -> dict[str, PathData]:
    """Rebuild geometry once, including centre/support knots for exact correspondence.

    Inserting a correspondence knot interpolates the original xi and time;
    it never redefines xi by the modified arclength. Execution is retimed later.
    """
    result = {}
    for key, path in originals.items():
        profiles = [record.weight_xi for record in records if record.path_id == key]
        query = np.unique(np.concatenate([path.xi, *profiles]))
        if path.xi[0] > path.xi[-1]:
            query = query[::-1]
        result[key] = PathData(key, evaluate_path(path, records, config, query), query,
                               _interp(path.times, path.xi, query),
                               path.reference_length, path.reference_duration)
    return result


def copy_records(records: list[Correction]) -> list[Correction]:
    """Copy memory and metadata without sharing any mutable correction arrays."""
    return [record.copy() for record in records]


def build_drag_candidate(originals: dict[str, PathData], records: list[Correction], config: Config,
                         selected_path: str, center_xi: float, requested_delta: np.ndarray,
                         mode: EditMode, group_id: str) -> list[Correction]:
    """Build actual persistable gesture memory from a FIXED gesture-start snapshot.

    At the same original xi, let U be the current visible offset divided by
    the endpoint weight, K the raw sum of retained records, S norm saturation.
    New coefficient = S(S(U)+drag)-K. At existing geometry knots U=S(raw_all).
    The new source-local record has unit gate/centre weight. Consequently its
    reconstructed pre-boundary field is precisely the bounded target. Boundary
    protection may reduce real movement. Stored residuals are never clipped.
    Support distances (m) use the current gesture-start polyline arclength.
    """
    if selected_path not in PATH_IDS or mode not in ("both", *PATH_IDS):
        raise ValueError("Invalid selection/mode")
    if mode != "both" and selected_path != mode:
        raise ValueError("Selected path is outside current edit mode")
    if not _finite_scalar(center_xi) or not 0 <= center_xi <= 1:
        raise ValueError("center_xi must be in [0,1]")
    drag = clip_norm(_vector(requested_delta, "drag delta"), config.drag_limit)
    if center_xi in (0., 1.) or not np.any(drag):
        return copy_records(records)
    current = apply_corrections(originals, records, config)
    candidate = copy_records(records)
    for path_id in PATH_IDS if mode == "both" else (selected_path,):
        original = originals[path_id]
        # Corresponding centres are evaluated on the shown, piecewise-linear
        # geometry; this is the actual selected point, never the hand position.
        center = interpolate_at_xi(current[path_id], center_xi)
        keep = [record for record in candidate if not (
            record.path_id == path_id and record.applies_to == mode and
            spatial_gate(center[None, :], record.center, config.gate) > 0.)]
        path = current[path_id]
        lengths = arc_lengths(path.points)
        center_length = _interp(lengths, path.xi, center_xi)
        support_s = np.clip([center_length - config.influence_length,
                             center_length + config.influence_length], 0., lengths[-1])
        support_xi = np.interp(support_s, lengths, path.xi)
        profile_xi = np.unique(np.concatenate((path.xi, [center_xi], support_xi)))
        profile_lengths = _interp(lengths, path.xi, profile_xi)
        weights = local_weights(np.abs(profile_lengths - center_length), config.influence_length)
        boundary = boundary_weights(profile_xi, original.length, config.endpoint_protection)
        before_offset = (interpolate_at_xi(path, profile_xi) -
                         interpolate_at_xi(original, profile_xi))
        desired = clip_norm(before_offset + (boundary * weights)[:, None] * drag,
                            config.total_offset_limit)
        remainder = desired - visible_field(original, keep, config, profile_xi)
        coefficients = np.divide(remainder, boundary[:, None],
                                 out=np.zeros_like(remainder), where=boundary[:, None] > 0)
        residual = _interp(coefficients, profile_xi, center_xi)
        keep.append(Correction(
            id=f"{group_id}:{path_id}", group_id=group_id, center=center, delta=residual,
            applies_to=mode, path_id=path_id, center_xi=float(center_xi),
            source_fingerprint=original.fingerprint, weight_xi=profile_xi, weight_values=weights,
            profile_residual=coefficients - weights[:, None] * residual,
        ))
        candidate = keep
    return candidate


def max_corresponding_displacement(before: dict[str, PathData], after: dict[str, PathData]) -> float:
    """Maximum paired displacement in m at the union of immutable xi knots.

    Piecewise-linear paths may have different sample counts. For each interval
    of their common refinement the displacement is affine; its norm has its
    maximum at an interval end, so this comparison is independent of indices.
    """
    if set(before) != set(after):
        raise ValueError("Path sets must agree for displacement comparison")
    maximum = 0.
    for path_id, path in before.items():
        query = np.union1d(path.xi, after[path_id].xi)
        delta = interpolate_at_xi(after[path_id], query) - interpolate_at_xi(path, query)
        maximum = max(maximum, float(np.max(np.linalg.norm(delta, axis=1))))
    return maximum
