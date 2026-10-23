"""Editing transactions for explicitly tagged edit-frame paths.

All positions and displacements are three-component vectors in metres in the
configured edit frame. ``xi`` is the immutable original-path correspondence, with
base at zero and target at one; it is never a time fraction.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any
from uuid import uuid4

import numpy as np

from .core import (
    PATH_IDS,
    Config,
    Correction,
    EditMode,
    PathData,
    apply_corrections,
    build_drag_candidate,
    clip_norm,
    copy_records,
    interpolate_at_xi,
    max_corresponding_displacement,
)
from .validation import ValidationReport, validate_paths


def _copy_paths(paths: dict[str, PathData]) -> dict[str, PathData]:
    return {name: path.copy() for name, path in paths.items()}


def _position(value: Any) -> np.ndarray | None:
    """Return an independent finite edit-frame vector with shape (3,)."""
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if result.shape != (3,) or not np.isfinite(result).all():
        return None
    return result.copy()


@dataclass
class DragState:
    """Frozen gesture reference plus live preview in configured frame units.

    ``centers`` holds shape (3,) points on each target path, never hand points.
    ``start_hand``, ``requested_delta`` and ``applied_delta`` have shape (3,).
    The latter is the input after its norm limit; boundary protection and the
    total-field limit may further reduce the visible movement.
    """

    selected_path: str
    center_xi: float
    start_hand: np.ndarray
    centers: dict[str, np.ndarray]
    before_paths: dict[str, PathData]
    start_records: list[Correction]
    candidate_records: list[Correction]
    requested_delta: np.ndarray
    applied_delta: np.ndarray
    group_id: str


@dataclass
class _Snapshot:
    records: list[Correction]
    last_change: float | None


class Editor:
    """Manage committed, editable and gesture-preview correction memories.

    ``originals`` contains independent ``PathData`` objects with positions of
    shape (N, 3) in the configured frame, immutable correspondence xi of shape (N,),
    and time samples of shape (N,) in s. Inputs are copied on construction.
    Path accessors also return independent evaluated arrays.
    """

    def __init__(
        self,
        config: Config,
        originals: dict[str, PathData],
        committed_records: list[Correction] | None = None,
        revision: int = 0,
    ) -> None:
        config.validate()
        if set(originals) != set(PATH_IDS):
            raise ValueError("Original paths must contain outbound and return.")
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise ValueError("Revision must be a nonnegative integer.")
        if any(originals[name].path_id != name for name in PATH_IDS):
            raise ValueError("Original path IDs must match their dictionary keys.")
        self.config = deepcopy(config)
        self.originals = _copy_paths(originals)
        self.committed_records = copy_records(committed_records or [])
        self.candidate_records = copy_records(self.committed_records)
        self.mode: EditMode = "both"
        self.editing = False
        self.revision = revision
        self.validation_report: ValidationReport | None = None
        self.drag: DragState | None = None
        self.last_message = "Ready. Start editing to modify the paths."
        self._undo: list[_Snapshot] = []
        self._redo: list[_Snapshot] = []
        self._last_change: float | None = None

    @property
    def paths(self) -> dict[str, PathData]:
        """Current displayed geometry as independent configured-frame arrays."""
        if self.drag is not None:
            records = self.drag.candidate_records
        elif self.editing:
            records = self.candidate_records
        else:
            records = self.committed_records
        return apply_corrections(self.originals, records, self.config)

    @property
    def committed_paths(self) -> dict[str, PathData]:
        """Return committed geometry as independent configured-frame arrays."""
        return apply_corrections(self.originals, self.committed_records, self.config)

    @property
    def can_undo(self) -> bool:
        return self.editing and self.drag is None and bool(self._undo)

    @property
    def can_redo(self) -> bool:
        return self.editing and self.drag is None and bool(self._redo)

    def _invalidate(self) -> None:
        self.revision += 1
        self.validation_report = None

    def _snapshot(self) -> _Snapshot:
        return _Snapshot(copy_records(self.candidate_records), self._last_change)

    def _restore(self, snapshot: _Snapshot) -> None:
        self.candidate_records = copy_records(snapshot.records)
        self._last_change = snapshot.last_change

    def _idle_edit_required(self) -> bool:
        if not self.editing:
            self.last_message = "Start editing first."
            return False
        if self.drag is not None:
            self.last_message = "Release or cancel the current drag first."
            return False
        return True

    def start_edit(self) -> bool:
        """Begin an edit session from a deep copy of the committed memory."""
        if self.editing:
            self.last_message = "Editing is already active."
            return False
        self.candidate_records = copy_records(self.committed_records)
        self._undo.clear()
        self._redo.clear()
        self._last_change = None
        self.editing = True
        self.last_message = "Editing started. Pinch a projected path point."
        return True

    def set_edit_mode(self, mode: EditMode) -> bool:
        """Choose both, outbound or return; selection stays fixed during drag."""
        if mode not in ("both", *PATH_IDS):
            raise ValueError("Edit mode must be both, outbound or return.")
        if self.drag is not None:
            self.last_message = "Release or cancel the drag before changing mode."
            return False
        self.mode = mode
        self.last_message = f"Editing mode: {mode}."
        return True

    def begin_drag(
        self, path_id: str, xi: float, hand_position: np.ndarray
    ) -> bool:
        """Lock a screen-picked path/xi and finite control position (3,).

        Screen selection belongs to the input adapter. In both mode the second
        center is interpolated at this same original xi without another pick.
        """
        if not self._idle_edit_required():
            return False
        if path_id not in PATH_IDS or self.mode not in ("both", path_id):
            self.last_message = "The selected path is outside the current mode."
            return False
        hand = _position(hand_position)
        try:
            center_xi = float(xi)
        except (TypeError, ValueError, OverflowError):
            center_xi = float("nan")
        if hand is None or not np.isfinite(center_xi) or not 0 <= center_xi <= 1:
            self.last_message = "A finite 3D hand position and valid path point are required."
            return False
        if center_xi in (0., 1.):
            self.last_message = "Exact endpoints are fixed; select a nearby interior point."
            return False
        before = self.paths
        targets = PATH_IDS if self.mode == "both" else (path_id,)
        self.drag = DragState(
            selected_path=path_id,
            center_xi=center_xi,
            start_hand=hand,
            centers={name: interpolate_at_xi(before[name], center_xi).copy() for name in targets},
            before_paths=_copy_paths(before),
            start_records=copy_records(self.candidate_records),
            candidate_records=copy_records(self.candidate_records),
            requested_delta=np.zeros(3),
            applied_delta=np.zeros(3),
            group_id=uuid4().hex,
        )
        self.last_message = "Dragging. Release to keep this gesture; Escape cancels it."
        return True

    def update_drag(self, hand_position: np.ndarray | None) -> bool:
        """Preview total input displacement from gesture start in frame units.

        Missing depth, nonfinite values and wrong shapes freeze the last valid
        preview. Releasing then retains that last valid preview; cancellation
        restores the gesture start. Each update uses the frozen start memory.
        """
        if self.drag is None:
            self.last_message = "No active drag."
            return False
        hand = _position(hand_position)
        if hand is None:
            self.last_message = "Hand/depth input unavailable; keeping the last valid preview."
            return False
        drag = self.drag
        with np.errstate(over="ignore", invalid="ignore"):
            requested = hand - drag.start_hand
        if not np.isfinite(requested).all():
            self.last_message = "Invalid hand movement; keeping the last valid preview."
            return False
        applied = clip_norm(requested, self.config.drag_limit)
        if not np.isfinite(applied).all():
            self.last_message = "Invalid hand movement; keeping the last valid preview."
            return False
        drag.requested_delta = requested.copy()
        if np.array_equal(applied, drag.applied_delta):
            self.last_message = "Dragging; movement is unchanged or limited."
            return True
        # Returning the hand to its origin must exactly restore the start,
        # including memory metadata; zero movement must not trigger absorption.
        if not np.any(applied):
            candidate = copy_records(drag.start_records)
        else:
            candidate = build_drag_candidate(
                self.originals,
                drag.start_records,
                self.config,
                drag.selected_path,
                drag.center_xi,
                requested,
                self.mode,
                drag.group_id,
            )
        drag.candidate_records = copy_records(candidate)
        drag.applied_delta = applied.copy()
        self._invalidate()
        self.last_message = "Preview updated. Release to keep this gesture."
        return True

    def end_drag(self) -> bool:
        """Keep one whole gesture, or reject movement below the threshold in m."""
        if self.drag is None:
            self.last_message = "No active drag."
            return False
        drag = self.drag
        change = max_corresponding_displacement(drag.before_paths, self.paths)
        if not np.isfinite(change) or change + self.config.numeric_tolerance < self.config.min_effective_change:
            self.candidate_records = copy_records(drag.start_records)
            self.drag = None
            if np.any(drag.applied_delta):
                self._invalidate()
            self.last_message = (
                f"Drag rejected: effective movement {change:.6f} m is below "
                f"{self.config.min_effective_change:.6f} m."
            )
            return False
        # A pure no-op is never a history transaction, including when the
        # configured minimum is zero or numeric tolerance exceeds the minimum.
        if change == 0:
            self.candidate_records = copy_records(drag.start_records)
            self.drag = None
            self.last_message = "No effective movement; gesture discarded."
            return False
        self._undo.append(_Snapshot(copy_records(drag.start_records), self._last_change))
        self._redo.clear()
        self.candidate_records = copy_records(drag.candidate_records)
        self._last_change = float(change)
        self.drag = None
        self._invalidate()
        self.last_message = f"Gesture kept ({change:.4f} m maximum movement). Finish editing to validate."
        return True

    def cancel_drag(self) -> bool:
        """Restore the exact memory from before this gesture, without history."""
        if self.drag is None:
            self.last_message = "No active drag."
            return False
        self.candidate_records = copy_records(self.drag.start_records)
        self.drag = None
        self._invalidate()
        self.last_message = "Drag cancelled."
        return True

    def finish_edit(self) -> ValidationReport | None:
        """Validate the current revision and publish only a passing candidate.

        Publication preserves the validated revision because geometry and
        correction content do not change. Saving is the storage/UI layer's job.
        Robot checks remain explicitly unevaluated by the prototype validator.
        """
        if not self._idle_edit_required():
            return None
        report = validate_paths(
            self.originals,
            self.paths,
            self.candidate_records,
            self.config,
            self.revision,
            last_change=self._last_change,
        )
        self.validation_report = report
        if not report.passed:
            self.last_message = "Path checks failed. The candidate remains available for editing."
            return report
        self.committed_records = copy_records(self.candidate_records)
        self.candidate_records = copy_records(self.committed_records)
        self.editing = False
        self._undo.clear()
        self._redo.clear()
        self.last_message = "Path checks passed; robot validation not performed."
        return report

    def cancel_edit(self) -> bool:
        """Discard this session, including an active gesture, to committed state."""
        if not self.editing:
            self.last_message = "No active edit session."
            return False
        self.drag = None
        self.candidate_records = copy_records(self.committed_records)
        self._undo.clear()
        self._redo.clear()
        self._last_change = None
        self.editing = False
        self._invalidate()
        self.last_message = "Editing cancelled; committed paths restored."
        return True

    def undo(self) -> bool:
        """Restore one complete gesture/reset, including all record metadata."""
        if not self._idle_edit_required():
            return False
        if not self._undo:
            self.last_message = "Nothing to undo."
            return False
        self._redo.append(self._snapshot())
        self._restore(self._undo.pop())
        self._invalidate()
        self.last_message = "Undo complete."
        return True

    def redo(self) -> bool:
        """Reapply one complete gesture/reset with independent record arrays."""
        if not self._idle_edit_required():
            return False
        if not self._redo:
            self.last_message = "Nothing to redo."
            return False
        self._undo.append(self._snapshot())
        self._restore(self._redo.pop())
        self._invalidate()
        self.last_message = "Redo complete."
        return True

    def reset(self) -> bool:
        """Clear editable correction memory as one undoable transaction."""
        if not self._idle_edit_required():
            return False
        if not self.candidate_records:
            self.last_message = "Paths are already original."
            return False
        self._undo.append(self._snapshot())
        self._redo.clear()
        self.candidate_records = []
        self._last_change = None
        self._invalidate()
        self.last_message = "Original paths restored. Reset can be undone."
        return True

    def execute_command(self, name: str, **kwargs: Any) -> bool | ValidationReport | None:
        """Shared typed actions for buttons, keys and future voice/hand adapters.

        Drag arguments are configured-frame vectors (3,), never screen pixels.
        Unknown names and invalid argument names raise rather than silently
        turning into different commands.
        """
        commands = {
            "start_edit": self.start_edit,
            "set_edit_mode": self.set_edit_mode,
            "select_mode": self.select_mode,
            "finish_edit": self.finish_edit,
            "cancel_edit": self.cancel_edit,
            "undo": self.undo,
            "redo": self.redo,
            "reset": self.reset,
            "begin_drag": self.begin_drag,
            "update_drag": self.update_drag,
            "end_drag": self.end_drag,
            "cancel_drag": self.cancel_drag,
        }
        if name not in commands:
            raise ValueError(f"Unknown editor command: {name}")
        return commands[name](**kwargs)

    def select_mode(self, mode: EditMode) -> bool:
        """Voice/keyboard selection starts editing if idle, otherwise switches target."""
        if not self.set_edit_mode(mode):
            return False
        return True if self.editing else self.start_edit()
