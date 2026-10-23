"""Adapter: pac_path_editor correction algorithm <-> pinch_sim one-way CartesianPath.

The editor is the repository's own `path 편집/src/pac_path_editor` (branch work/webcam-path-editor,
imported unchanged; it is put on sys.path below). It edits a round-trip pair {outbound, return}
with an immutable correspondence xi (normalized original arclength). Here:

  * the selected one-way path is the editor's `outbound` original; a reversed copy
    fills the required `return` slot and is never edited (edit mode "outbound");
  * editor frame "robot_base" == pinch_sim "world" (= base_link, identity);
  * control points are world positions on the VERTICAL line through the picked point
    (x_i, y_i fixed, 2026-10-11; before that the z plane was fixed), so every drag delta
    is (0, 0, dz). The correction field is a weighted sum of such deltas, hence the x, y
    of every path point are preserved exactly and only heights change; the editor's
    smoothstep endpoint protection keeps the start and end points exactly fixed;
  * the editor's own `finish_edit` / `validate_paths` (geometry-only PASS) is never
    called. Edited paths go through pinch_sim.validation like any other path.

Inputs are world-space events (pick index, control point) so a screen adapter
(mouse now, hand tracking later) only has to produce those.
"""
from __future__ import annotations

import numpy as np

from .frames import WORLD, unit
from .paths import CartesianPath
import sys

from .model import ROOT

_EDITOR_SRC = ROOT.parent / "path 편집" / "src"
if not (_EDITOR_SRC / "pac_path_editor" / "core.py").exists():
    raise ImportError(f"pac_path_editor not found at {_EDITOR_SRC} (expected in this repository)")
if str(_EDITOR_SRC) not in sys.path:
    sys.path.insert(0, str(_EDITOR_SRC))

from pac_path_editor.core import Config as EditorConfig  # noqa: E402
from pac_path_editor.core import PathData, arc_lengths  # noqa: E402
from pac_path_editor.editor import Editor  # noqa: E402

EDITED = "edited"


def _editor_config(path: CartesianPath, ecfg: dict) -> EditorConfig:
    c = EditorConfig(base=tuple(path.points_m[0]), target=tuple(path.points_m[-1]), frame="robot_base")
    for key in ("influence_length", "drag_limit", "total_offset_limit", "endpoint_protection",
                "min_effective_change", "selection_radius_px"):
        setattr(c, key, float(ecfg[key]))
    c.validate()
    return c


def _originals(path: CartesianPath) -> dict:
    P = path.points_m
    s = arc_lengths(P)
    xi = s / s[-1]
    times = np.linspace(0.0, 1.0, len(P))   # placeholder: editor timing is unused, pinch_sim retimes
    return {"outbound": PathData("outbound", P, xi, times),
            "return": PathData("return", P[::-1], 1.0 - xi[::-1], times)}


class PathEdit:
    """Editing session for one generated path (initial or return). Pure 3D, no UI."""

    def __init__(self, base: CartesianPath, ecfg: dict):
        if base.frame != WORLD:
            raise ValueError("only world-frame paths can be edited")
        self.base, self.ecfg = base, ecfg
        self.editor = Editor(_editor_config(base, ecfg), _originals(base))
        self.editor.set_edit_mode("outbound")
        self.editor.start_edit()
        self.xi = self.editor.originals["outbound"].xi.copy()
        self._wp_xi = [float(self.xi[i]) for i in base.waypoint_index]
        self.anchor_xy: np.ndarray | None = None   # x, y of the picked point (fixed during the drag)

    # ------------------------------------------------------------------ geometry
    @property
    def dragging(self) -> bool:
        return self.editor.drag is not None

    @property
    def edited(self) -> bool:
        return bool(self.editor.candidate_records) or self.dragging

    def current_path(self) -> CartesianPath:
        """Displayed geometry (preview during a drag) as a pinch_sim path in world."""
        if not self.edited:
            return self.base
        pd = self.editor.paths["outbound"]
        P, q = pd.points.copy(), pd.xi.copy()
        A = np.array([unit([np.interp(x, self.xi, self.base.tool_axis[:, k]) for k in range(3)]) for x in q])
        wp = tuple(int(np.argmin(np.abs(q - x))) for x in self._wp_xi)
        P[0], P[-1] = self.base.points_m[0], self.base.points_m[-1]   # exact endpoints (already equal)
        return CartesianPath(self.base.path_id, EDITED, WORLD, P, A, wp, self.base.start_q_rad.copy(),
                             dict(self.base.meta, edits=len(self.editor.candidate_records),
                                  base_fingerprint=self.base.fingerprint))

    def selected_point(self) -> np.ndarray | None:
        if not self.dragging:
            return None
        pd = self.editor.paths["outbound"]
        return np.array([np.interp(self.editor.drag.center_xi, pd.xi, pd.points[:, k]) for k in range(3)])

    # ------------------------------------------------------------------ gestures (world space)
    def begin(self, index: int, control_point, free: bool = False) -> bool:
        """Pick displayed point `index` of current_path(); the control point is forced onto its vertical line.

        free=True (voice nudges along a 3-D direction): no vertical constraint, deltas may have x, y.
        """
        path = self.current_path()
        if not 0 < index < len(path.points_m) - 1:
            self.editor.last_message = "Endpoints are fixed; pick an interior point."
            return False
        pd = self.editor.paths["outbound"]
        xi = float(pd.xi[index]) if len(pd.xi) == len(path.points_m) else float(self.xi[index])
        self.anchor_xy = None if free else path.points_m[index, :2].copy()
        return self.editor.begin_drag("outbound", xi, self._on_vertical(control_point))

    def update(self, control_point) -> bool:
        """None (invalid ray / out of workspace) keeps the last valid preview."""
        return self.editor.update_drag(None if control_point is None else self._on_vertical(control_point))

    def end(self) -> bool:
        return self.editor.end_drag()

    def cancel(self) -> bool:
        return self.editor.cancel_drag()

    def undo(self) -> bool:
        return self.editor.undo()

    def redo(self) -> bool:
        return self.editor.redo()

    def reset(self) -> bool:
        return self.editor.reset()

    @property
    def message(self) -> str:
        return self.editor.last_message

    def _on_vertical(self, p) -> np.ndarray:
        p = np.asarray(p, float).copy()
        if self.anchor_xy is not None:
            p[:2] = self.anchor_xy   # x, y of the picked point are fixed: deltas are (0, 0, dz)
        return p
