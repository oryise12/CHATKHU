"""Camera-plane prototype UI; geometry uses robot-base metres, picking uses pixels."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import matplotlib
from matplotlib import colors, font_manager
from matplotlib.collections import LineCollection
import numpy as np

from .core import (
    PATH_IDS, CameraConfig, ColorConfig, Config, PathData, arc_lengths,
    generate_paths, interpolate_at_xi, local_weights,
)
from .editor import Editor
from .storage import load_config, load_session, save_session


# Keep presentation language outside the geometry and editing command layers.
UI_TEXT: dict[str, dict[str, str]] = {
    "en": {
        "title": "Shape the next journey",
        "subtitle": "ROUND TRIP PATH EDITOR   /   VIRTUAL CAMERA",
        "view": "WORKBENCH  ·  robot-base coordinates / metres",
        "base": "BASE", "target": "TARGET", "outbound": "Outbound →", "return": "← Return",
        "both": "Both paths", "mode": "EDIT TARGET", "state": "SESSION",
        "editing": "Editing · changes are not saved", "dragging": "Pinch held · live preview",
        "committed": "Confirmed prototype path", "ready": "Start editing to select a path point.",
        "start": "Start edit [E]", "finish": "Finish / save [F]", "cancel": "Cancel edit [C]",
        "undo": "Undo [Ctrl Z]", "redo": "Redo [Ctrl Y]", "reset": "Reset [R]",
        "mode_both": "Both [1]", "mode_out": "Outbound [2]", "mode_return": "Return [3]",
        "help": "Drag: pinch + move    Wheel / PgUp / PgDn: depth    Esc: cancel pinch    I: details",
        "help2": "A release keeps one edit. Finish checks and saves. No robot is connected or started.",
        "height": "Height z (m)", "selection": "SELECTED POINT", "request": "Requested move",
        "actual": "Applied at selected point", "depth": "Virtual hand depth", "current_height": "Current height",
        "normal": "SHARED HEIGHT SCALE", "fixed": "Fixed workspace range", "path": "Path range (held during pinch)",
        "check": "VALIDATION", "unchecked": "Path check pending", "pass": "Path check passed",
        "fail": "Path check failed · keep editing", "robot": "Robot checks not performed",
        "prototype": "PROTOTYPE ONLY · no execution", "save": "SAVE FILE",
        "saved": "Prototype data saved", "save_failed": "File save failed; press F to retry",
        "no_pick": "No path point within the selection radius.",
        "need_edit": "Press E or Start edit before selecting a point.",
        "paused": "Pointer outside image · keeping last valid input.",
        "lost": "Pointer left window · current pinch cancelled.",
        "legend": "Solid: outbound   Dashed: return\nBlack: before pinch   Halo: influence\nBlue dashed: requested   Orange: actual",
        "revision": "Revision", "details": "TECHNICAL DETAILS", "no_details": "Press I for check details.",
        "endpoint": "Endpoint changed", "offset": "Movement limit exceeded", "speed": "Motion is too fast",
        "minimum": "This move is too small", "shape": "Invalid path data", "timing": "Invalid motion timing",
        "mock": "Simulated check failure", "loaded": "Saved prototype loaded", "duration": "EXECUTION TIMING",
        "duration_note": "Preview only · robot checks still required",
    },
    "ko": {
        "title": "다음 왕복 경로를 다듬어 보세요",
        "subtitle": "왕복 경로 편집기   /   가상 카메라",
        "view": "작업대  ·  로봇 기준 좌표 / 미터",
        "base": "출발점", "target": "목표점", "outbound": "가는 길 →", "return": "← 오는 길",
        "both": "두 경로 함께", "mode": "편집 대상", "state": "편집 상태",
        "editing": "편집 중 · 아직 저장하지 않았습니다", "dragging": "핀치 유지 · 수정 미리보기",
        "committed": "확정된 프로토타입 경로", "ready": "편집 시작 후 경로의 점을 선택하세요.",
        "start": "편집 시작 [E]", "finish": "수정 완료 [F]", "cancel": "편집 취소 [C]",
        "undo": "실행 취소 [Ctrl Z]", "redo": "다시 실행 [Ctrl Y]", "reset": "원본 복원 [R]",
        "mode_both": "함께 [1]", "mode_out": "가는 길 [2]", "mode_return": "오는 길 [3]",
        "help": "마우스 드래그: 핀치 + 이동    휠 / PgUp / PgDn: 깊이    Esc: 핀치 취소    I: 상세",
        "help2": "마우스를 놓으면 수정 1회를 기록합니다. 수정 완료 후 검사·저장하며 로봇은 실행하지 않습니다.",
        "height": "높이 z (m)", "selection": "선택한 점", "request": "요청한 이동",
        "actual": "선택점의 실제 이동", "depth": "가상 손의 카메라 깊이", "current_height": "현재 높이",
        "normal": "공통 높이 색상 범위", "fixed": "고정 작업공간 범위", "path": "경로 범위 (핀치 중 고정)",
        "check": "검사 결과", "unchecked": "경로 검사 대기", "pass": "경로 검사 통과",
        "fail": "경로 검사 실패 · 계속 수정하세요", "robot": "로봇 검증 미수행",
        "prototype": "프로토타입 전용 · 실행하지 않음", "save": "저장 파일",
        "saved": "프로토타입 편집 데이터를 저장했습니다", "save_failed": "파일 저장 실패 · F 키로 재시도하세요",
        "no_pick": "선택 반경 안에 경로점이 없습니다.", "need_edit": "E 키 또는 편집 시작 버튼을 누르세요.",
        "paused": "영상 밖으로 이동 · 마지막 유효 입력을 유지합니다.",
        "lost": "창을 벗어나 이번 핀치를 취소했습니다.",
        "legend": "실선: 가는 길   점선: 오는 길\n검은 선: 수정 전   굵은 띠: 영향 구간\n파란 점선: 요청   주황 선: 실제 이동",
        "revision": "수정 번호", "details": "개발 확인 정보", "no_details": "I 키로 검사 상세 정보를 표시합니다.",
        "endpoint": "출발점 또는 목표점이 바뀌었습니다", "offset": "변형 한도를 초과했습니다",
        "speed": "경로 이동 속도가 너무 빠릅니다", "minimum": "이번 이동이 너무 작습니다",
        "shape": "경로 데이터가 올바르지 않습니다", "timing": "실행 시간이 올바르지 않습니다",
        "mock": "모의 검사 실패", "loaded": "저장된 프로토타입을 불러왔습니다", "duration": "실행 시간 계획",
        "duration_note": "계획 확인용 · 로봇 검증은 별도입니다",
    },
}

EDITOR_MESSAGES_KO = {
    "Ready. Start editing to modify the paths.": "편집 시작 후 경로를 수정하세요.",
    "Start editing first.": "먼저 편집을 시작하세요.",
    "Release or cancel the current drag first.": "진행 중인 핀치를 먼저 놓거나 취소하세요.",
    "Editing is already active.": "이미 편집 중입니다.",
    "Editing started. Pinch a projected path point.": "편집을 시작했습니다. 화면의 경로점을 선택하세요.",
    "Release or cancel the drag before changing mode.": "핀치를 놓거나 취소한 후 편집 대상을 바꾸세요.",
    "The selected path is outside the current mode.": "선택한 경로가 현재 편집 대상에 포함되지 않습니다.",
    "A finite 3D hand position and valid path point are required.": "올바른 손 위치와 경로점이 필요합니다.",
    "Dragging. Release to keep this gesture; Escape cancels it.": "핀치 중입니다. 놓으면 기록하고 Esc로 취소합니다.",
    "No active drag.": "진행 중인 핀치가 없습니다.",
    "Hand/depth input unavailable; keeping the last valid preview.": "손 위치 또는 깊이 입력이 없어 마지막 미리보기를 유지합니다.",
    "Invalid hand movement; keeping the last valid preview.": "올바르지 않은 손 입력입니다. 마지막 미리보기를 유지합니다.",
    "Dragging; movement is unchanged or limited.": "핀치 중 · 이동이 같거나 이동 한도에 도달했습니다.",
    "Preview updated. Release to keep this gesture.": "미리보기를 갱신했습니다. 놓으면 이번 수정을 기록합니다.",
    "No effective movement; gesture discarded.": "실제 이동이 없어 이번 핀치를 기록하지 않았습니다.",
    "Drag cancelled.": "이번 핀치를 취소했습니다.",
    "Path checks failed. The candidate remains available for editing.": "경로 검사에 실패했습니다. 현재 경로를 계속 수정하세요.",
    "Path checks passed; robot validation not performed.": "경로 검사 통과 · 로봇 검증 미수행",
    "No active edit session.": "진행 중인 편집이 없습니다.",
    "Editing cancelled; committed paths restored.": "편집을 취소하고 확정 경로로 돌아왔습니다.",
    "Nothing to undo.": "취소할 수정이 없습니다.",
    "Undo complete.": "마지막 수정을 취소했습니다.",
    "Nothing to redo.": "다시 실행할 수정이 없습니다.",
    "Redo complete.": "수정을 다시 적용했습니다.",
    "Paths are already original.": "이미 원본 경로입니다.",
    "Original paths restored. Reset can be undone.": "원본 경로로 복원했습니다. 실행 취소로 되돌릴 수 있습니다.",
}


class Camera:
    """Pinhole camera mapping robot-base metres to top-left-origin image pixels."""

    def __init__(self, config: CameraConfig):
        self.config = config
        self.position = np.asarray(config.position, dtype=float)
        forward = np.asarray(config.look_at, dtype=float) - self.position
        norm = float(np.linalg.norm(forward))
        if norm == 0 or not np.isfinite(norm):
            raise ValueError("Camera position and look_at must differ and be finite")
        forward /= norm
        right = np.cross(forward, np.asarray(config.up, dtype=float))
        norm = float(np.linalg.norm(right))
        if norm < 1e-12 or not np.isfinite(norm):
            raise ValueError("Camera up must not be parallel to its viewing direction")
        right /= norm
        self.rotation = np.stack((right, np.cross(right, forward), forward))
        if config.fx <= 0 or config.fy <= 0 or config.near <= 0:
            raise ValueError("Camera focal lengths and near distance must be positive")

    def project(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Project (N,3) base-frame m to (N,2) pixels and (N,) camera depth m.

        Points at/behind the near plane have NaN pixels, so they cannot be picked.
        """
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        camera_points = (points - self.position) @ self.rotation.T
        depth = camera_points[:, 2]
        pixels = np.full((len(points), 2), np.nan)
        visible = np.isfinite(camera_points).all(axis=1) & (depth > self.config.near)
        pixels[visible, 0] = self.config.width / 2 + self.config.fx * camera_points[visible, 0] / depth[visible]
        pixels[visible, 1] = self.config.height / 2 - self.config.fy * camera_points[visible, 1] / depth[visible]
        return pixels, depth.copy()

    def unproject(self, pixel: np.ndarray, depth: float) -> np.ndarray:
        """Unproject pixel shape (2,) at positive camera-axis depth m to base (3,) m."""
        pixel = np.asarray(pixel, dtype=float)
        if pixel.shape != (2,) or not np.isfinite(pixel).all():
            raise ValueError("pixel must contain two finite coordinates")
        if not np.isfinite(depth) or depth <= self.config.near:
            raise ValueError("depth must be finite and beyond the camera near plane")
        local = np.array([
            (pixel[0] - self.config.width / 2) * depth / self.config.fx,
            -(pixel[1] - self.config.height / 2) * depth / self.config.fy,
            depth,
        ])
        return self.position + local @ self.rotation


@dataclass(frozen=True)
class Selection:
    """Selected execution sample index, immutable original xi, and distance in px."""

    path_id: str
    index: int
    xi: float
    pixel_distance: float


def pick_point(
    paths: dict[str, PathData], camera: Camera, pixel: np.ndarray,
    mode: str, radius_px: float,
) -> Selection | None:
    """Pick by 2D pixel distance only; ties: outbound, return, then sample index."""
    if mode not in ("both", *PATH_IDS):
        raise ValueError("Unknown editing mode")
    pixel = np.asarray(pixel, dtype=float)
    if pixel.shape != (2,) or not np.isfinite(pixel).all():
        return None
    if not np.isfinite(radius_px) or radius_px < 0:
        raise ValueError("Selection radius must be finite and nonnegative pixels")
    best: tuple[float, int, int, str] | None = None
    for priority, path_id in enumerate(PATH_IDS):
        if mode != "both" and mode != path_id:
            continue
        if path_id not in paths:
            continue
        pixels, _ = camera.project(paths[path_id].points)
        distances = np.linalg.norm(pixels - pixel, axis=1)
        indices = np.flatnonzero(np.isfinite(distances) & (distances <= radius_px))
        for index in indices:
            key = (float(distances[index]), priority, int(index), path_id)
            if best is None or key < best:
                best = key
    if best is None:
        return None
    distance, _, index, path_id = best
    return Selection(path_id, index, float(paths[path_id].xi[index]), distance)


def height_to_rgba(
    heights: np.ndarray, normalization: ColorConfig,
    bounds: tuple[float, float] | None = None,
) -> np.ndarray:
    """Convert z heights in m, shape (...,), to (...,4) RGBA without artists.

    Pass common bounds for path-range mode to compare multiple paths/states.
    Fixed mode always uses the configured workspace range.
    """
    values = np.asarray(heights, dtype=float)
    if normalization.mode == "fixed":
        lower, upper = normalization.z_min, normalization.z_max
    elif normalization.mode == "path":
        if bounds is None:
            finite = values[np.isfinite(values)]
            lower, upper = (float(finite.min()), float(finite.max())) if finite.size else (0.0, 1.0)
        else:
            lower, upper = bounds
    else:
        raise ValueError("Height normalization mode must be 'fixed' or 'path'")
    if not np.isfinite([lower, upper]).all() or lower > upper:
        raise ValueError("Height normalization bounds must be finite and increasing")
    if lower == upper:
        lower -= 0.005
        upper += 0.005
    rgba = np.asarray(matplotlib.colormaps[normalization.cmap](colors.Normalize(lower, upper, clip=True)(values)))
    return rgba


def _ui_language(requested: str | None) -> str:
    """Use Korean when a suitable installed font exists; otherwise use English."""
    available = {font.name for font in font_manager.fontManager.ttflist}
    candidates = ("Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Sans CJK SC", "NanumGothic", "Malgun Gothic", "AppleGothic")
    font = next((name for name in candidates if name in available), None)
    if requested == "en" or font is None:
        matplotlib.rcParams["font.family"] = "DejaVu Sans"
        return "en"
    matplotlib.rcParams["font.family"] = font
    matplotlib.rcParams["axes.unicode_minus"] = False
    return "ko"


class Viewer:
    """Matplotlib camera-plane editor; simulated hand events feed base-frame m."""

    def __init__(self, editor: Editor, save_path: str | Path, language: str | None = None):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self.editor = editor
        self.save_path = Path(save_path)
        self.camera = Camera(editor.config.camera)
        self.language = _ui_language(language)
        self.ui = UI_TEXT[self.language]
        # Prevent matplotlib's fullscreen/home/back shortcuts from also running
        # when the shared editing commands F/R/C are invoked.
        for setting in ("keymap.fullscreen", "keymap.home", "keymap.back"):
            matplotlib.rcParams[setting] = [key for key in matplotlib.rcParams[setting] if key not in ("f", "r", "c")]
        self.figure = plt.figure(figsize=(16, 9.5), facecolor="#f3f5f7")
        self.figure.canvas.manager.set_window_title("PAC · Round trip path editor")
        self.axes = self.figure.add_axes((0.035, 0.195, 0.625, 0.705))
        self.color_axes = self.figure.add_axes((0.675, 0.275, 0.012, 0.495))
        self.panel = self.figure.add_axes((0.735, 0.18, 0.245, 0.72))
        self.figure.text(0.035, 0.95, self.ui["title"], size=23, color="#152735", weight="bold")
        self.figure.text(0.035, 0.918, self.ui["subtitle"], size=9, color="#607180")
        self.figure.text(0.98, 0.949, self.ui["prototype"], ha="right", size=10, color="#506474")
        self.figure.text(0.035, 0.037, self.ui["help"], size=10, color="#354959")
        self.figure.text(0.035, 0.014, self.ui["help2"], size=9, color="#6a7987")
        self._buttons: list[Any] = []
        commands = (("start", "start_edit"), ("finish", "finish_edit"), ("undo", "undo"),
                    ("redo", "redo"), ("reset", "reset"), ("cancel", "cancel_edit"))
        for index, (label, command) in enumerate(commands):
            ax = self.figure.add_axes((0.035 + index * 0.108, 0.088, 0.101, 0.044))
            button = Button(ax, self.ui[label], color="#ffffff", hovercolor="#dbeaf1")
            button.label.set_fontsize(9)
            button.on_clicked(lambda event, name=command: self._command(name))
            self._buttons.append(button)
        for index, (label, mode) in enumerate((("mode_both", "both"), ("mode_out", "outbound"), ("mode_return", "return"))):
            ax = self.figure.add_axes((0.735 + index * 0.083, 0.108, 0.079, 0.042))
            button = Button(ax, self.ui[label], color="#e7edf1", hovercolor="#cedfe8")
            button.label.set_fontsize(9)
            button.on_clicked(lambda event, target=mode: self._command("set_edit_mode", mode=target))
            self._buttons.append(button)
        self._last_pixel: np.ndarray | None = None
        self._depth: float | None = None
        self._frozen_bounds: tuple[float, float] | None = None
        self._notice = ""
        self._technical = False
        self._colorbar: Any = None
        canvas = self.figure.canvas
        for name, handler in (("button_press_event", self._on_press), ("motion_notify_event", self._on_move),
                              ("button_release_event", self._on_release), ("scroll_event", self._on_scroll),
                              ("key_press_event", self._on_key), ("figure_leave_event", self._on_leave),
                              ("close_event", self._on_close)):
            canvas.mpl_connect(name, handler)
        self.render()

    def _bounds(self, paths: dict[str, PathData]) -> tuple[float, float]:
        cfg = self.editor.config.colors
        if cfg.mode == "fixed":
            return float(cfg.z_min), float(cfg.z_max)
        if self.editor.drag is not None and self._frozen_bounds is not None:
            return self._frozen_bounds
        # A caller may start and update a gesture before the next paint. Its
        # first paint still freezes the range of the gesture-start paths.
        sets = list(self.editor.drag.before_paths.values()) if self.editor.drag is not None else list(paths.values())
        z = np.concatenate([path.points[:, 2] for path in sets])
        z = z[np.isfinite(z)]
        lo, hi = (float(z.min()), float(z.max())) if z.size else (0.0, 1.0)
        if hi - lo < 1e-12:
            lo, hi = lo - .005, hi + .005
        bounds = (lo, hi)
        if self.editor.drag is not None:
            self._frozen_bounds = bounds
        return bounds

    def _line(self, points: np.ndarray, bounds: tuple[float, float], *, alpha: float = 1,
              width: float = 3.6, path_id: str = "outbound", color: str | None = None,
              zorder: float = 4) -> None:
        pixels, _ = self.camera.project(points)
        segments = np.stack((pixels[:-1], pixels[1:]), axis=1)
        valid = np.isfinite(segments).all(axis=(1, 2))
        if path_id == "return":
            # LineCollection restarts a dash pattern on every tiny segment.
            # Use image arclength to keep a visible dashed return at any sample density.
            lengths = np.nan_to_num(np.linalg.norm(np.diff(pixels, axis=0), axis=1))
            positions = np.cumsum(lengths) - lengths / 2
            valid &= np.mod(positions, 17) < 10
        rgba: Any = color if color is not None else height_to_rgba((points[:-1, 2] + points[1:, 2]) / 2, self.editor.config.colors, bounds)[valid]
        collection = LineCollection(segments[valid], colors=rgba, linewidths=width,
                                    alpha=alpha, linestyles="solid",
                                    zorder=zorder)
        self.axes.add_collection(collection)

    def _workbench(self) -> None:
        ax = self.axes
        cfg = self.camera.config
        ax.set_facecolor("#e8edf0")
        ax.set_xlim(0, cfg.width)
        ax.set_ylim(cfg.height, 0)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#d3dce2")
        # All geometry is projected in true base coordinates; only labels are offset.
        from matplotlib.patches import Polygon
        corner = np.array([[-.25, -.35, 0], [.85, -.35, 0], [.85, .35, 0], [-.25, .35, 0]])
        polygon, _ = self.camera.project(corner)
        if np.isfinite(polygon).all():
            ax.add_patch(Polygon(polygon, facecolor="#f7f7f2", edgecolor="#c0cbd1", linewidth=1.2, zorder=0))
        for x in np.arange(-.2, .81, .1):
            pixels, _ = self.camera.project(np.array([[x, -.35, 0], [x, .35, 0]]))
            ax.plot(pixels[:, 0], pixels[:, 1], color="#dce3e5", lw=.8, zorder=.5)
        for y in np.arange(-.3, .31, .1):
            pixels, _ = self.camera.project(np.array([[-.25, y, 0], [.85, y, 0]]))
            ax.plot(pixels[:, 0], pixels[:, 1], color="#dce3e5", lw=.8, zorder=.5)
        ax.text(.025, .966, self.ui["view"], transform=ax.transAxes, va="top", size=9, color="#536674")
        ax.text(.025, .035, self.ui["legend"], transform=ax.transAxes, va="bottom", size=9,
                color="#546675", linespacing=1.7, bbox=dict(boxstyle="round,pad=.7", facecolor="white", edgecolor="none", alpha=.88))
        origin = np.array([-.14, -.21, .002])
        for axis, color in ((0, "#be665f"), (1, "#5e9a74"), (2, "#638db8")):
            end = origin.copy()
            end[axis] += .075
            p, _ = self.camera.project(np.stack((origin, end)))
            ax.annotate("", xy=p[1], xytext=p[0], arrowprops=dict(arrowstyle="->", color=color, lw=1.5), zorder=2)
            ax.annotate("XYZ"[axis], p[1], xytext=(2, 2), textcoords="offset points", color=color, size=8)

    def _directions(self, path: PathData, alpha: float) -> None:
        n = len(path.points)
        index = max(0, min(n - 2, int(.34 * (n - 1))))
        end = min(n - 1, index + max(2, n // 15))
        p, _ = self.camera.project(path.points[[index, end]])
        if np.isfinite(p).all():
            self.axes.annotate("", xy=p[1], xytext=p[0], arrowprops=dict(arrowstyle="-|>", color="#304656", lw=1.6, alpha=alpha), zorder=7)
            self.axes.annotate(self.ui[path.path_id], p[0], xytext=(0, 15 if path.path_id == "outbound" else -22),
                               textcoords="offset points", size=9, ha="center", color="#304656", alpha=alpha,
                               bbox=dict(facecolor="#ffffff", edgecolor="none", alpha=.78, pad=2), zorder=8)

    def render(self) -> None:
        """Redraw the shared camera view and UI; all path arrays remain unchanged."""
        self.axes.clear()
        self._workbench()
        paths = self.editor.paths
        drag = self.editor.drag
        if drag is None:
            self._frozen_bounds = None
        bounds = self._bounds(paths)
        if drag is not None:
            for path_id, before in drag.before_paths.items():
                if path_id not in drag.centers:
                    continue
                s = arc_lengths(before.points)
                order = np.argsort(before.xi)
                center_s = float(np.interp(drag.center_xi, before.xi[order], s[order]))
                influence = local_weights(np.abs(s - center_s), self.editor.config.influence_length) > 0
                for index in np.flatnonzero(influence[:-1] & influence[1:]):
                    self._line(before.points[index:index + 2], bounds, color="#73bdc6", alpha=.20, width=15, zorder=2)
                self._line(before.points, bounds, color="#101820", alpha=1, width=2.1, path_id=path_id, zorder=3)
        for path_id in PATH_IDS:
            active = self.editor.mode == "both" or self.editor.mode == path_id
            alpha = 1.0 if active else .36
            self._line(paths[path_id].points, bounds, alpha=alpha, path_id=path_id,
                       width=3.8 if path_id == "outbound" else 3.1)
            self._directions(paths[path_id], alpha)
        for label, point in (("base", self.editor.config.base), ("target", self.editor.config.target)):
            pixel, _ = self.camera.project(np.asarray(point, dtype=float)[None, :])
            if np.isfinite(pixel).all():
                self.axes.scatter(*pixel[0], s=48, facecolor="#f8fbfc", edgecolor="#364e5c", linewidth=1.6, zorder=9)
                self.axes.annotate(self.ui[label], pixel[0], xytext=(0, -20), textcoords="offset points", size=9,
                                   weight="bold", color="#344d5b", ha="center", zorder=10)
        if drag is not None:
            self._draw_drag(paths)
        self._draw_problems(paths)
        self.color_axes.clear()
        scalar = matplotlib.cm.ScalarMappable(norm=colors.Normalize(*bounds), cmap=self.editor.config.colors.cmap)
        self._colorbar = self.figure.colorbar(scalar, cax=self.color_axes)
        self._colorbar.set_label(self.ui["height"], color="#506574", size=9)
        self.color_axes.tick_params(labelsize=8, colors="#617583")
        self._colorbar.outline.set_visible(False)
        self._draw_panel(paths)
        self.figure.canvas.draw_idle()

    def _draw_drag(self, paths: dict[str, PathData]) -> None:
        drag = self.editor.drag
        if drag is None:
            return
        for path_id, center in drag.centers.items():
            actual = interpolate_at_xi(paths[path_id], drag.center_xi)
            projected, _ = self.camera.project(np.stack((center, center + drag.requested_delta, actual)))
            if not np.isfinite(projected).all():
                continue
            origin, requested, endpoint = projected
            self.axes.scatter(*origin, marker="o" if path_id == drag.selected_path else "D", s=135,
                              facecolors="#f5c44e", edgecolors="#4c4a3c", linewidths=1.3, zorder=12)
            self.axes.annotate("", xy=requested, xytext=origin,
                               arrowprops=dict(arrowstyle="->", color="#287fb9", lw=1.8, linestyle="dashed"), zorder=13)
            if np.linalg.norm(actual - center - drag.requested_delta) > self.editor.config.numeric_tolerance:
                self.axes.annotate("", xy=endpoint, xytext=origin,
                                   arrowprops=dict(arrowstyle="->", color="#d66b27", lw=2.5), zorder=14)
                self.axes.scatter(*requested, marker="x", s=38, c="#287fb9", zorder=14)
            self.axes.scatter(*endpoint, s=45, facecolor="#fff5ce", edgecolor="#c47d22", linewidth=1.3, zorder=15)

    def _draw_problems(self, paths: dict[str, PathData]) -> None:
        report = self.editor.validation_report
        if report is None or report.revision != self.editor.revision:
            return
        for check in report.checks:
            if check.status != "FAIL" or check.path_id not in paths:
                continue
            path = paths[check.path_id]
            interval = check.details.get("xi_interval")
            if isinstance(interval, (list, tuple)) and len(interval) == 2 and np.isfinite(interval).all():
                lower, upper = sorted(interval)
                inside = path.xi[(path.xi > lower) & (path.xi < upper)]
                xi = np.unique(np.concatenate(([lower], inside, [upper])))
                self._line(interpolate_at_xi(path, xi), self._bounds(paths), color="#d54444", width=8, alpha=.65, zorder=10)
            if check.xi is None:
                self._line(path.points, self._bounds(paths), color="#d54444", width=7, alpha=.35, zorder=9)
                continue
            point = interpolate_at_xi(path, check.xi)
            pixel, _ = self.camera.project(point[None, :])
            if np.isfinite(pixel).all():
                self.axes.scatter(*pixel[0], marker="X", s=100, c="#d83a3a", edgecolor="white", linewidth=1, zorder=20)

    def _friendly_failure(self, check: Any) -> str:
        if check.details.get("mock") or check.message.startswith("MOCK"):
            return self.ui["mock"]
        kind = check.kind.lower()
        for needle, key in (("mock", "mock"), ("endpoint", "endpoint"), ("offset", "offset"), ("speed", "speed"),
                            ("effective", "minimum"), ("minimum", "minimum"), ("time", "timing"), ("shape", "shape"),
                            ("finite", "shape")):
            if needle in kind:
                return self.ui[key]
        return check.message

    def _draw_panel(self, paths: dict[str, PathData]) -> None:
        panel = self.panel
        panel.clear()
        panel.set_axis_off()
        ui = self.ui
        def text(y: float, value: str, size: float = 10, color: str = "#314958", weight: str = "normal") -> None:
            panel.text(0, y, value, va="top", size=size, color=color, weight=weight, transform=panel.transAxes,
                       linespacing=1.5, wrap=True)
        text(1, ui["mode"], 9, "#7b8b97", "bold")
        text(.959, ui[self.editor.mode], 17, "#204251", "bold")
        text(.889, ui["state"], 9, "#7b8b97", "bold")
        state = "dragging" if self.editor.drag is not None else "editing" if self.editor.editing else "committed"
        text(.852, ui[state], 11, "#24596a")
        count = len(self.editor.candidate_records if self.editor.editing else self.editor.committed_records)
        memory = f"수정 기록 {count}개" if self.language == "ko" else f"{count} corrections"
        text(.81, f'{ui["revision"]} {self.editor.revision}   ·   {memory}', 8, "#748794")
        drag = self.editor.drag
        if drag is not None:
            center = drag.centers[drag.selected_path]
            actual = interpolate_at_xi(paths[drag.selected_path], drag.center_xi)
            request = drag.requested_delta * 100
            effective = (actual - center) * 100
            text(.748, f'{ui["selection"]} · {ui[drag.selected_path]}', 9, "#7b8b97", "bold")
            text(.708, f'ξ = {drag.center_xi:.3f}   ·   z = {actual[2]:.3f} m', 10)
            text(.66, f'{ui["request"]} (cm)\n[{request[0]:+.1f}, {request[1]:+.1f}, {request[2]:+.1f}]', 10, "#287fb9")
            text(.572, f'{ui["actual"]} (cm)\n[{effective[0]:+.1f}, {effective[1]:+.1f}, {effective[2]:+.1f}]', 10, "#b35b22")
            if self._depth is not None:
                text(.48, f'{ui["depth"]}: {self._depth:.3f} m', 9, "#748794")
        else:
            text(.748, ui["normal"], 9, "#7b8b97", "bold")
            text(.708, ui[self.editor.config.colors.mode], 10)
            lo, hi = self._bounds(paths)
            text(.669, f'{lo:.3f} — {hi:.3f} m', 10, "#748794")
            text(.611, ui["duration"], 9, "#7b8b97", "bold")
            report = self.editor.validation_report
            execution = report.executions if report is not None and report.revision == self.editor.revision else paths
            text(.571, '  /  '.join(f'{ui[path_id]} {execution[path_id].times[-1]:.2f} s' for path_id in PATH_IDS if path_id in execution), 9)
            text(.528, ui["duration_note"], 8, "#748794")
        text(.434, ui["check"], 9, "#7b8b97", "bold")
        report = self.editor.validation_report
        valid_report = report is not None and report.revision == self.editor.revision
        if valid_report:
            text(.394, ui["pass"] if report.passed else ui["fail"], 11, "#27785b" if report.passed else "#bd4141", "bold")
            failures = [check for check in report.checks if check.status == "FAIL"]
            if failures:
                message = self._friendly_failure(failures[0])
                if failures[0].path_id:
                    message = ui.get(failures[0].path_id, failures[0].path_id) + " · " + message
                text(.32, message[:120], 9, "#a33d3d")
        else:
            text(.394, ui["unchecked"], 11, "#6c8190")
        text(.357, ui["robot"], 9, "#9a7535")
        if self._technical:
            text(.244, ui["details"], 8, "#7b8b97", "bold")
            details = f'records={len(self.editor.candidate_records)}; undo={self.editor.can_undo}; redo={self.editor.can_redo}'
            if valid_report:
                failures = [check for check in report.checks if check.status == "FAIL"]
                if failures:
                    details = f'{failures[0].kind}: {str(failures[0].details)[:95]}'
            text(.21, details, 8)
        else:
            text(.232, ui["save"], 8, "#7b8b97", "bold")
            filename = str(self.save_path)
            text(.20, filename if len(filename) <= 48 else "…" + filename[-47:], 9)
        # The command layer also exposes last_message for external input adapters.
        notice = self._notice or self._editor_notice()
        text(.127, notice[:170], 9, "#345b6b")

    def _editor_notice(self) -> str:
        message = self.editor.last_message
        if self.language == "en":
            return message
        if message.startswith("Editing mode:"):
            return f'편집 대상: {self.ui[self.editor.mode]}'
        if message.startswith("Drag rejected:"):
            return f"이번 이동이 최소 수정량 {self.editor.config.min_effective_change * 1000:.1f} mm보다 작아 취소했습니다."
        if message.startswith("Gesture kept ("):
            return "이번 수정을 기록했습니다. 수정 완료를 누르면 전체 경로를 검사합니다."
        return EDITOR_MESSAGES_KO.get(message, message)

    def _save(self) -> None:
        report = self.editor.validation_report
        if report is None or not report.passed or report.revision != self.editor.revision:
            return
        try:
            save_session(self.save_path, self.editor.config, self.editor.originals,
                         self.editor.committed_records, report, self.editor.revision)
            self._notice = self.ui["saved"]
        except (OSError, ValueError) as exc:
            self._notice = f'{self.ui["save_failed"]}: {exc}'

    def _command(self, command: str, **kwargs: Any) -> None:
        self._notice = ""
        if command == "finish_edit" and not self.editor.editing:
            self._save()
        else:
            self.editor.execute_command(command, **kwargs)
            if command == "finish_edit" and not self.editor.editing:
                self._save()
        if self.editor.drag is None:
            self._last_pixel = None
            self._depth = None
            self._frozen_bounds = None
        self.render()

    def _on_press(self, event: Any) -> None:
        if event.inaxes is not self.axes or event.button != 1 or self.editor.drag is not None:
            return
        if not self.editor.editing:
            self._notice = self.ui["need_edit"]
            self.render()
            return
        pixel = np.array([event.xdata, event.ydata], dtype=float)
        paths = self.editor.paths
        picked = pick_point(paths, self.camera, pixel, self.editor.mode, self.editor.config.selection_radius_px)
        if picked is None:
            self._notice = self.ui["no_pick"]
            self.render()
            return
        _, depth = self.camera.project(paths[picked.path_id].points[picked.index:picked.index + 1])
        self._depth = float(depth[0])
        self._last_pixel = pixel
        # Freeze the common start range before any movement is applied.
        self._frozen_bounds = self._bounds(paths)
        hand = self.camera.unproject(pixel, self._depth)
        self.editor.begin_drag(picked.path_id, picked.xi, hand)
        self._notice = ""
        self.render()

    def _update_hand(self) -> None:
        if self._last_pixel is None or self._depth is None:
            self.editor.update_drag(None)
            return
        try:
            hand = self.camera.unproject(self._last_pixel, self._depth)
        except ValueError:
            self.editor.update_drag(None)
        else:
            self.editor.update_drag(hand)
            self._notice = ""

    def _on_move(self, event: Any) -> None:
        if self.editor.drag is None:
            return
        if event.inaxes is not self.axes or event.xdata is None or event.ydata is None:
            self.editor.update_drag(None)
            self._notice = self.ui["paused"]
        else:
            self._last_pixel = np.array([event.xdata, event.ydata], dtype=float)
            self._update_hand()
        self.render()

    def _on_release(self, event: Any) -> None:
        if self.editor.drag is not None and event.button == 1:
            if event.inaxes is self.axes and event.xdata is not None and event.ydata is not None:
                self._last_pixel = np.array([event.xdata, event.ydata], dtype=float)
                self._update_hand()
            self.editor.end_drag()
            self._depth = None
            self._last_pixel = None
            self._notice = ""
            self.render()

    def _change_depth(self, steps: float) -> None:
        if self.editor.drag is None or self._depth is None:
            return
        self._depth = max(self.camera.config.near * 1.01,
                          self._depth - steps * self.editor.config.mouse_depth_scale)
        self._update_hand()
        self.render()

    def _on_scroll(self, event: Any) -> None:
        if event.inaxes is self.axes:
            self._change_depth(float(event.step))

    def _on_key(self, event: Any) -> None:
        key = (event.key or "").lower()
        if key in ("1", "2", "3"):
            self._command("set_edit_mode", mode={"1": "both", "2": "outbound", "3": "return"}[key])
        elif key in ("e", "f", "c", "r", "ctrl+z", "ctrl+y", "cmd+z", "cmd+y"):
            self._command({"e": "start_edit", "f": "finish_edit", "c": "cancel_edit", "r": "reset",
                           "ctrl+z": "undo", "ctrl+y": "redo", "cmd+z": "undo", "cmd+y": "redo"}[key])
        elif key == "escape" and self.editor.drag is not None:
            self.editor.cancel_drag()
            self._notice = ""
            self._depth = None
            self._last_pixel = None
            self.render()
        elif key in ("pageup", "pagedown"):
            self._change_depth(1 if key == "pageup" else -1)
        elif key == "i":
            self._technical = not self._technical
            self.render()

    def _on_leave(self, event: Any) -> None:
        if self.editor.drag is not None:
            self.editor.cancel_drag()
            self._depth = None
            self._last_pixel = None
            self._notice = self.ui["lost"]
            self.render()

    def _on_close(self, event: Any) -> None:
        if self.editor.drag is not None:
            self.editor.cancel_drag()

    def show(self) -> None:
        """Show the interactive window using the selected matplotlib GUI backend."""
        import matplotlib.pyplot as plt
        plt.show()


def main(argv: list[str] | None = None) -> int:
    """CLI entry point; headless mode uses Agg and can export a PNG screenshot."""
    parser = argparse.ArgumentParser(description="Edit prototype round trip paths in a virtual camera image.")
    parser.add_argument("--config", type=Path, help="Configuration JSON (default: configs/demo.json; saved config when loading)")
    parser.add_argument("--demo", choices=("straight", "bump"), help="Original path shape (default: straight)")
    parser.add_argument("--save", type=Path, default=Path("output/session.json"), help="Validated prototype JSON destination")
    parser.add_argument("--load", type=Path, help="Reload validated prototype JSON")
    parser.add_argument("--headless", action="store_true", help="Render once with Agg without opening a window")
    parser.add_argument("--screenshot", type=Path, help="Export the initial camera editor view as an image")
    parser.add_argument("--language", choices=("en", "ko"), help="UI language; missing Korean font always falls back to English")
    args = parser.parse_args(argv)
    if args.headless:
        matplotlib.use("Agg", force=True)
    if args.load is not None and args.demo is not None:
        parser.error("--load already contains its original geometry; --demo cannot be combined with --load")
    try:
        if args.load is not None:
            expected = load_config(args.config) if args.config else None
            session = load_session(args.load, expected_config=expected)
            editor = Editor(session.config, session.originals, session.records, revision=session.revision)
            editor.validation_report = session.report
        else:
            config = load_config(args.config or Path("configs/demo.json"))
            editor = Editor(config, generate_paths(config, args.demo or "straight"))
        viewer = Viewer(editor, args.save, language=args.language)
        if args.load is not None:
            viewer._notice = viewer.ui["loaded"]
            viewer.render()
        if args.screenshot is not None:
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            viewer.figure.savefig(args.screenshot, dpi=130, facecolor=viewer.figure.get_facecolor())
        if args.headless:
            viewer.figure.canvas.draw()
            import matplotlib.pyplot as plt
            plt.close(viewer.figure)
            print("Headless camera view rendered" + (f": {args.screenshot}" if args.screenshot else ""))
        else:
            viewer.show()
        return 0
    except (OSError, ValueError, KeyError) as exc:
        print(f"pac_path_editor: {exc}", file=sys.stderr)
        return 2
