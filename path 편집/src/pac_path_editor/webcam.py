"""Webcam input and OpenCV overlay for a virtual, uncalibrated editing frame.

Normalized landmark XYZ is a relative control signal, never measured hand
position in metres. Geometry/history/limits/validation belong to Editor/core.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, fields, replace
import json
from pathlib import Path
import sys
import time

import numpy as np

from .core import Config, PATH_IDS, generate_paths, interpolate_at_xi
from .editor import Editor
from .ros_bridge import EditorCommands, RosBridge
from .storage import load_config, load_session
from .viewer import height_to_rgba, pick_point


@dataclass
class WebcamConfig:
    camera: int = 0
    width: int = 1280
    height: int = 720
    mirror: bool = True
    start_x: float = .15
    end_x: float = .85
    path_y: float = .58
    virtual_y_span: float = .48
    gain_x: float = .85
    gain_y: float = .48
    gain_z: float = 1.0
    sign_x: int = 1
    sign_y: int = -1
    sign_z: int = -1
    pinch_close: float = .28
    pinch_open: float = .45
    debounce_s: float = .08
    filter_tau_s: float = .05
    loss_timeout_s: float = .5
    model_path: str = 'models/hand_landmarker.task'
    hand_api: str = 'auto'
    confidence: float = .5

    def validate(self):
        if type(self.camera) is not int or self.camera < 0 or type(self.mirror) is not bool:
            raise ValueError('camera must be a nonnegative index; mirror must be boolean')
        if any(type(v) is not int or v < 160 for v in (self.width, self.height)):
            raise ValueError('width/height must be integers >= 160')
        if not 0 <= self.start_x < self.end_x <= 1 or not 0 < self.path_y < 1:
            raise ValueError('Endpoint fractions must lie inside the image')
        if not 0 < self.pinch_close < self.pinch_open or not 0 < self.confidence <= 1:
            raise ValueError('Require 0 < pinch_close < pinch_open and confidence in (0,1]')
        for key in ('virtual_y_span', 'gain_x', 'gain_y', 'gain_z', 'loss_timeout_s'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f'{key} must be positive and finite')
        for key in ('debounce_s', 'filter_tau_s'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) < 0:
                raise ValueError(f'{key} must be nonnegative and finite')
        if any(v not in (-1, 1) for v in (self.sign_x, self.sign_y, self.sign_z)):
            raise ValueError('Axis signs must be -1 or 1')
        if self.hand_api not in ('auto', 'solutions', 'tasks'):
            raise ValueError('hand_api must be auto, solutions or tasks')

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        if not isinstance(data, dict) or set(data) - {f.name for f in fields(cls)}:
            raise ValueError('Unknown webcam configuration fields')
        result = cls(**data)
        result.validate()
        return result


def hand_to_editor(xyz, config: WebcamConfig) -> np.ndarray:
    """Relative normalized landmark control -> virtual XYZ; replace for depth later."""
    xyz = np.asarray(xyz, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError('Expected finite landmark XYZ')
    return xyz * np.array([config.gain_x * config.sign_x, config.gain_y * config.sign_y,
                           config.gain_z * config.sign_z])


def prepare_frame(frame, mirror: bool):
    """Only mirror operation: tracker, landmarks, cursor and picking use its output."""
    return np.ascontiguousarray(frame[:, ::-1] if mirror else frame)


class OverlayProjection:
    def __init__(self, config: Config, webcam: WebcamConfig, width: int, height: int):
        if config.target[0] <= config.base[0] or config.target[1] != config.base[1]:
            raise ValueError('Webcam demo needs target.x > base.x and equal endpoint Y')
        self.base = np.asarray(config.base)
        self.origin = np.array([webcam.start_x * width, webcam.path_y * height])
        self.scale = np.array([(webcam.end_x - webcam.start_x) * width /
                               (config.target[0] - config.base[0]), -height / webcam.virtual_y_span])

    def project(self, points):
        points = np.asarray(points)
        return self.origin + (points[..., :2] - self.base[:2]) * self.scale, np.ones(points.shape[:-1])


def hand_sample(landmarks, width, height):
    points = np.asarray(landmarks, dtype=float)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError('Expected 21 finite normalized XYZ landmarks')
    pixel_points = points[:, :2] * [width, height]
    palm = max(np.linalg.norm(pixel_points[0] - pixel_points[9]),
               np.linalg.norm(pixel_points[5] - pixel_points[17]))
    if palm < 2:
        raise ValueError('Hand scale is too small')
    return ((points[4] + points[8]) / 2,
            (pixel_points[4] + pixel_points[8]) / 2,
            float(np.linalg.norm(pixel_points[4] - pixel_points[8]) / palm))


class HandInput:
    """Release-to-arm pinch state; missing tracking cannot commit a gesture."""
    def __init__(self, commands: EditorCommands, config: WebcamConfig):
        config.validate()
        self.commands, self.config = commands, config
        self.armed = False
        self.pinched = False
        self.tracked = False
        self.cursor = None
        self.xyz = None
        self._filtered = None
        self._last_time = None
        self._missing_since = None
        self._blocked_since_loss = False
        self._transition = None
        self._transition_since = 0.
        self._editing = False

    def _stable(self, value, now):
        if self._transition != value:
            self._transition, self._transition_since = value, now
        return now - self._transition_since + 1e-12 >= self.config.debounce_s

    def step(self, landmarks, now: float, width: int, height: int):
        e, c = self.commands.editor, self.config
        # A new edit session must observe an open hand, including keyboard/voice entry.
        if e.editing != self._editing:
            self.armed = self.pinched = False
            self._transition = None
            self._editing = e.editing
        sample = None
        if landmarks is not None:
            try:
                sample = hand_sample(landmarks, width, height)
            except (ValueError, TypeError):
                pass
        self.tracked = sample is not None
        if sample is None:
            self.cursor = None
            self.armed = False
            self._transition = None
            self._blocked_since_loss = True
            if self._missing_since is None:
                self._missing_since = now
            if e.drag is not None and now - self._missing_since >= c.loss_timeout_s:
                e.execute_command('cancel_drag')
                self.pinched = False
            self.commands.refresh()
            return
        xyz, self.cursor, ratio = sample
        self.xyz = xyz
        elapsed = 0 if self._last_time is None else max(0., now - self._last_time)
        self._last_time = now
        if self._filtered is None or self._blocked_since_loss:
            self._filtered = xyz.copy()
        else:
            alpha = 1. if c.filter_tau_s == 0 else -np.expm1(-elapsed / c.filter_tau_s)
            self._filtered += alpha * (xyz - self._filtered)
        # Even a short loss blocks all resumed motion. Keep preview until an open
        # hand cancels it, or the loss deadline expires; never turn loss into release.
        if self._blocked_since_loss:
            if e.drag is not None and now - self._missing_since >= c.loss_timeout_s:
                e.execute_command('cancel_drag')
            if ratio >= c.pinch_open and self._stable('reacquired_open', now):
                if e.drag is not None:
                    e.execute_command('cancel_drag')
                self._blocked_since_loss = False
                self._missing_since = None
                self.pinched = False
                self.armed = e.editing
                self._transition = None
            elif ratio < c.pinch_open:
                self._transition = None
            self.commands.refresh()
            return
        if not e.editing:
            self.armed = False
            return
        desired = 'open' if ratio >= c.pinch_open else ('closed' if ratio <= c.pinch_close else None)
        if desired is None:
            self._transition = None
        elif self._stable(desired, now):
            if desired == 'open':
                if self.pinched and e.drag is not None:
                    e.execute_command('end_drag')
                self.pinched = False
                self.armed = True
            elif not self.pinched:
                self.pinched = True
                was_armed, self.armed = self.armed, False
                if was_armed:
                    projection = OverlayProjection(e.config, c, width, height)
                    selection = pick_point(e.paths, projection, self.cursor, e.mode,
                                           e.config.selection_radius_px)
                    if selection is not None:
                        # Same filtered signal for baseline and subsequent updates.
                        e.execute_command('begin_drag', path_id=selection.path_id, xi=selection.xi,
                                          hand_position=hand_to_editor(self._filtered, c))
        if self.pinched and e.drag is not None and desired != 'open':
            e.execute_command('update_drag', hand_position=hand_to_editor(self._filtered, c))
        self.commands.refresh()


class HandTracker:
    """One hand, legacy Hands when present; Tasks API fallback with explicit model."""
    def __init__(self, config: WebcamConfig):
        import mediapipe as mp
        self.mp = mp
        legacy = hasattr(mp, 'solutions') and hasattr(mp.solutions, 'hands')
        self.legacy = config.hand_api == 'solutions' or (config.hand_api == 'auto' and legacy)
        if self.legacy:
            if not legacy:
                raise ValueError('mp.solutions.hands is unavailable; use hand_api=tasks and model_path')
            self.engine = mp.solutions.hands.Hands(max_num_hands=1,
                min_detection_confidence=config.confidence, min_tracking_confidence=config.confidence)
        else:
            if not Path(config.model_path).is_file():
                raise ValueError(f'Hand Landmarker model missing: {config.model_path}; see README download command')
            self.engine = mp.tasks.vision.HandLandmarker.create_from_options(
                mp.tasks.vision.HandLandmarkerOptions(
                    base_options=mp.tasks.BaseOptions(model_asset_path=config.model_path),
                    running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=1,
                    min_hand_detection_confidence=config.confidence,
                    min_tracking_confidence=config.confidence))
        self._timestamp = -1

    def process(self, bgr, now):
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if self.legacy:
            result = self.engine.process(rgb)
            hands = result.multi_hand_landmarks
            landmarks = hands[0].landmark if hands else None
        else:
            self._timestamp = max(self._timestamp + 1, int(now * 1000))
            result = self.engine.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB,
                                                                 data=rgb), self._timestamp)
            landmarks = result.hand_landmarks[0] if result.hand_landmarks else None
        return None if landmarks is None else np.array([[p.x, p.y, p.z] for p in landmarks])

    def close(self):
        self.engine.close()


HAND_EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
              (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
              (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]


def render(frame, inputs: HandInput, landmarks=None):
    import cv2
    e, c = inputs.commands.editor, inputs.config
    h, w = frame.shape[:2]
    canvas = frame.copy()
    projection = OverlayProjection(e.config, c, w, h)
    paths = e.paths
    norm = replace(e.config.colors, mode='fixed')

    def pixel(point):
        return tuple(np.rint(projection.project(np.asarray(point))[0]).astype(int))

    def line(path, alpha=1., black=False):
        layer = canvas.copy()
        pixels = projection.project(path.points)[0]
        rgb = height_to_rgba(path.points[:, 2], norm)[:, :3]
        distance = 0.
        for i, (a, b) in enumerate(zip(pixels[:-1], pixels[1:])):
            length = float(np.linalg.norm(b - a))
            # Dash in pixels, independent of the nonuniform geometry sampling.
            steps = max(1, int(np.ceil(length / 3)))
            for j in range(steps):
                if path.path_id == 'return' and int((distance + length * (j + .5) / steps) / 9) % 2:
                    continue
                p = np.rint(a + (b - a) * j / steps).astype(int)
                q = np.rint(a + (b - a) * (j + 1) / steps).astype(int)
                color = (0, 0, 0) if black else tuple(int(v) for v in rgb[i, ::-1] * 255)
                cv2.line(layer, p, q, color, 6 if black else 3, cv2.LINE_AA)
            distance += length
        cv2.addWeighted(layer, alpha, canvas, 1 - alpha, 0, dst=canvas)

    targets = set(PATH_IDS if e.mode == 'both' else (e.mode,))
    for name in sorted(PATH_IDS, key=lambda n: n in targets):
        active = name in targets
        if active and e.drag:
            line(e.drag.before_paths[name], black=True)
        line(paths[name], 1. if active else .22)
        # Direction markers at different xi distinguish coincident reverse paths.
        xi, next_xi = ((.33, .40) if name == 'outbound' else (.70, .63))
        a, b = [pixel(interpolate_at_xi(paths[name], x)) for x in (xi, next_xi)]
        color = (230, 240, 250) if active else (95, 100, 105)
        cv2.arrowedLine(canvas, a, b, color, 2, cv2.LINE_AA, tipLength=.3)
        cv2.putText(canvas, name + (' ->' if name == 'outbound' else ' <-'),
                    (a[0] - 30, a[1] + (-18 if name == 'outbound' else 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
    for point, label in ((e.config.base, 'START'), (e.config.target, 'GOAL')):
        p = pixel(point)
        cv2.circle(canvas, p, 6, (250, 250, 250), -1)
        cv2.putText(canvas, label, (p[0] - 20, p[1] + 48), 0, .55, (255, 255, 255), 1, cv2.LINE_AA)
    detail = ''
    if e.drag:
        for name, center in e.drag.centers.items():
            a = pixel(center)
            actual = interpolate_at_xi(paths[name], e.drag.center_xi)
            wanted = center + e.drag.requested_delta
            cv2.drawMarker(canvas, pixel(actual), (0, 255, 255),
                           cv2.MARKER_CROSS if name == e.drag.selected_path else cv2.MARKER_DIAMOND, 20, 2)
            cv2.arrowedLine(canvas, a, pixel(wanted), (255, 150, 40), 2, tipLength=.2)
            if np.linalg.norm(actual - wanted) > e.config.numeric_tolerance:
                cv2.arrowedLine(canvas, a, pixel(actual), (0, 140, 255), 2, tipLength=.2)
                cv2.line(canvas, pixel(actual), pixel(wanted), (0, 140, 255), 1)
            if name == e.drag.selected_path:
                detail = f'xi={e.drag.center_xi:.4f} request XYZ={np.round(e.drag.requested_delta, 4)} applied={np.round(actual-center, 4)} Z={actual[2]:.4f}'
    if landmarks is not None:
        xy = np.rint(np.asarray(landmarks)[:, :2] * [w, h]).astype(int)
        for a, b in HAND_EDGES:
            cv2.line(canvas, xy[a], xy[b], (90, 225, 120), 1, cv2.LINE_AA)
        for p in xy:
            cv2.circle(canvas, p, 3, (220, 240, 220), -1)
    if inputs.cursor is not None:
        cv2.circle(canvas, tuple(np.rint(inputs.cursor).astype(int)), 9,
                   (0, 255, 255) if inputs.pinched else (255, 255, 255), 2)
    rows = [
        'VIRTUAL PATH DEMO | virtual_edit_frame | uncalibrated overlay',
        f'{"EDITING" if e.editing else "IDLE"}  mode={e.mode}  hand={inputs.tracked}  pinch={inputs.pinched}  armed={inputs.armed}  revision={e.revision}',
        f'check={inputs.commands.last.validation_status} | Robot validation NOT_EVALUATED; execution disabled',
        'E edit | 1 both | 2 outbound | 3 return | U undo | Y redo | R reset | C cancel session | F finish/save | Esc cancel drag | Q quit',
        e.last_message,
        detail or f'Height colors fixed: {norm.z_min:.2f} .. {norm.z_max:.2f} virtual m. Landmark Z is relative input.',
    ]
    if inputs.xyz is not None:
        rows.append(f'landmark XYZ={np.round(inputs.xyz, 4)} | Z gain={c.gain_z:g} sign={c.sign_z:+d}')
    panel = canvas.copy()
    cv2.rectangle(panel, (0, 0), (w, 22 * len(rows) + 16), (16, 20, 26), -1)
    cv2.addWeighted(panel, .84, canvas, .16, 0, dst=canvas)
    for i, row in enumerate(rows):
        cv2.putText(canvas, row, (12, 24 + i * 22), 0, .44, (235, 240, 245), 1, cv2.LINE_AA)
    # Common fixed height color bar also makes pure Z drags visible.
    for y in range(150):
        z = norm.z_max - (norm.z_max - norm.z_min) * y / 149
        color = tuple(int(v) for v in height_to_rgba(np.array([z]), norm)[0, :3][::-1] * 255)
        cv2.line(canvas, (w - 30, h - 180 + y), (w - 15, h - 180 + y), color, 1)
    cv2.putText(canvas, f'{norm.z_max:.2f}', (w - 76, h - 166), 0, .42,
                (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(canvas, 'virtual Z', (w - 105, h - 92), 0, .38,
                (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(canvas, f'{norm.z_min:.2f}', (w - 76, h - 25), 0, .42,
                (240, 240, 240), 1, cv2.LINE_AA)
    return canvas


def synthetic_hand(cursor=(.5, .58), pinch=.1, z=0.):
    """Deterministic normalized landmarks for device-free state/input tests."""
    hand = np.tile([cursor[0], cursor[1], z], (21, 1))
    hand[0, 1] += .18
    hand[9, 1] += .03
    hand[5, 0] -= .04
    hand[17, 0] += .04
    hand[4, 0] -= pinch / 2
    hand[8, 0] += pinch / 2
    return hand


def run_synthetic(commands, config, screenshot):
    """Exercise actual hand adapter -> Editor -> OpenCV, without a device/ROS."""
    import cv2
    inputs = HandInput(commands, config)
    commands.execute('SELECT_MODE', 'both', source='synthetic')
    cursor = ((config.start_x + config.end_x) / 2, config.path_y)
    frames = []
    for t, hand in ((0., synthetic_hand(cursor)), (.12, synthetic_hand(cursor)),
                    (.2, synthetic_hand(cursor, .01)), (.32, synthetic_hand(cursor, .01)),
                    (.55, synthetic_hand((cursor[0]+.04, cursor[1]-.05), .01, -.08))):
        inputs.step(hand, t, config.width, config.height)
        frames.append(render(np.full((config.height, config.width, 3), (65, 60, 52), np.uint8), inputs, hand))
    if commands.editor.drag is None:
        raise RuntimeError('Synthetic pinch did not start')
    if screenshot:
        screenshot.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(screenshot), frames[-1]):
            raise OSError('Could not write screenshot')
    inputs.step(synthetic_hand(cursor), .7, config.width, config.height)
    inputs.step(synthetic_hand(cursor), .82, config.width, config.height)
    result = commands.execute('FINISH_EDIT', source='synthetic')
    if not result.accepted:
        raise RuntimeError(result.message)
    print(f'Synthetic XYZ pinch + both + render + save: PASS; revision={result.revision}; no camera/microphone used')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--webcam', action='store_true')
    parser.add_argument('--camera', type=int)
    parser.add_argument('--webcam-config', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--load', type=Path)
    parser.add_argument('--save', type=Path, default=Path('output/webcam-session.json'))
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--screenshot', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--no-voice', action='store_true')
    mode.add_argument('--ros', action='store_true')
    args = parser.parse_args(argv)
    bridge = capture = tracker = None
    try:
        import cv2
        c = WebcamConfig.load(args.webcam_config) if args.webcam_config else WebcamConfig()
        if args.camera is not None:
            c.camera = args.camera
        c.validate()
        if args.load:
            session = load_session(args.load, load_config(args.config) if args.config else None)
            if session.config.frame != 'virtual_edit_frame':
                raise ValueError('Webcam cannot load robot_base as virtual_edit_frame; create a new virtual session')
            e = Editor(session.config, session.originals, session.records, session.revision)
            e.validation_report = session.report
        else:
            config = load_config(args.config) if args.config else Config(frame='virtual_edit_frame')
            if config.frame != 'virtual_edit_frame':
                raise ValueError('Webcam config must explicitly use frame=virtual_edit_frame')
            e = Editor(config, generate_paths(config))
        commands = EditorCommands(e, args.save)
        if args.synthetic:
            if args.ros:
                parser.error('--synthetic uses no ROS; use ROS integration tests for transport')
            run_synthetic(commands, c, args.screenshot)
            return 0
        if args.ros:
            bridge = RosBridge(commands)
        tracker = HandTracker(c)
        capture = cv2.VideoCapture(c.camera)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
        if not capture.isOpened():
            raise RuntimeError(f'Cannot open camera index {c.camera}')
        inputs = HandInput(commands, c)
        title = 'Virtual webcam path editor'
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        keymap = {ord('e'): ('START_EDIT', ''), ord('1'): ('SELECT_MODE', 'both'),
                  ord('2'): ('SELECT_MODE', 'outbound'), ord('3'): ('SELECT_MODE', 'return'),
                  ord('u'): ('UNDO', ''), ord('y'): ('REDO', ''), ord('r'): ('RESET', ''),
                  ord('c'): ('CANCEL', ''), ord('f'): ('FINISH_EDIT', ''), 27: ('CANCEL_DRAG', '')}
        while True:
            commands.drain()
            ok, frame = capture.read()
            if not ok:
                e.execute_command('cancel_drag')
                raise RuntimeError('Camera frame unavailable; active drag cancelled')
            frame = prepare_frame(frame, c.mirror)
            now = time.monotonic()
            landmarks = tracker.process(frame, now)
            h, w = frame.shape[:2]
            inputs.step(landmarks, now, w, h)
            cv2.imshow(title, render(frame, inputs, landmarks))
            if bridge:
                bridge.publish()
            key = cv2.waitKey(1) & 0xff
            if key == ord('q') or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in keymap:
                commands.execute(*keymap[key])
        e.execute_command('cancel_drag')
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f'pac_path_editor webcam: {exc}', file=sys.stderr)
        return 2
    finally:
        if tracker:
            tracker.close()
        if capture:
            capture.release()
            cv2.destroyAllWindows()
        if bridge:
            bridge.close()
