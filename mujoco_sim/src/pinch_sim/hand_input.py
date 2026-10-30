"""Hand input for the side_cam monitor (MediaPipe): pointing = click, pinch = drag.

The hand stands in for the mouse on the monitor image, so it goes through the same MouseEditor
(same picking, same ray / vertical-line geometry, same Session methods):

  * point (index finger straight, the other fingers folded, thumb away from the index):
    hold it still for `point_dwell_s` -> the nearest path point is selected (= a mouse click).
    The fingertip is drawn as a dot.
  * pinch (thumb tip and index tip together): the midpoint of the two tips is drawn as a circle
    (like the path editor's webcam demo). Closing the pinch near a path point grabs the point nearest to it
    on the screen (2-D), moving the pinch drags it, opening the pinch releases it.
      - with a depth camera (`HandController(depth=HandDepth(...))`): the grabbed point follows the hand's
        3-D motion since the pinch closed, 1:1 (hand_depth.py). The path changes only after the hand moved
        `start_m`; a pinch opened before that only selects the point (like a click).
        A 3-D grab needs a verified camera calibration (session.calib_state, fixed markers) and a depth measured
        at the fingers in THIS frame, in sync with the colour frame. Otherwise nothing is grabbed and the reason is
        shown and sent as a "hand_refused" event - never a silent switch to the 2-D height drag. Depth lost during
        a drag freezes the preview; after `lost_cancel_s` the drag is cancelled (path restored), and opening the
        pinch while the depth is missing cancels instead of keeping a frozen preview.
      - without a depth camera (a webcam, explicit 2-D mode shown on screen): like the mouse - the point moves on
        its vertical line under the pinch (height only).
  * anything in between (thumb neither on the index tip nor clearly away) does nothing, so pointing never
    turns into a grab on the way to a pinch and the other way round.
  * the hand edits only in edit mode (session.edit_mode: voice "경로 수정할래", key E, a mouse drag); outside it a
    point / pinch is refused with a hint, so a stray hand never changes the path. The mouse is not gated.

Rules taken from the path editor's HandInput (path 편집/src/pac_path_editor/webcam.py):
  * pinch hysteresis: closed below `pinch_close`, open above `pinch_open` (thumb-index distance over
    the palm size), each change must hold for `debounce_s`;
  * a new pinch only after an open hand (release-to-arm);
  * losing the hand is not a release: the drag preview freezes and is cancelled after `loss_timeout_s`.

Camera and MediaPipe run in their own thread; the Astra depth too (hand_depth.AstraDepthSource).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

WRIST, THUMB_TIP, INDEX_MCP, INDEX_PIP, INDEX_TIP = 0, 4, 5, 6, 8
MIDDLE_MCP, PINKY_MCP = 9, 17
FOLD_PAIRS = ((12, 10), (16, 14), (20, 18))       # (tip, pip) of middle, ring, little finger
HAND_EDGES = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
              (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


@dataclass
class HandConfig:
    camera: int = 0
    width: int = 640
    height: int = 480
    mirror: bool = True              # selfie view: moving the hand right moves the cursor right
    rotate_180: bool = False         # camera mounted upside down (pac_vision camera.rotate_180): frames turned first
    pinch_close: float = 0.28
    pinch_open: float = 0.45
    debounce_s: float = 0.08
    point_dwell_s: float = 0.4       # hold the pointing finger this long to select
    point_still_px: float = 12.0     # ... moving less than this (displayed pixels)
    smooth_tau_s: float = 0.06
    loss_timeout_s: float = 0.5
    select_radius_px: float = 32.0   # render pixels; wider than the mouse (18) because a hand is less exact
    confidence: float = 0.5


@dataclass
class Gesture:
    kind: str                         # "pinch" | "point" | "open" | "other"
    cursor: tuple[float, float]       # normalized image coordinates of the pinch midpoint / index tip
    ratio: float                      # thumb-index distance over palm size


def _px(lm, w, h):
    return np.asarray(lm, float)[:, :2] * [w, h]


def classify(landmarks, w: int, h: int, pinched: bool, cfg: HandConfig) -> Gesture | None:
    """21 normalized landmarks -> gesture. `pinched` = current pinch state (hysteresis)."""
    lm = np.asarray(landmarks, float)
    if lm.shape != (21, 3) or not np.isfinite(lm).all():
        return None
    p = _px(lm, w, h)
    palm = max(np.linalg.norm(p[WRIST] - p[MIDDLE_MCP]), np.linalg.norm(p[INDEX_MCP] - p[PINKY_MCP]))
    if palm < 2:
        return None
    ratio = float(np.linalg.norm(p[THUMB_TIP] - p[INDEX_TIP]) / palm)
    mid = tuple((lm[THUMB_TIP, :2] + lm[INDEX_TIP, :2]) / 2)
    if ratio <= cfg.pinch_close or (pinched and ratio < cfg.pinch_open):
        return Gesture("pinch", mid, ratio)
    d = lambda i: np.linalg.norm(p[i] - p[WRIST])
    index_straight = d(INDEX_TIP) > 1.15 * d(INDEX_PIP) and \
        np.linalg.norm(p[INDEX_TIP] - p[INDEX_MCP]) > 0.6 * palm
    others_folded = all(d(tip) < d(pip) * 1.05 for tip, pip in FOLD_PAIRS)
    if index_straight and others_folded and ratio >= cfg.pinch_open:
        return Gesture("point", tuple(lm[INDEX_TIP, :2]), ratio)
    return Gesture("open" if ratio >= cfg.pinch_open else "other", mid, ratio)


REFUSAL_TEXT = {
    "not_edit_mode": "수정 모드가 아니에요 - E 또는 '쿠옹아, 경로 수정할래'",
    "calib_unverified": "정합 미검증: 3D 핀치 꺼짐 (마우스로 고칠 수 있어요)",
    "no_depth": "깊이를 못 읽었어요: 손을 카메라에서 조금 떨어뜨려 주세요",
    "rgb_depth_out_of_sync": "깊이와 영상 시각이 안 맞아요: 다시 집어 주세요",
    "depth_stream_stopped": "깊이 카메라가 멈췄어요",
    "depth_lost": "깊이가 끊겨 이번 수정을 취소했어요",
    "edit_mode_off": "수정 모드가 꺼져 잡은 점을 놓았어요",
}
MESSAGE_S = 2.5                  # a refusal stays next to the cursor this long


class HandController:
    """Gestures -> MouseEditor calls (main thread). Camera independent, so it can be tested."""

    def __init__(self, cfg: HandConfig, depth=None):
        self.cfg = cfg
        self.depth = depth            # HandDepth: pinch moves the point in 3-D; None: height only (2-D)
        # {"k", "p0" world point, "h0" hand point, "started", "lost_since"} during a 3-D pinch
        self.grab3d: dict | None = None
        self.message, self._message_until = "", 0.0     # last refusal, shown next to the cursor
        self._now = 0.0
        self.pinched = False          # hysteresis state of the hand pose
        self.armed = False            # an open hand was seen since the last pinch / loss
        self.grabbing = False         # a press was sent to the MouseEditor
        self.gesture: Gesture | None = None
        self.cursor_px: tuple[float, float] | None = None    # smoothed, displayed-image pixels
        self._pending, self._pending_since = None, 0.0
        self._last_t: float | None = None
        self._missing_since: float | None = None
        self._dwell_xy, self._dwell_since, self._dwell_done = None, 0.0, False
        self.hover: np.ndarray | None = None    # world point of the path point the finger would take now

    def _stable(self, kind: str, now: float) -> bool:
        if self._pending != kind:
            self._pending, self._pending_since = kind, now
        return now - self._pending_since + 1e-12 >= self.cfg.debounce_s

    def step(self, landmarks, now: float, cam_w: int, cam_h: int, img_w: int, img_h: int,
             mouse_editor, session, d, depth_mm=None, depth_reason: str = "") -> None:
        """depth_mm: the depth frame matching this colour frame (with `depth`; hand_depth.pick_depth), None without
        a depth camera or when it is missing - then depth_reason says why (no_depth_frame, rgb_depth_out_of_sync,
        depth_stream_stopped)."""
        cfg = self.cfg
        self._now = now
        g = classify(landmarks, cam_w, cam_h, self.pinched, cfg) if landmarks is not None else None
        self.gesture = g
        editing = bool(session.edit_mode)
        if self.grabbing and not editing:                 # edit mode switched off (E, voice) during a grab
            self._abort(mouse_editor, session, "edit_mode_off")
        elif self.grab3d is not None and session.calib_state != "verified":   # the camera moved during a grab
            self._abort(mouse_editor, session, "calib_unverified")
        if g is None:                                     # hand lost: never a release
            if self._missing_since is None:
                self._missing_since = now
            self.armed, self.cursor_px, self._dwell_xy = False, None, None
            if self.grabbing and now - self._missing_since >= cfg.loss_timeout_s:
                mouse_editor.cancel(session)
                self.grabbing = self.pinched = False
                self.grab3d = None
            return
        self._missing_since = None
        self.hover = None
        target = (g.cursor[0] * img_w, g.cursor[1] * img_h)
        if self.cursor_px is None or self._last_t is None:
            self.cursor_px = target
        else:
            a = 1.0 if cfg.smooth_tau_s <= 0 else -np.expm1(-max(0.0, now - self._last_t) / cfg.smooth_tau_s)
            self.cursor_px = tuple(c + a * (t - c) for c, t in zip(self.cursor_px, target))
        self._last_t = now
        x, y = self.cursor_px

        is_pinch = g.kind == "pinch"
        lm_px = None if self.depth is None else _px(landmarks, cam_w, cam_h)
        if self.depth is not None:                        # measured only while the fingers are together
            self.depth.update(lm_px if is_pinch else None, depth_mm, now)
            if is_pinch and depth_mm is None and depth_reason:
                self.depth.reason = depth_reason          # why there was no depth frame to measure in
        if is_pinch != self.pinched and self._stable("pinch" if is_pinch else "release", now):
            self.pinched = is_pinch
            if is_pinch:
                if self.armed:                            # grab the point nearest to the pinch on the screen
                    self.grabbing = self._grab(mouse_editor, session, d, x, y) if editing else \
                        self._refuse(session, "not_edit_mode")
                self.armed = False
            elif self.grabbing:                           # open the pinch: release
                self._release(mouse_editor, session, d)
                self.grabbing = False
        elif is_pinch == self.pinched:
            self._pending = None
        if not self.pinched and not is_pinch:          # a hand seen not pinching (release-to-arm)
            self.armed = True
            if self.depth is not None:
                self.depth.reset()
        if self.pinched and self.grabbing and is_pinch:   # an opening hand (thumb leaving) moves nothing
            if self.grab3d is not None:
                self._move3d(mouse_editor, session, now)
            else:
                mouse_editor.move(session, d, x, y)

        if editing and not self.grabbing and session.path is not None and session.can_edit()[0]:
            from .screen import pick_index
            k = pick_index(session.path.points_m, mouse_editor.cam, x / mouse_editor.scale, y / mouse_editor.scale,
                           mouse_editor.radius)
            self.hover = None if k is None else session.path.points_m[k].copy()

        # pointing finger held still -> select the nearest point once (mouse click)
        if g.kind == "point" and not self.pinched:
            if self._dwell_xy is None or np.hypot(x - self._dwell_xy[0], y - self._dwell_xy[1]) > cfg.point_still_px:
                self._dwell_xy, self._dwell_since, self._dwell_done = (x, y), now, False
            elif not self._dwell_done and now - self._dwell_since >= cfg.point_dwell_s:
                if not editing:
                    self._refuse(session, "not_edit_mode")
                elif mouse_editor.press(session, d, x, y):
                    mouse_editor.release(session, d)
                self._dwell_done = True
        else:
            self._dwell_xy = None

    # ------------------------------------------------------------------ pinch: grab / move / release
    def _refuse(self, session, reason: str, detail: str = "") -> bool:
        """Show and send why the hand did nothing (False, for `self.grabbing = ...`)."""
        self.message, self._message_until = REFUSAL_TEXT.get(reason, reason), self._now + MESSAGE_S
        session._emit(f"hand refused: {reason}" + (f" ({detail})" if detail else ""), "hand_refused", reason=reason)
        return False

    def _abort(self, mouse_editor, session, reason: str) -> None:
        """Drop the current grab: a started edit is cancelled (the path goes back to before the pinch)."""
        g, self.grab3d = self.grab3d, None
        if g is not None:
            if g["started"]:
                session.edit_cancel()
        elif self.grabbing:
            mouse_editor.cancel(session)
        self.grabbing = False
        self._refuse(session, reason)

    def _grab(self, mouse_editor, session, d, x, y) -> bool:
        """3-D with a depth camera (or nothing, with the reason), the 2-D (vertical line) drag without one."""
        if self.depth is None:                             # explicit webcam mode: height only, shown on screen
            self.grab3d = None
            return bool(mouse_editor.press(session, d, x, y))
        if session.calib_state != "verified":
            return self._refuse(session, "calib_unverified", session.calib_state or "no calibration")
        if self.depth.raw is None:                         # the fingers were not measured in this frame
            why = self.depth.reason
            return self._refuse(session, why if why in REFUSAL_TEXT else "no_depth", why)
        ok, why = session.can_edit()
        if not ok:
            session._emit(f"edit refused: {why}", "refused")
            return False
        from .screen import pick_index
        k = pick_index(session.path.points_m, mouse_editor.cam, x / mouse_editor.scale, y / mouse_editor.scale,
                       mouse_editor.radius, near_world=self.depth.out)
        if k is None:
            session._emit(f"no interior path point within {mouse_editor.radius:.0f} px of the pinch")
            return False
        self.grab3d = {"k": k, "p0": session.path.points_m[k].copy(), "h0": self.depth.out.copy(), "started": False,
                       "lost_since": None, "z0": self.depth.z_m}
        return True

    def _move3d(self, mouse_editor, session, now: float) -> None:
        g, c = self.grab3d, self.depth.cfg
        if self.depth.raw is None:                         # no good depth this frame: the preview stays ...
            if g["lost_since"] is None:
                g["lost_since"] = now
            elif now - g["lost_since"] >= c.lost_cancel_s:  # ... and a long gap cancels the drag
                self._abort(mouse_editor, session, "depth_lost")
            return
        g["lost_since"] = None
        delta = self.depth.out - g["h0"]                   # the hand's own motion since the pinch closed, 1:1
        n = float(np.linalg.norm(delta))
        if n > c.max_move_m:
            delta *= c.max_move_m / n
        if not g["started"]:
            if n < c.start_m:
                return
            if not session.edit_begin(g["k"], g["p0"], free=True):
                self.grab3d, self.grabbing = None, False
                return
            g["started"] = True
        g["delta"] = delta
        session.edit_update(g["p0"] + delta)

    def _release(self, mouse_editor, session, d) -> None:
        g, self.grab3d = self.grab3d, None
        if g is None:
            mouse_editor.release(session, d)
        elif g["lost_since"] is not None:                  # opened while the depth was missing: never keep a frozen
            if g["started"]:                               # preview - cancel it
                session.edit_cancel()
            self._refuse(session, "depth_lost")
        elif g["started"]:
            session.edit_end()
            _log_move(g, self.depth.z_m)
        else:
            session.point_select(g["k"])                   # pinched without moving: select it, like a click

    def draw(self, img, top_h: int, cam=None, scale: float = 1.0, landmarks=None, img_wh=None, text=None) -> None:
        """Cursor on the monitor window: a dot for pointing, a circle for pinching.

        cam/scale: also ring the path point the finger would take now (hover). landmarks: the hand is in
        this very image (real camera mode) -> draw its skeleton thinly so the user sees what is tracked.
        """
        if landmarks is not None:
            iw, ih = img_wh or (img.shape[1], img.shape[0] - top_h)
            pts = np.asarray(landmarks)[:, :2] * [iw, ih] + [0, top_h]
            pts = np.round(pts).astype(int)
            for a, b in HAND_EDGES:
                cv2.line(img, tuple(pts[a]), tuple(pts[b]), (200, 255, 200), 1, cv2.LINE_AA)
        if cam is not None and self.hover is not None:
            uv, front = cam.project(self.hover)
            if front[0]:
                c = (int(round(uv[0][0] * scale)), int(round(uv[0][1] * scale + top_h)))
                cv2.circle(img, c, 10, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.circle(img, c, 10, (255, 230, 80), 2, cv2.LINE_AA)
        if self.cursor_px is None or self.gesture is None:
            return
        cx, cy = int(round(self.cursor_px[0])), int(round(self.cursor_px[1] + top_h))
        if self.gesture.kind == "pinch" or self.pinched:
            color = (0, 215, 255) if self.grabbing else (255, 255, 255)
            if self.depth is not None and self.depth.out is None:
                color = (60, 60, 230)                      # no depth at the fingers: 3-D move not available
            cv2.circle(img, (cx, cy), 16, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.circle(img, (cx, cy), 16, color, 2, cv2.LINE_AA)
            cv2.circle(img, (cx, cy), 3, color, -1, cv2.LINE_AA)
            if text is not None:
                self._draw_text(img, text, cx + 22, cy - 12)
        elif self.gesture.kind == "point":
            done = self._dwell_done
            cv2.circle(img, (cx, cy), 7, (0, 0, 0), -1, cv2.LINE_AA)
            cv2.circle(img, (cx, cy), 5, (0, 215, 255) if done else (255, 200, 0), -1, cv2.LINE_AA)
            if text is not None:
                self._draw_text(img, text, cx + 16, cy - 12)
        else:
            cv2.circle(img, (cx, cy), 5, (200, 200, 200), 1, cv2.LINE_AA)
            if text is not None and self.message and self._now < self._message_until:   # e.g. cancelled on release
                self._draw_text(img, text, cx + 16, cy - 12)

    def status_line(self) -> str:
        """Short Korean line next to the cursor: a refusal (why the hand did nothing), how far the grabbed point
        moved, a paused grab, or the explicit 2-D mode."""
        if self.message and self._now < self._message_until:
            return self.message
        pinching = self.gesture is not None and (self.gesture.kind == "pinch" or self.pinched)
        if self.depth is None:
            return "2D 높이 모드" if pinching else ""
        g = self.grab3d
        if g is not None and g.get("lost_since") is not None:
            return "깊이 끊김 - 잠시 멈춤 (1초 넘으면 취소)"
        if g is not None and g.get("started") and "delta" in g:
            return "옮기는 중 " + _delta_text(g["delta"])
        if pinching and self.depth.out is None:
            return "깊이 없음: 카메라에서 조금 멀리"
        return "집기" if self.grabbing else ""

    def _draw_text(self, img, text, x, y) -> None:
        s = self.status_line()
        if not s:
            return
        from .ui import blit
        spr = text.sprite(s, 18, (255, 255, 255))
        h, w = spr.shape[:2]
        x = int(min(max(0, x), img.shape[1] - w - 4))
        y = int(min(max(0, y), img.shape[0] - h - 4))
        cv2.rectangle(img, (x - 4, y - 2), (x + w + 4, y + h + 2), (30, 30, 30), -1)
        blit(img, spr, x, y)


def _log_move(g, z1) -> None:
    """Diagnosis of a kept 3-D pinch (2026-10-29): the hand's displacement in the calibration frame and as the user's
    directions, and the fingers' optical depth before / after (Astra gives no depth closer than its min range)."""
    from .canvas import world_to_cal
    d = np.asarray(g.get("delta", np.zeros(3)), float)
    dc = world_to_cal(d) - world_to_cal(np.zeros(3))
    z0 = g.get("z0")
    print(f"[hand] 3-D move kept: calibration frame dx {dc[0]*100:+.1f} dy {dc[1]*100:+.1f} dz {dc[2]*100:+.1f} cm "
          f"(x = robot right, y = toward the paper, z = up) = {_delta_text(d)}; finger depth "
          f"{'?' if z0 is None else f'{z0:.3f}'} -> {'?' if z1 is None else f'{z1:.3f}'} m", flush=True)


def _delta_text(delta_world) -> str:
    """World displacement -> e.g. "위 2.3 · 오른쪽 1.0 cm" along the user's axes (largest first, >= 0.5 cm)."""
    from .canvas import user_axes
    ax = dict(user_axes(_USER_SIDE[0]))
    if _SWAP_LR[0]:                       # --swap-left-right: the same left / right as the voice directions
        ax["right"], ax["left"] = ax["left"], ax["right"]
    parts = []
    for pos, neg, key in (("위", "아래", "up"), ("오른쪽", "왼쪽", "right"), ("뒤쪽", "앞쪽", "away")):
        c = float(np.dot(delta_world, ax[key])) * 100
        if abs(c) >= 0.5:
            parts.append((abs(c), f"{pos if c > 0 else neg} {abs(c):.1f}"))
    return " · ".join(t for _, t in sorted(parts, reverse=True)) + " cm" if parts else "0 cm"


_USER_SIDE = ["right"]      # set by the app from --user-side (for the displayed directions only)
_SWAP_LR = [False]          # set by the app from --swap-left-right (displayed directions only)


class HandCamera:
    """Camera + MediaPipe Hands in a background thread; `latest()` gives the newest result.

    device: a camera index or a video file (looped). With the scene camera (real camera mode) the same
    feed is also the monitor image (`frame()`), so the camera is opened once and the finger is drawn
    exactly where it is in the picture.
    """

    def __init__(self, cfg: HandConfig, device=None, set_size: bool = True):
        import mediapipe as mp                       # optional dependency (requirements-hand.txt)
        self.cfg = cfg
        self._hands = mp.solutions.hands.Hands(max_num_hands=1, min_detection_confidence=cfg.confidence,
                                               min_tracking_confidence=cfg.confidence)
        dev = cfg.camera if device is None else device
        self._file = isinstance(dev, str) and not dev.isdigit()
        self._cap = cv2.VideoCapture(dev if self._file else int(dev))
        if set_size and not self._file:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera {dev}")
        self._period = 1.0 / (self._cap.get(cv2.CAP_PROP_FPS) or 30.0) if self._file else 0.0
        self._lock = threading.Lock()
        self._latest = (None, 0.0, cfg.width, cfg.height, None)   # landmarks, t, w, h, preview frame
        self._stop = False
        threading.Thread(target=self._run, name="hand-camera", daemon=True).start()

    def _run(self) -> None:
        while not self._stop:
            t0 = time.monotonic()
            ok, frame = self._cap.read()
            t_cap = time.monotonic()        # capture time (before MediaPipe): matched against the depth frames
            if not ok:
                if self._file:                            # video file: loop
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                time.sleep(0.01)
                continue
            if self.cfg.rotate_180:
                frame = np.ascontiguousarray(frame[::-1, ::-1])
            if self.cfg.mirror:
                frame = np.ascontiguousarray(frame[:, ::-1])
            res = self._hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            lm = None
            if res.multi_hand_landmarks:
                lm = np.array([[p.x, p.y, p.z] for p in res.multi_hand_landmarks[0].landmark])
            h, w = frame.shape[:2]
            with self._lock:
                self._latest = (lm, t_cap, w, h, frame)
            if self._period:                              # play a file at its own frame rate
                time.sleep(max(0.0, self._period - (time.monotonic() - t0)))

    def frame(self):
        """Newest camera frame (BGR) for the monitor, None until the first one arrives."""
        with self._lock:
            f = self._latest[4]
        return None if f is None else f.copy()

    def latest(self):
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop = True
        time.sleep(0.05)
        self._cap.release()


def draw_preview(img, frame, landmarks, top_h: int, width: int = 200) -> None:
    """Small webcam view with the hand skeleton in the lower-left corner of the monitor image."""
    if frame is None:
        return
    h = int(round(frame.shape[0] * width / frame.shape[1]))
    small = cv2.resize(frame, (width, h))
    if landmarks is not None:
        pts = (np.asarray(landmarks)[:, :2] * [width, h]).astype(int)
        for a, b in HAND_EDGES:
            cv2.line(small, tuple(pts[a]), tuple(pts[b]), (0, 255, 0), 1, cv2.LINE_AA)
    y0 = img.shape[0] - h - 44
    if y0 > top_h:
        img[y0:y0 + h, 8:8 + width] = small
        cv2.rectangle(img, (8, y0), (8 + width, y0 + h), (255, 255, 255), 1)
