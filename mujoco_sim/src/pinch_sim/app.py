"""Two-window demo: MuJoCo passive viewer (free camera) + OpenCV monitor window.

Monitor = a camera frame (sim render of `--camera`, or a real camera with `--video`) with the path
drawn as a 2-D graph projected through a calibrated camera model (`--monitor-calib`, default: the
exact model of the sim camera). The planner overlay is NOT drawn into the frame's 3-D scene, so the
same view works on a real camera image. The MuJoCo viewer keeps its 3-D overlay.

Commands come from the side_cam (OpenCV) window: click its button bar, or press the key while
that window has keyboard focus:
  N new seeded target   S select shown path   V request validation   Y approve & execute
  H return path to ready   X cancel/stop (also cancels a drag)   U undo  R redo  O original path   Q quit
  E edit mode on/off (the hand edits only in edit mode)   G target = the object the depth camera sees (--depth)
Path editing (side_cam image area): left-press near a path point picks it, dragging changes only its
height: x, y stay fixed and z is the point of the vertical line through it that is closest to the
mouse ray (real side_cam pose/fovy), release keeps the edit,
right-click cancels the drag. Any edit clears validation/approval: S, V, Y are needed again.
Keys pressed in the MuJoCo window are MuJoCo's own shortcuts (S shadow, V tendon, H hull, ...),
NOT these commands. The MuJoCo window keeps its mouse camera controls (and Ctrl+drag perturbation).
Without --target-r the first seeded target is generated at start (selection/validation/approval
are still required).

Both windows draw the same Overlay built from the same MjData instant (see overlay.py);
the sim time and frame number are printed in both.

  python -m pinch_sim.app --target-r 0.275 --target-deg 0
  python -m pinch_sim.app --seed 7
  python -m pinch_sim.app --target-r 0.275 --target-deg 0 --work-height -0.02 --allow-unsafe-test-geometry
  python -m pinch_sim.app ... --script "2:S,3:V,8:Y,20:Q"   (scripted key presses at wall-clock seconds,
                                                            for unattended GUI smoke tests)
  --script "2:DRAG:0.4:30:-20"  scripted mouse drag through the same mouse handler: press on the path
                                point at 40 % of its samples, move by (+30, -20) display pixels, release
  --voice-gateway [SOCKET]      accept the voice process (쿠옹이: python -m voice --sim, one client at a time) on a
                                local socket; its commands run in this main loop through the same Session
                                methods as the buttons (default socket $XDG_RUNTIME_DIR/pinch_sim/voice.sock)
  --calib CAM2BASE --video auto --hand --depth    real Astra: the calibration must pass, match the current
                                pac_vision settings and the camera's USB identity; fixed markers (markers.yaml)
                                are checked on the live image before the overlay / 3-D pinch count as calibrated
"""
from __future__ import annotations

import argparse
import faulthandler
import json
import os
import threading
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")   # offscreen Renderer on EGL, independent of the viewer's GLFW

import cv2  # noqa: E402
import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402

from .model import ROOT, config_sha, load_config, load_scene, ready_q, with_overrides  # noqa: E402
from .camera_model import CameraModel  # noqa: E402
from .monitor import SimFrameSource, VideoFrameSource, draw_overlay_2d  # noqa: E402
from .overlay import add_overlay, build_overlay, peak_text  # noqa: E402
from .screen import pick_index, pixel_to_vertical  # noqa: E402
from .session import EDITABLE, KOREAN, Session  # noqa: E402
from .ui import (Layout, MonitorUI, Notices, arrange_viewer, auto_layout, monitors, notice_text,  # noqa: E402
                 protect_points, window_pos)
from .voice_gateway import VoiceGateway  # noqa: E402

WIN = "side_cam (click buttons / keys here)"
BUTTONS = (("N", "new"), ("E", "edit"), ("S", "select"), ("V", "valid"), ("Y", "RUN"), ("H", "home"),
           ("X", "stop"), ("U", "undo"), ("R", "redo"), ("O", "orig"), ("Q", "quit"))
BAR_H = 36                       # button bar at the bottom of the side_cam window (= ui.Layout.bar_h)
ENABLED = {"N": lambda st: st not in ("EXECUTING", "VALIDATING"), "E": lambda st: st in EDITABLE,
           "S": lambda st: st == "GENERATED",
           "V": lambda st: st in ("SELECTED", "FAILED"), "Y": lambda st: st == "PASSED",
           "H": lambda st: st in ("HOLD", "DONE", "ABORTED"), "X": lambda st: True, "Q": lambda st: True,
           "U": lambda st: st in EDITABLE, "R": lambda st: st in EDITABLE, "O": lambda st: st in EDITABLE}


def button_at(x: int, y: int, w: int, h: int) -> str | None:
    """Key of the button under image pixel (x, y) of a w x h side_cam image, or None."""
    if not (h - BAR_H <= y < h and 0 <= x < w):
        return None
    return BUTTONS[min(len(BUTTONS) - 1, int(x * len(BUTTONS) / w))][0]


def parse_script(s: str | None) -> list[tuple[float, str]]:
    """'t:KEY', 't:DRAG:frac:dx:dy' or 't:CLICK:frac' items, sorted by wall-clock seconds."""
    if not s:
        return []
    out = []
    for item in s.split(","):
        t, rest = item.split(":", 1)
        out.append((float(t), rest.strip().upper()))
    return sorted(out, key=lambda x: x[0])


class MouseEditor:
    """side_cam mouse -> world-space edit events for the Session (hand input can replace this class).

    Display pixels (x, y) are divided by the display scale to get camera pixels (u, v), so a
    different --side-scale does not change the geometry. Picking and the height ray use the same
    calibrated CameraModel as the 2-D monitor graph. Invalid rays / heights outside
    editor.input_z_range_m are passed on as None (the editor keeps its last valid preview).
    """

    CLICK_PX = 4.0       # press + release closer than this (displayed pixels) = a click, not a drag

    def __init__(self, cam: CameraModel, cfg, scale: float):
        self.cam, self.scale = cam, scale
        self.radius = cfg["editor"]["selection_radius_px"]
        self.z_range = tuple(cfg["editor"]["input_z_range_m"])
        self.pending: tuple | None = None    # (index, control point, x, y) pressed, not yet a drag

    def _uv(self, x, y):
        return x / self.scale, y / self.scale

    def press(self, session: Session, d, x, y) -> bool:
        """Pick the nearest path point. The drag starts only once the mouse moves (a click selects it)."""
        self.pending = None
        ok, why = session.can_edit()
        if not ok:
            session._emit(f"edit refused: {why}")
            return False
        u, v = self._uv(x, y)
        k = pick_index(session.path.points_m, self.cam, u, v, self.radius)
        if k is None:
            session._emit(f"no interior path point within {self.radius:.0f} px of the click")
            return False
        p = pixel_to_vertical(self.cam, u, v, session.path.points_m[k, :2], self.z_range)
        if p is None:
            session._emit("click ray gives no valid height on the point's vertical line")
            return False
        self.pending = (k, p, x, y)
        return True

    def move(self, session: Session, d, x, y) -> bool:
        if self.pending is not None:
            k, p, x0, y0 = self.pending
            if np.hypot(x - x0, y - y0) < self.CLICK_PX:
                return False
            self.pending = None
            if not session.edit_begin(k, p):
                return False
        if session.edit is None or not session.edit.dragging:
            return False
        u, v = self._uv(x, y)
        p = pixel_to_vertical(self.cam, u, v, session.edit.anchor_xy, self.z_range)
        return session.edit_update(p)

    def release(self, session: Session, d) -> bool:
        """Release after a drag ends it; release without moving selects the point for voice edits."""
        if self.pending is not None:
            k = self.pending[0]
            self.pending = None
            return session.point_select(k)
        return session.edit_end()

    def cancel(self, session: Session) -> bool:
        self.pending = None
        return session.edit_cancel()

    def path_point_display(self, session: Session, d, frac: float):
        P = session.path.points_m
        k = int(round(frac * (len(P) - 1)))
        u, v = self.cam.project(P[k])[0][0]
        return u * self.scale, v * self.scale


def viewer_pose(cam: CameraModel) -> tuple[np.ndarray, float, float, float]:
    """MuJoCo free-camera (lookat, distance, azimuth, elevation) looking like `cam` (a camera has no roll here)."""
    c = -cam.R_cw.T @ cam.t_cw                       # camera position, world
    f = cam.R_cw[2] / np.linalg.norm(cam.R_cw[2])     # optical axis (viewing direction), world
    s = -c[2] / f[2] if f[2] < -1e-3 else 0.8         # where the view meets the desk (z = 0)
    s = float(np.clip(s, 0.3, 2.0))
    # the viewer's field of view (45 deg) is narrower than the monitor camera's (60 deg): step back a bit
    # ... and look a little above the desk point so the arm (up to ~0.4 m) stays in view
    return c + s * f + [0.0, 0.0, 0.12], 1.6 * s, float(np.degrees(np.arctan2(f[1], f[0]))), \
        float(np.degrees(np.arcsin(f[2])))


def camera_list() -> str:
    """Connected V4L2 cameras, for the 'cannot open camera' message."""
    names = []
    for dev in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            names.append(f"{dev.name[5:]} = {(dev / 'name').read_text().strip()}")
        except OSError:
            pass
    return ("connected cameras: " + ", ".join(names) if names else "no camera is connected") + \
        " (an Astra shows up as 'Astra' / 'Orbbec'; check the USB cable)"


def screen_axes(cam: CameraModel) -> dict[str, np.ndarray]:
    """World unit vectors for voice directions, as seen on the monitor (the user looks at this view).

    up/down = world +-Z. right/left = camera right, flattened onto the desk. away/toward_me = camera
    viewing direction, flattened (away = into the screen). The camera is assumed not to look straight down.
    """
    R = np.asarray(cam.R_cw, float)          # rows: camera x (right), y (down), z (forward) in world
    def flat(v):
        v = np.array([v[0], v[1], 0.0])
        return v / np.linalg.norm(v)
    right, away = flat(R[0]), flat(R[2])
    up = np.array([0.0, 0.0, 1.0])
    return {"up": up, "down": -up, "right": right, "left": -right, "away": away, "toward_me": -away}


class LatestTextPusher:
    """Hands the newest viewer text to viewer.set_texts in its own thread (older pending texts are dropped)."""

    def __init__(self, viewer):
        self._viewer, self._latest, self._cv = viewer, None, threading.Condition()
        threading.Thread(target=self._run, name="viewer-texts", daemon=True).start()

    def put(self, text) -> None:
        with self._cv:
            self._latest = text
            self._cv.notify()

    def _run(self) -> None:
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self._latest is not None)
                text, self._latest = self._latest, None
            try:
                if not self._viewer.is_running():
                    return
                self._viewer.set_texts(text)
            except Exception:            # viewer closing: nothing left to show
                return


CALIB_LEVEL = {"verified": "ok", "checking": "warn", "not_visible": "warn", "no_config": "warn", "moved": "bad",
               "unverified": "bad"}


def calib_text(state: str, monitor) -> str:
    """One line about the real camera's calibration for the monitor badge and the voice status."""
    if state == "unverified":
        return "정합 미검증 보정 (리허설용): 화면 겹침을 믿지 마세요 · 3D 핀치 꺼짐"
    return monitor.text() if monitor is not None else ""


def draw_badge(canvas, text, layout, msg: str, level: str) -> None:
    """Calibration badge in the top-right corner of the camera image (green ok / orange warn / red bad)."""
    from .ui import blit, round_rect
    if not msg:
        return
    spr = text.sprite(msg, 14, (255, 255, 255))
    h, w = spr.shape[:2]
    x1, y0 = layout.img_w - 10, layout.top_h + 10
    color = {"ok": (60, 150, 60), "warn": (0, 140, 230), "bad": (40, 40, 210)}[level]
    round_rect(canvas, x1 - w - 16, y0, x1, y0 + h + 10, color, 0.85, r=8)
    blit(canvas, spr, x1 - w - 8, y0 + 5)


def start_stall_watch(beat: list, path, limit_s: float = 3.0) -> None:
    """Diagnosis: if the main loop does not come round for limit_s, dump every thread's stack to `path`
    (once per stall) and print a warning. Validation runs in a worker, so normal loops are << limit_s."""
    def run():
        dumped = False
        while True:
            time.sleep(0.5)
            gap = time.perf_counter() - beat[0]
            if gap > limit_s and not dumped:
                with open(path, "a") as f:
                    f.write(f"--- main loop stalled {gap:.1f} s at {time.strftime('%H:%M:%S')} ---\n")
                    f.flush()
                    faulthandler.dump_traceback(f, all_threads=True)
                print(f"WARNING: main loop stalled {gap:.1f} s - thread stacks saved to {path}", flush=True)
                dumped = True
            elif gap <= limit_s and dumped:
                print(f"main loop running again", flush=True)
                dumped = False
    threading.Thread(target=run, name="stall-watch", daemon=True).start()


def main(argv=None):
    faulthandler.enable(all_threads=True)      # a native crash (GLFW, OpenNI, MediaPipe) prints where it happened
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--target-r", type=float)
    ap.add_argument("--target-deg", type=float)
    ap.add_argument("--work-height", type=float, help="override target.work_height_m (failure demo)")
    ap.add_argument("--allow-unsafe-test-geometry", action="store_true",
                    help="let the path generator create TCP points below tcp_z_min (validation still blocks)")
    ap.add_argument("--script", help="scripted keys 't_s:KEY,...' for unattended smoke tests")
    ap.add_argument("--snapshot-dir", help="save side_cam frames on every state change")
    ap.add_argument("--camera", default="side_cam", choices=["side_cam", "side_cam_alt"],
                    help="fixed camera shown in the second window")
    ap.add_argument("--monitor-calib", help="camera calibration JSON for the monitor (default: the sim camera)")
    ap.add_argument("--video", help="real camera for the monitor: device index, 'auto' (the Astra colour camera found "
                    "by name, pac_vision camera.yaml rgb.name_match) or a video file (needs --calib)")
    ap.add_argument("--allow-unverified-calib", action="store_true",
                    help="rehearsal only: use a calibration that failed its checks, differs from the current "
                         "pac_vision settings or came from another camera/USB port. The overlay is shown as "
                         "UNVERIFIED and the 3-D pinch stays off")
    ap.add_argument("--side-pos", default="auto",
                    help="side_cam window top-left 'x,y' in screen pixels (auto: right edge of the primary monitor)")
    ap.add_argument("--side-scale", default="auto",
                    help="side_cam image scale over the 16:10 render (auto: 1.1 = about twice the old area, "
                         "reduced to fit the monitor)")
    ap.add_argument("--no-arrange", action="store_true",
                    help="do not move/resize the MuJoCo window next to side_cam (default: arrange, X11 only)")
    ap.add_argument("--voice-gateway", nargs="?", const="", metavar="SOCKET",
                    help="accept voice commands (쿠옹이) on a local socket; off by default")
    ap.add_argument("--hand", nargs="?", const=0, type=int, metavar="CAMERA",
                    help="hand input from a webcam (MediaPipe): pointing finger = click, pinch = drag "
                         "(default camera 0; needs requirements-hand.txt)")
    ap.add_argument("--no-hand-preview", action="store_true", help="hide the small webcam view")
    ap.add_argument("--canvas", action="store_true",
                    help="blank monitor: origin, x/y/z axes of the camera calibration, A, B and the path on white")
    ap.add_argument("--calib", metavar="CAM2BASE_YAML",
                    help="camera calibration in pac_vision cam2base.yaml format (default with --canvas: "
                         "configs/cam2base_mock.yaml, made from the sim camera if missing)")
    ap.add_argument("--start", metavar="X,Y,Z", help="A: end-effector start in the calibration frame, m "
                    "(+X forward, +Y left, +Z up, origin = robot base on the desk)")
    ap.add_argument("--goal", metavar="X,Y", help="B: target point on the desk in the calibration frame, m")
    ap.add_argument("--user-side", choices=("right", "front", "behind", "left"), default="right",
                    help="where the user stands around the robot; voice directions (right, away, ...) are fixed "
                         "to the calibration axes as seen from there (default right: at -Y, facing +Y, "
                         "so right = +X, away = +Y)")
    ap.add_argument("--swap-left-right", action="store_true",
                    help="swap the voice directions right and left only (up/down/away/toward stay as --user-side)")
    ap.add_argument("--robot-pose", metavar="X,Y,YAW_DEG",
                    help="where the robot base centre stands in the calibration frame (m on the desk) and which "
                         "way it faces (deg, counter-clockwise from +X seen from above); default: robot_pose.yaml "
                         "next to --calib if it exists, else 0,0,0 = on the calibration origin facing +X")
    ap.add_argument("--stations", nargs="?", const="", metavar="YAML",
                    help="demo stations from ArUco markers (stations.py): HOME -> S (basket, 6) by itself, then B (7) / "
                         "C (8) chosen by voice, straight paths edited, ends at S. Default file: stations.yaml next to "
                         "--calib")
    ap.add_argument("--real-arm", action="store_true",
                    help="plan for the real arm and run approved paths on it: python -m manipulator_control pinch "
                         "(<repo>/manipulator_control_ws) must be running and at HOME; its robot.yaml sets HOME and the "
                         "joint ranges, the simulator shows the measured pose (real_arm.py, arm_link.py)")
    ap.add_argument("--robot-yaml", metavar="PATH",
                    help="--real-arm: the arm program's config/robot.yaml (default <repo>/manipulator_control_ws/config/robot.yaml, or $MANIPULATOR_CONTROL_WS)")
    ap.add_argument("--rehearse", action="store_true",
                    help="with --real-arm: the real arm's profile (HOME, ranges, its side of the desk) without the arm: "
                         "the simulator executes as usual (rehearsal on the real camera image, nothing moves)")
    ap.add_argument("--arm-link", metavar="SOCKET",
                    help="--real-arm: the arm program's socket (default $XDG_RUNTIME_DIR/manipulator_control/arm.sock)")
    ap.add_argument("--ui", choices=("web", "cv"), default="web",
                    help="monitor: web = full-screen browser window (webui.py, Figma design 2026-10-28), cv = the "
                         "OpenCV side_cam window (progress strip, bubbles, button bar)")
    ap.add_argument("--browser", choices=("auto", "chrome", "firefox", "none"), default="auto",
                    help="--ui web: which browser opens the monitor (none: only print the URL)")
    ap.add_argument("--browser-mode", choices=("window", "kiosk"), default="window",
                    help="--ui web: a normal resizable window (default) or full screen without resizing")
    ap.add_argument("--browser-size", default="1600x1000", metavar="WxH",
                    help="--ui web, window mode: first window size in px (the page scales with the window)")
    ap.add_argument("--depth", action="store_true",
                    help="Astra depth (pac_vision, OpenNI2) for the hand: a pinch moves the grabbed point by the "
                         "hand's 3-D motion, 1:1 (needs --hand, --video and a real pac_vision --calib)")
    a = ap.parse_args(argv)
    pose_file = Path(a.calib).resolve().parent / "robot_pose.yaml" if a.calib else None
    if a.robot_pose or (pose_file is not None and pose_file.exists()):
        from .canvas import set_robot_pose
        try:
            if a.robot_pose:
                rx, ry, ryaw = [float(v) for v in a.robot_pose.split(",")]
                src = "--robot-pose"
            else:
                import yaml
                doc = yaml.safe_load(pose_file.read_text(encoding="utf-8")) or {}
                rx, ry, ryaw = float(doc["x_m"]), float(doc["y_m"]), float(doc["yaw_deg"])
                src = str(pose_file)
        except (ValueError, KeyError, TypeError):
            ap.error("--robot-pose: X,Y,YAW_DEG (three numbers); robot_pose.yaml: x_m, y_m, yaw_deg")
        set_robot_pose(rx, ry, ryaw)
        print(f"robot base in the calibration frame: x={rx:.3f} y={ry:.3f} m, facing {ryaw:.1f} deg ({src})",
              flush=True)
        if a.real_arm and abs((ryaw + 90.0 + 180.0) % 360.0 - 180.0) > 2.0:
            ap.error(f"--real-arm: the real arm works on model -Y, so its pose must face -90 deg (robot_pose.yaml "
                     f"yaw_deg: -90, y_m: +0.003 for the J1 axis offset), got {ryaw:.1f} ({src})")
    elif a.real_arm and a.calib:
        ap.error("--real-arm with a camera calibration needs robot_pose.yaml next to it (yaw_deg: -90, y_m: 0.003)")
    if a.depth and (a.hand is None or not a.video or not a.calib):
        ap.error("--depth needs --hand, --video (the Astra colour camera) and --calib (pac_vision cam2base.yaml)")
    calib_trust = ""            # "" = no real camera calibration in use; "file_ok" / "unverified" (see pac_calib)
    if a.calib:
        from .pac_calib import calibration_problems, find_video_device, identity_problems, is_pac_vision_file
        real = is_pac_vision_file(a.calib)
        if real and not a.video and not a.canvas:
            ap.error("--calib with a real pac_vision calibration needs --video (the Astra colour camera): it would "
                     "be drawn over the simulator's own camera image, where it cannot match")
        if a.video and not real:
            ap.error("--video (real camera) needs a real pac_vision calibration, not the mock / sim camera file")
        if real and a.video:
            problems = calibration_problems(a.calib)
            if a.video == "auto" or a.video.isdigit():
                if a.video == "auto":
                    from pac_vision.config import load_config as pv_load_config
                    match = pv_load_config(Path(a.calib).resolve().parent, ("camera",))["camera"]["rgb"].get(
                        "name_match", "Astra")
                    dev = find_video_device(str(match))
                    if dev is None:
                        ap.error(f"--video auto: no camera named like '{match}' gives frames. {camera_list()}")
                    a.video = str(dev)
                    print(f"video: /dev/video{dev} (auto, name contains '{match}')", flush=True)
                problems += [f"camera identity: {p} - same camera on the same USB port as the calibration?"
                             for p in identity_problems(a.calib, int(a.video))]
            if a.video.isdigit():          # one program owns the Astra: the sim, or a pac_vision script, not both
                from pac_vision.devices import CameraBusy, camera_lock
                try:
                    cam_lock = camera_lock("pinch_sim (simulator)")   # noqa: F841 - held until the app exits
                except CameraBusy as exc:
                    ap.error(str(exc))
            for p in problems:
                print(f"CALIBRATION: {p}", flush=True)
            if problems and not a.allow_unverified_calib:
                ap.error("the calibration is not usable for this camera (see CALIBRATION lines above); calibrate "
                         "again, or --allow-unverified-calib for a rehearsal (overlay unverified, no 3-D pinch)")
            calib_trust = "unverified" if problems else "file_ok"

    cfg = load_config()
    arm = None
    if a.real_arm:
        from .real_arm import RealArm, apply_profile
        try:
            cfg = apply_profile(cfg, RealArm.load(a.robot_yaml))
        except ValueError as exc:
            ap.error(str(exc))
    if a.work_height is not None:
        if a.work_height < cfg["path"]["tcp_z_min_m"] and not a.allow_unsafe_test_geometry:
            ap.error("--work-height below tcp_z_min_m needs --allow-unsafe-test-geometry")
        cfg = with_overrides(cfg, target={"work_height_m": a.work_height})
    m, idx, sha = load_scene(cfg)
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    if a.rehearse and not a.real_arm:
        ap.error("--rehearse goes with --real-arm")
    if a.real_arm and a.rehearse:
        print("real arm profile, rehearsal: no link, the simulator executes (nothing real moves)", flush=True)
    elif a.real_arm:
        from .arm_link import ArmLink
        from .real_arm import hello as arm_hello
        try:
            arm = ArmLink(arm_hello(cfg, sha, {"config_sha": config_sha(cfg)}), a.arm_link).connect()
        except ConnectionError as exc:
            ap.error(f"real arm: {exc}")
        st = arm.state.get("state")
        if st not in ("LINK_WAIT", "ARM_CONFIRM", "ARMED", "HOLDING") or arm.q_meas() is None:
            arm.close()
            ap.error(f"real arm is in {st}: start the simulator once the arm program says 시뮬 연결 대기 (after HOME)")
        d.qpos[idx.qpos] = arm.q_meas()            # the simulator starts where the real arm is
        mujoco.mj_forward(m, d)
        print(f"real arm: {st}, q = {np.degrees(arm.q_meas()).round(2).tolist()} deg ({arm.address})", flush=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    (ROOT / "logs").mkdir(exist_ok=True)
    session = Session(m, idx, sha, cfg, d, seed=a.seed, allow_unsafe=a.allow_unsafe_test_geometry,
                      log_path=ROOT / f"logs/session_{stamp}.jsonl")
    session.real_arm = arm
    print(f"model {sha[:12]}  MuJoCo {mujoco.__version__}  log logs/session_{stamp}.jsonl")
    if a.stations is not None:
        pass                                             # the first path is HOME -> S once the markers are read
    elif a.target_r is not None:
        session.cmd_fixed_target(a.target_r, a.target_deg or 0.0, d)
    else:
        session.cmd_new_target(d, count_as_new=False)   # first seeded target at start: not a New press

    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    renderer = mujoco.Renderer(m, H, W)
    # monitor camera model: a real calibration file, or the exact model of the sim camera
    cam_model = CameraModel.load(a.monitor_calib) if a.monitor_calib else \
        CameraModel.from_mujoco(m, d, a.camera, W, H)
    ab_start = None
    if a.canvas or a.calib or a.start or a.goal:
        from .canvas import cal_to_world, load_cam2base, world_to_cal, write_mock_cam2base
        if a.canvas or a.calib:
            calib = Path(a.calib) if a.calib else ROOT / "configs" / "cam2base_mock.yaml"
            if not calib.exists() and not a.calib:
                write_mock_cam2base(calib, CameraModel.from_mujoco(m, d, a.camera, W, H))
                print(f"mock camera calibration written: {calib} (replace with the real cam2base.yaml)", flush=True)
            from .pac_calib import load_camera
            cam_model = load_camera(calib, load_cam2base)     # real pac_vision result or our MOCK file
        if a.start or a.goal:
            parse = lambda txt, n: [float(v) for v in txt.split(",")][:n]
            a_cal = parse(a.start, 3) if a.start else world_to_cal(session.robot_state(d).tcp.p_m)
            b_cal = (parse(a.goal, 2) + [0.0])[:3] if a.goal else world_to_cal(session.target.floor_point_m)
            ok, why = session.cmd_ab(cal_to_world(a_cal), cal_to_world(b_cal), d)
            if not ok:
                ap.error(f"--start/--goal: {why}")
            print(f"A = {np.round(a_cal, 3).tolist()}, B = {np.round(b_cal, 3).tolist()} (calibration frame, m)",
                  flush=True)
        ab_start = session.ab[0] if session.ab is not None else session.robot_state(d).tcp.p_m.copy()
    hand_cam = None
    # camera mounted upside down: the calibration was made on 180-degree turned frames (pac_vision camera.rotate_180),
    # so the live frames are turned the same way (the depth stream turns itself, AstraDepth.rotate_180)
    rotate_180 = bool(getattr(cam_model, "conv", None) and (cam_model.conv.info.get("camera") or {}).get("rotate_180"))
    if rotate_180:
        print("camera: mounted upside down (rotate_180): colour and depth frames turned by 180 deg", flush=True)
    if a.video:
        if not (a.monitor_calib or a.calib):
            ap.error("--video needs --calib (or --monitor-calib): the path is projected with the real camera's "
                     "calibration")
        if a.hand is not None:
            # real camera mode with hands: open the camera ONCE; the same frames are the monitor image and the
            # hand tracking input (the hand is in the picture, so the finger is drawn where it really is)
            from .hand_input import HandCamera, HandConfig
            try:
                hand_cam = HandCamera(HandConfig(mirror=False, rotate_180=rotate_180), device=str(a.video),
                                      set_size=False)
            except RuntimeError as exc:
                renderer.close()                  # no EGL clean-up noise after the message
                ap.error(f"{exc}. {camera_list()}")

            class _SharedSource:
                def prepare(self, d_):
                    pass

                def frame(self):
                    t_end = time.monotonic() + 5.0
                    f = hand_cam.frame()
                    while f is None and time.monotonic() < t_end:
                        time.sleep(0.02)
                        f = hand_cam.frame()
                    if f is None:
                        raise RuntimeError(f"no frame from camera {a.video}")
                    return f
            source = _SharedSource()
        else:
            try:
                source = VideoFrameSource(a.video, rotate_180=rotate_180)
            except RuntimeError as exc:
                renderer.close()
                ap.error(f"{exc}. {camera_list()}")
        f0 = source.frame()
        if (f0.shape[1], f0.shape[0]) != (cam_model.width, cam_model.height):
            cam_model = cam_model.scaled_to(f0.shape[1], f0.shape[0])
    elif a.canvas:
        from .canvas import blank_canvas
        canvas_img = blank_canvas(cam_model)

        class _CanvasSource:                       # the monitor shows a blank canvas instead of a camera
            def prepare(self, d_):
                pass

            def frame(self):
                return canvas_img.copy()
        source = _CanvasSource()
    else:
        source = SimFrameSource(m, renderer, a.camera)
    print(f"monitor: {cam_model.name} {cam_model.width}x{cam_model.height}, "
          f"frames from {'video ' + a.video if a.video else 'sim camera ' + a.camera}", flush=True)
    marker_mon = None
    if calib_trust:
        # the file passing its checks is not enough: fixed markers (markers.yaml next to the calibration) must agree
        # on the live image before the overlay / 3-D pinch count as calibrated
        from .pac_calib import MarkerMonitor
        marker_mon = MarkerMonitor(cam_model, Path(a.calib).resolve().parent / "markers.yaml")
        session.calib_state = "unverified" if calib_trust == "unverified" else marker_mon.state
        print(f"calibration: {calib_text(session.calib_state, marker_mon)}", flush=True)
    # demo stations: read the markers from the first camera frames (or base_m without a camera), then HOME -> S
    station_cfg = station_reader = None
    base_pending = False                 # HOME -> S waits for the armed arm, then 쿠옹이 asks before it runs
    if a.stations is not None:
        from . import stations as stations_mod
        from .canvas import cal_to_world
        spath = Path(a.stations) if a.stations else (Path(a.calib).resolve().parent / "stations.yaml" if a.calib
                                                     else None)
        if spath is None or not spath.exists():
            ap.error(f"--stations: no station file ({spath or 'give a path or --calib'})")
        station_cfg = stations_mod.load_config(spath)
        if a.video and getattr(cam_model, "conv", None) is not None:
            station_reader = stations_mod.StationReader(station_cfg, cam_model, cal_to_world)
            print(f"stations: reading markers {[s.marker for s in station_cfg.stations.values()]} from the camera "
                  f"({spath})", flush=True)
        elif stations_mod.from_base(station_cfg, cal_to_world):
            print(f"stations: no camera - base_m from {spath}", flush=True)
        else:
            ap.error("--stations needs the camera (--video with --calib) or base_m for every station")

    def stations_ready() -> None:
        nonlocal base_pending
        from .canvas import world_to_cal
        rx, ry = world_to_cal([0.0, 0.0, 0.0])[:2]     # robot base in the calibration frame
        print("stations located:\n" + "\n".join(stations_mod.describe(station_cfg, (rx, ry))), flush=True)
        session.set_stations(station_cfg.stations, dip_m=station_cfg.dip_m)
        with session.acting("sim"):
            ok, code = session.cmd_station("S", d)
        base_pending = ok
        if not ok:
            print(f"stations: HOME -> S path failed ({code}): {session.message}", flush=True)

    if station_cfg is not None and station_reader is None:      # no camera: base_m positions are known already
        stations_ready()

    mons = monitors()
    web = browser = alerts = ui_dispatch = None
    if a.ui == "web":
        # browser monitor (webui.py): only the camera image (+ path, hand) is a frame; everything around it is HTML.
        # The frame is sent at about 1280 px wide; mouse positions come back as fractions of it.
        from .voice_gateway import Dispatcher
        from .webui import Alerts, WebUI, open_browser
        layout = Layout(cam_model.width, cam_model.height, min(2.0, max(1.0, 1280 / cam_model.width)), top_h=0, bar_h=0)
        alerts = Alerts(notice_text)
        session.listeners.append(lambda ev: alerts.on_event(ev, session))
        ui_dispatch = Dispatcher()                 # chips / "검증 후 이동": the voice protocol's commands and checks
        web = WebUI().start()
        bw, bh = (int(v) for v in a.browser_size.lower().split("x"))
        browser = open_browser(web.url, a.browser, mons[0] if mons else None, a.browser_mode, (bw, bh))
        win_xy = (0, 0)
        print(f"web monitor: http://127.0.0.1:{web.port}/ (frame {layout.img_w}x{layout.img_h}, "
              + (f"{a.browser} browser, {a.browser_mode})" if browser is not None else f"open {web.url})"), flush=True)
    else:
        # 16:10 camera image (render W x H, scaled), progress strip above, buttons below (ui.py)
        layout = auto_layout(cam_model.width, cam_model.height, a.side_scale, mons[0] if mons else None)
        win_xy = window_pos(layout, a.side_pos, mons[0] if mons else None)
        # GUI_NORMAL: no Qt zoom/pan toolbar (zooming the image would break the pixel -> ray mapping)
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
        cv2.moveWindow(WIN, *win_xy)
        # the window manager may place the MuJoCo window over it; keep the button bar reachable
        cv2.setWindowProperty(WIN, cv2.WND_PROP_TOPMOST, 1)
        print(f"side_cam window {layout.width}x{layout.height} at {win_xy} (image {layout.img_w}x{layout.img_h}, "
              f"scale {layout.scale}, monitors {mons})", flush=True)
    ui = MonitorUI(layout)
    notices = Notices()
    session.listeners.append(lambda ev: notices.push(notice_text(ev, session)))
    clicks: list[str] = []
    mouse: list[tuple] = []          # ("press"|"move"|"release"|"cancel", x, y) in displayed-image pixels
    mouse_editor = MouseEditor(cam_model, cfg, layout.scale)
    # voice directions: fixed to the calibration axes as the user standing at --user-side sees them
    # (not the camera image, so moving the camera or re-calibrating does not turn "right" around)
    from .canvas import user_axes
    session.screen_axes = user_axes(a.user_side)
    if a.swap_left_right:
        ax = session.screen_axes
        ax["right"], ax["left"] = ax["left"], ax["right"]
        print("voice directions: right and left swapped (--swap-left-right)", flush=True)
    hand_ctrl = hand_editor = depth_src = None
    if a.hand is not None:
        from . import hand_input
        from .hand_input import HandCamera, HandConfig, HandController
        hand_input._USER_SIDE[0] = a.user_side
        hand_input._SWAP_LR[0] = bool(a.swap_left_right)
        hcfg = HandConfig(camera=a.hand, mirror=hand_cam is None)       # a separate webcam is a mirror view
        if hand_cam is None:
            hand_cam = HandCamera(hcfg)
        hand_depth = None
        if a.depth:
            from .hand_depth import AstraDepthSource, HandDepth
            from .pac_calib import PacVisionCamera
            if not isinstance(cam_model, PacVisionCamera):
                ap.error("--depth needs a real pac_vision calibration (--calib .../pac_vision/config/cam2base.yaml)")
            try:
                depth_src = AstraDepthSource(a.calib)
            except Exception as exc:          # noqa: BLE001 - OpenNI / device errors: say what failed
                ap.error(f"Astra depth did not open: {exc}")
            hand_depth = HandDepth(cam_model)
            print("hand depth: Astra (pinch = 3-D move, 1:1)", flush=True)
        hand_ctrl = HandController(hcfg, depth=hand_depth)
        hand_editor = MouseEditor(cam_model, cfg, layout.scale)      # same geometry as the mouse ...
        hand_editor.radius = hcfg.select_radius_px                  # ... with a wider pick radius
        session.hand_mode = "3d" if hand_depth is not None else "2d"
    if depth_src is not None:
        # key G / voice "이 물체로 목표 바꿔 줘": the object on the desk in the depth frame the sim already reads
        from .canvas import cal_to_world, world_to_cal
        from .vision_goal import VisionGoalConfig, choose, find_objects
        vg_cfg = VisionGoalConfig()

        def vision_target():
            if session.calib_state != "verified":
                return None, "calib_unverified"
            depth_now, t_d = depth_src.latest()
            if depth_now is None or time.monotonic() - t_d > 0.5:
                return None, "no_depth"
            base_xy = np.asarray(world_to_cal([0.0, 0.0, 0.0]), float).reshape(-1)[:2]   # robot base, base_ref
            objs = find_objects(cam_model.conv, depth_now, base_xy, vg_cfg)
            near = None                                  # a pointing finger chooses among several objects
            g = hand_ctrl.gesture if hand_ctrl is not None else None
            if g is not None and g.kind == "point" and hand_ctrl.cursor_px is not None:
                u, v = hand_ctrl.cursor_px[0] / layout.scale, hand_ctrl.cursor_px[1] / layout.scale
                near = np.asarray(world_to_cal(cam_model.desk_point(u, v)), float).reshape(-1)[:2]
            obj, why = choose(objs, near)
            print(f"camera target: {len(objs)} object(s) " + (f"-> top {np.round(obj.top_base, 3).tolist()} m "
                  f"(base_ref, {obj.n_points} points)" if obj is not None else f"-> none ({why})"), flush=True)
            return (None, why) if obj is None else (cal_to_world(obj.top_base), "")
        session.vision_target = vision_target
        print(f"hand input: camera {a.hand} (point = select, pinch = drag "
              f"{'in 3-D with depth' if hand_depth is not None else 'height only (2-D mode, no depth)'}; "
              "only in edit mode: E or voice)", flush=True)

    def on_mouse(event, x, y, flags, param):
        xi, yi = layout.to_image(x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            k = button_at(x, y, layout.width, layout.height)
            if k:
                clicks.append(k)
            elif layout.region(x, y) == "image" and not ui.hit_ui(xi, yi):   # bubbles / notice: not an edit
                mouse.append(("press", xi, yi))
        elif event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON:
            if mouse and mouse[-1][0] == "move":
                mouse[-1] = ("move", xi, yi)          # coalesce: only the latest position matters
            else:
                mouse.append(("move", xi, yi))
        elif event == cv2.EVENT_LBUTTONUP:
            mouse.append(("release", xi, yi))
        elif event == cv2.EVENT_RBUTTONDOWN:
            mouse.append(("cancel", xi, yi))

    if web is None:
        cv2.setMouseCallback(WIN, on_mouse)
    gateway = VoiceGateway(a.voice_gateway or None, ROOT / f"logs/voice_gateway_{stamp}.jsonl").start(session) \
        if a.voice_gateway is not None else None
    beat = [time.perf_counter()]
    start_stall_watch(beat, ROOT / f"logs/stall_{stamp}.txt")
    script = parse_script(a.script)
    snap_dir = (ROOT / a.snapshot_dir) if a.snapshot_dir else None
    if snap_dir:
        snap_dir.mkdir(parents=True, exist_ok=True)
    render_dt = 1.0 / cfg["app"]["render_hz"]
    sync_log: list[float] = []       # colour capture -> matched depth frame time (s), to set max_skew_s on site
    frame, last_state, last_render = 0, None, 0.0
    lag_resets = 0
    app_start = time.perf_counter()

    # web monitor: the 3-D view is a small panel in the page (sim_view.py), not a GLFW window beside it
    if web is not None:
        from .sim_view import SimView
        view_cm = SimView(m)
    else:
        view_cm = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
    with view_cm as viewer:
        sim_view = viewer if web is not None else None
        texts = LatestTextPusher(viewer)
        # start the 3-D view from where the user stands, so "right" (a user direction) is right on screen;
        # the user can still rotate the 3-D view with the mouse
        from .canvas import user_view
        look, dist, az, el = user_view(a.user_side, float(cfg["target"].get("front_y", 1.0)))
        viewer.cam.lookat[:] = look
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = dist, az, el
        wall0, sim0 = time.perf_counter(), d.time
        while viewer.is_running():
            now = time.perf_counter()
            beat[0] = now
            target_sim = sim0 + (now - wall0)
            with viewer.lock():
                n = 0
                while d.time < target_sim and n < 200:
                    session.physics_step(d)
                    n += 1
            if n == 200:            # cannot keep real time (e.g. during validation): resync, never skip steps
                wall0, sim0 = time.perf_counter(), d.time
                lag_resets += 1
            session.watch(d)
            if now - last_render < render_dt:
                time.sleep(0.001)
                continue
            last_render = now
            ov = build_overlay(session, d)
            frame += 1
            if frame in (10, 60) and not a.no_arrange and web is None:   # the viewer window exists by now; once more in case
                print("window arrange:", arrange_viewer("MuJoCo : 6dof scene", win_xy[0],   # the WM moved it
                                                        mons[0] if mons else None), flush=True)
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                add_overlay(viewer.user_scn, ov)
                source.prepare(d)                    # sim frame: robot + floor only, no 3-D path
                t_sim = float(d.time)
            # set_texts waits for the viewer's render thread to take the texts. Called while holding
            # viewer.lock() that thread can never get the lock -> both stop (stall 2026-10-12). Even outside the
            # lock it waits for seconds when the render thread itself is held up (GNOME Wayland, 2026-10-14:
            # the GLFW window covered by side_cam). So a separate thread pushes the latest text; the main loop
            # (clicks, voice, validation, execution) never waits for the 3-D window.
            texts.put((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                       f"{session.state}\ncommands: side_cam window" + (f"\n{peak_text(ov)}" if ov.peak is not None else ""),
                       f"frame {frame}  t={t_sim:.2f}s\n " + ("\n " if ov.peak is not None else "")))
            viewer.sync()
            if sim_view is not None and sim_view.due(now):    # the page's 3-D panel turns around the path's middle
                sim_view.follow(ov.path)
                web.push_sim(sim_view.render(d, ov))
            # path etc. as a projected 2-D graph; on a real camera image only the path, A, B and the axes
            raw = source.frame()
            if station_reader is not None and station_reader.update(raw, time.monotonic()):
                stations_ready()
            if base_pending and session.state == "GENERATED" and session.move_kind == "to_base" \
                    and (arm is None or not arm.gate()) and (gateway is None or gateway.connected):
                # armed (terminal y) and 쿠옹이 listening: it asks "설거지 바구니로 이동할까요?" and only the yes
                # (validate_and_move) validates and runs HOME -> S. Without voice: the page's 검증 후 이동 button.
                base_pending = False
                with session.acting("sim"):
                    session._emit("ready to move HOME -> S: waiting for the user's yes", "stations_ready")
            if marker_mon is not None and marker_mon.update(raw, time.monotonic()) and calib_trust == "file_ok":
                session.calib_state = marker_mon.state
                session._emit(f"camera calibration {marker_mon.state}: {marker_mon.text()}", "calib",
                              calib=marker_mon.state)
            img = draw_overlay_2d(raw, cam_model, ov, line_px=5 if web is not None else 3, minimal=bool(a.video))
            if a.canvas or (a.video and a.calib):
                from .canvas import draw_ab, draw_axes
                if a.video:
                    draw_axes(img, cam_model)
                draw_ab(img, cam_model, ab_start, session.target.floor_point_m
                        if session.target is not None and not session.stations else None)
                if session.stations:
                    stations_mod.draw_stations(img, cam_model, session.stations, session.visited, ui.text,
                                               current=session.station_goal)
            hint = "" if gateway is None else (gateway.client_line if gateway.connected else "쿠옹이 연결 대기")
            if arm is not None:
                hint = arm.badge() + (f"  |  {hint}" if hint else "")
            buttons = [(k, label, ENABLED[k](session.state)) for k, label in BUTTONS]
            if web is None:
                img = ui.render(img, session, buttons, notices.current(), gateway.chat if gateway else [], hint,
                                protect_points(ov, cam_model, layout.scale))      # bubbles/notices never cover these
            else:
                img = cv2.resize(img, (layout.img_w, layout.img_h), interpolation=cv2.INTER_LINEAR)
            if hand_ctrl is not None:
                from .hand_input import draw_preview
                lm, _, _, _, hframe = hand_cam.latest()
                shared = bool(a.video)                  # the hand is in the monitor image itself
                if not a.no_hand_preview and not shared:
                    draw_preview(img, hframe, lm, layout.top_h)
                hand_ctrl.draw(img, layout.top_h, cam_model, layout.scale,
                               landmarks=lm if shared and not a.no_hand_preview else None,
                               img_wh=(layout.img_w, layout.img_h), text=ui.text)
            if web is not None:
                from .webui import ui_state
                calib = None if not session.calib_state else {
                    "state": session.calib_state, "text": calib_text(session.calib_state, marker_mon),
                    "level": CALIB_LEVEL.get(session.calib_state, "warn")}
                web.push_frame(img)
                web.push_state(ui_state(
                    session, alerts=alerts, chat=list(gateway.chat) if gateway else [], hint=hint,
                    voice_connected=bool(gateway is not None and gateway.connected), calib=calib,
                    arm_badge=arm.badge() if arm is not None else "", tcp=ov.tcp, frame_wh=(layout.img_w, layout.img_h),
                    enabled={k: bool(ENABLED[k](session.state)) for k, _ in BUTTONS}))
            else:
                if session.calib_state:
                    draw_badge(img, ui.text, layout, calib_text(session.calib_state, marker_mon),
                               CALIB_LEVEL.get(session.calib_state, "warn"))
                cv2.imshow(WIN, img)
            if snap_dir and session.state != last_state:
                cv2.imwrite(str(snap_dir / f"f{frame:05d}_{session.state}.png"), img)
            last_state = session.state
            actions = []
            if web is None:
                key = cv2.waitKey(1) & 0xFF
                keys = [chr(key).upper()] if key not in (255,) and 32 <= key < 127 else []
            else:                            # browser: keys, mouse on the image (fractions of the frame), actions
                keys = []
                for ev in web.drain():
                    if ev["type"] == "key":
                        keys.append(ev["key"])
                    elif ev["type"] == "mouse":
                        pt = (ev["kind"], ev["u"] * layout.img_w, ev["v"] * layout.img_h)
                        if pt[0] == "move" and mouse and mouse[-1][0] == "move":
                            mouse[-1] = pt                # coalesce: only the latest position matters
                        else:
                            mouse.append(pt)
                    else:
                        actions.append(ev)
            keys += clicks
            clicks.clear()
            while script and script[0][0] <= now - app_start:
                item = script.pop(0)[1]
                if item.startswith("DRAG:") and session.path is not None:
                    _, frac, dx, dy = item.split(":")
                    x0, y0 = mouse_editor.path_point_display(session, d, float(frac))
                    mouse.append(("press", x0, y0))
                    mouse.extend(("move", x0 + float(dx) * i / 10, y0 + float(dy) * i / 10) for i in range(1, 11))
                    mouse.append(("release", x0 + float(dx), y0 + float(dy)))
                elif item.startswith("CLICK:") and session.path is not None:
                    x0, y0 = mouse_editor.path_point_display(session, d, float(item.split(":")[1]))
                    mouse.extend([("press", x0, y0), ("release", x0, y0)])
                elif not item.startswith(("DRAG:", "CLICK:")):
                    keys.append(item)
            # mouse edits: one queued event per rendered frame so the drag preview is visible
            if mouse:
                kind, mx, my = mouse.pop(0)
                with session.acting("mouse"):
                    if kind == "press":
                        mouse_editor.press(session, d, mx, my)
                    elif kind == "move":
                        mouse_editor.move(session, d, mx, my)
                    elif kind == "release":
                        mouse_editor.release(session, d)
                    else:
                        mouse_editor.cancel(session)
            for act in actions:              # browser buttons: keys of the same path, or a voice-protocol command
                from .webui import apply_action
                if act["name"] == "viewport":
                    print(f"monitor page {act.get('w')}x{act.get('h')} px", flush=True)
                if act["name"] == "orbit":   # drag on the page's 3-D panel
                    if sim_view is not None:
                        sim_view.orbit(act.get("dx", 0.0), act.get("dy", 0.0))
                    continue
                more, why = apply_action(session, d, act, ui_dispatch)
                keys += more
                if why:
                    alerts.toast("요청을 처리하지 않았어요", why)
            for k in keys:
                if k == "Q":
                    viewer.close()
                    break
                print(f"key {k} ({KOREAN[session.state]})", flush=True)
                with session.acting("key"):
                    session.handle_key(k, d)
            if hand_ctrl is not None:        # hand: same MouseEditor and Session methods as the mouse
                lm, t_hand, cw, ch, _ = hand_cam.latest()       # t_hand = capture time of that colour frame
                t_now = time.monotonic()
                depth_mm, depth_why = None, ""
                if depth_src is not None:                       # the depth frame of the same moment, or why not
                    from .hand_depth import pick_depth
                    depth_mm, depth_why, skew = pick_depth(depth_src.recent(), t_hand, t_now, hand_ctrl.depth.cfg)
                    if skew is not None:
                        sync_log.append(skew)
                with session.acting("hand"):
                    hand_ctrl.step(lm if t_now - t_hand < 0.3 else None, t_now, cw, ch,
                                   layout.img_w, layout.img_h, hand_editor, session, d, depth_mm=depth_mm,
                                   depth_reason=depth_why)
            if gateway is not None:          # voice requests: same thread, same Session methods, after keys
                gateway.drain(session, d)
            if arm is not None:              # real arm: ping, measured pose (twin), ack / result, lost link
                msgs = arm.drain()
                with viewer.lock():
                    session.on_arm(d, msgs)
            if web is None and cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                viewer.close()
            if browser is not None and browser.poll() is not None:   # the monitor window was closed
                viewer.close()
        # the run's summary before the 3-D window is torn down: GLFW's terminate has crashed the process at exit
        # (2026-10-28, Wayland, twice), and the summary must not be lost with it
        summary = {"frames": frame, "lag_resets": lag_resets, "final_state": session.state,
                   "last_run": session.last_run, "events": len(session.events)}
        if sync_log:
            s_ms = np.abs(np.array(sync_log)) * 1000
            summary["rgb_depth_skew_ms"] = {"n": int(s_ms.size), "p50": round(float(np.percentile(s_ms, 50)), 1),
                                            "p95": round(float(np.percentile(s_ms, 95)), 1),
                                            "max": round(float(s_ms.max()), 1)}
        if marker_mon is not None and marker_mon.result is not None:
            summary["markers_last"] = marker_mon.result.to_dict()
        print(json.dumps(summary, ensure_ascii=False, default=str), flush=True)
    if gateway is not None:
        gateway.close()
    if web is not None:
        web.close()
        if browser is not None and browser.poll() is None:
            browser.terminate()
    if arm is not None:
        arm.close()
    if hand_cam is not None:
        hand_cam.close()
    if depth_src is not None:
        depth_src.close()
    session.close()
    if web is None:
        cv2.destroyAllWindows()
    return summary


if __name__ == "__main__":
    main()
