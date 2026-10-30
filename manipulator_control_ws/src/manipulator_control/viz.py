"""SIM window of the real STATE (kinematics only: qpos -> mj_kinematics, no physics step) with the EE gizmo.

grey solid  = measured arm (angles from the encoders)        - pale blue when UNCALIBRATED (relative only)
orange      = servo internal target (Position Trajectory 140) - only while torque is on
green       = goal (reference pose while waiting, HOME while moving, IK solution in TELEOP)
During ALIGN (calibrate): green = reference pose (it sweeps the + direction during the direction
check), yellow = the joint to align now, grey = joints already held; joints not aligned yet are not drawn.
TELEOP: the EE target is a gizmo (gizmo.py): drag an arrow = move along that axis, drag a ring = rotate about it,
one handle per drag; the axes are the base axes (small triad on the floor) or the tool's own (control panel). Empty space: left drag =
orbit, right drag = pan, wheel / middle drag = zoom (Shift: horizontal variants), as in MuJoCo's viewer.
The window is our own GLFW loop (the passive viewer does not hand mouse clicks to the program). The overlay is
ASCII (MuJoCo's font has no Hangul); prompts and keys are in the terminal. Closing the window asks the program
to quit through the normal exit path.
"""
from __future__ import annotations

import time

import mujoco
import numpy as np

from .gizmo import AXIS_RGB, Gizmo, ScreenCam, frame_axes
from .model import ARM_BODIES, ARMS, load_viz
from .panel import mat_to_quat, quat_to_mat

MEASURED = (0.72, 0.74, 0.78, 1.0)
UNCALIBRATED = (0.55, 0.65, 0.95, 0.75)
CURRENT = (0.98, 0.78, 0.15, 1.0)
CURRENT_GHOST = (0.98, 0.78, 0.15, 0.6)
SEND_HZ = 30.0          # gizmo drag -> TARGET events
HOLD_AFTER_DRAG_S = 0.3  # keep showing the dragged pose until the supervisor's target has caught up


class StateViewer:
    def __init__(self, show_cmd: bool = True, show_target: bool = True, settings: dict | None = None):
        self.m, self.arms = load_viz()
        self.d = mujoco.MjData(self.m)
        self.settings = settings if settings is not None else {"frame": "base"}   # shared with the control panel
        self.gizmo = Gizmo()
        self.cam, self.opt, self.pert = mujoco.MjvCamera(), mujoco.MjvOption(), mujoco.MjvPerturb()
        self.cam.lookat[:] = (0.0, 0.05, 0.25)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 1.3, 135.0, -20.0
        self.scn = mujoco.MjvScene(self.m, maxgeom=self.m.ngeom + 2000)
        self.screen: ScreenCam | None = None
        self.mouse = {"left": False, "right": False, "middle": False, "x": 0.0, "y": 0.0, "shift": False}
        self.drag_target = None          # (pos, R) while / just after dragging the gizmo
        self._shown = None               # (pos, R) of the gizmo drawn in the last frame
        self._drag_until = 0.0
        self._pending, self._last_send = False, 0.0
        self.on_target = None
        self.mocap = self.m.body_mocapid[self.m.body("ee_target").id]
        self.marker = [g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] == self.m.body("ee_target").id]
        self.marker_ball = self.m.geom("ee_target_ball").id
        self.show = {"cmd": show_cmd, "target": show_target}
        self.base_rgba = self.m.geom_rgba.copy()
        self.base_site_alpha = {a: float(self.m.site_rgba[i.tcp_site, 3]) for a, i in self.arms.items()}
        # visual geoms of each joint's child link, per arm (measured: group 1, ghosts: group 2)
        self.links = {arm: [[g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] == self.m.body(prefix + b).id
                             and self.m.geom_group[g] == (1 if arm == "measured" else 2)] for b in ARM_BODIES]
                      for arm, prefix in ARMS.items()}

    def _alpha(self, arm: str, visible: bool) -> None:
        g = self.arms[arm].geoms
        self.m.geom_rgba[g, 3] = np.where(visible, self.base_rgba[g, 3], 0.0)
        self.m.site_rgba[self.arms[arm].tcp_site, 3] = self.base_site_alpha[arm] if visible else 0.0

    def _measured_color(self, rgba) -> None:
        g = [x for x in self.arms["measured"].geoms if self.m.geom_group[x] == 1]
        self.m.geom_rgba[g] = rgba

    def apply(self, view) -> None:
        self.m.geom_rgba[:] = self.base_rgba
        self._apply_marker(view)
        if view.joint_status is not None:
            self._apply_align(view)
            return
        q = view.q_meas if view.q_meas is not None else (np.radians(view.rel_deg) if view.rel_deg is not None
                                                          else np.zeros(6))
        self.d.qpos[self.arms["measured"].qpos] = q
        self._measured_color(MEASURED if view.calibrated else UNCALIBRATED)
        for arm, value in (("cmd", view.q_cmd), ("target", view.q_target)):
            visible = self.show[arm] and value is not None and view.calibrated
            if arm == "target" and value is not None and self.show[arm] and not view.calibrated:
                visible = True       # the reference pose to align to is useful before calibration
            self.d.qpos[self.arms[arm].qpos] = value if value is not None else q
            self._alpha(arm, visible)
        mujoco.mj_kinematics(self.m, self.d)

    def _apply_marker(self, view) -> None:
        if view.target_pose is None:
            self.m.geom_rgba[self.marker, 3] = 0.0
            return
        if not view.target_ok:
            self.m.geom_rgba[self.marker_ball] = (1.0, 0.15, 0.1, 0.8)
        pos, quat = view.target_pose          # written every frame: a mouse drag of the marker has no effect
        self.d.mocap_pos[self.mocap], self.d.mocap_quat[self.mocap] = pos, quat

    def _apply_align(self, view) -> None:
        m = self.m
        self.d.qpos[self.arms["measured"].qpos] = view.q_meas
        self.d.qpos[self.arms["target"].qpos] = view.q_target
        self.d.qpos[self.arms["cmd"].qpos] = view.q_meas
        self._alpha("cmd", False)
        self._alpha("target", True)
        for k, status in enumerate(view.joint_status):
            meas, ghost = self.links["measured"][k], self.links["target"][k]
            if status == "held":
                m.geom_rgba[meas] = MEASURED
                m.geom_rgba[ghost, 3] = 0.0           # its target is reached: show the measured link alone
            elif status == "current_live":
                m.geom_rgba[meas] = CURRENT
            else:
                m.geom_rgba[meas, 3] = 0.0
            if status in ("current", "current_live"):
                m.geom_rgba[ghost] = CURRENT_GHOST
        mujoco.mj_kinematics(m, self.d)

    # ---------------------------------------------------------------- mouse (pure: GLFW callbacks and tests)
    def _target(self, view):
        """(pos, R) of the EE target shown now, or None (no gizmo outside TELEOP)."""
        if view.state != "TELEOP" or view.target_pose is None:
            return None
        if self.drag_target is not None and (self.gizmo.active or time.monotonic() < self._drag_until):
            return self.drag_target
        pos, quat = view.target_pose
        return np.asarray(pos, float), quat_to_mat(quat)

    def mouse_button(self, button: str, pressed: bool, shift: bool, x: float, y: float) -> None:
        """button "left" / "right" / "middle"; x, y in window coordinates (origin top-left)."""
        self.mouse.update(x=x, y=y, shift=shift)
        if not pressed:
            if button == "left" and self.gizmo.active:
                self.gizmo.end()
                self._send(force=True)
                self._drag_until = time.monotonic() + HOLD_AFTER_DRAG_S
            self.mouse[button] = False
            return
        if any(self.mouse[b] for b in ("left", "right", "middle")) or self.gizmo.active:
            return                                           # one action at a time
        if button == "left" and self._shown is not None and self.screen is not None:
            pos, R = self._shown
            axes = frame_axes(R, self.settings["frame"])
            name = self.gizmo.pick(self.screen, x, y, pos, axes)
            if name is not None and self.gizmo.begin(name, self.screen, x, y, pos, R, axes):
                self.drag_target = (pos.copy(), R.copy())
                return
        self.mouse[button] = True

    def mouse_move(self, x: float, y: float) -> None:
        dx, dy = x - self.mouse["x"], y - self.mouse["y"]
        self.mouse.update(x=x, y=y)
        if self.gizmo.active:
            res = self.gizmo.drag(self.screen, x, y) if self.screen is not None else None
            if res is not None:
                self.drag_target = res
                self._pending = True
                self._send()
            return
        h = self.screen.height if self.screen is not None else 800.0
        sh = self.mouse["shift"]
        if self.mouse["right"]:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if sh else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self.mouse["left"]:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if sh else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif self.mouse["middle"]:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        else:
            if self._shown is not None and self.screen is not None:
                self.gizmo.hot = self.gizmo.pick(self.screen, x, y, self._shown[0],
                                                 frame_axes(self._shown[1], self.settings["frame"]))
            return
        mujoco.mjv_moveCamera(self.m, action, dx / h, dy / h, self.cam)

    def scroll(self, dy: float) -> None:
        mujoco.mjv_moveCamera(self.m, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * dy, self.cam)

    def _send(self, force: bool = False) -> None:
        if self.on_target is None or self.drag_target is None or not (self._pending or force):
            return
        now = time.monotonic()
        if force or now - self._last_send >= 1.0 / SEND_HZ:
            pos, R = self.drag_target
            self.on_target(pos.copy(), mat_to_quat(R))
            self._last_send, self._pending = now, False

    # ---------------------------------------------------------------- one frame
    def frame(self, view, width: float, height: float) -> mujoco.MjvScene:
        """Update the scene for this view (window size in cursor units); the gizmo is drawn in TELEOP only."""
        self.apply(view)
        if view.state != "TELEOP" and self.gizmo.active:
            self.gizmo.end()                                 # stopped / left TELEOP while dragging
        self._shown = self._target(view)
        if self._shown is not None:
            self.m.geom_rgba[self.marker, 3] = 0.0           # the gizmo replaces the model's marker
        mujoco.mjv_updateScene(self.m, self.d, self.opt, self.pert, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scn)
        self.screen = ScreenCam.from_scene(self.scn, width, height)
        if self._shown is not None:
            pos, R = self._shown
            if self.settings["frame"] == "base":                 # small base X/Y/Z triad on the floor
                for k, name in enumerate("xyz"):
                    self.gizmo._add(self.scn, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, np.zeros(3), 0.12 * np.eye(3)[k],
                                    np.array([*AXIS_RGB[name], 1.0], np.float32))
            self.gizmo.draw(self.scn, self.screen, pos, frame_axes(R, self.settings["frame"]),
                            ok=view.target_ok or self.gizmo.active is not None)
        else:
            self.gizmo.hot = None
        return self.scn

    def overlay(self, view) -> str:
        if view.state != "TELEOP":
            return view.overlay
        f = self.settings["frame"].upper()
        return view.overlay + (f"\nGIZMO axes: {f} (control panel). Drag an ARROW = move along it, a RING = rotate "
                               f"about it. Empty space: camera" + (f"   [dragging {self.gizmo.active}]"
                                                                   if self.gizmo.active else ""))

    def run(self, get_view, request_quit, is_done, on_target=None, hz: float = 60.0) -> None:
        import os
        import glfw
        self.on_target = on_target
        ok = False
        if os.environ.get("DISPLAY") and hasattr(glfw, "platform_supported") and glfw.platform_supported(glfw.PLATFORM_X11):
            glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)    # XWayland like the Tk panel, if this GLFW has it
            ok = glfw.init()
            glfw.init_hint(glfw.PLATFORM, glfw.ANY_PLATFORM)
        if not ok and not glfw.init():
            raise RuntimeError("GLFW init failed (no display?)")
        glfw.window_hint(glfw.SAMPLES, 4)
        win = glfw.create_window(1280, 860, "manipulator_control SIM", None, None)
        if not win:
            glfw.terminate()
            raise RuntimeError("GLFW window creation failed")
        glfw.make_context_current(win)
        glfw.swap_interval(1)
        con = mujoco.MjrContext(self.m, mujoco.mjtFontScale.mjFONTSCALE_150)
        names = {glfw.MOUSE_BUTTON_LEFT: "left", glfw.MOUSE_BUTTON_RIGHT: "right", glfw.MOUSE_BUTTON_MIDDLE: "middle"}

        def on_button(w, b, action, mods):
            if b in names:
                self.mouse_button(names[b], action == glfw.PRESS, bool(mods & glfw.MOD_SHIFT), *glfw.get_cursor_pos(w))

        glfw.set_mouse_button_callback(win, on_button)
        glfw.set_cursor_pos_callback(win, lambda w, x, y: self.mouse_move(x, y))
        glfw.set_scroll_callback(win, lambda w, dx, dy: self.scroll(dy))
        closed_by_user = False
        try:
            while not is_done():
                if glfw.window_should_close(win):
                    closed_by_user = True
                    break
                t0 = time.monotonic()
                v = get_view()
                ww, wh = glfw.get_window_size(win)
                fw, fh = glfw.get_framebuffer_size(win)
                if min(ww, wh, fw, fh) > 0:
                    self.frame(v, ww, wh)
                    viewport = mujoco.MjrRect(0, 0, fw, fh)
                    mujoco.mjr_render(viewport, self.scn, con)
                    mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport,
                                       self.overlay(v), "", con)
                    glfw.swap_buffers(win)
                glfw.poll_events()
                self._send()
                time.sleep(max(0.0, 1.0 / hz - (time.monotonic() - t0)))
        finally:
            con.free()
            glfw.destroy_window(win)
            glfw.terminate()
        if closed_by_user and not is_done():
            request_quit()
