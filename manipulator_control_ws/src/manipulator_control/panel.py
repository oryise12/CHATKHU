"""Control panel (a Tk window next to the SIM): stop, HOME preview, resume, target = real pose, torque off (after a
confirmation dialog) and the gizmo's axes (base / tool). The EE itself is moved in the SIM window with the gizmo
(gizmo.py, viz.py), not here. The panel only puts events on the input queue and reads the latest View; every
decision stays in the bus loop. Other confirmations (y) and quit (q) stay in the terminal. Closing the panel sends
STOP (the arm stops and holds).

Pinch arming dialog (2026-10-29, user decision): while the session waits for the arming (ARM_CONFIRM) a small window
stays on top ("실물을 무장할까요?"); y or its button arms (source "arm_dialog", accepted for that state only) and the
window closes. It also closes when the state moves on (e.g. y in the terminal); the terminal y keeps working.

Frames: "base" = robot base X/Y/Z (the big triad on the floor of the SIM); "tool" = the target's own axes. The
status line shows the target as base-frame XYZ and RPY = fixed X, Y, Z angles (R = Rz(yaw) Ry(pitch) Rx(roll)).
"""
from __future__ import annotations

import gc
import math
import threading

import mujoco
import numpy as np

from .supervisor import ARM_DIALOG_SOURCE, TORQUE_OFF_NOW_STATES


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def mat_to_rpy(R) -> tuple[float, float, float]:
    R = np.asarray(R, float)
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) > 1 - 1e-9:                       # gimbal lock: roll folded into yaw
        return 0.0, pitch, math.atan2(-R[0, 1], R[1, 1])
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def mat_to_quat(R) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(-1))
    return q


def quat_to_mat(quat) -> np.ndarray:
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(quat, float))
    return m.reshape(3, 3)


def target_fields(view) -> tuple[np.ndarray, np.ndarray] | None:
    """(xyz mm, rpy deg) of the current target."""
    if view.target_pose is None:
        return None
    pos, quat = view.target_pose
    return np.asarray(pos, float) * 1000.0, np.degrees(mat_to_rpy(quat_to_mat(quat)))


def enabled(state: str, purpose: str = "teleop") -> dict[str, bool]:
    ee = purpose == "teleop"            # pinch: the simulator plans every motion; the panel only stops / turns off
    return {"stop": True, "reset": ee and state == "TELEOP", "teleop": ee and state == "HOLDING",
            "home": ee and state in ("TELEOP", "HOLDING"), "torque_off": state in TORQUE_OFF_NOW_STATES}


class ControlPanel:
    def __init__(self, put, get_view, is_done, settings: dict, purpose: str = "teleop"):
        self.put, self.get_view, self.is_done, self.settings = put, get_view, is_done, settings
        self.purpose = purpose
        self._closing = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="control-panel", daemon=True)
        self._thread.start()

    def close(self, timeout: float = 2.0) -> None:
        self._closing.set()
        if self._thread is not None:
            self._thread.join(timeout)

    # ---------------------------------------------------------------- Tk (all calls in this thread)
    _TK_ATTRS = ("root", "frame_var", "status", "detail", "detail_label", "groups", "arm_win")

    def _run(self) -> None:
        try:
            self._build()
            self.root.mainloop()
        finally:
            # Tcl objects must be freed by the thread that created them, otherwise the interpreter aborts the
            # process later ("Tcl_AsyncDelete: async handler deleted by the wrong thread")
            for name in self._TK_ATTRS:
                self.__dict__.pop(name, None)
            gc.collect()

    def _build(self) -> None:
        import tkinter as tk
        from tkinter import font as tkfont

        root = tk.Tk()
        self.root = root
        root.title("조작 패널")
        root.attributes("-topmost", True)
        families = set(tkfont.families(root))
        family = next((f for f in ("Noto Sans CJK KR", "NanumGothic", "NanumSquare") if f in families), None)
        if family:
            for name in ("TkDefaultFont", "TkTextFont", "TkFixedFont"):
                tkfont.nametofont(name).configure(family=family, size=11)
        big = (family, 13, "bold") if family else ("TkDefaultFont", 13, "bold")

        self.frame_var = tk.StringVar(value=self.settings["frame"])
        self.status = tk.StringVar(value="연결 중")
        self.detail = tk.StringVar(value="")
        self.groups: dict[str, list] = {k: [] for k in enabled("")}

        pad = {"padx": 6, "pady": 3}
        top = tk.Frame(root)
        top.pack(fill="x", **pad)
        tk.Label(top, textvariable=self.status, font=big, anchor="w").pack(fill="x")
        self.detail_label = tk.Label(top, textvariable=self.detail, anchor="w", justify="left", wraplength=440)
        self.detail_label.pack(fill="x")

        f = tk.LabelFrame(root, text="EE 조작은 SIM 창에서: 화살표를 끌면 그 축으로 이동, 링을 끌면 그 축으로 회전")
        f.pack(fill="x", **pad)
        tk.Label(f, text="기즈모 축 기준", anchor="w").pack(anchor="w")
        for value, text in (("base", "베이스 (로봇 바닥 X/Y/Z, SIM 바닥의 큰 축)"), ("tool", "툴 (EE 자신의 X/Y/Z)")):
            tk.Radiobutton(f, text=text, variable=self.frame_var, value=value,
                           command=lambda: self.settings.__setitem__("frame", self.frame_var.get())).pack(anchor="w")
        tk.Label(f, text="빨강 X · 초록 Y · 파랑 Z. 한 번에 핸들 하나만 움직입니다. 빈 곳을 끌면 카메라.",
                 fg="#555").pack(anchor="w")

        f = tk.Frame(root)
        f.pack(fill="x", **pad)
        for group, text, kind in (("reset", "목표를 실제 자세로", "TARGET_RESET"), ("home", "HOME 미리보기 (h)", "HOME"),
                                  ("teleop", "조작 재개 (t)", "TELEOP")):
            b = tk.Button(f, text=text, command=lambda k=kind: self.put(k))
            b.pack(side="left", padx=2)
            self.groups[group].append(b)
        b = tk.Button(root, text="■ 정지 (s) - 그 자리에서 유지", bg="#c62828", fg="white", activebackground="#e53935",
                      font=big, command=lambda: self.put("STOP"))
        b.pack(fill="x", padx=6, pady=(6, 3))
        self.groups["stop"].append(b)
        b = tk.Button(root, text="토크 OFF (팔을 받친 뒤)", bg="#424242", fg="white", activebackground="#616161",
                      command=self._torque_off)
        b.pack(fill="x", padx=6, pady=3)
        self.groups["torque_off"].append(b)
        tk.Label(root, text="y(확인)·q(종료)는 터미널에서. 이 창을 닫으면 정지(유지).", fg="#555").pack(anchor="w", padx=6,
                                                                                          pady=(0, 6))
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.arm_win = None
        self._arm_done = False                # answered / closed for this ARM_CONFIRM: do not reopen until it leaves
        self._family, self._big = family, big
        root.after(100, self._poll)

    # ---------------------------------------------------------------- pinch arming dialog
    def _arm_dialog(self, state: str) -> None:
        """Open the small arming window while ARM_CONFIRM, close it in any other state."""
        if state != "ARM_CONFIRM" or self.purpose != "pinch":
            self._close_arm()
            self._arm_done = False
            return
        if self.arm_win is not None or self._arm_done:
            return
        import tkinter as tk
        w = tk.Toplevel(self.root)
        self.arm_win = w
        w.title("실물 무장")
        w.attributes("-topmost", True)
        w.resizable(False, False)
        tk.Label(w, text="시뮬 연결됨. 실물을 무장할까요?", font=self._big).pack(padx=16, pady=(14, 4))
        tk.Label(w, text="무장하면 승인할 때마다 실물이 움직입니다.\n팔 주변을 확인하고 12 V 스위치에 손을 두세요.",
                 justify="center", fg="#555").pack(padx=16)
        tk.Button(w, text="y  무장", bg="#c62828", fg="white", activebackground="#e53935", font=self._big,
                  width=12, command=self._arm_yes).pack(pady=(10, 14))
        for key in ("<y>", "<Y>"):
            w.bind(key, lambda _e: self._arm_yes())
        w.protocol("WM_DELETE_WINDOW", self._close_arm)     # closing it is not a yes (terminal y still works)
        w.update_idletasks()
        sw, sh = w.winfo_screenwidth(), w.winfo_screenheight()
        w.geometry(f"+{max(0, (sw - w.winfo_width()) // 2)}+{int(sh * 0.15)}")
        w.lift()
        w.focus_force()

    def _arm_yes(self) -> None:
        self.put("CONFIRM", source=ARM_DIALOG_SOURCE)
        self._close_arm()

    def _close_arm(self) -> None:
        if self.arm_win is not None:
            self._arm_done = True
            self.arm_win.destroy()
            self.arm_win = None

    def _torque_off(self) -> None:
        from tkinter import messagebox
        if messagebox.askyesno("토크 OFF", "모든 관절의 토크를 끕니다. 팔이 자중으로 처집니다.\n\n팔을 손이나 지그로 받쳤나요?",
                               icon="warning", default="no", parent=self.root):
            self.put("TORQUE_OFF_NOW")

    def _on_close(self) -> None:
        self.put("STOP")
        self.root.destroy()

    def _poll(self) -> None:
        if self._closing.is_set() or self.is_done():
            self.root.destroy()
            return
        v = self.get_view()
        self.status.set(f"상태: {v.state}" + ("  (STALE)" if v.stale else ""))
        tf = target_fields(v)
        if v.state == "TELEOP" and tf is not None:
            p, r = [int(round(x)) for x in tf[0]], [int(round(x)) for x in tf[1]]          # no "-0"
            self.detail.set(f"목표 [{p[0]}, {p[1]}, {p[2]}] mm, RPY [{r[0]}, {r[1]}, {r[2]}]°\n"
                            + (v.progress.split(" | ")[1] if " | " in v.progress else ""))
            self.detail_label.configure(fg="#2e7d32" if v.target_ok else "#c62828")
        else:
            self.detail.set(v.message)
            self.detail_label.configure(fg="black")
        self._arm_dialog(v.state)
        on = enabled(v.state, self.purpose)
        for group, buttons in self.groups.items():
            for b in buttons:
                b.configure(state="normal" if on[group] else "disabled")
        self.root.after(100, self._poll)
