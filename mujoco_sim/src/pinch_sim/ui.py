"""side_cam monitor window (2026-10-12): 16:10 camera image + 4-stage progress strip + chat bubbles + buttons.

Layout (display pixels, top to bottom):
    progress strip  : 초기 경로 → 수정 → 검증 → 이동   (reached = green, not yet = white, current = outlined)
    camera image    : the 16:10 render (or real camera frame) with the projected 2-D path graph, scaled by
                      `scale`; a short notice (refusal / validation result) may appear at its top-left and
                      the conversation bubbles (user right/yellow, 쿠옹이 left/white, semi-transparent) on its
                      right side, outside the region where paths are projected
    button bar      : the existing N S V Y H X U R O Q buttons

The stage comes from the Session itself (state, edit mode, edited path, drag), never from what was said.
Clicks on bubbles / notice are UI clicks, not path edits. Text is drawn with Pillow and a Korean font
(Noto Sans CJK KR via fontconfig; `sudo apt install fonts-noto-cjk` on a new PC); without such a font the
text falls back to OpenCV ASCII rendering.
"""
from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

STAGES = ("초기 경로", "수정", "검증", "이동")
GREEN = (72, 178, 76)          # BGR
WHITE = (245, 245, 245)
OUTLINE = (0, 200, 255)        # current stage
STRIP_BG = (32, 32, 32)
KOREAN_STATE = {"HOLD": "대기", "GENERATED": "경로 생성됨", "SELECTED": "경로 선택됨", "VALIDATING": "검증 중",
                "PASSED": "검증 통과", "FAILED": "검증 실패", "EXECUTING": "이동 중", "DONE": "완료", "ABORTED": "중단됨"}
FAIL_REASON = {
    "path_wellformed": "경로 형식 오류", "start_matches_robot": "시작점이 로봇 위치와 다름",
    "goal_matches_target": "끝점이 목표와 다름", "workspace": "작업 공간 벗어남(너무 낮거나 멂)",
    "continuous_ik": "따라갈 관절 자세 없음", "joint_limits": "관절 한계 초과", "singularity": "특이 자세 근접",
    "collision": "책상·자기 충돌", "timing": "시간 배치 실패", "joint_vel_acc": "관절 속도·가속도 초과",
    "torque": "모터 토크 부족", "sim_rollout": "시뮬레이션 추종 실패",
}


# --------------------------------------------------------------------------------------------- stage
def progress_stage(session) -> tuple[int, str]:
    """(index into STAGES, short sub-label) from the live Session. -1 = no path yet (all white)."""
    st = session.state
    path = session.path
    if path is None or st == "HOLD":
        return -1, "목표 없음 - N으로 새 목표"
    dragging = session.edit is not None and session.edit.dragging
    edited = path.source == "edited"
    if st in ("GENERATED", "SELECTED"):
        if dragging:
            return 1, "드래그 중"
        if session.edit_mode or edited:
            if st == "SELECTED":
                return 1, "수정 경로 선택됨" if edited else "경로 선택됨"
            return 1, "수정 중"
        if path.path_id == "return_ready":
            return 0, "홈 복귀 경로" + (" 선택됨" if st == "SELECTED" else "")
        return 0, "경로 선택됨" if st == "SELECTED" else ""
    if st == "FAILED":
        return 1, "검증 실패 - 수정하세요"
    if st == "VALIDATING":
        return 2, "검증 중" + (" (통과하면 자동 이동)" if session.auto_run is not None else "")
    if st == "PASSED":
        return 2, "통과 - 자동 이동" if session.auto_run is not None else "통과 - Y로 이동"
    if st == "EXECUTING":
        return 3, "이동 중"
    if st == "DONE":
        return 3, "완료"
    return 3, "중단됨 - 현재 자세 유지"


# --------------------------------------------------------------------------------------------- notices
def notice_text(ev, session) -> str | None:
    """Short Korean notice for Session events a button user would otherwise miss (None = no notice)."""
    k, msg, st = ev.kind, ev.message, ev.state
    if k == "refused":
        rules = (("N ignored", "검증·이동 중에는 새 목표를 만들 수 없어요"),
                 ("S ignored: release", "드래그를 놓은 뒤 선택하세요"),
                 ("S ignored", f"선택할 경로가 없어요 ({KOREAN_STATE.get(st, st)})"),
                 ("V ignored", "먼저 경로를 선택(S)하세요" if st == "GENERATED" else
                  f"지금은 검증할 수 없어요 ({KOREAN_STATE.get(st, st)})"),
                 ("Y refused: validation failed", "검증에 실패한 경로는 이동할 수 없어요"),
                 ("Y refused: not validated (VALIDATING)", "아직 검증 중이에요. 통과하면 Y를 누르세요"),
                 ("Y refused: not validated (EXECUTING)", "이미 이동 중이에요"),
                 ("Y refused", "검증을 통과한 경로가 없어요 (S → V 먼저)"),
                 ("H ignored", "홈 복귀는 대기·완료·중단 상태에서만 할 수 있어요"),
                 ("undo refused: no edit", "되돌릴 수정이 없어요"),
                 ("undo: Nothing", "되돌릴 수정이 없어요"),
                 ("redo: Nothing", "다시 실행할 수정이 없어요"),
                 ("reset: Paths are already", "이미 원래 경로예요"),
                 ("use path refused: this path failed", "검증에 실패한 경로예요. 수정하거나 원래 경로로 돌리세요"),
                 ("fixed target rejected", "목표가 허용 범위를 벗어나요"))
        for prefix, text in rules:
            if msg.startswith(prefix):
                return text
        if msg.startswith(("edit refused", "edit not started", "undo refused", "redo refused", "reset refused")):
            why = ("검증 중 - X로 취소 후" if st == "VALIDATING" else "이동 중" if st == "EXECUTING" else
                   "N 또는 H로 새 경로를 만든 뒤" if st in ("DONE", "ABORTED", "HOLD") else "드래그를 놓은 뒤")
            return f"지금은 경로를 수정할 수 없어요 ({why})"
        return "요청을 처리하지 않았어요"
    if k == "validation":
        rep = session.report
        if st == "FAILED":
            kinds = [c.kind for c in rep.checks if c.status != "PASS"] if rep is not None else []
            reasons = ", ".join(dict.fromkeys(FAIL_REASON.get(x, x) for x in kinds[:2])) or "검증 항목 실패"
            return f"검증 실패: {reasons}"
        if st == "PASSED":
            return "검증 통과" + ("" if session.auto_run is not None else " - Y를 누르면 이동")
    if k == "execution_end":
        return "이동 완료" if st == "DONE" else "이동 중단 - 현재 자세 유지"
    if k == "regenerated":
        return "로봇 상태가 바뀌어 경로를 다시 만들었어요 (수정 내용 버림)"
    if k == "auto_run_cancelled" and not str(ev.data.get("reason", "")).startswith(("validation failed",
                                                                                     "execution finished")):
        return "자동 이동 예약이 취소됐어요"
    if k == "cancel":
        return "검증을 취소했어요" if "validation" in msg else "선택을 취소했어요"
    if k == "new_target" and st == "HOLD":
        return "목표를 만들지 못했어요"
    if k == "edit_end" and not ev.data.get("kept", True):
        return "움직임이 너무 작아 수정하지 않았어요"
    if k == "voice_link":
        return "쿠옹이 연결됨" if "connected" in msg else "쿠옹이 연결 끊김"
    return None


@dataclass
class Notices:
    hold_s: float = 7.0
    items: list = field(default_factory=list)      # [(t, text)]

    def push(self, text: str) -> None:
        if text and (not self.items or self.items[-1][1] != text):
            self.items.append((time.monotonic(), text))
            del self.items[:-2]

    def current(self) -> list:
        now = time.monotonic()
        self.items = [(t, s) for t, s in self.items if now - t < self.hold_s]
        return [s for _, s in self.items]


# --------------------------------------------------------------------------------------------- text
def find_korean_font() -> tuple[str, int] | None:
    try:
        out = subprocess.run(["fc-match", "-f", "%{file}|%{index}|%{family}", "Noto Sans CJK KR:style=Medium"],
                             capture_output=True, text=True, timeout=3).stdout
        path, index, family = out.split("|", 2)
        if "CJK KR" in family or "Nanum" in family:
            return path, int(index or 0)
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    for path, index in (("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc", 1),
                        ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 1),
                        ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf", 0)):
        try:
            open(path, "rb").close()
            return path, index
        except OSError:
            continue
    return None


class Text:
    """Cached text sprites (RGBA) rendered with Pillow; ASCII OpenCV fallback without a Korean font."""

    def __init__(self):
        self.font_ref = find_korean_font()
        self._fonts: dict = {}
        self._cache: dict = {}
        try:
            from PIL import Image, ImageDraw, ImageFont   # noqa: F401
            self._pil = True
        except ImportError:
            self._pil = False
        if self.font_ref is None or not self._pil:
            print("monitor UI: no Korean font/Pillow - text falls back to ASCII "
                  "(install: pip install pillow; sudo apt install fonts-noto-cjk)", flush=True)

    def font(self, size: int):
        from PIL import ImageFont
        if size not in self._fonts:
            self._fonts[size] = ImageFont.truetype(self.font_ref[0], size, index=self.font_ref[1])
        return self._fonts[size]

    def width(self, s: str, size: int) -> int:
        if self.font_ref is None or not self._pil:
            return cv2.getTextSize(_ascii(s), cv2.FONT_HERSHEY_SIMPLEX, size / 30, 1)[0][0]
        return int(np.ceil(self.font(size).getlength(s)))

    def sprite(self, s: str, size: int, rgb=(255, 255, 255)) -> np.ndarray:
        """(h, w, 4) uint8 RGBA, tight line box."""
        key = (s, size, tuple(rgb))
        if key in self._cache:
            return self._cache[key]
        if self.font_ref is None or not self._pil:
            (w, h), b = cv2.getTextSize(_ascii(s), cv2.FONT_HERSHEY_SIMPLEX, size / 30, 1)
            img = np.zeros((h + b + 4, w + 2, 4), np.uint8)
            cv2.putText(img, _ascii(s), (1, h + 2), cv2.FONT_HERSHEY_SIMPLEX, size / 30, (*rgb, 255), 1, cv2.LINE_AA)
        else:
            from PIL import Image, ImageDraw
            f = self.font(size)
            asc, desc = f.getmetrics()
            w = max(1, self.width(s, size))
            im = Image.new("RGBA", (w + 2, asc + desc + 2), (0, 0, 0, 0))
            ImageDraw.Draw(im).text((1, 1), s, font=f, fill=(*rgb, 255))
            img = np.asarray(im)
        if len(self._cache) > 600:
            self._cache.clear()
        self._cache[key] = img
        return img

    def wrap(self, s: str, size: int, max_w: int, max_lines: int = 4) -> list[str]:
        words, lines, cur = s.split(" "), [], ""
        for word in words:
            cand = word if not cur else cur + " " + word
            if self.width(cand, size) <= max_w:
                cur = cand
                continue
            if cur:
                lines.append(cur)
            cur = ""
            for ch in word:                      # a word longer than the line: break by character
                if self.width(cur + ch, size) > max_w and cur:
                    lines.append(cur)
                    cur = ""
                cur += ch
        if cur:
            lines.append(cur)
        if len(lines) > max_lines:
            lines = lines[:max_lines]
            while lines[-1] and self.width(lines[-1] + "…", size) > max_w:
                lines[-1] = lines[-1][:-1]
            lines[-1] += "…"
        return lines


def _ascii(s: str) -> str:
    return s.encode("ascii", "replace").decode()


def blit(dst: np.ndarray, rgba: np.ndarray, x: int, y: int, alpha: float = 1.0) -> None:
    """Alpha-composite an RGBA sprite onto a BGR image (clipped)."""
    h, w = rgba.shape[:2]
    x0, y0, x1, y1 = max(x, 0), max(y, 0), min(x + w, dst.shape[1]), min(y + h, dst.shape[0])
    if x1 <= x0 or y1 <= y0:
        return
    src = rgba[y0 - y:y1 - y, x0 - x:x1 - x]
    a = (src[..., 3:4].astype(np.float32) / 255.0) * alpha
    roi = dst[y0:y1, x0:x1].astype(np.float32)
    dst[y0:y1, x0:x1] = (roi * (1 - a) + src[..., 2::-1].astype(np.float32) * a).astype(np.uint8)


def round_rect(dst: np.ndarray, x0: int, y0: int, x1: int, y1: int, bgr, alpha: float, r: int = 10,
               border=None, thickness: int = 2) -> None:
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    pad = thickness + 1
    bx0, by0 = max(x0 - pad, 0), max(y0 - pad, 0)
    bx1, by1 = min(x1 + pad, dst.shape[1]), min(y1 + pad, dst.shape[0])
    if bx1 <= bx0 or by1 <= by0:
        return
    roi = dst[by0:by1, bx0:bx1]
    mask = np.zeros(roi.shape[:2], np.uint8)
    r = max(1, min(r, (x1 - x0) // 2, (y1 - y0) // 2))
    ox, oy = x0 - bx0, y0 - by0
    w, h = x1 - x0, y1 - y0
    cv2.rectangle(mask, (ox + r, oy), (ox + w - r, oy + h), 255, -1)
    cv2.rectangle(mask, (ox, oy + r), (ox + w, oy + h - r), 255, -1)
    for cx, cy in ((ox + r, oy + r), (ox + w - r, oy + r), (ox + r, oy + h - r), (ox + w - r, oy + h - r)):
        cv2.circle(mask, (cx, cy), r, 255, -1, cv2.LINE_AA)
    a = (mask.astype(np.float32) / 255.0 * alpha)[..., None]
    roi[:] = (roi * (1 - a) + np.array(bgr, np.float32) * a).astype(np.uint8)
    if border is not None:
        edge = cv2.dilate(mask, np.ones((2 * thickness + 1, 2 * thickness + 1), np.uint8)) - mask
        roi[edge > 0] = border


# --------------------------------------------------------------------------------------------- layout
@dataclass
class Layout:
    render_w: int
    render_h: int
    scale: float
    top_h: int = 50
    bar_h: int = 36

    @property
    def img_w(self) -> int:
        return int(round(self.render_w * self.scale))

    @property
    def img_h(self) -> int:
        return int(round(self.render_h * self.scale))

    @property
    def width(self) -> int:
        return self.img_w

    @property
    def height(self) -> int:
        return self.top_h + self.img_h + self.bar_h

    def region(self, x: int, y: int) -> str:
        if y < self.top_h:
            return "top"
        if y >= self.top_h + self.img_h:
            return "bar"
        return "image"

    def to_image(self, x: int, y: int) -> tuple[int, int]:
        """Window pixel -> pixel of the displayed (scaled) camera image."""
        return x, y - self.top_h


def monitors() -> list[tuple[int, int, int, int]]:
    """[(x, y, w, h)] of connected monitors (xrandr), primary first; [] if unknown."""
    try:
        out = subprocess.run(["xrandr", "--listmonitors"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    mons = []
    for line in out.splitlines()[1:]:
        m = re.search(r"(\d+)/\d+x(\d+)/\d+\+(\d+)\+(\d+)", line)
        if m:
            w, h, x, y = map(int, m.groups())
            mons.append((("*" in line), (x, y, w, h)))
    mons.sort(key=lambda t: not t[0])
    return [m for _, m in mons]


def auto_layout(render_w: int, render_h: int, scale_arg: str = "auto",
                monitor: tuple[int, int, int, int] | None = None) -> Layout:
    """Default: about twice the old 544x408 display area (render 768x480 x 1.1 = 845x528), shrunk to fit."""
    lay = Layout(render_w, render_h, 1.1)
    if scale_arg != "auto":
        lay.scale = float(scale_arg)
        return lay
    if monitor is not None:
        _, _, mw, mh = monitor
        fit = min(0.55 * mw / render_w, (mh - 120 - lay.top_h - lay.bar_h) / render_h)
        lay.scale = round(max(0.5, min(lay.scale, fit)), 3)
    return lay


def window_pos(lay: Layout, pos_arg: str, monitor: tuple[int, int, int, int] | None) -> tuple[int, int]:
    if pos_arg != "auto":
        x, y = (int(v) for v in pos_arg.split(","))
        return x, y
    if monitor is None:
        return 1000, 60
    mx, my, mw, mh = monitor
    # bottom-right corner of the monitor (room left for the title bar and a bottom panel/dock)
    return mx + max(0, mw - lay.width - 12), my + max(0, mh - lay.height - 70)


def _xwin_geometry(wid: int) -> tuple[int, int, int, int] | None:
    out = subprocess.run(["xwininfo", "-id", str(wid)], capture_output=True, text=True, timeout=3).stdout
    vals = dict(re.findall(r"(Absolute upper-left X|Absolute upper-left Y|Width|Height):\s+(-?\d+)", out))
    try:
        return (int(vals["Absolute upper-left X"]), int(vals["Absolute upper-left Y"]), int(vals["Width"]),
                int(vals["Height"]))
    except KeyError:
        return None


def _ewmh_moveresize(wid: int, x: int, y: int, w: int, h: int) -> bool:
    """_NET_MOVERESIZE_WINDOW to the root window (the window manager does it; GNOME ignores a plain
    XMoveResizeWindow resize of the GLFW viewer)."""
    import ctypes
    import ctypes.util
    X = ctypes.cdll.LoadLibrary(ctypes.util.find_library("X11") or "libX11.so.6")
    X.XOpenDisplay.restype = ctypes.c_void_p
    X.XDefaultRootWindow.restype = ctypes.c_ulong
    X.XInternAtom.restype = ctypes.c_ulong
    raw = X.XOpenDisplay(None)
    if not raw:
        return False
    dpy = ctypes.c_void_p(raw)

    class XClientMessageEvent(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                    ("display", ctypes.c_void_p), ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                    ("format", ctypes.c_int), ("data", ctypes.c_long * 5)]

    class XEvent(ctypes.Union):
        _fields_ = [("xclient", XClientMessageEvent), ("pad", ctypes.c_long * 24)]

    ev = XEvent()
    ev.xclient.type, ev.xclient.window, ev.xclient.format = 33, wid, 32          # ClientMessage
    ev.xclient.message_type = X.XInternAtom(dpy, b"_NET_MOVERESIZE_WINDOW", 0)
    ev.xclient.data[0] = 1 | (0xF << 8) | (2 << 12)       # NorthWest gravity, x y w h given, from a pager/tool
    for i, v in enumerate((x, y, w, h)):
        ev.xclient.data[i + 1] = v
    ok = X.XSendEvent(dpy, ctypes.c_ulong(X.XDefaultRootWindow(dpy)), 0, ctypes.c_long((1 << 20) | (1 << 19)),
                      ctypes.byref(ev))
    X.XFlush(dpy)
    X.XCloseDisplay(dpy)
    return bool(ok)


def protect_points(ov, cam, scale: float) -> np.ndarray:
    """Displayed-image pixels of what the user must see: path, before-edit path, markers, TCP, failures."""
    pts = []
    for arr in (ov.path, ov.reference):
        if arr is not None and len(arr):
            pts.extend(np.asarray(arr, float))
    for p in (ov.floor_point, ov.approach, ov.tcp_goal, ov.selected, getattr(ov, "pointed", None), ov.tcp, ov.peak):
        if p is not None:
            pts.append(np.asarray(p, float))
    pts.extend(np.asarray(p, float) for p, _ in getattr(ov, "failures", []))
    if not pts:
        return np.zeros((0, 2))
    uv, front = cam.project(np.array(pts))
    return uv[front] * scale


def arrange_viewer(title: str, side_x: int, monitor: tuple[int, int, int, int] | None) -> str:
    """Best effort: put the MuJoCo viewer window into the free space left of side_cam on the same monitor so
    the two windows do not overlap. Requests a size, reads back what the window manager made of it (frame
    decorations add to it) and corrects once. X11 only; does nothing without X11/xwininfo."""
    if monitor is None:
        return "no monitor info"
    try:
        out = subprocess.run(["xwininfo", "-name", title, "-int"], capture_output=True, text=True, timeout=3).stdout
        m = re.search(r"Window id: (\d+)", out)
        if not m:
            return "viewer window not found"
        wid = int(m.group(1))
        mx, my, mw, mh = monitor
        x, y = mx + 8, my + 60
        w = max(480, side_x - x - 16)
        h = int(min(mh - 160, w * 0.75))
        for _ in range(2):
            if not _ewmh_moveresize(wid, x, y, w, h):
                return "no X display"
            time.sleep(0.3)
            g = _xwin_geometry(wid)
            if g is None:
                return "geometry unknown"
            over_x = g[0] + g[2] - (side_x - 8)
            over_y = g[1] + g[3] - (my + mh - 40)
            if over_x <= 0 and over_y <= 0:
                break
            w, h = w - max(0, over_x), h - max(0, over_y)
        return f"viewer {g[2]}x{g[3]} at ({g[0]}, {g[1]}), side_cam from x={side_x}"
    except (OSError, subprocess.SubprocessError, AttributeError) as exc:
        return f"not arranged: {exc}"


# --------------------------------------------------------------------------------------------- renderer
class MonitorUI:
    def __init__(self, layout: Layout):
        self.lay = layout
        self.text = Text()
        self.ui_rects: list = []          # [(x0, y0, x1, y1)] in displayed-image pixels: bubbles + notice
        self._protect = np.zeros((0, 2))

    def hit_ui(self, xi: int, yi: int) -> bool:
        return any(x0 <= xi <= x1 and y0 <= yi <= y1 for x0, y0, x1, y1 in self.ui_rects)

    # .......................................................................................
    def render(self, frame_bgr: np.ndarray, session, buttons: list, notices: list, chat: list,
               hint: str = "", protect=None) -> np.ndarray:
        """protect: (N, 2) displayed-image pixels (path, targets, TCP, ...) that bubbles/notices must not cover."""
        lay = self.lay
        canvas = np.zeros((lay.height, lay.width, 3), np.uint8)
        canvas[:lay.top_h] = STRIP_BG
        canvas[lay.top_h + lay.img_h:] = (24, 24, 24)
        img = cv2.resize(frame_bgr, (lay.img_w, lay.img_h), interpolation=cv2.INTER_AREA if lay.scale < 1
                         else cv2.INTER_LINEAR)
        self.ui_rects = []
        self._protect = np.zeros((0, 2)) if protect is None or not len(protect) else \
            np.asarray(protect, float).reshape(-1, 2)
        self._notices(img, notices)
        self._bubbles(img, chat, hint)
        canvas[lay.top_h:lay.top_h + lay.img_h] = img
        self._strip(canvas, session)
        self._buttons(canvas, buttons)
        return canvas

    def _blocked(self, x0, y0, x1, y1, margin: int = 12) -> int:
        """Number of protected points inside the rectangle (grown by margin)."""
        P = self._protect
        if not len(P):
            return 0
        return int(np.count_nonzero((P[:, 0] >= x0 - margin) & (P[:, 0] <= x1 + margin) &
                                    (P[:, 1] >= y0 - margin) & (P[:, 1] <= y1 + margin)))

    def _strip(self, canvas: np.ndarray, session) -> None:
        lay, T = self.lay, self.text
        idx, sub = progress_stage(session)
        if idx >= 0:                       # which of the two phases: outbound first, then the way back
            sub = ("가는 길" if getattr(session, "phase", "outbound") == "outbound" else "돌아오는 길") + \
                (f" · {sub}" if sub else "")
        size = 17
        h = 30
        y0 = (lay.top_h - h) // 2
        x = 12
        for i, name in enumerate(STAGES):
            w = T.width(name, size) + 28
            reached, current = i <= idx, i == idx
            fill = GREEN if reached else WHITE
            round_rect(canvas, x, y0, x + w, y0 + h, fill, 1.0, r=h // 2,
                       border=OUTLINE if current else None, thickness=3)
            spr = T.sprite(name, size, (255, 255, 255) if reached else (60, 60, 60))
            blit(canvas, spr, x + (w - spr.shape[1]) // 2, y0 + (h - spr.shape[0]) // 2)
            x += w
            if i < len(STAGES) - 1:
                col = GREEN if i + 1 <= idx else WHITE
                cv2.arrowedLine(canvas, (x + 6, y0 + h // 2), (x + 30, y0 + h // 2), col, 3, cv2.LINE_AA,
                                tipLength=0.4)
                x += 36
        if sub:
            spr = T.sprite(sub, 15, (230, 230, 230))
            blit(canvas, spr, x + 16, (lay.top_h - spr.shape[0]) // 2)
        t = session.target
        info = (f"목표 r {t.r_m:.3f} m · {np.degrees(t.theta_rad):+.0f}°" if t is not None else
                "홈 복귀" if session.path is not None else "")
        if info:
            spr = T.sprite(info, 13, (170, 170, 170))
            blit(canvas, spr, lay.width - spr.shape[1] - 12, (lay.top_h - spr.shape[0]) // 2)

    def _notices(self, img: np.ndarray, notices: list) -> None:
        """Top-left; moved to the bottom-left when the path/targets are under the top-left spot."""
        T = self.text
        sprites = [T.sprite(s, 15, (255, 255, 255)) for s in notices[-2:]]
        if not sprites:
            return

        def stack(top: bool):
            rects, y = [], 10 if top else self.lay.img_h - 10
            for spr in (sprites if top else sprites[::-1]):
                h, w = spr.shape[0] + 10, spr.shape[1] + 20
                r = (10, y, 10 + w, y + h) if top else (10, y - h, 10 + w, y)
                rects.append((spr, r))
                y = r[3] + 6 if top else r[1] - 6
            return rects
        plans = [stack(True), stack(False)]
        plan = min(plans, key=lambda pl: sum(self._blocked(*r) for _, r in pl))   # ties: top-left
        for spr, (x0, y0, x1, y1) in plan:
            round_rect(img, x0, y0, x1, y1, (20, 20, 20), 0.72, r=8)
            blit(img, spr, x0 + 10, y0 + 5)
            self.ui_rects.append((x0, y0, x1, y1))

    def _bubbles(self, img: np.ndarray, chat: list, hint: str) -> None:
        """chat: [(t_monotonic, role, text)] oldest first. Right column; stacked from the bottom (newest
        lowest, KakaoTalk style) or, when the current path/targets are there, from the top - whichever shows
        more recent messages without covering a protected point. Messages that do not fit are left out."""
        T, lay = self.text, self.lay
        col_w = int(0.34 * lay.img_w)
        col_x1 = lay.img_w - 10
        col_x0 = col_x1 - col_w
        size, pad, gap = 15, 8, 6
        line_h = T.sprite("가", size).shape[0]
        name_h = T.sprite("쿠옹이", 12).shape[0] + 2
        now = time.monotonic()
        items = []                                   # oldest first
        for t, role, s in [(t, r, s) for t, r, s in chat if now - t < 90.0][-4:]:
            lines = T.wrap(s, size, col_w - 2 * pad - 30)
            bw = max(T.width(l, size) for l in lines) + 2 * pad
            bh = line_h * len(lines) + 2 * pad - 4
            user = role == "user"
            x1 = col_x1 if user else col_x0 + bw
            fade = 1.0 if now - t < 60.0 else max(0.0, 1.0 - (now - t - 60.0) / 30.0)
            items.append({"lines": lines, "x0": x1 - bw, "x1": x1, "h": bh, "label": 0 if user else name_h,
                          "user": user, "fade": fade})
        hint_item = None
        if hint:
            spr = T.sprite(hint, 13, (235, 235, 235))
            hint_item = {"spr": spr, "x0": col_x1 - spr.shape[1] - 18, "x1": col_x1, "h": spr.shape[0] + 8}

        def place(anchor: str, chosen: list):
            """[(item, (x0, y0, x1, y1))] for the hint + chosen bubbles (chronological), or None if blocked."""
            seq = ([hint_item] if hint_item else []) + (chosen[::-1] if anchor == "bottom" else chosen)
            out, y = [], (lay.img_h - 10 if anchor == "bottom" else 10)
            for it in seq:
                full = it["h"] + it.get("label", 0)
                if anchor == "bottom":
                    y0, y1 = y - full, y
                    y = y0 - gap
                else:
                    y0, y1 = y, y + full
                    y = y1 + gap
                if y0 < 8 or y1 > lay.img_h - 8 or self._blocked(it["x0"], y0, it["x1"], y1):
                    return None
                out.append((it, (it["x0"], y0 + it.get("label", 0), it["x1"], y1)))
            return out

        best = None
        for anchor in ("bottom", "top"):
            for k in range(len(items), -1, -1):        # as many of the newest messages as fit
                plan = place(anchor, items[len(items) - k:])
                if plan is not None:
                    if best is None or k > best[0]:
                        best = (k, plan)
                    break
        if best is None:                                  # even the hint alone is blocked: show nothing
            return
        for it, (x0, y0, x1, y1) in best[1]:
            if it is hint_item:
                round_rect(img, x0, y0, x1, y1, (40, 40, 40), 0.65, r=10)
                blit(img, it["spr"], x0 + 9, y0 + 4)
            else:
                round_rect(img, x0, y0, x1, y1, (0, 229, 254) if it["user"] else (255, 255, 255),
                           0.80 * it["fade"], r=10)
                for i, line in enumerate(it["lines"]):
                    blit(img, T.sprite(line, size, (20, 20, 20)), x0 + pad, y0 + pad - 2 + i * line_h, it["fade"])
                if not it["user"]:
                    name = T.sprite("쿠옹이", 12, (235, 235, 235))
                    blit(img, name, x0 + 4, y0 - name.shape[0] - 2, it["fade"])
            self.ui_rects.append((x0, y0, x1, y1))

    def _buttons(self, canvas: np.ndarray, buttons: list) -> None:
        lay = self.lay
        h0 = lay.top_h + lay.img_h
        bw = lay.width / len(buttons)
        for i, (k, label, on) in enumerate(buttons):
            x0, x1 = int(i * bw) + 2, int((i + 1) * bw) - 2
            fill = ((40, 150, 40) if k == "Y" else (90, 90, 90)) if on else (45, 45, 45)
            cv2.rectangle(canvas, (x0, h0 + 3), (x1, lay.height - 3), fill, -1)
            cv2.putText(canvas, f"{k} {label}", (x0 + 4, lay.height - 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255) if on else (120, 120, 120), 1, cv2.LINE_AA)

    def button_at(self, x: int, y: int, buttons: list) -> str | None:
        if self.lay.region(x, y) != "bar":
            return None
        return buttons[min(len(buttons) - 1, int(x * len(buttons) / self.lay.width))][0]
