"""스크립트 공용 화면 도우미 (cv2 + Pillow). 핵심 모듈(pac_vision)에는 넣지 않는다.

한글: OpenCV putText는 한글을 못 그리므로 Pillow + 한글 글꼴로 그린다.
Pillow가 없거나 글꼴이 없으면 같은 경로로 영어 문구 + cv2.putText로 대체한다 (⚠️ 팀원 컴퓨터에서 달라질 수 있음).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_DIR = SCRIPTS_DIR.parent / "config"

GREEN, RED, YELLOW, WHITE, GRAY = (0, 200, 0), (0, 0, 255), (0, 255, 255), (255, 255, 255), (160, 160, 160)
BLUE, CYAN, PURPLE, BLACK = (255, 0, 0), (255, 255, 0), (200, 0, 200), (0, 0, 0)

PANEL_MIN_W = 900       # 문구 패널 최소 폭 (영상 640 px보다 긴 줄이 잘리지 않게)
ENTER_KEYS = (13, 10)   # ⚠️ 창 백엔드(GTK/Qt)에 따라 Enter 코드가 13 또는 10
opened_rgb_device: int | None = None   # open_rgb가 실제로 연 /dev/videoN 번호 (보정 파일에 카메라 신원으로 남긴다)
_camera_lock = None                    # 카메라를 이 프로그램만 쓰는 동안 열어 두는 잠금 파일 (devices.camera_lock)


def log(msg: str):
    print(msg, flush=True)


class TextDrawer:
    """draw(img, [(한글, 영어, (x, y), 색, 크기), ...]) — 한 번에 모아 그린다."""

    def __init__(self, font_path: str | None):
        self.korean = False
        self._fonts = {}
        self._path = None
        try:
            from PIL import Image, ImageDraw, ImageFont  # noqa: F401
            path = os.path.expanduser(font_path or "")
            if path and os.path.exists(path):
                ImageFont.truetype(path, 16)
                self._path = path
                self.korean = True
        except Exception:
            self.korean = False
        if not self.korean:
            log("[pac_vision] ⚠️ 한글 글꼴 또는 Pillow가 없어 화면 문구를 영어로 표시합니다 (터미널은 한글)")

    def _font(self, size):
        from PIL import ImageFont
        if size not in self._fonts:
            self._fonts[size] = ImageFont.truetype(self._path, size)
        return self._fonts[size]

    def draw(self, img, items, bg=True):
        if not items:
            return img
        if self.korean:
            from PIL import Image, ImageDraw
            pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            d = ImageDraw.Draw(pil)
            for ko, _en, (x, y), color, size in items:
                ko = screen_text(ko)
                font = self._font(size)
                if bg:
                    l, t, r, b = d.textbbox((x, y), ko, font=font)
                    d.rectangle((l - 3, t - 2, r + 3, b + 2), fill=(0, 0, 0))
                d.text((x, y), ko, font=font, fill=(color[2], color[1], color[0]))
            img[:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
        else:
            for _ko, en, (x, y), color, size in items:
                scale = size / 30.0
                (w, h), base = cv2.getTextSize(en, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
                if bg:
                    cv2.rectangle(img, (x - 3, y - 2), (x + w + 3, y + h + base + 2), BLACK, -1)
                cv2.putText(img, en, (x, y + h), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        return img


def screen_text(s: str) -> str:
    """한글 글꼴(Noto Sans CJK)에 없는 기호 바꾸기: ✅ → ✓, 이모지 변형 선택자 제거 (터미널은 그대로)."""
    return s.replace("✅", "✓").replace("\ufe0f", "")


def depth_colormap(depth_mm, max_mm=4000):
    d = np.clip(np.asarray(depth_mm, dtype=np.float32), 0, max_mm) / max_mm
    vis = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_JET)
    vis[np.asarray(depth_mm) <= 0] = 0
    return vis


def ipt(p):
    return int(round(float(p[0]))), int(round(float(p[1])))


def finite(p):
    return bool(np.all(np.isfinite(np.asarray(p, dtype=float))))


def dashed_rect(img, p0, p1, color, dash=8):
    x0, y0 = ipt(p0)
    x1, y1 = ipt(p1)
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(np.linalg.norm(b - a) // dash))
        for i in range(0, n, 2):
            s, e = a + (b - a) * i / n, a + (b - a) * min(i + 1, n) / n
            cv2.line(img, ipt(s), ipt(e), color, 1, cv2.LINE_AA)


def arrow(img, p0, p1, color, thickness=2):
    if finite(p0) and finite(p1):
        cv2.arrowedLine(img, ipt(p0), ipt(p1), color, thickness, cv2.LINE_AA, tipLength=0.15)


def base_axes(img, conv, axis_len, rgb_size):
    """base 원점에서 빨강 X, 초록 Y, 파랑 Z(위) 화살표. Z를 먼저 그려 X·Y가 위에 오게 한다.
    끝점이 카메라 뒤(nan)인 축은 arrow()가 건너뛴다."""
    L = float(axis_len)
    o, x, y, z = conv.base_to_pixel(np.array([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]], float), rgb_size=rgb_size)
    arrow(img, o, z, BLUE, 2)
    arrow(img, o, x, RED, 2)
    arrow(img, o, y, GREEN, 2)


def magnifier(img, cursor, zoom=4, half=20, size=None, top=40):
    """커서 주변을 확대해 커서 반대쪽 모서리에 붙인다. 가운데 십자가 실제 클릭 위치."""
    if cursor is None:
        return img
    h, w = img.shape[:2]
    x, y = ipt(cursor)
    if not (0 <= x < w and 0 <= y < h):     # 커서가 영상 밖(문구 패널 등)이면 그리지 않는다
        return img
    pad = cv2.copyMakeBorder(img, half, half, half, half, cv2.BORDER_CONSTANT, value=0)
    patch = pad[y:y + 2 * half + 1, x:x + 2 * half + 1]
    side = size or (2 * half + 1) * zoom
    big = cv2.resize(patch, (side, side), interpolation=cv2.INTER_NEAREST)
    c = side // 2
    cv2.line(big, (c, 0), (c, side), YELLOW, 1)
    cv2.line(big, (0, c), (side, c), YELLOW, 1)
    cv2.rectangle(big, (0, 0), (side - 1, side - 1), WHITE, 1)
    ox = 8 if x > w // 2 else w - side - 8
    oy = top if y > h // 2 else h - side - 8      # 위쪽에 붙일 때는 안내 문구 아래로
    img[oy:oy + side, ox:ox + side] = big
    return img


def load_config_or_exit(cfg_dir, names):
    """config 읽기 + local.yaml·pose.yaml 덮어쓰기. 키가 약속과 다르면 어느 파일의 어느 키인지 알리고 멈춘다."""
    from pac_vision.config import ConfigError, config_sources, load_config
    try:
        cfg = load_config(cfg_dir, names)
        sources = config_sources(cfg_dir)
    except ConfigError as e:
        sys.exit(f"[pac_vision] 설정 오류: {e}")
    for line in sources:
        log(f"[pac_vision] 설정 덮어씀: {line}")
    return cfg


def rgb_device_candidates(name_match: str, root="/sys/class/video4linux"):
    """장치 이름에 name_match가 들어간 /dev/videoN 번호 (작은 순). ⚠️ 리눅스 전용 (Jazzy에서 이름 확인 대상)."""
    out = []
    for p in Path(root).glob("video*"):
        try:
            name = (p / "name").read_text(encoding="utf-8").strip()
            n = int(p.name[5:])
        except (OSError, ValueError):
            continue
        if name_match.lower() in name.lower():
            out.append((n, name))
    return sorted(out)


class Rotated180:
    """cv2.VideoCapture처럼 쓰되 read()가 180° 돌린 프레임을 준다 (camera.rotate_180, depth도 같이 돌린다)."""

    def __init__(self, cap):
        self._cap = cap

    def read(self):
        ok, frame = self._cap.read()
        return ok, (np.ascontiguousarray(frame[::-1, ::-1]) if ok and frame is not None else frame)

    def __getattr__(self, name):
        return getattr(self._cap, name)


def open_rgb(camera_cfg):
    """Astra RGB(UVC) 열기. 실제로 받은 크기를 돌려준다.

    rgb.device가 있으면(local.yaml) 그 장치, 없으면 이름에 rgb.name_match가 들어간 장치 중 프레임이 읽히는 첫 장치."""
    global _camera_lock
    from pac_vision.devices import CameraBusy, camera_lock
    try:
        _camera_lock = camera_lock(f"pac_vision {Path(sys.argv[0]).name}")
    except CameraBusy as e:
        sys.exit(f"[pac_vision] {e}")
    r = camera_cfg["rgb"]
    if r.get("device") is not None:
        devices, auto = [int(r["device"])], False
    else:
        cands = rgb_device_candidates(str(r.get("name_match", "Astra")))
        devices, auto = [n for n, _ in cands], True
        if not devices:
            sys.exit(f"[pac_vision] 이름에 '{r.get('name_match', 'Astra')}'가 들어간 RGB 장치가 없습니다. "
                     "카메라 연결을 확인하거나 config/local.yaml에 rgb.device를 적으세요")
    for dev in devices:
        # ⚠️ CAP_V4L2는 리눅스 전용
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(r["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(r["height"]))
        ok, frame = False, None
        if cap.isOpened():
            for _ in range(5):
                ok, frame = cap.read()
        if ok:
            if auto:
                log(f"[pac_vision] RGB 장치 자동 선택: /dev/video{dev}")
            size = (frame.shape[1], frame.shape[0])
            if size != (int(r["width"]), int(r["height"])):
                log(f"[pac_vision] ⚠️ RGB 요청 {r['width']}x{r['height']} → 실제 {size[0]}x{size[1]}")
            global opened_rgb_device
            opened_rgb_device = dev
            if camera_cfg.get("rotate_180"):
                log("[pac_vision] 카메라를 뒤집어 단 설정(rotate_180): RGB·depth를 180° 돌려 읽습니다")
                return Rotated180(cap), size
            return cap, size
        cap.release()
    sys.exit(f"[pac_vision] RGB 카메라를 열지 못했습니다 (시도한 장치 {devices}). 다른 프로그램이 쓰는 중인지 "
             "(fuser -v /dev/video*) 확인하거나 config/local.yaml의 rgb.device를 확인하세요")


def compose(text, size, img, lines, top=None):
    """영상 아래에 문구 패널을 붙인 화면 한 장."""
    h, w = img.shape[:2]
    line_h = size + 7
    canvas = np.zeros((h + 10 + line_h * max(1, len(lines)), max(w, PANEL_MIN_W), 3), np.uint8)
    canvas[:h, :w] = img
    items = [(ko, en, (8, h + 6 + i * line_h), color, size) for i, (ko, en, color) in enumerate(lines)]
    if top is not None:
        items.append((top[0], top[1], (8, 8), top[2], size + 4))
    text.draw(canvas, items)
    return canvas


class Screen:
    """한 창에 영상 + 아래 문구 패널. 마우스는 영상 영역 안에서만 받는다."""

    def __init__(self, name: str, text: TextDrawer, size: int):
        self.name, self.text, self.size = name, text, size
        self.mouse, self.click = None, None
        self.img_h = 0
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if y >= self.img_h:
            return
        self.mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = (x, y)

    def take_click(self):
        c, self.click = self.click, None
        return c

    def show(self, img, lines, top=None):
        """lines: [(한글, 영어, 색)], top: 영상 위쪽에 띄울 안내 한 줄 (한글, 영어, 색)."""
        self.img_h = img.shape[0]
        cv2.imshow(self.name, compose(self.text, self.size, img, lines, top))

    def key(self, wait=15):
        k = cv2.waitKey(wait) & 0xFF
        try:
            if cv2.getWindowProperty(self.name, cv2.WND_PROP_VISIBLE) < 1:    # 창 닫기 버튼 = q
                return ord("q")
        except cv2.error:
            pass
        return k


# ---------------------------------------------------------------------------
# show_base_coords.py용 (추가만, 위 함수들의 동작은 그대로)
# ---------------------------------------------------------------------------
def header_panel(text, size, lines, width):
    """영상 위에 붙일 상태 패널. lines: [(한글, 영어, 색)]."""
    line_h = size + 6
    panel = np.zeros((8 + line_h * max(1, len(lines)), width, 3), np.uint8)
    text.draw(panel, [(ko, en, (8, 4 + i * line_h), color, size) for i, (ko, en, color) in enumerate(lines)])
    return panel


class HeaderScreen(Screen):
    """Screen + 영상 위 상태 패널. 마우스 좌표는 영상 기준(패널 높이를 뺀 값)으로 돌려준다."""

    def __init__(self, name: str, text: TextDrawer, size: int):
        self.header_h = 0
        self.canvas = None          # 마지막으로 띄운 화면 전체 (저장용)
        super().__init__(name, text, size)

    def _on_mouse(self, event, x, y, flags, param):
        # ⚠️ 창 백엔드(GTK/Qt)에 따라 창 밖 드래그 등에서 좌표가 음수일 수 있다 → 패널·영상 밖은 무시
        y -= self.header_h
        if y < 0:
            return
        super()._on_mouse(event, x, y, flags, param)

    def show(self, img, lines, top=None, header=()):
        self.img_h = img.shape[0]
        body = compose(self.text, self.size, img, lines, top)
        head = header_panel(self.text, max(10, self.size - 2), list(header), body.shape[1]) if header else \
            np.zeros((0, body.shape[1], 3), np.uint8)
        self.header_h = head.shape[0]
        self.canvas = np.vstack([head, body])
        cv2.imshow(self.name, self.canvas)


def polylines(img, lines, color, thickness=1):
    """꺾은선 목록 [(N, 2) 픽셀]을 그린다."""
    pts = [np.round(p).astype(np.int32) for p in lines if len(p) >= 2]
    if pts:
        cv2.polylines(img, pts, False, color, thickness, cv2.LINE_AA)


def label_text(img, s, org, color, scale=0.45):
    """영문·숫자 라벨 (검은 바탕 상자). 영상 밖으로 나가지 않게 위치를 당긴다."""
    (w, h), base = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x = int(min(max(3, org[0]), img.shape[1] - w - 3))
    y = int(min(max(h + 3, org[1]), img.shape[0] - base - 3))
    cv2.rectangle(img, (x - 3, y - h - 3), (x + w + 3, y + base + 2), BLACK, -1)
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
