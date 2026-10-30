"""캘리브레이션 결과를 눈으로 확인하기: RGB 화면을 클릭하면 그 점의 base_ref 좌표를 화면과 터미널에 보여 준다.

python scripts/show_base_coords.py                         카메라 + config/cam2base.yaml (없으면 실행 거부)
python scripts/show_base_coords.py --config-dir <폴더>      그 폴더의 cam2base.yaml·pose.yaml
python scripts/show_base_coords.py --session <세션.npz>     카메라 없이: 세션 정지 화면 + 그 세션으로 다시 계산한 보정
  --no-gui --click U,V [--click U,V ...]                    (--session과 함께) 창 없이 좌표만 출력

좌표 두 가지:
  P (책상면)  클릭 광선과 보정 책상면(z = 0)의 교점. 책상 위 점에만 맞다. depth가 없어도 나온다.
  D (depth)   클릭 → depth 픽셀 → 여러 프레임 평균 depth(주변 중앙값) → 역투영. 높이가 있는 점용.
키: c 점 지우기, g 격자, f 화면 정지/재개, s 저장(png·csv), q 종료
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from pac_vision import AstraDepth
from pac_vision.base_coords import D_REASONS, click_coords, desk_extent, grid_polylines
from pac_vision.cam2base import Cam2Base, CameraModel
from pac_vision.config import has_pose
from pac_vision.session import MissingCalibration, calibration_matches_current, replay_session, require_cam2base

import _ui
from _ui import GRAY, GREEN, WHITE, YELLOW, log

WIN = "pac_vision base coords"
OUTLINE = (0, 165, 255)         # calibrate 결과 화면의 A4 외곽선과 같은 색 (주황, BGR)
CHECK_NAMES = {"base_error": "base 점 오차", "a4_sides": "A4 변 최대", "reflection": "반사 검사"}


# ---------------------------------------------------------------------------
# 보정 읽기
# ---------------------------------------------------------------------------
class Calib:
    """쓰는 보정과 그 출처. kind: "cam2base" (실시간) / "session" (세션 재계산)."""

    def __init__(self, conv: Cam2Base, kind: str, path: Path, created: str, layout: dict):
        self.conv, self.kind, self.path, self.created, self.layout = conv, kind, path, created, layout
        self.coef_source = ""


def load_live_calib(cfg_dir: Path, cfg) -> Calib:
    """<config-dir>/cam2base.yaml. 없으면 MissingCalibration (예시 파일로 대체하지 않는다)."""
    path = (cfg_dir / "cam2base.yaml").resolve()
    conv = require_cam2base(path)
    layout = cfg["a4_layout"]
    sess = conv.info.get("session_file")
    if sess and (cfg_dir.parent / sess).exists():        # the layout the calibration was made with (one / two sheets)
        from pac_vision.session import load_session
        layout = load_session(cfg_dir.parent / sess)["configs"]["a4_layout"]
    return Calib(conv, "cam2base", path, str(conv.info.get("created")), layout)


def load_session_calib(session_path: Path) -> tuple[Calib, dict]:
    """세션에 저장된 설정 사본으로 --replay와 같은 계산을 해서 그 세션 자체의 보정을 만든다."""
    res, s = replay_session(session_path)
    cam = CameraModel.from_config(s["configs"]["camera"])
    info = res.summary()
    conv = Cam2Base(res.R, res.t, cam, info=info)
    return Calib(conv, "session", session_path.resolve(), s["created"], s["configs"]["a4_layout"]), s


def mm(x: float) -> str:
    """m → mm 문자열, 소수 한 자리 (끝의 .0은 뺌: 0.005 → '5')."""
    s = f"{x * 1000:.1f}"
    return s[:-2] if s.endswith(".0") else s


def check_text(info: dict) -> tuple[str, bool]:
    """cam2base.yaml(또는 세션 결과)의 검사 결과를 숫자 그대로 한 줄로. 기준값은 보정 때 기록된 값."""
    checks = info.get("checks") or {}
    failed = []
    for key, c in checks.items():
        if not c.get("blocking", True) or c.get("ok", True):
            continue
        name = CHECK_NAMES.get(key, key)
        if key == "reflection":
            failed.append(f"{name} 실패 (카메라 base z {c['value']:+.3f} m)")
        else:
            failed.append(f"{name} {mm(c['value'])} mm > 허용 {mm(c['limit'])} mm")
    if not checks:
        return "검사: 기록 없음", False
    if failed:
        return "검사: 기존 기준 미통과 (" + ", ".join(failed) + ")", False
    return "검사: 기존 기준 통과", True


def status_lines(calib: Calib, cfg_dir: Path, cfg) -> list:
    """[(한글, 영어, 색)] — 화면 위 패널과 터미널 시작 출력에 같은 내용."""
    info = calib.conv.info
    hold = info.get("holdout_m", float("nan"))
    if calib.kind == "cam2base":
        src_ko, src_en = f"보정 파일: {calib.path}", f"calibration: {calib.path}"
        copy_ko, copy_en = "cam2base.yaml 안 사본(보정 때)", "copy in cam2base.yaml"
    else:
        src_ko, src_en = f"보정: 세션 {calib.path} 를 다시 계산", f"calibration: replay of {calib.path}"
        copy_ko, copy_en = "세션 안 사본(보정 때)", "copy in session"
    chk, ok = check_text(info)
    pose = (cfg_dir / "pose.yaml").resolve()
    if has_pose(cfg_dir):
        same = calibration_matches_current(calib.conv, cfg["camera"])
        now_ko = f"지금 {pose}과 {'같음 ✅' if same else '다름 ⚠️'}"
        now_en = f"now {pose}: {'same' if same else 'DIFFERENT'}"
    else:
        same = False
        now_ko, now_en = f"지금 {pose} 없음", f"no {pose}"
    calib.coef_source = f"{copy_en}; {now_en}"
    return [
        (src_ko, src_en, WHITE),
        (f"  생성 {calib.created} · base 점 오차 {mm(hold)} mm", f"  created {calib.created}, base error {mm(hold)} mm",
         WHITE),
        (f"  {chk}", f"  checks: {'pass' if ok else 'FAIL (see terminal)'}", GREEN if ok else YELLOW),
        (f"정합 계수: {copy_ko}", f"RGB->depth coef: {copy_en}", WHITE),
        (f"  {now_ko}", f"  {now_en}", GREEN if same else YELLOW),
    ]


# ---------------------------------------------------------------------------
# 클릭 → 좌표
# ---------------------------------------------------------------------------
def cm(xs) -> str:
    """m 좌표들 → '12.3, -4.5' (cm, 소수 한 자리, -0.0은 0.0으로)."""
    return ", ".join(f"{round(x * 100, 1) + 0.0:.1f}" for x in xs)


def coords_label(cc) -> str:
    """'P (12.3, -4.5) cm | D (12.1, -4.9, 4.7) cm' (화면 라벨, 영문·숫자만)."""
    p = f"P ({cm(cc.p[:2])}) cm" if cc.p_ok else "P none"
    d = f"D ({cm(cc.d)}) cm" if cc.d_ok else "D no depth"
    return f"{p} | {d}"


def coords_text(cc) -> str:
    """터미널용 (depth가 없으면 이유를 한글로)."""
    p = f"P ({cm(cc.p[:2])}) cm" if cc.p_ok else "P 없음 (광선이 책상면과 만나지 않음)"
    d = f"D ({cm(cc.d)}) cm" if cc.d_ok else f"D {D_REASONS.get(cc.d_reason, cc.d_reason)}"
    return f"{p} | {d}"


def frames_text(cc, n_frames, session: bool) -> str:
    if session:
        return "depth 세션 평균"
    return f"depth 유효 {cc.valid_frames if cc.valid_frames is not None else '-'}/{n_frames}프레임"


class Recorder:
    """클릭 기록 (화면에는 최근 max_points개, 저장에는 전부)."""

    def __init__(self, calib: Calib):
        self.calib = calib
        self.rows = []          # (시각, ClickCoords, 유효 프레임 문자열)
        self.shown = []         # 화면에 남은 (번호, ClickCoords)

    def add(self, cc, frames: str, max_points: int):
        self.rows.append((datetime.now().isoformat(timespec="seconds"), cc, frames))
        self.shown = (self.shown + [(len(self.rows), cc)])[-max_points:]

    def write_csv(self, path: Path):
        c = self.calib
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time", "u", "v", "P_x_m", "P_y_m", "D_x_m", "D_y_m", "D_z_m", "D_note",
                        "depth_valid_frames", "cam2base", "cam2base_created", "rgb_depth_coef_source"])
            for t, cc, frames in self.rows:
                p = [f"{x:.4f}" for x in cc.p[:2]] if cc.p_ok else ["", ""]
                d = [f"{x:.4f}" for x in cc.d] if cc.d_ok else ["", "", ""]
                w.writerow([t, f"{cc.u:.0f}", f"{cc.v:.0f}", *p, *d, D_REASONS.get(cc.d_reason, ""), frames,
                            str(c.path), c.created, c.coef_source])


# ---------------------------------------------------------------------------
# 그리기
# ---------------------------------------------------------------------------
def overlay(rgb, calib: Calib, rgb_size, axis_len, grid, points):
    vis = rgb.copy()
    conv = calib.conv
    if grid:
        _ui.polylines(vis, grid, GRAY, 1)
    corners = np.array(calib.layout["corners"], float)
    corners = corners[[int(i) - 1 for i in calib.layout.get("cycle", range(1, len(corners) + 1))]]   # edge order
    poly = conv.base_to_pixel(corners, rgb_size=rgb_size)
    if np.all(np.isfinite(poly)):
        cv2.polylines(vis, [np.round(poly).astype(np.int32)], True, OUTLINE, 2, cv2.LINE_AA)
    _ui.base_axes(vis, conv, axis_len, rgb_size)
    # 점마다 번호, 마지막 점에만 좌표 라벨 (긴 라벨이 겹쳐 읽히지 않으므로). 모든 점의 좌표는 터미널과 아래 줄에.
    for n, cc in points:
        p = (cc.u, cc.v)
        cv2.circle(vis, _ui.ipt(p), 4, YELLOW, -1, cv2.LINE_AA)
        cv2.circle(vis, _ui.ipt(p), 6, (0, 0, 0), 1, cv2.LINE_AA)
        _ui.label_text(vis, str(n), (int(cc.u) + 7, int(cc.v) - 7), YELLOW)
    if points:
        n, cc = points[-1]
        _ui.label_text(vis, f"{n}: {coords_label(cc)}", (int(cc.u) + 7, int(cc.v) + 22), YELLOW)
    return vis


# ---------------------------------------------------------------------------
# 실행
# ---------------------------------------------------------------------------
def click_type(s: str):
    try:
        u, v = (int(round(float(a))) for a in s.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"U,V 형식이어야 합니다: {s}")
    return u, v


def save_outputs(out_dir: Path, canvas, rec: Recorder):
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f"demo_{datetime.now():%Y-%m-%d_%H%M%S}"
    if canvas is not None:
        cv2.imwrite(str(stem.with_suffix(".png")), canvas)
    rec.write_csv(stem.with_suffix(".csv"))
    log(f"[저장] {stem}.png, {stem}.csv  (클릭 {len(rec.rows)}개)")


def run_no_gui(calib, s, clicks, half):
    rgb_size = s["rgb_size"]
    for i, (u, v) in enumerate(clicks, 1):
        cc = click_coords(calib.conv, u, v, s["depth"], half, rgb_size=rgb_size)
        log(f"[점 {i}] RGB ({u}, {v}) → {coords_text(cc)} | {frames_text(cc, 0, True)}")
    return 0


def demo_settings(cfg, cfg_dir: Path) -> dict:
    """thresholds.yaml의 demo: 절. 없으면 (예전에 복사한 설정 폴더 등) 무엇을 할지 알리고 멈춘다."""
    demo = cfg["thresholds"].get("demo") or {}
    missing = [k for k in ("grid_step_m", "max_points") if k not in demo]
    if missing:
        sys.exit(f"[pac_vision] {cfg_dir / 'thresholds.yaml'}에 demo: 절의 키 {', '.join(missing)}가 없습니다.\n"
                 f"  예전에 복사한 설정 폴더라면 저장소의 {_ui.DEFAULT_CONFIG_DIR / 'thresholds.yaml'} 맨 끝 demo: 절을 "
                 "그 파일 끝에 복사하세요 (기존 값은 그대로)")
    return demo


def run_gui(args, cfg, cfg_dir, calib, session, header):
    thr, demo = cfg["thresholds"], demo_settings(cfg, cfg_dir)
    half = int(thr["calibration"]["point_patch_half_px"])
    n_frames, min_frac = int(thr["capture"]["average_frames"]), float(thr["capture"]["min_valid_fraction"])
    far_max, axis_len = float(thr["placement"]["far_max_m"]), float(cfg["camera"]["ui"]["axis_len_m"])
    max_points = int(demo["max_points"])

    cap = dev = None
    if session is not None:
        rgb_size = session["rgb_size"]
    else:
        cap, rgb_size = _ui.open_rgb(cfg["camera"])
        try:
            dev = AstraDepth.from_config(cfg["camera"]).open()
        except Exception as e:      # openni 없음, 장치 사용 중 등
            cap.release()
            sys.exit(f"[pac_vision] depth 카메라(OpenNI)를 열지 못했습니다: {e}\n"
                     "  다른 프로그램(calibrate.py, detect_objects.py, path editor)이 카메라를 쓰는 중인지 확인하고, "
                     "openni.redist 경로(config/camera.yaml 또는 local.yaml)를 확인하세요")
        log(f"[pac_vision] RGB 열림 {rgb_size[0]}x{rgb_size[1]} / depth(OpenNI) 열림")

    grid = grid_polylines(calib.conv, desk_extent(calib.conv, rgb_size, far_max), float(demo["grid_step_m"]),
                          rgb_size, far_max)
    rec = Recorder(calib)
    text = _ui.TextDrawer(cfg["camera"]["ui"]["font"])
    screen = _ui.HeaderScreen(WIN, text, cfg["camera"]["ui"]["font_size"])
    show_grid, frozen = False, None         # frozen: (rgb, depth, 유효 비율) 정지 화면
    if session is not None:
        frozen = (session["rgb"], session["depth"], None)

    def measure(c):
        if session is not None:
            depth, frac, dt = session["depth"], None, 0.0
        elif frozen is not None:
            depth, frac, dt = frozen[1], frozen[2], 0.0
        else:
            t0 = time.time()
            depth, frac = dev.read_average(n_frames, min_frac)
            dt = time.time() - t0
        cc = click_coords(calib.conv, c[0], c[1], depth, half, rgb_size=rgb_size, valid_frac=frac, n_frames=n_frames)
        frames = frames_text(cc, n_frames, session is not None)
        rec.add(cc, frames, max_points)
        extra = "" if session is not None or frozen is not None else f" (depth 평균 {dt:.1f} s)"
        log(f"[점 {len(rec.rows)}] {datetime.now():%H:%M:%S} RGB ({c[0]}, {c[1]}) → {coords_text(cc)} | {frames}{extra}")

    for c in args.click or []:
        measure(c)
    rgb = frozen[0] if frozen is not None else None
    try:
        while True:
            if frozen is None:
                ok, live = cap.read()
                if not ok:
                    continue
                rgb = live
            c = screen.take_click()
            if c is not None and 0 <= c[0] < rgb.shape[1] and 0 <= c[1] < rgb.shape[0]:
                measure(c)
            vis = overlay(rgb, calib, rgb_size, axis_len, grid if show_grid else None, rec.shown)
            _ui.magnifier(vis, screen.mouse)
            last = rec.rows[-1][1] if rec.rows else None
            lines = []
            if last is not None:
                lines.append((f"마지막 점 {len(rec.rows)}: RGB ({last.u:.0f}, {last.v:.0f}) → {coords_text(last)}",
                              f"last {len(rec.rows)}: RGB ({last.u:.0f}, {last.v:.0f}) -> {coords_label(last)}", YELLOW))
            lines.append(("P: 보정 책상면(z = 0) 교점 · D: depth 역투영   빨강 X(앞) 초록 Y(왼쪽) 파랑 Z(위) 주황: A4 외곽선",
                          "P: desk plane z=0  D: depth   red X(fwd) green Y(left) blue Z(up) orange: A4", GRAY))
            lines.append(("c: 점 지우기   g: 격자   f: 정지/재개   s: 저장   q: 종료",
                          "c: clear  g: grid  f: freeze  s: save  q: quit", WHITE))
            top = None
            if frozen is not None and session is None:
                top = ("정지 화면 (f: 재개)", "frozen (f: resume)", YELLOW)
            screen.show(vis, lines, top=top, header=header)
            k = screen.key(15)          # ⚠️ 창 백엔드(GTK/Qt)에 따라 키 코드가 다를 수 있음 (영문 소문자 기준)
            if k == ord("q"):
                return
            if k == ord("c"):
                rec.shown = []
                log("[지움] 화면의 점을 지웠습니다 (저장 기록은 유지)")
            elif k == ord("g"):
                show_grid = not show_grid
                if show_grid and not grid:
                    log("[격자] 그릴 책상면 범위가 없습니다")
            elif k == ord("f") and session is None:
                if frozen is None:
                    depth, frac = dev.read_average(n_frames, min_frac)
                    frozen = (rgb.copy(), depth, frac)
                    log(f"[정지] RGB 1장 + depth {n_frames}프레임 평균. 이 화면에서 클릭하면 이 depth를 씁니다")
                else:
                    frozen = None
                    log("[재개] 실시간 화면")
            elif k == ord("s"):
                save_outputs(cfg_dir.parent / "data" / "sessions", screen.canvas, rec)
    finally:
        if cap is not None:
            cap.release()
        if dev is not None:
            dev.close()
        cv2.destroyAllWindows()


def main(argv=None):
    ap = argparse.ArgumentParser(description="RGB 화면 클릭 → base_ref 좌표 (캘리브레이션 결과 확인)")
    ap.add_argument("--config-dir", default=str(_ui.DEFAULT_CONFIG_DIR))
    ap.add_argument("--session", metavar="SESSION.npz", help="카메라 없이: 세션 정지 화면과 그 세션으로 다시 계산한 보정")
    ap.add_argument("--no-gui", action="store_true", help="--session과 함께: 창 없이 --click 점의 좌표만 출력")
    ap.add_argument("--click", type=click_type, action="append", metavar="U,V", help="--session과 함께: 미리 클릭할 RGB 픽셀")
    args = ap.parse_args(argv)
    if (args.no_gui or args.click) and not args.session:
        ap.error("--no-gui, --click은 --session과 함께 씁니다")
    if args.no_gui and not args.click:
        ap.error("--no-gui에는 --click이 하나 이상 필요합니다")

    cfg_dir = Path(args.config_dir).resolve()
    cfg = _ui.load_config_or_exit(cfg_dir, ("camera", "a4_layout", "thresholds"))
    log(f"[pac_vision] 설정 폴더: {cfg_dir}")
    session = None
    if args.session:
        calib, session = load_session_calib(Path(args.session))
    else:
        try:
            calib = load_live_calib(cfg_dir, cfg)
        except MissingCalibration as e:
            sys.exit(f"[pac_vision] {e}")
    header = status_lines(calib, cfg_dir, cfg)
    for ko, _en, _c in header:
        log(f"[pac_vision] {ko.strip()}")
    if args.no_gui:
        return run_no_gui(calib, session, args.click, int(cfg["thresholds"]["calibration"]["point_patch_half_px"]))
    run_gui(args, cfg, cfg_dir, calib, session, header)
    return 0


if __name__ == "__main__":
    sys.exit(main())
