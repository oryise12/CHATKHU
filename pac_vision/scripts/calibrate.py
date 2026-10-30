"""캘리브레이션 (카메라 → base_ref).

python scripts/calibrate.py                         장면 1: 배치 점검 → Enter 정지 → 클릭 5회 → 결과 → s 저장
python scripts/calibrate.py --layout aruco          ArUco 보정판(마커 4개): 클릭 대신 마커 중심을 자동으로 찾는다
python scripts/calibrate.py --check                 장면 3: 테이프 표시 클릭으로 재점검 (이어서 다른 점 클릭 시 base 좌표)
python scripts/calibrate.py --replay 세션.npz        장면 4: 카메라 없이 같은 계산 (--no-gui, --use-current-config)
공통: --depth-mode plane|point, --config-dir
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from pac_vision import AstraDepth
from pac_vision.cam2base import (Cam2Base, CalibrationError, CameraModel, average_depth, clip_roi,
                                 fit_desk_plane_near, patch_median, roi_points, solve_calibration)
from pac_vision.config import has_pose
from pac_vision.devices import rgb_identity
from pac_vision.markers import find_board, shortest_side_px, square_center
from pac_vision.session import (MissingCalibration, calibration_matches_current, compare_summaries, replay_session,
                                require_cam2base,
                                save_session, write_cam2base)

import _ui
from _ui import CYAN, ENTER_KEYS, GRAY, GREEN, RED, WHITE, YELLOW, log

WIN = "pac_vision calibrate"
CLICK_GUIDE = [
    ("① base 중심 십자를 클릭하세요", "1) click the base cross center", "base"),
    ("② 꼭짓점 1번을 클릭하세요", "2) click corner 1", "꼭짓점1"),
    ("③ 꼭짓점 2번을 클릭하세요", "3) click corner 2", "꼭짓점2"),
    ("④ 꼭짓점 3번을 클릭하세요", "4) click corner 3", "꼭짓점3"),
    ("⑤ 꼭짓점 4번을 클릭하세요", "5) click corner 4", "꼭짓점4"),
]
POINT_LABELS = ["0 base", "1", "2", "3", "4"]     # 결과 화면에서 이 번호 키로 그 점만 다시 클릭
POINT_NAMES = ["원점(0번)", "꼭짓점 1번", "꼭짓점 2번", "꼭짓점 3번", "꼭짓점 4번"]
SIDE_NAMES = ["1-2", "2-3", "3-4", "4-1"]
N_POINTS = 5            # 클릭 수 (원점을 찍지 않는 배치는 4)
KEY0 = 0                # 결과 화면에서 이 숫자 키가 클릭 목록의 0번 (원점을 찍지 않는 배치는 1 = 꼭짓점 1)
SHAPE = "A4"            # 기준점이 이루는 사각형 이름 (결과 화면)
KIND = "꼭짓점"          # 기준점 이름 (결과 화면): 꼭짓점 / 마커
ARUCO = None            # 보정판 배치의 aruco 설정 (마커 번호·사전·크기). None = 클릭 배치
LAYOUT_FILES = {"double": "a4_layout.yaml", "single": "a4_layout_single.yaml", "aruco": "a4_layout_aruco.yaml"}


def use_layout(layout):
    """배치(a4_layout*.yaml)에 맞춰 클릭 안내·번호 키·변 이름을 바꾼다. base_clicked: false면 꼭짓점 4개만 찍는다."""
    global CLICK_GUIDE, POINT_LABELS, POINT_NAMES, SIDE_NAMES, N_POINTS, KEY0, SHAPE, KIND, ARUCO
    cyc = [int(i) for i in layout.get("cycle", (1, 2, 3, 4))]
    SIDE_NAMES = [f"{a}-{b}" for a, b in zip(cyc, cyc[1:] + cyc[:1])]
    SHAPE = str(layout.get("shape_name", "A4"))
    ARUCO = layout.get("aruco")
    KIND = "마커" if ARUCO else "꼭짓점"
    if layout.get("base_clicked", True):
        return
    names = list(layout.get("click_names") or [f"꼭짓점 {i}번" for i in range(1, 5)])
    marks = "①②③④"
    CLICK_GUIDE = [(f"{marks[i]} {names[i]}을 클릭하세요", f"{i + 1}) click point {i + 1}",
                    names[i].split(" (")[0].replace(" ", "")) for i in range(4)]
    POINT_LABELS, POINT_NAMES, N_POINTS, KEY0 = ["1", "2", "3", "4"], names, 4, 1


def mark(ok: bool, fail: str = "주의") -> str:
    """통과 / 주의(진행은 됨) / 실패. 화면 글꼴(Noto Sans CJK)에 있는 기호만 쓴다."""
    return "✓ 통과" if ok else ("× " + fail if fail == "실패" else "⚠ " + fail)


# ---------------------------------------------------------------------------
# 장면 1-2 배치 점검, 1-3 클릭
# ---------------------------------------------------------------------------
def band_z(depth, cam, band):
    h = depth.shape[0]
    rows = depth[int(h * band[0]):int(h * band[1])]
    vals = rows[rows > 0]
    frac = vals.size / float(max(rows.size, 1))
    return (float(np.median(cam.depth_m(vals))) if vals.size else None), frac


def placement_lines(depth, cam, thr):
    pl = thr["placement"]
    near, near_frac = band_z(depth, cam, pl["near_band"])
    far, far_frac = band_z(depth, cam, pl["far_band"])
    lines = [("【배치 점검】 카메라가 책상을 잘 보는지 실시간으로 봅니다", "[live] camera placement check", WHITE),
             ("ⓘ 빨간 점 = 깊이(depth)를 못 잰 곳", "red dots: no depth", GRAY)]
    if near is None or near_frac < 0.5:
        lines.append(("⚠ 화면 아래쪽(가까운 쪽): 깊이를 거의 못 잼 → 카메라에 너무 가까울 수 있음 (0.6 m 안쪽은 못 잼)",
                      "! bottom of view too close (little depth)", YELLOW))
        lines.append(("   ▶ 카메라를 더 높이 / 뒤로 두거나 아래로 덜 숙이세요", "  > raise or move the camera back", YELLOW))
    elif near < pl["near_min_m"]:
        lines.append((f"⚠ 화면 아래쪽(가까운 쪽)까지 {near:.2f} m → 너무 가까움 (기준: {pl['near_min_m']} m 이상)",
                      f"near (bottom): {near:.2f} m ! too close (>= {pl['near_min_m']} m)", YELLOW))
        lines.append(("   ▶ 카메라를 더 높이 / 뒤로 두세요", "  > raise or move the camera back", YELLOW))
    else:
        lines.append((f"✓ 화면 아래쪽(가까운 쪽)까지 {near:.2f} m (기준: {pl['near_min_m']} m 이상)",
                      f"near (bottom): {near:.2f} m OK (>= {pl['near_min_m']} m)", GREEN))
    if far is None:
        lines.append(("⚠ 화면 위쪽(먼 쪽): 깊이를 못 잼 → 너무 멀거나 비어 있음", "far (top): no depth !", YELLOW))
    else:
        ok = far <= pl["far_max_m"]
        if ok:
            lines.append((f"✓ 화면 위쪽(먼 쪽)까지 {far:.2f} m (기준: {pl['far_max_m']} m 이하)",
                          f"far (top): {far:.2f} m OK (<= {pl['far_max_m']} m)", GREEN))
        else:
            lines.append((f"⚠ 화면 위쪽(먼 쪽)까지 {far:.2f} m → 거기 있는 것은 깊이 오차가 큼 (기준: {pl['far_max_m']} m 이하)",
                          f"far (top): {far:.2f} m ! (<= {pl['far_max_m']} m)", YELLOW))
            lines.append(("   ▶ A4와 로봇 작업 영역은 화면 아래쪽에 오게 두세요 (뒤쪽 벽·바닥만 멀면 괜찮음)",
                          "  > keep the A4 and the work area in the lower part of the view", YELLOW))
    valid = 100.0 * np.count_nonzero(depth) / depth.size
    lines.append((f"ⓘ 화면 중 깊이를 잰 비율: {valid:.0f} %", f"depth valid: {valid:.0f} %", WHITE))
    lines.append(("▶ Enter: 이 화면으로 보정 시작    q: 종료", "Enter: start with this view   q: quit", WHITE))
    return lines


def placement_loop(screen, cap, dev, cam, thr, rgb_size):
    log("[pac_vision] 실시간 화면에서 배치를 확인한 뒤 Enter를 누르세요.")
    step = int(thr["placement"]["invalid_dot_step_px"])
    vv, uu = np.mgrid[step // 2:cam.height:step, step // 2:cam.width:step]
    while True:
        ok, rgb = cap.read()
        depth = dev.read()
        if not ok:
            continue
        vis = rgb.copy()
        bad = depth[vv, uu] == 0
        us, vs = cam.depth_to_rgb(uu[bad], vv[bad], rgb_size, z_m=cam.z_ref_m)    # 표시용 (depth 없는 점이라 Z 근사)
        for x, y in zip(us, vs):
            if 0 <= x < rgb_size[0] and 0 <= y < rgb_size[1]:
                cv2.circle(vis, (int(x), int(y)), 1, RED, -1)
        screen.show(vis, placement_lines(depth, cam, thr))
        k = screen.key(1)
        if k in ENTER_KEYS:
            return freeze(cap, dev, thr)
        if k == ord("q"):
            return None


def freeze(cap, dev, thr):
    n = int(thr["capture"]["average_frames"])
    log(f"[정지] {n}프레임 평균 중...")
    rgbs, depths, last = [], [], None
    while len(depths) < n:
        ok, rgb = cap.read()
        d = dev.read()
        if dev.last_index is not None and dev.last_index == last:
            continue
        last = dev.last_index
        if ok:
            rgbs.append(rgb.astype(np.float32))
        depths.append(d)
    rgb = np.clip(np.mean(rgbs, axis=0) + 0.5, 0, 255).astype(np.uint8)
    depth, _ = average_depth(depths, thr["capture"]["min_valid_fraction"])
    log(f"[정지] 완료: RGB {len(rgbs)}장, depth {len(depths)}장 평균, 유효 {100 * np.count_nonzero(depth) / depth.size:.0f} %")
    return rgb, depth


def draw_clicks(vis, clicks, labels=None):
    for i, p in zip(labels if labels is not None else range(len(clicks)), clicks):
        cv2.circle(vis, _ui.ipt(p), 4, GREEN, -1)
        cv2.putText(vis, POINT_LABELS[i], (int(p[0]) + 6, int(p[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1,
                    cv2.LINE_AA)


def depth_missing(c, depth, cam, thr, rgb_size, mode) -> bool:
    """depth 모드 point에서 클릭 점 주변에 depth가 없으면 True (그 점은 쓸 수 없다)."""
    if mode != "point":
        return False
    half = int(thr["calibration"]["point_patch_half_px"])
    ud, vd = (float(a[0]) for a in cam.rgb_to_depth_on_depth(c[0], c[1], depth, half, rgb_size))
    return not (0 <= ud < cam.width and 0 <= vd < cam.height) or patch_median(depth, ud, vd, half) <= 0


def reclick_one(screen, rgb, depth, cam, thr, rgb_size, mode, clicks, i):
    """클릭 점 중 i번(클릭 목록 번호, 5점 배치는 0 = 원점)만 다시 찍는다. 반환: 새 클릭 목록 / None(취소 → 그대로)."""
    log(f"[다시 클릭] {POINT_NAMES[i]}만 다시 찍습니다 (Esc: 취소)")
    warn_until = 0
    while True:
        c = screen.take_click()
        if c is not None:
            if depth_missing(c, depth, cam, thr, rgb_size, mode):
                log("⚠ 이 점은 depth가 없습니다. 다시 클릭하세요")
                warn_until = cv2.getTickCount() + 2 * cv2.getTickFrequency()
            else:
                new = list(clicks)
                old, new[i] = new[i], c
                log(f"[다시 클릭] {POINT_NAMES[i]}: ({old[0]}, {old[1]}) → ({c[0]}, {c[1]})")
                return new
        vis = rgb.copy()
        draw_clicks(vis, [p for j, p in enumerate(clicks) if j != i], [j for j in range(N_POINTS) if j != i])
        cv2.circle(vis, _ui.ipt(clicks[i]), 7, GRAY, 1, cv2.LINE_AA)      # 지금 위치 (회색 고리)
        _ui.magnifier(vis, screen.mouse)
        lines = [("○ 회색 고리 = 지금 위치    Esc: 취소 (그대로 두기)", "gray ring: current   Esc: cancel", WHITE)]
        if cv2.getTickCount() < warn_until:
            lines.insert(0, ("⚠ 이 점은 depth가 없습니다. 다시 클릭하세요", "! no depth here, click again", YELLOW))
        screen.show(vis, lines, top=(f"▶ {POINT_NAMES[i]}만 다시 클릭하세요", f"re-click point {i}", YELLOW))
        k = screen.key(15)
        if k == 27:
            log("[다시 클릭] 취소: 그대로 둡니다")
            return None


def click_loop(screen, rgb, depth, cam, thr, rgb_size, mode, back_allowed=True):
    """클릭 점(5점 또는 꼭짓점 4점)을 돌려준다. None = 종료, "back" = 배치 점검으로 돌아가기."""
    clicks, warn, warn_until = [], None, 0
    while len(clicks) < N_POINTS:
        c = screen.take_click()
        if c is not None:
            if depth_missing(c, depth, cam, thr, rgb_size, mode):
                log("⚠ 이 점은 depth가 없습니다. 다시 클릭하세요")
                warn, warn_until = ("⚠ 이 점은 depth가 없습니다. 다시 클릭하세요", "! no depth here, click again"), \
                    cv2.getTickCount() + 2 * cv2.getTickFrequency()
                continue
            clicks.append(c)
            log(f"[클릭] {CLICK_GUIDE[len(clicks) - 1][2]:<7} RGB 픽셀 ({c[0]}, {c[1]})")
            if len(clicks) == N_POINTS:
                break
        vis = rgb.copy()
        draw_clicks(vis, clicks)
        _ui.magnifier(vis, screen.mouse)
        ko, en, _ = CLICK_GUIDE[len(clicks)]
        keys = "u: 되돌리기   r: 처음부터   q: 종료" + ("   b: 배치 점검으로 (새 정지 화면)" if back_allowed else "")
        lines = [(keys, "u: undo   r: restart   q: quit" + ("   b: back to live" if back_allowed else ""), WHITE)]
        if warn and cv2.getTickCount() < warn_until:
            lines.insert(0, (warn[0], warn[1], YELLOW))
        screen.show(vis, lines, top=(f"{ko}  ({len(clicks) + 1}/{N_POINTS})", f"{en}  ({len(clicks) + 1}/{N_POINTS})",
                                     YELLOW))
        k = screen.key(15)
        if k == ord("u") and clicks:
            p = clicks.pop()
            log(f"[되돌리기] {CLICK_GUIDE[len(clicks)][2]} ({p[0]}, {p[1]}) 지움")
        elif k == ord("r"):
            clicks = []
            log("[처음부터] 클릭을 모두 지웠습니다")
        elif k == ord("b") and back_allowed:
            return "back"
        elif k == ord("q"):
            return None
    return clicks


def board_problems(det, min_px):
    """보정판 마커를 다 못 찾은 이유 [(한글, 영어)]."""
    out = []
    if det.missing:
        out.append((f"안 보이는 마커: ID {', '.join(map(str, det.missing))} (가려졌거나 화면 밖, 빛 반사)",
                    f"not found: id {det.missing}"))
    if det.duplicated:
        out.append((f"같은 번호가 두 개 보임: ID {', '.join(map(str, det.duplicated))} (보정판 밖의 같은 번호 마커를 치우세요)",
                    f"seen twice: id {det.duplicated}"))
    if det.small:
        out.append((f"너무 작게 보임 (한 변 {min_px:.0f} px 미만): ID {', '.join(map(str, det.small))} "
                    "(카메라를 가깝게 하거나 덜 기울이세요)", f"too small: id {det.small}"))
    return out


def aruco_loop(screen, rgb, depth, cam, thr, rgb_size, mode, back_allowed=True):
    """보정판 마커 중심(ID 순서)을 정지 화면에서 찾는다. 반환 형식은 click_loop와 같다.
    다 못 찾으면 이유를 보여 주고 b(새 정지 화면) / c(손으로 클릭) / q(종료)를 기다린다."""
    min_px = float(ARUCO.get("min_side_px", 30))
    det = find_board(rgb, ARUCO["ids"], ARUCO.get("dictionary", "DICT_4X4_50"), min_px)
    if det.points is not None:
        for name, k, p in zip(POINT_NAMES, ARUCO["ids"], det.points):
            log(f"[검출] {name:<10} ID {k}  RGB 픽셀 ({p[0]:.1f}, {p[1]:.1f})  화면 한 변 {shortest_side_px(det.quads[k]):.0f} px")
        return [(float(p[0]), float(p[1])) for p in det.points]
    problems = board_problems(det, min_px)
    for ko, _en in problems:
        log(f"⚠ 보정판: {ko}")
    vis = rgb.copy()
    for k, q in det.quads.items():
        color = YELLOW if k in det.small else GREEN
        cv2.polylines(vis, [np.round(q).astype(np.int32)], True, color, 2, cv2.LINE_AA)
        c = square_center(q)
        cv2.putText(vis, str(k), (int(c[0]) - 5, int(c[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    keys = "c: 손으로 클릭   q: 종료" + ("   b: 배치 점검으로 (새 정지 화면)" if back_allowed else "")
    lines = [(ko, en, YELLOW) for ko, en in problems]
    lines += [("▶ 보정판 전체가 화면에 보이는지, 손·물건이 가리거나 빛이 반사되지 않는지 확인하세요", "> check the board", WHITE),
              (keys, "c: click by hand   q: quit" + ("   b: back to live" if back_allowed else ""), WHITE)]
    while True:
        screen.show(vis, lines, top=("⚠ 보정판 마커를 다 찾지 못했습니다 (초록 = 찾은 마커)", "board markers missing", YELLOW))
        k = screen.key(30)
        if k == ord("b") and back_allowed:
            return "back"
        if k == ord("c"):
            log("[클릭] 마커 중심을 손으로 찍습니다")
            return click_loop(screen, rgb, depth, cam, thr, rgb_size, mode, back_allowed)
        if k == ord("q"):
            return None


# ---------------------------------------------------------------------------
# 장면 1-4 결과 화면
# ---------------------------------------------------------------------------
def result_lines(res):
    """[(한글, 영어, 색)] — 화면과 터미널에 같은 내용. 검사마다: 무엇을 쟀는지 · 값 · 기준 · 통과 여부, 안 되면 ▶ 고치는 법."""
    c_base, c_sides, c_refl = res.check("base_error"), res.check("a4_sides"), res.check("reflection")
    c_size, c_desk = res.check("a4_size"), res.check("desk_z")
    verdict = "✓ 모든 검사 통과 — s로 저장하세요" if res.passed else "× 통과 못 한 검사가 있습니다 — 아래 ▶ 안내를 보세요"
    L = [("【보정 결과】 " + verdict + ("" if res.mode == "plane" else "   (depth 모드: point)"),
          "Calibration result - " + ("all checks passed" if res.passed else "some checks failed"),
          GREEN if res.passed else RED),
         ("ⓘ ● " + ("찾은 마커 중심" if ARUCO else "클릭한 점") + "  ○ 계산 결과로 다시 그린 점 (둘이 겹치면 좋음)   축: " +
          ("빨강 X(앞) 초록 Y(왼쪽) 파랑 Z(위)" if N_POINTS == 5 else "빨강 X(로봇의 오른쪽) 초록 Y(로봇의 앞) 파랑 Z(위)"),
          "dot: clicked  ring: reprojected  red X(fwd) green Y(left) blue Z(up)", GRAY)]
    # 1. 원점 (원점을 찍지 않는 배치: 꼭짓점 하나를 빼고 맞춘 오차)
    if res.holdout_index is None:
        L.append((f"1. 원점 위치 오차  {c_base.value * 100:.1f} cm   {mark(c_base.ok, '실패')}  "
                  f"(기준 {c_base.limit * 100:.1f} cm 이하 · 꼭짓점 4개로 계산한 원점과 클릭한 원점의 거리)",
                  f"base holdout {c_base.value * 100:.1f} cm (limit {c_base.limit * 100:.1f})", GREEN if c_base.ok else RED))
        if not c_base.ok:
            L.append(("   ▶ 원점 클릭이 대각선 교차점에서 벗어났거나 종이가 밀림 → 0 키: 원점만 다시 클릭 (종이를 옮겼으면 r 다음 b)",
                      "  > key 0: re-click the base point (paper moved: r, then b)", RED))
    else:
        worst_c = POINT_LABELS[res.holdout_index]
        L.append((f"1. {KIND} 맞춤 일관성  {c_base.value * 100:.1f} cm   {mark(c_base.ok, '실패')}  "
                  f"(기준 {c_base.limit * 100:.1f} cm 이하 · {KIND} 하나를 빼고 나머지 3개로 맞췄을 때 뺀 {KIND}의 오차 중 최대, "
                  f"{KIND} {worst_c})",
                  f"corner leave-one-out {c_base.value * 100:.1f} cm (limit {c_base.limit * 100:.1f})",
                  GREEN if c_base.ok else RED))
        if not c_base.ok and ARUCO:
            L.append((f"   ▶ 마커 {worst_c}이(가) 다른 셋과 안 맞음 → 보정판이 평평한지, 화살표가 로봇 쪽인지 확인 "
                      f"(r: 새 정지 화면에서 다시 찾기, {worst_c} 키: 그 마커 중심을 손으로 클릭)",
                      f"  > marker {worst_c} off: flatten the board, r: detect again", RED))
        elif not c_base.ok:
            L.append((f"   ▶ 꼭짓점 {worst_c}이(가) 다른 셋과 안 맞음 → {worst_c} 키: 그 꼭짓점만 모서리에 정확히 다시 클릭 "
                      "(번호 위치가 좌상·우상·좌하·우하인지 확인)",
                      f"  > key {worst_c}: re-click that corner", RED))
    # 2. A4 변
    worst = int(np.argmax(np.abs(res.sides_m - res.sides_expected_m) / res.sides_tol_m))
    sides = "  ".join(f"{n}: {m * 100:.1f}" for n, m in zip(SIDE_NAMES, res.sides_m))
    exp = "/".join(f"{e * 100:.1f}" for e in res.sides_expected_m[:2])
    L.append((f"2. {SHAPE} 네 변 길이  {sides} cm   {mark(c_sides.ok, '실패')}  (실제 {exp} cm, "
              f"허용 ±{min(res.sides_tol_m) * 100:.1f}~{max(res.sides_tol_m) * 100:.1f} cm)",
              f"sides cm {sides}", GREEN if c_sides.ok else RED))
    if not c_sides.ok and ARUCO:
        d = (res.sides_m[worst] - res.sides_expected_m[worst]) * 100
        L.append((f"   ▶ 변 {SIDE_NAMES[worst]}의 길이가 실제보다 {abs(d):.1f} cm {'길게' if d > 0 else '짧게'} 계산됨 → "
                  f"보정판이 평평한지, 실제 크기(100 %)로 인쇄됐는지(마커 한 변 {float(ARUCO['size_m']) * 100:.1f} cm) 확인",
                  f"  > side {SIDE_NAMES[worst]} off by {d:+.1f} cm: check the board print and flatness", RED))
    elif not c_sides.ok:
        d = (res.sides_m[worst] - res.sides_expected_m[worst]) * 100
        L.append((f"   ▶ 변 {SIDE_NAMES[worst]}의 길이가 실제보다 {abs(d):.1f} cm {'길게' if d > 0 else '짧게'} 계산됨 → "
                  f"{SIDE_NAMES[worst].replace('-', ' 또는 ')} 키: 그 꼭짓점만 모서리에 정확히 다시 클릭",
                  f"  > side {SIDE_NAMES[worst]} off by {d:+.1f} cm: press {SIDE_NAMES[worst].replace('-', ' or ')}", RED))
        L.append(("   ▶ 다시 눌러도 1 cm 안팎이 남으면 알려진 한계(컬러·깊이 정합 오차)입니다. s 두 번으로 저장은 되지만 "
                  "시뮬레이터는 통과 못 한 보정을 받지 않습니다",
                  "  > ~1 cm left after re-clicking = known limit; s twice saves, the simulator refuses it", RED))
    # 3. 화면 크기
    L.append((f"3. 화면에 보이는 {SHAPE} 크기  가장 짧은 변 {res.a4_short_px:.0f} px   {mark(c_size.ok)}  "
              f"(기준 {c_size.limit:.0f} px 이상 · 1 px ≈ {res.a4_short_m_per_px * 1000:.1f} mm)",
              f"shortest side {res.a4_short_px:.0f} px", GREEN if c_size.ok else YELLOW))
    if not c_size.ok:
        L.append((f"   ▶ {SHAPE}이(가) 작게 보여 1 px 오차가 크게 번집니다. 카메라를 가깝게 하거나 덜 기울이세요",
                  "  > A4 small on screen: move the camera closer", YELLOW))
    # 4. 책상면 높이
    if res.desk_z:
        def zs(k):
            d = res.desk_z.get(k)
            return "-" if d is None else f"{d['median'] * 100:+.1f} cm"
        L.append((f"4. 책상면 높이 (0이어야 함)  가까운 쪽 {zs('near')} / 먼 쪽 {zs('far')}   {mark(c_desk.ok)}  "
                  f"(먼 쪽 점의 95%가 +{c_desk.limit * 100:.1f} cm보다 낮아야 함)",
                  f"desk z near {zs('near')} / far {zs('far')}", GREEN if c_desk.ok else YELLOW))
        if not c_desk.ok:
            L.append(("   ▶ 계산된 책상면이 기울어 있습니다 (먼 쪽일수록 뜸). 종이가 평평한지 보고, 어긋난 점은 번호 키로 다시 클릭",
                      "  > desk plane tilted: re-click or flatten the paper", YELLOW))
    # 5. 뒤집힘
    L.append((f"5. 좌표 뒤집힘 검사   {mark(c_refl.ok, '실패')}  (좌우가 거울처럼 뒤집혀 계산되지 않았는지)",
              f"reflection check {'OK' if c_refl.ok else 'FAIL'}", GREEN if c_refl.ok else RED))
    if not c_refl.ok and ARUCO:
        L.append(("   ▶ 보정판을 화살표가 로봇 쪽으로 오게 놓았는지(돌리거나 뒤집지 않았는지), depth 좌우반전 설정을 확인하세요",
                  "  > check the board orientation or depth flip", RED))
    elif not c_refl.ok:
        L.append(("   ▶ 꼭짓점을 1 → 2 → 3 → 4 순서(위에서 볼 때 반시계)로 눌렀는지, depth 좌우반전 설정을 확인하세요",
                  "  > check corner order or depth flip", RED))
    # 참고
    info = f"ⓘ 참고: 책상이 카메라 정면에서 약 {res.tilt_deg:.0f}° 기울어 보임"
    if np.isfinite(res.plane_rms_m):
        info += f" · 책상 평면 맞춤 오차 {res.plane_rms_m * 1000:.1f} mm · 그 영역 깊이 측정 {res.roi_valid_ratio * 100:.0f} %"
    L.append((info, f"desk tilt ~{res.tilt_deg:.0f} deg (info)", GRAY))
    keys = "0~4: 그 번호 점만 다시 클릭 (0 = 원점)" if N_POINTS == 5 else \
        ("1~4: 그 마커 중심을 손으로 클릭" if ARUCO else "1~4: 그 번호 꼭짓점만 다시 클릭")
    again = "r: 새 정지 화면에서 다시 찾기" if ARUCO else "r: 처음부터 다시"
    L.append((f"▶ s: 저장    {keys}    {again}    q: 저장 없이 종료",
              f"s: save   {KEY0}-{KEY0 + N_POINTS - 1}: re-click that point   r: restart   q: quit", WHITE))
    return L


def log_result(res):
    for ko, _en, _c in result_lines(res)[:-1]:
        log("  " + ko)
    if not res.handedness:
        log("[반사] 법선 손잡이 검사 실패: 꼭짓점이 위에서 볼 때 반시계 순서가 아니게 계산됨 (번호 순서 또는 depth 좌우반전 확인)")
    if not res.camera_above:
        log(f"[반사] 카메라 z 검사 실패: 카메라 위치 base z = {res.t[2]:+.3f} m (책상 아래로 계산됨)")
    log(f"  ({N_POINTS}점 RMS {np.sqrt(np.mean(res.residuals_m ** 2)) * 100:.2f} cm, 카메라 위치 base "
        f"({res.t[0]:+.3f}, {res.t[1]:+.3f}, {res.t[2]:+.3f}) m)")


def draw_result(rgb, res, cam, rgb_size, axis_len):
    vis = rgb.copy()
    conv = Cam2Base(res.R, res.t, cam)
    u0, v0, u1, v1 = res.roi
    us, vs = cam.depth_to_rgb(np.array([u0, u1 - 1]), np.array([v0, v1 - 1]), rgb_size, z_m=cam.z_ref_m)  # 표시용 근사
    _ui.dashed_rect(vis, (us[0], vs[0]), (us[1], vs[1]), CYAN)
    poly = conv.base_to_pixel(res.base_points[list(res.polygon_index)], rgb_size=rgb_size)
    if np.all(np.isfinite(poly)):
        cv2.polylines(vis, [np.round(poly).astype(np.int32)], True, (0, 165, 255), 2, cv2.LINE_AA)
    proj = conv.base_to_pixel(res.base_points, rgb_size=rgb_size)
    draw_clicks(vis, res.clicks_rgb)
    for p in proj:
        if _ui.finite(p):
            cv2.circle(vis, _ui.ipt(p), 8, WHITE, 1, cv2.LINE_AA)
    _ui.base_axes(vis, conv, axis_len, rgb_size)
    return vis


def result_loop(screen, rgb, res, cam, rgb_size, axis_len):
    """s / 0~4 / r / q → "save" / ("redo", 번호) / "reclick" / "quit"."""
    log_result(res)
    vis = draw_result(rgb, res, cam, rgb_size, axis_len)
    lines = result_lines(res)
    confirm = False
    while True:
        screen.show(vis, lines)
        k = screen.key(30)
        if k == ord("s"):
            if not res.passed and not confirm:
                log("⚠ 검사를 통과하지 못했습니다. 그래도 저장하려면 s를 한 번 더 누르세요")
                confirm = True
                continue
            return "save"
        if ord("0") + KEY0 <= k < ord("0") + KEY0 + N_POINTS:
            return ("redo", k - ord("0") - KEY0)
        if k == ord("r"):
            return "reclick"
        if k == ord("q"):
            return "quit"


def save_all(cfg_dir, res, rgb, depth, rgb_size, configs, layout):
    now = datetime.now()
    created = now.isoformat(timespec="seconds")
    sess_dir = cfg_dir.parent / "data" / "sessions"
    session = sess_dir / f"{now:%Y-%m-%d_%H%M}.npz"
    if session.exists():
        session = sess_dir / f"{now:%Y-%m-%d_%H%M%S}.npz"
    save_session(session, rgb, depth, res.clicks_rgb, rgb_size, res.mode, configs, res.summary(), created)
    rel = session.relative_to(cfg_dir.parent) if session.is_relative_to(cfg_dir.parent) else session
    ident = None if _ui.opened_rgb_device is None else rgb_identity(_ui.opened_rgb_device)
    write_cam2base(cfg_dir / "cam2base.yaml", res, configs["camera"], session_file=str(rel), created=created,
                   camera_identity=ident)
    log(f"[저장] {cfg_dir.name}/cam2base.yaml  (R, t, 날짜, depth 모드, 검사 결과, 카메라 신원)")
    log(f"[저장] {rel}  (RGB, depth, 클릭 {N_POINTS}점)")
    rc = int(layout["recheck_corner"])
    p = layout["corners"][rc - 1]
    if layout.get("base_clicked", True):
        log(f"[안내] 재점검용 표시: 꼭짓점 {rc}번 자리에 테이프를 붙이세요. base 좌표 ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m")
        log("[안내] 이제 로봇(앞면을 X 화살표에 맞춤)과 물체를 올려도 됩니다. 카메라는 움직이지 마세요.")
    else:
        log(f"[안내] 재점검(--check) 때 클릭할 점: {POINT_NAMES[rc - 1]} 자리, base 좌표 ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m. "
            "종이를 치울 거면 그 자리에 테이프를 붙이세요")
        log("[안내] 로봇 베이스는 배치의 원점에 그대로 두고, 카메라는 움직이지 마세요.")


# 기준점 찾기 방식: 정지 화면 → 배치 순서(base, 꼭짓점 1-4)의 화면 픽셀 배열 / None(종료) / "back".
# 다른 방식은 같은 반환 형식의 함수를 여기에 더한다 (solve_calibration의 입력·출력은 그대로).
POINT_FINDERS = {"click": click_loop, "aruco": aruco_loop}


def default_points(layout) -> str:
    """배치에 aruco 설정이 있으면 보정판 자동 찾기, 없으면 클릭."""
    return "aruco" if layout.get("aruco") else "click"


def find_reference_points(method, screen, rgb, depth, cam, thr, rgb_size, mode, back_allowed=True):
    return POINT_FINDERS[method](screen, rgb, depth, cam, thr, rgb_size, mode, back_allowed)


def calibrate_frame(screen, rgb, depth, cam, cfg, rgb_size, mode, back_allowed, axis_len, points="click",
                    clicks=None, redo=None):
    """기준점 찾기 → 계산 → 결과 화면. 반환: ("save", res) / ("quit", None) / ("back", None).
    clicks + redo: 이미 찍은 5점에서 redo번만 다시 찍고 시작 (재현 모드의 결과 화면에서 번호 키)."""
    thr = cfg["thresholds"]
    while True:
        if clicks is None:
            clicks = find_reference_points(points, screen, rgb, depth, cam, thr, rgb_size, mode, back_allowed)
            if clicks is None:
                return "quit", None
            if clicks == "back":
                return "back", None
        if redo is not None:
            clicks = reclick_one(screen, rgb, depth, cam, thr, rgb_size, mode, clicks, redo) or clicks
            redo = None
        try:
            res = solve_calibration(depth, clicks, cam, cfg["a4_layout"], thr, mode=mode, rgb_size=rgb_size)
        except CalibrationError as e:
            names = ", ".join(POINT_NAMES[i] for i in e.indices)
            log(f"⚠ 계산 실패: {e}" + (f" ({names})" if names else ""))
            if len(e.indices) == 1:                     # 문제인 점이 하나면 그 점만 다시
                redo = e.indices[0]
            elif points == "aruco" and back_allowed:    # 같은 화면을 다시 찾으면 같은 결과: 새 정지 화면으로
                log("  → 새 정지 화면에서 다시 찾습니다 (보정판 전체가 평평하게 보이는지 확인)")
                return "back", None
            else:
                log("  → 처음부터 다시 클릭하세요")
                clicks, points = None, "click"
            continue
        act = result_loop(screen, rgb, res, cam, rgb_size, axis_len)
        if act == "save":
            return "save", res
        if act == "quit":
            return "quit", None
        if act == "reclick" and points == "aruco" and back_allowed:
            return "back", None                         # 보정판: 같은 화면을 다시 찾지 않고 새 정지 화면으로
        if act == "reclick":
            clicks = None
        else:                                           # ("redo", 번호): 그 점만 다시 찍고 다시 계산
            redo = act[1]


# ---------------------------------------------------------------------------
# 실행 모드
# ---------------------------------------------------------------------------
def run_calibrate(args, cfg, cfg_dir, text):
    cam = CameraModel.from_config(cfg["camera"])
    cap, rgb_size = _ui.open_rgb(cfg["camera"])
    dev = AstraDepth.from_config(cfg["camera"]).open()
    d0 = dev.read()
    log(f"[pac_vision] RGB 열림 {rgb_size[0]}x{rgb_size[1]} / depth(OpenNI) 열림 {d0.shape[1]}x{d0.shape[0]}")
    screen = _ui.Screen(WIN, text, cfg["camera"]["ui"]["font_size"])
    try:
        while True:
            frozen = placement_loop(screen, cap, dev, cam, cfg["thresholds"], rgb_size)
            if frozen is None:
                log("[종료] 저장하지 않았습니다")
                return
            rgb, depth = frozen
            act, res = calibrate_frame(screen, rgb, depth, cam, cfg, rgb_size, args.depth_mode or "plane", True,
                                       cfg["camera"]["ui"]["axis_len_m"], args.points or default_points(cfg["a4_layout"]))
            if act == "back":
                continue
            if act == "save":
                configs = {k: cfg[k] for k in ("camera", "a4_layout", "thresholds")}
                save_all(cfg_dir, res, rgb, depth, rgb_size, configs, cfg["a4_layout"])
            else:
                log("[종료] 저장하지 않았습니다")
            return
    finally:
        cap.release()
        dev.close()
        cv2.destroyAllWindows()


def measure_desk_point(conv, depth, click, rgb_size, thr):
    """클릭 주변 영역에서 책상 평면을 맞추고 클릭 광선과의 교점 (base 좌표). 실패하면 None.

    거리 식이면 영역은 보정 결과의 책상면 Z로 옮긴 자리에 잡고, 교점은 맞춘 평면의 Z로 고정점 반복한다."""
    pc = thr["plane"]
    m = pc["roi_margin_px"]
    cam = conv.cam
    _, ud, vd = conv.pixel_to_desk(click[0], click[1], rgb_size=rgb_size)
    ud, vd = float(ud[0]), float(vd[0])
    if not (0 <= ud < cam.width and 0 <= vd < cam.height):
        return None
    roi = clip_roi(ud - m, vd - m, ud + m, vd + m, cam.width, cam.height)
    pts, _, _, _ = roi_points(depth, cam, roi)
    # 보정 책상면 ±check.desk_band_m 안의 점만 (책상 끝에서 책상 너머 바닥·벽을 평면으로 잡지 않게)
    plane = fit_desk_plane_near(pts, conv.R, conv.t, thr["check"]["desk_band_m"], pc["ransac_iters"], pc["inlier_m"],
                                pc["max_points"], pc["seed"])
    if plane is None or plane.n_points < 50:
        return None
    p = conv.pixel_to_desk(click[0], click[1], rgb_size=rgb_size, plane=plane)[0][0]
    return p if np.all(np.isfinite(p)) else None


def warn_calibration_coefficients(conv, cfg):
    """보정 때 정합 계수와 지금 설정이 다르면 경고 (정합을 다시 쟀으면 보정도 다시 계산해야 함)."""
    if not calibration_matches_current(conv, cfg["camera"]):
        log("[pac_vision] ⚠ 보정(cam2base.yaml) 때와 지금 설정의 RGB→depth 정합 계수가 다릅니다. 보정은 그때 계수로 계산합니다. "
            "정합을 다시 쟀다면 python scripts/calibrate.py --replay <세션> --use-current-config로 보정을 갱신하세요")


def run_check(args, cfg, cfg_dir, text):
    try:
        conv = require_cam2base(cfg_dir / "cam2base.yaml")
    except MissingCalibration as e:
        sys.exit(f"[pac_vision] {e}")
    info, thr = conv.info, cfg["thresholds"]
    warn_calibration_coefficients(conv, cfg)
    log(f"[pac_vision] 보정 읽음: {cfg_dir.name}/cam2base.yaml ({info.get('created')}, "
        f"base 점 오차 {info.get('holdout_m', float('nan')) * 100:.1f} cm)")
    rc = int(cfg["a4_layout"]["recheck_corner"])
    expected = np.array(cfg["a4_layout"]["corners"][rc - 1], float)
    cap, rgb_size = _ui.open_rgb(cfg["camera"])          # 장치 번호·경로는 지금 컴퓨터의 camera.yaml
    dev = AstraDepth.from_config(cfg["camera"]).open()
    screen = _ui.Screen(WIN, text, cfg["camera"]["ui"]["font_size"])
    prompt = f"재점검: 테이프 표시(꼭짓점 {rc}번 자리)를 클릭하세요"
    log(prompt)
    axis_len = cfg["camera"]["ui"]["axis_len_m"]
    corners = np.array(cfg["a4_layout"]["corners"], float)
    first, points = True, []
    try:
        while True:
            ok, rgb = cap.read()
            if not ok:
                continue
            c = screen.take_click()
            if c is not None:
                depth, _ = dev.read_average(int(thr["capture"]["average_frames"]), thr["capture"]["min_valid_fraction"])
                p = measure_desk_point(conv, depth, c, rgb_size, thr)
                if p is None:
                    log("⚠ 이 점 주변에 depth가 부족합니다. 다시 클릭하세요")
                elif first:
                    err = float(np.linalg.norm(p - expected))
                    ok_err = err <= thr["check"]["error_max_m"]
                    log(f"→ 측정 ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m / 기대 ({expected[0]:.3f}, {expected[1]:.3f}, "
                        f"{expected[2]:.3f}) m / 오차 {err * 100:.1f} cm {mark(ok_err)} "
                        + ("보정 유지" if ok_err else "→ 카메라가 움직였을 수 있습니다. 장면 1을 다시 하세요"))
                    log("[안내] 이어서 다른 점을 클릭하면 그 점의 base 좌표(책상면 평면 기준)를 출력합니다. q: 종료")
                    log("[안내] ⚠ 이 비교는 RGB 클릭 경로라 dx·dy 오프셋은 보이지 않습니다 (A4 밖 외삽 오차만). "
                        "오프셋은 detect_objects.py 물체 좌표를 줄자와 비교해 확인하세요.")
                    first = False
                    points.append(c)
                else:
                    log(f"[점] RGB ({c[0]}, {c[1]}) → base ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m  (줄자와 비교: 외삽 오차, 오프셋 제외)")
                    points.append(c)
            vis = rgb.copy()
            poly = conv.base_to_pixel(corners, rgb_size=rgb_size)
            if np.all(np.isfinite(poly)):
                cv2.polylines(vis, [np.round(poly).astype(np.int32)], True, (0, 165, 255), 1, cv2.LINE_AA)
            _ui.base_axes(vis, conv, axis_len, rgb_size)
            for q in points:
                cv2.circle(vis, q, 4, YELLOW, -1)
            _ui.magnifier(vis, screen.mouse)
            top = (prompt, f"re-check: click the tape mark (corner {rc})", YELLOW) if first else \
                ("다른 점 클릭: base 좌표 출력", "click points: print base coords", YELLOW)
            screen.show(vis, [("주황 선: 보정으로 다시 그린 A4 외곽선 (종이 자리와 겹치면 카메라가 그대로)   q: 종료",
                               "orange: A4 outline from calibration   q: quit", WHITE)], top=top)
            if screen.key(15) == ord("q"):
                return
    finally:
        cap.release()
        dev.close()
        cv2.destroyAllWindows()


def run_replay(args, cfg, cfg_dir, text):
    path = Path(args.replay)
    current = cfg["camera"] if args.use_current_config else None
    res, s = replay_session(path, mode=args.depth_mode, camera_cfg=current)
    use_layout(s["configs"]["a4_layout"])                 # 세션의 배치 (꼭짓점만 찍은 세션이면 4점)
    used_cam = current if current is not None else s["configs"]["camera"]
    log(f"[재현] {path} (저장 {s['created']}, depth 모드 {res.mode}"
        + (", 현재 camera.yaml 사용" if current is not None else ", 세션의 설정 사본 사용") + ")")
    if current is not None:
        a = CameraModel.from_config(s["configs"]["camera"]).rgb_depth_text()
        b = CameraModel.from_config(current).rgb_depth_text()
        log(f"[재현] rgb_to_depth 세션 {a}\n       → 현재 {b}")
    diff = compare_summaries(res.summary(), s["summary"])
    same_inputs = current is None and (args.depth_mode is None or args.depth_mode == s["mode"])
    if args.no_gui:
        log_result(res)
    if same_inputs:
        log(f"[재현] 저장 때 결과와 {'같음 ✓' if diff < 1e-6 else '다름 ⚠'} (R·t·점 좌표 최대 차이 {diff:.1e})")
    else:
        log(f"[재현] 저장 때 결과와의 최대 차이 {diff:.1e} (설정을 바꿔 계산했으므로 달라질 수 있음)")
    if args.no_gui:
        return 0 if (not same_inputs or diff < 1e-6) else 1

    cam = CameraModel.from_config(used_cam)
    configs = dict(s["configs"], camera=used_cam)
    screen = _ui.Screen(WIN, text, cfg["camera"]["ui"]["font_size"])
    try:
        act = result_loop(screen, s["rgb"], res, cam, s["rgb_size"], cfg["camera"]["ui"]["axis_len_m"])
        if act == "reclick" or isinstance(act, tuple):
            redo = act[1] if isinstance(act, tuple) else None
            act, res = calibrate_frame(screen, s["rgb"], s["depth"], cam, configs, s["rgb_size"], res.mode, False,
                                       cfg["camera"]["ui"]["axis_len_m"],
                                       args.points or default_points(configs["a4_layout"]),
                                       clicks=[tuple(c) for c in res.clicks_rgb] if redo is not None else None, redo=redo)
        if act == "save":
            save_all(cfg_dir, res, s["rgb"], s["depth"], s["rgb_size"], configs, configs["a4_layout"])
        else:
            log("[종료] 저장하지 않았습니다")
    finally:
        cv2.destroyAllWindows()
    return 0


def main():
    ap = argparse.ArgumentParser(description="pac_vision 캘리브레이션 (카메라 → base_ref)")
    ap.add_argument("--config-dir", default=str(_ui.DEFAULT_CONFIG_DIR))
    ap.add_argument("--depth-mode", choices=["plane", "point"], default=None,
                    help="plane: 책상 평면 맞춤 + 광선 교차 (기본), point: 클릭 점 주변 depth")
    ap.add_argument("--check", action="store_true", help="재점검 (장면 3)")
    ap.add_argument("--replay", metavar="SESSION.npz", help="카메라 없이 세션으로 재계산 (장면 4)")
    ap.add_argument("--no-gui", action="store_true", help="--replay와 함께: 창 없이 숫자만 출력")
    ap.add_argument("--use-current-config", action="store_true",
                    help="--replay와 함께: 세션의 설정 사본 대신 현재 설정(camera.yaml + pose.yaml)으로 계산 (정합만 다시 쟀을 때)")
    ap.add_argument("--points", choices=sorted(POINT_FINDERS), default=None,
                    help="기준점 찾기 방식: click = 손 클릭, aruco = 보정판 마커 자동 (기본: 배치에 aruco 설정이 있으면 aruco)")
    ap.add_argument("--allow-default-pose", action="store_true",
                    help="시연용: config/pose.yaml(이 설치 자세의 정합 계수) 없이 camera.yaml 기본값으로 캘리브레이션")
    ap.add_argument("--layout", choices=sorted(LAYOUT_FILES), default="double",
                    help="기준 종이: double = A4 두 장 29.7x42.0 cm (a4_layout.yaml, 기본), "
                         "single = A4 한 장 21.0x29.7 cm (a4_layout_single.yaml), "
                         "aruco = ArUco 보정판 A4 한 장, 마커 4개 자동 (a4_layout_aruco.yaml)")
    args = ap.parse_args()
    if (args.no_gui or args.use_current_config) and not args.replay:
        ap.error("--no-gui, --use-current-config는 --replay와 함께 씁니다")

    cfg_dir = Path(args.config_dir).resolve()
    cfg = _ui.load_config_or_exit(cfg_dir, ("camera", "a4_layout", "thresholds"))
    layout_file = LAYOUT_FILES["double"]
    if args.layout != "double" and not args.replay:      # --replay는 세션에 저장된 배치를 그대로 쓴다
        from pac_vision.config import load_yaml
        layout_file = LAYOUT_FILES[args.layout]
        cfg["a4_layout"] = load_yaml(cfg_dir / layout_file)
    if args.points == "aruco" and not args.replay and not cfg["a4_layout"].get("aruco"):
        ap.error("--points aruco는 보정판 배치(--layout aruco)에서만 씁니다")
    use_layout(cfg["a4_layout"])
    log(f"[pac_vision] 설정 읽음: {cfg_dir.name}/camera.yaml, {cfg_dir.name}/{layout_file}, {cfg_dir.name}/thresholds.yaml")
    if not has_pose(cfg_dir):
        msg = (f"{cfg_dir.name}/pose.yaml(이 설치 자세의 RGB→depth 정합 계수)이 없습니다. "
               "check_camera.py로 쌍을 잰 뒤 python tools/fit_rgb_to_depth.py <쌍 CSV> --write-pose로 만드세요")
        if args.replay and not args.use_current_config:
            pass                                    # 세션의 설정 사본으로 계산하므로 상관없음
        elif args.check or args.replay or args.allow_default_pose:
            log(f"[pac_vision] ⚠ {msg} (camera.yaml 기본값 사용)")
        else:
            sys.exit(f"[pac_vision] {msg}\n  (시연용으로 기본값을 쓰려면 --allow-default-pose)")
    text = None if (args.replay and args.no_gui) else _ui.TextDrawer(cfg["camera"]["ui"]["font"])
    if args.replay:
        sys.exit(run_replay(args, cfg, cfg_dir, text))
    if args.check:
        run_check(args, cfg, cfg_dir, text)
    else:
        run_calibrate(args, cfg, cfg_dir, text)


if __name__ == "__main__":
    main()
