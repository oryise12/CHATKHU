"""카메라 확인: depth 읽기·좌우반전(현실 오른쪽 = X+)·Z, 같은 점 30프레임 노이즈, RGB↔depth 정합(dx, dy).

조작
- depth 창 클릭: 그 점을 30프레임 읽어 카메라 좌표(X, Y, Z)와 표준편차 출력
- RGB 창 클릭 → 바로 다음 depth 창 클릭: 한 쌍으로 dx, dy 기록 (다른 순서의 클릭은 쌍으로 인정하지 않음)
- s: 지금까지의 dx 분석 출력   u: 마지막 쌍 지우기   q: 분석 출력 후 종료

실행: python scripts/check_camera.py
"""
from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from pac_vision import AstraDepth
from pac_vision.cam2base import CameraModel, patch_median

import _ui
from _ui import GREEN, WHITE, YELLOW, log


def position_label(u, v, w, h):
    horiz = "왼쪽 끝" if u < w * 0.2 else "오른쪽 끝" if u > w * 0.8 else "가운데"
    vert = "위쪽 끝(먼 쪽)" if v < h * 0.2 else "아래쪽 끝(가까운 쪽)" if v > h * 0.8 else "가운데"
    return f"{vert}/{horiz}"


def measure_point(depth_dev, cam, ud, vd, n):
    """같은 depth 픽셀을 서로 다른 n프레임에서 읽는다: 픽셀 하나 값과 5x5 중앙값."""
    single, patch, last = [], [], None
    t0 = time.time()
    while len(patch) < n and time.time() - t0 < 10:
        d = depth_dev.read()
        idx = depth_dev.last_index
        if idx is not None and idx == last:
            continue
        last = idx
        single.append(float(d[int(vd), int(ud)]))
        patch.append(patch_median(d, ud, vd, 2))
    single, patch = np.array(single), np.array(patch)
    return single[single > 0], patch[patch > 0], len(patch)


def fit_line(x, y):
    """y = a + b x 최소제곱. 반환 (a, b, 잔차 표준편차)."""
    A = np.column_stack([np.ones_like(x), x])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ coef
    return float(coef[0]), float(coef[1]), float(np.std(res, ddof=2)) if len(x) > 2 else float("nan")


def analyse_pairs(pairs, cam):
    if not pairs:
        log("[dx 분석] 쌍이 없습니다")
        return
    p = np.array([[q["u_ref"], q["v_ref"], q["dx"], q["dy"], q["Z"]] for q in pairs], dtype=float)
    u, v, dx, dy, z = p.T
    log(f"\n[dx 분석] 쌍 {len(p)}개")
    log("  번호  위치                         RGB(u,v)      dx     dy     Z(m)")
    for i, q in enumerate(pairs):
        zs = f"{q['Z']:.3f}" if np.isfinite(q["Z"]) else "  -"
        log(f"  {i + 1:>3}  {q['where']:<26} ({q['u_ref']:5.0f},{q['v_ref']:4.0f})  {q['dx']:+5.0f}  {q['dy']:+5.0f}  {zs}")
    log(f"  dx 평균 {dx.mean():+.1f} px (표준편차 {dx.std():.1f}), dy 평균 {dy.mean():+.1f} px (표준편차 {dy.std():.1f})")
    W, H = cam.rgb_width, cam.rgb_height
    cover = {"왼쪽 끝": (u < W * 0.2).any(), "오른쪽 끝": (u > W * 0.8).any(),
             "위쪽 끝": (v < H * 0.2).any(), "아래쪽 끝": (v > H * 0.8).any()}
    missing = [k for k, ok in cover.items() if not ok]
    if missing:
        log(f"  ⚠️ 아직 없는 위치: {', '.join(missing)} → 판정 전에 이 위치도 재 주세요")
    if len(p) < 4:
        log("  (판정은 쌍 4개 이상부터)")
        return
    for name, x, y, span, axis, sname in (("dx ~ u (가로 위치)", u, dx, W, "sx", "폭"), ("dy ~ v (세로 위치)", v, dy, H, "sy", "높이")):
        a, b, s = fit_line(x, y)
        change = b * span
        trend = abs(change) > max(5.0, 3 * s) and np.ptp(x) > 0.5 * span
        verdict = (f"한 방향 변화 → 배율 차이 의심 (화면 {sname}에서 {change:+.1f} px, 제안 {axis} ≈ {1 + b:.4f})"
                   if trend else f"뚜렷한 한 방향 변화 없음 (화면 {sname}에서 {change:+.1f} px, 잔차 {s:.1f} px → 클릭·시차 수준)")
        log(f"  {name}: 기울기 {b:+.4f} px/px, 잔차 {s:.1f} px → {verdict}")
    ok = np.isfinite(z) & (z > 0)
    if ok.sum() >= 4 and np.ptp(1 / z[ok]) > 0:
        a, c, s = fit_line(1 / z[ok], dx[ok])
        log(f"  dx ~ 1/Z (거리, 시차): dx ≈ {a:+.1f} {c:+.1f}/Z px, 잔차 {s:.1f} px "
            f"(Z {z[ok].min():.2f}-{z[ok].max():.2f} m에서 {c / z[ok].min() - c / z[ok].max():+.1f} px 차이)")


def main():
    ap = argparse.ArgumentParser(description="카메라 확인 (depth, 좌우반전, 노이즈, RGB↔depth 정합)")
    ap.add_argument("--config-dir", default=str(_ui.DEFAULT_CONFIG_DIR))
    ap.add_argument("--frames", type=int, default=None, help="점 측정 프레임 수 (기본: thresholds capture.average_frames)")
    args = ap.parse_args()

    cfg = _ui.load_config_or_exit(args.config_dir, ("camera", "thresholds"))
    cam = CameraModel.from_config(cfg["camera"])
    n_frames = args.frames or int(cfg["thresholds"]["capture"]["average_frames"])
    text = _ui.TextDrawer(cfg["camera"]["ui"]["font"])
    log(f"[pac_vision] 설정 읽음: {args.config_dir}/camera.yaml ({cam.rgb_depth_text()})")

    cap, rgb_size = _ui.open_rgb(cfg["camera"])
    depth_dev = AstraDepth.from_config(cfg["camera"]).open()
    d0 = depth_dev.read()
    log(f"[pac_vision] RGB 열림 {rgb_size[0]}x{rgb_size[1]} / depth(OpenNI) 열림 {d0.shape[1]}x{d0.shape[0]}")
    log("[안내] Z 비교용 줄자는 '카메라 하우징 앞면 → 물체 앞면'으로, 카메라가 향한 방향을 따라 잽니다.")
    log("[안내] 좌우반전 확인: 카메라 뒤에서 카메라와 같은 방향을 볼 때 오른쪽에 물체를 두고 depth 창에서 클릭 → X가 양수여야 합니다.")
    log("[안내] dx: RGB 창에서 모서리 클릭 → 바로 depth 창에서 같은 모서리 클릭. 좌·우 끝, 위·아래 끝을 꼭 포함하세요.\n")

    state = {"pending": None, "pairs": [], "depth_click": None, "rgb_mouse": None, "depth_mouse": None}
    measurements = []

    def on_rgb(event, x, y, flags, param):
        state["rgb_mouse"] = (x, y)
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if state["pending"] is not None:
            log("[RGB] ⚠️ 이전 RGB 클릭은 쌍이 되지 못해 버립니다")
        state["pending"] = (x, y)
        log(f"[RGB] ({x}, {y}) → 이제 depth 창에서 같은 지점을 클릭하세요")

    def on_depth(event, x, y, flags, param):
        state["depth_mouse"] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            state["depth_click"] = (x, y)

    cv2.namedWindow("RGB")
    cv2.namedWindow("depth")
    cv2.setMouseCallback("RGB", on_rgb)
    cv2.setMouseCallback("depth", on_depth)

    try:
        while True:
            ok, rgb = cap.read()
            depth = depth_dev.read()
            if not ok:
                continue
            if state["depth_click"] is not None:
                ud, vd = state["depth_click"]
                state["depth_click"] = None
                single, patch, n = measure_point(depth_dev, cam, ud, vd, n_frames)
                where = position_label(ud, vd, cam.width, cam.height)
                if patch.size == 0:
                    log(f"[depth] ({ud}, {vd}) {where}: ⚠️ depth 없음 (0.6 m 이내·검정·광택·가장자리?)")
                    Z = float("nan")
                else:
                    z_mm = float(np.median(patch))
                    P = cam.backproject(ud, vd, z_mm)
                    Z = float(P[2])
                    sd_single = float(single.std()) if single.size >= 2 else float("nan")
                    single_txt = (f"{sd_single:.1f} mm (유효 {single.size}/{n})" if single.size >= 2
                                  else f"⚠️ 유효 {single.size}/{n} (이 픽셀은 거의 항상 depth 없음 → 5x5 중앙값만 의미 있음)")
                    log(f"[depth] ({ud}, {vd}) {where}: 카메라 X={P[0]:+.3f} Y={P[1]:+.3f} Z={P[2]:.3f} m  "
                        f"| σ 픽셀 하나 {single_txt}, 5x5 중앙값 {patch.std():.1f} mm (유효 {patch.size}/{n})")
                    measurements.append(dict(u=ud, v=vd, where=where, X=P[0], Y=P[1], Z=P[2],
                                             sd_single=sd_single, n_single=single.size,
                                             sd_patch=float(patch.std()), n_patch=patch.size, n=n,
                                             kind="pair_edge" if state["pending"] is not None else "surface"))
                if state["pending"] is not None:
                    ur, vr = cam.screen_to_ref(*state["pending"], rgb_size)
                    q = dict(u_ref=float(ur), v_ref=float(vr), u_d=ud, v_d=vd, dx=ud - float(ur), dy=vd - float(vr),
                             Z=Z, where=position_label(float(ur), float(vr), cam.rgb_width, cam.rgb_height))
                    state["pairs"].append(q)
                    state["pending"] = None
                    log(f"  [쌍 {len(state['pairs'])}] dx={q['dx']:+.0f} dy={q['dy']:+.0f} px ({q['where']})")
                else:
                    log("  (dx 쌍 아님: RGB 창을 먼저 클릭하세요)")

            vis_rgb = rgb.copy()
            vis_d = _ui.depth_colormap(depth)
            for i, q in enumerate(state["pairs"]):
                u, v = cam.ref_to_screen(q["u_ref"], q["v_ref"], rgb_size)
                cv2.circle(vis_rgb, _ui.ipt((u, v)), 4, GREEN, -1)
                cv2.circle(vis_d, (int(q["u_d"]), int(q["v_d"])), 4, WHITE, -1)
            if state["pending"] is not None:
                cv2.drawMarker(vis_rgb, state["pending"], YELLOW, cv2.MARKER_CROSS, 16, 2)
            valid = 100.0 * np.count_nonzero(depth) / depth.size
            items_d = [(f"depth 유효 {valid:.0f} %   쌍 {len(state['pairs'])}개   s: 분석  u: 쌍 지우기  q: 종료",
                        f"valid {valid:.0f}%  pairs {len(state['pairs'])}  s:analyse u:undo q:quit",
                        (8, cam.height - 28), WHITE, 18)]
            items_r = [("RGB 클릭 → 바로 depth 창에서 같은 지점 클릭" if state["pending"] is None
                        else "이제 depth 창에서 같은 지점을 클릭하세요",
                        "click RGB, then the same point in depth" if state["pending"] is None
                        else "now click the same point in depth", (8, 8), YELLOW, 18)]
            text.draw(vis_d, items_d)
            text.draw(vis_rgb, items_r)
            cv2.imshow("RGB", vis_rgb)
            cv2.imshow("depth", vis_d)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s"):
                analyse_pairs(state["pairs"], cam)
            if k == ord("u") and state["pairs"]:
                state["pairs"].pop()
                log(f"[되돌리기] 남은 쌍 {len(state['pairs'])}개")
    finally:
        analyse_pairs(state["pairs"], cam)
        if measurements:
            log("\n[30프레임 노이즈] 위치별 표준편차 (mm)")
            for kind, title in (("surface", "책상면·일반 점 (노이즈 판단용)"),
                                ("pair_edge", "dx 쌍 모서리 (가장자리라 노이즈가 큼, 노이즈 판단에 쓰지 않음)")):
                rows = [m for m in measurements if m["kind"] == kind]
                if not rows:
                    continue
                log(f"  [{title}]")
                for m in rows:
                    s1 = (f"{m['sd_single']:.1f} (유효 {m['n_single']}/{m['n']})" if m["n_single"] >= 2
                          else f"⚠️ 유효 {m['n_single']}/{m['n']}")
                    log(f"    {m['where']:<26} Z {m['Z']:.3f} m  픽셀 하나 σ {s1}  5x5 중앙값 σ {m['sd_patch']:.1f}")
        if state["pairs"] or measurements:
            out_dir = Path(args.config_dir).parent / "data" / "sessions"
            out_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y-%m-%d_%H%M")
            for name, rows in (("camera_check_pairs", state["pairs"]), ("camera_check_noise", measurements)):
                if rows:
                    path = out_dir / f"{name}_{stamp}.csv"
                    with open(path, "w", newline="", encoding="utf-8") as f:
                        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                        w.writeheader()
                        w.writerows(rows)
                    log(f"[저장] {path}")
        cap.release()
        depth_dev.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
