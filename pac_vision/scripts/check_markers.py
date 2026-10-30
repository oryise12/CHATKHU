"""고정 표지(ArUco)로 보정이 지금도 맞는지 확인한다 (카메라 필요, 시뮬레이터는 닫은 상태 - 카메라는 한 프로그램만 연다).

python scripts/check_markers.py --sheet markers_sheet.png    인쇄용 표지 한 장 (markers.yaml 또는 markers.example.yaml의 id·크기)
python scripts/check_markers.py                              카메라로 확인: 표지별 이미지 오차(px), 책상면 오차(mm)
python scripts/check_markers.py --frames 30 --no-gui         창 없이 숫자만

준비 (CALIBRATION_GUIDE.md "고정 표지"):
  1. 표지를 100 % 크기로 인쇄해 책상(보정 종이 밖, 로봇 작업 영역을 가리지 않는 곳)에 평평하게 붙인다.
  2. 각 표지 중심의 base_ref 좌표(m, A4 원점 기준 +X 앞, +Y 왼쪽)를 줄자로 재서 config/markers.yaml에 적는다.
     보정 계산에는 쓰지 않는 독립 점이다. 형식은 config/markers.example.yaml.
결과는 config/markers.check.yaml(git 제외)에 날짜와 함께 남는다. 판정 규칙: src/pac_vision/markers.py.
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from pac_vision.devices import identity_problems, rgb_identity
from pac_vision.markers import (average_detections, detect_quads, load_markers, make_sheet, shortest_side_px,
                                square_center, verify)
from pac_vision.session import MissingCalibration, calibration_mismatches, require_cam2base

import _ui
from _ui import GREEN, RED, WHITE, YELLOW, log

MIN_SIDE_PX = 30.0      # 이보다 작게 보이는 표지는 검출이 흔들린다 (합성 시험: 13 x 24 px에서 검출 누락)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", default=str(_ui.DEFAULT_CONFIG_DIR))
    ap.add_argument("--sheet", metavar="PNG", help="인쇄용 표지 이미지를 만들고 끝낸다")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--frames", type=int, default=20, help="표지 위치를 평균할 프레임 수")
    ap.add_argument("--no-gui", action="store_true")
    a = ap.parse_args()
    cfg_dir = Path(a.config_dir).resolve()
    mpath = cfg_dir / "markers.yaml"
    if a.sheet:
        src = mpath if mpath.exists() else cfg_dir / "markers.example.yaml"
        ms = load_markers(src)
        cv2.imwrite(a.sheet, make_sheet(ms, dpi=a.dpi))
        log(f"[표지] {a.sheet}: id {sorted(ms.markers)}, 한 변 {ms.size_m * 1000:.0f} mm ({src.name}). "
            "100 % 크기로 인쇄하고 자로 한 변을 재 확인하세요")
        return 0
    if not mpath.exists():
        sys.exit(f"[pac_vision] {mpath.name}가 없습니다. markers.example.yaml을 복사해 표지 id와 잰 좌표를 적으세요")
    ms = load_markers(mpath)
    cfg = _ui.load_config_or_exit(cfg_dir, ("camera",))
    try:
        conv = require_cam2base(cfg_dir / "cam2base.yaml")
    except MissingCalibration as e:
        sys.exit(f"[pac_vision] {e}")
    for m in calibration_mismatches(conv, cfg["camera"]):
        log(f"[pac_vision] ⚠ 보정 때와 지금 카메라 설정이 다릅니다: {m}")
    if not conv.info.get("passed", False):
        log("[pac_vision] ⚠ 이 보정은 통과하지 못한 결과(passed: false)입니다")
    cap, size = _ui.open_rgb(cfg["camera"])
    ident = rgb_identity(_ui.opened_rgb_device) if _ui.opened_rgb_device is not None else {}
    for p in identity_problems(conv.info.get("camera_identity"), ident):
        log(f"[pac_vision] ⚠ 카메라 신원: {p}")
    seen_frames, quads, frame = [], {}, None
    for _ in range(max(1, a.frames)):
        ok, frame = cap.read()
        if not ok:
            continue
        q = detect_quads(frame, ms.dictionary)
        quads.update(q)
        seen_frames.append({k: square_center(v) for k, v in q.items()})
    cap.release()
    if frame is None:
        sys.exit("[pac_vision] RGB 프레임을 받지 못했습니다")
    seen = average_detections(seen_frames)
    res = verify(conv, ms, seen, rgb_size=size)
    log(f"[표지] {res.summary()}  (허용 {ms.max_pixel_px:.0f} px, {ms.max_desk_mm:.0f} mm, 최소 {ms.min_visible}개)")
    for c in res.checks:
        if c.ok is None:
            log(f"  id {c.id}: 안 보임 (기대 위치 {np.round(c.expected_px, 1).tolist()} px)")
            continue
        side = shortest_side_px(quads[c.id]) if c.id in quads else float("nan")
        warn = f"  ⚠ 화면에서 {side:.0f} px로 작음 (≥ {MIN_SIDE_PX:.0f} px 권장: 더 크게 인쇄하거나 카메라 가까이)" \
            if side < MIN_SIDE_PX else ""
        log(f"  id {c.id}: 이미지 {c.pixel_px:.1f} px, 책상면 {c.desk_mm:.1f} mm {'✓' if c.ok else '×'}{warn}")
    out = cfg_dir / "markers.check.yaml"
    doc = {"created": datetime.now().isoformat(timespec="seconds"), "calibration_created": conv.info.get("created"),
           "rgb_size": list(size), "frames": len(seen_frames), "camera_identity": ident,
           "thresholds": {"max_pixel_px": ms.max_pixel_px, "max_desk_mm": ms.max_desk_mm, "min_visible": ms.min_visible},
           **res.to_dict()}
    out.write_text("# check_markers.py 결과 (현장 전용, git 제외)\n" +
                   yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    log(f"[저장] {out.name}")
    if not a.no_gui:
        vis = frame.copy()
        for c in res.checks:
            e = tuple(int(round(x)) for x in c.expected_px)
            cv2.drawMarker(vis, e, RED, cv2.MARKER_CROSS, 18, 2)
            if c.seen_px is not None:
                s = tuple(int(round(x)) for x in c.seen_px)
                cv2.circle(vis, s, 8, GREEN if c.ok else YELLOW, 2)
                cv2.putText(vis, f"{c.id}: {c.pixel_px:.1f}px {c.desk_mm:.0f}mm", (s[0] + 10, s[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
        cv2.putText(vis, "red + = expected (measured base_m), circle = seen. any key: quit", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
        cv2.imshow("pac_vision markers", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0 if res.verified else 1


if __name__ == "__main__":
    sys.exit(main())
