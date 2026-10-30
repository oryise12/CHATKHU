"""장면 2: 물체 좌표 (base_ref 기준 윗면 중심 x, y, z_top) 표시, 클릭 선택·추적, --ros 발행.

python scripts/detect_objects.py                 보기만
python scripts/detect_objects.py --ros           /pac/objects (PoseArray), /pac/target_point (PointStamped) 발행
python scripts/detect_objects.py --model 경로     로컬 가중치 사용 (기본: config/detect.yaml의 파일 이름, 없으면 내려받음)

조작: 물체 클릭 = 선택, c 또는 빈 곳 클릭 = 선택 해제, z = 책상면 z 점검 켜기/끄기, q = 종료
⚠️ 물체 좌표는 depth 픽셀에서 직접 계산하므로 dx·dy가 밀려 있으면 일정한 오프셋이 남는다 → 줄자로 잰 위치와 비교해 확인.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from pac_vision import AstraDepth
from pac_vision.cam2base import roi_points, z_distribution
from pac_vision.config import has_pose
from pac_vision.objects import box_to_depth_roi, nearest_detection, top_center
from pac_vision.session import MissingCalibration, calibration_matches_current, require_cam2base

import _ui
from _ui import GRAY, GREEN, PURPLE, WHITE, YELLOW, log

WIN = "pac_vision detect"
HEADER_H = 40          # 위쪽 제목줄 높이 (박스 라벨이 가리지 않게)


def load_model(path):
    # 실행 중 pip 자동 설치를 막는다 (가중치 내려받기는 허용). ultralytics import 전에 설정해야 한다.
    os.environ["YOLO_AUTOINSTALL"] = "false"
    from ultralytics import YOLO
    return YOLO(path)


def run_model(model, rgb, det_cfg, tracking):
    kw = dict(imgsz=int(det_cfg["imgsz"]), conf=float(det_cfg["conf"]), classes=det_cfg.get("classes"),
              device=det_cfg.get("device", "cpu"), verbose=False)
    if tracking:
        res = model.track(rgb, persist=True, tracker=det_cfg.get("tracker", "bytetrack.yaml"), **kw)
    else:
        res = model.predict(rgb, **kw)
    b = res[0].boxes
    if b is None or len(b) == 0:
        return []
    xyxy = b.xyxy.cpu().numpy()
    cls = b.cls.cpu().numpy().astype(int)
    conf = b.conf.cpu().numpy()
    ids = b.id.cpu().numpy().astype(int) if (tracking and b.id is not None) else [None] * len(xyxy)
    return [dict(box=xyxy[i], cls=int(cls[i]), name=model.names[int(cls[i])], conf=float(conf[i]),
                 tid=None if ids[i] is None else int(ids[i])) for i in range(len(xyxy))]


def process_frame(dets, depth, conv, rgb_size, obj_cfg):
    """검출마다 박스 안 depth → base 좌표 → 윗면 중심. det에 valid, xyz, top_uv(윗면 띠 RGB 픽셀)를 채운다."""
    cam = conv.cam
    desk = conv.desk_plane() if cam.has_z_term else None
    for d in dets:
        d.update(valid=False, xyz=None, top_uv=None)
        z_box = None
        if desk is not None:        # 거리 식: 박스 아래 가운데를 책상면에 투영한 Z로 영역을 옮긴다
            x1, _, x2, y2 = (float(a) for a in d["box"])
            z_box = float(cam.rgb_to_depth_on_plane((x1 + x2) / 2, y2, desk, rgb_size)[2][0, 2])
            z_box = z_box if np.isfinite(z_box) and z_box > 0 else None
        roi = box_to_depth_roi(d["box"], cam, rgb_size, obj_cfg["box_shrink"], z_m=z_box)
        if roi is None:
            continue
        pts, ud, vd, _ = roi_points(depth, cam, roi)
        if len(pts) == 0:
            continue
        tc = top_center(conv.camera_to_base(pts), obj_cfg["min_height_m"], obj_cfg["max_height_m"],
                        obj_cfg["top_percentile"], obj_cfg["top_band_m"], obj_cfg["min_pixels"])
        if tc.valid:
            u, v = cam.depth_to_rgb(ud[tc.top_mask], vd[tc.top_mask], rgb_size, z_m=pts[tc.top_mask, 2])
            d.update(valid=True, xyz=tc.xyz, top_uv=np.stack([u, v], axis=1))
    return dets


def desk_z_overlay(depth, conv, rgb_size, min_h, band_m=0.02, step=2):
    """화면 전체 depth → base z. 반환: (분포 사전, 기준 높이를 넘는 픽셀의 RGB 좌표)."""
    cam = conv.cam
    pts, ud, vd, _ = roi_points(depth, cam, (0, 0, cam.width, cam.height), step)
    if len(pts) == 0:
        return {}, np.zeros((0, 2))
    z = conv.camera_to_base(pts)[:, 2]
    dist = z_distribution(z, pts[:, 2], band_m, min_h)
    hi = z > min_h
    u, v = cam.depth_to_rgb(ud[hi], vd[hi], rgb_size, z_m=pts[hi, 2])
    return dist, np.stack([u, v], axis=1)


def fmt_xyz(p):
    return f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m"


class Selection:
    """선택한 물체: 추적 ID로 따라가고, ID를 잃으면 마지막 위치에서 가장 가까운 같은 클래스 검출로 대체."""

    def __init__(self, obj_cfg):
        self.cfg = obj_cfg
        self.clear()

    def clear(self):
        self.tid = self.cls = self.name = self.xyz = None
        self.last_seen = 0.0
        self.det = None

    @property
    def active(self):
        return self.cls is not None

    def select(self, d):
        self.tid, self.cls, self.name = d["tid"], d["cls"], d["name"]
        self.xyz = d["xyz"]
        self.last_seen = time.time()
        self.det = d

    def update(self, dets):
        """이번 프레임에서 선택 물체 찾기. 반환: 'ok' / 'fallback' / 'lost' / 'missing'"""
        if not self.active:
            return None
        found, how = None, "ok"
        if self.tid is not None:
            found = next((d for d in dets if d["tid"] == self.tid and d["cls"] == self.cls), None)
        if found is None and self.xyz is not None:
            i = nearest_detection(self.xyz, self.cls, [(d["cls"], d["xyz"]) for d in dets], self.cfg["nearest_max_m"])
            if i is not None:
                found, how = dets[i], "fallback"
                self.tid = found["tid"]
        now = time.time()
        if found is not None:
            self.det = found
            self.last_seen = now
            if found["valid"]:
                self.xyz = found["xyz"]
            return how
        self.det = None
        if now - self.last_seen > self.cfg["lost_timeout_s"]:
            return "lost"
        return "missing"


def draw(rgb, dets, sel, conv, rgb_size, axis_len, overlay_pts):
    vis = rgb.copy()
    if overlay_pts is not None and len(overlay_pts):
        p = np.round(overlay_pts).astype(int)
        ok = (p[:, 0] >= 0) & (p[:, 0] < vis.shape[1]) & (p[:, 1] >= 0) & (p[:, 1] < vis.shape[0])
        vis[p[ok, 1], p[ok, 0]] = PURPLE
    for d in dets:
        x1, y1, x2, y2 = (int(round(a)) for a in d["box"])
        chosen = sel.det is d
        if d["valid"]:
            if d.get("top_uv") is not None and len(d["top_uv"]):
                p = np.round(d["top_uv"]).astype(int)
                ok = (p[:, 0] >= 0) & (p[:, 0] < vis.shape[1]) & (p[:, 1] >= 0) & (p[:, 1] < vis.shape[0])
                layer = vis.copy()
                layer[p[ok, 1], p[ok, 0]] = (0, 200, 255)
                vis = cv2.addWeighted(layer, 0.5, vis, 0.5, 0)    # 윗면으로 쓴 픽셀 표시
            color = YELLOW if chosen else GREEN
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 4 if chosen else 2)
        else:
            _ui.dashed_rect(vis, (x1, y1), (x2, y2), GRAY)
            if chosen:
                cv2.rectangle(vis, (x1, y1), (x2, y2), YELLOW, 4)
    _ui.base_axes(vis, conv, axis_len, rgb_size)
    return vis


def box_labels(dets, text_size):
    items = []
    for d in dets:
        x1, y1 = int(d["box"][0]), int(d["box"][1])
        y1 = y1 if y1 - text_size - 8 >= HEADER_H else y1 + text_size + 12     # 위쪽 제목줄에 가리면 박스 안쪽에
        if d["valid"]:
            items.append((f"{d['name']} {d['conf']:.2f}  {fmt_xyz(d['xyz'])}", f"{d['name']} {d['conf']:.2f} {fmt_xyz(d['xyz'])}",
                          (x1, max(0, y1 - text_size - 8)), WHITE, text_size))
        else:
            items.append((f"{d['name']} {d['conf']:.2f}  depth 부족 (좌표 없음)", f"{d['name']} no depth",
                          (x1, max(0, y1 - text_size - 8)), GRAY, text_size))
    return items


def main():
    ap = argparse.ArgumentParser(description="pac_vision 물체 좌표 (base_ref 윗면 중심)")
    ap.add_argument("--config-dir", default=str(_ui.DEFAULT_CONFIG_DIR))
    ap.add_argument("--model", default=None, help="YOLO 가중치 경로 (기본: detect.yaml의 model)")
    ap.add_argument("--ros", action="store_true", help="ROS 2로 발행 (먼저 ROS를 source)")
    args = ap.parse_args()

    cfg_dir = Path(args.config_dir).resolve()
    cfg = _ui.load_config_or_exit(cfg_dir, ("camera", "thresholds", "frames", "detect"))
    if not has_pose(cfg_dir):
        log(f"[pac_vision] ⚠️ {cfg_dir.name}/pose.yaml(이 설치 자세의 정합 계수)이 없습니다. "
            "물체 좌표는 보정 때 계수로 계산하지만, 이 자세에서 잰 계수인지 확인하세요 (README 3) 카메라 확인)")
    try:
        conv = require_cam2base(cfg_dir / "cam2base.yaml")      # 형식 예시로 대체하지 않는다
    except MissingCalibration as e:
        sys.exit(f"[pac_vision] {e}")
    if not calibration_matches_current(conv, cfg["camera"]):
        log("[pac_vision] ⚠️ 보정(cam2base.yaml) 때와 지금 설정의 RGB→depth 정합 계수가 다릅니다. 물체 좌표는 보정 때 계수로 계산합니다. "
            "정합을 다시 쟀다면 calibrate.py --replay <세션> --use-current-config로 보정을 갱신하세요")
    info = conv.info
    created = str(info.get("created", "?")).replace("T", " ")[:16]
    log(f"[pac_vision] 보정 읽음: {cfg_dir.name}/cam2base.yaml ({created}, base 점 오차 "
        f"{info.get('holdout_m', float('nan')) * 100:.1f} cm)")
    det_cfg, obj_cfg, frames = cfg["detect"], cfg["thresholds"]["objects"], cfg["frames"]

    pub = None
    if args.ros:
        try:
            from pac_vision.ros_pub import ObjectPublisher      # rclpy는 --ros일 때만
        except ImportError as e:
            sys.exit(f"[pac_vision] ROS 2를 불러오지 못했습니다 ({e}). 먼저 source /opt/ros/<humble|jazzy>/setup.bash")
        pub = ObjectPublisher(frames)

    model = load_model(args.model or det_cfg["model"])
    log(f"[pac_vision] YOLO 모델 로드 완료 ({str(det_cfg.get('device', 'cpu')).upper()})")
    tracking = importlib.util.find_spec("lap") is not None
    if not tracking:
        log("[pac_vision] ⚠️ lap 패키지가 없어 추적 없이 검출만 합니다 (선택은 최근접 대체로 잇습니다, 자동 설치 안 함)")
    if pub is not None:
        log(f"[pac_vision] 발행: {frames['topics']['objects']} (PoseArray), {frames['topics']['target']} (PointStamped), "
            f"frame={frames['base_frame']}")
    log("[pac_vision] ⚠️ 물체 좌표의 일정한 오프셋(dx·dy)은 줄자로 잰 위치와 비교해서만 확인할 수 있습니다")

    cap, rgb_size = _ui.open_rgb(cfg["camera"])
    dev = AstraDepth.from_config(cfg["camera"]).open()
    text = _ui.TextDrawer(cfg["camera"]["ui"]["font"])
    size = cfg["camera"]["ui"]["font_size"]
    screen = _ui.Screen(WIN, text, size)
    axis_len = cfg["camera"]["ui"]["axis_len_m"]
    sel = Selection(obj_cfg)
    show_z, fps, t_prev = False, 0.0, time.time()
    try:
        while True:
            ok, rgb = cap.read()
            depth = dev.read()
            if not ok:
                continue
            dets = process_frame(run_model(model, rgb, det_cfg, tracking), depth, conv, rgb_size, obj_cfg)

            c = screen.take_click()
            if c is not None:
                inside = [d for d in dets if d["box"][0] <= c[0] <= d["box"][2] and d["box"][1] <= c[1] <= d["box"][3]]
                if inside:
                    d = min(inside, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))
                    sel.select(d)
                    where = (f"base ({d['xyz'][0]:.3f}, {d['xyz'][1]:.3f}, {d['xyz'][2]:.3f}) m" if d["valid"]
                             else "좌표 없음 (depth 부족)")
                    log(f"[선택] {d['name']}  {where}" + (f" → {frames['topics']['target']} 발행 시작" if pub else ""))
                elif sel.active:
                    sel.clear()
                    log("[선택 해제]")

            state = sel.update(dets)
            if state == "fallback":
                log(f"[추적] ID를 잃어 마지막 위치에서 가장 가까운 {sel.name} 검출로 대체했습니다")
            elif state == "lost":
                log("⚠️ 선택 물체를 놓침. 발행 중지")
                sel.clear()

            if pub is not None:
                pub.publish_objects([d["xyz"] for d in dets if d["valid"]])
                if sel.active and sel.det is not None and sel.det["valid"]:
                    pub.publish_target(sel.det["xyz"])

            overlay = None
            if show_z:
                dist, overlay = desk_z_overlay(depth, conv, rgb_size, obj_cfg["min_height_m"], obj_cfg["desk_band_m"])

            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3) if fps else 1.0 / max(now - t_prev, 1e-3)
            t_prev = now
            vis = draw(rgb, dets, sel, conv, rgb_size, axis_len, overlay)
            text.draw(vis, box_labels(dets, size - 2))
            sel_txt = "없음" if not sel.active else sel.name + ("" if sel.det is not None else " (찾는 중)")
            top = (f"물체 좌표 (base 기준)   FPS {fps:.0f}   선택: {sel_txt}",
                   f"objects (base)  FPS {fps:.0f}  selected: {sel.name if sel.active else 'none'}", WHITE)
            lines = [("물체 클릭: 선택   c: 선택 해제   z: 책상면 z 점검   q: 종료",
                      "click: select   c: clear   z: desk z check   q: quit", WHITE)]
            if show_z:
                lines.insert(0, ("보라: 책상면 위 {:.0f} cm를 넘는 픽셀 (빈 책상이면 거의 없어야 함)".format(obj_cfg["min_height_m"] * 100),
                                 "purple: pixels above min height", PURPLE))
            screen.show(vis, lines, top=top)

            k = screen.key(1)
            if k == ord("q"):
                break
            if k == ord("c") and sel.active:
                sel.clear()
                log("[선택 해제]")
            if k == ord("z"):
                show_z = not show_z
                if show_z:
                    dist, _ = desk_z_overlay(depth, conv, rgb_size, obj_cfg["min_height_m"], obj_cfg["desk_band_m"])
                    for key, name in (("near", "가까운 쪽"), ("far", "먼 쪽")):
                        if key in dist:
                            d = dist[key]
                            log(f"[책상면 z] {name} 중앙값 {d['median'] * 100:+.1f} cm (5-95%: {d['p5'] * 100:+.1f} ~ "
                                f"{d['p95'] * 100:+.1f} cm, 점 {d['n']}), 1 cm 넘게 뜬 점 {d['raised'] * 100:.1f} %, "
                                f"±2 cm 밖이라 뺀 점 {d['n_outside']}")
                    log("  (책상면 ±2 cm 안의 점으로만 계산. 1 cm 넘게 뜬 점이 많으면 책상이 물체로 세질 수 있음. 빈 책상에서 보세요)")
    finally:
        cap.release()
        dev.close()
        if pub is not None:
            pub.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
