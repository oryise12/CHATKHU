"""Demo stations on the desk from ArUco markers (2026-10-29): S = dish basket (6), B = soup bowl (7), C = rice bowl (8).

The robot goes from HOME (A) to S by itself once the arm is armed, then the user picks B or C by voice: a straight path
S -> station is drawn, edited (pinch / pointing + voice) and run, the way back is edited or reversed, and the remaining
station follows. With both done, 쿠옹이 asks whether to go again (a new round from S).

Positions: the markers are read from the live colour image once at start (the arm covers them later): pixel centres
averaged over `frames` frames -> the calibrated desk plane (pac_vision cam2base) -> MuJoCo world. Without a camera
(sim / canvas test) a station may carry `base_m` (calibration frame, m) instead.

Config (pac_vision/config/stations.yaml):
    dictionary: DICT_4X4_50
    frames: 20
    stations:
      S: {marker: 6, label: 설거지 바구니, height_m: 0.20}
      B: {marker: 7, label: 국그릇, height_m: 0.12}
      C: {marker: 8, label: 밥그릇, height_m: 0.12}
height_m: the target TCP height above the desk at that station (the straight path runs between these points).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml

ORDER = ("S", "B", "C")
COLORS = {"S": (150, 110, 40), "B": (30, 120, 230), "C": (60, 160, 60)}       # BGR on the camera image
RGBA = {"S": (0.16, 0.43, 0.59, 1), "B": (0.90, 0.47, 0.12, 1), "C": (0.24, 0.63, 0.24, 1)}   # 3-D panel


@dataclass
class Station:
    name: str                      # S | B | C
    label: str                     # 설거지 바구니 / 국그릇 / 밥그릇
    marker: int
    height_m: float
    floor_m: np.ndarray | None = None     # marker centre on the desk, MuJoCo world (z = 0)
    base_m: np.ndarray | None = None      # the same in the calibration frame (printed, tests)

    @property
    def tcp_goal_m(self) -> np.ndarray:
        return self.floor_m + np.array([0.0, 0.0, self.height_m])

    @property
    def title(self) -> str:
        return f"{self.name} {self.label}"


@dataclass
class StationConfig:
    dictionary: str
    frames: int
    stations: dict[str, Station] = field(default_factory=dict)
    dip_m: float = 0.0              # S -> B / C and the way back to S end this far below the target and stay


def load_config(path: str | Path) -> StationConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    st = {}
    for name in ORDER:
        s = (raw.get("stations") or {}).get(name)
        if s is None:
            raise ValueError(f"{path}: station {name} missing (needs S, B, C)")
        base = s.get("base_m")
        st[name] = Station(name, str(s.get("label", name)), int(s["marker"]), float(s.get("height_m", 0.12)),
                           base_m=None if base is None else np.array([*base[:2], 0.0], float))
    return StationConfig(str(raw.get("dictionary", "DICT_4X4_50")), int(raw.get("frames", 20)), st,
                         float(raw.get("dip_m", 0.0)))


def from_base(cfg: StationConfig, cal_to_world) -> bool:
    """No camera: use base_m from the config. True if every station has one."""
    if any(s.base_m is None for s in cfg.stations.values()):
        return False
    for s in cfg.stations.values():
        s.floor_m = np.asarray(cal_to_world(s.base_m), float)
        s.floor_m[2] = 0.0
    return True


class StationReader:
    """Collects marker centres from live frames until every station marker was seen in `frames` frames."""

    def __init__(self, cfg: StationConfig, cam, cal_to_world, timeout_note_s: float = 5.0):
        self.cfg, self.cam, self.cal_to_world = cfg, cam, cal_to_world
        self.seen: dict[int, list[np.ndarray]] = {s.marker: [] for s in cfg.stations.values()}
        self.done = False
        self.n_frames = 0
        self.timeout_note_s = timeout_note_s
        self._noted = 0.0

    def missing(self) -> list[str]:
        return [s.title for s in self.cfg.stations.values() if len(self.seen[s.marker]) < self.cfg.frames]

    def update(self, frame_bgr, now: float) -> bool:
        """One frame. True once (when all stations were located)."""
        if self.done or frame_bgr is None:
            return False
        from pac_vision.markers import detect
        self.n_frames += 1
        for mid, uv in detect(frame_bgr, self.cfg.dictionary).items():
            if mid in self.seen:
                self.seen[mid].append(np.asarray(uv, float).reshape(2))
        if self.missing():
            if now - self._noted >= self.timeout_note_s:
                self._noted = now
                print(f"stations: looking for markers - not yet: {', '.join(self.missing())} "
                      f"({self.n_frames} frames)", flush=True)
            return False
        from pac_vision.markers import _plane_at
        h, w = frame_bgr.shape[:2]
        conv = self.cam.conv
        for s in self.cfg.stations.values():
            uv = np.median(np.array(self.seen[s.marker][-self.cfg.frames:]), axis=0)
            pts, _, _ = conv.pixel_to_desk(uv[0], uv[1], rgb_mirrored=self.cam.mirrored, rgb_size=(w, h),
                                           plane=_plane_at(conv, 0.0))
            p = np.asarray(pts, float).reshape(-1, 3)[0]
            s.base_m = np.array([p[0], p[1], 0.0])
            s.floor_m = np.asarray(self.cal_to_world(s.base_m), float)
            s.floor_m[2] = 0.0
        self.done = True
        return True


def describe(cfg: StationConfig, robot_xy=(0.0, 0.0)) -> list[str]:
    """One line per station: calibration-frame position, distance from the robot base and a reach note."""
    out = []
    for s in cfg.stations.values():
        if s.base_m is None:
            continue
        r = float(np.hypot(s.base_m[0] - robot_xy[0], s.base_m[1] - robot_xy[1]))
        note = "" if 0.20 <= r <= 0.35 else "  <- far from the usual reach (0.25-0.30 m): validation decides"
        out.append(f"  {s.title} (marker {s.marker}): ({s.base_m[0]:+.3f}, {s.base_m[1]:+.3f}) m, "
                   f"r = {r:.3f} m, target height {s.height_m:.2f} m{note}")
    return out


def draw_stations(img, cam, stations: dict[str, Station], visited, text, current: str | None = None) -> None:
    """Camera image: a ring on each marker and its name (S 설거지 바구니, B 국그릇 ✓, ...)."""
    from .ui import blit
    for s in stations.values():
        if s.floor_m is None:
            continue
        uv, front = cam.project(s.floor_m)
        if not front[0]:
            continue
        x, y = (int(round(v)) for v in uv[0])
        color = COLORS.get(s.name, (200, 200, 200))
        cv2.circle(img, (x, y), 9 if s.name == current else 7, color, 2, cv2.LINE_AA)   # frame is shown about x2
        label = s.title + (" ✓" if s.name in visited else "")
        spr = text.sprite(label, 13, rgb=(255, 255, 255))
        h, w = spr.shape[:2]
        x0, y0 = x - w // 2, y + 11
        x0 = int(np.clip(x0, 2, img.shape[1] - w - 2))
        y0 = int(np.clip(y0, 2, img.shape[0] - h - 2))
        cv2.rectangle(img, (x0 - 4, y0 - 2), (x0 + w + 4, y0 + h + 2), color, -1, cv2.LINE_AA)
        blit(img, spr, x0, y0)
