"""Astra depth 읽기 (OpenNI2). RGB는 UVC라 OpenCV로 따로 연다.

좌우반전: depth를 먼저 좌우로 뒤집어 RGB와 같은 방향으로 만들고, 역투영의 X 부호는 +로 둔다 (한 가지 방식으로 통일).
openni는 open() 안에서만 import하므로, 이 모듈을 import해도 카메라 라이브러리가 필요 없다.
"""
from __future__ import annotations

import time

import numpy as np

from .cam2base import average_depth
from .config import expand_path


class AstraDepth:
    def __init__(self, redist: str, flip: bool = True, expected_size=(640, 480), rotate_180: bool = False):
        self.redist = expand_path(redist)
        self.flip = bool(flip)
        self.rotate_180 = bool(rotate_180)       # 카메라를 뒤집어 달았을 때: 좌우반전 뒤에 180° 회전 (RGB도 같이 돌린다)
        self.expected_size = tuple(expected_size)
        self._openni2 = None
        self._dev = None
        self._stream = None
        self._last_index = None

    @classmethod
    def from_config(cls, camera_cfg: dict) -> "AstraDepth":
        d = camera_cfg["depth"]
        return cls(camera_cfg["openni"]["redist"], d.get("flip", True), (d["width"], d["height"]),
                   rotate_180=bool(camera_cfg.get("rotate_180", False)))

    def open(self) -> "AstraDepth":
        # ⚠️ Jazzy: openni 파이썬 패키지와 Orbbec redist 경로가 컴퓨터마다 다름 (config/camera.yaml)
        from openni import openni2
        self._openni2 = openni2
        openni2.initialize(self.redist)
        self._dev = openni2.Device.open_any()
        self._stream = self._dev.create_depth_stream()
        self._stream.start()
        return self

    def read(self) -> np.ndarray:
        """다음 depth 프레임 (uint16, mm, 좌우반전 보정 완료). 새 프레임이 올 때까지 기다린다."""
        frame = self._stream.read_frame()
        self._last_index = getattr(frame, "frameIndex", None)
        d = np.frombuffer(frame.get_buffer_as_uint16(), dtype=np.uint16).reshape(frame.height, frame.width)
        d = d[:, ::-1] if self.flip else d
        d = d[::-1, ::-1] if self.rotate_180 else d
        return np.array(d, copy=True)           # OpenNI 버퍼는 다음 프레임에 덮어쓰이므로 항상 복사 (flip이 없어도)

    @property
    def last_index(self):
        """마지막으로 읽은 프레임 번호 (같은 프레임을 두 번 읽었는지 확인용). 없으면 None."""
        return self._last_index

    def latest(self):
        """read()와 같지만 실패하면 None."""
        try:
            return self.read()
        except Exception:
            return None

    def read_average(self, n: int, min_valid_fraction: float = 0.5, timeout_s: float = 10.0):
        """서로 다른 프레임 n장을 평균 (0 제외). 반환: (평균 depth float32 mm, 픽셀별 유효 비율)"""
        frames, last, t0 = [], None, time.time()
        while len(frames) < n and time.time() - t0 < timeout_s:
            d = self.read()
            idx = self._last_index
            if idx is not None and idx == last:     # 같은 프레임을 두 번 읽지 않기
                time.sleep(0.005)
                continue
            last = idx
            frames.append(d)
        if not frames:
            raise RuntimeError("depth 프레임을 읽지 못했습니다")
        return average_depth(frames, min_valid_fraction)

    def close(self):
        if self._stream is not None:
            self._stream.stop()
            self._stream = None
        if self._openni2 is not None:
            self._openni2.unload()      # 이 openni 버전의 종료 함수는 unload()
            self._openni2 = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
