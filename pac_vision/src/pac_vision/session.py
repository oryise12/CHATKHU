"""캘리브레이션 세션(npz)과 보정 결과(cam2base.yaml) 저장·읽기. numpy + PyYAML만 쓴다.

세션 파일에는 정지 화면(RGB, 평균 depth), 클릭 5점, 그때의 설정 사본, 결과 요약이 들어 있어
카메라 없이 같은 계산을 다시 할 수 있다 (--replay).
"""
from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from .cam2base import CalibResult, Cam2Base, CameraModel, solve_calibration

SESSION_VERSION = 1


def save_session(path, rgb, depth_mm, clicks_rgb, rgb_size, mode, configs: dict, summary: dict,
                 created: str | None = None) -> Path:
    """configs: {"camera": ..., "a4_layout": ..., "thresholds": ...} 계산에 쓴 설정 사본."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    created = created or datetime.now().isoformat(timespec="seconds")
    # 문자열은 0차원 유니코드 배열로 저장 → allow_pickle 없이 읽힘
    np.savez_compressed(
        path,
        version=np.array(SESSION_VERSION),
        rgb=np.asarray(rgb, dtype=np.uint8),
        depth=np.asarray(depth_mm, dtype=np.float32),
        clicks=np.asarray(clicks_rgb, dtype=float),
        rgb_size=np.asarray(rgb_size, dtype=int),
        mode=np.array(mode),
        configs=np.array(json.dumps(configs, ensure_ascii=False)),
        summary=np.array(json.dumps(summary, ensure_ascii=False)),
        created=np.array(created),
    )
    return path


def load_session(path) -> dict:
    with np.load(path, allow_pickle=False) as z:
        return {
            "version": int(z["version"]),
            "rgb": z["rgb"],
            "depth": z["depth"],
            "clicks": z["clicks"],
            "rgb_size": tuple(int(x) for x in z["rgb_size"]),
            "mode": str(z["mode"]),
            "configs": json.loads(str(z["configs"])),
            "summary": json.loads(str(z["summary"])),
            "created": str(z["created"]),
        }


def replay_session(path, mode: str | None = None, camera_cfg: dict | None = None) -> tuple[CalibResult, dict]:
    """세션에 저장된 설정 사본으로 같은 계산을 다시 한다.

    mode를 주면 depth 모드만 바꿔 계산. camera_cfg를 주면 세션 사본 대신 그 카메라 설정(현재 camera.yaml)으로
    계산한다 (같은 자세에서 dx·dy만 다시 쟀을 때 재클릭 없이 갱신). 클릭은 RGB 픽셀이라 그대로 쓴다.
    """
    s = load_session(path)
    cfg = s["configs"]
    cam = CameraModel.from_config(camera_cfg if camera_cfg is not None else cfg["camera"])
    res = solve_calibration(s["depth"], s["clicks"], cam, cfg["a4_layout"], cfg["thresholds"],
                            mode=mode or s["mode"], rgb_size=s["rgb_size"])
    return res, s


def compare_summaries(a: dict, b: dict, keys=("R", "t", "camera_points", "holdout_m", "sides_m", "rms_m")) -> float:
    """두 결과 요약의 수치 최대 차이 (m 또는 R 원소)."""
    worst = 0.0
    for k in keys:
        worst = max(worst, float(np.max(np.abs(np.asarray(a[k], float) - np.asarray(b[k], float)))))
    return worst


def write_cam2base(path, result: CalibResult, camera_cfg: dict, session_file: str | None = None,
                   created: str | None = None) -> Path:
    """보정 결과 yaml. 계산에 쓴 카메라 설정 사본을 같이 넣어, 나중에 camera.yaml이 바뀌어도 같은 변환을 쓴다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    s = result.summary()
    doc = {
        "created": created or datetime.now().isoformat(timespec="seconds"),
        "depth_mode": result.mode,
        "R": s["R"],
        "t": s["t"],
        "checks": s["checks"],
        "passed": s["passed"],
        "rms_m": s["rms_m"],
        "holdout_m": s["holdout_m"],
        "sides_m": s["sides_m"],
        "plane_rms_m": s["plane_rms_m"],
        "tilt_deg": s["tilt_deg"],
        "desk_z": s["desk_z"],
        "session_file": session_file,
        "camera": camera_cfg,
    }
    header = ("# 카메라 → base_ref 보정 결과 (calibrate.py가 생성). P_base = R @ P_cam + t, 단위 m.\n"
              "# 현장 전용 값이라 git에 올리지 않는다. 형식은 cam2base.example.yaml 참고.\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False, default_flow_style=None)
    return path


class MissingCalibration(FileNotFoundError):
    pass


def require_cam2base(path) -> Cam2Base:
    """detect·--check용: 현장 보정 파일이 없으면 멈춘다. 형식 예시(cam2base.example.yaml)로 대체하지 않는다."""
    path = Path(path)
    if path.name.endswith(".example.yaml"):
        raise MissingCalibration(f"{path}는 형식 예시입니다. 먼저 calibrate.py를 실행하세요")
    if not path.exists():
        raise MissingCalibration(f"보정 파일이 없습니다: {path}\n먼저 calibrate.py를 실행하세요")
    return load_cam2base(path)


def load_cam2base(path) -> Cam2Base:
    with open(os.fspath(path), encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    cam = CameraModel.from_config(doc["camera"])
    return Cam2Base(doc["R"], doc["t"], cam, info=doc)


def rgb_depth_key(cam: CameraModel) -> tuple:
    """정합 계수 비교용 (sx, sy, dx 계수, dy 계수)."""
    return (cam.sx, cam.sy, tuple(cam.dx_coef), tuple(cam.dy_coef))


def calibration_matches_current(conv: Cam2Base, current_camera_cfg: dict) -> bool:
    """보정(cam2base.yaml) 때의 정합 계수가 지금 설정(camera.yaml + pose.yaml)과 같은가.

    다르면 정합을 다시 잰 뒤 보정을 다시 계산하지 않은 것이다 (--replay --use-current-config로 갱신)."""
    cur = CameraModel.from_config(current_camera_cfg)
    a, b = rgb_depth_key(conv.cam), rgb_depth_key(cur)
    return a[:2] == b[:2] and all(abs(x - y) < 1e-9 for x, y in zip(a[2] + a[3], b[2] + b[3]))

