"""config/ 폴더의 yaml 설정 읽기. 이름·상수·기준값은 모두 여기로 들어온다.

커밋된 설정 위에 git에 올리지 않는 두 파일을 덮어쓴다 (있을 때만):
- local.yaml: 이 컴퓨터에만 맞는 값 (장치 번호, OpenNI 경로, 글꼴, YOLO 모델 경로). 예시: local.example.yaml
- pose.yaml : 이 설치 자세의 RGB→depth 정합 계수. tools/fit_rgb_to_depth.py --write-pose가 만든다. 예시: pose.example.yaml
두 파일은 정해진 키만 받는다. 다른 키가 있으면 어느 파일의 어느 키인지 알려 주고 멈춘다 (ConfigError).
"""
from __future__ import annotations

import copy
import os
from pathlib import Path

import yaml

CONFIG_NAMES = ("camera", "a4_layout", "frames", "thresholds", "detect")

# 덮어쓰기 파일이 받는 키 (설정 이름 → 점으로 이은 경로)
LOCAL_KEYS = {
    "camera": ("rgb.device", "openni.redist", "ui.font", "rotate_180"),
    "detect": ("model", "device"),
}
POSE_KEYS = {
    "camera": ("rgb_to_depth.sx", "rgb_to_depth.sy", "rgb_to_depth.z_ref_m", "rgb_to_depth.dx", "rgb_to_depth.dy")
    + tuple(f"rgb_to_depth.{a}.{k}" for a in ("dx", "dy") for k in "abcd"),
}
OVERLAYS = (("local.yaml", LOCAL_KEYS), ("pose.yaml", POSE_KEYS))


class ConfigError(ValueError):
    """설정 파일 내용이 약속과 다를 때."""


def default_config_dir() -> Path:
    """저장소 안의 pac_vision/config (editable 설치 기준). 스크립트는 --config-dir로 바꿀 수 있다."""
    return Path(__file__).resolve().parents[2] / "config"


def load_yaml(path: str | os.PathLike) -> dict:
    # ⚠️ PyYAML 5.4(Humble) / 6.x(Jazzy) 모두 safe_load만 쓴다
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def _leaves(d: dict, prefix: str = ""):
    """중첩 사전 → (점으로 이은 경로, 값) 목록."""
    for k, v in d.items():
        path = f"{prefix}{k}"
        if isinstance(v, dict):
            yield from _leaves(v, path + ".")
        else:
            yield path, v


def _allowed(path: str, allowed: tuple) -> bool:
    return path in allowed


def check_overlay(doc: dict, keys: dict, filename: str) -> None:
    """덮어쓰기 파일이 정해진 키만 담았는지. 아니면 ConfigError."""
    if not isinstance(doc, dict):
        raise ConfigError(f"{filename}: 맨 위는 사전이어야 합니다")
    bad = []
    for name, sub in doc.items():
        if name not in keys:
            bad.append(name)
            continue
        if not isinstance(sub, dict):
            bad.append(name)
            continue
        bad += [f"{name}.{p}" for p, _ in _leaves(sub) if not _allowed(p, keys[name])]
    if bad:
        allowed = ", ".join(f"{n}.{k}" for n, ks in keys.items() for k in ks)
        raise ConfigError(f"{filename}: 받지 않는 키 {', '.join(bad)} (이 파일이 받는 키: {allowed})")


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and not _is_coef(k, v):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)       # 정합 계수(dx·dy)는 통째로 바꾼다 (옛 숫자 형식과 섞이지 않게)
    return out


def _is_coef(key, value) -> bool:
    return key in ("dx", "dy") and isinstance(value, dict)


def load_overlays(config_dir) -> dict:
    """{파일 이름: 내용} — 있는 덮어쓰기 파일만. 키 검사 포함."""
    out = {}
    for fname, keys in OVERLAYS:
        p = Path(config_dir) / fname
        if p.exists():
            doc = load_yaml(p)
            check_overlay(doc, keys, fname)
            out[fname] = doc
    return out


def load_config(config_dir: str | os.PathLike | None = None,
                names: tuple[str, ...] = CONFIG_NAMES, overlays: bool = True) -> dict:
    """{이름: 내용} 사전. 없는 파일은 FileNotFoundError. overlays=True면 local.yaml, pose.yaml을 덮어쓴다."""
    d = Path(config_dir) if config_dir is not None else default_config_dir()
    cfg = {name: load_yaml(d / f"{name}.yaml") for name in names}
    if overlays:
        for doc in load_overlays(d).values():
            for name, sub in doc.items():
                if name in cfg:
                    cfg[name] = _merge(cfg[name], sub)
    return cfg


def config_sources(config_dir: str | os.PathLike | None = None) -> list[str]:
    """덮어쓴 값의 출처 한 줄씩 (시작 로그용). 예: 'camera.rgb_to_depth.dx ← pose.yaml'."""
    d = Path(config_dir) if config_dir is not None else default_config_dir()
    lines = []
    for fname, doc in load_overlays(d).items():
        for name, sub in doc.items():
            seen = []
            for p, _ in _leaves(sub):
                parts = p.split(".")
                if len(parts) == 3 and parts[0] == "rgb_to_depth":      # dx.a … → dx 하나로
                    p = ".".join(parts[:2])
                if p not in seen:
                    seen.append(p)
            lines += [f"{name}.{p} ← {fname}" for p in seen]
    return lines


def has_pose(config_dir: str | os.PathLike | None = None) -> bool:
    d = Path(config_dir) if config_dir is not None else default_config_dir()
    return (d / "pose.yaml").exists()


# 저장소 맨 위 폴더 (pac_vision/src/pac_vision/config.py에서 세 단계 위). 설정 파일에서 $PINCH_KHU_ROOT로 쓴다.
REPO_ROOT = Path(__file__).resolve().parents[3]


def expand_path(p: str) -> str:
    """설정 파일의 ~ 와 환경변수를 실제 경로로. $PINCH_KHU_ROOT는 따로 정하지 않으면 이 저장소 폴더다."""
    os.environ.setdefault("PINCH_KHU_ROOT", str(REPO_ROOT))
    return os.path.expandvars(os.path.expanduser(str(p)))
