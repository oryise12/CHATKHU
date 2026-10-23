"""config 덮어쓰기 (local.yaml·pose.yaml): 병합, 파일별 허용 키, 출처, 옛 세션 호환."""
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

from pac_vision.cam2base import CameraModel
from pac_vision.config import ConfigError, config_sources, has_pose, load_config
from pac_vision.session import replay_session, compare_summaries

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = Path(__file__).resolve().parent / "data" / "sample_session.npz"


@pytest.fixture
def cfg_dir(tmp_path):
    d = tmp_path / "config"
    d.mkdir()
    for name in ("camera", "a4_layout", "frames", "thresholds", "detect"):
        shutil.copy(ROOT / "config" / f"{name}.yaml", d)
    return d


def write(d, name, doc):
    (d / name).write_text(yaml.safe_dump(doc, allow_unicode=True), encoding="utf-8")


def test_without_overlays_uses_committed_defaults(cfg_dir):
    cfg = load_config(cfg_dir)
    assert not has_pose(cfg_dir) and config_sources(cfg_dir) == []
    cam = CameraModel.from_config(cfg["camera"])
    assert cam.is_fixed_offset                               # camera.yaml은 옛 형식 기본값만
    assert "device" not in cfg["camera"]["rgb"]               # 장치 번호는 자동 탐색 (local.yaml로만 지정)


def test_pose_example_overlay_replaces_coefficients(cfg_dir):
    shutil.copy(ROOT / "config" / "pose.example.yaml", cfg_dir / "pose.yaml")
    cfg = load_config(cfg_dir)
    cam = CameraModel.from_config(cfg["camera"])
    assert cam.has_z_term and cam.dx_coef == (10.337, -0.020835, 0.025303, 13.064)
    assert cfg["camera"]["depth"]["fx"] == 570.0             # 다른 값은 그대로
    assert "camera.rgb_to_depth.dx ← pose.yaml" in config_sources(cfg_dir)
    assert load_config(cfg_dir, overlays=False)["camera"]["rgb_to_depth"]["dx"] == 23.0


def test_pose_preset_copies_as_pose_yaml(cfg_dir):
    # README 빠른 시작 4번: cp config/pose_presets/astra_unit1.yaml config/pose.yaml
    shutil.copy(ROOT / "config" / "pose_presets" / "astra_unit1.yaml", cfg_dir / "pose.yaml")
    cfg = load_config(cfg_dir)                               # 허용 키 검사 통과
    cam = CameraModel.from_config(cfg["camera"])
    assert cam.has_z_term and cam.dx_coef == (8.580472, -0.012502, 0.01603, 14.56444)
    assert cam.dy_coef == (8.475, 0.0, 0.0, 0.0)             # dy는 고정값
    assert {"camera.rgb_to_depth.dx ← pose.yaml", "camera.rgb_to_depth.dy ← pose.yaml"} <= set(config_sources(cfg_dir))


def test_local_overlay_only_machine_values(cfg_dir):
    write(cfg_dir, "local.yaml", {"camera": {"rgb": {"device": 4}, "openni": {"redist": "/opt/x"}},
                                  "detect": {"model": "/m/yolo.pt"}})
    cfg = load_config(cfg_dir)
    assert cfg["camera"]["rgb"]["device"] == 4 and cfg["camera"]["rgb"]["width"] == 640
    assert cfg["camera"]["openni"]["redist"] == "/opt/x" and cfg["detect"]["model"] == "/m/yolo.pt"
    assert cfg["detect"]["conf"] == 0.25


@pytest.mark.parametrize("fname, doc, bad", [
    ("local.yaml", {"camera": {"depth": {"fx": 547.0}}}, "camera.depth.fx"),
    ("local.yaml", {"camera": {"rgb_to_depth": {"dx": 20}}}, "camera.rgb_to_depth.dx"),
    ("local.yaml", {"thresholds": {"check": {"error_max_m": 0.05}}}, "thresholds"),
    ("pose.yaml", {"camera": {"rgb": {"device": 3}}}, "camera.rgb.device"),
    ("pose.yaml", {"camera": {"rgb_to_depth": {"dx": {"a": 1, "e": 2}}}}, "camera.rgb_to_depth.dx.e"),
    ("pose.yaml", {"rgb_to_depth": {"dx": 20}}, "rgb_to_depth"),
])
def test_disallowed_keys_stop_with_file_and_key(cfg_dir, fname, doc, bad):
    write(cfg_dir, fname, doc)
    with pytest.raises(ConfigError) as e:
        load_config(cfg_dir)
    assert fname in str(e.value) and bad in str(e.value)


def test_old_and_sample_sessions_replay_identically_with_overlays(cfg_dir, tmp_path):
    """세션은 계산 때의 설정 사본으로 다시 계산하므로 local·pose가 있어도 같은 결과."""
    shutil.copy(ROOT / "config" / "pose.example.yaml", cfg_dir / "pose.yaml")
    write(cfg_dir, "local.yaml", {"camera": {"rgb": {"device": 9}}})
    res, s = replay_session(SAMPLE)
    assert compare_summaries(res.summary(), s["summary"]) < 1e-6
    # --use-current-config: 덮어쓴 현재 설정 = 세션 때 계수(같은 값)라 결과도 같음
    cur = load_config(cfg_dir)["camera"]
    res2, _ = replay_session(SAMPLE, camera_cfg=cur)
    assert compare_summaries(res2.summary(), s["summary"]) < 1e-6
    # 옛 형식(고정 dx·dy) 설정 사본을 가진 세션
    import synth
    from test_session import make_session
    path, res3, _ = make_session(tmp_path)
    res4, s4 = replay_session(path)
    assert compare_summaries(res4.summary(), s4["summary"]) < 1e-9
    assert np.isscalar(s4["configs"]["camera"]["rgb_to_depth"]["dx"])
    assert synth.CAMERA_CFG["rgb_to_depth"]["dx"] == 25.0
