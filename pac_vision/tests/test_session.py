"""세션 저장 → 읽기 → 재계산(--replay)이 같은 결과를 내는지, cam2base.yaml 왕복."""
from pathlib import Path

import numpy as np
import pytest

from pac_vision.cam2base import layout_points, solve_calibration
from pac_vision.session import (compare_summaries, load_cam2base, replay_session, save_session,
                                write_cam2base)
import synth

SAMPLE = Path(__file__).resolve().parent / "data" / "sample_session.npz"


def make_session(tmp_path):
    cfg = synth.scene_config()
    configs = {"camera": synth.CAMERA_CFG, "a4_layout": cfg["a4_layout"], "thresholds": cfg["thresholds"]}
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    depth = synth.render_depth(cam, R, t, noise_mm=0.5)
    clicks = synth.clicks_for(cam, R, t, layout_points(cfg["a4_layout"]))
    res = solve_calibration(depth, clicks, cam, cfg["a4_layout"], cfg["thresholds"])
    rgb = np.zeros((480, 640, 3), np.uint8)
    path = save_session(tmp_path / "s.npz", rgb, depth, clicks, (640, 480), "plane", configs, res.summary())
    return path, res, configs


def test_replay_matches_saved_result(tmp_path):
    path, res, _ = make_session(tmp_path)
    res2, s = replay_session(path)
    assert compare_summaries(res2.summary(), s["summary"]) < 1e-9
    assert compare_summaries(res2.summary(), res.summary()) < 1e-9


def test_cam2base_yaml_round_trip(tmp_path):
    _, res, configs = make_session(tmp_path)
    p = write_cam2base(tmp_path / "cam2base.yaml", res, configs["camera"], session_file="s.npz")
    conv = load_cam2base(p)
    assert np.allclose(conv.R, res.R, atol=0, rtol=0)
    assert np.allclose(conv.t, res.t, atol=0, rtol=0)
    assert conv.info["depth_mode"] == "plane" and conv.info["passed"]


@pytest.mark.skipif(not SAMPLE.exists(), reason="실제 세션 샘플은 캘리브레이션 단계에서 추가")
def test_sample_session_replays_identically():
    res, s = replay_session(SAMPLE)
    assert compare_summaries(res.summary(), s["summary"]) < 1e-6


def test_detect_and_check_refuse_without_calibration(tmp_path):
    """cam2base.yaml이 없거나 예시 파일이면 실행 거부 (예시 숫자 오용 방지)."""
    from pac_vision.session import MissingCalibration, require_cam2base
    with pytest.raises(MissingCalibration, match="먼저 calibrate.py를 실행하세요"):
        require_cam2base(tmp_path / "cam2base.yaml")
    example = Path(__file__).resolve().parents[1] / "config" / "cam2base.example.yaml"
    assert example.exists()
    with pytest.raises(MissingCalibration, match="먼저 calibrate.py를 실행하세요"):
        require_cam2base(example)


def test_replay_with_current_camera_config(tmp_path):
    """--use-current-config: 같은 클릭·depth에 다른 dx를 넣으면 결과가 바뀌고, 같은 dx면 그대로."""
    import copy
    path, res, configs = make_session(tmp_path)
    same, _ = replay_session(path, camera_cfg=configs["camera"])
    assert compare_summaries(same.summary(), res.summary()) < 1e-9
    cam2 = copy.deepcopy(configs["camera"])
    cam2["rgb_to_depth"]["dx"] += 5
    moved, _ = replay_session(path, camera_cfg=cam2)
    assert np.max(np.abs(moved.clicks_depth[:, 0] - res.clicks_depth[:, 0] - 5)) < 1e-9
    assert compare_summaries(moved.summary(), res.summary()) > 1e-4
