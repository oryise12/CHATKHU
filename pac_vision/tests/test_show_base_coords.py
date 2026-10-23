"""scripts/show_base_coords.py (클릭 → base 좌표 확인 화면) 중 카메라 없이 도는 부분."""
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

from pac_vision.base_coords import click_coords, desk_extent, grid_polylines
from pac_vision.cam2base import Cam2Base, Plane, intersect_ray_plane
import synth

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "show_base_coords.py"
SAMPLE = ROOT / "tests" / "data" / "sample_session.npz"
sys.path.insert(0, str(ROOT / "scripts"))
import calibrate  # noqa: E402
import show_base_coords as sbc  # noqa: E402

RGB_SIZE = (640, 480)
CYL = (0.30, 0.0, 0.05, 0.06)       # 원기둥 (xc, yc, 반지름, 높이): z > 0 점용


def scene():
    """거리 식 정합 + 비스듬한 카메라 + 원기둥 하나. (conv, depth)"""
    cam = synth.camera(synth.DIST_RGB_TO_DEPTH)
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    depth = synth.render_depth(cam, R, t, cylinders=[CYL])
    return Cam2Base(R, t, cam), depth


def known_point(conv, near_base, z):
    """near_base 근처에서 depth 픽셀이 정수인 점 (높이 z 평면 위). depth 패치 중앙값이 그 점의 값과 같아지게."""
    ud, vd = conv.cam.project(conv.base_to_camera(np.asarray(near_base, float)))
    ud, vd = np.round(ud), np.round(vd)
    plane = Plane(normal=conv.R[2].copy(), d=float(conv.t[2]) - z, rms_m=0.0, inlier_ratio=1.0, n_points=0)
    return conv.camera_to_base(intersect_ray_plane(conv.cam.ray(ud, vd), plane)[0])


def test_desk_and_raised_points_recovered_within_1mm():
    conv, depth = scene()
    desk = [known_point(conv, p, 0.0) for p in ([0, 0, 0], [0.1485, -0.105, 0], [-0.1485, 0.315, 0], [0.05, 0.2, 0])]
    top = [known_point(conv, [CYL[0], CYL[1], CYL[3]], CYL[3])]
    for b in desk + top:
        u, v = conv.base_to_pixel(b, rgb_size=RGB_SIZE)
        cc = click_coords(conv, u, v, depth, half=2, rgb_size=RGB_SIZE)
        assert cc.d_ok, cc.d_reason
        assert np.linalg.norm(cc.d - b) < 1e-3, (b, cc.d)
        if b[2] == 0.0:
            assert cc.p_ok and np.linalg.norm(cc.p - b) < 1e-3, (b, cc.p)
    # 높이가 있는 점에서 P는 책상면 교점이라 맞지 않는다 (D를 따로 보여 주는 이유)
    u, v = conv.base_to_pixel(top[0], rgb_size=RGB_SIZE)
    assert np.linalg.norm(click_coords(conv, u, v, depth, 2, RGB_SIZE).p[:2] - top[0][:2]) > 0.02


def test_p_equals_first_step_of_check(monkeypatch):
    """P = calibrate.py --check '다른 점 클릭'이 첫 단계로 부르는 pixel_to_desk 결과 (평면 재맞춤 전)."""
    conv, depth = scene()
    calls = []
    orig = conv.pixel_to_desk

    def spy(*a, **k):
        out = orig(*a, **k)
        calls.append((k.get("plane"), out[0].copy()))
        return out

    monkeypatch.setattr(conv, "pixel_to_desk", spy)
    thr = synth.repo_config()["thresholds"]
    for u, v in ((300, 300), (420, 360), (250, 200)):
        calls.clear()
        p_check = calibrate.measure_desk_point(conv, depth, (u, v), RGB_SIZE, thr)
        first_plane, first_out = calls[0]
        assert first_plane is None                          # 첫 단계 = 보정 책상면
        cc = click_coords(conv, u, v, depth, 2, RGB_SIZE)
        assert np.array_equal(cc.p, first_out[0])
        assert cc.p[2] == 0.0 or abs(cc.p[2]) < 1e-9
        # 평평한 합성 책상에서는 --check의 최종값(주변 평면 재맞춤)도 거의 같다
        assert p_check is not None and np.linalg.norm(p_check - cc.p) < 1e-3


def _config_dir(tmp_path):
    """커밋된 설정만 복사한 폴더 (cam2base.yaml·pose.yaml 없음)."""
    d = tmp_path / "config"
    d.mkdir()
    for name in ("camera", "a4_layout", "frames", "thresholds", "detect"):
        shutil.copy(ROOT / "config" / f"{name}.yaml", d)
    return d


def _run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True, timeout=120)


def test_live_mode_refuses_without_cam2base(tmp_path):
    r = _run("--config-dir", str(_config_dir(tmp_path)))
    assert r.returncode != 0
    assert "보정 파일이 없습니다" in r.stderr and "먼저 calibrate.py를 실행하세요" in r.stderr


def test_sample_session_no_gui_prints_coords(tmp_path):
    d = _config_dir(tmp_path)                       # cam2base.yaml 없이도 세션 자체 보정으로 실행
    r = _run("--config-dir", str(d), "--session", str(SAMPLE), "--no-gui", "--click", "320,240", "--click", "400,300")
    assert r.returncode == 0, r.stderr
    assert str(SAMPLE.resolve()) in r.stdout and "검사: 기존 기준 통과" in r.stdout
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("[점 ")]
    assert len(lines) == 2 and all("P (" in ln for ln in lines)
    assert "D (" in lines[1]


def test_check_text_uses_recorded_numbers():
    info = {"checks": {
        "base_error": {"ok": True, "value": 0.0062, "limit": 0.01, "blocking": True},
        "a4_sides": {"ok": False, "value": 0.010381005983054192, "limit": 0.005, "blocking": True},
        "a4_size": {"ok": False, "value": 50.0, "limit": 80.0, "blocking": False}}}
    text, ok = sbc.check_text(info)
    assert not ok and text == "검사: 기존 기준 미통과 (A4 변 최대 10.4 mm > 허용 5 mm)"
    info["checks"]["a4_sides"]["ok"] = True
    assert sbc.check_text(info) == ("검사: 기존 기준 통과", True)


def test_grid_stays_inside_image_and_far_max():
    conv, _ = scene()
    far = 1.5
    ext = desk_extent(conv, RGB_SIZE, far)
    assert ext is not None and ext[0] < 0 < ext[1] and ext[2] < 0 < ext[3]
    lines = grid_polylines(conv, ext, 0.10, RGB_SIZE, far)
    assert lines
    for px in lines:
        assert np.all(px[:, 0] >= 0) and np.all(px[:, 0] <= 639) and np.all(px[:, 1] >= 0) and np.all(px[:, 1] <= 479)
    # 격자 끝점의 카메라 Z ≤ far_max
    ends = np.vstack([conv.pixel_to_desk(px[[0, -1], 0], px[[0, -1], 1], rgb_size=RGB_SIZE)[0] for px in lines])
    assert np.all(conv.base_to_camera(ends)[:, 2] <= far + 1e-3)


def test_old_config_without_demo_section_gives_guidance(tmp_path):
    """demo: 절이 없는 예전 설정 폴더 → traceback 대신 안내하고 멈춤 (카메라를 열기 전)."""
    d = _config_dir(tmp_path)
    text = (d / "thresholds.yaml").read_text(encoding="utf-8")
    (d / "thresholds.yaml").write_text(text[:text.index("\ndemo:")] + "\n", encoding="utf-8")
    r = _run("--config-dir", str(d), "--session", str(SAMPLE))
    assert r.returncode != 0 and "Traceback" not in r.stderr
    assert "demo: 절의 키 grid_step_m, max_points가 없습니다" in r.stderr
