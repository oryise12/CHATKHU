"""scripts/check_setup.py 중 카메라 없이 도는 점검."""
import shutil
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import check_setup as cs  # noqa: E402


def test_versions_list_core_packages():
    names = [i[1] for i in cs.check_versions()]
    assert names[0] == "Python" and "numpy" in names and "PyYAML" in names and "rclpy" in names
    assert next(i for i in cs.check_versions() if i[1] == "numpy")[0]


def test_git_reports_branch_and_config_changes():
    items = cs.check_git(ROOT)
    assert items[0][1] == "git"
    if items[0][0]:                                           # git 저장소 안에서 실행할 때
        assert "브랜치" in items[0][2] and items[1][1] == "추적되는 config 로컬 수정"


def test_configs_flag_bad_overlay_and_missing_pose(tmp_path):
    d = tmp_path / "config"
    d.mkdir()
    for name in ("camera", "a4_layout", "frames", "thresholds", "detect"):
        shutil.copy(ROOT / "config" / f"{name}.yaml", d)
    items = {i[1]: i for i in cs.check_configs(d)}
    assert items["config 파일"][0] and items["local.yaml / pose.yaml 키 검사"][0]
    assert not items["pose.yaml (이 설치 자세의 정합 계수)"][0]
    (d / "local.yaml").write_text(yaml.safe_dump({"camera": {"depth": {"fx": 1.0}}}), encoding="utf-8")
    items = {i[1]: i for i in cs.check_configs(d)}
    bad = items["local.yaml / pose.yaml 키 검사"]
    assert not bad[0] and "camera.depth.fx" in bad[2]


def test_run_without_camera_and_without_calibration(tmp_path):
    d = tmp_path / "config"
    d.mkdir()
    for name in ("camera", "a4_layout", "frames", "thresholds", "detect"):
        shutil.copy(ROOT / "config" / f"{name}.yaml", d)
    items = {i[1]: i for i in cs.run(d, camera=False)}
    assert not items["cam2base.yaml (보정 결과)"][0]
    assert "RGB 열기" not in items
