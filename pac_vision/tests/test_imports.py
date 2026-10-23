"""핵심 모듈이 rclpy·cv2·openni 없이 import되는지, 설정 파일이 읽히는지."""
import subprocess
import sys

import synth


def test_core_does_not_import_ros_cv2_openni():
    code = ("import sys, pac_vision, pac_vision.cam2base, pac_vision.objects, pac_vision.session, "
            "pac_vision.config, pac_vision.astra_depth; "
            "print(sorted(m for m in ('rclpy', 'cv2', 'openni') if m in sys.modules))")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "[]"


def test_config_files_load():
    cfg = synth.repo_config()
    assert cfg["frames"]["base_frame"]
    assert len(cfg["a4_layout"]["corners"]) == 4
    assert cfg["camera"]["rgb"]["width"] == 640 and cfg["camera"]["rgb"]["height"] == 480
    for sec in ("placement", "capture", "plane", "calibration", "check", "objects"):
        assert sec in cfg["thresholds"]
    assert cfg["detect"]["model"] == "yolo26n.pt"
