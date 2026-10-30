"""보정 파일의 카메라 신원과 설정 사본 비교 (시뮬레이터가 다른 카메라·다른 설정의 보정을 받지 않게)."""
import copy
import os

import yaml

from pac_vision.cam2base import Cam2Base
from pac_vision.devices import identity_problems, rgb_identity
from pac_vision.session import calibration_mismatches, load_cam2base, write_cam2base
import synth


def _conv(camera_cfg):
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    from pac_vision.cam2base import CameraModel
    return Cam2Base(R, t, CameraModel.from_config(camera_cfg), info={"camera": camera_cfg})


def test_same_settings_have_no_mismatch():
    cfg = copy.deepcopy(synth.CAMERA_CFG)
    assert calibration_mismatches(_conv(cfg), cfg) == []


def test_each_setting_change_is_reported():
    base = copy.deepcopy(synth.CAMERA_CFG)
    conv = _conv(base)
    for path, value, key in ((("rgb_to_depth", "dx"), 30.0, "rgb_to_depth"), (("depth", "fx"), 580.0, "fx"),
                             (("depth", "z_offset_m"), 0.0, "z_offset_m"), (("depth", "flip"), False, "depth.flip"),
                             (("rgb", "width"), 1280, "rgb_width")):
        cur = copy.deepcopy(base)
        cur[path[0]][path[1]] = value
        assert key in calibration_mismatches(conv, cur), key


def test_identity_from_sysfs_and_v4l_links(tmp_path):
    sysfs, dev = tmp_path / "sys", tmp_path / "dev"
    (sysfs / "video2").mkdir(parents=True)
    (sysfs / "video2" / "name").write_text("Astra Pro HD Camera: Astra Pro\n")
    (dev / "by-path").mkdir(parents=True)
    (dev / "by-id").mkdir()
    target = tmp_path / "video2"
    target.touch()
    os.symlink(target, dev / "by-path" / "pci-0000:00:14.0-usb-0:2:1.0-video-index0")
    os.symlink(target, dev / "by-id" / "usb-Orbbec_Astra_Pro-video-index0")
    ident = rgb_identity(2, sysfs, dev)
    assert ident == {"device": 2, "name": "Astra Pro HD Camera: Astra Pro",
                     "by_path": "pci-0000:00:14.0-usb-0:2:1.0-video-index0",
                     "by_id": "usb-Orbbec_Astra_Pro-video-index0"}
    assert identity_problems(ident, dict(ident, device=4)) == []          # 번호만 바뀐 것은 같은 카메라
    moved = dict(ident, by_path="pci-0000:00:14.0-usb-0:1:1.0-video-index0")
    assert identity_problems(ident, moved) and "by_path" in identity_problems(ident, moved)[0]
    assert identity_problems(None, ident) == ["no_identity"]


def test_one_program_holds_the_camera(tmp_path):
    import pytest
    from pac_vision.devices import CameraBusy, camera_lock
    lock = tmp_path / "cam.lock"
    held = camera_lock("pinch_sim (simulator)", lock)
    with pytest.raises(CameraBusy, match="pinch_sim"):
        camera_lock("pac_vision calibrate.py", lock)
    held.close()                                        # the simulator exits
    camera_lock("pac_vision calibrate.py", lock).close()


def test_identity_is_written_to_cam2base(tmp_path):
    import numpy as np
    from pac_vision.cam2base import solve_calibration
    cam = synth.camera()
    R, t = synth.look_at(synth.DEFAULT_POSITION, synth.DEFAULT_TARGET)
    layout = synth.scene_config()["a4_layout"]
    from pac_vision.cam2base import layout_points
    pts = layout_points(layout)
    clicks = synth.clicks_for(cam, R, t, pts)
    res = solve_calibration(synth.render_depth(cam, R, t), clicks, cam, layout, synth.scene_config()["thresholds"])
    ident = {"device": 2, "name": "Astra Pro", "by_path": "usb-0:2", "by_id": ""}
    p = write_cam2base(tmp_path / "cam2base.yaml", res, synth.CAMERA_CFG, camera_identity=ident)
    doc = yaml.safe_load(p.read_text(encoding="utf-8"))
    assert doc["camera_identity"] == ident and doc["passed"] is True
    assert np.allclose(load_cam2base(p).R, res.R)
