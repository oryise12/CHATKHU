"""Real camera mode: minimal overlay (path, A, B, axes only), one shared camera feed, hover highlight."""
import time

import cv2
import numpy as np
import pytest

from pinch_sim.app import MouseEditor
from pinch_sim.camera_model import CameraModel
from pinch_sim.hand_input import HandConfig, HandController
from pinch_sim.monitor import draw_overlay_2d
from pinch_sim.overlay import build_overlay
from pinch_sim.session import Session

from test_hand_input import CAM_H, CAM_W, hand
from test_voice_gateway import RM


@pytest.fixture
def sess(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    return s, CameraModel.from_mujoco(m, data, "side_cam", 768, 480)


def test_minimal_overlay_draws_only_the_path(sess, data):
    s, cam = sess
    ov = build_overlay(s, data)
    full = draw_overlay_2d(np.full((480, 768, 3), 255, np.uint8), cam, ov)
    mini = draw_overlay_2d(np.full((480, 768, 3), 255, np.uint8), cam, ov, minimal=True)
    u, v = np.round(cam.project(ov.floor_point)[0][0]).astype(int)   # target ring: gone
    assert (full[v, u] != 255).any() and (mini[v, u] == 255).all()
    u, v = np.round(cam.project(ov.tcp)[0][0]).astype(int)           # simulated TCP dot (orange): gone
    tcp_bgr = np.array([0, 128, 255])
    assert (full[v, u] == tcp_bgr).all() and not (mini[v, u] == tcp_bgr).all()
    u, v = np.round(cam.project(s.path.points_m[len(s.path.points_m) // 2])[0][0]).astype(int)
    assert (mini[v, u] != 255).any()                       # the path is still drawn


def test_shared_feed_from_a_video_file(tmp_path):
    pytest.importorskip("mediapipe")
    from pinch_sim.hand_input import HandCamera
    f = tmp_path / "cam.avi"
    w = cv2.VideoWriter(str(f), cv2.VideoWriter_fourcc(*"MJPG"), 30, (320, 240))
    for i in range(10):
        w.write(np.full((240, 320, 3), 20 * i, np.uint8))
    w.release()
    cam = HandCamera(HandConfig(mirror=False), device=str(f), set_size=False)
    try:
        t_end = time.monotonic() + 5
        while cam.frame() is None and time.monotonic() < t_end:
            time.sleep(0.02)
        frame = cam.frame()
        lm, t, fw, fh, _ = cam.latest()
        assert frame.shape == (240, 320, 3) and (fw, fh) == (320, 240) and lm is None   # no hand in it
    finally:
        cam.close()


def test_hover_rings_the_point_the_finger_would_take(sess, cfg, data):
    s, cam = sess
    me = MouseEditor(cam, cfg, 1.0)
    me.radius = 32
    ctrl = HandController(HandConfig(mirror=False))
    k = len(s.path.points_m) // 3
    u, v = cam.project(s.path.points_m[k])[0][0]
    ctrl.step(hand((u + 6) / 768, (v + 4) / 480, "point"), 0.0, CAM_W, CAM_H, 768, 480, me, s, data)
    assert ctrl.hover is None                              # outside edit mode the hand offers nothing
    s.set_edit_mode(True, "test")
    for i in range(3):
        ctrl.step(hand((u + 6) / 768, (v + 4) / 480, "point"), i / 30, CAM_W, CAM_H, 768, 480, me, s, data)
    uv = cam.project(s.path.points_m)[0]
    nearest = int(np.argmin(np.hypot(uv[:, 0] - (u + 6), uv[:, 1] - (v + 4))))
    assert ctrl.hover is not None and np.allclose(ctrl.hover, s.path.points_m[nearest])   # the closest one
    for i in range(3):                                     # far from the path: no hover
        ctrl.step(hand(0.02, 0.05, "point"), 1 + i / 30, CAM_W, CAM_H, 768, 480, me, s, data)
    assert ctrl.hover is None
