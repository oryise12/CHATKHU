"""Hand input without a camera: synthetic MediaPipe landmarks drive the same MouseEditor as the mouse.

Pointing (index straight, others folded) held still selects the nearest path point; a pinch near a path
point grabs it, moving the pinch drags it, opening releases it; losing the hand cancels a drag.
"""
import numpy as np
import pytest

from pinch_sim.app import MouseEditor, screen_axes
from pinch_sim.camera_model import CameraModel
from pinch_sim.hand_input import HandConfig, HandController, classify
from pinch_sim.session import Session

from test_voice_gateway import RM

CAM_W, CAM_H = 640, 480


def hand(cx, cy, pose, s=0.12):
    """21 normalized landmarks. (cx, cy) = where the cursor should be (pinch midpoint / index tip)."""
    lm = np.zeros((21, 3))
    wrist = np.array([0.0, 0.9])
    mcp = {5: (-0.25, 0.0), 9: (-0.05, -0.05), 13: (0.12, 0.0), 17: (0.28, 0.08)}
    lm[0, :2] = wrist
    lm[1, :2], lm[2, :2], lm[3, :2] = (-0.35, 0.65), (-0.5, 0.45), (-0.55, 0.25)
    for base, (x, y) in mcp.items():
        lm[base, :2] = (x, y)
        folded = pose in ("point", "pinch") and base != 5 or (pose == "pinch" and base == 5 and False)
        if base == 5:
            lm[6, :2], lm[7, :2], lm[8, :2] = (x, y - 0.3), (x, y - 0.55), (x, y - 0.8)       # index straight
        elif folded:
            lm[base + 1, :2], lm[base + 2, :2], lm[base + 3, :2] = (x, y - 0.25), (x, y - 0.05), (x, y + 0.15)
        else:
            lm[base + 1, :2], lm[base + 2, :2], lm[base + 3, :2] = (x, y - 0.3), (x, y - 0.55), (x, y - 0.8)
    if pose == "pinch":
        lm[4, :2] = lm[8, :2] + (0.03, 0.0)                                                # thumb on index tip
    else:
        lm[4, :2] = (-0.75, 0.2)                                                           # thumb out
    lm[:, :2] *= s
    lm[:, 1] *= CAM_W / CAM_H                                                              # square in pixels
    ref = (lm[4, :2] + lm[8, :2]) / 2 if pose == "pinch" else lm[8, :2]
    lm[:, :2] += np.array([cx, cy]) - ref
    return lm


def test_classify_poses():
    cfg = HandConfig()
    assert classify(hand(0.5, 0.5, "point"), CAM_W, CAM_H, False, cfg).kind == "point"
    assert classify(hand(0.5, 0.5, "pinch"), CAM_W, CAM_H, False, cfg).kind == "pinch"
    assert classify(hand(0.5, 0.5, "open"), CAM_W, CAM_H, False, cfg).kind == "open"
    g = classify(hand(0.3, 0.6, "pinch"), CAM_W, CAM_H, False, cfg)
    assert np.allclose(g.cursor, (0.3, 0.6))
    assert classify(None if False else np.full((21, 3), np.nan), CAM_W, CAM_H, False, cfg) is None


@pytest.fixture
def rig(scene, cfg, data):
    m, idx, sha = scene
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    cam = CameraModel.from_mujoco(m, data, "side_cam", W, H)
    s.screen_axes = screen_axes(cam)
    hcfg = HandConfig()
    me = MouseEditor(cam, cfg, 1.0)
    me.radius = hcfg.select_radius_px
    return s, data, cam, me, HandController(hcfg), W, H


def _run(ctrl, s, d, me, W, H, frames, t0=0.0, dt=1 / 30):
    t = t0
    for lm in frames:
        t += dt
        with s.acting("hand"):
            ctrl.step(lm, t, CAM_W, CAM_H, W, H, me, s, d)
    return t


def test_pointing_held_still_selects_without_editing(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    u, v = cam.project(s.path.points_m[k])[0][0]
    fp = s.path.fingerprint
    _run(ctrl, s, d, me, W, H, [hand(u / W, v / H, "point")] * 20)          # 0.67 s > dwell 0.4 s
    assert s.pointed is not None and s.path.fingerprint == fp
    assert np.linalg.norm(s.path.points_m[s.pointed_index()] - s.path.points_m[k]) < 1e-9


def test_pinch_grab_drag_release_edits_the_path(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    u, v = cam.project(s.path.points_m[k])[0][0]
    z0 = s.path.points_m[k, 2]
    frames = [hand(u / W, v / H, "open")] * 6 + [hand(u / W, v / H, "pinch")] * 6
    frames += [hand(u / W, (v - i * 3) / H, "pinch") for i in range(1, 16)]   # drag up 45 px
    frames += [hand(u / W, (v - 45) / H, "open")] * 6
    _run(ctrl, s, d, me, W, H, frames)
    assert s.edit.editor.can_undo and not s.edit.dragging
    assert s.path.points_m[s.pointed_index()][2] > z0 + 0.01                 # the grabbed point went up
    assert s.events[-1].actor == "hand" and s.events[-1].kind == "edit_end" and s.events[-1].data["kept"]


def test_pinch_needs_an_open_hand_first_and_loss_cancels(rig):
    s, d, cam, me, ctrl, W, H = rig
    k = s.region_index("middle")
    u, v = cam.project(s.path.points_m[k])[0][0]
    fp = s.path.fingerprint
    _run(ctrl, s, d, me, W, H, [hand(u / W, v / H, "pinch")] * 10)           # pinch without opening first
    assert not ctrl.grabbing and s.path.fingerprint == fp
    t = _run(ctrl, s, d, me, W, H, [hand(u / W, v / H, "open")] * 6 + [hand(u / W, v / H, "pinch")] * 6
             + [hand(u / W, (v - i * 3) / H, "pinch") for i in range(1, 10)])
    assert s.edit.dragging
    _run(ctrl, s, d, me, W, H, [None] * 20, t0=t)                            # hand lost for 0.67 s
    assert not s.edit.dragging and not s.edit.editor.can_undo                 # drag cancelled, nothing kept
