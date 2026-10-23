"""Monitor: calibrated camera model and the 2-D graph drawn on the camera frame (no 3-D path)."""
import json

import mujoco
import numpy as np

from pinch_sim.camera_model import CameraModel
from pinch_sim.model import ROOT, load_config
from pinch_sim.monitor import SimFrameSource, draw_overlay_2d
from pinch_sim.overlay import _geom, build_overlay, segment_colors
from pinch_sim.session import Session

_T = load_config()["target"]
RM = round((_T["r_min_m"] + _T["r_max_m"]) / 2, 3)


def test_sim_camera_model_matches_the_mujoco_render(scene, cfg, data):
    """A magenta ball rendered by MuJoCo lands where the OpenCV model projects it (< 0.6 px).
    Probe points come from the target band (any side_cam pose that sees the band works)."""
    m, _, _ = scene
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    cam = CameraModel.from_mujoco(m, data, "side_cam", W, H)
    cands = [[rr * np.sin(np.radians(th)), rr * np.cos(np.radians(th)), z]
             for rr in (_T["r_min_m"], RM, _T["r_max_m"]) for th in (-30, 0, 30) for z in (0.0, 0.05, 0.12)]
    uv, front = cam.project(cands)
    inside = [p for p, (u, v), f in zip(cands, uv, front) if f and 20 < u < W - 20 and 20 < v < H - 20]
    assert len(inside) >= 9                                     # side_cam sees the target band
    r = mujoco.Renderer(m, H, W)
    for p in inside[::len(inside) // 3][:3]:
        r.update_scene(data, camera="side_cam")
        g = _geom(r.scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.004, 0, 0], p, (1, 0, 1, 1))
        g.emission = 1.0
        im = r.render().astype(int)
        ys, xs = np.nonzero((im[..., 0] > 200) & (im[..., 1] < 80) & (im[..., 2] > 200))
        uv = cam.project(p)[0][0]
        assert np.hypot(xs.mean() - uv[0], ys.mean() - uv[1]) < 0.6
    r.close()


def test_real_style_calibration_roundtrip_with_distortion(tmp_path):
    """Load a JSON calibration (T_world_cam form, with lens distortion): pixel -> ray hits the point."""
    c, look = np.array([0.3, -0.2, 0.6]), np.array([0.0, 0.35, 0.05])     # camera looking at the workspace
    z = (look - c) / np.linalg.norm(look - c)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)                                                     # optical: x right, y down, z forward
    T_wc = np.eye(4)
    T_wc[:3, :3], T_wc[:3, 3] = np.column_stack([x, y, z]), c
    f = tmp_path / "calib.json"
    f.write_text(json.dumps({"camera": "test", "image_size": [1280, 720],
                             "K": [[900, 0, 640], [0, 905, 360], [0, 0, 1]], "dist": [-0.12, 0.05, 0.001, -0.002, 0.0],
                             "T_world_cam": T_wc.tolist(), "units": "m"}))
    cam = CameraModel.load(f)
    pts = np.array([[0.0, 0.35, 0.05], [0.1, 0.3, 0.2], [-0.1, 0.4, 0.0]])
    uv, front = cam.project(pts)
    assert front.all() and (uv > 0).all() and (uv[:, 0] < 1280).all() and (uv[:, 1] < 720).all()
    for p, (u, v) in zip(pts, uv):
        o, d = cam.pixel_ray(u, v)
        dist = np.linalg.norm(np.cross(p - o, d))
        assert dist < 1e-6
    half = cam.scaled_to(640, 360)
    assert np.allclose(half.project(pts)[0], (uv + 0.5) / 2 - 0.5, atol=1e-9)


def test_monitor_draws_2d_graph_and_frame_has_no_3d_path(scene, cfg, data):
    m, idx, sha = scene
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    s = Session(m, idx, sha, cfg, data, threaded=False)
    s.cmd_fixed_target(RM, 10.0, data)
    ov = build_overlay(s, data)
    cam = CameraModel.from_mujoco(m, data, "side_cam", W, H)
    r = mujoco.Renderer(m, H, W)
    src = SimFrameSource(m, r, "side_cam")
    src.prepare(data)
    n_plain = r.scene.ngeom                                   # the frame's 3-D scene: robot + floor only
    frame = src.frame()
    plain = frame.copy()
    img = draw_overlay_2d(frame, cam, ov)
    # the coloured part of the path appears at its projected pixels, with its own colour
    idx_all = np.arange(len(ov.path))
    cols = segment_colors(ov, idx_all)
    uv, front = cam.project(ov.path)
    a0, a1 = ov.height_from, ov.height_until
    k = (a0 + a1) // 2
    mid = ((uv[k] + uv[k + 1]) / 2).round().astype(int)
    want = (np.array(cols[k][:3])[::-1] * 255)
    got = img[mid[1], mid[0]].astype(float)
    assert np.abs(got - want).max() < 60
    assert not np.array_equal(plain[mid[1], mid[0]], img[mid[1], mid[0]])
    # the 3-D scene used for the frame has no planner geoms; drawing them in 3-D would add > 50
    from pinch_sim.overlay import add_overlay
    r.update_scene(data, camera="side_cam")
    assert r.scene.ngeom == n_plain
    add_overlay(r.scene, ov)
    assert r.scene.ngeom > n_plain + 50
    r.close()
