"""Headless snapshots + projection check (MUJOCO_GL=egl).

1. For side_cam and side_cam_alt: render the ready robot with the overlay of a fixed
   target (r=mid of configured band, theta=0) and its initial path, plus the 9 grid floor markers
   (r min/mid/max x theta -max/0/+max from configs/demo.json) -> logs/snapshots/<cam>.png. Reports whether
   every marker projects inside the image.
2. Projection accuracy: each test point is rendered alone as a small magenta sphere;
   the centroid of its pixels is compared with overlay.project() (pinhole model).
3. A free-camera "main view" snapshot with the same overlay (what the passive viewer shows).
Writes logs/render_views.json.
"""
from __future__ import annotations

import json
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from pinch_sim.kinematics import Kinematics  # noqa: E402
from pinch_sim.model import ROOT, load_config, load_scene, ready_q  # noqa: E402
from pinch_sim.overlay import Overlay, _geom, add_overlay, project  # noqa: E402
from pinch_sim.paths import initial_path  # noqa: E402
from pinch_sim.targets import make_target  # noqa: E402


def main():
    cfg = load_config()
    R_FIX = round((cfg["target"]["r_min_m"] + cfg["target"]["r_max_m"]) / 2, 3)
    m, idx, _ = load_scene(cfg)
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
    r = mujoco.Renderer(m, H, W)
    kin = Kinematics(m, idx, cfg["ik"])
    q = d.qpos[idx.qpos].copy()
    t = make_target(R_FIX, 0.0, cfg["target"])
    path = initial_path(kin.fk(q), q, t, cfg)
    ov = Overlay("GENERATED", t.floor_point_m, t.approach_m, t.tcp_goal_m, t.tool_axis, path.points_m,
                 d.site_xpos[idx.tcp_site].copy())
    grid = np.array([make_target(rr, np.radians(th), cfg["target"]).floor_point_m
                     for rr in np.linspace(cfg["target"]["r_min_m"], cfg["target"]["r_max_m"], 3)
                     for th in (-cfg["target"]["theta_max_deg"], 0, cfg["target"]["theta_max_deg"])])
    out_dir = ROOT / "logs/snapshots"
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {"width_px": W, "height_px": H, "cameras": {}}
    test_pts = np.vstack([grid, t.approach_m, t.tcp_goal_m, d.site_xpos[idx.tcp_site]])
    for cam in ("side_cam", "side_cam_alt"):
        r.update_scene(d, camera=cam)
        add_overlay(r.scene, ov)
        for p in grid:
            _geom(r.scene, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.006, 0.001, 0], p + [0, 0, 0.001], (1, 0.1, 0.6, 0.6))
        img = r.render()
        cv2.imwrite(str(out_dir / f"{cam}.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        uv = project(m, d, cam, test_pts, W, H)
        inside = bool(np.all((uv[:, 0] > 10) & (uv[:, 0] < W - 10) & (uv[:, 1] > 10) & (uv[:, 1] < H - 10)))
        errs = []
        for p, (u, v) in zip(test_pts, uv):
            r.update_scene(d, camera=cam)
            g = _geom(r.scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.004, 0, 0], p, (1, 0, 1, 1))
            g.emission = 1.0
            im = r.render().astype(int)
            mask = (im[..., 0] > 200) & (im[..., 1] < 80) & (im[..., 2] > 200)
            if mask.sum() == 0:
                errs.append(None)
                continue
            ys, xs = np.nonzero(mask)
            # pixel centres are at +0.5
            errs.append(float(np.hypot(xs.mean() + 0.5 - u, ys.mean() + 0.5 - v)))
        good = [e for e in errs if e is not None]
        report["cameras"][cam] = {
            "pos_m": m.cam_pos[m.camera(cam).id].tolist(), "quat": m.cam_quat[m.camera(cam).id].tolist(),
            "fovy_deg": float(m.cam_fovy[m.camera(cam).id]), "all_test_points_inside": inside,
            "uv_px": np.round(uv, 1).tolist(), "projection_err_px": errs,
            "projection_err_max_px": max(good) if good else None, "points_not_visible": errs.count(None)}
    # main (free camera) view, like the passive viewer's default framing
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.0, 0.2, 0.15]
    cam.distance, cam.azimuth, cam.elevation = 1.1, 135, -25
    r.update_scene(d, camera=cam)
    add_overlay(r.scene, ov)
    cv2.imwrite(str(out_dir / "main_free.png"), cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
    report["snapshots"] = sorted(str(p.relative_to(ROOT)) for p in out_dir.glob("*.png"))
    (ROOT / "logs/render_views.json").write_text(json.dumps(report, indent=1))
    print(json.dumps({c: {k: v for k, v in x.items() if k != "uv_px"} for c, x in report["cameras"].items()},
                     indent=1))


if __name__ == "__main__":
    main()
