"""Headless path-edit scenarios through the SAME pixel -> ray -> plane -> editor route as the mouse.

  edit_ok     : initial path PASS, drag a mid point by display pixels -> PASS dropped, S V Y on the
                edited path -> DONE; executed TCP compared with the edited and the initial path
  edit_fail   : drag the point next to the approach point down below the floor -> FAILED (reasons), Y refused,
                O (reset to original) -> S V Y -> DONE
Snapshots (side_cam and a free camera) go to logs/edit_demo/, results to logs/edit_demo.json.
`--approve-in-test` is required for Y (app.py has no automatic approval).
"""
from __future__ import annotations

import argparse
import json
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from pinch_sim.app import MouseEditor  # noqa: E402
from pinch_sim.camera_model import CameraModel  # noqa: E402
from pinch_sim.monitor import SimFrameSource, draw_overlay_2d  # noqa: E402
from pinch_sim.model import ROOT, load_config, load_scene, ready_q  # noqa: E402
from pinch_sim.overlay import add_overlay, build_overlay  # noqa: E402
from pinch_sim.paths import polyline_distance  # noqa: E402
from pinch_sim.session import Session  # noqa: E402

OUT = ROOT / "logs/edit_demo"


class Demo:
    def __init__(self, cfg, name, approve, scale=0.85):
        self.cfg, self.name, self.approve = cfg, name, approve
        self.m, self.idx, sha = load_scene(cfg)
        self.d = mujoco.MjData(self.m)
        self.d.qpos[:] = ready_q(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.s = Session(self.m, self.idx, sha, cfg, self.d, threaded=False, log_path=OUT / f"{name}.jsonl")
        W, H = cfg["cameras"]["width_px"], cfg["cameras"]["height_px"]
        self.cam = CameraModel.from_mujoco(self.m, self.d, "side_cam", W, H)
        self.mouse = MouseEditor(self.cam, cfg, scale)
        self.r = mujoco.Renderer(self.m, H, W)
        self.src = SimFrameSource(self.m, self.r, "side_cam")
        self.n = 0

    def shot(self, tag):
        ov = build_overlay(self.s, self.d)
        self.src.prepare(self.d)                                   # monitor: frame + 2-D graph
        cv2.imwrite(str(OUT / f"{self.name}_{self.n:02d}_{tag}_monitor.png"),
                    draw_overlay_2d(self.src.frame(), self.cam, ov))
        c = mujoco.MjvCamera()                                     # free 3-D view (MuJoCo viewer style)
        c.lookat[:], c.distance, c.azimuth, c.elevation = [0.05, 0.25, 0.12], 0.9, 150, -35
        self.r.update_scene(self.d, camera=c)
        add_overlay(self.r.scene, ov)
        cv2.imwrite(str(OUT / f"{self.name}_{self.n:02d}_{tag}_free.png"), cv2.cvtColor(self.r.render(), cv2.COLOR_RGB2BGR))
        self.n += 1

    def key(self, k):
        if k == "Y" and not self.approve:
            raise SystemExit("refusing to press Y without --approve-in-test")
        self.s.handle_key(k, self.d)

    def run(self):
        self.key("Y")
        while self.s.state == "EXECUTING":
            self.s.physics_step(self.d)

    def mouse_drag(self, frac, dx, dy, steps=10, shot_mid=False):
        x0, y0 = self.mouse.path_point_display(self.s, self.d, frac)
        ok = self.mouse.press(self.s, self.d, x0, y0)
        for i in range(1, steps + 1):
            self.mouse.move(self.s, self.d, x0 + dx * i / steps, y0 + dy * i / steps)
            if shot_mid and i == steps // 2:
                self.shot("dragging")
        kept = self.s.edit_end()
        return {"press_px": [round(x0, 1), round(y0, 1)], "move_px": [dx, dy], "picked": ok, "kept": kept,
                "message": self.s.message}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approve-in-test", action="store_true")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    r_mid = round((cfg["target"]["r_min_m"] + cfg["target"]["r_max_m"]) / 2, 3)
    out = {}

    D = Demo(cfg, "edit_ok", a.approve_in_test)
    D.s.cmd_fixed_target(r_mid, 10.0, D.d)
    base = D.s.path
    D.key("S")
    D.key("V")
    row = {"initial_validation": D.s.report.summary()}
    D.shot("initial_passed")
    row["drag"] = D.mouse_drag(0.45, 0, -40, shot_mid=True)          # mouse up -> point raised
    edited = D.s.path
    row["after_drag_state"], row["report_after_drag"] = D.s.state, D.s.report
    row["Y_right_after_edit"] = D.s.cmd_approve(D.d)
    xi = D.s.edit.editor.paths["outbound"].xi
    xyref = np.column_stack([np.interp(xi, D.s.edit.xi, base.points_m[:, i]) for i in (0, 1)])
    zref = np.interp(xi, D.s.edit.xi, base.points_m[:, 2])
    row["max_dxy_m"] = float(np.max(np.abs(edited.points_m[:, :2] - xyref)))
    row["max_dz_m"] = float(np.max(np.abs(edited.points_m[:, 2] - zref)))
    row["start_end_unchanged"] = bool(np.array_equal(edited.points_m[0], base.points_m[0])
                                      and np.array_equal(edited.points_m[-1], base.points_m[-1]))
    row["max_xy_change_m"] = float(np.max(polyline_distance(edited.points_m, base.points_m)))
    D.shot("edited")
    D.key("S")
    D.key("V")
    row["edited_validation"] = D.s.report.summary() if D.s.report else D.s.message
    D.shot("edited_" + D.s.state)
    if D.s.state == "PASSED":
        D.run()
        row["run"] = {k: D.s.last_run[k] for k in ("status", "path_source", "path_dev_max_m", "final_tcp_err_m",
                                                   "max_q_err_rad", "contact_steps", "saturation_steps")}
        D.shot("edited_" + D.s.state)
    out["edit_ok"] = row

    D = Demo(cfg, "edit_fail", a.approve_in_test)
    D.s.cmd_fixed_target(r_mid, 10.0, D.d)
    base = D.s.path
    k = base.waypoint_index[1] - 3
    frac = k / (len(base.points_m) - 1)
    # pixel offset that moves the point 0.15 m straight down (below the floor)
    p = base.points_m[k]
    uv0, uv1 = D.cam.project(np.vstack([p, p + [0.0, 0.0, -0.15]]))[0]
    dx, dy = (uv1 - uv0) * 0.85
    row = {"drag": D.mouse_drag(frac, float(dx), float(dy))}
    D.key("S")
    D.key("V")
    row["validation"] = D.s.report.summary() if D.s.report else D.s.message
    row["blocking"] = D.s.report.blocking() if D.s.report else None
    D.shot("fail_" + D.s.state)
    row["Y_after_fail"] = D.s.cmd_approve(D.d)
    row["state_after_Y"] = D.s.state
    D.key("O")
    row["reset_restores_initial"] = D.s.path.fingerprint == base.fingerprint
    D.key("S")
    D.key("V")
    row["after_reset_validation"] = D.s.report.summary() if D.s.report else D.s.message
    if D.s.state == "PASSED":
        D.run()
        row["after_reset_run"] = {k: D.s.last_run[k] for k in ("status", "path_source", "path_dev_max_m")}
    out["edit_fail"] = row
    (ROOT / "logs/edit_demo.json").write_text(json.dumps(out, indent=1, ensure_ascii=False, default=str))
    print(json.dumps(out, indent=1, ensure_ascii=False, default=str))


if __name__ == "__main__":
    main()
