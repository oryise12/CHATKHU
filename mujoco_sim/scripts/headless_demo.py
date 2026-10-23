"""Headless end-to-end runs through the same Session used by the GUI (MUJOCO_GL=egl).

Scenarios (all in one process, one simulation each):
  fixed   : target r=mid of configured band, th=0 -> S -> V -> Y -> DONE -> H -> S -> V -> Y -> DONE
  seeded  : --seed S, --count K random targets, each S V Y then H S V Y
  failure : work_height -0.02 with --allow-unsafe-test-geometry -> S -> V -> FAILED -> Y refused
            plus a drift case: PASSED, the robot is pushed, Y is refused and the report dropped.
`--approve-in-test` is required to let this script press Y; app.py has no automatic approval.
Writes logs/headless_demo.json and side_cam snapshots in logs/headless/.
"""
from __future__ import annotations

import argparse
import json
import os
import time

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from pinch_sim.model import ROOT, load_config, load_scene, ready_q, with_overrides  # noqa: E402
from pinch_sim.overlay import add_overlay, build_overlay  # noqa: E402
from pinch_sim.session import Session  # noqa: E402


class Runner:
    def __init__(self, cfg, name, seed=0, allow_unsafe=False, approve=False):
        self.m, self.idx, sha = load_scene(cfg)
        self.d = mujoco.MjData(self.m)
        self.d.qpos[:] = ready_q(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.s = Session(self.m, self.idx, sha, cfg, self.d, seed=seed, allow_unsafe=allow_unsafe, threaded=False,
                         log_path=ROOT / f"logs/headless/{name}.jsonl")
        self.name, self.approve, self.shots = name, approve, []
        self.r = mujoco.Renderer(self.m, cfg["cameras"]["height_px"], cfg["cameras"]["width_px"])

    def step(self, seconds):
        for _ in range(int(round(seconds / self.m.opt.timestep))):
            self.s.physics_step(self.d)
        self.s.watch(self.d)

    def key(self, k):
        if k == "Y" and not self.approve:
            raise SystemExit("refusing to press Y without --approve-in-test")
        self.s.handle_key(k, self.d)
        self.step(0.05)

    def run_until_idle(self, limit_s=30.0):
        t0 = self.d.time
        while self.s.state == "EXECUTING" and self.d.time - t0 < limit_s:
            self.step(0.1)

    def shot(self, tag):
        self.r.update_scene(self.d, camera="side_cam")
        add_overlay(self.r.scene, build_overlay(self.s, self.d))
        p = ROOT / f"logs/headless/{self.name}_{len(self.shots):02d}_{tag}.png"
        cv2.imwrite(str(p), cv2.cvtColor(self.r.render(), cv2.COLOR_RGB2BGR))
        self.shots.append(str(p.relative_to(ROOT)))

    def cycle(self, label):
        """S V (Y run) for the current GENERATED path; returns a result row."""
        row = {"label": label, "generated": self.s.state == "GENERATED"}
        if self.s.state != "GENERATED":
            row["message"] = self.s.message
            return row
        row["start_tcp_m"] = self.s.path.points_m[0].round(6).tolist()
        row["fk_now_m"] = self.s.kin.fk(self.d.qpos[self.idx.qpos]).p_m.round(6).tolist()
        self.key("S")
        self.key("V")
        row["validation"] = self.s.report.summary() if self.s.report else self.s.message
        row["state_after_V"] = self.s.state
        self.shot(f"{label}_{self.s.state}")
        if self.s.state == "PASSED":
            self.key("Y")
            self.run_until_idle()
            row["state_after_run"] = self.s.state
            row["run"] = self.s.last_run
            self.shot(f"{label}_{self.s.state}")
        else:
            self.key("Y")
            row["Y_after_fail_state"] = self.s.state
            row["Y_message"] = self.s.message
            row["blocking"] = self.s.events[-2].data.get("report", {}).get("blocking") if len(self.s.events) > 1 \
                else None
        return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--count", type=int, default=3)
    ap.add_argument("--approve-in-test", action="store_true")
    a = ap.parse_args()
    (ROOT / "logs/headless").mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    R_FIX = round((cfg["target"]["r_min_m"] + cfg["target"]["r_max_m"]) / 2, 3)
    out = {"started": time.strftime("%Y-%m-%dT%H:%M:%S"), "mujoco": mujoco.__version__}
    t0 = time.time()

    # fixed target ------------------------------------------------------------------------------
    R = Runner(cfg, "fixed", approve=a.approve_in_test)
    R.step(0.5)
    R.s.cmd_fixed_target(R_FIX, 0.0, R.d)
    R.shot("generated")
    rows = [R.cycle("to_target")]
    R.step(0.5)
    R.key("H")
    rows.append(R.cycle("return"))
    rows[-1]["ready_err_m"] = float(np.linalg.norm(R.s.kin.fk(R.d.qpos[R.idx.qpos]).p_m - R.s.ready_pose.p_m))
    out["fixed"] = rows

    # seeded random targets, then a second session with the same seed must reproduce them ---------
    R = Runner(cfg, f"seed{a.seed}", seed=a.seed, approve=a.approve_in_test)
    seeded = []
    for i in range(a.count):
        R.key("N")
        tgt = R.s.target.summary() if R.s.target else None
        row = R.cycle(f"trial{i}")
        row["target"] = tgt
        seeded.append(row)
        R.key("H")
        seeded.append(R.cycle(f"trial{i}_return"))
    out["seeded"] = seeded
    R2 = Runner(cfg, f"seed{a.seed}_repeat", seed=a.seed)
    repeat = []
    for i in range(a.count):
        R2.s.cmd_new_target(R2.d)
        repeat.append(R2.s.target.summary() if R2.s.target else None)
    first = [r["target"] for r in seeded if "target" in r]
    out["seed_reproducible"] = all(
        x is not None and y is not None and x["floor_point_m"] == y["floor_point_m"] for x, y in zip(first, repeat))
    out["seed_targets"] = [{"trial": t["trial"], "draw": t["draw"], "r_m": t["r_m"], "theta_deg": t["theta_deg"],
                            "floor_point_m": t["floor_point_m"]} for t in repeat if t]

    # failure: TCP goal below the floor --------------------------------------------------------------
    bad = with_overrides(cfg, target={"work_height_m": -0.02})
    R = Runner(bad, "failure", allow_unsafe=True, approve=a.approve_in_test)
    R.s.cmd_fixed_target(R_FIX, 0.0, R.d)
    out["failure"] = R.cycle("below_floor")
    out["failure"]["moved"] = float(np.max(np.abs(R.d.qpos[R.idx.qpos] - ready_q(R.m)[R.idx.qpos])))

    # drift: validated, then pushed before approval -----------------------------------------------
    R = Runner(cfg, "drift", approve=a.approve_in_test)
    R.s.cmd_fixed_target(R_FIX, 0.0, R.d)
    R.key("S")
    R.key("V")
    st_pass = R.s.state
    body = R.m.site_bodyid[R.idx.tcp_site]
    R.d.xfrc_applied[body, :3] = [3.0, 0, 0]
    for _ in range(300):
        R.s.physics_step(R.d)
    R.d.xfrc_applied[body, :3] = 0
    pushed = float(np.max(np.abs(R.d.qpos[R.idx.qpos] - R.s.report.binding.start_q_rad))) if R.s.report else None
    report_before_watch = R.s.report is not None
    R.key("Y")   # watch() inside step() may already have invalidated; either way Y must not execute
    out["drift"] = {"state_after_V": st_pass, "pushed_rad": pushed, "report_alive_before_watch": report_before_watch,
                    "state_after_Y": R.s.state, "message": R.s.message,
                    "executed": any(e.state == "EXECUTING" for e in R.s.events)}
    out["elapsed_s"] = time.time() - t0
    (ROOT / "logs/headless_demo.json").write_text(json.dumps(out, indent=1, ensure_ascii=False, default=str))
    print(json.dumps(out, indent=1, ensure_ascii=False, default=str))


if __name__ == "__main__":
    main()
