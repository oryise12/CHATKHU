"""Full validation (all 12 checks incl. dry rollout) from the ready pose for a target grid.

r = target.r_min_m..r_max_m (0.01 step), theta = +-theta_max_deg (5 deg step). Saves logs/workspace_scan.json
with per-point summary and the measured distributions used to set thresholds.
Usage: python scripts/workspace_scan.py [--work-height 0.05]
"""
from __future__ import annotations

import argparse
import json
import time

import mujoco
import numpy as np

from pinch_sim.collision import CollisionChecker
from pinch_sim.kinematics import Kinematics
from pinch_sim.model import ROOT, load_config, load_scene, ready_q, with_overrides
from pinch_sim.paths import initial_path
from pinch_sim.targets import make_target, precheck
from pinch_sim.validation import Goal, PlanningContext, validate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--work-height", type=float, default=None)
    ap.add_argument("--out", default="logs/workspace_scan.json")
    a = ap.parse_args()
    cfg = load_config()
    if a.work_height is not None:
        cfg = with_overrides(cfg, target={"work_height_m": a.work_height})
    m, idx, sha = load_scene(cfg)
    ctx = PlanningContext(m, idx, sha, cfg)
    kin, col = Kinematics(m, idx, cfg["ik"]), CollisionChecker(m, idx)
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    q0 = d.qpos[idx.qpos].copy()
    rows, t0 = [], time.time()
    tc = cfg["target"]
    th_max = int(round(tc["theta_max_deg"]))
    for r in np.round(np.arange(tc["r_min_m"], tc["r_max_m"] + 1e-4, 0.01), 3):
        for th in range(-th_max, th_max + 1, 5):
            t = make_target(float(r), np.radians(th), cfg["target"])
            pre_ok, pre_reason = precheck(t, kin, col, q0, cfg)
            p = initial_path(kin.fk(q0), q0, t, cfg)
            rep = validate(ctx, p, d.qpos.copy(), d.qvel.copy(), d.time, Goal(t.tcp_goal_m, t.tool_axis))
            c = {x.kind: x for x in rep.checks}
            roll = rep.extras.get("rollout") or {}
            rows.append({
                "r_m": float(r), "theta_deg": th, "precheck": pre_reason, "summary": rep.summary(),
                "allowed": rep.execution_allowed,
                "duration_s": c["timing"].details.get("duration_s"),
                "sigma_min": c["singularity"].details.get("sigma_min"),
                "floor_clearance_min_m": c["collision"].details.get("floor_clearance_min_m"),
                "self_clearance_min_m": c["collision"].details.get("self_clearance_min_m"),
                "floor_clearance_distal_m": c["collision"].details.get("floor_clearance_distal_m"),
                "qd_max": c["joint_vel_acc"].details.get("qd_max_rad_s"),
                "qdd_max": c["joint_vel_acc"].details.get("qdd_max_rad_s2"),
                "tau_max": max(c["torque"].details.get("tau_max_nm", [np.nan])),
                "spline_dev_m": c["continuous_ik"].details.get("spline_tcp_dev_max_m"),
                "roll_max_q_err": roll.get("max_q_err_rad"), "roll_max_tcp_err": roll.get("max_tcp_err_m"),
                "roll_final_tcp_err": roll.get("final_tcp_err_m"), "roll_rms_q_err": roll.get("rms_q_err_rad"),
            })
            print(f"r={r:.2f} th={th:+3d} {rep.summary():<40} pre={pre_reason}", flush=True)

    def stat(key):
        v = np.array([x[key] for x in rows if x[key] is not None], float)
        return {"min": float(v.min()), "max": float(v.max()), "mean": float(v.mean())} if len(v) else None

    out = {"work_height_m": cfg["target"]["work_height_m"], "points": len(rows),
           "allowed": sum(x["allowed"] for x in rows), "precheck_ok": sum(x["precheck"] == "ok" for x in rows),
           "elapsed_s": time.time() - t0,
           "stats": {k: stat(k) for k in ("duration_s", "sigma_min", "floor_clearance_min_m", "floor_clearance_distal_m",
                                          "self_clearance_min_m",
                                          "qd_max", "qdd_max", "tau_max", "spline_dev_m", "roll_max_q_err",
                                          "roll_max_tcp_err", "roll_final_tcp_err", "roll_rms_q_err")},
           "failures": [x for x in rows if not x["allowed"]], "rows": rows}
    (ROOT / a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}, indent=1))


if __name__ == "__main__":
    main()
