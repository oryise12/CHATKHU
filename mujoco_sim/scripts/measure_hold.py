"""Hold the ready pose under computed-torque control and record drift/saturation.

Also runs the uncontrolled case (ctrl = 0) for comparison: the robot must fall,
which proves the motors are torque inputs and the hold is doing the work.
"""
from __future__ import annotations

import json
import sys

import mujoco
import numpy as np

from pinch_sim.control import ComputedTorque, controlled_step
from pinch_sim.model import ROOT, load_config, load_scene, ready_q


def hold(seconds: float = 10.0) -> dict:
    cfg = load_config()
    m, idx, _ = load_scene(cfg)
    d = mujoco.MjData(m)
    q0 = ready_q(m)
    d.qpos[:] = q0
    mujoco.mj_forward(m, d)
    ctrl = ComputedTorque(m, idx, cfg["control"])
    z = np.zeros(6)
    drift, sat, tau_max, tau_ss = 0.0, 0, 0.0, None
    for _ in range(int(seconds / m.opt.timestep)):
        tau, s = controlled_step(m, d, idx, ctrl, q0, z, z)
        drift = max(drift, float(np.max(np.abs(d.qpos[idx.qpos] - q0))))
        sat += int(s)
        tau_max = max(tau_max, float(np.max(np.abs(tau))))
        tau_ss = tau
    d2 = mujoco.MjData(m)
    d2.qpos[:] = q0
    for _ in range(1000):
        mujoco.mj_step(m, d2)
    # disturbance: 2 N push on the TCP body along +X for 0.3 s, then release; ready offset step 0.02 rad
    d3 = mujoco.MjData(m)
    d3.qpos[:] = q0
    mujoco.mj_forward(m, d3)
    body = m.site_bodyid[idx.tcp_site]
    dev_push, t_rec = 0.0, None
    for k in range(3000):
        d3.xfrc_applied[body, :3] = [2.0, 0, 0] if k < 300 else 0.0
        controlled_step(m, d3, idx, ctrl, q0, z, z)
        e = float(np.max(np.abs(d3.qpos[idx.qpos] - q0)))
        dev_push = max(dev_push, e)
        if k >= 300 and t_rec is None and e < 1e-3:
            t_rec = (k - 300) * m.opt.timestep
    d4 = mujoco.MjData(m)
    d4.qpos[:] = q0 + 0.02
    mujoco.mj_forward(m, d4)
    overshoot, t_settle = 0.0, None
    for k in range(2000):
        controlled_step(m, d4, idx, ctrl, q0, z, z)
        e = d4.qpos[idx.qpos] - q0
        overshoot = max(overshoot, float(np.max(-e)))
        if t_settle is None and np.max(np.abs(e)) < 1e-3:
            t_settle = (k + 1) * m.opt.timestep
    return {"seconds": seconds, "max_drift_rad": drift, "saturation_steps": sat, "max_abs_tau_nm": tau_max,
            "steady_tau_nm": np.round(tau_ss, 4).tolist(), "contacts_end": int(d.ncon),
            "push_2N_0.3s_max_dev_rad": dev_push, "push_recover_to_1e-3_s": t_rec,
            "step_0.02rad_settle_to_1e-3_s": t_settle, "step_overshoot_rad": overshoot,
            "uncontrolled_1s_max_dev_rad": float(np.max(np.abs(d2.qpos[idx.qpos] - q0)))}


if __name__ == "__main__":
    rep = hold(float(sys.argv[1]) if len(sys.argv) > 1 else 10.0)
    (ROOT / "logs/measure_hold.json").write_text(json.dumps(rep, indent=1))
    print(json.dumps(rep, indent=1))
