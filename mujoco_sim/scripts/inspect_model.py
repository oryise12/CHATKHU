"""Print and save (logs/inspect_model.json) what the model files actually contain.

Checks joint order/limits, actuators, collision geoms, contacts at q=0 and ready,
and a random-pose contact survey of both the original scene and the generated one
(to show that only the J1-adjacent base/r_a_1 pair is permanently in contact).
"""
from __future__ import annotations

import collections
import json
import sys

import mujoco
import numpy as np

from pinch_sim.model import ROOT, load_config, load_scene, ready_q


def survey(m, n=2000, seed=0):
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    cnt = collections.Counter()
    for _ in range(n):
        d.qpos[:] = rng.uniform(m.jnt_range[:, 0], m.jnt_range[:, 1])
        mujoco.mj_forward(m, d)
        pairs = {tuple(sorted((m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name)))
                 for c in d.contact[:d.ncon]}
        cnt.update(pairs)
    return {f"{a}--{b}": round(c / n, 4) for (a, b), c in cnt.most_common()}


def contacts_at(m, q):
    d = mujoco.MjData(m)
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    return [(m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name, round(float(c.dist), 5))
            for c in d.contact[:d.ncon]]


def main():
    cfg = load_config()
    orig = mujoco.MjModel.from_xml_path(str(ROOT / cfg["model"]["source_scene"]))
    m, idx, sha = load_scene(cfg)
    rep = {
        "mujoco": mujoco.__version__,
        "generated_sha256": sha,
        "timestep_s": m.opt.timestep,
        "integrator": mujoco.mjtIntegrator(m.opt.integrator).name,
        "gravity": m.opt.gravity.tolist(),
        "joints": [{"name": m.joint(j).name, "type": mujoco.mjtJoint(m.jnt_type[j]).name,
                    "qposadr": int(m.jnt_qposadr[j]), "limited": bool(m.jnt_limited[j]),
                    "range_rad": m.jnt_range[j].tolist(), "axis_local": m.jnt_axis[j].tolist(),
                    "damping": float(m.dof_damping[m.jnt_dofadr[j]]),
                    "frictionloss": float(m.dof_frictionloss[m.jnt_dofadr[j]]),
                    "body": m.body(m.jnt_bodyid[j]).name} for j in range(m.njnt)],
        "actuators": [{"name": m.actuator(a).name, "joint": m.joint(m.actuator_trnid[a, 0]).name,
                       "gain": mujoco.mjtGain(m.actuator_gaintype[a]).name,
                       "bias": mujoco.mjtBias(m.actuator_biastype[a]).name,
                       "gear": float(m.actuator_gear[a, 0]), "ctrlrange": m.actuator_ctrlrange[a].tolist()}
                      for a in range(m.nu)],
        "collision_geoms": [{"id": g, "body": m.body(m.geom_bodyid[g]).name,
                             "type": mujoco.mjtGeom(m.geom_type[g]).name,
                             "mesh": m.mesh(m.geom_dataid[g]).name if m.geom_dataid[g] >= 0 else None}
                            for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]],
        "sites": [m.site(i).name for i in range(m.nsite)],
        "cameras": [m.camera(i).name for i in range(m.ncam)],
        "keys": [m.key(i).name for i in range(m.nkey)],
        "excludes": [(m.body(int(s >> 16)).name, m.body(int(s & 0xFFFF)).name) for s in m.exclude_signature],
        "mesh_files": [m.mesh(i).name for i in range(m.nmesh)],
        "contacts_original_q0": contacts_at(orig, np.zeros(6)),
        "contacts_generated_q0": contacts_at(m, np.zeros(6)),
        "contacts_generated_ready": contacts_at(m, ready_q(m)),
        "survey_original_2000": survey(orig),
        "survey_generated_2000": survey(m),
    }
    (ROOT / "logs").mkdir(exist_ok=True)
    (ROOT / "logs/inspect_model.json").write_text(json.dumps(rep, indent=1, ensure_ascii=False))
    json.dump(rep, sys.stdout, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
