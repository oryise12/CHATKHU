"""Print the seeded target sequence the app's N key will produce (same precheck, no GUI).

Usage: python scripts/sample_targets.py --seed 7 --count 10
"""
import argparse

import numpy as np

from pinch_sim.collision import CollisionChecker
from pinch_sim.kinematics import Kinematics
from pinch_sim.model import load_config, load_scene, ready_q
from pinch_sim.targets import sample_target


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--count", type=int, default=10)
    a = ap.parse_args()
    cfg = load_config()
    m, idx, _ = load_scene(cfg)
    kin, col = Kinematics(m, idx, cfg["ik"]), CollisionChecker(m, idx)
    q = ready_q(m)[idx.qpos]
    print("trial draw    r[m]  theta[deg]  floor_point[m]            tcp_goal[m]")
    for k in range(a.count):
        res = sample_target(a.seed, k, cfg, kin, col, q)
        t = res.target
        if t is None:
            print(f"{k:5d}  -  {res.message}")
            continue
        print(f"{k:5d} {t.draw:4d}  {t.r_m:.4f}  {np.degrees(t.theta_rad):+8.3f}  "
              f"{np.round(t.floor_point_m, 4).tolist()!s:<26}{np.round(t.tcp_goal_m, 4).tolist()}")


if __name__ == "__main__":
    main()
