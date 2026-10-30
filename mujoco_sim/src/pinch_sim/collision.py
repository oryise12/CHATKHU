"""Collision and clearance checks with the model's own collision geoms.

Collision geometry is the original CAD mesh, which MuJoCo collides as its
convex hull (conservative: hulls are never smaller than the mesh). The only
exclusion is base(world)-r_a_1, the two bodies joined by Revolute_1 (MuJoCo's
parent filter does not apply to children of the world body); see docs/FRAMES.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import RobotIndex


@dataclass
class CollisionResult:
    contacts: list = field(default_factory=list)   # [(body1, body2, dist_m)]
    floor_clearance_m: float = np.inf
    floor_geom_body: str = ""
    self_clearance_m: float = np.inf
    self_pair: tuple = ()
    floor_by_body: dict = field(default_factory=dict)   # body -> clearance m

    @property
    def in_contact(self) -> bool:
        return bool(self.contacts)


class CollisionChecker:
    def __init__(self, m: mujoco.MjModel, idx: RobotIndex, distmax: float = 0.2):
        self.m, self.idx, self.distmax = m, idx, distmax
        self.d = mujoco.MjData(m)
        self.self_pairs = self._candidate_pairs()

    def _candidate_pairs(self) -> list[tuple[int, int]]:
        """Robot geom pairs MuJoCo itself would test (not same body, not parent/child, not excluded)."""
        m = self.m
        geoms = list(self.idx.robot_geoms) + [self.idx.base_geom]
        excluded = {tuple(sorted((int(m.exclude_signature[i] >> 16), int(m.exclude_signature[i] & 0xFFFF))))
                    for i in range(m.nexclude)}
        pairs = []
        for i, g1 in enumerate(geoms):
            for g2 in geoms[i + 1:]:
                b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
                if b1 == b2 or tuple(sorted((b1, b2))) in excluded:
                    continue
                if (m.body_parentid[b1] == b2 and b2 != 0) or (m.body_parentid[b2] == b1 and b1 != 0):
                    continue
                pairs.append((g1, g2))
        return pairs

    def check(self, q, clearance: bool = True) -> CollisionResult:
        m, d = self.m, self.d
        d.qpos[self.idx.qpos] = q
        mujoco.mj_fwdPosition(m, d)
        res = CollisionResult()
        for c in d.contact[:d.ncon]:
            res.contacts.append((m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name,
                                 float(c.dist)))
        if clearance:
            for g in self.idx.robot_geoms:
                dist = min(mujoco.mj_geomDistance(m, d, int(g), int(s), self.distmax, None)
                           for s in self.idx.support_geoms)                # desk top and the base plate
                body = m.body(m.geom_bodyid[g]).name                  # a body may have several collision geoms
                res.floor_by_body[body] = min(float(dist), res.floor_by_body.get(body, np.inf))
                if dist < res.floor_clearance_m:
                    res.floor_clearance_m, res.floor_geom_body = float(dist), m.body(m.geom_bodyid[g]).name
            for g1, g2 in self.self_pairs:
                dist = mujoco.mj_geomDistance(m, d, g1, g2, self.distmax, None)
                if dist < res.self_clearance_m:
                    res.self_clearance_m = float(dist)
                    res.self_pair = (m.body(m.geom_bodyid[g1]).name, m.body(m.geom_bodyid[g2]).name)
        return res


def robot_contacts(m: mujoco.MjModel, d: mujoco.MjData) -> list:
    """Contacts present in live data (every contact involves the robot: world-world never collides)."""
    return [(m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name, float(c.dist))
            for c in d.contact[:d.ncon]]
