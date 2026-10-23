import mujoco
import numpy as np
import pytest

from pinch_sim.collision import CollisionChecker
from pinch_sim.kinematics import Kinematics
from pinch_sim.model import load_config, load_scene, ready_q
from pinch_sim.validation import PlanningContext


@pytest.fixture(scope="session")
def cfg():
    return load_config()


@pytest.fixture(scope="session")
def scene(cfg):
    return load_scene(cfg)


@pytest.fixture(scope="session")
def kin(scene, cfg):
    m, idx, _ = scene
    return Kinematics(m, idx, cfg["ik"])


@pytest.fixture(scope="session")
def col(scene):
    m, idx, _ = scene
    return CollisionChecker(m, idx)


@pytest.fixture
def data(scene):
    m, _, _ = scene
    d = mujoco.MjData(m)
    d.qpos[:] = ready_q(m)
    mujoco.mj_forward(m, d)
    return d


@pytest.fixture(scope="session")
def ctx(scene, cfg):
    m, idx, sha = scene
    return PlanningContext(m, idx, sha, cfg)


@pytest.fixture(scope="session")
def q_ready(scene):
    m, idx, _ = scene
    return ready_q(m)[idx.qpos]
