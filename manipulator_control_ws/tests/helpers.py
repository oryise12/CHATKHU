"""Deterministic harness: fake_bus + supervisor + runner driven by a simulated clock (no threads, no sleep)."""
from __future__ import annotations

import copy
import math
import tempfile
from pathlib import Path

import mujoco

from manipulator_control import config as config_mod
from manipulator_control.__main__ import make_fake_bus
from manipulator_control.bus import WritePolicy
from manipulator_control.calibration import CalibrationStore
from manipulator_control.logs import NullLog
from manipulator_control.model import PathChecker, load_viz, tcp_position
from manipulator_control.runner import Runner
from manipulator_control.pinch_traj import Acceptance
from manipulator_control.supervisor import Event, S, Supervisor
from manipulator_control.terminal import ScriptedInput

FAKE_SCENE_SHA = "f" * 64


class FakeLink:
    """The pinch link without a socket: tests put simulator events in, read what the controller sent."""

    def __init__(self):
        self.events: list = []
        self.sent: list[dict] = []

    def put(self, kind, arg=None, t=None):
        self.events.append((kind, arg, t))

    def drain(self, now):
        out = [Event(k, now if t is None else t, a, "link") for k, a, t in self.events]
        self.events = []
        return out

    def send(self, msg):
        self.sent.append(dict(msg))
        return None

    def of(self, typ):
        return [m for m in self.sent if m.get("type") == typ]

_MODEL = None


def viz():
    global _MODEL
    if _MODEL is None:
        _MODEL = load_viz()
    return _MODEL


class SimClock:
    def __init__(self, t0: float = 1000.0):
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def fake_cfg(**motion_overrides):
    cfg = config_mod.load(config_mod.FAKE_YAML)
    for section, values in motion_overrides.items():
        cfg.motion[section].update(values)
    return cfg


class System:
    """purpose "home" / "teleop" start like the real program: from the stored calibration. calibrated=True (their
    default) first runs a calibrate session on the same fake servos (joint by joint, then q: held, record stored)."""

    def __init__(self, purpose="home", cfg=None, servo_overrides=None, policy=None, with_checker=True,
                 calibrated=None, store=None, pinch_kw=None):
        self.cfg = cfg or fake_cfg()
        self.pinch_kw = dict(pinch_kw or {})
        self.link = FakeLink()
        self.clock = SimClock()
        self.policy = policy or WritePolicy("control", True)
        self.bus = make_fake_bus(self.cfg, self.policy, self.clock)
        for i, ov in (servo_overrides or {}).items():
            self.bus.servos[i].regs.update(ov)
        self.store = store or CalibrationStore(Path(tempfile.mkdtemp(prefix="mc_store_")) / "calibration_state.json")
        self.with_checker = with_checker
        self.dt = 1.0 / self.cfg.m("loop", "read_hz")
        if calibrated if calibrated is not None else purpose in ("home", "teleop", "pinch"):
            self.start("hold")
            self.align_all()
            assert self.until_state(S.HOLDING, timeout_s=5), (self.state, self.sup.fault)
            self.press("QUIT")
            assert self.until_state(S.EXITED, timeout_s=2)
            assert self.store.load() is not None
        self.start(purpose)

    def start(self, purpose: str) -> None:
        """A new run of the program on the same (fake) servos: new log, supervisor, runner and connection."""
        self.log = NullLog()
        m, arms = viz()
        d = mujoco.MjData(m)
        checker = PathChecker(m, arms["measured"]) if self.with_checker else None
        from manipulator_control.ik import IKSolver
        self.ik = IKSolver(m, arms["measured"])
        kw = {}
        if purpose == "pinch":
            kw = dict(acceptance=Acceptance.from_config(self.cfg, FAKE_SCENE_SHA), **self.pinch_kw)
        self.sup = Supervisor(self.cfg, purpose, self.log, fk=lambda q: tcp_position(m, d, arms["measured"], q),
                              path_checker=checker.check if checker else None, ik=self.ik,
                              saved=None if purpose == "watch" else self.store.load(), **kw)
        self.inputs = ScriptedInput([], self.clock)
        self.runner = Runner(self.bus, self.sup, self.log, self.inputs, self.clock,
                             store=None if purpose == "watch" else self.store,
                             link=self.link if purpose == "pinch" else None)
        self.connected = self.runner.connect()

    @property
    def state(self) -> S:
        return self.sup.state

    def press(self, kind: str, arg=None) -> None:
        self.clock.advance(1e-3)
        self.inputs.put(kind, arg)

    def cycle(self, n: int = 1) -> None:
        for _ in range(n):
            self.clock.advance(self.dt)
            self.runner.cycle()

    def run_until(self, pred, timeout_s: float = 60.0) -> bool:
        end = self.clock.t + timeout_s
        while self.clock.t < end:
            if pred(self):
                return True
            self.cycle()
        return pred(self)

    def until_state(self, *states, timeout_s: float = 60.0) -> bool:
        return self.run_until(lambda s: s.state in states, timeout_s)

    def events(self, kind: str):
        return [e for e in self.log.events if e["event"] == kind]

    def writes(self, addr=None):
        return [w for w in self.bus.writes if addr is None or w[1] == addr]

    def at_step(self, k: int, sub: str) -> bool:
        return self.state == S.ALIGN and self.sup.ctx.get("k") == k and self.sup.ctx.get("sub") == sub

    def align_joint(self, k: int, hand_counts_s: float | None = None) -> None:
        """One joint of the sequential alignment. hand_counts_s: turn the joint by hand first (direction check)."""
        if hand_counts_s is not None:
            assert self.run_until(lambda s: s.at_step(k, "sign"), timeout_s=5), (self.state, self.sup.ctx)
            sv = self.bus.servos[self.cfg.joints[k].dxl_id]
            sv.external_v = hand_counts_s
            assert self.run_until(lambda s: s.sup.ctx.get("detected") is not None, timeout_s=3)
            sv.external_v = 0.0
            self.cycle(2)
            self.press("CONFIRM")
        assert self.run_until(lambda s: s.at_step(k, "zero"), timeout_s=5), (self.state, self.sup.ctx, self.sup.fault)
        self.cycle(1)
        self.press("START")
        assert self.run_until(lambda s: not (s.state == S.ALIGN and s.sup.ctx.get("k") == k), timeout_s=10), \
            (self.state, self.sup.ctx.get("sub"), self.sup.fault)

    def align_all(self, hand=None) -> None:
        for k in range(6):
            self.align_joint(k, None if hand is None else hand[k])

    def confirm_preview(self):
        assert self.until_state(S.HOME_PREVIEW, timeout_s=15), (self.state, self.sup.fault)
        self.cycle(2)
        self.press("CONFIRM")

    def to_homing(self):
        self.confirm_preview()
        assert self.until_state(S.HOMING, timeout_s=15), (self.state, self.sup.fault)


def deg(x):
    return math.degrees(x)


def copy_cfg(cfg):
    return copy.deepcopy(cfg)
