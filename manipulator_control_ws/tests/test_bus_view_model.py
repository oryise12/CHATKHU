"""Write guards, read-only scan, port ownership, display separation, model provenance, reference repo untouched."""
import hashlib
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from helpers import System, fake_cfg, viz
from manipulator_control import REF_REPO as REF
from manipulator_control import WS
from manipulator_control import control_table as ct
from manipulator_control.bus import DxlBus, WriteBlocked, WritePolicy, port_holders
from manipulator_control.supervisor import S



# ---------------------------------------------------------------- write guards

def test_write_policy_read_only_refuses_everything():
    p = WritePolicy("read_only")
    for addr, n in ((64, 1), (108, 12), (11, 1), (20, 4)):
        with pytest.raises(WriteBlocked):
            p.check(addr, n)


def test_write_policy_control_allows_only_ram_items_and_mode_when_enabled():
    p = WritePolicy("control")
    p.check(64, 1)
    p.check(108, 12)
    for addr, n in ((11, 1), (10, 1), (20, 4), (60, 1), (98, 1), (44, 4), (36, 2)):
        with pytest.raises(WriteBlocked):
            p.check(addr, n)
    WritePolicy("control", allow_operating_mode_write=True).check(11, 1)


class _Port:
    def __init__(self):
        self.sent, self.is_using = [], True

    def writePort(self, packet):
        self.sent.append(bytes(packet))
        return len(packet)


def _guarded(mode):
    b = DxlBus.__new__(DxlBus)          # no serial port: only the instruction guard is exercised
    from manipulator_control.bus import BusStats
    b.policy, b.stats, b.port = WritePolicy(mode), BusStats(), _Port()
    b._guard_port()
    return b


def _packet(inst):
    return [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, inst, 0x00, 0x00]


def test_instruction_guard_blocks_writes_on_the_wire_in_read_only_mode():
    b = _guarded("read_only")
    for inst in (0x01, 0x02, 0x82):
        b.port.writePort(_packet(inst))
    for inst in (0x03, 0x83, 0x04, 0x05, 0x06, 0x08, 0x10, 0x93):
        with pytest.raises(WriteBlocked):
            b.port.writePort(_packet(inst))
    assert len(b.port.sent) == 3 and b.stats.writes_sent == 0
    assert b.stats.refused["WRITE"] == 1 and b.stats.refused["REBOOT"] == 1 and b.stats.refused["FACTORY_RESET"] == 1
    c = _guarded("control")
    c.port.writePort(_packet(0x83))
    with pytest.raises(WriteBlocked):
        c.port.writePort(_packet(0x08))          # never REBOOT / FACTORY_RESET / CLEAR, even in control mode


def test_scan_on_fake_bus_sends_no_writes():
    from manipulator_control.__main__ import make_fake_bus
    from manipulator_control.scan import run_scan
    cfg = fake_cfg()
    bus = make_fake_bus(cfg, WritePolicy("read_only"), time.monotonic)
    rep = run_scan(bus, cfg, range(0, 11), timing_s=0.2, noise_s=0.2)
    assert rep["found_ids"] == [1, 2, 3, 4, 5, 6] and rep["writes_sent"] == 0 and not bus.writes
    a = rep["axes"][1]
    assert a["firmware_supports"]["startup_configuration"] and a["bus_watchdog"]["enabled"] is False
    assert a["drive_mode"]["time_based_profile"] is False
    with pytest.raises(WriteBlocked):
        bus.sync_write(64, 1, {1: b"\x01"})


def test_watch_is_read_only_even_with_a_full_config():
    s = System(purpose="watch", cfg=fake_cfg(), policy=WritePolicy("read_only"))
    s.cycle(2)
    assert s.state == S.MONITOR
    s.press("START")
    assert s.run_until(lambda x: x.sup.cal is not None, timeout_s=3)            # read-only calibration
    assert s.state == S.MONITOR
    v = s.sup.view()
    assert v.calibrated and v.q_meas is not None and v.q_cmd is None            # no servo target shown
    assert not s.writes() and s.bus.stats.writes_sent == 0


def test_watch_uncalibrated_shows_relative_motion_only():
    cfg = fake_cfg()
    for j in cfg.robot["joints"]:
        j["sign"] = None
    import yaml, tempfile
    from manipulator_control import config as cfgmod
    tmp = Path(tempfile.mkdtemp()) / "fake.yaml"
    tmp.write_text(yaml.safe_dump(cfg.robot, allow_unicode=True))
    s = System(purpose="watch", cfg=cfgmod.load(tmp), policy=WritePolicy("read_only"))
    s.cycle(2)
    s.bus.servos[2].external_v = 4096.0         # 1 s of hand motion = 22.5 deg at the joint
    s.cycle(20)
    s.bus.servos[2].external_v = 0.0
    s.cycle(2)
    v = s.sup.view()
    assert not v.calibrated and v.q_meas is None and v.rel_deg is not None
    assert v.rel_deg[1] == pytest.approx(22.5, abs=0.6)
    assert "UNCALIBRATED" in v.overlay and "NOT the real pose" in v.overlay


def test_port_holders_detects_another_process(tmp_path):
    dev = tmp_path / "fake_tty"
    dev.write_text("")
    holder = subprocess.Popen([sys.executable, "-c", f"import time; f=open({str(dev)!r}); time.sleep(10)"])
    try:
        for _ in range(50):
            if port_holders(dev):
                break
            time.sleep(0.05)
        assert any(pid == holder.pid for pid, _ in port_holders(dev))
    finally:
        holder.kill()


def test_dxlbus_refuses_a_held_port_without_touching_the_holder(tmp_path, monkeypatch):
    import manipulator_control.bus as busmod
    monkeypatch.setattr(busmod, "port_holders", lambda dev: [(4242, "DynamixelWizard")])
    with pytest.raises(busmod.PortBusy, match="DynamixelWizard"):
        DxlBus(str(tmp_path / "x"), 1_000_000, WritePolicy("read_only"))


# ---------------------------------------------------------------- display

def test_overlay_is_ascii_and_terminal_message_is_korean():
    s = System(cfg=fake_cfg(home={"speed_deg_s": 11.25, "countdown_s": 1.0}))
    s.to_homing()
    v = s.sup.view()
    assert v.overlay.isascii() and "MEASURED" in v.overlay and "SERVO TARGET" in v.overlay
    assert re.search("[가-힣]", v.message)
    assert "TERMINAL" in v.overlay


def test_measured_cmd_target_are_separate_sources():
    s = System(cfg=fake_cfg(home={"speed_deg_s": 11.25, "countdown_s": 1.0}))
    s.to_homing()
    s.cycle(30)
    v = s.sup.view()
    f = s.sup.fast
    q_meas = s.sup.cal.q(f)
    q_traj = s.sup.cal.q(f, "position_trajectory")
    assert np.allclose(v.q_meas, q_meas) and np.allclose(v.q_cmd, q_traj)
    assert not np.allclose(v.q_meas, v.q_cmd)                 # the servo target leads the measured arm
    assert np.allclose(v.q_target, s.cfg.home_q_rad, atol=1e-4)


def test_viewer_applies_each_arm_by_joint_name_and_hides_ghosts_without_data():
    from manipulator_control.supervisor import View
    from manipulator_control.viz import StateViewer
    sv = StateViewer()
    m, arms = sv.m, sv.arms
    for arm, idx in arms.items():
        prefix = {"measured": "", "cmd": "cmd_", "target": "tgt_"}[arm]
        for k, adr in enumerate(idx.qpos):
            assert m.jnt_qposadr[m.joint(f"{prefix}Revolute_{k + 1}").id] == adr
    v = View(calibrated=True, q_meas=np.radians([10, 20, 30, 40, 50, 60]), q_cmd=None, q_target=None)
    sv.apply(v)
    assert np.allclose(sv.d.qpos[arms["measured"].qpos], v.q_meas)
    assert np.all(m.geom_rgba[arms["cmd"].geoms, 3] == 0) and np.all(m.geom_rgba[arms["target"].geoms, 3] == 0)
    v.q_cmd = np.zeros(6)
    sv.apply(v)
    assert m.geom_rgba[arms["cmd"].geoms, 3].max() > 0


def test_closing_the_viewer_requests_the_normal_quit(monkeypatch):
    import sys
    import types
    import mujoco
    from manipulator_control.supervisor import View
    from manipulator_control.viz import StateViewer
    calls = []
    fake = types.SimpleNamespace(
        init=lambda: True, window_hint=lambda *a: None, SAMPLES=0, create_window=lambda *a: "win",
        make_context_current=lambda w: None, swap_interval=lambda n: None, PRESS=1, MOD_SHIFT=1,
        MOUSE_BUTTON_LEFT=0, MOUSE_BUTTON_RIGHT=1, MOUSE_BUTTON_MIDDLE=2,
        set_mouse_button_callback=lambda *a: None, set_cursor_pos_callback=lambda *a: None,
        set_scroll_callback=lambda *a: None,
        window_should_close=lambda w: True,             # the user closed the window
        destroy_window=lambda w: calls.append("destroy"), terminate=lambda: calls.append("terminate"))
    monkeypatch.setitem(sys.modules, "glfw", fake)
    monkeypatch.setattr(mujoco, "MjrContext", lambda *a: types.SimpleNamespace(free=lambda: calls.append("free")))
    asked = []
    StateViewer().run(lambda: View(), lambda: asked.append("QUIT"), lambda: False)
    assert asked == ["QUIT"] and calls == ["free", "destroy", "terminate"]


# ---------------------------------------------------------------- model provenance and the reference repo

def test_model_copy_matches_manifest_and_viz_manifest():
    for line in (WS / "models" / "manifest.sha256").read_text().splitlines():
        h, rel = line.split("  ", 1)
        assert hashlib.sha256((WS / "models" / rel).read_bytes()).hexdigest() == h, rel
    man = json.loads((WS / "models" / "derived" / "manifest.json").read_text())
    assert hashlib.sha256((WS / man["output"]).read_bytes()).hexdigest() == man["output_sha256"]
    assert man["checks"]["errors"] == [] and man["checks"]["urdf"]["differences"] == []
    assert man["checks"]["frames_vs_6dof_mj.urdf"] == "identical"


def test_model_joint_table_matches_the_copy():
    from manipulator_control.model import compare_urdf_kinematics
    joints = compare_urdf_kinematics()["joints"]
    assert list(joints) == [f"Revolute_{i}" for i in range(1, 7)]
    assert joints["Revolute_1"]["xyz"] == [0.0, 0.002984, 0.043554]          # +2 mm in eeaca7f
    assert joints["Revolute_3"]["axis"] == [-1.0, 0.0, 0.0]
    assert all(j["limit"]["lower"] == -3.14159 and j["limit"]["upper"] == 3.14159 for j in joints.values())


def test_display_model_has_no_sim_assumptions():
    text = (WS / "models" / "derived" / "viz_scene.xml").read_text()
    for absent in ("desk", "keyframe", "<actuator", "<sensor", "side_cam"):
        assert absent not in text
    m, arms = viz()
    collide = [g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
    ghost = set(arms["cmd"].geoms) | set(arms["target"].geoms)
    assert not ghost & set(collide) and m.geom("decor_floor").id not in collide


MODEL_PATHS = ["6dof_description", "mujoco_sim/models/generated", "mujoco_sim/configs/demo.json",
               "mujoco_sim/tests/original_model_sha256.json"]


@pytest.mark.skipif(not REF.exists(), reason="reference repo not on this machine")
def test_reference_model_unchanged_since_the_copy():
    """Others may commit to the reference repo (e.g. 14e7eac, camera code, 2026-10-26); this workspace never writes
    there. The copied model must still equal the reference: no committed or uncommitted change to the model paths."""
    git = lambda *a: subprocess.run(["git", "-C", str(REF), *a], capture_output=True, text=True).stdout  # noqa: E731
    from manipulator_control.model import UPSTREAM_COMMIT
    assert git("diff", "--stat", UPSTREAM_COMMIT, "HEAD", "--", *MODEL_PATHS) == ""
    assert git("status", "--porcelain", "--", *MODEL_PATHS) == ""


def test_alignment_view_colors_held_current_free():
    from manipulator_control.viz import CURRENT_GHOST, MEASURED, StateViewer
    s = System(purpose="hold", cfg=fake_cfg())
    s.align_joint(0)
    s.align_joint(1)
    assert s.run_until(lambda x: x.at_step(2, "zero"), timeout_s=3)
    v = s.sup.view()
    assert v.overlay.isascii() and "ALIGN joint 3/6" in v.overlay and re.search("[가-힣]", v.message)
    assert np.allclose(v.q_target, 0.0)                                   # reference pose = all joints 0
    sv = StateViewer()
    sv.apply(v)
    rgba = sv.m.geom_rgba
    for k, status in enumerate(v.joint_status):
        meas, ghost = sv.links["measured"][k], sv.links["target"][k]
        if status == "held":
            assert np.allclose(rgba[meas], MEASURED) and np.all(rgba[ghost, 3] == 0)
        elif status == "current":
            assert np.all(rgba[meas, 3] == 0) and np.allclose(rgba[ghost], CURRENT_GHOST)
        else:
            assert np.all(rgba[meas, 3] == 0) and np.all(rgba[ghost, 3] > 0)
