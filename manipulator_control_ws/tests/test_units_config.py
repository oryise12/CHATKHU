"""Units, raw limits, and the gate between read-only tools and motion."""
import math
import re
from pathlib import Path

import pytest

from manipulator_control import WS
from manipulator_control import config as cfgmod
from manipulator_control import control_table as ct
from manipulator_control.units import (JointMap, max_pv_raw_for_rpm, pa_unit_rad_s2, pv_raw_to_rpm, pv_unit_rad_s,
                                       goal_in_range)


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("p_ref", [4000, -70000, 0, 4095, 4096, 1_000_000])
def test_count_rad_round_trip_multi_turn_both_signs(sign, p_ref):
    j = JointMap("J", 1, 16.0, sign)
    for q in (-3.0, -0.5, 0.0, 0.123456, 1.5, 3.1):
        p = j.p_goal(q, p_ref, 0.3)
        assert abs(j.q_rad(p, p_ref, 0.3) - q) <= 0.5 / j.k + 1e-12      # within half a count
    assert j.q_rad(p_ref + 4096, p_ref, 0.0) == pytest.approx(sign * math.radians(22.5), abs=1e-12)


def test_4096_counts_is_22_5_deg_after_16_to_1():
    j = JointMap("J", 1, 16.0, 1)
    assert math.degrees(j.q_rad(4096, 0, 0.0)) == pytest.approx(22.5, abs=1e-12)
    assert j.p_goal(math.radians(22.5), 0, 0.0) == 4096
    assert j.p_goal(math.radians(-360), -70000, 0.0) == -70000 - 65536


def test_negative_and_large_counts_are_not_wrapped():
    j = JointMap("J", 1, 16.0, -1)
    assert j.q_rad(-70000, 4000, 0.0) == pytest.approx((70000 + 4000) / j.k)
    src = "\n".join(p.read_text() for p in (WS / "src").rglob("*.py"))
    assert not re.search(r"position\w*\s*%\s*4096", src)          # only the fake servo's power-on model wraps
    assert not re.search(r"present_position\]?\s*%", src)


def test_signed_decoding():
    item = ct.ITEMS["present_position"]
    assert ct.decode(item, 132, b"\xff\xff\xff\xff") == -1
    assert ct.decode(ct.ITEMS["present_current"], 126, b"\x00\x80") == -32768
    assert ct.decode(ct.ITEMS["profile_velocity"], 112, b"\x06\x01\x00\x00") == 262


def test_raw_speed_and_acceleration_units():
    assert max_pv_raw_for_rpm(60) == 262
    assert pv_raw_to_rpm(262) == pytest.approx(59.998, abs=1e-3)
    assert pv_raw_to_rpm(306) == pytest.approx(70.074, abs=1e-3)
    assert math.degrees(pv_unit_rad_s(16)) * 262 == pytest.approx(22.499, abs=1e-3)
    pa = math.degrees(pa_unit_rad_s2(16))
    assert pa == pytest.approx(1.34110625, abs=1e-8)
    assert 4 * pa == pytest.approx(5.3644, abs=1e-4)                 # PA 4 exceeds a 5 deg/s^2 cap
    assert math.floor(5.0 / pa) == 3
    assert goal_in_range(1_048_575) and not goal_in_range(1_048_576) and not goal_in_range(-1_048_576)


def test_hardware_config_values_and_gates():
    cfg = cfgmod.load(cfgmod.ROBOT_YAML)
    assert cfg.kind == "hardware"
    # user decisions 2026-10-26 17:05: upper arm straight up, Revolute_3 -90, Revolute_5 +90;
    # ranges set by the user (not measured): 2026-10-28 all +-180 like the simulator's model (before: 2026-10-26 23:20
    # 1 +-45, 2 +-90, 3 +-160, 4 +-170, 5 +-170, 6 0..+170)
    assert [round(math.degrees(j.q_ref_rad), 6) for j in cfg.joints] == [0, 0, -90, 0, 90, 0]
    assert [tuple(round(math.degrees(x)) for x in j.limit_rad) for j in cfg.joints] == \
        [(-180, 180)] * 6
    # HOME = mirrored sim ready (user decision 2026-10-26 17:10), 5 cm lower since 2026-10-28
    assert [round(math.degrees(q), 3) for q in cfg.home_q_rad] == [0.0, -47.124, -96.275, 40.849, 90.0, 8.008]
    assert "범위, 실측 아님" in cfgmod.ROBOT_YAML.read_text()                 # the file says the ranges are not measured
    assert cfgmod.motion_blockers(cfg, "hold") == []
    unknown = [j.name for j in cfg.joints if j.sign is None]
    home = cfgmod.motion_blockers(cfg, "home")
    assert len(home) == len(unknown) and cfgmod.motion_blockers(cfg, "teleop") == home
    for j, q in zip(cfg.joints, cfg.home_q_rad):                     # HOME and the reference are inside the ranges
        assert j.limit_rad[0] <= q <= j.limit_rad[1] and j.limit_rad[0] <= j.q_ref_rad <= j.limit_rad[1]
    assert [j.dxl_id for j in cfg.joints] == [1, 2, 3, 4, 5, 6] and all(j.gear == 16 for j in cfg.joints)


def test_unknown_values_block_motion_but_not_calibrate(tmp_path):
    text = cfgmod.ROBOT_YAML.read_text()
    text = re.sub(r"sign: -?1,", "sign: null,", text, count=1)
    text = text.replace("limit_deg: [-180, 180]}", "limit_deg: null}", 1)
    p = Path(tmp_path) / "robot.yaml"
    p.write_text(text)
    cfg = cfgmod.load(p)
    assert cfgmod.motion_blockers(cfg, "hold") == []
    home = cfgmod.motion_blockers(cfg, "home")
    assert any("sign 미확정" in b for b in home) and any("limit_deg 미확정" in b for b in home)
    assert cfgmod.motion_blockers(cfg, "teleop") == home
    with pytest.raises(ValueError):
        cfg.joints[0].joint_map()                                 # no sign: no conversion at all


def test_fake_config_is_separate_and_complete():
    fake = cfgmod.load(cfgmod.FAKE_YAML)
    assert fake.kind == "fake" and cfgmod.motion_blockers(fake, "home") == []
    assert "FAKE" in cfgmod.FAKE_YAML.read_text() and "fake" not in cfgmod.ROBOT_YAML.read_text().split("kind:")[1][:12]


def test_motion_commands_need_a_person_at_the_terminal(monkeypatch):
    from manipulator_control import __main__ as cli
    from manipulator_control import bus
    monkeypatch.setattr(bus, "DxlBus", lambda *a, **k: pytest.fail("port must not be opened"))
    cfg = cfgmod.load(cfgmod.ROBOT_YAML)
    for argv, purpose in ((["calibrate", "--no-viewer"], "hold"), (["prepare", "--hold-only", "--no-viewer"], "hold"),
                          (["prepare", "--no-viewer"], "home"), (["teleop", "--no-viewer"], "teleop")):
        if cfgmod.motion_blockers(cfg, purpose):
            assert cli.main(argv) == 2                               # refused by the config, port never opened
        else:
            with pytest.raises(SystemExit, match="대화형 터미널"):   # no unattended torque-on / motion
                cli.main(argv)


def test_auto_start_is_fake_only():
    from manipulator_control import __main__ as cli
    with pytest.raises(SystemExit):
        cli.main(["prepare", "--auto-start", "1"])


def test_structural_config_errors(tmp_path):
    text = cfgmod.ROBOT_YAML.read_text().replace("name: Revolute_2, id: 2", "name: Revolute_2, id: 1")
    p = Path(tmp_path) / "bad.yaml"
    p.write_text(text)
    with pytest.raises(ValueError, match="duplicate"):
        cfgmod.load(p)


def test_preview_home_tool(capsys):
    from manipulator_control import __main__ as cli
    assert cli.main(["preview-home", "--fake", "--no-viewer"]) == 0
    assert "T_max_s" in capsys.readouterr().out
    assert cli.main(["preview-home", "--no-viewer"]) == 0          # real config: reference pose -> HOME
    assert "T_max_s" in capsys.readouterr().out
