"""Read-only scan: who answers, every control-table value (0..147) decoded, measured Sync Read timing and rest noise.

The bus is opened with WritePolicy("read_only") and the instruction guard: only PING, READ and SYNC_READ can
leave the PC. The report ends with the number of packets per instruction actually sent (writes must be 0).
"""
from __future__ import annotations

import datetime as dt
import json
import statistics
import time

from . import WS
from . import control_table as ct
from .config import Config, motion_blockers
from .state import AxisFast, SlowSnapshot
from .supervisor import servo_setting_problems

FW_FEATURES = {"drive_mode": 38, "bus_watchdog": 38, "time_based_profile": 42, "startup_configuration": 45,
               "torque_on_by_goal_update": 45}


def decode_axis(regs: dict[str, int]) -> dict:
    fw = regs["firmware_version"]
    return {
        "model_number": regs["model_number"], "firmware_version": fw,
        "baud_rate_bps": ct.BAUD_RATES.get(regs["baud_rate"]),
        "return_delay_us": regs["return_delay_time"] * 2,
        "operating_mode": f"{regs['operating_mode']} ({ct.OPERATING_MODES.get(regs['operating_mode'], '?')})",
        "drive_mode": {"value": regs["drive_mode"], **ct.drive_mode_bits(regs["drive_mode"])},
        "startup_configuration": {"value": regs["startup_configuration"],
                                  **ct.startup_configuration_bits(regs["startup_configuration"])},
        "homing_offset": regs["homing_offset"],
        "velocity_limit_raw": regs["velocity_limit"], "velocity_limit_rpm": round(regs["velocity_limit"] * 0.229, 3),
        "profile_velocity_raw": regs["profile_velocity"], "profile_acceleration_raw": regs["profile_acceleration"],
        "bus_watchdog": {"value": regs["bus_watchdog"], "enabled": regs["bus_watchdog"] > 0,
                         "error": regs["bus_watchdog"] == -1, "timeout_ms": max(regs["bus_watchdog"], 0) * 20},
        "torque_enable": regs["torque_enable"], "status_return_level": regs["status_return_level"],
        "hardware_error": {"value": regs["hardware_error_status"], **ct.hardware_error_bits(regs["hardware_error_status"])},
        "shutdown": {"value": regs["shutdown"], **ct.hardware_error_bits(regs["shutdown"])},
        "pwm_limit": regs["pwm_limit"], "current_limit": regs["current_limit"],
        "gains": {k: regs[k] for k in ("position_p_gain", "position_i_gain", "position_d_gain", "velocity_p_gain",
                                       "velocity_i_gain", "feedforward_1st_gain", "feedforward_2nd_gain")},
        "present_position": regs["present_position"], "present_velocity_raw": regs["present_velocity"],
        "goal_position": regs["goal_position"],
        "present_input_voltage_v": regs["present_input_voltage"] / 10, "present_temperature_c": regs["present_temperature"],
        "secondary_id": regs["secondary_id"], "protocol_type": regs["protocol_type"],
        "moving_status": ct.moving_status_bits(regs["moving_status"]),
        "firmware_supports": {k: fw >= v for k, v in FW_FEATURES.items()},
    }


def run_scan(bus, cfg: Config, ping_range=range(0, 21), timing_s: float = 5.0, noise_s: float = 3.0) -> dict:
    ids = list(cfg.ids)
    t0 = time.monotonic()
    found = {}
    for i in sorted(set(ping_range) | set(ids)):
        model, err = bus.ping(i)
        if model is not None:
            found[i] = {"model_number": model, "status_error": ct.status_error(err)}
    axes, raw = {}, {}
    for i in found:
        data, err = bus.read(i, *ct.FULL)
        if data is None:
            axes[i] = {"read": "failed"}
            continue
        regs = ct.decode_block(ct.FULL[0], data)
        raw[i] = regs
        axes[i] = decode_axis(regs)
        axes[i]["status_error"] = ct.status_error(err)

    timing = measure_timing(bus, [i for i in ids if i in found], timing_s, noise_s) if found else None
    slow = SlowSnapshot(0, time.monotonic(), {i: raw[i] for i in ids if i in raw}, {}, tuple(i for i in ids if i not in raw))
    problems, notes = servo_setting_problems(cfg, slow, {i: found.get(i, {}).get("model_number") for i in ids})
    return {
        "time": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "duration_s": round(time.monotonic() - t0, 2),
        "bus": bus.info, "config": str(cfg.robot_path), "config_sha": cfg.robot_sha,
        "expected_ids": ids, "found_ids": sorted(found), "missing_ids": [i for i in ids if i not in found],
        "unexpected_ids": [i for i in found if i not in ids],
        "axes": axes, "timing": timing,
        "prepare_refused_by_servo_settings": problems, "notes": notes,
        "config_blockers": {"hold": motion_blockers(cfg, "hold"), "home": motion_blockers(cfg, "home")},
        "instructions_sent": dict(bus.stats.sent), "instructions_refused": dict(bus.stats.refused),
        "writes_sent": bus.stats.writes_sent,
    }


def measure_timing(bus, ids, timing_s: float, noise_s: float) -> dict:
    """Back-to-back fast Sync Reads (achievable cycle) and position spread at rest."""
    durations, failures = [], 0
    pos = {i: [] for i in ids}
    end = time.monotonic() + max(timing_s, noise_s)
    t_noise_end = time.monotonic() + noise_s
    while time.monotonic() < end:
        r = bus.sync_read(*ct.FAST, ids)
        durations.append(r.t_done - r.t_start)
        if r.missing:
            failures += 1
        if time.monotonic() <= t_noise_end:
            for i, d in r.data.items():
                pos[i].append(AxisFast.from_bytes(d, 0).present_position)
    ds = sorted(durations)
    q = lambda p: round(ds[min(len(ds) - 1, int(p * len(ds)))] * 1000, 2)   # noqa: E731
    return {
        "reads": len(ds), "failed_reads": failures,
        "cycle_ms": {"p50": q(0.5), "p95": q(0.95), "p99": q(0.99), "max": round(ds[-1] * 1000, 2)} if ds else None,
        "achievable_hz_p99": round(1000 / q(0.99), 1) if ds and q(0.99) > 0 else None,
        "rest_spread_counts": {i: (max(v) - min(v)) if v else None for i, v in pos.items()},
        "rest_stdev_counts": {i: round(statistics.pstdev(v), 3) if len(v) > 1 else None for i, v in pos.items()},
        "note": "back-to-back reads without the control loop's sleep; the 20 Hz loop adds slow reads at 2 Hz",
    }


def save(report: dict, fake: bool) -> str:
    d = WS / "logs" / ("fake" if fake else "scans")
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"scan_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    return str(path)


def print_summary(rep: dict) -> None:
    print(f"포트 {rep['bus'].get('device')}  latency_timer {rep['bus'].get('latency_timer_ms')} ms  "
          f"ModemManager {rep['bus'].get('modemmanager')}  TIOCEXCL {rep['bus'].get('tiocexcl')}")
    print(f"응답 ID {rep['found_ids']}  (기대 {rep['expected_ids']}, 누락 {rep['missing_ids']}, 예상 밖 {rep['unexpected_ids']})")
    for i, a in rep["axes"].items():
        if "model_number" not in a:
            print(f"  ID {i}: 읽기 실패")
            continue
        print(f"  ID {i}: model {a['model_number']} FW {a['firmware_version']} {a['baud_rate_bps']} bps | mode {a['operating_mode']} | "
              f"drive {a['drive_mode']['value']} | startup {a['startup_configuration']['value']} | "
              f"homing_offset {a['homing_offset']} | watchdog {a['bus_watchdog']['value']} | torque {a['torque_enable']} | "
              f"HW err {a['hardware_error']['value']} | PV/PA {a['profile_velocity_raw']}/{a['profile_acceleration_raw']} | "
              f"vel_limit {a['velocity_limit_raw']} | pos {a['present_position']} | {a['present_input_voltage_v']} V "
              f"{a['present_temperature_c']} C")
    t = rep["timing"]
    if t:
        print(f"Sync Read(6축, 39 byte): {t['reads']}회, 실패 {t['failed_reads']}, 주기 ms {t['cycle_ms']}, "
              f"p99 기준 {t['achievable_hz_p99']} Hz | 정지 잡음 max-min count {t['rest_spread_counts']}")
    if rep["prepare_refused_by_servo_settings"]:
        print("prepare 거부 사유(서보 설정, 자동으로 고치지 않음):")
        for p in rep["prepare_refused_by_servo_settings"]:
            print(f"  - {p}")
    print(f"설정 미확정(hold): {len(rep['config_blockers']['hold'])}개, (home): {len(rep['config_blockers']['home'])}개")
    print(f"송신 명령: {rep['instructions_sent']}  거부: {rep['instructions_refused']}  -> 모터 쓰기 {rep['writes_sent']}회")
