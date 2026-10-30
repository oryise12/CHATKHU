"""XM430-W210 (Protocol 2.0) control table: addresses, sizes, signedness, and bit decoders.

Source: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ (checked 2026-10-26). Everything here is a plain
fact table; which items may be written is decided by bus.WritePolicy, not here.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Item:
    name: str
    addr: int
    size: int
    signed: bool = False
    eeprom: bool = False
    readonly: bool = False


_ITEMS = [
    Item("model_number", 0, 2, eeprom=True, readonly=True),
    Item("model_information", 2, 4, eeprom=True, readonly=True),
    Item("firmware_version", 6, 1, eeprom=True, readonly=True),
    Item("id", 7, 1, eeprom=True),
    Item("baud_rate", 8, 1, eeprom=True),
    Item("return_delay_time", 9, 1, eeprom=True),
    Item("drive_mode", 10, 1, eeprom=True),
    Item("operating_mode", 11, 1, eeprom=True),
    Item("secondary_id", 12, 1, eeprom=True),
    Item("protocol_type", 13, 1, eeprom=True),
    Item("homing_offset", 20, 4, signed=True, eeprom=True),
    Item("moving_threshold", 24, 4, eeprom=True),
    Item("temperature_limit", 31, 1, eeprom=True),
    Item("max_voltage_limit", 32, 2, eeprom=True),
    Item("min_voltage_limit", 34, 2, eeprom=True),
    Item("pwm_limit", 36, 2, eeprom=True),
    Item("current_limit", 38, 2, eeprom=True),
    Item("velocity_limit", 44, 4, eeprom=True),
    Item("max_position_limit", 48, 4, eeprom=True),
    Item("min_position_limit", 52, 4, eeprom=True),
    Item("startup_configuration", 60, 1, eeprom=True),
    Item("shutdown", 63, 1, eeprom=True),
    Item("torque_enable", 64, 1),
    Item("led", 65, 1),
    Item("status_return_level", 68, 1),
    Item("registered_instruction", 69, 1, readonly=True),
    Item("hardware_error_status", 70, 1, readonly=True),
    Item("velocity_i_gain", 76, 2),
    Item("velocity_p_gain", 78, 2),
    Item("position_d_gain", 80, 2),
    Item("position_i_gain", 82, 2),
    Item("position_p_gain", 84, 2),
    Item("feedforward_2nd_gain", 88, 2),
    Item("feedforward_1st_gain", 90, 2),
    Item("bus_watchdog", 98, 1, signed=True),
    Item("goal_pwm", 100, 2, signed=True),
    Item("goal_current", 102, 2, signed=True),
    Item("goal_velocity", 104, 4, signed=True),
    Item("profile_acceleration", 108, 4),
    Item("profile_velocity", 112, 4),
    Item("goal_position", 116, 4, signed=True),
    Item("realtime_tick", 120, 2, readonly=True),
    Item("moving", 122, 1, readonly=True),
    Item("moving_status", 123, 1, readonly=True),
    Item("present_pwm", 124, 2, signed=True, readonly=True),
    Item("present_current", 126, 2, signed=True, readonly=True),
    Item("present_velocity", 128, 4, signed=True, readonly=True),
    Item("present_position", 132, 4, signed=True, readonly=True),
    Item("velocity_trajectory", 136, 4, signed=True, readonly=True),
    Item("position_trajectory", 140, 4, signed=True, readonly=True),
    Item("present_input_voltage", 144, 2, readonly=True),
    Item("present_temperature", 146, 1, readonly=True),
    Item("backup_ready", 147, 1, readonly=True),
]
ITEMS = {i.name: i for i in _ITEMS}
BY_ADDR = {i.addr: i for i in _ITEMS}

MODEL_NUMBER_XM430_W210 = 1030
# Read blocks: (start address, length). FAST: every cycle (profile, goal, tick, moving, current, velocity,
# position, servo trajectory, voltage, temperature). SLOW: settings + torque + HW error + watchdog.
FAST = (108, 39)          # 108..146
SLOW = (10, 89)           # 10..98
FULL = (0, 148)           # 0..147 (scan)
# One 12-byte write block for the profile and the goal, so PA, PV and Goal Position arrive in one packet.
PROFILE_GOAL = (108, 12)  # 108 PA, 112 PV, 116 Goal Position

OPERATING_MODES = {0: "current", 1: "velocity", 3: "position", 4: "extended_position",
                   5: "current_based_position", 16: "pwm"}
BAUD_RATES = {0: 9_600, 1: 57_600, 2: 115_200, 3: 1_000_000, 4: 2_000_000, 5: 3_000_000, 6: 4_000_000,
              7: 4_500_000}
# Status packet error byte: bit7 = hardware alert, bits0-6 = error number.
STATUS_ERRORS = {1: "result_fail", 2: "instruction_error", 3: "crc_error", 4: "data_range_error",
                 5: "data_length_error", 6: "data_limit_error", 7: "access_error"}


def decode(item: Item, block_start: int, data: bytes | list[int]) -> int:
    off = item.addr - block_start
    raw = bytes(data[off:off + item.size])
    if len(raw) != item.size:
        raise ValueError(f"{item.name} not inside block starting at {block_start}")
    return int.from_bytes(raw, "little", signed=item.signed)


def decode_block(block_start: int, data: bytes | list[int]) -> dict[str, int]:
    """Every item fully inside [block_start, block_start + len(data))."""
    end = block_start + len(data)
    return {i.name: decode(i, block_start, data) for i in _ITEMS if block_start <= i.addr and i.addr + i.size <= end}


def encode(value: int, size: int, signed: bool) -> bytes:
    return int(value).to_bytes(size, "little", signed=signed)


def drive_mode_bits(v: int) -> dict[str, bool]:
    return {"reverse": bool(v & 0x01), "time_based_profile": bool(v & 0x04), "torque_on_by_goal_update": bool(v & 0x08)}


def startup_configuration_bits(v: int) -> dict[str, bool]:
    return {"startup_torque_on": bool(v & 0x01), "ram_restore": bool(v & 0x02)}


def hardware_error_bits(v: int) -> dict[str, bool]:
    """Hardware Error Status(70) / Shutdown(63) bit names."""
    return {"input_voltage": bool(v & 0x01), "overheating": bool(v & 0x04), "motor_encoder": bool(v & 0x08),
            "electrical_shock": bool(v & 0x10), "overload": bool(v & 0x20)}


def moving_status_bits(v: int) -> dict[str, object]:
    profile = {0: "step", 1: "rectangular", 2: "triangular", 3: "trapezoidal"}[(v >> 4) & 0x03]
    return {"in_position": bool(v & 0x01), "profile_ongoing": bool(v & 0x02), "following_error": bool(v & 0x08),
            "profile": profile}


def status_error(v: int) -> dict[str, object]:
    return {"alert": bool(v & 0x80), "error": STATUS_ERRORS.get(v & 0x7F) if v & 0x7F else None}
