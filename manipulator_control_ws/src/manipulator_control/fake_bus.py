"""A simulated XM430 chain behind the Bus interface, for tests and the --fake demo. It is a model, not the servo.

What it reproduces (from the e-manual): multi-turn signed counts; power-on position inside one turn; velocity-based
profile (PV 0.229 rpm, PA 214.577 rev/min^2, 0 = infinite) that re-plans from the current velocity when the goal
changes mid-move; Position/Velocity Trajectory registers; torque-on starts the profile at the present position;
EEPROM writes refused while torque is on; Operating Mode change resets PV/PA to 0; Bus Watchdog (only while torque
is on; any received instruction resets it; on expiry the servo stops with PV/PA applied as 0 and goal writes are
refused with a data range error); reboot (position back inside one turn, torque off, RAM reset).
Faults: link_down (nothing reaches the servos, no replies), rx_loss (servos receive, replies lost), per-ID silence,
a stuck joint, hardware error with shutdown, gravity sag while torque is off and the arm is not supported.
"""
from __future__ import annotations

import math
from collections import Counter

from . import control_table as ct
from .bus import Bus, BusError, BusStats, ReadResult, WritePolicy
from .units import pa_counts_s2, pv_counts_s

DEFAULTS = {
    "model_number": 1030, "model_information": 0, "firmware_version": 50, "baud_rate": 3, "return_delay_time": 250,
    "drive_mode": 0, "operating_mode": 4, "secondary_id": 255, "protocol_type": 2, "homing_offset": 0,
    "moving_threshold": 10, "temperature_limit": 80, "max_voltage_limit": 160, "min_voltage_limit": 95,
    "pwm_limit": 885, "current_limit": 1193, "velocity_limit": 330, "max_position_limit": 4095,
    "min_position_limit": 0, "startup_configuration": 0, "shutdown": 52, "torque_enable": 0, "led": 0,
    "status_return_level": 2, "registered_instruction": 0, "hardware_error_status": 0, "velocity_i_gain": 1920,
    "velocity_p_gain": 100, "position_d_gain": 0, "position_i_gain": 0, "position_p_gain": 800,
    "feedforward_2nd_gain": 0, "feedforward_1st_gain": 0, "bus_watchdog": 0, "goal_pwm": 885, "goal_current": 1193,
    "goal_velocity": 0, "profile_acceleration": 0, "profile_velocity": 0, "goal_position": 0, "realtime_tick": 0,
    "moving": 0, "moving_status": 0, "present_pwm": 0, "present_current": 0, "present_velocity": 0,
    "present_position": 0, "velocity_trajectory": 0, "position_trajectory": 0, "present_input_voltage": 123,
    "present_temperature": 32, "backup_ready": 0,
}
DT = 0.001
TRACK_TAU_S = 0.03          # actual position follows the profile with this lag (first order)
FOLLOWING_ERROR_COUNTS = 300


class FakeServo:
    def __init__(self, dxl_id: int, power_on_counts: int = 0, t0: float = 0.0, **overrides):
        self.regs = dict(DEFAULTS, id=dxl_id, **overrides)
        self.pos = float(power_on_counts)
        self.prof_x, self.prof_v = self.pos, 0.0
        self.vel = 0.0
        self.t = t0
        self.last_instruction_t = t0
        self.external_v = 0.0       # counts/s moved by a hand while torque is off
        self.sag_v = 0.0            # counts/s drift while torque is off and the arm is not supported
        self.supported = True
        self.stuck = False
        self.regs["goal_position"] = int(self.pos)

    # ---------------------------------------------------------------- physics
    def advance(self, t: float) -> None:
        while self.t + DT <= t + 1e-12:
            self._step(DT)
            self.t += DT

    def _step(self, dt: float) -> None:
        r = self.regs
        wd = r["bus_watchdog"]
        if r["torque_enable"] and wd > 0 and self.t - self.last_instruction_t > wd * 0.020:
            r["bus_watchdog"] = -1
            self.prof_v = 0.0
            r["goal_position"] = int(round(self.prof_x))
        before = self.pos
        if r["torque_enable"]:
            if r["bus_watchdog"] != -1:
                self._profile(dt)
            if not self.stuck:
                self.pos += (self.prof_x - self.pos) * min(1.0, dt / TRACK_TAU_S)
        else:
            self.prof_x, self.prof_v = self.pos, 0.0
            self.pos += (self.external_v + (0.0 if self.supported else self.sag_v)) * dt
            self.prof_x = self.pos
            r["goal_position"] = int(round(self.pos))     # real XM430 (FW 50): Goal follows Present while torque is
            #                                               off (read 2026-10-26 22:59: goal - present = 0 on all six)
        self.vel = (self.pos - before) / dt

    def _profile(self, dt: float) -> None:
        r = self.regs
        goal = float(r["goal_position"])
        pv, pa = r["profile_velocity"], r["profile_acceleration"]
        e = goal - self.prof_x
        if pv == 0:                                   # step profile
            self.prof_x, self.prof_v = goal, 0.0
            return
        vmax = pv_counts_s(pv)
        amax = pa_counts_s2(pa) if pa > 0 else math.inf
        if abs(e) < 0.5 and abs(self.prof_v) <= (amax * dt if amax < math.inf else vmax):
            self.prof_x, self.prof_v = goal, 0.0
            return
        v_des = math.copysign(min(vmax, math.sqrt(2 * amax * abs(e)) if amax < math.inf else vmax), e)
        dv = v_des - self.prof_v
        step = amax * dt if amax < math.inf else abs(dv)
        self.prof_v += max(-step, min(step, dv))
        nxt = self.prof_x + self.prof_v * dt
        if (goal - self.prof_x) * (goal - nxt) < 0 and abs(self.prof_v) <= step + 1e-9:
            nxt, self.prof_v = goal, 0.0
        self.prof_x = nxt

    # ---------------------------------------------------------------- registers
    def snapshot_regs(self) -> dict[str, int]:
        r = self.regs
        ongoing = r["torque_enable"] and (abs(self.prof_x - r["goal_position"]) > 0.5 or abs(self.prof_v) > 1e-6)
        vel_raw = int(round(self.vel / pv_counts_s(1)))
        r["present_position"] = int(math.floor(self.pos + 0.5))
        r["present_velocity"] = vel_raw
        r["position_trajectory"] = int(math.floor(self.prof_x + 0.5))
        r["velocity_trajectory"] = int(round(self.prof_v / pv_counts_s(1)))
        r["realtime_tick"] = int(self.t * 1000) % 32768
        r["moving"] = int(abs(vel_raw) > r["moving_threshold"] or bool(ongoing))
        profile_bits = 0 if r["profile_velocity"] == 0 else (1 if r["profile_acceleration"] == 0 else 3)
        following = r["torque_enable"] and abs(self.prof_x - self.pos) > FOLLOWING_ERROR_COUNTS
        in_pos = r["torque_enable"] and not ongoing and abs(r["goal_position"] - self.pos) < 10
        r["moving_status"] = (int(in_pos) | (int(bool(ongoing)) << 1) | (int(bool(following)) << 3)
                              | (profile_bits << 4))
        r["present_current"] = int(max(-1193, min(1193, (r["goal_position"] - self.pos) * 0.5))) if r["torque_enable"] else 0
        return r

    def encode(self, addr: int, length: int) -> bytes:
        regs = self.snapshot_regs()
        buf = bytearray(length)
        for item in ct.ITEMS.values():
            if addr <= item.addr and item.addr + item.size <= addr + length:
                off = item.addr - addr
                v = regs[item.name]
                if not item.signed and v < 0:
                    v &= (1 << (8 * item.size)) - 1
                buf[off:off + item.size] = ct.encode(v, item.size, item.signed)
        return bytes(buf)

    def status_error(self, err: int = 0) -> int:
        return err | (0x80 if self.regs["hardware_error_status"] else 0)

    def receive(self, t: float) -> None:
        self.last_instruction_t = t

    def write(self, addr: int, data: bytes) -> int:
        """Apply a write; returns the status error number (0 = ok)."""
        r = self.regs
        end = addr + len(data)
        touched = [i for i in ct.ITEMS.values() if addr <= i.addr and i.addr + i.size <= end]
        if not touched or sum(i.size for i in touched) != len(data):
            return 5
        for item in touched:
            if item.readonly:
                return 7
            if item.eeprom and r["torque_enable"]:
                return 7
            if item.name == "goal_position" and r["bus_watchdog"] == -1:
                return 4
        for item in touched:
            v = ct.decode(item, addr, data)
            self._set(item.name, v)
        return 0

    def _set(self, name: str, v: int) -> None:
        r = self.regs
        if name == "torque_enable":
            if v and not r["torque_enable"]:
                self.prof_x, self.prof_v = self.pos, 0.0      # profile starts at the present position
                r["goal_position"] = int(round(self.pos))     # seen on the real XM430 (FW 50): Goal = Present at
                #                                               torque on (2026-10-26, ID 2: 17194 written -> 17193)
            r[name] = int(bool(v))
        elif name == "operating_mode":
            if v != r[name]:
                r["profile_velocity"] = r["profile_acceleration"] = 0
                if v == 3:
                    self.pos = float(int(self.pos) % 4096 + r["homing_offset"])
            r[name] = v
        elif name == "bus_watchdog":
            r[name] = 0 if v == 0 else v
        else:
            r[name] = v

    def reboot(self) -> None:
        r = self.regs
        self.pos = float(int(math.floor(self.pos)) % 4096 + r["homing_offset"])
        self.prof_x, self.prof_v, self.vel = self.pos, 0.0, 0.0
        for k in ("profile_velocity", "profile_acceleration", "hardware_error_status", "bus_watchdog", "led"):
            r[k] = 0
        r["torque_enable"] = 1 if r["startup_configuration"] & 1 else 0
        r["goal_position"] = int(self.pos)

    def hardware_error(self, bits: int) -> None:
        self.regs["hardware_error_status"] |= bits
        if bits & self.regs["shutdown"]:
            self.regs["torque_enable"] = 0


class FakeBus(Bus):
    def __init__(self, clock, ids=(1, 2, 3, 4, 5, 6), policy: WritePolicy | None = None, power_on_counts=None,
                 **servo_overrides):
        self.clock = clock
        self.policy = policy or WritePolicy("control", allow_operating_mode_write=True)
        self.stats = BusStats()
        self.info = {"device": "fake", "latency_timer_ms": None, "modemmanager": None, "tiocexcl": None}
        counts = power_on_counts or [0] * len(ids)
        t0 = self.clock()
        self.servos = {i: FakeServo(i, c, t0, **servo_overrides) for i, c in zip(ids, counts)}
        self.link_down = False      # nothing reaches the servos, nothing comes back
        self.rx_loss = False        # servos receive and act, replies are lost
        self.silent_ids: set[int] = set()   # these IDs receive but do not reply
        self.dead_ids: set[int] = set()     # these IDs neither receive nor reply (unpowered)
        self.read_time_s = 0.0
        self.writes: list[tuple] = []       # (t, addr, length, {id: bytes}) for tests
        self.closed = False

    def _sync(self, t: float) -> None:
        for s in self.servos.values():
            s.advance(t)

    def advance(self) -> None:
        self._sync(self.clock())

    def _reaches(self, dxl_id: int) -> bool:
        return not self.link_down and dxl_id not in self.dead_ids

    def _replies(self, dxl_id: int) -> bool:
        return self._reaches(dxl_id) and not self.rx_loss and dxl_id not in self.silent_ids

    def _deliver(self, t: float) -> None:
        for i, s in self.servos.items():
            if self._reaches(i):
                s.receive(t)

    def ping(self, dxl_id):
        t = self.clock()
        self._sync(t)
        self.stats.sent["PING"] += 1
        self._deliver(t)
        s = self.servos.get(dxl_id)
        if s is None or not self._replies(dxl_id):
            return None, 0
        return s.regs["model_number"], s.status_error()

    def read(self, dxl_id, addr, length):
        t = self.clock()
        self._sync(t)
        self.stats.sent["READ"] += 1
        self._deliver(t)
        s = self.servos.get(dxl_id)
        if s is None or not self._replies(dxl_id):
            return None, 0
        return s.encode(addr, length), s.status_error()

    def sync_read(self, addr, length, ids):
        t0 = self.clock()
        self._sync(t0)
        self.stats.sent["SYNC_READ"] += 1
        self._deliver(t0)
        data, errors = {}, {}
        for i in ids:
            s = self.servos.get(i)
            if s is not None and self._replies(i):
                data[i], errors[i] = s.encode(addr, length), s.status_error()
        missing = tuple(i for i in ids if i not in data)
        t1 = t0 + self.read_time_s
        if (addr, length) == ct.FAST:
            self.stats.note_read(self.read_time_s, not missing)
        return ReadResult(addr, length, data, errors, missing, t0, t1)

    def sync_write(self, addr, length, values):
        self.policy.check(addr, length)
        t = self.clock()
        self._sync(t)
        self.stats.sent["SYNC_WRITE"] += 1
        self.writes.append((t, addr, length, dict(values)))
        self._deliver(t)
        for i, data in values.items():
            s = self.servos.get(i)
            if s is not None and self._reaches(i):
                s.write(addr, data)      # no status packet for sync write: errors are invisible to the PC
        return t

    def write(self, dxl_id, addr, data):
        self.policy.check(addr, len(data))
        t = self.clock()
        self._sync(t)
        self.stats.sent["WRITE"] += 1
        self.writes.append((t, addr, len(data), {dxl_id: bytes(data)}))
        self._deliver(t)
        s = self.servos.get(dxl_id)
        if s is None or not self._reaches(dxl_id):
            raise BusError(f"ID {dxl_id}: no status packet")
        err = s.write(addr, data)
        if not self._replies(dxl_id):
            raise BusError(f"ID {dxl_id}: no status packet")
        return s.status_error(err)

    def close(self):
        self.closed = True

    def sent_counts(self) -> Counter:
        return self.stats.sent
