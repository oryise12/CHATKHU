"""The single owner of the Dynamixel bus.

DxlBus (real, dynamixel_sdk) and fake_bus.FakeBus share this interface. Two independent write guards:
  - WritePolicy: which addresses may be written at all (read_only: none; control: Torque Enable, LED, the
    PA/PV/Goal block 108..119, and Operating Mode(11) only if the config allows it).
  - InstructionGuard (DxlBus): inspects every packet right before it reaches the serial port and refuses any
    instruction outside the mode's set (read_only: PING, READ, SYNC_READ). It also counts what was sent.
Opening refuses a port another process holds (e.g. DYNAMIXEL Wizard) and never kills that process; after opening,
TIOCEXCL keeps other programs from opening the port, and a flock keeps a second copy of this program out.
"""
from __future__ import annotations

import fcntl
import os
import subprocess
import termios
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

from . import control_table as ct

INSTRUCTION_NAMES = {0x01: "PING", 0x02: "READ", 0x03: "WRITE", 0x04: "REG_WRITE", 0x05: "ACTION",
                     0x06: "FACTORY_RESET", 0x08: "REBOOT", 0x10: "CLEAR", 0x82: "SYNC_READ", 0x83: "SYNC_WRITE",
                     0x8A: "FAST_SYNC_READ", 0x92: "BULK_READ", 0x93: "BULK_WRITE", 0x9A: "FAST_BULK_READ"}
READ_INSTRUCTIONS = frozenset({0x01, 0x02, 0x82})
CONTROL_INSTRUCTIONS = READ_INSTRUCTIONS | {0x03, 0x83}


class BusError(RuntimeError):
    pass


class PortBusy(BusError):
    pass


class WriteBlocked(BusError):
    """A write the current mode does not allow. Nothing was sent."""


@dataclass(frozen=True)
class WritePolicy:
    mode: str                          # "read_only" | "control"
    allow_operating_mode_write: bool = False

    RAM_WRITABLE = ((64, 1), (65, 1), (108, 12), (108, 4), (112, 4), (116, 4))

    def check(self, addr: int, length: int) -> None:
        if self.mode == "read_only":
            raise WriteBlocked(f"read-only bus: write to {addr} refused")
        if (addr, length) in self.RAM_WRITABLE:
            return
        if (addr, length) == (11, 1) and self.allow_operating_mode_write:
            return
        raise WriteBlocked(f"write to address {addr} (length {length}) is not allowed")


@dataclass
class ReadResult:
    addr: int
    length: int
    data: dict[int, bytes]
    errors: dict[int, int]
    missing: tuple[int, ...]
    t_start: float
    t_done: float


@dataclass
class BusStats:
    sent: Counter = field(default_factory=Counter)        # instruction name -> packets
    refused: Counter = field(default_factory=Counter)
    read_durations: list = field(default_factory=list)    # fast sync read durations, s (bounded)
    failed_reads: int = 0

    def note_read(self, duration: float, ok: bool) -> None:
        self.read_durations.append(duration)
        if len(self.read_durations) > 20000:
            del self.read_durations[:10000]
        self.failed_reads += 0 if ok else 1

    @property
    def writes_sent(self) -> int:
        return self.sent["WRITE"] + self.sent["SYNC_WRITE"] + self.sent["REG_WRITE"] + self.sent["BULK_WRITE"]


class Bus:
    """Interface. ids: the servo IDs this program expects."""
    policy: WritePolicy
    stats: BusStats
    info: dict

    def ping(self, dxl_id: int) -> tuple[int | None, int]: ...
    def read(self, dxl_id: int, addr: int, length: int) -> tuple[bytes | None, int]: ...
    def sync_read(self, addr: int, length: int, ids) -> ReadResult: ...
    def sync_write(self, addr: int, length: int, values: dict[int, bytes]) -> float: ...
    def write(self, dxl_id: int, addr: int, data: bytes) -> int: ...
    def close(self) -> None: ...


# ---------------------------------------------------------------- port ownership

def port_holders(device: str | Path) -> list[tuple[int, str]]:
    """(pid, command) of other processes with the device open (read from /proc/*/fd; nothing is changed)."""
    real = os.path.realpath(device)
    out = []
    for pid_dir in Path("/proc").iterdir():
        if not pid_dir.name.isdigit() or int(pid_dir.name) == os.getpid():
            continue
        try:
            for fd in (pid_dir / "fd").iterdir():
                if os.path.realpath(fd) == real:
                    comm = (pid_dir / "comm").read_text().strip()
                    out.append((int(pid_dir.name), comm))
                    break
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
    return out


def bus_lock(path: Path | None = None):
    """One copy of this program per bus (pattern: pac_vision.devices.camera_lock). Keep the file open."""
    path = path or Path(os.environ.get("XDG_RUNTIME_DIR") or f"/tmp/mctl-{os.getuid()}") / "manipulator_bus.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    f = open(path, "a+", encoding="utf-8")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.seek(0)
        who = f.read().strip() or "another copy"
        f.close()
        raise PortBusy(f"이 프로그램의 다른 실행({who})이 버스를 쓰고 있습니다") from None
    f.seek(0)
    f.truncate()
    f.write(f"manipulator_control (PID {os.getpid()})")
    f.flush()
    return f


def serial_environment(device: str | Path) -> dict:
    """FTDI latency timer and ModemManager state, for the logs (read-only)."""
    name = Path(os.path.realpath(device)).name
    info = {"device": os.path.realpath(device)}
    try:
        info["latency_timer_ms"] = int(Path(f"/sys/bus/usb-serial/devices/{name}/latency_timer").read_text())
    except (OSError, ValueError):
        info["latency_timer_ms"] = None
    try:
        info["modemmanager"] = subprocess.run(["systemctl", "is-active", "ModemManager"], capture_output=True,
                                              text=True, timeout=2).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        info["modemmanager"] = None
    return info


# ---------------------------------------------------------------- real bus

class DxlBus(Bus):
    def __init__(self, port: str, baudrate: int, policy: WritePolicy):
        from dynamixel_sdk import PacketHandler, PortHandler
        self.policy, self.stats = policy, BusStats()
        holders = port_holders(port)
        if holders:
            names = ", ".join(f"{c} (PID {p})" for p, c in holders)
            raise PortBusy(f"포트를 다른 프로그램이 쓰고 있습니다: {names}. 그 프로그램을 직접 닫은 뒤 다시 실행하세요")
        if not os.path.exists(port):
            raise BusError(f"포트 {port}가 없습니다 (U2D2 연결·전원 확인). 다른 장치를 자동으로 고르지 않습니다")
        self._lock = bus_lock()
        self.info = serial_environment(port)
        self.port = PortHandler(port)
        self.port.baudrate = baudrate
        if not self.port.openPort():
            self._lock.close()
            raise BusError(f"{port}를 {baudrate} bps로 열 수 없습니다")
        try:
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            self.info["tiocexcl"] = True
        except OSError as e:
            self.info["tiocexcl"] = f"failed: {e}"
        after = port_holders(port)
        if after:
            self.close()
            raise PortBusy(f"포트를 연 직후 다른 프로그램이 함께 열고 있습니다: {after}")
        self.ph = PacketHandler(2.0)
        self._guard_port()

    def _guard_port(self) -> None:
        allowed = READ_INSTRUCTIONS if self.policy.mode == "read_only" else CONTROL_INSTRUCTIONS
        raw_write = self.port.writePort

        def guarded(packet):
            inst = packet[7] if len(packet) > 7 else None
            name = INSTRUCTION_NAMES.get(inst, f"0x{inst:02X}" if inst is not None else "?")
            if inst not in allowed:
                self.stats.refused[name] += 1
                self.port.is_using = False
                raise WriteBlocked(f"instruction {name} refused in {self.policy.mode} mode (not sent)")
            self.stats.sent[name] += 1
            return raw_write(packet)
        self.port.writePort = guarded

    def ping(self, dxl_id):
        model, result, error = self.ph.ping(self.port, dxl_id)
        return (model if result == 0 else None), error

    def read(self, dxl_id, addr, length):
        data, result, error = self.ph.readTxRx(self.port, dxl_id, addr, length)
        return (bytes(data) if result == 0 else None), error

    def sync_read(self, addr, length, ids):
        ids = list(ids)
        t0 = time.monotonic()
        self.port.ser.reset_input_buffer()
        result = self.ph.syncReadTx(self.port, addr, length, ids, len(ids), False)
        data, errors = {}, {}
        if result == 0:
            while len(data) < len(ids):
                pkt, r = self.ph.rxPacket(self.port, False)
                if r != 0:
                    break
                pid = pkt[4]
                if pid in ids and pid not in data and len(pkt) >= 9 + length + 2:
                    errors[pid] = pkt[8]
                    data[pid] = bytes(pkt[9:9 + length])
        t1 = time.monotonic()
        missing = tuple(i for i in ids if i not in data)
        if (addr, length) == ct.FAST:
            self.stats.note_read(t1 - t0, not missing)
        return ReadResult(addr, length, data, errors, missing, t0, t1)

    def sync_write(self, addr, length, values):
        self.policy.check(addr, length)
        param = []
        for dxl_id, data in values.items():
            if len(data) != length:
                raise ValueError(f"ID {dxl_id}: {len(data)} bytes for a {length}-byte write")
            param += [dxl_id, *data]
        result = self.ph.syncWriteTxOnly(self.port, addr, length, param, len(param))
        if result != 0:
            raise BusError(f"sync write {addr} failed: {self.ph.getTxRxResult(result)}")
        return time.monotonic()

    def write(self, dxl_id, addr, data):
        self.policy.check(addr, len(data))
        result, error = self.ph.writeTxRx(self.port, dxl_id, addr, len(data), list(data))
        if result != 0:
            raise BusError(f"ID {dxl_id} write {addr} failed: {self.ph.getTxRxResult(result)}")
        return error

    def close(self):
        try:
            if getattr(self, "port", None) is not None and self.port.is_open:
                self.port.closePort()
        finally:
            if getattr(self, "_lock", None) is not None:
                self._lock.close()
                self._lock = None
