"""Session calibration: link the configured reference pose q_ref to this session's counts P_ref.

Enter/Start does not detect the pose: it states "the arm is at the marks now". The program checks that the arm is
still and every axis answered, then stores P_ref = median of the window. A Calibration is created only from an
accepted live window, or (resume) from the stored record of the newest calibrate when the servos prove that their
count frame was not reset since: P_ref is meaningless after a power cycle / reboot (Present Position restarts inside
one turn), so the record is used only while every servo still holds exactly the Profile Acceleration / Profile
Velocity this program wrote last (a reboot clears PV/PA to 0 and turns torque off; this program never writes 0)
and the joint mapping and servo settings are the ones the calibration was made with. Goal Position is not
evidence: the servo sets it to the present position while torque is off and when torque turns on (FW 50, seen).

runtime/calibration_state.json holds only the newest calibration: a new calibrate deletes it at its start and writes
its own when every joint is held; any later session updates the evidence (the PA/PV read back) when the arm holds
still and at exit, and deletes the file on evidence of a reset or when the communication was lost (user rule
2026-10-26: after a lost link, calibrate again).
"""
from __future__ import annotations

import json
import os
import statistics
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import WS
from .state import FastSnapshot
from .units import JointMap

STORE_PATH = WS / "runtime" / "calibration_state.json"
FAKE_STORE_PATH = WS / "runtime" / "fake_calibration_state.json"

_TOKEN = object()


class Calibration:
    def __init__(self, token, session_id: str, t: float, joints: tuple[JointMap, ...], p_ref: dict[int, int],
                 q_ref: tuple[float, ...], spread: dict[int, int]):
        if token is not _TOKEN:
            raise TypeError("Calibration is created only from an accepted live window or a verified stored record")
        self.session_id, self.t, self.joints = session_id, t, joints
        self.p_ref, self.q_ref, self.spread = dict(p_ref), tuple(q_ref), dict(spread)

    def q(self, snap: FastSnapshot, field: str = "present_position") -> np.ndarray:
        return np.array([j.q_rad(getattr(snap[j.dxl_id], field), self.p_ref[j.dxl_id], qr)
                         for j, qr in zip(self.joints, self.q_ref)])

    def q_of(self, counts: dict[int, int]) -> np.ndarray:
        return np.array([j.q_rad(counts[j.dxl_id], self.p_ref[j.dxl_id], qr) for j, qr in zip(self.joints, self.q_ref)])

    def counts(self, q) -> dict[int, int]:
        return {j.dxl_id: j.p_goal(float(v), self.p_ref[j.dxl_id], qr) for j, v, qr in zip(self.joints, q, self.q_ref)}

    @classmethod
    def merge(cls, parts: list["Calibration"], t: float) -> "Calibration":
        """One calibration from per-joint ones (sequential alignment); each part came from its own live window."""
        if not all(isinstance(p, Calibration) for p in parts):
            raise TypeError("only accepted per-joint calibrations can be merged")
        joints = tuple(j for p in parts for j in p.joints)
        p_ref = {k: v for p in parts for k, v in p.p_ref.items()}
        spread = {k: v for p in parts for k, v in p.spread.items()}
        q_ref = tuple(q for p in parts for q in p.q_ref)
        return cls(_TOKEN, uuid.uuid4().hex[:12], t, joints, p_ref, q_ref, spread)

    def to_log(self) -> dict:
        return {"session_id": self.session_id, "p_ref": self.p_ref, "q_ref_rad": list(self.q_ref),
                "spread_counts": self.spread, "joints": [j.__dict__ for j in self.joints],
                "note": "stored as the newest calibration; reused only while the servos prove no reset"}

    def record(self, settings: dict[int, dict], written: dict[int, tuple[int, int, int]]) -> dict:
        """The stored form: mapping, P_ref, the settings it was made with and the evidence (last PA/PV/Goal written)."""
        return {"schema": RECORD_SCHEMA, "calibration_id": self.session_id,
                "joints": [{"name": j.name, "id": j.dxl_id, "gear": j.gear, "sign": j.sign, "q_ref_rad": qr,
                            "p_ref": self.p_ref[j.dxl_id]} for j, qr in zip(self.joints, self.q_ref)],
                "servo": {str(i): {k: settings[i][k] for k in RECORD_SETTINGS} for i in self.p_ref},
                "written": {str(i): {"pa": pa, "pv": pv, "goal": goal} for i, (pa, pv, goal) in sorted(written.items())}}

    @classmethod
    def resume(cls, rec: dict | None, joints_cfg, fast: FastSnapshot, slow, now: float, hold_tol_counts: int):
        """(Calibration, ids with torque off) from the stored record, or Reject with every reason.

        joints_cfg: config JointCfg tuple; fast / slow: fresh complete reads of this session."""
        if not rec:
            return Reject("저장된 보정이 없습니다")
        if rec.get("schema") != RECORD_SCHEMA:
            return Reject(f"저장 형식 {rec.get('schema')} != {RECORD_SCHEMA}")
        problems, maps, p_ref, q_ref, free, regs_bad = [], [], {}, [], [], []
        stored = {j["name"]: j for j in rec.get("joints", [])}
        for jc in joints_cfg:
            j = stored.get(jc.name)
            if j is None:
                problems.append(f"{jc.name}: 저장된 보정에 없음")
                continue
            if (j["id"], float(j["gear"]), j["sign"]) != (jc.dxl_id, jc.gear, jc.sign) or jc.q_ref_rad is None or \
                    abs(j["q_ref_rad"] - jc.q_ref_rad) > 1e-9:
                problems.append(f"{jc.name}: 설정(ID/감속비/부호/기준 각도)이 보정할 때와 다름")
                continue
            i = jc.dxl_id
            if i not in fast.axes or i not in slow.axes:
                problems.append(f"ID {i}: 응답 없음")
                continue
            a, st = fast[i], slow.axes[i]
            for k in RECORD_SETTINGS:
                if st[k] != rec["servo"][str(i)][k]:
                    problems.append(f"ID {i}: {k} {rec['servo'][str(i)][k]} -> {st[k]}")
            w = rec["written"].get(str(i))
            now_regs = (a.profile_acceleration, a.profile_velocity)
            if w is None or now_regs != (w["pa"], w["pv"]) or not (w["pa"] > 0 and w["pv"] > 0):
                regs_bad.append(f"ID {i} {now_regs} != {None if w is None else (w['pa'], w['pv'])}")
            elif st["torque_enable"] and abs(a.present_position - a.goal_position) > hold_tol_counts:
                problems.append(f"ID {i}: 토크 ON인데 목표에서 {a.present_position - a.goal_position} count 벗어나 있음")
            if not st["torque_enable"]:
                free.append(i)
            maps.append(JointMap(jc.name, i, jc.gear, jc.sign))
            p_ref[i] = int(j["p_ref"])
            q_ref.append(jc.q_ref_rad)
        if regs_bad:
            problems.append("PA/PV 읽기값이 마지막으로 보낸 값과 다름: " + ", ".join(regs_bad) + " (PV/PA 0 = 전원 "
                            "재인가·재부팅 흔적, 그 밖에는 다른 프로그램의 쓰기 - 엔코더 기준이 바뀌었을 수 있음)")
        if problems:
            return Reject("; ".join(problems))
        return cls(_TOKEN, rec["calibration_id"], now, tuple(maps), p_ref, tuple(q_ref), {}), free


@dataclass
class Reject:
    reason: str


RECORD_SCHEMA = 1
RECORD_SETTINGS = ("operating_mode", "drive_mode", "homing_offset")


class CalibrationStore:
    """The one stored calibration (the newest calibrate). Written atomically; discard deletes it."""

    def __init__(self, path: Path):
        self.path = Path(path)

    def load(self) -> dict | None:
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as e:
            return {"schema": None, "error": repr(e)}

    def save(self, rec: dict, session: str | None = None, calibrated_wall: str | None = None) -> str:
        old = self.load() or {}
        now = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        same = old.get("calibration_id") == rec["calibration_id"]
        out = dict(rec, calibrated_wall=old.get("calibrated_wall") if same else calibrated_wall or now,
                   calibrated_session=old.get("calibrated_session") if same else session, updated_wall=now)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)
        return f"saved {self.path}"

    def discard(self) -> str:
        try:
            self.path.unlink()
            return f"deleted {self.path}"
        except FileNotFoundError:
            return "nothing stored"


class CalibrationWindow:
    """Consecutive complete snapshots; a missing axis restarts the window."""

    def __init__(self, ids, samples: int, max_spread_counts: int, max_velocity_raw: int):
        self.ids, self.n = tuple(ids), samples
        self.max_spread, self.max_vel = max_spread_counts, max_velocity_raw
        self.snaps: list[FastSnapshot] = []
        self.restarts = 0
        self._last_seq = None

    def add(self, snap: FastSnapshot | None) -> None:
        if snap is None or snap.seq == self._last_seq:
            return
        self._last_seq = snap.seq
        if not snap.complete or any(i not in snap.axes for i in self.ids):
            self.snaps.clear()
            self.restarts += 1
            return
        self.snaps.append(snap)

    @property
    def full(self) -> bool:
        return len(self.snaps) >= self.n

    def evaluate(self, joints: tuple[JointMap, ...], q_ref, now: float):
        """Calibration or Reject (only call when full)."""
        snaps = self.snaps[-self.n:]
        problems = []
        p_ref, spread = {}, {}
        for j in joints:
            pos = [s[j.dxl_id].present_position for s in snaps]
            spread[j.dxl_id] = max(pos) - min(pos)
            if spread[j.dxl_id] > self.max_spread:
                problems.append(f"{j.name}(ID {j.dxl_id}) 움직임 {spread[j.dxl_id]} count > {self.max_spread}")
            vmax = max(abs(s[j.dxl_id].present_velocity) for s in snaps)
            if vmax > self.max_vel:
                problems.append(f"{j.name}(ID {j.dxl_id}) 속도 {vmax} raw > {self.max_vel}")
            if any(s[j.dxl_id].alert for s in snaps):
                problems.append(f"{j.name}(ID {j.dxl_id}) 하드웨어 경보")
            p_ref[j.dxl_id] = int(statistics.median_low(pos)) if len(pos) % 2 == 0 else int(statistics.median(pos))
        if problems:
            return Reject("; ".join(problems))
        return Calibration(_TOKEN, uuid.uuid4().hex[:12], now, joints, p_ref, tuple(q_ref), spread)
