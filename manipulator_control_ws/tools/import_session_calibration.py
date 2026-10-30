"""Store the calibration of a finished calibrate session from its log (for a calibration made before the program
stored calibrations itself, 2026-10-26 17:28).

Builds the same record calibrate now stores at the end (mapping, P_ref, servo settings, the last PA/PV/Goal sent to
each servo) from logs/sessions/<session>/ and writes runtime/calibration_state.json. Nothing is sent to the servos;
whether the record is usable is decided by the servos at the next prepare / teleop (or calibration-status): their
PA/PV/Goal must still be exactly the stored ones. Refused when the session did not end with all six joints held,
or a later control session (calibrate / prepare / teleop) exists.

--adopt-servo-goals (read-only Sync Reads): the servo sets Goal = Present when torque turns on, which the old log
did not record. Like calibrate now does after torque on, the read-back Goal is stored instead of the logged one,
only if every servo has torque on, exactly the logged PA/PV (non-zero: no reboot since), and a Goal within the
hold tolerance of the logged one.

    python tools/import_session_calibration.py logs/sessions/20261026_172814_hold --adopt-servo-goals
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from manipulator_control import WS, config  # noqa: E402
from manipulator_control import control_table as ct  # noqa: E402
from manipulator_control.calibration import (RECORD_SCHEMA, RECORD_SETTINGS, STORE_PATH,  # noqa: E402
                                             CalibrationStore)


def build(session: Path) -> tuple[dict, str]:
    meta = json.loads((session / "session.json").read_text(encoding="utf-8"))
    events = [json.loads(line) for line in (session / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    problems = []
    if meta.get("purpose") != "hold" or meta.get("fake"):
        problems.append(f"calibrate(hold) 실기 세션이 아님: purpose={meta.get('purpose')} fake={meta.get('fake')}")
    cal = meta.get("calibration")
    final = meta.get("final") or {}
    if not cal:
        problems.append("세션에 보정 결과가 없음")
    if final.get("fault") is not None or not all(v.startswith("토크 ON") for v in final.get("axes", {}).values()) \
            or len(final.get("axes", {})) != 6:
        problems.append(f"6축 모두 토크 ON으로 끝나지 않음: {final}")
    later = [d.name for d in sorted((WS / "logs" / "sessions").iterdir())
             if d.name > session.name and not d.name.split("_", 2)[-1].startswith(("watch", "scan"))]
    if later:
        problems.append(f"이후의 제어 세션이 있음 (서보 값이 바뀌었을 수 있음): {later}")
    if problems:
        raise SystemExit("가져오기 거부: " + "; ".join(problems))
    written = {}
    for e in events:
        if e.get("event") == "command_sent" and e.get("addr") == ct.PROFILE_GOAL[0]:
            for i, v in e["values"].items():
                written[i] = {"pa": v["pa"], "pv": v["pv"], "goal": v["goal"]}
    wall = next(e["wall"] for e in events if e.get("event") == "calibration")
    servo = {i: {k: meta["servo_settings"][i][k] for k in RECORD_SETTINGS} for i in sorted(written)}
    joints = [{"name": j["name"], "id": j["dxl_id"], "gear": j["gear"], "sign": j["sign"], "q_ref_rad": q,
               "p_ref": cal["p_ref"][str(j["dxl_id"])]} for j, q in zip(cal["joints"], cal["q_ref_rad"])]
    return {"schema": RECORD_SCHEMA, "calibration_id": cal["session_id"], "joints": joints, "servo": servo,
            "written": written, "imported_from": str(session)}, wall


def adopt(rec: dict, cfg) -> None:
    from manipulator_control.__main__ import read_snapshots
    from manipulator_control.bus import DxlBus, WritePolicy
    bus = DxlBus(cfg.robot["bus"]["port"], int(cfg.robot["bus"]["baudrate"]), WritePolicy("read_only"))
    try:
        fast, slow = read_snapshots(bus, cfg.ids)
    finally:
        sent = dict(bus.stats.sent)
        bus.close()
    if not fast.complete or not slow.complete:
        raise SystemExit(f"응답 없는 ID {sorted(set(fast.missing) | set(slow.missing))}")
    tol = cfg.m("hold", "max_error_counts")
    problems, changed = [], {}
    for i in cfg.ids:
        w, a = rec["written"][str(i)], fast[i]
        if not slow.value(i, "torque_enable"):
            problems.append(f"ID {i} 토크 OFF")
        if (a.profile_acceleration, a.profile_velocity) != (w["pa"], w["pv"]) or not (w["pa"] > 0 and w["pv"] > 0):
            problems.append(f"ID {i} PA/PV {(a.profile_acceleration, a.profile_velocity)} != 기록 {(w['pa'], w['pv'])}")
        if abs(a.goal_position - w["goal"]) > tol:
            problems.append(f"ID {i} Goal {a.goal_position} - 기록 {w['goal']} > {tol} count")
        elif a.goal_position != w["goal"]:
            changed[i] = (w["goal"], a.goal_position)
    print(f"서보 읽기 (읽기 전용, 송신 {sent})")
    if problems:
        raise SystemExit("서보 값 반영 거부: " + "; ".join(problems))
    for i, (old, new) in changed.items():
        rec["written"][str(i)]["goal"] = new
    rec["adopted_servo_goals"] = {str(i): {"logged": o, "read": n} for i, (o, n) in changed.items()}
    print(f"Goal 반영 (기록 -> 서보 읽기값): {changed or '차이 없음'}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--replace", action="store_true", help="replace an existing stored calibration")
    ap.add_argument("--adopt-servo-goals", action="store_true", help="read-only: store the Goal the servos hold now")
    a = ap.parse_args()
    store = CalibrationStore(STORE_PATH)
    if store.load() is not None and not a.replace:
        raise SystemExit(f"{store.path}가 이미 있습니다 (바꾸려면 --replace)")
    rec, wall = build(a.session.resolve())
    cfg = config.load()
    mismatch = [j.name for j, r in zip(cfg.joints, rec["joints"])
                if (r["id"], r["gear"], r["sign"]) != (j.dxl_id, j.gear, j.sign) or abs(r["q_ref_rad"] - j.q_ref_rad) > 1e-9]
    if mismatch:
        raise SystemExit(f"현재 config/robot.yaml과 보정 당시 설정이 다름: {mismatch}")
    if a.adopt_servo_goals:
        adopt(rec, cfg)
    print(store.save(rec, str(a.session), calibrated_wall=wall))
    print(json.dumps({k: rec[k] for k in ("calibration_id", "joints", "written")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
