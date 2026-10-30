"""Read-only position recorder for the calibration steps (sign, gear, limits): logs every ID's Present Position.

Opens the bus read-only (PING / READ / SYNC_READ only; the instruction guard refuses anything else), samples the
fast block at --hz and appends one JSON line per sample to logs/calib/positions_<time>.jsonl. Nothing is written
to the servos. Stop with Ctrl+C or --duration-s.

    python tools/record_positions.py --duration-s 1200
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from manipulator_control import WS, config  # noqa: E402
from manipulator_control import control_table as ct  # noqa: E402
from manipulator_control.bus import DxlBus, WritePolicy  # noqa: E402
from manipulator_control.state import AxisFast  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--duration-s", type=float, default=1200.0)
    a = ap.parse_args()
    cfg = config.load()
    bus = DxlBus(cfg.robot["bus"]["port"], int(cfg.robot["bus"]["baudrate"]), WritePolicy("read_only"))
    out_dir = WS / "logs" / "calib"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"positions_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"recording {cfg.ids} -> {path}", flush=True)
    end = time.monotonic() + a.duration_s
    try:
        with open(path, "a", encoding="utf-8") as f:
            while time.monotonic() < end:
                t0 = time.monotonic()
                r = bus.sync_read(*ct.FAST, cfg.ids)
                axes = {i: AxisFast.from_bytes(d, r.errors.get(i, 0)) for i, d in r.data.items()}
                f.write(json.dumps({"t": round(r.t_done, 4), "wall": dt.datetime.now().isoformat(timespec="milliseconds"),
                                    "pos": {i: x.present_position for i, x in axes.items()},
                                    "vel": {i: x.present_velocity for i, x in axes.items()},
                                    "torque_hint_current": {i: x.present_current for i, x in axes.items()},
                                    "goal": {i: x.goal_position for i, x in axes.items()},
                                    "pos_traj": {i: x.position_trajectory for i, x in axes.items()},
                                    "pwm": {i: x.present_pwm for i, x in axes.items()},
                                    "temp_c": {i: x.present_temperature for i, x in axes.items()},
                                    "volt": {i: x.voltage_v for i, x in axes.items()},
                                    "alert": {i: x.alert for i, x in axes.items()},
                                    "missing": list(r.missing)}) + "\n")
                f.flush()
                time.sleep(max(0.0, 1.0 / a.hz - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        print(f"stopped; sent {dict(bus.stats.sent)}, writes {bus.stats.writes_sent}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
