"""Entry points.

  scan       read-only: IDs, every register decoded, Sync Read timing, rest noise, prepare refusal reasons
  watch      read-only: live STATE in MuJoCo; relative change until calibrated (Enter = read-only calibration)
  calibrate  joint 1..6 one at a time: [direction check if the sign is unknown] -> align with the SIM reference
             pose -> Enter -> that joint's origin is registered and it is held with torque on (no motion).
             The result replaces the stored calibration (runtime/calibration_state.json, only the newest is kept).
  prepare    no alignment: the stored calibration (verified on the servos) -> HOME (SIM path preview + y) -> READY
             (--hold-only: as calibrate)
  teleop     prepare, then the real arm follows an EE target dragged with the gizmo in the SIM window (IK)
  pinch      prepare, then the pinch simulator (pinch_sim --real-arm) connects; after y at this terminal the real arm
             runs the joint trajectories the simulator validated (checked again here, streamed: pinch_traj.py)
  calibration-status  read-only: is the stored calibration usable now (servo read-back vs the record)?
  pose     no hardware: show a joint pose / sweep one joint's + direction in MuJoCo (for the calibration guide)
  preview-home  no hardware: the HOME plan from the configured reference pose, printed and replayed in MuJoCo

Real hardware is used by scan / watch (read-only), calibrate, prepare and teleop. calibrate needs q_ref in
config/robot.yaml (unknown signs are found in the session); prepare / teleop also need every sign and the ranges,
and a stored calibration the servos still match (otherwise they refuse without writing: run calibrate). All of
them refuse before opening the port when the config is incomplete, and need an interactive terminal: they only act
on a person at the arm.
--fake runs the same code on fake_bus with config/fake_robot.yaml (made-up values, logs under logs/fake/,
runtime/fake_calibration_state.json). A fake bus starts powered on fresh: prepare / teleop --fake need
--fake-calibrated (the fake servos start as a calibrate leaves them, with the matching stored record).
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

from . import config as config_mod


def _load(args, fake: bool):
    path = Path(args.config) if args.config else (config_mod.FAKE_YAML if fake else config_mod.ROBOT_YAML)
    cfg = config_mod.load(path)
    if fake and cfg.kind != "fake":
        sys.exit(f"--fake needs a kind: fake config, got {path}")
    if not fake and cfg.kind != "hardware":
        sys.exit(f"{path} is a fake config; real hardware uses config/robot.yaml")
    return cfg


def _open_bus(cfg, mode: str, fake: bool, clock=time.monotonic):
    from .bus import DxlBus, WritePolicy
    policy = WritePolicy(mode, allow_operating_mode_write=bool(cfg.expect("allow_operating_mode_write")))
    if fake:
        return make_fake_bus(cfg, policy, clock)
    return DxlBus(cfg.robot["bus"]["port"], int(cfg.robot["bus"]["baudrate"]), policy)


def make_fake_bus(cfg, policy, clock):
    from .fake_bus import FakeBus
    fk = cfg.robot["fake"]
    bus = FakeBus(clock, cfg.ids, policy, power_on_counts=fk["power_on_counts"])
    for i, off, sag in zip(cfg.ids, fk["aligned_offset_counts"], fk["sag_counts_per_s"]):
        s = bus.servos[i]
        s.pos += off                         # the "hand" turned the joint (multi-turn) before Enter
        s.prof_x = s.pos
        s.regs["goal_position"] = int(s.pos)  # a held servo holds where it is
        s.sag_v = sag
    return bus


def fake_calibrated(bus, cfg, store) -> None:
    """--fake-calibrated: the fake servos as a calibrate session leaves them (every joint held at the reference pose,
    hold PV/PA) and the matching stored record, so prepare / teleop --fake can resume from it."""
    from .calibration import RECORD_SCHEMA, RECORD_SETTINGS
    h = cfg.motion["hold"]
    joints, written, servo = [], {}, {}
    for j in cfg.joints:
        s = bus.servos[j.dxl_id]
        p = int(s.pos)
        s.pos = s.prof_x = float(p)
        s.prof_v = 0.0
        s.regs.update(torque_enable=1, profile_acceleration=h["profile_acceleration_raw"],
                      profile_velocity=h["profile_velocity_raw"], goal_position=p)
        joints.append({"name": j.name, "id": j.dxl_id, "gear": j.gear, "sign": j.sign, "q_ref_rad": j.q_ref_rad,
                       "p_ref": p})
        written[str(j.dxl_id)] = {"pa": h["profile_acceleration_raw"], "pv": h["profile_velocity_raw"], "goal": p}
        servo[str(j.dxl_id)] = {k: s.regs[k] for k in RECORD_SETTINGS}
    store.save({"schema": RECORD_SCHEMA, "calibration_id": "fake-calibrated", "joints": joints, "servo": servo,
                "written": written}, "--fake-calibrated")


def cmd_scan(args) -> int:
    from .bus import BusError
    from .scan import print_summary, run_scan, save
    cfg = _load(args, args.fake)
    try:
        bus = _open_bus(cfg, "read_only", args.fake)
    except BusError as e:
        print(f"scan 중단: {e}")
        return 3
    try:
        lo, hi = (int(x) for x in args.ping_range.split("-"))
        rep = run_scan(bus, cfg, range(lo, hi + 1), args.timing_s, args.noise_s)
    finally:
        bus.close()
    path = save(rep, args.fake)
    print_summary(rep)
    print(f"저장: {path}")
    return 0 if not rep["missing_ids"] and rep["writes_sent"] == 0 else 1


def read_snapshots(bus, ids):
    """One slow and one fast Sync Read (read-only tools)."""
    from . import control_table as ct
    from .state import AxisFast, FastSnapshot, SlowSnapshot
    r = bus.sync_read(*ct.SLOW, ids)
    slow = SlowSnapshot.decode(1, r.t_done, r.data, r.errors, r.missing)
    r = bus.sync_read(*ct.FAST, ids)
    fast = FastSnapshot(2, r.t_done, r.t_done - r.t_start,
                        {i: AxisFast.from_bytes(d, r.errors.get(i, 0)) for i, d in r.data.items()}, r.missing)
    return fast, slow


def cmd_calibration_status(args) -> int:
    """Read-only: the same check prepare / teleop do at their start, without writing anything."""
    import numpy as np
    from .bus import BusError
    from .calibration import FAKE_STORE_PATH, STORE_PATH, Calibration, CalibrationStore, Reject
    from .supervisor import _k
    cfg = _load(args, args.fake)
    store = CalibrationStore(FAKE_STORE_PATH if args.fake else STORE_PATH)
    try:
        bus = _open_bus(cfg, "read_only", args.fake)
    except BusError as e:
        print(f"확인 중단: {e}")
        return 3
    if args.fake and args.fake_calibrated:
        fake_calibrated(bus, cfg, store)
    rec = store.load()
    try:
        fast, slow = read_snapshots(bus, cfg.ids)
    finally:
        sent = dict(bus.stats.sent)
        bus.close()
    print(f"저장 파일: {store.path}")
    if rec:
        print(f"  calibrate 시각 {rec.get('calibrated_wall')}, 세션 {rec.get('calibrated_session')}, "
              f"마지막 갱신 {rec.get('updated_wall')}")
    if not fast.complete or not slow.complete:
        print(f"응답 없는 ID: {sorted(set(fast.missing) | set(slow.missing))}")
        return 4
    tol = round(math.radians(cfg.m("ready", "drift_deg")) * _k(min(j.gear for j in cfg.joints)))
    res = Calibration.resume(rec, cfg.joints, fast, slow, time.monotonic(), tol)
    print(f"(읽기 전용: 송신 {sent})")
    if isinstance(res, Reject):
        print(f"사용 불가: {res.reason}")
        print("-> calibrate로 다시 등록해야 prepare / teleop을 실행할 수 있습니다")
        return 1
    cal, free = res
    print("사용 가능: prepare / teleop이 이 보정으로 바로 시작합니다 (HOME 미리보기부터)")
    for j, q in zip(cfg.joints, np.degrees(cal.q(fast))):
        print(f"  {j.name}: {q:8.2f} deg  토크 {'OFF - 시작 때 y로 그 자리 고정' if j.dxl_id in free else 'ON'}")
    return 0


def _fake_hooks(cfg, specs, bus):
    """--fake-event kind@seconds[:arg]  (start, stop, quit, confirm, jog:+x.., linkdown, linkup, rxloss, rxok,
    reboot:ID, stuck:ID, hwerror:ID, silent:ID)."""
    events = []
    for spec in specs or []:
        kind, _, rest = spec.partition("@")
        t, _, arg = rest.partition(":")
        events.append([float(t), kind, (int(arg) if arg.isdigit() else arg) if arg else None, False])
    t0 = [None]

    def hook(now, runner):
        t0[0] = now if t0[0] is None else t0[0]
        for ev in events:
            if ev[3] or now - t0[0] < ev[0]:
                continue
            ev[3] = True
            kind, i = ev[1], ev[2]
            runner.log.event("fake_injection", kind=kind, id=i, t=round(now - t0[0], 3))
            if kind in ("stop", "quit", "start"):
                runner.inputs.put(kind.upper())
            elif kind == "confirm":
                runner.inputs.put("CONFIRM")
            elif kind == "jog":
                runner.inputs.put("JOG", i)
            elif kind == "confirm_in":           # y once the session is in state i (e.g. ARM_CONFIRM)
                ev[3] = False
                if runner.sup.state.value == i and not ev[4:]:
                    ev.append(True)
                    runner.inputs.put("CONFIRM")
                elif runner.sup.state.value != i:
                    del ev[4:]
            elif kind == "linkdown":
                bus.link_down = True
            elif kind == "linkup":
                bus.link_down = False
            elif kind == "rxloss":
                bus.rx_loss = True
            elif kind == "rxok":
                bus.rx_loss = False
            elif kind == "silent":
                bus.silent_ids.add(i)
            elif kind == "reboot":
                bus.servos[i].reboot()
            elif kind == "stuck":
                bus.servos[i].stuck = True
            elif kind == "hwerror":
                bus.servos[i].hardware_error(0x20)
    return hook


def run_session(args, purpose: str) -> int:
    from .bus import BusError
    from .calibration import FAKE_STORE_PATH, STORE_PATH, CalibrationStore
    from .logs import SessionLog
    from .model import PathChecker, load_viz, tcp_position
    from .runner import Runner
    from .supervisor import Supervisor
    from .terminal import ScriptedInput, TerminalInput, print_status

    fake = args.fake
    cfg = _load(args, fake)
    if getattr(args, "home_speed_deg_s", None) is not None:
        v = float(args.home_speed_deg_s)
        if not 0 < v <= cfg.m("limits", "v_cap_deg_s"):
            sys.exit(f"--home-speed-deg-s must be in (0, {cfg.m('limits', 'v_cap_deg_s')}]")
        cfg.motion["home"]["speed_deg_s"] = v
    if purpose != "watch":
        blockers = config_mod.motion_blockers(cfg, purpose)
        if blockers:
            print(f"{ {'hold': 'calibrate', 'home': 'prepare', 'teleop': 'teleop', 'pinch': 'pinch'}[purpose]} 거부 - 포트를 열지 않았습니다 "
                  "(토크 ON·이동 없음):")
            for b in blockers:
                print(f"  - {b}")
            print("값을 정하는 방법: docs/CALIBRATION_GUIDE.md, 설정: config/robot.yaml")
            return 2
        if not fake and (args.auto_start is not None or not sys.stdin.isatty()):
            sys.exit("실기 calibrate/prepare는 대화형 터미널에서 사람이 Enter를 눌러야 진행합니다 (무인 자동 시작 금지)")

    log = SessionLog(purpose, fake)
    clock = time.monotonic
    try:
        bus = _open_bus(cfg, "read_only" if purpose == "watch" else "control", fake, clock)
    except BusError as e:
        print(f"중단: {e}")
        log.event("open_failed", error=str(e))
        return 3
    store = CalibrationStore(FAKE_STORE_PATH if fake else STORE_PATH)
    if fake and getattr(args, "fake_calibrated", False):
        fake_calibrated(bus, cfg, store)
    saved = store.load() if purpose != "watch" else None
    m, arms = load_viz()
    import mujoco
    d = mujoco.MjData(m)
    checker = PathChecker(m, arms["measured"])
    from .ik import IKSolver
    tc = cfg.motion["teleop"]
    ik = IKSolver(m, arms["measured"], max_iters=tc["ik_max_iters"], pos_tol_m=tc["ik_pos_tol_m"],
                  rot_tol_deg=tc["ik_rot_tol_deg"])
    pinch_kw, link, pool = {}, None, None
    if purpose == "pinch":
        from concurrent.futures import ThreadPoolExecutor
        from .link import ArmLink
        from .pinch_traj import Acceptance
        acc = Acceptance.from_config(cfg, scene_sha())
        pc = cfg.motion["pinch"]
        need = 2 / cfg.m("loop", "read_hz") + acc.v_cap_rad_s / (2 * float(acc.a_stream_rad_s2.min()))
        if pc["lead_s"] < need:
            sys.exit(f"motion.yaml pinch.lead_s {pc['lead_s']} < {need:.3f} s (2/read_hz + v_cap/2a)")
        pool = ThreadPoolExecutor(1, thread_name_prefix="traj-check")
        pinch_kw = dict(acceptance=acc, check_submit=pool.submit,
                        traj_path_checker=PathChecker(m, arms["measured"]).check,      # its own MjData (worker thread)
                        speed_scale=args.speed_scale, check_only=args.check_only)
        hello = {"kind": cfg.kind, "robot_sha": cfg.robot_sha, "model_sha": acc.model_sha,
                 "joints": [j.name for j in cfg.joints], "limits_deg": acc.hello()["limits_deg"],
                 "home_q_rad": [float(x) for x in cfg.home_q_rad], "speed_scale": args.speed_scale,
                 "check_only": args.check_only, "accept": acc.hello()}
        link = ArmLink(hello, lambda: runner.view(), ping_timeout_s=pc["ping_timeout_s"], state_hz=pc["state_hz"],
                       max_samples=int(pc["max_duration_s"] / pc["dt_range_s"][0]) + 2, address=args.link, log=log)
    sup = Supervisor(cfg, purpose, log, fk=lambda q: tcp_position(m, d, arms["measured"], q),
                     path_checker=checker.check, ik=ik, saved=saved, **pinch_kw)
    log.session(purpose=purpose, config=str(cfg.robot_path), config_sha=cfg.robot_sha, motion=cfg.motion,
                fake=fake, model_manifest=str(Path("models/derived/manifest.json")))

    script = [(args.auto_start, "START")] if fake and args.auto_start is not None else []
    inputs_cm = TerminalInput() if sys.stdin.isatty() and not script else None
    inputs = inputs_cm or ScriptedInput(script, clock)
    hooks = [_fake_hooks(cfg, args.fake_event, bus)] if fake else []
    runner = Runner(bus, sup, log, inputs, clock, hooks,
                    save_signs=lambda signs: config_mod.save_signs(cfg.robot_path, signs),
                    store=None if purpose == "watch" else store, link=link)
    exit_code = 0
    last: dict = {}
    viewer = None
    panel = None
    try:
        if inputs_cm is not None:
            inputs_cm.__enter__()
        if not runner.connect():
            print_status(runner.view(), {})
            return 4
        if link is not None:
            link.start()
            print(f"pinch 연결: {link.address} (시뮬: python -m pinch_sim.app --real-arm ...)", flush=True)
        th = threading.Thread(target=runner.run, name="bus-loop", daemon=True)
        th.start()
        status = threading.Thread(target=_status_loop, args=(runner, last), daemon=True)
        status.start()
        settings = {"frame": "base"}                     # gizmo axes, chosen in the panel, used by the viewer
        if purpose in ("teleop", "pinch") and not args.no_panel:
            from .panel import ControlPanel
            panel = ControlPanel(inputs.put, runner.view, lambda: sup.done or not th.is_alive(), settings, purpose)
            panel.start()
        if args.no_viewer:
            while th.is_alive() and not sup.done:
                time.sleep(0.05)
        else:
            from .viz import StateViewer
            viewer = StateViewer(not args.no_cmd_ghost, not args.no_target_ghost, settings)
            viewer.run(runner.view, runner.request_quit, lambda: sup.done or not th.is_alive(),
                       on_target=(lambda pos, quat: inputs.put("TARGET", (pos, quat))) if purpose == "teleop" else None)
            # after the window closed: the bus loop finishes the exit path (stop confirmation if moving)
            deadline = time.monotonic() + 30
            while th.is_alive() and not sup.done and time.monotonic() < deadline:
                time.sleep(0.05)
        th.join(timeout=2)
        print_status(runner.view(), last)
    except KeyboardInterrupt:
        print("\nCtrl+C: 일반 종료 절차 (이동 중이면 정지 확인 후 유지). 한 번 더 누르면 확인 없이 종료")
        runner.request_quit()
        deadline = time.monotonic() + 30
        while not sup.done and time.monotonic() < deadline:
            try:
                time.sleep(0.05)
            except KeyboardInterrupt:
                print("\n두 번째 Ctrl+C: 정지 확인 없이 종료합니다 - 최종 상태 미확인")
                exit_code = 130
                break
        print_status(runner.view(), last)
    finally:
        runner.stop.set()
        if panel is not None:
            panel.close()
        if link is not None:
            link.close()
        if pool is not None:
            pool.shutdown(wait=False, cancel_futures=True)
        rep = sup.final_report or sup._final_report()
        print("\n최종 상태:")
        for i, s in rep["axes"].items():
            print(f"  ID {i}: {s}")
        print(f"  {rep['note']}")
        print(f"  송신 명령 {dict(bus.stats.sent)}; 로그 {log.dir}")
        ds = sorted(bus.stats.read_durations)
        read_stats = {"fast_reads": len(ds), "failed": bus.stats.failed_reads,
                      "p50_ms": round(ds[len(ds) // 2] * 1000, 2) if ds else None,
                      "p99_ms": round(ds[min(len(ds) - 1, int(0.99 * len(ds)))] * 1000, 2) if ds else None,
                      "max_ms": round(ds[-1] * 1000, 2) if ds else None}
        print(f"  읽기 통계 {read_stats}")
        if purpose != "watch":
            rec = store.load()
            print(f"  저장된 보정: {store.path} ({'calibrate ' + str(rec.get('calibrated_wall')) if rec else '없음 - 다음 '
                  'prepare / teleop 전에 calibrate 필요'})")
        log.session(final=rep, instructions_sent=dict(bus.stats.sent), read_stats=read_stats)
        bus.close()
        if inputs_cm is not None:
            inputs_cm.__exit__(None, None, None)
    if sup.fault is not None and exit_code == 0:
        exit_code = 5
    return exit_code


def scene_sha() -> str:
    """sha256 of the simulator's generated scene in the model copy: the pinch simulator must load the same file."""
    import hashlib
    from .model import UPSTREAM
    return hashlib.sha256((UPSTREAM / "mujoco_sim" / "models" / "generated" / "pinch_scene.xml").read_bytes()).hexdigest()


def _status_loop(runner, last) -> None:
    from .terminal import print_status
    while not runner.sup.done and not runner.stop.is_set():
        print_status(runner.view(), last)
        time.sleep(0.2)


def cmd_pose(args) -> int:
    import mujoco
    import mujoco.viewer
    import numpy as np
    from .model import JOINTS, load_viz
    m, arms = load_viz()
    d = mujoco.MjData(m)
    q = np.radians([float(x) for x in args.q_deg.split(",")]) if args.q_deg else np.zeros(6)
    for arm in ("cmd", "target"):
        m.geom_rgba[arms[arm].geoms, 3] = 0.0
        m.site_rgba[arms[arm].tcp_site, 3] = 0.0
    m.geom_rgba[[g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body("ee_target").id], 3] = 0.0
    ghost = np.radians([float(x) for x in args.ghost_deg.split(",")]) if args.ghost_deg else None
    if ghost is not None:                         # second pose as the green ghost (visual geoms only)
        g = arms["target"].geoms
        m.geom_rgba[g, 3] = np.where(m.geom_group[g] == 2, 0.45, 0.0)
        d.qpos[arms["target"].qpos] = ghost
    if args.png:
        return _render_png(m, d, arms, q, args)
    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
        t0 = time.monotonic()
        while viewer.is_running():
            qq = q.copy()
            if args.sweep:
                qq[args.sweep - 1] += math.radians(30) * 0.5 * (1 - math.cos(2 * math.pi * (time.monotonic() - t0) / 4))
            with viewer.lock():
                d.qpos[arms["measured"].qpos] = qq
                if ghost is not None:
                    d.qpos[arms["target"].qpos] = ghost
                mujoco.mj_kinematics(m, d)
            viewer.sync()
            time.sleep(1 / 30)
    print("joints:", ", ".join(JOINTS))
    return 0


class _PreviewCalibration:
    """Preview only (no hardware): P_ref = 0 at q_ref. Not a calibration.Calibration and never used to move."""

    def __init__(self, joints):
        self.maps = [j.joint_map() for j in joints]
        self.q_ref = [j.q_ref_rad for j in joints]

    def q_of(self, counts):
        import numpy as np
        return np.array([m.q_rad(counts[m.dxl_id], 0, qr) for m, qr in zip(self.maps, self.q_ref)])

    def counts(self, q):
        return {m.dxl_id: m.p_goal(float(v), 0, qr) for m, v, qr in zip(self.maps, q, self.q_ref)}


def cmd_preview_home(args) -> int:
    """Plan reference pose -> HOME exactly as prepare would (integer PV/PA), print it, replay it in MuJoCo."""
    import dataclasses
    import json
    import mujoco
    import numpy as np
    from .model import PathChecker, load_viz, tcp_position
    from .planner import plan_home
    cfg = _load(args, args.fake)
    if args.home_speed_deg_s is not None:
        cfg.motion["home"]["speed_deg_s"] = float(args.home_speed_deg_s)
    missing = [j.name for j in cfg.joints if j.q_ref_rad is None]
    if missing:
        print(f"q_ref_deg 미확정: {missing} - 기준 자세를 모르면 경로를 계산할 수 없습니다")
        return 2
    joints = [dataclasses.replace(j, sign=j.sign if j.sign is not None else 1) for j in cfg.joints]
    if any(j.sign is None for j in cfg.joints):
        print("주의: sign 미확정 축은 +1로 가정 (각도 경로는 부호와 무관, raw 값만 달라짐)")
    if any(j.limit_rad is None for j in joints):
        print("주의: limit_deg 미확정 - 범위 검사 없이 경로만 보여줍니다 (prepare는 이 상태로 실행되지 않음)")
        joints = [dataclasses.replace(j, limit_rad=j.limit_rad or (-math.pi, math.pi)) for j in joints]
    m, arms = load_viz()
    d = mujoco.MjData(m)
    checker = PathChecker(m, arms["measured"])
    cal = _PreviewCalibration(joints)
    p0 = cal.counts([j.q_ref_rad for j in joints])
    h = cfg.motion["hold"]
    plan = plan_home(joints, cal, p0, cfg.home_q_rad, cfg.motion, hold_pv=h["profile_velocity_raw"],
                     hold_pa=h["profile_acceleration_raw"], path_checker=checker.check,
                     fk=lambda q: tcp_position(m, d, arms["measured"], q))
    print(json.dumps(plan.summary(), ensure_ascii=False, indent=1))
    if plan.errors:
        print("이 경로는 prepare에서 거부됩니다:", plan.errors)
    if args.no_viewer:
        return 0 if not plan.errors else 1
    import mujoco.viewer
    m.geom_rgba[arms["cmd"].geoms, 3] = 0.0
    m.site_rgba[arms["cmd"].tcp_site, 3] = 0.0
    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
        t0 = time.monotonic()
        while viewer.is_running():
            t = (time.monotonic() - t0) % (plan.T_max + 2.0)
            with viewer.lock():
                d.qpos[arms["measured"].qpos] = plan.predict(min(t, plan.T_max))
                d.qpos[arms["target"].qpos] = np.array(cfg.home_q_rad)
                mujoco.mj_kinematics(m, d)
            viewer.sync()
            time.sleep(1 / 30)
    return 0 if not plan.errors else 1


def _render_png(m, d, arms, q, args) -> int:
    import mujoco
    import numpy as np
    d.qpos[arms["measured"].qpos] = q
    frames = []
    if getattr(args, "ghost_deg", None):
        d.qpos[arms["target"].qpos] = np.radians([float(x) for x in args.ghost_deg.split(",")])
    if args.sweep:
        # target ghost shows the + direction: joint sweep +30 deg
        m.geom_rgba[arms["target"].geoms, 3] = np.where(m.geom_group[arms["target"].geoms] == 2, 0.45, 0.0)
        q2 = q.copy()
        q2[args.sweep - 1] += math.radians(30)
        d.qpos[arms["target"].qpos] = q2
    mujoco.mj_kinematics(m, d)
    with mujoco.Renderer(m, 720, 960) as r:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (0.0, 0.0, 0.28)
        cam.distance, cam.azimuth, cam.elevation = 1.25, float(args.azimuth), float(args.elevation)
        r.update_scene(d, cam)
        frames.append(r.render())
    from .pngio import write_png
    write_png(args.png, frames[0])
    print(f"wrote {args.png}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m manipulator_control", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("scan", help="read-only scan (no motor writes)")
    p.add_argument("--config")
    p.add_argument("--fake", action="store_true")
    p.add_argument("--ping-range", default="0-20")
    p.add_argument("--timing-s", type=float, default=5.0)
    p.add_argument("--noise-s", type=float, default=3.0)
    for name, helptext in (("watch", "read-only live state"),
                           ("calibrate", "joint-by-joint alignment with the SIM reference pose, each joint held"),
                           ("prepare", "stored calibration, then HOME -> READY (no alignment)"),
                           ("teleop", "prepare, then the arm follows an EE target (SIM gizmo drag / keys), IK"),
                           ("pinch", "prepare, then the arm runs the trajectories the pinch simulator validated")):
        p = sub.add_parser(name, help=helptext)
        p.add_argument("--config")
        p.add_argument("--fake", action="store_true")
        p.add_argument("--no-viewer", action="store_true")
        p.add_argument("--no-cmd-ghost", action="store_true")
        p.add_argument("--no-target-ghost", action="store_true")
        p.add_argument("--auto-start", type=float, help="--fake only: press Enter after N s")
        p.add_argument("--fake-event", action="append", help="--fake only: kind@seconds[:id]")
        if name == "prepare":
            p.add_argument("--hold-only", action="store_true", help="stop after the torque-on hold (no HOME)")
        if name in ("prepare", "teleop", "pinch"):
            p.add_argument("--home-speed-deg-s", type=float)
            p.add_argument("--fake-calibrated", action="store_true",
                           help="--fake only: start the fake servos as a calibrate leaves them (with its record)")
        if name in ("teleop", "pinch"):
            p.add_argument("--no-panel", action="store_true", help="no control panel window (terminal keys only)")
        if name == "pinch":
            p.add_argument("--speed-scale", type=float, default=1.0,
                           help="0 < s <= 1: the same path, s times as fast as the simulator planned (default 1)")
            p.add_argument("--check-only", action="store_true", help="check every trajectory, move nothing")
            p.add_argument("--link", help="socket path (default $XDG_RUNTIME_DIR/manipulator_control/arm.sock)")
    p = sub.add_parser("calibration-status", help="read-only: can prepare / teleop use the stored calibration now?")
    p.add_argument("--config")
    p.add_argument("--fake", action="store_true")
    p.add_argument("--fake-calibrated", action="store_true")
    p = sub.add_parser("pose", help="no hardware: show a pose / one joint's + direction")
    p.add_argument("--q-deg", help="six comma-separated joint angles (deg), default all 0")
    p.add_argument("--sweep", type=int, choices=range(1, 7), help="animate joint N by +30 deg")
    p.add_argument("--ghost-deg", help="a second pose drawn as the green ghost (six comma-separated angles, deg)")
    p.add_argument("--png", help="write an image instead of opening the viewer")
    p.add_argument("--azimuth", default="135")
    p.add_argument("--elevation", default="-15")
    p = sub.add_parser("preview-home", help="no hardware: plan reference -> HOME as prepare would and replay it")
    p.add_argument("--config")
    p.add_argument("--fake", action="store_true")
    p.add_argument("--no-viewer", action="store_true")
    p.add_argument("--home-speed-deg-s", type=float)
    a = ap.parse_args(argv)
    if a.cmd == "preview-home":
        return cmd_preview_home(a)
    if a.cmd == "scan":
        return cmd_scan(a)
    if a.cmd == "pose":
        return cmd_pose(a)
    if a.cmd == "calibration-status":
        return cmd_calibration_status(a)
    if (a.auto_start is not None or a.fake_event or getattr(a, "fake_calibrated", False)) and not a.fake:
        sys.exit("--auto-start / --fake-event / --fake-calibrated are only for --fake runs")
    if a.cmd == "pinch" and not 0 < a.speed_scale <= 1:
        sys.exit("--speed-scale must be in (0, 1]")
    purpose = {"watch": "watch", "calibrate": "hold", "teleop": "teleop", "pinch": "pinch"}.get(a.cmd) or \
        ("hold" if a.hold_only else "home")
    return run_session(a, purpose)


if __name__ == "__main__":
    sys.exit(main())
