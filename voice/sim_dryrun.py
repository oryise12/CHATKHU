"""AI 없이 쿠옹이 함수를 직접 불러 시뮬레이터 흐름을 시험한다 (API 키·마이크가 없을 때의 대체 시연).

  python -m voice.sim_dryrun                    한 줄에 함수 하나 (예: start_edit, finish_edit, validate_and_move,
                                                undo_edit, stop_motion, move_region region=middle direction=up
                                                amount=small). q로 끝. finish_edit는 묻기만 하고, 움직이는 것은
                                                그 답인 validate_and_move다
  python -m voice.sim_dryrun --script "1:start_edit, 6:undo_edit, 3:finish_edit, 2:validate_and_move, 25:"
                                                "초:함수 인자..." 를 차례로 (초 = 앞 항목 뒤 기다리는 시간)

AI가 함수를 고른 뒤의 길(voice/sim_link.SimEditor.handle)을 그대로 지나므로, 시뮬레이터의 상태 변화와 쿠옹이가 말할
결과·알림 문장을 그대로 볼 수 있다. 호출어·질문 창 판정과 AI의 말 고르기는 여기서 시험하지 않는다
(voice/tests/test_gate.py, 실제 마이크 시험).
"""
from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time

from voice.config import load_config
from voice.sim_link import SimEditor, SimLinkError


def parse_call(text: str) -> tuple[str, dict]:
    parts = text.split()
    args = {}
    for p in parts[1:]:
        k, _, v = p.partition('=')
        try:
            args[k] = float(v) if k == 'distance_cm' else v
        except ValueError:
            args[k] = v
    return (parts[0] if parts else ''), args


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='python -m voice.sim_dryrun')
    ap.add_argument('--sim', metavar='SOCKET', help='시뮬레이터 소켓 (기본 $XDG_RUNTIME_DIR/pinch_sim/voice.sock)')
    ap.add_argument('--script', help="'초:함수 인자, ...' 를 차례로 실행")
    a = ap.parse_args(argv)
    cfg = load_config()
    try:
        ed = SimEditor(cfg.amount_cm, a.sim)
    except SimLinkError as exc:
        print(f'오류: {exc}', file=sys.stderr)
        return 2
    t0 = time.time()
    stamp = lambda: f'[{time.time() - t0:6.1f}s]'
    print(f'{stamp()} 시뮬레이터 연결됨: {ed.status.get("state")} rev {ed.rev}', flush=True)

    def speaker():
        while True:
            try:
                text = ed.announcements.get(timeout=0.2)
            except queue.Empty:
                if not ed.connected:
                    return
                continue
            print(f'{stamp()} 쿠옹이(알림) «{text}»  [시뮬레이터 {ed.status.get("state")}]', flush=True)
    threading.Thread(target=speaker, daemon=True).start()

    items = [x.strip() for x in a.script.split(',')] if a.script else None
    while True:
        if items is not None:
            if not items:
                break
            delay, _, line = items.pop(0).partition(':')
            time.sleep(float(delay or 0))
        else:
            line = sys.stdin.readline()
            if not line or line.strip() in ('q', 'quit'):
                break
        name, args = parse_call(line.strip())
        if not name:
            continue
        res = ed.handle(name, args)
        print(f'{stamp()} 함수 {name}{(" " + json.dumps(args, ensure_ascii=False)) if args else ""} → '
              f'{json.dumps(res, ensure_ascii=False)}  [시뮬레이터 {ed.status.get("state")}, '
              f'수정 모드 {"켜짐" if ed.status.get("edit_mode") else "꺼짐"}]', flush=True)
    time.sleep(1.5)
    ed.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
