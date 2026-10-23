"""MuJoCo 시뮬레이터(pinch_sim)에 경로 수정 명령을 보내는 처리기. FakeEditor 자리에 들어간다.

시뮬레이터를 `python -m pinch_sim.app --voice-gateway`로 켜 두면, 로컬 소켓(AF_UNIX)으로 붙는다.
메시지 형식은 pinch_sim/voice_gateway.py의 프로토콜 1이다(JSON, multiprocessing.connection + 인증 키).

- 함수 → 시뮬레이터 명령
  move_here   → move_point(target="here")  : 사용자가 시뮬레이터 화면에서 클릭한 점
  move_region → move_point(target=start|middle|end) (whole은 아직 지원하지 않음)
  adjust_amount → undo_last_edit + move_point(한 단계 큰/작은 양) : "조금만 더" / "좀 덜"
  undo_edit   → undo_last_edit, reset_edit → reset_to_original
  finish_edit → use_current_path(auto_run=false) : 검증만 한다. 통과하면 "이동할까요, 더 고칠까요?"를 묻는다
  approve_move → use_current_path : 그 질문에 "이동해·응"이면 움직인다 (물어본 그 경로일 때만)
  keep_editing → 시뮬레이터 명령 없음 : "더 고칠래"면 질문을 접고 수정으로 돌아간다
  stop_motion → stop (말이 들리자마자 세션이 stop_now로 먼저 멈춘다), continue_motion → resume(forward)
  go_back → 멈춘 뒤면 resume(back), 목표에 도착한 뒤면 return_reverse (가던 길을 거꾸로, 검증 후 자동 이동)
  edit_return_path → edit_return, new_target → new_target, save_paths → save_paths
- 멈춘 뒤 돌아가서 그 이동을 시작한 자리에 도착하면, 시뮬레이터가 그 이동의 수정 상태로 되돌리고(back_to_edit)
  "이동할까요, 더 고칠까요?"를 다시 묻는다. 거꾸로 오던 길이면 목표 자리에서 ⑤ 질문을 앞말 없이 다시 묻는다.
- 손(마우스 끌기, 나중에 핀치)으로 고치면 "경로를 수정했어요."라고 알린다.
- 두 단계: 가는 길을 고쳐 목표까지 간 뒤, 돌아오는 길을 고치거나 가던 길을 거꾸로 돌아온다.
- 멈춘 뒤 STOP_ASK_S 동안 계속/돌아가기 명령이 없으면 "계속 갈까요, 돌아갈까요?"를 묻는다.
- 검증·실행 결과는 나중에 이벤트로 오므로 알림 큐(announcements)에 한국어 문장으로 넣는다.
  세션이 그 문장을 AI에게 넘겨 말하게 한다.
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
import uuid
from multiprocessing.connection import Client
from pathlib import Path
from typing import Any

from voice.tools import EDIT_FUNCTIONS, MOTION_FUNCTIONS, NAMES, AmountLadder, _check_args

PROTOCOL = 1
REPLY_TIMEOUT_S = 5.0
REQUEST_DEADLINE_S = 4.0
STOP_ASK_S = 5.0          # 멈춘 뒤 이 시간 동안 명령이 없으면 계속 갈지 돌아갈지 묻는다
STOP_RECENT_S = 10.0      # 즉시 정지 뒤 AI가 부른 stop_motion은 "멈췄음"으로 답한다
ASK_STOPPED = '계속 갈까요, 돌아갈까요?'
ASK_RETURN = '돌아오는 길을 수정할까요, 가던 길 그대로 반대로 돌아갈까요?'
ASK_ARRIVED = '목표에 도착했어요. ' + ASK_RETURN
ASK_HOME = '처음 자리로 돌아왔어요. 새 목표로 다시 해 볼까요?'
ASK_APPROVE = '이동할까요, 더 고칠까요?'
ASK_PASSED = '검증을 통과했어요. ' + ASK_APPROVE
MOVE_START = '이동을 시작할게요.'
HAND_EDITED = '경로를 수정했어요.'
HAND_EDIT_QUIET_S = 1.0   # 손(마우스·핀치)으로 고친 뒤 이 시간 동안 또 고치지 않으면 한 번만 알린다

FAIL_REASON = {
    'path_wellformed': '경로 모양이 잘못됐어요', 'start_matches_robot': '경로 시작점이 로봇 위치와 달라요',
    'goal_matches_target': '경로 끝이 목표와 달라요', 'workspace': '경로가 로봇이 닿는 범위를 벗어나요',
    'continuous_ik': '팔이 닿지 않는 곳이 있어요', 'joint_limits': '관절이 꺾일 수 있는 범위를 넘어요',
    'singularity': '팔이 펴지거나 꺾이는 위험한 자세에 너무 가까워요', 'collision': '책상이나 로봇 자신과 부딪혀요',
    'timing': '시간 배치가 맞지 않아요', 'joint_vel_acc': '관절이 너무 빠르게 움직여야 해요',
    'torque': '모터 힘이 모자라요', 'sim_rollout': '시뮬레이션으로 따라가 보니 경로를 벗어나요',
}


def default_address() -> Path:
    base = os.environ.get('XDG_RUNTIME_DIR') or f'/tmp/pinch_sim-{os.getuid()}'
    return Path(base) / 'pinch_sim' / 'voice.sock'


class SimLinkError(RuntimeError):
    pass


class SimEditor:
    fail_mode = False   # 세션·기록이 읽는 값. 시뮬레이터 연결에서는 쓰지 않는다

    def __init__(self, amount_cm: dict[str, float], address: str | None = None) -> None:
        self.amount_cm = amount_cm
        self.ladder = AmountLadder(amount_cm)
        addr = Path(address) if address else default_address()
        key_path = addr.with_suffix('.key')
        if not addr.exists() or not key_path.exists():
            raise SimLinkError(f'시뮬레이터 소켓이 없습니다: {addr}. 먼저 시뮬레이터를 --voice-gateway로 실행하세요.')
        try:
            self._conn = Client(str(addr), family='AF_UNIX', authkey=key_path.read_bytes())
        except (OSError, EOFError) as exc:
            raise SimLinkError(f'시뮬레이터에 연결하지 못했습니다: {exc}') from exc
        self.rev = 0
        self.status: dict = {}
        self._stopped_at = 0.0
        self._stop_seq = 0        # 멈춤마다 +1, 계속/돌아가기 명령마다 +1 (5초 질문 취소)
        self._hand_timer: threading.Timer | None = None
        self._ask_fp: str | None = None   # "이동할까요, 더 고칠까요?"를 물은 경로 (그 경로일 때만 이동 승인)
        self.connected = True
        self.announcements: queue.Queue[str] = queue.Queue()
        self._results: dict[str, dict] = {}
        self._cv = threading.Condition()
        self._send_lock = threading.Lock()
        threading.Thread(target=self._reader, name='sim-reader', daemon=True).start()

    # ------------------------------------------------------------------ 송수신
    def _send(self, msg: dict) -> None:
        with self._send_lock:
            self._conn.send_bytes(json.dumps(msg, ensure_ascii=False).encode('utf-8'))

    def _reader(self) -> None:
        try:
            while True:
                msg = json.loads(self._conn.recv_bytes(1 << 22).decode('utf-8'))
                status = msg.get('status')
                if isinstance(status, dict) and isinstance(status.get('rev'), int):
                    self.rev = status['rev']
                    self.status = status
                if msg.get('type') == 'result':
                    with self._cv:
                        self._results[msg.get('request_id', '')] = msg
                        self._cv.notify_all()
                elif msg.get('type') == 'event':
                    if msg.get('kind') == 'execution_end' and msg.get('state') == 'ABORTED':
                        self._stopped_at = time.time()
                        self._stop_seq += 1
                        threading.Timer(STOP_ASK_S, self._ask_if_still_stopped, (self._stop_seq,)).start()
                    if msg.get('kind') == 'edit_end' and msg.get('actor') in ('mouse', 'hand') and \
                            (msg.get('data') or {}).get('kept'):
                        self._hand_edited()
                    text = _announcement(msg)
                    if text in (ASK_PASSED, ASK_APPROVE):
                        self._ask_fp = (msg.get('status') or {}).get('path_fp')
                    elif msg.get('kind') == 'execution_start':
                        self._ask_fp = None
                    if text:
                        self.announcements.put(text)
        except (EOFError, OSError, ValueError):
            self.connected = False
            self.announcements.put('시뮬레이터와 연결이 끊겼어요.')
            with self._cv:
                self._cv.notify_all()

    def _command(self, command: str, **fields) -> dict:
        if not self.connected:
            return {'accepted': False, 'reason_code': 'SIM_DISCONNECTED'}
        rid = uuid.uuid4().hex[:12]
        self._send({'v': PROTOCOL, 'type': 'command', 'request_id': rid, 'command': command,
                    'expected_rev': self.rev, 'deadline_unix': time.time() + REQUEST_DEADLINE_S, **fields})
        with self._cv:
            self._cv.wait_for(lambda: rid in self._results or not self.connected, REPLY_TIMEOUT_S)
            res = self._results.pop(rid, None)
        if res is None:
            return {'accepted': False, 'reason_code': 'SIM_TIMEOUT'}
        return res

    def _command_fresh(self, command: str, **fields) -> dict:
        """사용자의 직전 클릭·수정으로 rev가 막 바뀐 경우(STALE_REVISION)만 최신 rev로 한 번 다시 보낸다."""
        res = self._command(command, **fields)
        if not res.get('accepted') and res.get('reason_code') == 'STALE_REVISION':
            res = self._command(command, **fields)
        return res

    def _hand_edited(self) -> None:
        """손으로 고친 직후: 말로 한 직전 수정이 아니게 되므로 단계 조절을 잊고, 잠시 뒤 한 번만 알린다."""
        self.ladder.forget()
        if self._hand_timer is not None:
            self._hand_timer.cancel()
        self._hand_timer = threading.Timer(HAND_EDIT_QUIET_S, self.announcements.put, (HAND_EDITED,))
        self._hand_timer.daemon = True
        self._hand_timer.start()

    def _ask_if_still_stopped(self, seq: int) -> None:
        if seq == self._stop_seq and self.status.get('state') == 'ABORTED':
            self.announcements.put(ASK_STOPPED)

    def stop_now(self) -> bool:
        """말에서 멈춤이 들리자마자(AI 응답 전) 부른다. 움직이거나 검증 중일 때만 멈춘다."""
        if self.status.get('state') not in ('EXECUTING', 'VALIDATING'):
            return False
        res = self._command('stop')
        return bool(res.get('accepted'))

    def _refusal(self, res: dict) -> dict[str, Any]:
        """거절 이유. 상태 때문에 못 하는 경우는 지금 상태를 알려 주는 이유로 바꾼다."""
        code = res.get('reason_code', 'refused')
        st = res.get('status') or self.status
        if code in ('NOT_EDITABLE', 'NOT_ALLOWED', 'BUSY', 'BUSY_EXECUTING', 'NO_PATH'):
            last = (st.get('last_exec') or {}).get('phase')
            code = {'DONE': 'ARRIVED_TARGET' if last == 'outbound' else 'ARRIVED_HOME',
                    'ABORTED': 'STOPPED', 'EXECUTING': 'MOVING', 'VALIDATING': 'VALIDATING'}.get(st.get('state'), code)
        return {'status': 'failed', 'reason': code.lower()}

    def hint(self, text: str) -> None:
        """시뮬레이터 화면 아래쪽 한 줄 표시 (예: 쿠옹이가 듣고 있어요)."""
        if self.connected:
            try:
                self._send({'v': PROTOCOL, 'type': 'client_state', 'text': text})
            except OSError:
                pass

    def chat(self, role: str, text: str) -> None:
        """시뮬레이터 화면의 대화 말풍선 (role: user | kuong)."""
        text = text.replace('«', '').replace('»', '').strip()   # 알림 문장 표시(« »)는 말풍선에 넣지 않는다
        if self.connected and text:
            try:
                self._send({'v': PROTOCOL, 'type': 'chat', 'role': role, 'text': text})
            except OSError:
                pass

    # ------------------------------------------------------------------ 함수 처리 (FakeEditor와 같은 모양)
    def handle(self, name: str, args: dict[str, Any]) -> dict[str, Any]:
        if name not in NAMES:
            return {'status': 'error', 'reason': f'unknown_function:{name}'}
        if name == 'adjust_amount':
            return self._adjust(args.get('change'))
        if name in EDIT_FUNCTIONS:
            return self._move(name, args)
        # 직전 말 수정의 단계는 수정 기록이 바뀔 때만 잊는다. 멈춤·이동·수정 완료 뒤 같은 길의 수정으로
        # 돌아오면(back_to_edit) "조금만 더"가 그 수정을 이어서 조절해야 하기 때문이다.
        if name in ('undo_edit', 'reset_edit', 'new_target', 'edit_return_path'):
            self.ladder.forget()
        if name in MOTION_FUNCTIONS:
            return self._motion(name)
        if name in ('approve_move', 'keep_editing'):
            return self._approval(name)
        if name == 'save_paths':
            res = self._command('save_paths')
            if not res.get('accepted'):
                return {'status': 'failed', 'reason': 'nothing_to_save'}
            return {'status': 'applied', 'file': Path((res.get('detail') or {}).get('file', '')).name}
        command = {'undo_edit': 'undo_last_edit', 'reset_edit': 'reset_to_original',
                   'finish_edit': 'use_current_path'}[name]
        fields = {'auto_run': False} if name == 'finish_edit' else {}
        res = self._command_fresh(command, **fields)
        if not res.get('accepted'):
            return self._refusal(res)
        if name == 'finish_edit':
            st = res.get('status') or {}
            if res.get('previous_state') == 'PASSED':   # 이미 통과한 경로: 알림 없이 AI가 이동할지 다시 묻는다
                self._ask_fp = st.get('path_fp')
                return {'status': 'passed', 'next': 'ask_move_or_edit'}
            return {'status': 'validating', 'next': 'ask_before_moving'}
        return {'status': 'applied'}

    def _approval(self, name: str) -> dict[str, Any]:
        """"이동할까요, 더 고칠까요?"의 답. 이동은 물어본 그 경로가 그대로일 때만 한다."""
        asked = self._ask_fp is not None and self._ask_fp == self.status.get('path_fp')
        if name == 'keep_editing':
            self._ask_fp = None
            return {'status': 'applied', 'next': 'click_and_edit'}
        if not asked or self.status.get('state') not in ('PASSED', 'GENERATED', 'SELECTED'):
            self._ask_fp = None
            return {'status': 'failed', 'reason': 'not_asked'}
        res = self._command_fresh('use_current_path')
        if not res.get('accepted'):
            return self._refusal(res)
        self._ask_fp = None
        if (res.get('status') or {}).get('state') == 'EXECUTING':
            return {'status': 'moving'}
        return {'status': 'validating', 'next': 'auto_run_if_passed'}

    def _motion(self, name: str) -> dict[str, Any]:
        if name == 'stop_motion':
            res = self._command('stop')
            if res.get('accepted'):
                return {'status': 'stopped' if res.get('detail') == 'execution' else 'applied'}
            if time.time() - self._stopped_at < STOP_RECENT_S:
                return {'status': 'stopped'}   # 말이 들리자마자 이미 멈췄다
            return {'status': 'failed', 'reason': 'nothing_to_stop'}
        if name in ('continue_motion', 'go_back'):
            self._stop_seq += 1                # 5초 뒤 질문 취소
            state = self.status.get('state')
            if state == 'ABORTED':
                res = self._command_fresh('resume', direction='forward' if name == 'continue_motion' else 'back')
            elif name == 'go_back' and state == 'DONE':
                res = self._command_fresh('return_reverse')
            else:
                return {'status': 'failed', 'reason': 'not_stopped' if name == 'continue_motion' else 'not_at_target'}
            return {'status': 'validating', 'next': 'auto_run_if_passed'} if res.get('accepted') else self._refusal(res)
        command = {'edit_return_path': 'edit_return', 'new_target': 'new_target'}[name]
        res = self._command_fresh(command)
        if not res.get('accepted'):
            return self._refusal(res)
        return {'status': 'applied', 'next': 'click_and_edit' if name == 'edit_return_path' else 'edit_outbound'}

    def _adjust(self, change) -> dict[str, Any]:
        """직전 수정을 취소하고 한 단계 큰/작은 양으로 다시 옮긴다. 다시 옮기기가 실패하면 직전 수정을 되살린다."""
        step = self.ladder.step(change)
        if isinstance(step, str):
            return {'status': 'failed', 'reason': step}
        move, new_args, old = step
        old_args = dict(new_args, amount=old)
        res = self._command_fresh('undo_last_edit')
        if not res.get('accepted'):
            return self._refusal(res)
        result = self._move(move, new_args)
        if result.get('status') != 'applied':
            self._command_fresh('redo_last_edit')  # 취소했던 직전 수정을 그대로 되살린다
            self.ladder.remember(move, old_args)
            return result
        result['replaced_cm'] = int(self.amount_cm[old]) if float(self.amount_cm[old]).is_integer() else self.amount_cm[old]
        return result

    def _move(self, name: str, args: dict[str, Any]) -> dict[str, Any]:
        bad = _check_args(name, args)
        if bad:
            return {'status': 'error', 'reason': bad}
        target = 'here' if name == 'move_here' else args['region']
        if target == 'whole':
            return {'status': 'failed', 'reason': 'not_supported'}
        dist = float(args['distance_cm']) if 'distance_cm' in args else float(self.amount_cm[args['amount']])
        res = self._command_fresh('move_point', target=target, direction=args['direction'], distance_cm=dist)
        if not res.get('accepted'):
            return self._refusal(res)
        moved = (res.get('detail') or {}).get('moved_cm', dist)
        self.ladder.remember(name, args)
        result = {'status': 'applied', 'target': target, 'direction': args['direction'],
                  'moved_cm': int(moved) if float(moved).is_integer() else moved, 'path': 'both'}
        return result

    def close(self) -> None:
        try:
            self._conn.close()
        except OSError:
            pass


def _announcement(ev: dict) -> str:
    """시뮬레이터 이벤트 중 사용자에게 말로 알릴 것만 한국어 한 문장으로."""
    kind, actor = ev.get('kind'), ev.get('actor')
    data = ev.get('data') or {}
    if kind == 'validation':
        if ev.get('state') == 'PASSED':
            # 계속 가·돌아가처럼 자동 실행이 예약된 검증이면 실행 시작 알림으로 한 번만 말한다
            return '' if (ev.get('status') or {}).get('auto_run') else ASK_PASSED
        # 실패 항목은 이벤트 data가 아니라 상태(status.report)에 들어 있다
        failed = (((ev.get('status') or {}).get('report') or {}).get('failed_kinds') or [])
        reason = FAIL_REASON.get(failed[0], '') if failed else ''
        return '검증을 통과하지 못했어요.' + (f' {reason}.' if reason else '') + ' 실패한 곳을 화면에 빨간 점으로 표시했어요.'
    if kind == 'execution_start':
        return MOVE_START
    if kind == 'execution_end':
        if ev.get('state') != 'DONE' or data.get('next'):
            return ''   # 멈춤: stop_motion 결과로 말하고 5초 뒤 묻는다 / 돌아가서 수정으로: back_to_edit에서 묻는다
        last = ((ev.get('status') or {}).get('last_exec') or {})
        if last.get('phase') != 'outbound':
            return ASK_HOME
        # 거꾸로 오던 길에서 멈추고 돌아가 목표 자리로 돌아왔으면 도착 인사 없이 다시 묻는다
        return ASK_RETURN if last.get('kind') == 'return_reverse' else ASK_ARRIVED
    if kind == 'back_to_edit':
        return ASK_APPROVE   # 이미 검증을 통과한 길이라 앞말 없이 묻는다
    if kind == 'point_select' and actor == 'mouse':
        return ''   # 클릭은 화면 표시로 충분하다 (말하지 않는다)
    return ''
