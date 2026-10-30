"""MuJoCo 시뮬레이터(pinch_sim)에 경로 수정 명령을 보내는 처리기. FakeEditor 자리에 들어간다.

시뮬레이터를 `python -m pinch_sim.app --voice-gateway`로 켜 두면, 로컬 소켓(AF_UNIX)으로 붙는다.
메시지 형식은 pinch_sim/voice_gateway.py의 프로토콜 1이다(JSON, multiprocessing.connection + 인증 키).

- 함수 → 시뮬레이터 명령
  start_edit  → start_edit : 시뮬레이터의 수정 모드를 실제로 켠다(손 입력은 수정 모드에서만 된다). 켜졌다는 결과가
                와야만 안내 문장(say: EDIT_GUIDE)을 돌려준다. 검증 중이면 검증을 취소하고, 목표에 도착한 뒤면 돌아오는 길을 고친다
  move_here   → move_point(target="here")  : 사용자가 시뮬레이터 화면에서 검지로 가리킨 점 (마우스 클릭도 되지만 안내하지 않는다)
  move_region → move_point(target=start|middle|end) (whole은 아직 지원하지 않음)
  adjust_amount → undo_last_edit + move_point(한 단계 큰/작은 양) : "조금만 더" / "좀 덜"
  undo_edit   → undo_last_edit ("이전으로": 손·마우스·말 수정이 모두 같은 되돌리기 기록), reset_edit → reset_to_original
  finish_edit → (시뮬레이터 명령 없음) 앱이 "검증 후 이동할까요?"(ASK_RUN)라고 묻는다. 답은 호출어 없이 한 번 받는다
  validate_and_move → use_current_path(auto_run) : 그 질문에 "어/응/네"라고 답하면 검증하고, 통과하면 그 경로로 바로
                움직인다(시뮬레이터 전용). 예약은 그 검증·그 경로 버전에만 묶인다(시뮬레이터 Session.auto_run): 그 사이
                편집·수정 모드·X·연결 끊김·실패면 취소된다. 결정한 뒤 경로가 바뀌었으면(STALE_REVISION) 다시 보내지 않고
                path_changed로 돌려준다
  stop_motion → stop (말이 들리자마자 세션이 stop_now로 먼저 멈춘다), continue_motion → resume(forward)
  go_back → 멈춘 뒤면 resume(back), 목표에 도착한 뒤면 return_reverse (가던 길을 거꾸로, 검증 후 자동 이동)
  edit_return_path → edit_return, new_target → new_target, save_paths → save_paths
- 손(마우스 끌기, 핀치)으로 고치면 "경로를 수정했어요."라고 알리고, 열린 호출어 창을 다시 채운다(on_activity).
  손 입력이 거절된 이유(수정 모드 아님, 깊이 없음 등)는 말하지 않는다: 시뮬레이터 화면(알림 줄, 손가락 옆)에만 보인다.
- 두 단계: 가는 길을 고쳐 목표까지 간 뒤, 돌아오는 길을 고치거나 가던 길을 거꾸로 돌아온다.
- 멈춘 뒤 STOP_ASK_S 동안 계속/돌아가기 명령이 없으면 "계속 갈까요, 돌아갈까요?"를 묻는다.
- 검증·실행 결과는 나중에 이벤트로 오므로 알림 큐(announcements)에 한국어 문장으로 넣는다.
  세션이 그 문장을 AI에게 넘겨 말하게 한다. 질문 문장이면 question_functions로 답이 될 함수만 알려 준다.
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
from typing import Any, Callable

from voice.tools import EDIT_FUNCTIONS, MOTION_FUNCTIONS, NAMES, AmountLadder, _check_args

PROTOCOL = 1
REPLY_TIMEOUT_S = 5.0
REQUEST_DEADLINE_S = 4.0
HELLO_TIMEOUT_S = 3.0
STOP_ASK_S = 5.0          # 멈춘 뒤 이 시간 동안 명령이 없으면 계속 갈지 돌아갈지 묻는다
STOP_RECENT_S = 10.0      # 즉시 정지 뒤 AI가 부른 stop_motion은 "멈췄음"으로 답한다
ASK_STOPPED = '계속 갈까요, 돌아갈까요?'
ASK_RETURN = '돌아오는 길을 수정할까요, 가던 길 그대로 반대로 돌아갈까요?'
ASK_ARRIVED = '목표에 도착했어요. ' + ASK_RETURN
ASK_HOME = '처음 자리로 돌아왔어요. 새 목표로 다시 해 볼까요?'
ASK_RUN = '검증 후 이동할까요?'
# 시연 지점 (pinch_sim --stations): S 설거지 바구니에서 B 국그릇 / C 밥그릇을 골라 다녀온다. 둘 다 다녀오면 다시 이동할지 묻는다
ASK_WHERE = '어디로 갈까요?'   # S에 도착하면 바로 (처음). "쿠옹아"라고 부르면 다시 묻는다
ASK_AGAIN = '두 곳 모두 다녀왔어요. 다시 이동할까요?'   # 둘 다 다녀온 뒤 (응 -> 새 한 바퀴, 어디로 갈까요?)
# 앱의 질문 → 답이 될 수 있는 함수 (호출어 없이 받는 것은 이 함수들뿐이다. 멈춤은 언제나 된다)
QUESTION_FUNCTIONS = {
    ASK_STOPPED: frozenset({'continue_motion', 'go_back'}),
    ASK_RETURN: frozenset({'edit_return_path', 'start_edit', 'go_back'}),
    ASK_ARRIVED: frozenset({'edit_return_path', 'start_edit', 'go_back'}),
    ASK_HOME: frozenset({'new_target', 'save_paths'}),
    ASK_RUN: frozenset({'validate_and_move'}),
}
EDIT_GUIDE = '경로 수정 후 수정 완료를 말해 주세요.'   # 수정 모드가 실제로 켜진 뒤 (가는 길·돌아오는 길 모두)
ARM_SAY = {   # 실물 팔(pinch_sim --real-arm)이 검증·이동을 받지 않는 이유: 검증 실패가 아니다
    'ARM_NOT_ARMED': '실물 로봇팔이 아직 무장 전이에요. 실물 제어 창에서 y를 눌러 주세요.',
    'ARM_DISCONNECTED': '실물 로봇팔과 연결되어 있지 않아요.',
}
MOVE_START = '이동을 시작할게요.'
PASSED_MOVE = '검증을 통과했어요. 이동을 시작할게요.'
PASSED_WAIT = '검증을 통과했어요. Y 버튼을 누르면 이동해요.'
BACK_TO_EDIT = '시작한 자리로 돌아왔어요. 경로를 더 고치거나 수정 완료라고 말씀해 주세요.'
HAND_EDITED = '경로를 수정했어요.'
HAND_EDIT_QUIET_S = 1.0   # 손(마우스·핀치)으로 고친 뒤 이 시간 동안 또 고치지 않으면 한 번만 알린다
CALIB_NOTICE = {
    'moved': '카메라가 움직인 것 같아요. 다시 보정하기 전에는 손가락 3D 수정을 쓸 수 없어요. 마우스로는 고칠 수 있어요.',
    'not_visible': '기준 표지가 안 보여서 카메라 정합을 확인할 수 없어요. 손가락 3D 수정을 잠시 쓸 수 없어요.',
}

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
        self.on_activity: Callable[[], None] | None = None   # 손으로 고쳤을 때 (세션이 호출어 창을 다시 채운다)
        self.connected = True
        self.refused = ''
        self.announcements: queue.Queue[str] = queue.Queue()
        self._results: dict[str, dict] = {}
        self._cv = threading.Condition()
        self._hello = threading.Event()
        self._send_lock = threading.Lock()
        threading.Thread(target=self._reader, name='sim-reader', daemon=True).start()
        if not self._hello.wait(HELLO_TIMEOUT_S) or self.refused:
            self.close()
            raise SimLinkError('시뮬레이터가 연결을 받지 않았습니다' +
                               (f' ({self.refused}). 다른 음성 프로그램(예: 예전 ROS 쿠옹이)을 끄고 다시 실행하세요.'
                                if self.refused else '.'))

    # ------------------------------------------------------------------ 송수신
    def _send(self, msg: dict) -> None:
        with self._send_lock:
            self._conn.send_bytes(json.dumps(msg, ensure_ascii=False).encode('utf-8'))

    def _reader(self) -> None:
        try:
            while True:
                msg = json.loads(self._conn.recv_bytes(1 << 22).decode('utf-8'))
                if msg.get('type') == 'refused':
                    self.refused = str(msg.get('reason') or 'refused')
                    self._hello.set()
                    return
                status = msg.get('status')
                if isinstance(status, dict) and isinstance(status.get('rev'), int):
                    self.rev = status['rev']
                    self.status = status
                if msg.get('type') == 'hello':
                    self._hello.set()
                elif msg.get('type') == 'result':
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
                    text = self._announcement(msg)
                    if text:
                        self.announcements.put(text)
        except (EOFError, OSError, ValueError):
            self.connected = False
            self._hello.set()
            if not self.refused:
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
        """사용자의 직전 클릭·수정으로 rev가 막 바뀐 경우(STALE_REVISION)만 최신 rev로 한 번 다시 보낸다.
        움직임으로 이어지는 finish_edit에는 쓰지 않는다 (결정한 뒤 바뀐 경로를 검증·실행하지 않게)."""
        res = self._command(command, **fields)
        if not res.get('accepted') and res.get('reason_code') == 'STALE_REVISION':
            res = self._command(command, **fields)
        return res

    def _hand_edited(self) -> None:
        """손으로 고친 직후: 말로 한 직전 수정이 아니게 되므로 단계 조절을 잊고, 잠시 뒤 한 번만 알린다."""
        self.ladder.forget()
        if self.on_activity is not None:
            self.on_activity()
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

    @staticmethod
    def question_functions(text: str) -> frozenset | None:
        """앱이 한 질문의 답이 될 함수 (모르는 질문이면 None: 호출어 없이는 받지 않는다)."""
        if text in QUESTION_FUNCTIONS:
            return QUESTION_FUNCTIONS[text]
        if text.endswith((ASK_WHERE, '다시 이동할까요?')) or ('남은 ' in text and text.endswith('갈까요?')):
            return frozenset({'go_to_station'})      # "설거지 바구니에 도착했어요. 어디로 갈까요?" 등
        if text.endswith(ASK_RETURN):            # "B 국그릇에 도착했어요. 돌아오는 길을 ..."
            return QUESTION_FUNCTIONS[ASK_RETURN]
        if text.endswith(('로 이동할까요?', '으로 이동할까요?')):   # "설거지 바구니로 이동할까요?" (HOME -> S)
            return frozenset({'validate_and_move'})
        return None

    def wake_question(self) -> str | None:
        """시연 지점: 로봇이 S(바구니)에 있고 다음 곳을 고를 차례면, 사용자가 "쿠옹아"라고 부를 때 다시 물을 질문."""
        st = self.status.get('stations') or {}
        if not st or not st.get('at_base') or self.status.get('state') not in ('DONE', 'HOLD'):
            return None
        return _next_question(st)

    def _refusal(self, res: dict) -> dict[str, Any]:
        """거절 이유. 상태 때문에 못 하는 경우는 지금 상태를 알려 주는 이유로 바꾼다."""
        code = res.get('reason_code', 'refused')
        st = res.get('status') or self.status
        if code in ('NOT_EDITABLE', 'NOT_ALLOWED', 'BUSY', 'BUSY_EXECUTING', 'NO_PATH'):
            last = (st.get('last_exec') or {}).get('phase')
            code = {'DONE': 'ARRIVED_TARGET' if last == 'outbound' else 'ARRIVED_HOME',
                    'ABORTED': 'STOPPED', 'EXECUTING': 'MOVING', 'VALIDATING': 'VALIDATING'}.get(st.get('state'), code)
        if code in ARM_SAY:                    # 정해진 문장 그대로 말한다 (모델이 "검증에 실패했어요"로 바꾸지 않게)
            return {'status': 'failed', 'reason': code.lower(), 'say': ARM_SAY[code]}
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
        if name in ('undo_edit', 'reset_edit', 'new_target', 'edit_return_path', 'set_target_from_camera',
                    'go_to_station'):
            self.ladder.forget()
        if name == 'go_to_station':
            return self._go_to_station(args.get('station'))
        if name == 'set_target_from_camera':
            res = self._command_fresh('vision_target')
            if not res.get('accepted'):
                return {'status': 'failed', 'reason': str(res.get('reason_code', 'refused')).lower()}
            return {'status': 'applied', 'say': '카메라로 찾은 물체 위로 목표를 바꾸고 새 경로를 만들었어요. '
                                                '고치려면 경로 수정할래, 이대로 가려면 수정 완료라고 말씀해 주세요.'}
        if name in MOTION_FUNCTIONS:
            return self._motion(name)
        if name == 'start_edit':
            return self._start_edit()
        if name == 'finish_edit':
            return self._finish()
        if name == 'validate_and_move':
            return self._validate_and_move()
        if name == 'save_paths':
            res = self._command('save_paths')
            if not res.get('accepted'):
                return {'status': 'failed', 'reason': 'nothing_to_save'}
            return {'status': 'applied', 'file': Path((res.get('detail') or {}).get('file', '')).name}
        command = {'undo_edit': 'undo_last_edit', 'reset_edit': 'reset_to_original'}[name]
        res = self._command_fresh(command)
        if not res.get('accepted'):
            return self._refusal(res)
        return {'status': 'applied'}

    def _go_to_station(self, station) -> dict[str, Any]:
        """시연: S에서 B / C까지 직선 경로를 그리고 수정 모드를 켠다 (시뮬레이터가 켠 뒤에만 안내한다)."""
        if station not in ('B', 'C'):
            return {'status': 'error', 'reason': 'bad_station'}
        res = self._command_fresh('station_target', station=station)
        if not res.get('accepted'):
            code = str(res.get('reason_code', 'refused'))
            if code in ('NOT_AT_BASE', 'NO_STATIONS'):
                return {'status': 'failed', 'reason': code.lower()}
            return self._refusal(res)
        label = (res.get('detail') or {}).get('label', '')
        return {'status': 'applied', 'station': station,
                'say': f'{station} {label}까지 직선 경로를 그렸어요. {EDIT_GUIDE}'.replace('  ', ' ')}

    def _start_edit(self) -> dict[str, Any]:
        """수정 모드를 시뮬레이터에서 실제로 켠다. 켜졌다는 결과가 온 뒤에만 안내 문장을 준다."""
        res = self._command_fresh('start_edit')
        if not res.get('accepted'):
            return self._refusal(res)
        st = res.get('status') or {}
        if not st.get('edit_mode'):
            return {'status': 'failed', 'reason': 'not_editable'}
        return {'status': 'applied', 'edit_mode': True, 'path': st.get('phase') or 'outbound', 'say': EDIT_GUIDE}

    def _finish(self) -> dict[str, Any]:
        """"수정 완료 / 이 경로로 할게": 바로 움직이지 않는다. 앱이 "검증 후 이동할까요?"라고 묻고, 그렇다는 답이
        validate_and_move다. 지금 검증할 수 없는 상태(이동·검증 중, 도착, 멈춤, 실패한 그 경로)면 묻지 않고 이유를 준다."""
        if self._hand_timer is not None:
            self._hand_timer.cancel()          # 방금 손 수정의 "경로를 수정했어요."는 질문과 겹쳐 늦게 말하지 않는다
        state = self.status.get('state')
        if state == 'FAILED':                  # 실패한 뒤 고치지 않은 그 경로 (고치면 상태가 바뀐다)
            return {'status': 'failed', 'reason': 'path_failed'}
        arm = self.status.get('real_arm') or {}
        if arm.get('gate') and state not in ('EXECUTING', 'VALIDATING', 'DONE', 'ABORTED'):
            # 실물이 무장 전이면 검증·이동을 받지 않는다: 묻지 않고 그 이유를 말한다
            return self._refusal({'reason_code': 'ARM_NOT_ARMED' if arm.get('up') else 'ARM_DISCONNECTED'})
        if state in ('EXECUTING', 'VALIDATING', 'DONE', 'ABORTED'):
            return self._refusal({'reason_code': 'NOT_ALLOWED', 'status': self.status})
        self.announcements.put(ASK_RUN)
        return {'status': 'asking'}

    def _validate_and_move(self) -> dict[str, Any]:
        """"검증 후 이동할까요?"에 "어": 지금 경로를 검증하고, 통과하면 그 경로로 바로 움직인다 (시뮬레이터 전용)."""
        if self._hand_timer is not None:
            self._hand_timer.cancel()          # 방금 손 수정의 "경로를 수정했어요."는 검증 중에 늦게 말하지 않는다
        res = self._command('use_current_path', auto_run=True)
        if not res.get('accepted'):
            if res.get('reason_code') == 'STALE_REVISION':
                return {'status': 'failed', 'reason': 'path_changed'}
            return self._refusal(res)
        if (res.get('status') or {}).get('state') == 'EXECUTING':
            return {'status': 'moving'}                      # 이미 통과한 그 경로: 바로 움직인다
        return {'status': 'validating', 'next': 'moves_if_passed'}

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
        if name == 'edit_return_path':
            return {'status': 'applied', 'next': 'edit_return', 'say': EDIT_GUIDE}
        return {'status': 'applied', 'next': 'edit_outbound'}

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

    # ------------------------------------------------------------------ 알림
    def _announcement(self, ev: dict) -> str:
        return _announcement(ev)


def _fail_place(status: dict) -> str:
    """검증 실패 지점을 말로: 가는 길(돌아오는 길)의 몇 % 쯤."""
    rep = status.get('report') or {}
    pct = rep.get('fail_at_pct')
    if pct is None:
        return ''
    way = '돌아오는 길' if status.get('phase') == 'return' else '가는 길'
    return f' 실패한 곳은 {way}의 약 {int(round(pct / 10.0)) * 10}% 지점이에요.'


def _ro(word: str) -> str:
    """'으로' / '로' (받침이 없거나 ㄹ 받침이면 '로')."""
    if not word:
        return '로'
    c = ord(word[-1]) - 0xAC00
    if not 0 <= c <= 11171:
        return '로'
    jong = c % 28
    return '로' if jong in (0, 8) else '으로'


def _next_question(st: dict) -> str:
    """S에서 다음에 물을 것: 처음이면 어디로 갈까요? / 하나 남았으면 남은 곳 / 둘 다 다녀왔으면 다시 이동할지."""
    if not st.get('visited'):
        return ASK_WHERE
    rest = st.get('remaining') or []
    if not rest:
        return ASK_AGAIN
    name = rest[0]
    label = (st.get('labels') or {}).get(name, '')
    return f'남은 {name} {label}{_ro(label)} 갈까요?'.replace('  ', ' ')


def _station_arrival(status: dict, last: dict) -> str:
    """시연 지점에서 이동이 끝났을 때 알림: S 도착 / 그릇 도착(돌아오는 길 질문) / S로 복귀 / 둘 다 다녀옴(다시 이동할까요?)."""
    st = status.get('stations') or {}
    labels = st.get('labels') or {}
    base = labels.get('S', '바구니')
    if last.get('kind') == 'to_base':
        return f'{base}에 도착했어요. {_next_question(st)}'
    if last.get('phase') == 'outbound':
        if last.get('kind') == 'return_reverse':
            return ASK_RETURN
        name = st.get('current') or ''
        return f'{name} {labels.get(name, "")}에 도착했어요. '.replace('  ', ' ').lstrip() + ASK_RETURN
    if st.get('remaining'):
        return f'{base}{_ro(base)} 돌아왔어요. {_next_question(st)}'
    return ASK_AGAIN


def _announcement(ev: dict) -> str:
    """시뮬레이터 이벤트 중 사용자에게 말로 알릴 것만 한국어 한 문장으로."""
    kind, actor = ev.get('kind'), ev.get('actor')
    data = ev.get('data') or {}
    status = ev.get('status') or {}
    if kind == 'validation':
        if ev.get('state') == 'PASSED':
            # 수정 완료·계속 가·돌아가처럼 자동 실행이 예약된 검증이면 실행 시작 알림으로 한 번만 말한다
            return '' if status.get('auto_run') else PASSED_WAIT
        # 실패 항목은 이벤트 data가 아니라 상태(status.report)에 들어 있다
        failed = ((status.get('report') or {}).get('failed_kinds') or [])
        reason = FAIL_REASON.get(failed[0], '') if failed else ''
        return ('검증을 통과하지 못했어요.' + (f' {reason}.' if reason else '') + _fail_place(status) +
                ' 화면의 빨간 점을 보고 다시 고쳐 주세요. 수정 모드로 돌아왔어요.')
    if kind == 'execution_start':
        return PASSED_MOVE if actor == 'voice' else MOVE_START   # voice = 예약된 자동 실행 (검증 통과 직후)
    if kind == 'execution_end':
        if ev.get('state') != 'DONE' or data.get('next'):
            return ''   # 멈춤: stop_motion 결과로 말하고 5초 뒤 묻는다 / 돌아가서 수정으로: back_to_edit에서 말한다
        last = (status.get('last_exec') or {})
        if status.get('stations'):
            return _station_arrival(status, last)
        if last.get('phase') != 'outbound':
            return ASK_HOME
        # 거꾸로 오던 길에서 멈추고 돌아가 목표 자리로 돌아왔으면 도착 인사 없이 다시 묻는다
        return ASK_RETURN if last.get('kind') == 'return_reverse' else ASK_ARRIVED
    if kind == 'back_to_edit':
        return BACK_TO_EDIT
    if kind == 'stations_ready':             # 무장됐다: 처음 움직이기 전에 한 번 묻는다
        base = ((status.get('stations') or {}).get('labels') or {}).get('S', '바구니')
        return f'{base}{_ro(base)} 이동할까요?'
    if kind == 'auto_run_cancelled':
        why = str(data.get('reason', ''))
        if why.startswith(('validation failed', 'execution finished', 'edit mode on', 'new validation')):
            return ''   # 실패·완료는 따로 말하고, 수정 모드·새 검증은 사용자가 방금 시킨 것이다
        return '자동 이동 예약이 취소됐어요. 경로를 확인하고 다시 수정 완료라고 말씀해 주세요.'
    if kind == 'regenerated':
        return '로봇 자세가 바뀌어 경로를 다시 만들었어요. 고친 내용은 사라졌어요. 다시 수정 완료라고 말씀해 주세요.'
    if kind == 'calib':
        return CALIB_NOTICE.get(data.get('calib', ''), '')
    if kind == 'hand_refused':
        return ''   # 손 입력 거절 이유는 시뮬레이터 화면(알림 줄, 손가락 옆)에만 보인다 (말하지 않는다)
    if kind == 'point_select' and actor == 'mouse':
        return ''   # 클릭은 화면 표시로 충분하다 (말하지 않는다)
    return ''
