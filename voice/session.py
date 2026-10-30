"""ChatKHU Gemini Live 세션 (원시 WebSocket JSON).

manipulator_voice/gemini_live.py의 흐름(수신·송신 동시 실행, GoAway·끊김 시 세션
재개 핸들로 재연결, 호출어 게이트)을 google-genai SDK 없이 다시 썼다.

대화 한 번의 끝:
  AI가 함수를 부른 차례가 끝날 때 turnComplete가 한 번 오고, toolResponse를 보낸 뒤
  AI가 결과를 말하는 차례가 끝날 때 또 온다. 그래서 함수를 부른 대화는 함수 호출 뒤
  AI가 말을 했거나 turnComplete를 두 번 받았을 때 끝난 것으로 본다.
"""

import asyncio
import base64
import json
import queue
import re
from typing import Optional

import websockets

from voice import console, gateway
from voice.config import Config
from voice.tools import FakeEditor, format_call, tools_setup
from voice.turn_log import Turn, TurnLogger
from voice.wake_word import WakeGate, gate_decision

INPUT_AUDIO_MIME = 'audio/pcm;rate=16000'
TRANSCRIPTION_LANGUAGE = 'ko-KR'
RECONNECT_DELAYS_S = (1, 2, 4, 8, 8)
SETUP_TIMEOUT_S = 15
TEXT_TURN_TIMEOUT_S = 30
MIC_MUTE_TAIL_S = 0.3
# 마이크 모드: 함수 호출이 입력 자막보다 먼저 오면 호출어 확인을 이만큼 기다린다
WAKE_GRACE_S = 0.6
STOP_WORDS = ('멈춰', '멈추', '정지', '스톱', '스탑', 'stop')
STOP_NEGATIONS = ('멈추지마', '멈추지말', '정지하지마', '멈추면안')
QUIT_WORDS = {'q', 'quit', 'exit', '종료'}

_TRANSCRIPT_MARKUP = re.compile(r'<[^<>]*>|\{[^{}]*\}')


def announcement_message(text: str) -> str:
    """시뮬레이터 알림을 AI가 읽게 보내는 메시지. 질문 문장(«…?»)이면 AI가 그 질문에 답하려 들어 문장을 바꿔
    말했다(2026-10-23 ChatKHU 시험: 도착 질문 2번 모두). 그래서 메시지 안에서 다시 '그대로 읽기'를 못 박는다."""
    head = '[시뮬레이터 알림] 아래 « » 안의 문장을 한 글자도 바꾸지 말고 사용자에게 그대로 한 번 말한다. 함수는 부르지 않는다.'
    if text.endswith('?'):
        head += ' 이 문장은 앱이 사용자에게 하는 질문이다. 네가 답하지 말고 질문 그대로 읽는다.'
    return f'{head} «{text}»'


def clean_transcript(text: str) -> str:
    """자막에 섞여 오는 표시(예: '<noise>')를 뺀다."""
    return ' '.join(_TRANSCRIPT_MARKUP.sub('', text).split())


class LiveError(RuntimeError):
    """재연결로 해결할 수 없는 문제."""


class _Reconnect(Exception):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


class LiveAssistant:
    """mic과 lines 중 하나만 준다. lines는 입력 문장이 들어오는 Queue (None = 입력 끝)."""

    def __init__(self, cfg: Config, editor: FakeEditor, logger: TurnLogger, *,
                 mic=None, speaker=None, lines: Optional[asyncio.Queue] = None,
                 echo_input: bool = False) -> None:
        if (mic is None) == (lines is None):
            raise ValueError('mic과 lines 중 정확히 하나를 지정하세요.')
        self._cfg = cfg
        self._system = cfg.system_instruction()
        self._editor = editor
        self._logger = logger
        self._mic = mic
        self._speaker = speaker
        self._lines = lines
        self._echo_input = echo_input
        self._gate = WakeGate(cfg.wake_spellings, cfg.wake_seconds, cfg.wake_fuzzy, call_words=cfg.call_words,
                              answer_seconds=cfg.answer_seconds)
        if not cfg.wake_required:                  # 호출어 없이: 처음부터 열어 두고 닫지 않는다
            self._gate.listen_seconds = float('inf')
            self._gate.open_now()
        self._handle: Optional[str] = None
        self._connected_once = False
        self._established = False
        self._tool_tasks: set[asyncio.Task] = set()
        self._turn_done = asyncio.Event()
        self._reset_turn()

    def _reset_turn(self) -> None:
        self._fast_stopped = False
        self._turn = Turn()
        self._turn_has_tool = False
        self._spoke_after_tool = False
        self._completes_after_tool = 0
        self._user_printed = False

    # ------------------------------------------------------------------ 연결
    def _setup_message(self) -> dict:
        generation = {'responseModalities': ['AUDIO']}
        if self._cfg.voice:
            generation['speechConfig'] = {
                'voiceConfig': {'prebuiltVoiceConfig': {'voiceName': self._cfg.voice}}}
        return {'setup': {
            'model': 'models/' + self._cfg.model,
            'generationConfig': generation,
            'systemInstruction': {'parts': [{'text': self._system}]},
            'tools': tools_setup(),
            'inputAudioTranscription': {'languageCodes': [TRANSCRIPTION_LANGUAGE]},
            'outputAudioTranscription': {},
            'contextWindowCompression': {'slidingWindow': {}},
            'sessionResumption': {'handle': self._handle} if self._handle else {},
        }}

    async def run(self) -> None:
        if hasattr(self._editor, 'on_activity'):
            # 손(핀치·마우스)으로 고치는 동안은 대화가 이어지는 것으로 본다: 열린 호출어 창만 다시 채운다
            loop = asyncio.get_running_loop()
            self._editor.on_activity = lambda: loop.call_soon_threadsafe(self._gate.extend)
        watcher = asyncio.create_task(self._watch_gate())
        try:
            await self._run_with_reconnect()
        finally:
            watcher.cancel()
            await asyncio.gather(watcher, return_exceptions=True)

    async def _run_with_reconnect(self) -> None:
        failures = 0
        while True:
            self._established = False
            try:
                await self._run_connection()
                return
            except _Reconnect as exc:
                console.info(f'{exc.reason}. 새 토큰으로 다시 연결합니다.')
                failures = 0
                continue
            except (gateway.GatewayError, OSError, TimeoutError,
                    websockets.WebSocketException, json.JSONDecodeError) as exc:
                reason = console.redact(f'{type(exc).__name__}: {exc}')
                if not self._connected_once:
                    raise LiveError(f'ChatKHU에 연결하지 못했습니다 ({reason}).') from None
            if self._established:
                failures = 0
            failures += 1
            if failures > len(RECONNECT_DELAYS_S):
                raise LiveError(f'재연결에 {failures - 1}번 실패했습니다 (마지막 이유: {reason}).')
            delay = RECONNECT_DELAYS_S[failures - 1]
            console.warn(f'연결이 끊겼습니다 ({reason}). {delay}초 후 새 토큰으로 다시 연결합니다.')
            await asyncio.sleep(delay)
            if failures >= 2 and self._handle is not None:
                console.warn('세션 재개가 계속 실패해 새 세션으로 연결합니다 (앞 대화 맥락은 사라집니다).')
                self._handle = None

    async def _run_connection(self) -> None:
        resumed = self._handle is not None
        session = await asyncio.to_thread(gateway.new_session, self._cfg.model)  # 1회용 토큰
        ws = await gateway.open_socket(session)
        try:
            await ws.send(json.dumps(self._setup_message()))
            await self._wait_setup(ws)
            self._connected_once = True
            self._established = True
            console.info(f'ChatKHU 연결됨 (모델 {self._cfg.model}{", 이전 대화 재개" if resumed else ""}).')
            if self._mic is not None:
                self._mic.discard_pending()
            receive = asyncio.create_task(self._receive_loop(ws))
            send = asyncio.create_task(self._mic_loop(ws) if self._mic else self._text_loop(ws))
            announce = asyncio.create_task(self._announce_loop(ws))
            try:
                done, _ = await asyncio.wait({receive, send}, return_when=asyncio.FIRST_COMPLETED)
            finally:  # 종료(Ctrl+C)로 취소될 때도 남은 작업을 정리한다
                for task in (receive, send, announce, *self._tool_tasks):
                    task.cancel()
                await asyncio.gather(receive, send, announce, *self._tool_tasks, return_exceptions=True)
            for task in done:
                if not task.cancelled() and task.exception() is not None:
                    raise task.exception()
            if send in done:
                return  # 입력 끝 (종료)
            raise _Reconnect('서버가 연결을 닫았습니다')
        finally:
            await ws.close()
            if not self._turn.empty:
                self._finish_turn(note='연결 끊김')

    async def _wait_setup(self, ws) -> None:
        async with asyncio.timeout(SETUP_TIMEOUT_S):
            async for raw in ws:
                msg = json.loads(raw)
                if 'setupComplete' in msg:
                    return
                if 'error' in msg:
                    raise LiveError(f'setup 거부: {str(msg["error"])[:300]}')
        raise _Reconnect('setup 중 연결이 닫혔습니다')

    # ------------------------------------------------------------------ 호출어
    def _observe(self, text: str) -> None:
        self._maybe_fast_stop(text)
        was_open = self._gate.open
        detection = self._gate.observe(text)
        if detection is not None and detection.wake_only and not self._gate.question_open:
            # 시연 지점: S(바구니)에서 "쿠옹아"라고만 부르면 앱이 다음 곳을 묻는다 (어디로 갈까요? / 남은 곳으로 갈까요?)
            ask = getattr(self._editor, 'wake_question', lambda: None)()
            if ask:
                self._editor.announcements.put(ask)
        if detection is not None and not was_open:
            secs = self._gate.listen_seconds
            console.info('[호출어] 인식. ' + ('이제부터 호출어 없이 명령을 받습니다.' if secs == float('inf')
                                             else f'{secs:.0f}초 동안 명령을 받습니다.'))
            self._hint('쿠옹이가 듣고 있어요')

    def _maybe_fast_stop(self, text: str) -> None:
        """"멈춰"가 들리면 AI 응답을 기다리지 않고 바로 멈춘다 (호출어 없이도). 한 말에 한 번만."""
        stop_now = getattr(self._editor, 'stop_now', None)
        if stop_now is None or getattr(self, '_fast_stopped', False):
            return
        t = re.sub(r'\s+', '', text).lower()
        if not any(w in t for w in STOP_WORDS) or any(n in t for n in STOP_NEGATIONS):
            return
        self._fast_stopped = True
        task = asyncio.get_running_loop().create_task(asyncio.to_thread(stop_now))
        task.add_done_callback(lambda tk: console.info('[즉시 정지] 멈췄습니다.')
                               if not tk.cancelled() and tk.exception() is None and tk.result() else None)
        self._tool_tasks.add(task)
        task.add_done_callback(self._tool_tasks.discard)

    def _hint(self, text: str) -> None:
        """시뮬레이터 화면의 한 줄 표시 (연결돼 있을 때만)."""
        hint = getattr(self._editor, 'hint', None)
        if hint is not None:
            hint(text)

    async def _watch_gate(self) -> None:
        while True:
            await asyncio.sleep(0.25)
            if self._gate.expire_if_due():
                self._hint(f"'{self._cfg.wake_word}'라고 불러 주세요")
                console.info(f'[호출어] 시간이 지나 대기로 돌아갑니다. 다시 "{self._cfg.wake_word}"라고 불러 주세요.')

    # ------------------------------------------------------------------ 송신
    def _moving(self) -> bool:
        return (getattr(self._editor, 'status', None) or {}).get('state') == 'EXECUTING'

    async def _mic_loop(self, ws) -> None:
        mute = self._cfg.mute_mic_while_speaking and self._speaker is not None
        while True:
            chunk = await self._mic.read_chunk()
            pcm = chunk.pcm16
            if not pcm:
                continue
            # listen_while_moving: 움직이는 동안은 AI가 말해도 마이크를 연다 ("멈춰"가 묻히지 않게, 현장 확인 후에만)
            if mute and self._speaker.is_playing(MIC_MUTE_TAIL_S) and \
                    not (self._cfg.listen_while_moving and self._moving()):
                pcm = bytes(len(pcm))  # AI가 말하는 동안은 무음을 보낸다
            await ws.send(json.dumps({'realtimeInput': {'audio': {
                'mimeType': INPUT_AUDIO_MIME, 'data': base64.b64encode(pcm).decode()}}}))

    async def _text_loop(self, ws) -> None:
        while True:
            console.raw('나> ', end='')
            line = await self._lines.get()
            if line is None:
                console.raw()
                return
            line = line.strip()
            if self._echo_input:
                console.raw(line)
            if not line:
                continue
            if line.lower() in QUIT_WORDS:
                return
            if line == '/fail':
                self._editor.fail_mode = not self._editor.fail_mode
                console.info(f'fail 모드 {"켜짐" if self._editor.fail_mode else "꺼짐"}')
                continue
            if not self._turn.empty:
                self._finish_turn(note='다음 입력 전에 끝나지 않음')
            self._turn = Turn(user=line)
            self._observe(line)
            self._turn_done.clear()
            await ws.send(json.dumps({'clientContent': {
                'turns': [{'role': 'user', 'parts': [{'text': line}]}], 'turnComplete': True}}))
            try:
                await asyncio.wait_for(self._turn_done.wait(), TEXT_TURN_TIMEOUT_S)
            except TimeoutError:
                console.warn(f'{TEXT_TURN_TIMEOUT_S}초 안에 AI 응답이 끝나지 않았습니다.')
                self._finish_turn(note='응답 시간 초과')

    # ------------------------------------------------------------------ 시뮬레이터 알림
    async def _announce_loop(self, ws) -> None:
        """시뮬레이터가 나중에 알려 주는 결과(검증 실패, 이동 시작·완료)를 AI가 그대로 말하게 한다."""
        q = getattr(self._editor, 'announcements', None)
        if q is None:
            return
        while True:
            try:
                text = await asyncio.to_thread(q.get, True, 0.5)
            except queue.Empty:
                continue
            while not self._turn.empty:  # 진행 중인 대화가 끝난 뒤에 말한다
                await asyncio.sleep(0.2)
            console.info(f'[시뮬레이터] {text}')
            if text.endswith('?'):
                # 질문의 답은 호출어 없이 받는다: 그 질문이 허용하는 함수만, 한 번, 지금 상태(rev)에서만
                functions = getattr(self._editor, 'question_functions', lambda _t: None)(text)
                if functions:
                    self._gate.ask(functions, getattr(self._editor, 'rev', None))
            self._turn = Turn(user=f'[알림] {text}')
            self._turn_done.clear()
            await ws.send(json.dumps({'clientContent': {'turns': [{'role': 'user', 'parts': [
                {'text': announcement_message(text)}]}], 'turnComplete': True}}))

    # ------------------------------------------------------------------ 수신
    async def _receive_loop(self, ws) -> None:
        async for raw in ws:
            msg = json.loads(raw)
            if msg:  # 빈 메시지({})가 많이 온다
                await self._on_message(ws, msg)

    async def _on_message(self, ws, msg: dict) -> None:
        update = msg.get('sessionResumptionUpdate')
        if update and update.get('resumable') and update.get('newHandle'):
            self._handle = update['newHandle']

        if 'toolCall' in msg:
            self._turn_has_tool = True
            self._spoke_after_tool = False
            self._completes_after_tool = 0
            task = asyncio.create_task(self._on_tool_call(ws, msg['toolCall']))
            self._tool_tasks.add(task)
            task.add_done_callback(self._tool_tasks.discard)

        if 'toolCallCancellation' in msg:
            console.warn(f'AI가 함수 호출을 취소했습니다: {msg["toolCallCancellation"]}')

        content = msg.get('serverContent')
        if content:
            heard = content.get('inputTranscription', {}).get('text')
            if heard:
                self._turn.user += heard
                self._observe(self._turn.user)
            said = content.get('outputTranscription', {}).get('text')
            if said:
                self._turn.ai += said
                self._mark_spoke()
            for part in content.get('modelTurn', {}).get('parts', []):
                data = part.get('inlineData', {}).get('data')
                if data:
                    self._mark_spoke()
                    if self._speaker is not None:
                        self._speaker.enqueue(base64.b64decode(data))
            if content.get('interrupted') and self._speaker is not None:
                self._speaker.clear()
            if content.get('turnComplete'):
                self._on_turn_complete()

        if 'error' in msg:
            console.warn(f'서버 오류 메시지: {str(msg["error"])[:300]}')
        if 'goAway' in msg:
            raise _Reconnect(f'서버가 연결 종료를 예고함(남은 시간 {msg["goAway"].get("timeLeft")})')

    def _mark_spoke(self) -> None:
        if self._turn_has_tool:
            self._spoke_after_tool = True

    def _on_turn_complete(self) -> None:
        if self._turn_has_tool and not self._spoke_after_tool:
            self._completes_after_tool += 1
            if self._completes_after_tool < 2:
                return  # 함수를 부른 차례가 끝났을 뿐. 결과를 말하는 차례를 기다린다
        self._finish_turn()

    def _print_user(self) -> None:
        if self._mic is not None and not self._user_printed and self._turn.user.strip():
            console.info(f'[사용자] {clean_transcript(self._turn.user)}')
            self._user_printed = True

    def _finish_turn(self, note: str = '') -> None:
        turn = self._turn
        self._print_user()
        self._reset_turn()
        if not turn.empty:
            turn.user = clean_transcript(turn.user)
            turn.ai = clean_transcript(turn.ai)
            turn.note = note
            if turn.ai:
                console.info(f'[AI] {turn.ai}')
            chat = getattr(self._editor, 'chat', None)
            if chat is not None:  # 시뮬레이터 화면 말풍선
                if turn.user and not turn.user.startswith('[알림]'):
                    chat('user', turn.user)
                if turn.ai:
                    chat('kuong', turn.ai)
            self._logger.write(turn, fail_mode=self._editor.fail_mode)
            if turn.user and not turn.user.startswith('[알림]'):
                self._gate.close_question()   # 질문의 답은 한 번만 (함수를 불렀든 "아니"였든)
            elif turn.user.startswith('[알림]') and turn.user.endswith('?'):
                self._gate.question_spoken()  # 답할 시간은 질문을 다 말한 뒤부터
        self._turn_done.set()

    # ------------------------------------------------------------------ 함수 호출
    async def _on_tool_call(self, ws, tool_call: dict) -> None:
        if self._mic is not None and not self._gate.open:
            await asyncio.sleep(WAKE_GRACE_S)  # 입력 자막(호출어)이 늦게 올 수 있다
        self._print_user()
        turn = self._turn
        responses = []
        for call in tool_call.get('functionCalls', []):
            name = call.get('name', '')
            args = call.get('args') or {}
            key = (name, json.dumps(args, sort_keys=True, ensure_ascii=False))
            wake_open = self._gate.open
            answer = '' if wake_open else self._gate.answer_check(name, getattr(self._editor, 'rev', None))
            result = gate_decision(name, announcement=turn.user.startswith('[알림]'), wake_open=wake_open,
                                   answer=answer, duplicate=key in turn.done_keys)
            if result is None:
                result = await asyncio.to_thread(self._editor.handle, name, args)  # 시뮬레이터 응답 대기
                turn.done_keys.add(key)
                if wake_open:
                    self._gate.extend()       # 호출어 창은 실제로 실행된 명령이 있을 때만 다시 채운다
            console.info(f'[명령] {format_call(name, args)}')
            console.info(f'[결과] {json.dumps(result, ensure_ascii=False)}')
            turn.calls.append({'name': name, 'args': args, 'result': result})
            responses.append({'id': call.get('id'), 'name': name, 'response': result})
        if responses:
            await ws.send(json.dumps({'toolResponse': {'functionResponses': responses}}))
