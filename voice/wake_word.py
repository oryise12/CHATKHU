"""호출어("쿠옹아") 게이트.

manipulator_voice/wake_word.py에서 가져와 고쳤다.
- 호출어와 표기 변형(음성 자막에서 "재비스"로 적힘)을 설정에서 받는다.
- 원본은 명령 하나를 보내면 바로 닫혔다. 여기서는 "조금만 더"처럼 호출어 없이
  이어지는 수정을 받아야 하므로, 호출어를 들은 뒤 listen_seconds 동안 열어 둔다.
  이 시간은 함수가 실제로 실행될 때만 다시 채운다 (옆 사람 말이나 거절된 말로는 늘지 않는다).
- 앱(시뮬레이터)이 질문하면("계속 갈까요, 돌아갈까요?") 호출어 창과 따로 "질문 창"을 연다:
  그 질문의 답이 되는 함수만, 답 한 번만, answer_seconds 안에, 시뮬레이터 상태(rev)가 그대로일 때만
  호출어 없이 받는다. 부정·엉뚱한 말에 다른 함수가 불리면 막는다(not_an_answer).
- 호출어는 문장 맨 앞 두 단어 안에서만 찾는다("자", "야", "저기" 같은 말이 앞에 붙어도 된다).
  문장 중간의 글자는 보지 않으므로 짧은 표기("공아")가 "성공아" 같은 말에 걸리지 않는다
  (manipulator_voice/sim_dialog.py의 쿠옹이와 같은 방식).
- 음성 자막에서 호출어 첫 글자가 자주 틀린다("구홍아", "구공아"). fuzzy를 켜면 첫 단어가
  호출어와 글자 수가 같고 첫 글자만 다를 때도 호출어로 본다.
"""

import re
import time
from dataclasses import dataclass
from typing import Callable, Optional

# 호출어 앞뒤에 붙어도 "호출어만 부른 것"으로 보는 말
_CALL_FILLERS = {'', '야', '아', '헤이', 'hey', '저기', '어이'}
_TRIM = ' \t,.!?~…'


def _compact(text: str) -> str:
    return ''.join(ch for ch in text.casefold() if ch.isalnum())


@dataclass(frozen=True)
class WakeDetection:
    remainder: str  # 호출어를 뺀 나머지 문장

    @property
    def wake_only(self) -> bool:
        return _compact(self.remainder) in _CALL_FILLERS


def detect_wake_word(text: str, spellings: tuple[str, ...], fuzzy: bool = False) -> Optional[WakeDetection]:
    """문장 맨 앞(앞말 하나까지 허용)에 호출어가 있으면 나머지 문장을 돌려준다."""
    tokens = [t for t in _SPLIT.split(text.strip()) if t]
    allowed = {s.casefold() for s in spellings}
    wake = spellings[0] if spellings else ''
    for i in range(min(2, len(tokens))):
        tok = tokens[i].casefold()
        if tok in allowed:
            return WakeDetection(' '.join(tokens[i + 1:]))
        # 호출어 뒤에 띄어쓰기 없이 말이 붙은 경우("쿠옹아여기"): 세 글자 이상 표기만
        for sp in allowed:
            if len(sp) >= 3 and tok.startswith(sp):
                return WakeDetection(' '.join([tokens[i][len(sp):], *tokens[i + 1:]]).strip(_TRIM))
        if fuzzy and any(sounds_like(tok, form) for form in call_forms(wake)):
            return WakeDetection(' '.join(tokens[i + 1:]))
        if tok not in _LEADING_FILLERS:
            return None
    return None


# ---------------------------------------------------------------- 소리가 비슷한지 (음성 자막의 오기 대응)
# 실제 자막: 쿠옹아 → 구홍아, 구공아, 구옹아, 구웅아, 구호나, 95아(구오아) ...
_DIGITS = dict(zip('0123456789', ['영', '일', '이', '삼', '사', '오', '육', '칠', '팔', '구']))
_INIT = 'ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ'
_MED = 'ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ'
_FIN = ['', 'ㄱ', 'ㄲ', 'ㄳ', 'ㄴ', 'ㄵ', 'ㄶ', 'ㄷ', 'ㄹ', 'ㄺ', 'ㄻ', 'ㄼ', 'ㄽ', 'ㄾ', 'ㄿ', 'ㅀ', 'ㅁ', 'ㅂ', 'ㅄ', 'ㅅ', 'ㅆ',
        'ㅇ', 'ㅈ', 'ㅊ', 'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ']
# 서로 헷갈리기 쉬운 소리 묶음 (같은 묶음 안의 차이는 작게 친다)
_INIT_NEAR = [set('ㄱㄲㅋ'), set('ㅇㅎ'), set('ㄷㄸㅌ'), set('ㅂㅃㅍ'), set('ㅈㅉㅊ'), set('ㄴㅇ')]
_MED_NEAR = [set('ㅗㅜㅝㅘㅛㅠ'), set('ㅓㅏㅕㅑ'), set('ㅐㅔㅒㅖ')]
_FIN_NEAR = [{'', 'ㅇ', 'ㄴ'}]


def _jamo(ch: str):
    code = ord(ch) - 0xAC00
    if not 0 <= code < 11172:
        return None
    return _INIT[code // 588], _MED[(code % 588) // 28], _FIN[code % 28]


def _part(a: str, b: str, near) -> float:
    if a == b:
        return 0.0
    return 0.3 if any(a in g and b in g for g in near) else 1.0


def _syllable_cost(x, y) -> float:
    return _part(x[0], y[0], _INIT_NEAR) + _part(x[1], y[1], _MED_NEAR) + _part(x[2], y[2], _FIN_NEAR)


def call_forms(wake: str) -> tuple[str, ...]:
    """부르는 말의 여러 꼴: 쿠옹아 → 쿠옹아, 쿠옹이야, 쿠옹이 ("누구야", "누구아" 어느 쪽으로 불러도)."""
    base = wake[:-1] if wake.endswith(('아', '야')) and len(wake) >= 3 else wake
    return tuple(dict.fromkeys((wake, base + '아', base + '이야', base + '이')))


def sounds_like(token: str, wake: str, limit: float = 1.5) -> bool:
    """첫 단어가 호출어와 글자 수가 같고, 음절마다 비슷한 소리로만 다를 때 True.

    음절마다 초성·중성·종성을 비교한다. 같은 묶음 안의 차이(ㄱ/ㅋ, ㅇ/ㅎ, ㅗ/ㅜ, 받침 ㅇ/없음)는 0.3,
    다른 소리는 1.0. 한 음절이라도 1.0을 넘거나(두 군데 이상 크게 다름) 합이 limit을 넘으면 아니다.
    숫자는 한글 읽기로 바꿔 본다(95아 → 구오아).
    """
    token = ''.join(_DIGITS.get(c, c) for c in token)
    a, b = [_jamo(c) for c in token], [_jamo(c) for c in wake]
    if len(a) != len(b) or len(b) < 3 or None in a or None in b:
        return False
    costs = [_syllable_cost(x, y) for x, y in zip(a, b)]
    return max(costs) <= 1.0 and sum(costs) <= limit


_SPLIT = re.compile(r"[\s,.!?~…·'\"]+")
_LEADING_FILLERS = {'자', '야', '아', '저기', '어이', '헤이', 'hey', '음', '어'}


class WakeGate:
    """호출어 대기 ↔ 대화 중. 이벤트 루프에서만 쓴다."""

    def __init__(self, spellings: tuple[str, ...], listen_seconds: float, fuzzy: bool = False,
                 clock: Callable[[], float] = time.monotonic, call_words: tuple[str, ...] = (),
                 answer_seconds: float = 20.0) -> None:
        self.spellings = spellings
        self.call_words = {w.casefold() for w in call_words}
        self.fuzzy = fuzzy
        self.listen_seconds = listen_seconds
        self.answer_seconds = answer_seconds
        self._clock = clock
        self._deadline: Optional[float] = None
        self._question: Optional[tuple[frozenset, Optional[int], float]] = None   # (함수들, rev, 마감)

    @property
    def open(self) -> bool:
        return self._deadline is not None and self._clock() < self._deadline

    def observe(self, text: str) -> Optional[WakeDetection]:
        """사용자 말을 본다. 호출어가 있으면 게이트를 연다."""
        detection = detect_wake_word(text, self.spellings, self.fuzzy)
        if detection is None and _compact(text) in self.call_words:
            detection = WakeDetection('')   # "야" 한 마디만: 부르는 말로 본다 (문장 앞에 붙은 "야"는 아니다)
        if detection is not None:
            self._deadline = self._clock() + self.listen_seconds
        return detection

    def open_now(self) -> None:
        """호출어 창을 지금 연다 (모든 함수). 앱의 질문에는 ask()를 쓴다."""
        self._deadline = self._clock() + self.listen_seconds

    # ------------------------------------------------------------------ 앱의 질문 (질문 창)
    def ask(self, functions, rev: Optional[int] = None) -> None:
        """앱이 질문했다: 그 질문의 답이 되는 함수(functions)만, 답 한 번, answer_seconds 안에 호출어 없이 받는다.
        rev: 질문할 때 시뮬레이터 상태 번호. 그 사이 상태가 바뀌면 질문은 지난 것이다."""
        self._question = (frozenset(functions), rev, self._clock() + self.answer_seconds)

    def question_spoken(self) -> None:
        """AI가 질문을 다 말했다: 답할 시간(answer_seconds)을 이때부터 센다 (말하는 동안 시간이 줄지 않게)."""
        if self._question is not None:
            functions, rev, _ = self._question
            self._question = (functions, rev, self._clock() + self.answer_seconds)

    @property
    def question_open(self) -> bool:
        return self._question is not None and self._clock() < self._question[2]

    def answer_check(self, name: str, rev: Optional[int] = None) -> str:
        """함수 name이 열린 질문의 답인가: '' (질문 없음), 'allowed', 'not_an_answer', 'expired'."""
        q = self._question
        if q is None:
            return ''
        functions, qrev, deadline = q
        if self._clock() >= deadline or (qrev is not None and rev is not None and rev != qrev):
            self._question = None
            return 'expired'
        return 'allowed' if name in functions else 'not_an_answer'

    def close_question(self) -> None:
        """답을 한 번 받았다 (함수를 불렀든 "아니"였든): 질문 창을 닫는다."""
        self._question = None

    def extend(self) -> None:
        """열려 있으면 대기 시간을 다시 채운다 (대화가 이어지는 동안)."""
        if self.open:
            self._deadline = self._clock() + self.listen_seconds

    def expire_if_due(self) -> bool:
        """대기 시간이 지났으면 닫고 True를 돌려준다. 지난 질문 창도 닫는다."""
        if self._question is not None and self._clock() >= self._question[2]:
            self._question = None
        if self._deadline is not None and self._clock() >= self._deadline:
            self._deadline = None
            return True
        return False


def gate_decision(name: str, *, announcement: bool, wake_open: bool, answer: str, duplicate: bool) -> Optional[dict]:
    """AI가 부른 함수를 실행할지. None = 실행, dict = 실행하지 않고 AI에게 돌려줄 결과.

    answer: WakeGate.answer_check(name, rev)의 값. 멈춤은 언제나 실행한다 (호출어·질문·rev와 무관)."""
    if name == 'stop_motion':
        return {'status': 'ignored', 'reason': 'duplicate'} if duplicate else None
    if announcement:
        # 앱 알림(질문)을 말하는 차례: 사용자가 답한 것이 아니므로 어떤 함수도 실행하지 않는다
        return {'status': 'ignored', 'reason': 'announcement'}
    if not (wake_open or answer == 'allowed'):
        reason = {'not_an_answer': 'not_an_answer', 'expired': 'question_expired'}.get(answer, 'no_wake_word')
        return {'status': 'blocked', 'reason': reason}
    if duplicate:
        # 한 말에서 같은 함수·인자를 다시 부르면 실행하지 않는다 (같은 수정이 두 번 적용되지 않게)
        return {'status': 'ignored', 'reason': 'duplicate'}
    return None
