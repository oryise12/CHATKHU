"""호출어 창·질문 창과 함수 실행 판정 (AI·마이크 없이).

- 호출어 없는 말, 질문의 답이 아닌 말, 지난 질문에는 함수를 실행하지 않는다. 멈춤은 언제나 실행한다.
- 질문의 답은 한 번, 그 질문이 허용한 함수만, 시뮬레이터 상태(rev)가 그대로일 때만.
"""
from voice.config import load_config
from voice.session import announcement_message
from voice.sim_link import ASK_ARRIVED, ASK_HOME, ASK_STOPPED, SimEditor
from voice.wake_word import WakeGate, gate_decision


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _gate(clock):
    return WakeGate(('쿠옹아', '구공아'), 30.0, True, clock=clock, answer_seconds=20.0)


def test_no_wake_word_no_function_but_stop_always_runs():
    for name in ('start_edit', 'finish_edit', 'move_here', 'go_back'):
        r = gate_decision(name, announcement=False, wake_open=False, answer='', duplicate=False)
        assert r == {'status': 'blocked', 'reason': 'no_wake_word'}
    assert gate_decision('stop_motion', announcement=False, wake_open=False, answer='', duplicate=False) is None
    assert gate_decision('stop_motion', announcement=False, wake_open=False, answer='not_an_answer',
                         duplicate=False) is None                     # 질문 중에도, 지난 rev에도 멈춘다
    assert gate_decision('finish_edit', announcement=False, wake_open=True, answer='', duplicate=False) is None


def test_an_announcement_turn_runs_nothing_and_duplicates_are_ignored():
    r = gate_decision('go_back', announcement=True, wake_open=True, answer='allowed', duplicate=False)
    assert r['status'] == 'ignored' and r['reason'] == 'announcement'
    r = gate_decision('move_here', announcement=False, wake_open=True, answer='', duplicate=True)
    assert r == {'status': 'ignored', 'reason': 'duplicate'}


def test_question_window_takes_one_allowed_answer():
    c = Clock()
    g = _gate(c)
    g.ask(SimEditor.question_functions(ASK_STOPPED), rev=7)
    assert not g.open                                                  # 호출어 창은 열리지 않는다
    assert g.answer_check('go_back', 7) == 'allowed'
    r = gate_decision('go_back', announcement=False, wake_open=False, answer='allowed', duplicate=False)
    assert r is None
    assert g.answer_check('start_edit', 7) == 'not_an_answer'          # "계속 갈까요?"에 엉뚱한 함수
    assert gate_decision('start_edit', announcement=False, wake_open=False, answer='not_an_answer',
                         duplicate=False) == {'status': 'blocked', 'reason': 'not_an_answer'}
    g.close_question()                                                 # 답 한 번 ("아니"였어도) 뒤
    assert g.answer_check('go_back', 7) == ''                         # 더는 호출어 없이 받지 않는다


def test_question_expires_on_time_and_on_state_change():
    c = Clock()
    g = _gate(c)
    g.ask(SimEditor.question_functions(ASK_HOME), rev=3)
    assert g.answer_check('new_target', 4) == 'expired'                # 그 사이 시뮬레이터 상태가 바뀜
    g.ask(SimEditor.question_functions(ASK_HOME), rev=3)
    c.t += 21
    assert not g.question_open and g.answer_check('new_target', 3) == 'expired'


def test_answer_time_counts_from_the_end_of_the_spoken_question():
    c = Clock()
    g = _gate(c)
    g.ask(SimEditor.question_functions(ASK_HOME), rev=3)
    c.t += 5                                                           # 쿠옹이가 질문을 말하는 데 5초
    g.question_spoken()
    c.t += 18                                                          # 다 말한 뒤 18초: 아직 답할 수 있다
    assert g.answer_check('save_paths', 3) == 'allowed'


def test_wake_window_is_not_extended_by_talk_alone():
    c = Clock()
    g = _gate(c)
    assert g.observe('쿠옹아 경로 수정할래') is not None and g.open
    c.t += 20
    g.extend()                                                        # 실행된 명령이 있을 때만 세션이 부른다
    c.t += 25
    assert g.open                                                     # 45 s: 실행 뒤 다시 채운 30 s 안
    c.t += 6
    assert not g.open
    g.extend()                                                        # 닫힌 창은 다시 열리지 않는다
    assert not g.open


def test_answer_sets_for_the_app_questions():
    assert SimEditor.question_functions(ASK_STOPPED) == {'continue_motion', 'go_back'}
    assert SimEditor.question_functions(ASK_ARRIVED) == {'edit_return_path', 'start_edit', 'go_back'}
    assert SimEditor.question_functions(ASK_HOME) == {'new_target', 'save_paths'}
    assert SimEditor.question_functions('위로 올릴까요, 옆으로 옮길까요?') is None   # AI 자신의 질문은 아님


def test_a_question_announcement_says_read_it_do_not_answer_it():
    """ChatKHU 시험(2026-10-23): «…?» 알림을 질문으로 받아 답하려다 문장을 바꿨다 → 메시지에서 그대로 읽기를 못 박는다."""
    q = announcement_message(ASK_HOME)
    assert q.endswith(f'«{ASK_HOME}»') and '답하지 말고' in q
    assert '답하지 말고' not in announcement_message('검증을 통과했어요. 이동을 시작할게요.')


def test_config_defaults_are_the_safe_ones():
    cfg = load_config()
    assert cfg.call_words == ()                                       # 옆 사람의 "야"에 열리지 않는다
    assert cfg.answer_seconds == 20 and cfg.listen_while_moving is False
