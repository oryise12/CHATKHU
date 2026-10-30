"""시뮬레이터 이벤트 → 쿠옹이가 말할 문장, 수정 모드 안내 (AI 없이)."""
import queue

from voice import tools
from voice.sim_link import (ARM_SAY, ASK_ARRIVED, ASK_RUN, BACK_TO_EDIT, EDIT_GUIDE, MOVE_START, PASSED_MOVE,
                            PASSED_WAIT, QUESTION_FUNCTIONS, SimEditor, _announcement)


def ev(kind, state='GENERATED', actor='sim', data=None, **status):
    return {'kind': kind, 'state': state, 'actor': actor, 'data': data or {}, 'status': status}


def test_passed_runs_without_a_question():
    assert _announcement(ev('validation', 'PASSED', auto_run={'request_id': 'r'})) == ''
    assert _announcement(ev('execution_start', 'EXECUTING', actor='voice')) == PASSED_MOVE
    assert '?' not in PASSED_MOVE
    assert _announcement(ev('validation', 'PASSED')) == PASSED_WAIT            # 버튼 V로 검증: Y를 안내
    assert _announcement(ev('execution_start', 'EXECUTING', actor='key')) == MOVE_START


def test_failure_says_why_where_and_back_to_editing():
    t = _announcement(ev('validation', 'FAILED', phase='outbound',
                         report={'failed_kinds': ['collision'], 'fail_at_pct': 63.0}))
    assert '책상이나 로봇 자신과 부딪혀요' in t and '가는 길의 약 60% 지점' in t and '수정 모드' in t


def test_arrival_back_to_edit_and_cancelled_booking():
    t = _announcement(ev('execution_end', 'DONE', last_exec={'phase': 'outbound', 'kind': 'outbound'}))
    assert t == ASK_ARRIVED
    assert _announcement(ev('back_to_edit')) == BACK_TO_EDIT and not BACK_TO_EDIT.endswith('?')
    assert _announcement(ev('auto_run_cancelled', data={'reason': 'validation failed'})) == ''
    assert _announcement(ev('auto_run_cancelled', data={'reason': 'edit mode on'})) == ''
    assert '자동 이동 예약이 취소' in _announcement(ev('auto_run_cancelled', data={'reason': 'voice link lost'}))


def test_calibration_is_spoken_hand_refusals_only_on_screen():
    assert '다시 보정' in _announcement(ev('calib', data={'calib': 'moved'}))
    assert _announcement(ev('calib', data={'calib': 'verified'})) == ''
    for reason in ('not_edit_mode', 'calib_unverified', 'no_depth', 'depth_lost', 'rgb_depth_out_of_sync'):
        assert _announcement(ev('hand_refused', data={'reason': reason})) == ''   # 시뮬레이터 화면에만 보인다


def test_edit_guide_is_one_sentence_about_finishing():
    assert EDIT_GUIDE == '경로 수정 후 수정 완료를 말해 주세요.'


def test_finish_asks_first_and_only_yes_validates_and_moves():
    names = {d['name'] for d in tools.DECLARATIONS}
    assert {'start_edit', 'finish_edit', 'validate_and_move'} <= names
    decl = {d['name']: d['description'] for d in tools.DECLARATIONS}
    assert '이 경로로 할게' in decl['finish_edit'] and '검증 후 이동할까요' in decl['finish_edit']
    assert '검증 후 이동할까요' in decl['validate_and_move'] and '"어"' in decl['validate_and_move']
    assert ASK_RUN == '검증 후 이동할까요?' and QUESTION_FUNCTIONS[ASK_RUN] == {'validate_and_move'}


def test_finish_before_the_real_arm_is_armed_says_so_without_asking():
    ed = SimEditor.__new__(SimEditor)                     # no simulator: _finish only reads the last status
    ed._hand_timer, ed.announcements = None, queue.Queue()
    ed.status = {'state': 'GENERATED', 'real_arm': {'up': True, 'gate': '실물이 무장 상태가 아님 (ARM_CONFIRM)'}}
    res = ed._finish()
    assert res == {'status': 'failed', 'reason': 'arm_not_armed', 'say': ARM_SAY['ARM_NOT_ARMED']}
    assert ed.announcements.empty() and '실패' not in res['say'] and 'y' in res['say']
    ed.status['real_arm'] = {'up': False, 'gate': '실물 연결 끊김 (gone)'}
    assert ed._finish()['reason'] == 'arm_disconnected'
    ed.status['real_arm'] = {'up': True, 'gate': ''}
    assert ed._finish() == {'status': 'asking'} and ed.announcements.get_nowait() == ASK_RUN
