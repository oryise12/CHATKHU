"""AI에게 주는 함수 선언과 가짜 처리기.

아직 경로를 실제로 고치지 않는다. 함수 이름과 인자를 받아 가짜 결과를 돌려준다.
fail 모드가 켜져 있으면 수정 함수(move_here, move_region)는 실패를 돌려준다.
"""

from typing import Any

DIRECTIONS = ['up', 'down', 'left', 'right', 'toward_me', 'away']
AMOUNTS = ['small', 'medium', 'large']
REGIONS = ['start', 'middle', 'end', 'whole']

_DIRECTION = {
    'type': 'STRING', 'enum': DIRECTIONS,
    'description': '사용자 몸 기준. up 위, down 아래, left 왼쪽(A 쪽), right 오른쪽(B 쪽), '
                   'toward_me 내 쪽(사용자 쪽), away 저쪽(사용자 반대쪽)',
}
_AMOUNT = {
    'type': 'STRING', 'enum': AMOUNTS,
    'description': 'small 조금, medium 보통, large 많이. 말하지 않았으면 small',
}
_DISTANCE = {
    'type': 'NUMBER',
    'description': '사용자가 "5센티", "2cm"처럼 숫자로 거리를 말했을 때만 넣는다(cm). 넣으면 amount 대신 이 값을 쓴다.',
}
MAX_DISTANCE_CM = 15.0

DECLARATIONS = [
    {
        'name': 'move_here',
        'description': '사용자가 손가락으로 가리킨 곳("여기", "이쪽", "이 부분")의 경로를 옮긴다.',
        'parameters': {'type': 'OBJECT', 'properties': {
            'direction': _DIRECTION, 'amount': _AMOUNT, 'distance_cm': _DISTANCE,
        }, 'required': ['direction', 'amount']},
    },
    {
        'name': 'move_region',
        'description': '경로의 구간을 옮긴다. start 시작 부분, middle 가운데, end 끝 부분, whole 전체.',
        'parameters': {'type': 'OBJECT', 'properties': {
            'region': {'type': 'STRING', 'enum': REGIONS},
            'direction': _DIRECTION, 'amount': _AMOUNT, 'distance_cm': _DISTANCE,
        }, 'required': ['region', 'direction', 'amount']},
    },
    {
        'name': 'adjust_amount',
        'description': '직전에 옮긴 양이 모자라거나 지나치다고 할 때 부른다. 직전 수정을 취소하고, 같은 곳·같은 방향으로 '
                       '한 단계 큰(more) 또는 작은(less) 양으로 다시 옮긴다. 단계는 조금 → 보통 → 많이. '
                       'more 예: "조금만 더", "좀 더", "아직 모자라", "부족해", "더 가". '
                       'less 예: "너무 갔어", "너무 많이 갔어", "좀 줄여", "살짝만 덜", "과해", "지나쳤어", "아까보다 조금만".',
        'parameters': {'type': 'OBJECT', 'properties': {
            'change': {'type': 'STRING', 'enum': ['more', 'less']},
        }, 'required': ['change']},
    },
    {'name': 'stop_motion',
     'description': '로봇 이동(또는 검증)을 즉시 멈춘다. 예: "멈춰", "정지", "스톱", "그만", "잠깐 멈춰"'},
    {'name': 'continue_motion',
     'description': '멈춘 뒤, 멈춘 자리에서 원래 가던 쪽으로 마저 간다. 예: "계속 가", "다시 가", "마저 가", "가던 대로 가"'},
    {'name': 'go_back',
     'description': '멈춘 뒤 또는 목표에 도착한 뒤, 가던 길을 거꾸로 따라 돌아간다. '
                    '예: "돌아가", "돌아와", "반대로 가", "그냥 돌아가", "가던 길 반대로"'},
    {'name': 'edit_return_path',
     'description': '목표에 도착한 뒤, 돌아오는 길을 고치기 시작한다. 예: "돌아오는 길 수정할래", "수정할래", "고칠래"'},
    {'name': 'new_target',
     'description': '처음 자리로 돌아온 뒤, 새 목표와 새 가는 길을 만든다. 예: "새 목표", "다시 해 보자"'},
    {'name': 'save_paths',
     'description': '지금까지 움직인 가는 길과 돌아오는 길을 파일로 저장한다. 예: "경로 저장해", "저장해 줘". '
                    '처음 자리로 돌아온 뒤 "새 목표로 다시 해 볼까요?"에 아니라고 답했을 때도 부른다.'},
    {'name': 'undo_edit', 'description': '바로 직전 수정 하나를 되돌린다. 예: "되돌려", "방금 거 취소"'},
    {'name': 'reset_edit', 'description': '모든 수정을 버리고 처음 경로로 돌아간다. 예: "처음으로", "초기화"'},
    {'name': 'finish_edit', 'description': '수정을 마치고 검증을 시작한다. 움직이지는 않는다. 예: "수정 완료", "됐어", "다 됐어", "이제 끝"'},
    {'name': 'approve_move',
     'description': '"이동할까요, 더 고칠까요?"에 움직이라고 답했을 때만 부른다. 예: "이동해", "응", "가", "움직여"'},
    {'name': 'keep_editing',
     'description': '"이동할까요, 더 고칠까요?"에 더 고치겠다고 답했을 때 부른다. 예: "더 고칠래", "좀 더 고칠게", "아직 더 고칠 거야"'},
]
NAMES = {d['name'] for d in DECLARATIONS}
EDIT_FUNCTIONS = {'move_here', 'move_region'}
MOTION_FUNCTIONS = {'stop_motion', 'continue_motion', 'go_back', 'edit_return_path', 'new_target'}


class AmountLadder:
    """직전 수정(함수, 인자, 단계)을 기억해 "조금만 더"·"좀 덜"을 조금 ↔ 보통 ↔ 많이 한 단계씩 바꾼다."""

    def __init__(self, amount_cm: dict[str, float]) -> None:
        self.amount_cm = amount_cm
        self.last: tuple[str, dict[str, Any]] | None = None

    def level(self, args: dict[str, Any]) -> str:
        if 'distance_cm' in args:  # 숫자로 말한 거리는 가장 가까운 단계로 본다
            return min(AMOUNTS, key=lambda a: abs(self.amount_cm[a] - float(args['distance_cm'])))
        return args['amount']

    def remember(self, name: str, args: dict[str, Any]) -> None:
        self.last = (name, {k: v for k, v in args.items() if k != 'distance_cm'} | {'amount': self.level(args)})

    def forget(self) -> None:
        self.last = None

    def step(self, change: str) -> tuple[str, dict[str, Any], str] | str:
        """(함수, 새 인자, 이전 단계) 또는 실패 이유."""
        if self.last is None:
            return 'no_last_move'
        if change not in ('more', 'less'):
            return 'bad_change'
        name, args = self.last
        i = AMOUNTS.index(args['amount']) + (1 if change == 'more' else -1)
        if i >= len(AMOUNTS):
            return 'already_max'
        if i < 0:
            return 'already_min'
        return name, dict(args, amount=AMOUNTS[i]), args['amount']


def tools_setup() -> list[dict[str, Any]]:
    return [{'functionDeclarations': DECLARATIONS}]


class FakeEditor:
    def __init__(self, amount_cm: dict[str, float], fail_mode: bool, fail_reason: str) -> None:
        self.amount_cm = amount_cm
        self.fail_mode = fail_mode
        self.fail_reason = fail_reason
        self.ladder = AmountLadder(amount_cm)

    def handle(self, name: str, args: dict[str, Any]) -> dict[str, Any]:
        if name not in NAMES:
            return {'status': 'error', 'reason': f'unknown_function:{name}'}
        if name == 'adjust_amount':
            step = self.ladder.step(args.get('change'))
            if isinstance(step, str):
                return {'status': 'failed', 'reason': step}
            move, new_args, old = step
            result = self.handle(move, new_args)
            if result.get('status') == 'applied':
                result['replaced_cm'] = _number(self.amount_cm[old])  # 직전 수정은 취소하고 이 양으로 다시 옮김
            return result
        if name not in EDIT_FUNCTIONS:
            self.ladder.forget()
        if name in EDIT_FUNCTIONS:
            bad = _check_args(name, args)
            if bad:
                return {'status': 'error', 'reason': bad}
            if self.fail_mode:
                return {'status': 'failed', 'reason': self.fail_reason}
            self.ladder.remember(name, args)
            result = {'status': 'applied',
                      'target': 'here' if name == 'move_here' else args['region'],
                      'direction': args['direction'],
                      'moved_cm': _number(_distance(args, self.amount_cm)),
                      'path': 'both'}
            return result
        return {'status': 'applied'}


def _check_args(name: str, args: dict[str, Any]) -> str:
    if args.get('direction') not in DIRECTIONS:
        return 'bad_direction'
    if args.get('amount') not in AMOUNTS:
        return 'bad_amount'
    if name == 'move_region' and args.get('region') not in REGIONS:
        return 'bad_region'
    if 'distance_cm' in args:
        d = args['distance_cm']
        if isinstance(d, bool) or not isinstance(d, (int, float)) or not 0 < d <= MAX_DISTANCE_CM:
            return 'bad_distance'
    return ''


def _distance(args: dict[str, Any], amount_cm: dict[str, float]) -> float:
    if 'distance_cm' in args:
        return round(float(args['distance_cm']), 1)
    return amount_cm[args['amount']]


def _number(value: float) -> float | int:
    return int(value) if float(value).is_integer() else value


def format_call(name: str, args: dict[str, Any]) -> str:
    return f'{name}({", ".join(f"{k}={v}" for k, v in args.items())})'
