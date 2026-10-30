"""대화 기록 (JSONL). 대화 한 번(사용자 말 → 함수 → AI 말)마다 한 줄을 쓴다."""

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class Turn:
    user: str = ''                       # 텍스트 모드: 입력 문장, 마이크 모드: 입력 자막
    calls: list[dict[str, Any]] = field(default_factory=list)  # {name, args, result}
    ai: str = ''                         # AI 말 (출력 자막)
    started: float = field(default_factory=time.time)
    note: str = ''                       # 끊김 등 특이 사항
    done_keys: set = field(default_factory=set)  # 이 말에서 이미 실행한 (함수, 인자). 기록하지 않는다

    @property
    def empty(self) -> bool:
        return not (self.user.strip() or self.calls or self.ai.strip())


class TurnLogger:
    def __init__(self, log_dir: Path, *, prompt_version: str, input_mode: str, model: str) -> None:
        log_dir.mkdir(parents=True, exist_ok=True)
        self.run_id = time.strftime('%Y%m%d-%H%M%S')
        self.path = log_dir / f'{self.run_id}_{input_mode}.jsonl'
        self._common = {'run': self.run_id, 'prompt_version': prompt_version,
                        'input_mode': input_mode, 'model': model}
        self._count = 0

    def write(self, turn: Turn, *, fail_mode: bool) -> None:
        self._count += 1
        record = {
            **self._common,
            'turn': self._count,
            'time': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(turn.started)),
            'fail_mode': fail_mode,
            **{k: v for k, v in asdict(turn).items() if k not in ('started', 'done_keys')},
        }
        if not record['note']:
            del record['note']
        with self.path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')
