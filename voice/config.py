"""config.yaml 읽기. 파일 안의 상대 경로는 설정 파일이 있는 폴더 기준이다."""

from dataclasses import dataclass
from pathlib import Path

import yaml

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = PACKAGE_DIR / 'config.yaml'
AMOUNTS = ('small', 'medium', 'large')


class ConfigError(ValueError):
    pass


@dataclass
class Config:
    wake_word: str
    wake_aliases: tuple[str, ...]
    wake_seconds: float
    wake_fuzzy: bool
    call_words: tuple[str, ...]
    model: str
    voice: str
    prompt_path: Path
    style: str
    amount_cm: dict[str, float]
    fail_mode: bool
    fail_reason: str
    mic: str
    mute_mic_while_speaking: bool
    log_dir: Path

    @property
    def wake_spellings(self) -> tuple[str, ...]:
        return (self.wake_word, *self.wake_aliases)

    @property
    def prompt_version(self) -> str:
        return self.prompt_path.stem

    def system_instruction(self) -> str:
        text = self.prompt_path.read_text(encoding='utf-8')
        return text.replace('{wake_word}', self.wake_word).replace('{style}', self.style)


def load_config(path: Path = DEFAULT_CONFIG) -> Config:
    path = Path(path).resolve()
    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except OSError as exc:
        raise ConfigError(f'설정 파일을 읽지 못했습니다: {path} ({exc})') from exc
    base = path.parent

    amount_cm = {k: float(v) for k, v in (raw.get('amount_cm') or {}).items()}
    if set(amount_cm) != set(AMOUNTS):
        raise ConfigError(f'amount_cm에는 {", ".join(AMOUNTS)} 세 값이 있어야 합니다: {amount_cm}')

    prompt_path = base / raw.get('prompt', 'prompts/system_v1.md')
    if not prompt_path.is_file():
        raise ConfigError(f'지시문 파일이 없습니다: {prompt_path}')

    return Config(
        wake_word=str(raw.get('wake_word', '쿠옹아')).strip(),
        wake_aliases=tuple(str(a).strip() for a in raw.get('wake_aliases') or ()),
        wake_seconds=float(raw.get('wake_seconds', 30)),
        wake_fuzzy=bool(raw.get('wake_fuzzy', True)),
        call_words=tuple(str(a).strip() for a in raw.get('call_words', ['야']) or ()),
        model=str(raw.get('model', 'gemini-3.8-live')).strip(),
        voice=str(raw.get('voice') or '').strip(),
        prompt_path=prompt_path,
        style=str(raw.get('style') or '').strip(),
        amount_cm=amount_cm,
        fail_mode=bool(raw.get('fail_mode', False)),
        fail_reason=str(raw.get('fail_reason', 'unreachable')),
        mic=str(raw.get('mic') or '').strip(),
        mute_mic_while_speaking=bool(raw.get('mute_mic_while_speaking', True)),
        log_dir=base / raw.get('log_dir', 'logs'),
    )
