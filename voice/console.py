"""터미널 출력. 등록된 비밀값(API 키)은 출력 전에 가린다."""

import sys
import time

_secrets: list[str] = []
_quiet = False


def set_quiet(quiet: bool) -> None:
    """True면 info/warn을 출력하지 않는다 (평가 도구용). error와 raw는 그대로 출력."""
    global _quiet
    _quiet = quiet


def register_secret(value: str) -> None:
    if value and value not in _secrets:
        _secrets.append(value)


def redact(text: str) -> str:
    for secret in _secrets:
        text = text.replace(secret, '***')
    return text


def clock(ns: int | None = None) -> str:
    """벽시계 시각 HH:MM:SS.mmm"""
    if ns is None:
        ns = time.time_ns()
    seconds, remainder = divmod(ns, 1_000_000_000)
    return time.strftime('%H:%M:%S', time.localtime(seconds)) + f'.{remainder // 1_000_000:03d}'


def _emit(prefix: str, message: str, stream) -> None:
    lines = redact(str(message)).splitlines() or ['']
    stream.write(f'[{clock()}] {prefix}{lines[0]}\n')
    for line in lines[1:]:
        stream.write(f'{" " * 15}{line}\n')
    stream.flush()


def info(message: str) -> None:
    if not _quiet:
        _emit('', message, sys.stdout)


def warn(message: str) -> None:
    if not _quiet:
        _emit('경고: ', message, sys.stdout)


def error(message: str) -> None:
    _emit('오류: ', message, sys.stderr)


def raw(message: str = '', end: str = '\n') -> None:
    sys.stdout.write(redact(message) + end)
    sys.stdout.flush()
