"""마이크 입력과 스피커 출력 (sounddevice / PortAudio).

manipulator_voice/audio_io.py에서 복사. 스피커가 말하는 중인지(is_playing) 추가.

마이크는 장치의 실제 샘플레이트로 열고, Gemini Live가 요구하는 16 kHz,
16-bit, mono, little-endian PCM으로 변환해 40 ms 청크 단위로 넘긴다.
전송이 밀리면 오래된 청크를 버려서 0.4초 이상 쌓아두지 않는다.
"""

import asyncio
import math
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

GEMINI_INPUT_RATE = 16000
GEMINI_OUTPUT_RATE = 24000
CHUNK_MS = 40
MAX_QUEUED_CHUNKS = 10
SILENCE_DBFS = -55.0
_MIN_DBFS = -120.0


class AudioError(RuntimeError):
    """사용자에게 그대로 보여줄 수 있는 오디오 오류."""


def load_sounddevice():
    try:
        import sounddevice
    except OSError as exc:
        raise AudioError(
            'PortAudio 라이브러리를 찾지 못했습니다. '
            '`sudo apt install libportaudio2`로 설치한 뒤 다시 실행하세요.'
        ) from exc
    except ImportError as exc:
        raise AudioError(
            'sounddevice 패키지가 없습니다. '
            '`pip install -r voice/requirements.txt`를 실행하세요.'
        ) from exc
    return sounddevice


@dataclass(frozen=True)
class InputDevice:
    index: int
    name: str
    hostapi: str
    max_input_channels: int
    default_samplerate: float
    is_default: bool


def list_input_devices(sd=None) -> list[InputDevice]:
    sd = sd or load_sounddevice()
    try:
        devices = sd.query_devices()
        hostapis = sd.query_hostapis()
    except sd.PortAudioError as exc:
        raise AudioError(f'오디오 장치 목록을 읽지 못했습니다: {exc}') from exc
    try:
        default_index = sd.query_devices(kind='input')['index']
    except (sd.PortAudioError, ValueError):
        default_index = None
    return [
        InputDevice(
            index=index,
            name=device['name'],
            hostapi=hostapis[device['hostapi']]['name'],
            max_input_channels=device['max_input_channels'],
            default_samplerate=float(device['default_samplerate']),
            is_default=index == default_index,
        )
        for index, device in enumerate(devices)
        if device['max_input_channels'] > 0
    ]


def resolve_input_device(spec: Optional[str], devices: list[InputDevice]) -> InputDevice:
    """--mic 값(번호 또는 이름 일부)으로 장치를 고른다. None이면 OS 기본 입력."""
    if not devices:
        raise AudioError(
            '사용할 수 있는 입력 장치(마이크)가 없습니다. 마이크 연결과 '
            'Ubuntu 설정 > 소리 > 입력을 확인하세요.'
        )
    if spec is None or not spec.strip():
        for device in devices:
            if device.is_default:
                return device
        raise AudioError(
            '운영체제 기본 입력 장치가 없습니다. `--list-mics`로 목록을 보고 '
            '`--mic 번호`로 지정하세요.'
        )

    spec = spec.strip()
    if spec.isdigit():
        index = int(spec)
        for device in devices:
            if device.index == index:
                return device
        raise AudioError(f'{index}번 입력 장치가 없습니다. `--list-mics`로 번호를 확인하세요.')

    wanted = spec.casefold()
    exact = [d for d in devices if d.name.casefold() == wanted]
    if exact:
        return exact[0]
    matches = [d for d in devices if wanted in d.name.casefold()]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise AudioError(f'이름에 "{spec}"가 들어간 입력 장치가 없습니다. `--list-mics`로 확인하세요.')
    candidates = ', '.join(f'{d.index}번 {d.name}' for d in matches)
    raise AudioError(f'"{spec}"와 일치하는 장치가 여러 개입니다({candidates}). 번호로 지정하세요.')


SYSTEM_ROUTED_DEVICES = ('default', 'pipewire', 'pulse')


def pipewire_default_source() -> Optional[str]:
    """PipeWire 기본 입력 장치 이름 (wpctl이 없거나 실패하면 None)."""
    try:
        out = subprocess.run(
            ['wpctl', 'inspect', '@DEFAULT_AUDIO_SOURCE@'],
            capture_output=True, text=True, timeout=2, check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    match = re.search(r'node\.description = "([^"]+)"', out)
    return match.group(1) if match else None


def describe_system_routing(device: InputDevice) -> str:
    """default/pipewire 장치가 실제로 어느 마이크로 연결되는지 설명한다."""
    if device.name not in SYSTEM_ROUTED_DEVICES:
        return ''
    source = pipewire_default_source()
    return f' → PipeWire 기본 입력 "{source}"' if source else ' → 운영체제 기본 입력'


def format_device_table(devices: list[InputDevice]) -> str:
    if not devices:
        return '(입력 장치 없음)'
    lines = ['  번호  이름 (호스트 API, 입력 채널, 기본 샘플레이트)   * = 운영체제 기본 입력']
    for d in devices:
        mark = '*' if d.is_default else ' '
        lines.append(
            f'{mark} {d.index:>4}  {d.name} '
            f'({d.hostapi}, {d.max_input_channels}ch, {d.default_samplerate:.0f} Hz)'
        )
    return '\n'.join(lines)


def to_mono(block: np.ndarray) -> np.ndarray:
    block = np.asarray(block, dtype=np.float32)
    if block.ndim == 1:
        return block
    if block.shape[1] == 1:
        return block[:, 0]
    return block.mean(axis=1, dtype=np.float32)


def float_to_pcm16(samples: np.ndarray) -> bytes:
    clipped = np.clip(samples, -1.0, 1.0)
    return (clipped * 32767.0).astype('<i2').tobytes()


def rms_dbfs(samples: np.ndarray) -> float:
    if samples.size == 0:
        return _MIN_DBFS
    rms = float(np.sqrt(np.mean(np.square(samples, dtype=np.float64))))
    if rms <= 0.0:
        return _MIN_DBFS
    return max(_MIN_DBFS, 20.0 * math.log10(rms))


def level_bar(dbfs: float, width: int = 30) -> str:
    filled = int(round((min(max(dbfs, -60.0), 0.0) + 60.0) / 60.0 * width))
    return '#' * filled + '-' * (width - filled)


class StreamingResampler:
    """청크 경계에서도 연속적으로 리샘플링한다 (soxr)."""

    def __init__(self, in_rate: float, out_rate: int) -> None:
        self.in_rate = in_rate
        self.out_rate = out_rate
        if int(round(in_rate)) == out_rate:
            self._stream = None
        else:
            import soxr

            self._stream = soxr.ResampleStream(in_rate, out_rate, 1, dtype='float32')

    def process(self, mono: np.ndarray) -> np.ndarray:
        mono = np.ascontiguousarray(mono, dtype=np.float32)
        if self._stream is None:
            return mono
        return self._stream.resample_chunk(mono)


@dataclass(frozen=True)
class AudioChunk:
    pcm16: bytes
    level_dbfs: float


class MicrophoneStream:
    """마이크를 열어 16 kHz PCM16 청크를 비동기로 제공한다."""

    def __init__(self, device: InputDevice, chunk_ms: int = CHUNK_MS, sd=None) -> None:
        self.device = device
        self.chunk_ms = chunk_ms
        self.samplerate: Optional[float] = None
        self.channels: Optional[int] = None
        self.dropped_chunks = 0
        self.overflows = 0
        self._sd = sd or load_sounddevice()
        self._stream = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queue: Optional[asyncio.Queue] = None
        self._resampler: Optional[StreamingResampler] = None

    def open(self) -> None:
        sd = self._sd
        self._loop = asyncio.get_running_loop()
        self._queue = asyncio.Queue(maxsize=MAX_QUEUED_CHUNKS)
        rate = self.device.default_samplerate
        blocksize = int(round(rate * self.chunk_ms / 1000))
        channel_options = [1]
        if self.device.max_input_channels >= 2:
            channel_options.append(2)
        last_error: Optional[Exception] = None
        for channels in channel_options:
            try:
                stream = sd.InputStream(
                    device=self.device.index,
                    samplerate=rate,
                    channels=channels,
                    dtype='float32',
                    blocksize=blocksize,
                    callback=self._callback,
                )
                stream.start()
            except sd.PortAudioError as exc:
                last_error = exc
                continue
            self._stream = stream
            self.samplerate = float(stream.samplerate)
            self.channels = channels
            self._resampler = StreamingResampler(self.samplerate, GEMINI_INPUT_RATE)
            return
        raise AudioError(
            f'마이크 {self.device.index}번 "{self.device.name}"을(를) 열 수 없습니다: {last_error}\n'
            '다른 프로그램이 사용 중이거나 장치가 분리되었을 수 있습니다. '
            '`--list-mics`로 다시 확인하세요.'
        )

    def _callback(self, indata, frames, time_info, status) -> None:
        if status.input_overflow:
            self.overflows += 1
        block = indata.copy()
        try:
            self._loop.call_soon_threadsafe(self._push, block)
        except RuntimeError:
            pass  # 이벤트 루프가 이미 닫힘

    def _push(self, block: np.ndarray) -> None:
        if self._queue.full():
            self._queue.get_nowait()
            self.dropped_chunks += 1
        self._queue.put_nowait(block)

    def discard_pending(self) -> None:
        """재연결 직후처럼 오래된 음성을 보내면 안 될 때 대기 중인 청크를 버린다."""
        if self._queue is None:
            return
        while not self._queue.empty():
            self._queue.get_nowait()

    async def read_chunk(self, timeout_s: float = 2.0) -> AudioChunk:
        if self._stream is None:
            raise AudioError('마이크가 열려 있지 않습니다.')
        try:
            block = await asyncio.wait_for(self._queue.get(), timeout_s)
        except asyncio.TimeoutError:
            raise AudioError(
                f'마이크에서 {timeout_s:.0f}초 넘게 소리 데이터가 들어오지 않았습니다. '
                '장치가 분리되었거나 다른 프로그램이 점유했을 수 있습니다.'
            ) from None
        mono = to_mono(block)
        resampled = self._resampler.process(mono)
        return AudioChunk(float_to_pcm16(resampled), rms_dbfs(mono))

    def close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except self._sd.PortAudioError:
                pass
            self._stream = None


class SpeakerOutput:
    """Gemini 음성(24 kHz PCM16 mono)을 별도 오디오 스레드에서 재생한다.

    enqueue()는 버퍼에 복사만 하고 바로 돌아오므로 수신 루프와 상태 변경을
    막지 않는다.
    """

    MAX_BUFFER_SECONDS = 30

    def __init__(self, sd=None) -> None:
        self._sd = sd or load_sounddevice()
        self._stream = None
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._resampler: Optional[StreamingResampler] = None
        self.device_name = ''
        self.samplerate: Optional[float] = None
        self._last_sound = 0.0

    def open(self) -> None:
        sd = self._sd
        try:
            device = sd.query_devices(kind='output')
        except (sd.PortAudioError, ValueError) as exc:
            raise AudioError(f'출력 장치(스피커)가 없습니다: {exc}') from exc
        self.device_name = device['name']
        rates = [GEMINI_OUTPUT_RATE]
        if int(device['default_samplerate']) != GEMINI_OUTPUT_RATE:
            rates.append(float(device['default_samplerate']))
        last_error: Optional[Exception] = None
        for rate in rates:
            try:
                stream = sd.OutputStream(
                    samplerate=rate, channels=1, dtype='int16', callback=self._callback
                )
                stream.start()
            except sd.PortAudioError as exc:
                last_error = exc
                continue
            self._stream = stream
            self.samplerate = float(stream.samplerate)
            if int(round(self.samplerate)) != GEMINI_OUTPUT_RATE:
                self._resampler = StreamingResampler(GEMINI_OUTPUT_RATE, int(round(self.samplerate)))
            return
        raise AudioError(f'출력 장치 "{self.device_name}"을(를) 열 수 없습니다: {last_error}')

    def enqueue(self, pcm16: bytes) -> None:
        if self._stream is None or not pcm16:
            return
        if self._resampler is not None:
            samples = np.frombuffer(pcm16, dtype='<i2').astype(np.float32) / 32768.0
            pcm16 = float_to_pcm16(self._resampler.process(samples))
        limit = int(self.samplerate * self.MAX_BUFFER_SECONDS) * 2
        with self._lock:
            self._buffer.extend(pcm16)
            if len(self._buffer) > limit:
                del self._buffer[: len(self._buffer) - limit]

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()

    def is_playing(self, tail_s: float = 0.0) -> bool:
        """재생할 소리가 남아 있거나, 마지막 소리 뒤 tail_s초가 지나지 않았으면 True."""
        with self._lock:
            if self._buffer:
                return True
        return time.monotonic() - self._last_sound < tail_s

    def _callback(self, outdata, frames, time_info, status) -> None:
        wanted = frames * 2
        with self._lock:
            chunk = bytes(self._buffer[:wanted])
            del self._buffer[:wanted]
        samples = np.frombuffer(chunk, dtype='<i2')
        if samples.size:
            self._last_sound = time.monotonic()
        outdata[: samples.size, 0] = samples
        outdata[samples.size:, 0] = 0

    def close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except self._sd.PortAudioError:
                pass
            self._stream = None


@dataclass(frozen=True)
class LevelSummary:
    max_dbfs: float
    chunks: int

    @property
    def silent(self) -> bool:
        return self.max_dbfs < SILENCE_DBFS


async def show_input_level(mic: MicrophoneStream, seconds: Optional[float]) -> LevelSummary:
    """입력 음량을 한 줄 막대로 표시한다. seconds가 None이면 취소될 때까지."""
    import sys

    loop = asyncio.get_running_loop()
    deadline = None if seconds is None else loop.time() + seconds
    max_db = _MIN_DBFS
    chunks = 0
    try:
        while deadline is None or loop.time() < deadline:
            chunk = await mic.read_chunk()
            chunks += 1
            max_db = max(max_db, chunk.level_dbfs)
            sys.stdout.write(
                f'\r  입력 음량 {chunk.level_dbfs:6.1f} dBFS [{level_bar(chunk.level_dbfs)}] '
                f'최대 {max_db:6.1f}'
            )
            sys.stdout.flush()
    finally:
        sys.stdout.write('\n')
        sys.stdout.flush()
    return LevelSummary(max_db, chunks)
