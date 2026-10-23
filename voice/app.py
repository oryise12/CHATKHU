"""실행 진입점: python3 -m voice [--text] [--fail] ..."""

import argparse
import asyncio
import sys
import threading
from pathlib import Path

from voice import console, gateway
from voice.config import DEFAULT_CONFIG, ConfigError, load_config
from voice.session import LiveAssistant, LiveError
from voice.tools import FakeEditor
from voice.turn_log import TurnLogger


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog='python3 -m voice', description='음성으로 로봇 경로 수정 명령 받기 (ChatKHU)')
    p.add_argument('--text', action='store_true', help='마이크 대신 키보드로 문장을 입력한다')
    p.add_argument('--sim', nargs='?', const='', metavar='SOCKET',
                   help='가짜 처리 대신 MuJoCo 시뮬레이터(pinch_sim --voice-gateway)에 명령을 보낸다')
    p.add_argument('--fail', action='store_true', help='수정 함수가 실패 결과를 돌려준다 (텍스트 모드에서는 /fail 로 켜고 끈다)')
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG, help='설정 파일 (기본 voice/config.yaml)')
    p.add_argument('--prompt', type=Path, help='지시문 파일 (설정의 prompt 대신)')
    p.add_argument('--mic', help='마이크 번호 또는 이름 일부 (설정의 mic 대신)')
    p.add_argument('--list-mics', action='store_true', help='마이크 목록을 보고 끝낸다')
    p.add_argument('--no-speaker', action='store_true', help='마이크 모드에서 AI 음성을 재생하지 않는다')
    p.add_argument('--speak', action='store_true', help='텍스트 모드에서도 AI 음성을 재생한다')
    return p.parse_args(argv)


def start_stdin_reader(loop: asyncio.AbstractEventLoop, queue: asyncio.Queue) -> None:
    def read() -> None:
        for line in sys.stdin:
            loop.call_soon_threadsafe(queue.put_nowait, line)
        loop.call_soon_threadsafe(queue.put_nowait, None)

    threading.Thread(target=read, daemon=True).start()


async def run(args: argparse.Namespace) -> int:
    from voice import audio_io

    cfg = load_config(args.config)
    if args.prompt:
        cfg.prompt_path = args.prompt.resolve()
    elif args.sim is not None:  # 시뮬레이터용 지시문 (클릭한 점, 수정 완료 = 검증 후 자동 이동)
        cfg.prompt_path = (Path(__file__).parent / 'prompts' / 'system_v5.md').resolve()
    if args.mic:
        cfg.mic = args.mic
    gateway.api_key()  # 키가 없으면 여기서 멈춘다 (값은 출력하지 않는다)

    if args.sim is not None:
        from voice.sim_link import SimEditor, SimLinkError
        try:
            editor = SimEditor(cfg.amount_cm, args.sim or None)
        except SimLinkError as exc:
            console.error(str(exc))
            return 2
        console.info(f'시뮬레이터 연결됨 (rev {editor.rev}). 시뮬레이터 화면에서 고칠 점을 클릭한 뒤 말하세요.')
    else:
        editor = FakeEditor(cfg.amount_cm, cfg.fail_mode or args.fail, cfg.fail_reason)
    mode = 'text' if args.text else 'mic'
    logger = TurnLogger(cfg.log_dir, prompt_version=cfg.prompt_version, input_mode=mode, model=cfg.model)

    mic = speaker = lines = None
    try:
        if (not args.text and not args.no_speaker) or (args.text and args.speak):
            speaker = audio_io.SpeakerOutput()
            try:
                speaker.open()
            except audio_io.AudioError as exc:
                console.warn(f'스피커 없이 진행합니다: {exc}')
                speaker = None
        if args.text:
            lines = asyncio.Queue()
            start_stdin_reader(asyncio.get_running_loop(), lines)
        else:
            device = audio_io.resolve_input_device(cfg.mic or None, audio_io.list_input_devices())
            mic = audio_io.MicrophoneStream(device)
            mic.open()
            console.info(f'마이크: {device.index}번 {device.name}{audio_io.describe_system_routing(device)}'
                         f' ({mic.samplerate:.0f} Hz → 16 kHz)')

        console.info(f'지시문 {cfg.prompt_version}, 호출어 "{cfg.wake_word}", '
                     f'fail 모드 {"켜짐" if editor.fail_mode else "꺼짐"}, 기록 {logger.path}')
        if args.text:
            console.info(f'문장을 입력하세요. 예: "{cfg.wake_word}, 가운데를 위로 조금 올려줘". /fail 로 실패 모드 전환, q 로 끝.')
        else:
            console.info(f'"{cfg.wake_word}"라고 부른 뒤 말하세요. 끝내려면 Ctrl+C.')

        assistant = LiveAssistant(cfg, editor, logger, mic=mic, speaker=speaker, lines=lines,
                                  echo_input=args.text and not sys.stdin.isatty())
        await assistant.run()
    finally:
        if mic is not None:
            mic.close()
        if speaker is not None:
            await asyncio.sleep(0)  # 남은 소리는 버린다
            speaker.close()
    console.info(f'기록: {logger.path}')
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list_mics:
        from voice import audio_io
        try:
            print(audio_io.format_device_table(audio_io.list_input_devices()))
        except audio_io.AudioError as exc:
            console.error(str(exc))
            return 1
        return 0
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        console.raw()
        console.info('종료합니다.')
        return 0
    except (ConfigError, gateway.GatewayError, LiveError) as exc:
        console.error(str(exc))
        return 1
    except Exception as exc:
        from voice.audio_io import AudioError
        if isinstance(exc, AudioError):
            console.error(str(exc))
            return 1
        raise
