# voice — 음성으로 로봇 경로 수정 명령 받기 (ChatKHU)

한국어로 말하면 ChatKHU(Gemini Live)가 알아듣고 경로 수정 함수를 부른다. 지금은 경로를 실제로 고치지 않는다. 함수 이름과 인자를 화면에 찍고 가짜 결과를 돌려주면, AI가 그 결과를 한 문장으로 말한다.

## 준비

```bash
pip install -r voice/requirements.txt   # websockets, pyyaml, numpy, sounddevice, soxr
export BAZE_API_KEY=...                 # 키는 환경변수에서만 읽는다
```

이 PC에서는 `~/venv_khuai`에 다 설치되어 있다 (`source ~/venv_khuai/bin/activate`).

## 실행 (`code/` 폴더에서)

```bash
python3 -m voice --text           # 키보드로 입력. /fail 로 실패 모드 켜고 끄기, q 로 끝
python3 -m voice                  # 마이크. "쿠옹아"라고 부른 뒤 말한다. Ctrl+C로 끝
python3 -m voice --fail           # 처음부터 실패 모드 ({"status":"failed","reason":"unreachable"})
python3 -m voice --list-mics      # 마이크 목록 → --mic 번호
python3 -m voice --prompt voice/prompts/system_v2.md   # 지시문 바꿔 비교
```

- 호출어를 들으면 마지막 대화 뒤 30초 동안은 호출어 없이 말해도 된다. 호출어 없이 온 함수 호출은 `blocked`로 막는다.
- 설정은 `voice/config.yaml`에서 바꾼다: 호출어, 모델, 목소리, 말투, 조금·보통·많이의 cm 값(2·5·8), 실패 모드.
- 대화 기록은 `voice/logs/*.jsonl`에 남는다 (사용자 말, 함수와 인자, 돌려준 결과, AI 말, 지시문 버전).

## 파일

| 파일 | 내용 |
|---|---|
| `gateway.py` | 1회용 토큰 발급 + WebSocket 연결 (User-Agent `curl/8.5.0`) |
| `session.py` | setup·송수신·함수 처리·재연결(새 토큰 + 세션 재개 핸들) |
| `tools.py` | 함수 선언(move_here, move_region, undo_edit, reset_edit, finish_edit)과 가짜 처리기 |
| `prompts/system_v1.md` | AI 지시문. `{wake_word}`, `{style}`은 설정값으로 바뀐다 |
| `audio_io.py`, `wake_word.py`, `console.py` | `manipulator_voice`에서 복사해 고침 |

## 게이트웨이 시험 결과 (2026-10-08)

- 마이크 음성(realtimeInput audio, 16 kHz PCM): 통과
- 입력 음성 자막(inputAudioTranscription, languageCodes 포함): 통과. 다만 "자비스"가 "재비스", "서비스", "다비스", "나비드" 등으로 적힌다. 첫 단어가 첫 글자만 다르면 호출어로 보도록 했지만(`wake_fuzzy`) 아직 놓칠 때가 있다.
- 세션 재개(sessionResumption): 통과. 끊긴 뒤 새 토큰 + 핸들로 붙으면 앞 대화를 이어 간다.
- speechConfig(목소리), contextWindowCompression: setup에서 받아들여짐.
