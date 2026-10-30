# voice — 쿠옹이: 말로 로봇 경로 고치기 (ChatKHU)

한국어로 말하면 ChatKHU(Gemini Live)의 쿠옹이가 알아듣고 함수를 부르고, `--sim`이면 그 함수가 MuJoCo 시뮬레이터(`mujoco_sim`, `--voice-gateway`)를 실제로 바꾼다. 시뮬레이터가 돌려준 결과와 알림(검증 결과, 이동 시작, 도착 질문)을 쿠옹이가 말한다. 설치·실행 전체 순서는 [../docs/PINCH_KHU_SETUP.md](../docs/PINCH_KHU_SETUP.md).

## 준비

```bash
cd 26_project_manipulator          # 받은 저장소 폴더 (./setup.sh가 아래 두 줄을 대신한다)
python3 -m venv .venv-voice && .venv-voice/bin/pip install -r voice/requirements.lock
source voice/env.sh      # ROS PYTHONPATH 제거, .venv-voice 켜기, 키 읽기: ~/.config/pinch_khu/env → voice/chatkhu_key.env (키는 출력하지 않음)
```

마이크·스피커에는 시스템 `libportaudio2`가 필요하다.

## 실행

```bash
python -m voice --sim             # 시뮬레이터에 연결, 마이크. "쿠옹아"라고 부른 뒤 말한다. Ctrl+C로 끝
python -m voice --sim --text      # 키보드로 문장 입력
python -m voice.sim_dryrun        # AI·키 없이 함수를 직접 불러 시뮬레이터 흐름 시험
python -m voice --list-mics       # 마이크 목록 → --mic 번호
python -m voice --text            # 시뮬레이터 없이 가짜 처리기 (지시문 system_v1)
python -m pytest -q voice/tests   # 호출어·질문 창, 알림 문장 시험
```

`--sim`이면 지시문은 `prompts/system_v8.md`다.

## 규칙

- **호출어 창:** "쿠옹아"(표기 변형·비슷한 소리 포함, 문장 맨 앞)를 들으면 `wake_seconds` 동안 호출어 없이 말해도 된다. 이 시간은 명령이 실제로 실행되거나 손으로 경로를 고칠 때만 다시 채운다. 지금 값은 0 = 한 번 부르면 쿠옹이를 끌 때까지 계속 연다(2026-10-28, 기본 30초에서 바꿈). "야" 한 마디로 부르기(`call_words`)는 기본으로 껐다.
- **질문 창:** 앱(시뮬레이터)이 묻는 질문 — "수정 완료" 뒤 "검증 후 이동할까요?", "계속 갈까요, 돌아갈까요?", 도착 후 "돌아오는 길을 수정할까요, …?", "새 목표로 다시 해 볼까요?" — 의 답 **한 번**은 호출어 없이 받는다. 그 질문의 답이 될 함수만, 20초(`answer_seconds`) 안에, 시뮬레이터 상태(rev)가 그대로일 때만 실행한다. 부정·엉뚱한 말로 다른 함수가 불리면 `blocked/not_an_answer`.
- **멈춤**("멈춰", "정지", "스톱")은 언제나: 말이 들리자마자 AI 응답 전에 멈추고(빠른 정지), 시뮬레이터도 늦게 도착한 정지 요청을 거절하지 않는다. 다만 기본 설정에서는 쿠옹이가 말하는 동안 마이크가 무음이라(`mute_mic_while_speaking`) 그동안의 "멈춰"는 들리지 않는다 → **X 키**가 확실한 정지 수단이다. 움직이는 동안 마이크를 여는 `listen_while_moving`은 실제 마이크·스피커로 에코 시험(재현 문서 8절 시험 G)을 통과한 뒤에만 켠다.
- **손 입력 거절**(수정 모드 아님, 손가락 깊이 없음, 깊이 끊김, 정합 미검증)은 말하지 않는다. 시뮬레이터 화면의 알림 줄과 손가락 옆 글자로만 보인다(2026-10-28 사용자 요청: 핀치할 때마다 음성 안내가 나와 시끄러움).
- 설정은 `config.yaml`: 호출어와 표기 변형, 모델, 목소리, 말투, 조금·보통·많이 cm(2·5·8), 위 시간들.
- 대화 기록은 `voice/logs/*.jsonl`(git 제외, 사용자 말이 들어 있음).

## 함수 (`tools.py`, 15개)

| 함수 | 시뮬레이터에서 하는 일 |
|---|---|
| `start_edit` | 수정 모드를 실제로 켠다(검증 중이면 취소, 도착 뒤면 돌아오는 길 수정). 켜진 결과가 와야 안내 문장(`say`: "경로 수정 후 수정 완료를 말해 주세요.")을 준다 |
| `move_here`, `move_region`, `adjust_amount` | 가리킨 점 / 시작·가운데·끝을 방향·거리만큼 옮김, "조금만 더 / 좀 덜" |
| `undo_edit`, `reset_edit` | 직전 수정 되돌리기(손·마우스·말 수정 공통 기록), 처음 경로 |
| `finish_edit` | "수정 완료": 움직이지 않고 앱이 "검증 후 이동할까요?"라고 묻는다(답은 호출어 없이 한 번). 실패한 그 경로·이동 중이면 묻지 않고 이유 |
| `validate_and_move` | 그 질문에 "어/응/네": 검증하고 통과하면 그 경로로 이동(자동 이동 예약). 결정한 뒤 경로가 바뀌었으면 `path_changed` |
| `stop_motion`, `continue_motion`, `go_back` | 멈춤, 멈춘 뒤 계속 / 돌아가기 |
| `edit_return_path`, `new_target`, `save_paths` | 돌아오는 길 수정, 새 목표, 경로 저장 |
| `set_target_from_camera` | 카메라가 본 책상 위 물체를 목표로 새 경로 (깊이 카메라 + 정합 확인 필요) |

## 파일

| 파일 | 내용 |
|---|---|
| `session.py` | ChatKHU 연결·송수신·함수 실행 판정·알림 전달·재연결 |
| `wake_word.py` | 호출어 감지, 호출어 창·질문 창, `gate_decision`(실행할지) |
| `sim_link.py` | 시뮬레이터 소켓 연결(한 번에 한 클라이언트), 함수 → 명령, 알림 문장 |
| `sim_dryrun.py` | AI 없이 함수 호출 시험 |
| `tools.py` | 함수 선언, 가짜 처리기(시뮬레이터 없이) |
| `prompts/system_v8.md` | 시뮬레이터용 지시문 (v1~v7은 이전 기록; v7 = 수정 완료 → 검증 후 이동할까요? → 어; v8 = v7 + 시연 지점 S 설거지 바구니 · B 국그릇 · C 밥그릇, go_to_station) |
| `gateway.py` | 1회용 토큰 + WebSocket 연결 |
| `audio_io.py`, `console.py`, `turn_log.py` | 마이크·스피커, 출력(키 가림), 대화 기록 |

예전 ROS 음성 모듈(`work/integration-all`의 `manipulator_voice`, Gemini Live 직접 연결)과는 다른 프로그램이다. 같은 호출어·같은 시뮬레이터 소켓을 쓰므로 함께 켜지 않는다(쿠옹이는 그 프로세스가 보이면 시작하지 않고, 시뮬레이터는 두 번째 음성 연결을 거부한다).
