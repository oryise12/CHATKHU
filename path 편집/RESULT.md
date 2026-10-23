# 구현 및 검증 결과

검증일: 2026-09-29  
작업 위치: `/home/jc/manipulator_branches`

브랜치 전환·병합·커밋·푸시는 하지 않았고 `urdf/`도 변경하지 않았다. 세 worktree의 `.git` 파일이 존재하지 않는 `/home/jc/26_project_manipulator_work/.git/worktrees/...`를 가리켜 `git status`는 실행할 수 없었다.

## 변경 파일

- 경로 편집기 기존 파일: `core.py`, `editor.py`, `validation.py`, `storage.py`, `__main__.py`, `__init__.py`, `pyproject.toml`, `configs/demo.json`
- 신규 구현 파일: `webcam.py`, `ros_bridge.py`
- 경로 테스트: `test_core.py`, `test_editor.py`, `test_storage_validation.py`, 신규 `test_webcam.py`
- 음성 기존 파일: `commands.py`, `edit_state.py`, `ros_client.py`, `guidance.py`, `gemini_live.py`, `state_node.py`, `voice_commander.py`, `live_eval.py`, `command_log.py` 및 관련 테스트
- ROS 인터페이스: `EditCommand.srv`, `EditState.msg`
- 문서: 경로 `README.md`, `DESIGN.md`, `RESULT.md`; 음성 `README.md`, `docs/CURRENT_STATE.md`

실제 로봇·MuJoCo 제어 코드는 추가하지 않았다.

## 수정 전후 재현 수치

수정 전 수치는 작업 전 소스의 읽기 전용 사본
`/home/jc/.local/share/Trash/files/26_project_manipulator_work/path 편집/src`를 `PYTHONPATH` 맨 앞에 두고 동일한 Editor API 재현을 다시 실행했다. 출력된 `loaded_core` 경로로 수정 전/후 소스를 구분했다. 아래 값은 추정값이 아니다.

### 1. 중앙 +8 cm 후 같은 위치 +0.001 mm

비교 기준은 첫 드래그 완료 직후 경로와 두 번째 드래그 preview의 모든 대응점이다.

| 구현 | 두 번째 입력 전후 경로 전체 최대 변화량 | 시작/갱신/완료 수락 | 완료 뒤 변화량 |
|---|---:|---|---:|
| 수정 전 | `0.0063193222049946762 m` | `True / True / True` | `0.0063193222049946762 m` |
| 수정 후 | `0.000001000000000001 m` | `True / True / False` | `0 m` |

수정 전 실제 출력:

```text
loaded_core= /home/jc/.local/share/Trash/files/26_project_manipulator_work/path 편집/src/pac_path_editor/core.py
BEFORE FIX CASE 1
first_drag(start,update,accept)= (True, True, True)
second_drag(start,update,accept)=(True,True,True)
max_change_before_to_preview_m=0.0063193222049946762
max_change_before_to_after_end_m=0.0063193222049946762
message= Gesture kept (0.0063 m maximum movement). Finish editing to validate.
```

수정 후 실제 출력:

```text
CASE 1: xi=0.5 +0.08 m, then +0.000001 m
first_drag(start,update,accept)= (True, True, True)
second_drag(start,update,accept)=(True,True,False)
max_change_before_to_preview_m=1.000000000001e-06
max_change_before_to_after_end_m=0
message= Drag rejected: effective movement 0.000001 m is below 0.001000 m.
```

수정 후 preview 자체가 입력과 같은 약 1 µm만 변한다. 완료 시 기본 최소 실효 변화 `0.001 m`보다 작아 거부되고 첫 수정 상태로 돌아간다. deadband로 큰 변화를 숨긴 결과가 아니다.

### 2. xi=0.2에서 +8 cm 후 시작점에 +1 mm

`기존 수정 위치 변화량`은 두 번째 입력 전후의 outbound `xi=0.2` 위치 거리다. 약 `0.074493 m`는 이 한 점이 아니라 경로 전체 최대 변화량이다.

| 구현 | 기존 수정 위치 `xi=0.2` 변화량 | 경로 전체 최대 변화량 | 시작/갱신/완료 수락 |
|---|---:|---:|---|
| 수정 전 | `0.072435534998738615 m` | `0.074493255742099154 m` | `True / True / True` |
| 수정 후 | `0 m` | `0 m` | `False / False / False` |

수정 전 실제 출력:

```text
BEFORE FIX CASE 2
first_drag(start,update,accept)= (True, True, True)
endpoint_drag(start,update,accept)=(True,True,True)
preview_change_at_existing_xi_0.2_m=0.072435534998738615
preview_whole_path_max_change_m=0.074493255742099154
after_end_change_at_existing_xi_0.2_m=0.072435534998738615
message= Gesture kept (0.0745 m maximum movement). Finish editing to validate.
```

수정 후 실제 출력:

```text
CASE 2: xi=0.2 +0.08 m, then xi=0 +0.001 m
first_drag(start,update,accept)= (True, True, True)
endpoint_drag(start,update,accept)=(False,False,False)
change_at_existing_xi_0.2_m=0
whole_path_max_change_m=0
message= Exact endpoints are fixed; select a nearby interior point.
```

### 3. 경로 길이 0.600 m에서 약 0.707 m로 증가

길이 정책만 분리하기 위해 `max_ee_speed=100 m/s`로 두어 속도 제한이 시간을 결정하지 않게 했다. 원본 기준은 `L0=0.600 m`, `T0=3.000 s`, `sample_dt=0.030 s`다. 긴 경로 길이는 `0.707106781186548 m`다.

| 구현 | 길이 `0.600 m` | 길이 `0.707106781186548 m` | 같은 경로 재계산 |
|---|---:|---:|---:|
| 수정 전 | `3.00 s` | `3.00 s` | 정책 없음 |
| 수정 후 | `3.00 s` | `3.54 s` | `3.54 s` |

수정 전 실제 출력:

```text
BEFORE FIX CASE 3 (speed limit isolated: max_ee_speed=100)
before: length_m=0.6, recalculated_s=3
after: length_m=0.707106781186548, recalculated_s=3
```

수정 후 실제 출력:

```text
loaded_core= /home/jc/manipulator_branches/main/path 편집/src/pac_path_editor/core.py
before: length_m=0.6, T_length_s=3, recalculated_s=3
after: length_m=0.707106781186548, T_length_s=3.53553390593274, recalculated_s=3.54
repeated_retime_s=3.54
```

`3 × 0.707106781186548 / 0.6 = 3.53553390593274 s`를 `sample_dt` 배수로 올려 `3.54 s`가 된다. 기본 속도 제한 `0.35 m/s`까지 적용한 별도 실행 출력은 `0.600 m -> 3.27 s`, `0.707106781186548 m -> 3.84 s`, 반복 재계산 `3.84 s`였다.

### 재현 실행 명령

수정 후 수치는 다음과 같이 가상환경에서 `Editor`, `max_corresponding_displacement`, `interpolate_at_xi`, `retime_path`를 직접 호출해 출력했다.

```bash
cd ~/manipulator_branches/main/'path 편집'
source ../../.venv-webcam/bin/activate
python - <<'PY'
# Config() 직선 경로에서 위 표의 두 drag를 순서대로 실행하고,
# 전후 max_corresponding_displacement 및 xi=0.2 위치 거리를 출력했다.
# 실행 시간은 PathData 길이 0.6/0.707106781186548,
# max_ee_speed=100, L0=0.6, T0=3.0으로 retime_path를 호출했다.
PY
```

수정 전에는 동일 계산을 다음처럼 읽기 전용 사본에서 실행했다.

```bash
cd ~/manipulator_branches
source .venv-webcam/bin/activate
PYTHONPATH='/home/jc/.local/share/Trash/files/26_project_manipulator_work/path 편집/src' python - <<'PY'
# 동일한 두 Editor drag 재현과 retime_path 계산
PY
```

## 전체 테스트와 빌드

작업 시작 전에 기존 경로 테스트를 실행해 `96 passed`를 확인했다. 신규 회귀 및 웹캠 테스트를 포함한 현재 전체 경로 테스트:

```bash
cd ~/manipulator_branches/main/'path 편집'
source ../../.venv-webcam/bin/activate
python -m pytest -q
```

```text
........................................................................ [ 65%]
......................................                                   [100%]
110 passed in 4.38s
```

ROS 2 Jazzy 환경의 전체 음성 테스트:

```bash
cd ~/manipulator_branches/voice
source /opt/ros/jazzy/setup.bash
source ../.venv-webcam/bin/activate
source install/setup.bash
python -m pytest -q src/manipulator_voice/test
```

```text
........................................................................ [ 54%]
............................................................             [100%]
132 passed in 3.49s
```

현재 전체 자동 테스트 합계는 **242 passed, 0 failed**다. 작업 전 96개는 현재 110개에 포함되는 기준 테스트이므로 합계에 다시 더하지 않았다.

ROS 빌드:

```bash
cd ~/manipulator_branches/voice
source /opt/ros/jazzy/setup.bash
source ../.venv-webcam/bin/activate
python -m colcon build --symlink-install --packages-select manipulator_interfaces manipulator_voice
```

```text
Starting >>> manipulator_interfaces
Finished <<< manipulator_interfaces [0.42s]
Starting >>> manipulator_voice
Finished <<< manipulator_voice [1.18s]

Summary: 2 packages finished [1.71s]
```

의존성 검사:

```bash
cd ~/manipulator_branches
source .venv-webcam/bin/activate
python -m pip check
```

```text
No broken requirements found.
```

## 합성 입력·렌더링·저장

```bash
cd ~/manipulator_branches/main/'path 편집'
source ../../.venv-webcam/bin/activate
python -m pac_path_editor --webcam --synthetic --no-voice \
  --screenshot /tmp/pac-synth-result.png --save /tmp/pac-session-result.json
```

```text
Synthetic XYZ pinch + both + render + save: PASS; revision=2; no camera/microphone used
schema_version=2 frame=virtual_edit_frame records=2 report_passed=True
```

기존 matplotlib 실행:

```bash
python -m pac_path_editor --config configs/demo.json --demo bump --headless \
  --screenshot /tmp/pac-matplotlib-result.png
```

```text
Headless camera view rendered: /tmp/pac-matplotlib-result.png
```

## 웹캠·MediaPipe·마이크·ROS 실제 검증 구분

| 항목 | 실제 장치/런타임 | 합성 검사 | 결론 |
|---|---|---|---|
| 웹캠 | 실제 `/dev/video0`에서 640×480 frame 1장을 읽음 | 합성 배경과 hand landmark로 전체 편집 흐름 검사 | 장치 frame 읽기는 확인. 사람이 핀치해 편집하는 조작은 미확인 |
| MediaPipe | 실제 웹캠 frame을 `solutions.hands`로 처리 | landmark 4/8, XYZ, pinch를 합성 입력으로 검사 | 런타임 처리는 확인. 실제 frame은 `hand_detected=False`여서 실제 손 추적/핀치는 미확인 |
| 마이크/Gemini/스피커 | **미확인**. 실제 마이크 녹음, Gemini 접속, 음성 인식·재생을 실행하지 않음 | fake session, 합성 tool call, 단위 테스트만 통과 | 실제 음성 문구 인식은 미확인 |
| ROS 2 | 설치된 Jazzy에서 실제 service/topic/executor/queue/Editor round trip 실행 | 명령 내용은 합성 Gemini tool call 사용 | ROS transport와 Editor 반영은 확인. 로봇 장치는 연결하지 않음 |
| OpenCV 화면 | 세 mode screenshot을 육안 확인 | 합성 배경으로 렌더링 | mode 표시/강조는 확인. 실제 카메라 창의 장시간 조작은 미확인 |

실제 웹캠과 MediaPipe probe의 출력은 다음과 같다. MediaPipe 초기화 정보 로그는 생략했다.

```text
camera=/dev/video0 opened=True read=True shape=(480, 640, 3)
mediapipe_api=solutions.hands processed=True hand_detected=False
```

## 가는 길/오는 길/둘 다 명령의 화면 반영

실제 마이크나 Gemini API를 사용한 결과가 아니다. `convert_function_call`로 해당 문구에 대응하는 합성 Gemini tool call을 만들고 실제 ROS 2 `EditCommand` service로 `RosBridge`에 전송했다. 응답 직후 OpenCV renderer로 screenshot을 만들고 상태 행과 대상 경로 명암을 육안 확인했다.

실행 환경:

```bash
cd ~/manipulator_branches/main/'path 편집'
source /opt/ros/jazzy/setup.bash
source ../../.venv-webcam/bin/activate
source ../../voice/install/setup.bash
# Python probe에서 convert_function_call -> rclpy EditCommand client
# -> RosBridge queue -> owner-thread drain -> Editor -> render 순서로 실행
```

실제 출력:

```text
phrase=가는 길 수정 synthetic_tool_mode=outbound ros_accepted=True response_mode=outbound editor_mode=outbound state=EDITING rendered_status="EDITING  mode=outbound" screenshot=/tmp/mode-outbound.png
phrase=오는 길 수정 synthetic_tool_mode=return ros_accepted=True response_mode=return editor_mode=return state=EDITING rendered_status="EDITING  mode=return" screenshot=/tmp/mode-return.png
phrase=둘 다 수정 synthetic_tool_mode=both ros_accepted=True response_mode=both editor_mode=both state=EDITING rendered_status="EDITING  mode=both" screenshot=/tmp/mode-both.png
```

- `outbound`: 가는 길 선명, 오는 길 흐림
- `return`: 오는 길 선명, 가는 길 흐림
- `both`: 두 경로 모두 선명

mode 인자 → ROS 요청 → 실제 Editor mode → 화면 렌더링 연결은 세 값 모두 확인했다. 실제 음성을 마이크와 Gemini가 올바른 tool call로 바꾸는 구간은 미확인이다.

## 구현 결과

- `Correction.profile_residual`로 기존 벡터 field를 profile knot마다 보존해 작은 재수정이 이전 곡선을 재구성하지 않는다.
- exact endpoint는 Editor와 core 직접 호출 모두 memory 변경 전에 거부한다. endpoint 근처 보호 가중치는 유지한다.
- schema version 2에 timing reference, residual, frame을 저장한다. schema 1은 명시적 migration만 허용한다.
- 실제 웹캠 배경, 왕복 실선/점선, 방향 화살표, mode alpha, 고정 Z colormap, drag 비교선과 요청/적용 표시를 구현했다.
- landmark 4·8 중점, 상대 XYZ, 축별 gain/sign, pinch hysteresis/debounce/filter, release-to-arm, 추적 상실 동결/timeout 취소를 구현했다.
- keyboard와 음성이 같은 `EditorCommands`를 사용한다. ROS callback은 queue에 넣고 OpenCV owner thread만 Editor를 수정한다.
- deadline 뒤 미적용과 request ID 중복 방지를 구현했다. 실행 요청은 항상 거부한다.

## 남은 오류와 미확인 사항

- 자동 테스트 실패는 없다.
- worktree `.git` 포인터 대상이 없어 `git status`가 실패한다.
- 실제 webcam frame에서 손이 검출되지 않았다. 조명, 손 크기, pinch threshold, XYZ gain의 실제 사용성은 미확인이다.
- 실제 마이크, 호출어, Gemini Live API, 스피커 동작은 미확인이다.
- 실제 OpenCV 카메라 창에서 장시간 상호작용과 tracking loss 복구 사용성은 미확인이다.
- 실제 로봇, depth 카메라, 캘리브레이션, IK, 관절 한계, 특이점, 충돌, 동역학, 토크 검사는 미확인이다. 화면에도 `NOT_EVALUATED`로 표시된다.
- ROS 실행 때 `ROS_LOCALHOST_ONLY is deprecated` 경고가 나왔지만 service round trip은 성공했다.

## 사용자가 직접 확인할 조작 순서

웹캠만 실행:

```bash
cd ~/manipulator_branches/main/'path 편집'
source ../../.venv-webcam/bin/activate
python -m pac_path_editor --webcam --camera 0 --no-voice
```

1. 실제 영상과 `virtual_edit_frame`, `uncalibrated overlay`, `execution disabled` 표시를 확인한다.
2. `E` 또는 `2`로 outbound 편집을 시작한다. 손을 보인 뒤 엄지와 검지를 먼저 충분히 벌린다.
3. 두 손끝 중간 커서를 경로 위에 두고 새로 핀치한다. 시작 순간 경로가 튀지 않는지 확인한다.
4. 손을 오른쪽/위쪽으로 움직여 X/Y를 확인한다. 손끝 자세를 앞뒤로 바꾸고 `landmark XYZ`, Z 수치와 색상 변화를 확인한다. Z는 실제 거리 m가 아닌 상대 가상 입력이다.
5. 손가락을 놓아 확정한다. `U` undo, `Y` redo, `R` reset, `C` 전체 취소, `F` 검사/저장을 확인한다.
6. `1` both, `2` outbound, `3` return으로 mode와 경로 명암을 확인한다.
7. drag 중 손을 짧게 가렸다가, timeout보다 길게 가려 preview 동결과 취소를 확인한다.

ROS 통합 실행:

```bash
cd ~/manipulator_branches/voice
source /opt/ros/jazzy/setup.bash
source ../.venv-webcam/bin/activate
python -m colcon build --symlink-install --packages-select manipulator_interfaces manipulator_voice
source install/setup.bash
cd "../main/path 편집"
python -m pac_path_editor --webcam --camera 0 --ros
```

두 번째 터미널의 음성 실행:

```bash
cd ~/manipulator_branches
source /opt/ros/jazzy/setup.bash
source .venv-webcam/bin/activate
source voice/install/setup.bash
ros2 run manipulator_voice voice_commander --env-file voice/.env --no-speaker
```

1. 호출어 뒤 “가는 길 수정”, “오는 길 수정”, “둘 다 수정”을 말해 화면의 `mode=`와 경로 명암을 확인한다.
2. 핀치 drag 도중 mode 변경을 말해 거부 이유가 안내되는지 확인한다.
3. gesture 후 “되돌려”, “다시 적용”, “처음으로”, “수정 취소”를 말해 undo, redo, reset, 전체 취소가 구분되는지 확인한다.
4. 완료 후 JSON의 `schema_version=2`, `frame=virtual_edit_frame`, validation 상태를 확인한다.

통합 모드에서는 `edit_state_node`를 동시에 실행하지 않는다.
