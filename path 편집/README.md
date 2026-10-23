# 웹캠 기반 가상 3D 왕복 경로 편집 시연

> 프로젝트 전체 상태, 새 clone에서 세 worktree를 만드는 방법, Ubuntu 24.04 설치와 음성·ROS 통합 순서는 [저장소 루트 README](../README.md)를 먼저 본다. 아래 명령의 `~/manipulator_branches/main`, `voice`, `urdf`는 현재 개발 PC의 **서로 다른 Git worktree**이며 한 checkout 안의 하위 폴더가 아니다.

일반 웹캠 영상 위에 outbound(가는 길, 실선)와 return(오는 길, 점선)을 겹쳐 표시하고, MediaPipe 한 손의 핀치 드래그로 XYZ 경로를 수정한다. 이 화면은 카메라-로봇 캘리브레이션이 없는 `virtual_edit_frame` 시연이다. landmark Z는 상대 제어 입력이며 실제 카메라 거리나 절대 손 위치가 아니다. 실제 로봇을 움직이거나 로봇 안전성을 검증하지 않는다.

기존 matplotlib 마우스 편집기는 그대로 사용할 수 있다. 계산, 제한, undo/redo, 검사와 저장은 두 UI가 같은 `core.py`/`editor.py`를 사용한다.

## 설치

확인 환경은 Ubuntu 24.04, Python 3.12.3, ROS 2 Jazzy, NumPy 1.26.4, MediaPipe 0.10.21, OpenCV 4.11.0.86이다. ROS의 Python 패키지를 같이 쓰므로 저장소 루트에서 별도 가상환경을 만든다.

```bash
cd ~/manipulator_branches
python3 -m venv --system-site-packages .venv-webcam
source .venv-webcam/bin/activate
python -m pip install -e "./main/path 편집[webcam,dev]" -r voice/requirements-voice.txt
python -m pip check
```

`opencv-python`과 `opencv-python-headless`를 추가 설치하지 않는다. 이 구성은 `opencv-contrib-python` 하나만 사용한다. 현재 MediaPipe 0.10.21에는 `mp.solutions.hands`가 있어 내장 모델을 쓴다. 향후 설치본에 Solutions API가 없으면 Tasks API와 모델 파일을 사용한다.

```bash
cd ~/manipulator_branches/main/'path 편집'
mkdir -p models
curl -L https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task \
  -o models/hand_landmarker.task
```

이때 `WebcamConfig(hand_api="tasks", model_path="models/hand_landmarker.task")`와 같은 항목을 JSON으로 저장하고 `--webcam-config`에 넘길 수 있다. 허용 설정과 기본값은 `webcam.py`의 `WebcamConfig`에 있다.

## 실행

음성/ROS 없이 웹캠만 실행:

```bash
cd ~/manipulator_branches/main/'path 편집'
source ../../.venv-webcam/bin/activate
python -m pac_path_editor --webcam --camera 0 --no-voice
```

장치 없이 전체 입력 연결과 렌더링·저장을 검사:

```bash
python -m pac_path_editor --webcam --synthetic --no-voice \
  --screenshot output/synthetic.png --save output/synthetic.json
```

ROS 통합 웹캠 서버는 ROS 2 Jazzy가 실제 설치된 이 환경에서 다음처럼 빌드하고 실행한다.

```bash
cd ~/manipulator_branches/voice
source /opt/ros/jazzy/setup.bash
source ../.venv-webcam/bin/activate
python -m colcon build --symlink-install --packages-select manipulator_interfaces manipulator_voice
source install/setup.bash
cd "../main/path 편집"
python -m pac_path_editor --webcam --camera 0 --ros
```

다른 터미널에서 Gemini 음성 명령을 연결한다. API 키가 있는 `voice/.env`를 준비한다.

```bash
cd ~/manipulator_branches
source /opt/ros/jazzy/setup.bash
source .venv-webcam/bin/activate
source voice/install/setup.bash
ros2 run manipulator_voice voice_commander --env-file voice/.env --no-speaker
```

통합 모드에서는 `ros_bridge.py`가 `/manipulator/edit_command`의 유일한 서버이자 실제 Editor 상태의 권한자다. `ros2 run manipulator_voice edit_state_node`는 음성 모듈 단독 시험용 상태 시뮬레이터이므로 동시에 실행하지 않는다.

기존 matplotlib 편집기와 headless 렌더링도 유지된다.

```bash
python -m pac_path_editor --config configs/demo.json
python -m pac_path_editor --config configs/demo.json --demo bump \
  --headless --screenshot output/bump.png
```

## 조작

| 입력 | 동작 |
|---|---|
| `E` | 편집 시작 |
| `1`, `2`, `3` | both, outbound, return 선택. 대기 중이면 편집도 시작 |
| 손을 펼친 뒤 경로에서 핀치 | 경로 선택과 드래그 시작 |
| 핀치 유지 이동 | landmark XYZ 차이를 gain/sign으로 변환해 미리보기 |
| 핀치 해제 | 제스처 하나를 공동 이력으로 확정 |
| `U`, `Y`, `R` | undo, redo, 원본 초기화 |
| `Esc` | 현재 드래그 취소 |
| `C` | 편집 세션 전체 취소 |
| `F` | 기하·시간 검사 후 저장 |
| `Q` | 종료. 진행 중 드래그는 취소 |

음성 표현은 “가는 길 수정”, “오는 길 수정”, “둘 다 수정”, “되돌려/뒤로 가기”, “다시 적용/앞으로 가기”, “처음으로/초기화”, “수정 취소”를 지원한다. 드래그 중 모드 변경·undo·redo·reset·finish는 Editor가 이유와 함께 거부한다. Gemini 안내는 실제 서비스 결과를 받은 뒤 생성된다.

핀치는 엄지 끝(4)과 검지 끝(8)의 영상상 거리/손 크기 비율로 판정한다. 시작·해제 임계값과 시간 debounce를 분리했다. 편집 진입 시 이미 닫힌 손, 경로 밖에서 시작한 핀치, 추적 상실 뒤 재인식은 자동으로 드래그를 시작하지 않는다. 짧은 추적 상실은 미리보기를 유지하고, 설정 시간을 넘으면 드래그를 취소한다.

## 화면과 좌표

- 영상 반전은 `prepare_frame`에서 한 번만 수행한다. 그 프레임을 MediaPipe, 관절 표시, 커서, 선택과 오버레이가 함께 사용한다.
- outbound는 시작→목표 실선, return은 목표→시작 점선이다. 좌표를 억지로 벌리지 않고 이름과 화살표로 구분한다.
- 비대상을 먼저 반투명으로 그리고 대상을 나중에 그린다. 드래그 중 이전 대상은 검정 비교선, 새 경로는 고정 공통 Z colormap으로 표시한다.
- `both`는 선택한 원본 `xi`를 두 경로에서 고정해 한 번의 undo로 되돌린다.
- X는 화면 오른쪽, Y는 화면 위, Z는 landmark z의 별도 gain/sign에 대응한다. 수치는 virtual 단위이고 화면에 landmark XYZ, 요청/적용 변위, Z 값을 표시한다.
- 현재 정사영 `OverlayProjection`은 경로의 X/Y만 화면 위치로 투영한다. Z는 선을 화면 위아래로 옮기지 않고 고정 colormap의 색과 숫자로 표시된다. MediaPipe landmark Z는 손목 기준 상대 깊이이므로 손 전체의 절대 앞뒤 거리로 해석할 수 없고, gain을 키워도 이 의미상의 한계는 해결되지 않는다.

## 저장과 schema

schema 2는 `frame`을 설정과 루트에 함께 기록한다. 웹캠 세션은 `virtual_edit_frame`, 기존 matplotlib 세션은 `robot_base`이므로 서로 조용히 섞지 않는다. correction에는 흡수 전 곡선의 벡터 field를 보존하는 `profile_residual`이 추가되었고, 경로에는 원본 길이/시간과 timing length가 저장된다.

schema 1 파일은 자동 추측하지 않는다. 명시적으로 이동할 때만 다음을 실행한다.

```bash
python -c 'from pac_path_editor.storage import migrate_v1_session; migrate_v1_session("old.json", "schema2.json")'
```

마이그레이션은 schema 1의 `robot_base`만 지원하고, 기하를 재구성해 현재 검사와 일치하는지 확인한 뒤 원자적으로 저장한다.

## 검사

```bash
cd ~/manipulator_branches/main/'path 편집'
../../.venv-webcam/bin/python -m pytest -q
```

검사는 끝점, xi, 원본 대비 변형, 실행 시간과 EE 속도를 다룬다. IK, 관절 한계, 충돌, 특이점, 관절 동역학과 토크는 `NOT_EVALUATED`다. 화면의 PASS는 실제 로봇 실행 허가가 아니다. 설계 근거는 [DESIGN.md](DESIGN.md), 실행 결과는 [RESULT.md](RESULT.md)에 있다.
