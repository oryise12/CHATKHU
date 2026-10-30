# pac_vision

`pac_vision`은 Orbbec Astra Pro(RGB + depth) 카메라 화면의 점을 **매니퓰레이터 base 기준 좌표(base_ref, m)** 로 바꾸는 파이썬 패키지다.
현장에서 A4 두 장을 기준으로 5점을 클릭해 카메라→base 변환을 만들고, 그 뒤 화면을 클릭하면 그 점의 base 좌표가 나온다.
좌표계·토픽 이름은 임시안이며 [`INTERFACE.md`](INTERFACE.md)를 따른다. 단위 m, 축은 REP-103(+X 앞, +Y 왼쪽, +Z 위).
물체 좌표(`detect_objects.py`)는 코드로 남아 있지만 현재 팀 시나리오에서는 쓰지 않는다. 손가락·핀치 인식은 다음 작업이다.

- 핵심 계산은 rclpy·OpenCV 없이 import되는 순수 파이썬(numpy, PyYAML)이다. 카메라(openni)·화면(OpenCV)·ROS 2(rclpy)는 그 위에 얹는다.
- 현장 절차(사전 점검 → 정합 → 캘리브레이션 → 좌표 확인)와 팀원 Jazzy 확인 체크리스트는 [`CALIBRATION_GUIDE.md`](CALIBRATION_GUIDE.md).
- 카메라 광학 좌표계: +X 오른쪽, +Y 아래, +Z 정면. base_ref: 책상면 z = 0.

## 현재 상태 (2026-10-15)

| 항목 | 상태 |
|---|---|
| 캘리브레이션 `calibrate.py` | 실제 카메라로 여러 번 실행. 정합 계수 맞춤에 쓰지 않은 설치 자세에서 A4 변 오차 최대 약 1 cm (2026-10-14 +11 mm, 2026-10-15 다른 설치 +11.9 mm. 화면에는 기존 기준 5 mm 미통과 ❌로 표시됨 → "알려진 한계"). 실제 세션 `--replay` 일치는 테스트로 고정 |
| 좌표 확인 `show_base_coords.py` | 실제 카메라에서 클릭·좌표 표시 동작 확인. **점별 오차 표는 아직 없음** |
| 정합 계수 | 카메라 실물 1용 프리셋 1개 ([`config/pose_presets/README.md`](config/pose_presets/README.md)) |
| 물체 좌표 `detect_objects.py`, ROS 2 발행 `--ros` | 한 설치 자세에서 실측 (측정 12절), Humble에서 `ros2 topic echo` 수신 확인. 현재 시나리오에서는 쓰지 않음 |
| 사전 점검 `check_setup.py`, 설정 덮어쓰기(`local.yaml`·`pose.yaml`), RGB 장치 자동 탐색 | 이 컴퓨터에서 카메라 포함 실행 |
| 테스트 `tests/` (카메라 없음) | 90 passed (2026-10-15, Python 3.10.12, numpy 2.2.6, OpenCV 5.0.0) |
| Ubuntu 24.04 / Jazzy / Python 3.12 | ⚠️ 미확인 ([`CALIBRATION_GUIDE.md`](CALIBRATION_GUIDE.md) 끝 체크리스트) |
| base_ref ↔ 실제 로봇 base_link | 🔲 로봇이 오면 확인 (지금은 테이프·종이로 만든 임시 기준) |

## 빠른 시작

터미널 하나에서 순서대로 한다. **ROS를 `source`하지 않은 새 터미널**에서 시작한다(ROS 발행을 쓸 때만 따로 source).
1–2번은 카메라 없이, 5번부터 카메라가 필요하다. 처음 한 번만 하는 단계와 현장마다 하는 단계를 구분해 적었다.
명령은 모두 `pac_vision/` 폴더에서, 가상환경을 켠 상태(`source ~/pac_ws/.venv/bin/activate`)로 실행한다.

```bash
# 0) 받기 — 처음 한 번
mkdir -p ~/pac_ws/src && cd ~/pac_ws/src
git clone -b work/astra-calibration git@github.com:jaemin0319/26_project_manipulator.git 26_project_manipulator
#    SSH 키가 없으면: https://github.com/jaemin0319/26_project_manipulator.git
cd ~/pac_ws/src/26_project_manipulator/pac_vision
```

```bash
# 1) 가상환경과 패키지 — 처음 한 번
#    Ubuntu 24.04는 시스템 파이썬에 pip 설치가 막혀 있어(PEP 668) 반드시 가상환경을 쓴다.
#    --system-site-packages: ROS의 rclpy와 시스템 numpy를 가상환경 안에서 쓰기 위해.
python3 -m venv --system-site-packages ~/pac_ws/.venv
source ~/pac_ws/.venv/bin/activate          # 프롬프트 앞에 (.venv)가 보여야 함
python -m pip install --upgrade pip "setuptools>=68,<80"   # <80: ROS colcon-core가 setuptools<80을 요구
python -m pip install -e .                  # 핵심 계산: numpy, PyYAML
python -m pip install opencv-contrib-python==4.11.0.86 openni Pillow   # 화면·카메라용 (아래 "설치 상세")
python -m pytest --version || python -m pip install pytest             # pytest가 없을 때만
```

- ⚠️ 화면·카메라용 패키지 줄과 pytest 줄은 Jazzy(Python 3.12, `numpy<2`)에서 아직 실행해 보지 않았다. numpy를 2.x로 올리지 않는지 출력을 확인한다.
- depth 카메라에는 pip 패키지 외에 Orbbec 드라이버(redist 폴더)와 USB 권한(udev, `sudo`)이 필요하다 → "설치 상세"의 카메라 드라이버.

```bash
# 2) 카메라 없이 확인 — 1분
python -m pytest -q tests                   # 기대: 87 passed
python scripts/show_base_coords.py --session tests/data/sample_session.npz --no-gui --click 320,240 --click 400,300
# 기대 마지막 두 줄:
#   [점 1] RGB (320, 240) → P (2.6, 44.3) cm | D depth 없음 | depth 세션 평균
#   [점 2] RGB (400, 300) → P (20.2, 15.1) cm | D (20.2, 15.2, 0.0) cm | depth 세션 평균
```

```bash
# 3) 이 컴퓨터 설정 — 처음 한 번 (카메라 드라이버·USB 권한은 "설치 상세")
cp config/local.example.yaml config/local.yaml   # OpenNI redist 경로, RGB 장치 번호 등을 자기 컴퓨터에 맞게 수정 (필요한 줄만 남김)
```

```bash
# 4) 정합 계수 — 카메라를 바꿀 때 (카메라 실물 1이면 아래 한 줄)
cp config/pose_presets/astra_unit1.yaml config/pose.yaml
#    config/pose.example.yaml 은 형식 예시(다른 설치 자세 값)이므로 pose.yaml 로 쓰지 말 것.
#    카메라 실물 1은 pose_presets/astra_unit1.yaml 을 쓴다.
#    다른 Astra 실물이면 이 프리셋을 쓰지 말 것 → CALIBRATION_GUIDE의 "정합 쌍 재기"
#    pose.yaml이 이미 있으면 덮어쓴다. 필요하면 먼저 사본을 남긴다.
python scripts/check_setup.py               # 3·4번 점검 (카메라 연결 후. 카메라 없이: --no-camera)
#    기대 마지막 줄: [요약] ⚠️ 1개   (그 하나는 'cam2base.yaml (보정 결과): 없음'. 5번 뒤 [요약] 모두 ✅)
```

```bash
# 5) 캘리브레이션 — 카메라를 놓을 때마다 (약 5분)
python scripts/calibrate.py
#    배치 점검 화면에서 Enter → base → 꼭짓점 1 → 2 → 3 → 4 클릭 → s 저장 (검사 ❌이면 s를 한 번 더)
```

```bash
# 6) 좌표 확인
python scripts/show_base_coords.py          # 클릭: 좌표 / g 격자 / f 정지 / s 저장 / c 지우기 / q 종료
```

- 5번과 6번은 동시에 실행할 수 없다(카메라를 함께 못 씀). 창은 `q`로 닫는다(창 닫기 버튼으로 닫으면 프로그램이 남아 다음 실행에서 카메라가 안 열린다).
- 기준 종이 배치 (`config/a4_layout.yaml` 값, base_ref m, 위에서 볼 때 반시계):

| 점 | x | y | 위치 |
|---|---|---|---|
| base | 0 | 0 | 대각선 종이의 대각선 교차점 |
| 꼭짓점 1 | −0.1485 | −0.105 | 대각선 종이 뒤-오른쪽 |
| 꼭짓점 2 | 0.1485 | −0.105 | 대각선 종이 앞-오른쪽 |
| 꼭짓점 3 | 0.1485 | 0.315 | 맞댄 종이 앞-왼쪽 |
| 꼭짓점 4 | −0.1485 | 0.315 | 맞댄 종이 뒤-왼쪽 (재점검 테이프 자리) |

  A4 두 장을 긴 변끼리 이어 붙여 29.7 × 42.0 cm 직사각형을 만들고, 한 장(대각선 종이)에 대각선 두 개와 X 화살표(+X, 로봇 앞쪽)를 그린다.
  맞댄 종이는 대각선 종이의 왼쪽(+Y). 자세한 준비는 "실행 상세 → 캘리브레이션".

## 파이프라인

### 전체 흐름

```text
[처음 한 번]                     [카메라를 놓을 때마다]                         [쓸 때]
설치(venv, -e .) ─▶ local.yaml ─▶ check_setup ─▶ 카메라 고정 ─▶ A4 두 장 배치 ─▶ calibrate.py ─▶ show_base_coords.py
pose.yaml ◀─ 프리셋 복사          (✅/⚠️ 점검)                                       │ s 저장          클릭 → base 좌표
            (카메라 실물마다)                                                         ▼
                                                    config/cam2base.yaml  +  data/sessions/<날짜_시각>.npz
                                                              │                          │
                                                  calibrate.py --check          calibrate.py --replay
                                                  (재점검: 카메라가 밀렸나)      (카메라 없이 같은 계산 재현)
```

### calibrate.py 내부

```text
설정 읽기   camera.yaml(intrinsics, Z 원점 보정, 좌우반전) + local.yaml + pose.yaml(정합 계수)
            + a4_layout.yaml(기준점의 base 좌표) + thresholds.yaml(검사 기준)
   │        pose.yaml이 없으면 시작하지 않음 (시연용 --allow-default-pose)
카메라 열기  RGB: UVC 640×480 (OpenCV)   depth: OpenNI 640×480, 좌우반전
   │
배치 점검   화면 아래쪽(가까운 쪽) ≥ 0.6 m, 위쪽(먼 쪽) ≤ 1.5 m(경고만), depth 유효율 → Enter
   │
정지 화면   30프레임 평균 (새 프레임만, depth는 0 제외)
   │
5점 클릭    RGB 화면: base → 꼭짓점 1 → 2 → 3 → 4 (확대경)
   │
정합        RGB 픽셀 → depth 픽셀 (pose.yaml 계수: dx 거리 식, dy 고정)
   │
3D화        클릭 영역의 책상 평면 맞춤 (RANSAC + 최소제곱) → 광선-평면 교차 → 카메라 좌표
   │
변환 계산   꼭짓점 1–4 ↔ base 좌표 → Kabsch(SVD) → base 점 홀드아웃 → 5점 Kabsch로 최종 R·t
   │
검사        base 점 홀드아웃, A4 변 길이, 반사(det R·카메라가 책상 위인가) — 실패 시 s 두 번
   │        A4 화면 크기·책상면 z — 경고만
저장 (s)    config/cam2base.yaml (R·t, 사용한 정합 계수 사본, 검사 결과) + 세션 npz (RGB·depth·클릭·설정 사본)
```

### show_base_coords.py 내부

```text
cam2base.yaml 읽기 (없으면 실행 거부) → 상태줄: 보정 파일 경로·생성 시각·검사 결과·정합 계수 출처(보정 때 사본 / 지금 pose.yaml과 같은지)
   │
RGB 클릭 (u, v)
   ├─▶ P (책상면):  보정 때 맞춘 책상 평면과 클릭 광선의 교점, z = 0. 책상 위 점에만 맞음. depth가 없어도 나옴
   └─▶ D (depth):   정합 → depth 30프레임 평균(5×5 중앙값) → 역투영 → R·t → (x, y, z). 높이가 있는 점용
   │
화면 표시(점 번호, 마지막 점 라벨) + 터미널 한 줄 + s 저장 시 PNG·CSV (data/sessions/demo_*)
```

- `--session <npz>`로 실행하면 카메라 대신 세션의 정지 화면과, 그 세션을 다시 계산한 보정을 쓴다 (D는 세션의 평균 depth 한 장).
- P와 `calibrate.py --check`의 출력은 조금 다를 수 있다. `--check`는 클릭 주변의 책상 평면을 다시 맞추고, P는 보정 때의 평면을 쓴다.

### 파일 입출력

| 파일 | 만드는 것 | git | 읽는 것 |
|---|---|---|---|
| `config/camera.yaml`, `a4_layout.yaml`, `thresholds.yaml`, `frames.yaml`, `detect.yaml` | 사람 | 커밋 | 모든 스크립트 |
| `config/local.yaml` | 사람 (`local.example.yaml` 복사) | 제외 | 모든 스크립트 |
| `config/pose_presets/*.yaml` | `tools/fit_rgb_to_depth.py --fragment` | 커밋 | 사람이 `pose.yaml`로 복사 |
| `config/pose.yaml` | 사람(프리셋 복사) 또는 `fit_rgb_to_depth.py --write-pose` | 제외 | calibrate, show_base_coords, detect_objects, check_setup |
| `config/cam2base.yaml` | calibrate (s) | 제외 | show_base_coords, `--check`, detect_objects, check_setup |
| `data/sessions/<날짜_시각>.npz` | calibrate (s) | 제외 | `--replay`, `show_base_coords --session` |
| `data/sessions/demo_*.png`·`.csv` | show_base_coords (s) | 제외 | 사람 |
| `data/sessions/camera_check_*.csv` | check_camera (q) | 제외 | `fit_rgb_to_depth.py` |
| `data/measurements/*.csv` | 측정 후 정리 | 커밋 | `tools/recompute_measurements.py`, 테스트 |

git 제외는 `pac_vision/.gitignore`가 한다 (`config/cam2base.yaml`, `config/local.yaml`, `config/pose.yaml`, `data/sessions/*`, `*.pt`).

## 폴더 구조

```text
pac_vision/
├── pyproject.toml          # setuptools, src 레이아웃. 의존성: numpy, PyYAML (버전 고정 없음)
├── README.md / INTERFACE.md / CALIBRATION_GUIDE.md (현장 절차)
├── src/pac_vision/         # 패키지 본체
│   ├── cam2base.py         # numpy만: 카메라 모델, 평면 맞춤, Kabsch, 검사, solve_calibration, Cam2Base
│   ├── base_coords.py      # numpy만: 클릭 → P(책상면)·D(depth) 좌표, 책상면 격자
│   ├── objects.py          # numpy만: 윗면 중심, 박스→depth 영역, 최근접 대체 선택
│   ├── session.py          # 세션(npz)·보정 결과(cam2base.yaml) 저장·읽기, 재계산
│   ├── config.py           # config/*.yaml 읽기, local.yaml·pose.yaml 덮어쓰기
│   ├── astra_depth.py      # OpenNI depth (openni는 open() 안에서만 import)
│   └── ros_pub.py          # ROS 2 발행 (rclpy는 이 파일에서만)
├── scripts/                # 실행 도구 (OpenCV 화면)
│   ├── calibrate.py        # 캘리브레이션 / --check 재점검 / --replay 카메라 없이 재계산
│   ├── show_base_coords.py # 화면 클릭 → base 좌표 (보정 확인)
│   ├── detect_objects.py   # 물체 좌표, 클릭 선택·추적, --ros
│   ├── check_camera.py     # 카메라 확인 (좌우반전, Z, 노이즈, RGB↔depth 정합 쌍)
│   ├── check_setup.py      # 사전 점검 (버전, git, 설정 파일, 경로, 장치, 보정 파일)
│   └── _ui.py              # 화면 공용 (한글 문구, 확대경, 화살표, 상태줄)
├── config/                 # 이름·상수·기준값은 모두 여기 (코드에 직접 쓰지 않음, 아래 "설정 파일")
│   └── pose_presets/       # 카메라 실물별 정합 계수 프리셋 (README에 출처·재계산 명령)
├── data/measurements/      # README 측정 수치의 원자료 (git에 올림, 목록은 그 폴더의 README)
├── data/sessions/          # 캘리브레이션 세션·카메라 확인 기록·좌표 확인 저장 (git 제외)
├── tools/                  # recompute_measurements.py (측정 재계산), fit_rgb_to_depth.py (정합 계수 맞춤), rgb_modes.py (RGB 모드 재측정)
├── tests/                  # 카메라 없이 도는 pytest (합성 장면 synth.py, 실제 세션 샘플 data/sample_session.npz)
└── prototype/              # 모듈 이전의 단독 스크립트. 측정 1-8절에 쓴 원본. 수정하지 않고 보존
```

## 설정 파일 (커밋된 파일은 고치지 않는다)

| 분류 | 값 | 파일 | git |
|---|---|---|---|
| 커밋 (모두 같음) | intrinsics, Z_OFFSET, 좌우반전, RGB 640×480, 글꼴 기본값 | `config/camera.yaml` | 커밋 |
| 커밋 (모두 같음) | 기준 종이 꼭짓점 / 검사 기준값 / 좌표계·토픽 이름 / YOLO 설정 | `a4_layout.yaml` / `thresholds.yaml` / `frames.yaml` / `detect.yaml` | 커밋 |
| 컴퓨터별 | RGB 장치 번호, OpenNI 경로, 글꼴, YOLO 모델 경로 | `config/local.yaml` (예시 `local.example.yaml`) | 제외 |
| 설치 자세별 | R·t (보정 때 쓴 정합 계수 사본 포함) | `config/cam2base.yaml` (calibrate.py가 만듦) | 제외 |
| 프리셋 (카메라 실물별) | RGB→depth 정합 계수 | `config/pose_presets/<실물>.yaml` → `config/pose.yaml`로 복사 | 프리셋 커밋, `pose.yaml` 제외 |

- `config/pose.example.yaml` 은 형식 예시(다른 설치 자세 값)이므로 pose.yaml 로 쓰지 말 것. 카메라 실물 1은 `pose_presets/astra_unit1.yaml` 을 쓴다.
- 프리셋이 없는 카메라는 `check_camera.py`로 정합 쌍을 재고 `fit_rgb_to_depth.py --write-pose`로 `pose.yaml`을 만든다 ("실행 상세 → 정합 계수 측정").
- `local.yaml`은 컴퓨터별 값만, `pose.yaml`은 정합 계수(`camera.rgb_to_depth`)만 받는다. 다른 키가 있으면 어느 파일의 어느 키인지 알리고 멈춘다.
- `camera.yaml`의 `rgb_to_depth` 값(dx 23, dy 8)은 옛 형식 호환용 기본값이다. `pose.yaml`이 없을 때만 쓰인다.
- RGB 장치 번호를 적지 않으면 장치 이름에 `Astra`(`camera.yaml`의 `rgb.name_match`)가 들어간 장치 중 프레임이 읽히는 첫 장치를 쓴다.
- `pose.yaml`이 없으면 `calibrate.py`는 실행을 거부한다 (시연용 `--allow-default-pose`). `detect_objects.py`는 경고만 한다.
- 보정 때 정합 계수와 지금 설정이 다르면 `detect_objects.py`·`--check`가 경고하고, `show_base_coords.py` 상태줄에 "다름 ⚠️"가 나온다 (정합을 다시 쟀다면 `--replay <세션> --use-current-config`로 보정 갱신).

## 설치 상세

### 확인한 환경

| 항목 | 실제 확인 (2026-10-03, 테스트 수는 2026-10-14) | 팀 실행 환경 |
|---|---|---|
| OS / ROS 2 | Ubuntu 22.04.5 / Humble | Ubuntu 24.04 / Jazzy |
| Python | 3.10.12 (`venv --system-site-packages`) | 3.12 |
| numpy | 2.2.6 (2026-10-03에 테스트 29개를 1.26.4에서도 확인) | `numpy<2` |
| PyYAML | 5.4.1 (테스트는 6.0.3에서도 통과) | 6.x 예상 |
| OpenCV | 5.0.0 | opencv-contrib-python 4.11.0.86 |
| ultralytics / torch | 8.4.104 / 2.13.0+cpu | 미확인 |
| openni / Pillow / lap | 2.3.0 / 12.1.1 / 0.5.13 | 미확인 |
| 결과 | 테스트 87 passed (2026-10-14). 캘리브레이션·좌표 확인·물체 좌표·ROS 발행 실측 | **미확인** |

### 설치 명령

빠른 시작 0–1번과 같다. 저장소를 받은 위치를 `REPO`라 한다 (예: `~/pac_ws/src/26_project_manipulator`). 가상환경 위치는 자유이며, 아래는 예시다.

```bash
REPO=~/pac_ws/src/26_project_manipulator     # 자기 경로로 바꾼다
git -C "$REPO" switch work/astra-calibration

python3 -m venv --system-site-packages ~/pac_ws/.venv
source ~/pac_ws/.venv/bin/activate
python -m pip install --upgrade pip "setuptools>=68,<80"
python -m pip install -e "$REPO/pac_vision"        # numpy, PyYAML만 필요 (이미 있으면 설치하지 않음)
```

스크립트(카메라·화면·YOLO)를 쓰려면 추가로 필요하다. `pyproject.toml`에는 넣지 않았다 (핵심 계산만 쓰는 쪽이 무거운 패키지를 받지 않게).

| 패키지 | 쓰는 곳 | 비고 |
|---|---|---|
| OpenCV (`cv2`) | 모든 스크립트 | 팀 고정 `opencv-contrib-python==4.11.0.86` |
| `openni` + Orbbec redist + udev 규칙 | depth | 아래 "카메라 드라이버" |
| `Pillow` + 한글 글꼴 | 화면의 한글 문구 | 없으면 영어 문구로 자동 대체. 글꼴 경로는 `config/camera.yaml`의 `ui.font` (컴퓨터별로는 `local.yaml`) |
| `ultralytics`, `torch` | `detect_objects.py`만 | CPU판 예: `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu` 후 `pip install ultralytics` |
| `lap` | 물체 추적 (ByteTrack) | 없으면 추적 없이 검출만 하고 최근접 대체로 선택을 잇는다. 실행 중 자동 설치는 막아 두었다 |
| ROS 2 (`rclpy`, `geometry_msgs`) | `--ros` | apt로 설치된 ROS를 `source` (venv는 `--system-site-packages`) |
| `pytest` | 테스트 | 시스템에 있으면 그대로 쓰인다 (`python -m pytest --version`) |

⚠️ **ultralytics 설치 시 opencv-python이 함께 설치되어 팀 고정 opencv-contrib-python 4.11과 충돌할 수 있다 → 설치 후 opencv-contrib-python==4.11.0.86 재설치.**

```bash
python -m pip install ultralytics
python -m pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python
python -m pip install opencv-contrib-python==4.11.0.86
python -c "import cv2; print(cv2.__version__)"    # 기대: 4.11.0
```

YOLO 가중치는 `config/detect.yaml`에 이름(`yolo26n.pt`)만 적혀 있고, 없으면 첫 실행 때 ultralytics가 내려받는다. 이미 받은 파일이 있으면 `--model 경로`로 지정한다.

### 카메라 드라이버 (depth)

Astra Pro는 센서 두 개가 다른 경로로 열린다. RGB는 UVC(일반 웹캠처럼 OpenCV `VideoCapture`), depth는 OpenNI2 + Orbbec 드라이버.
아래는 Ubuntu 22.04에서 실제로 한 순서다. 24.04에서는 확인하지 않았다.

```bash
# 1) OpenNI2 파이썬 바인딩 (가상환경을 켠 뒤 설치)
sudo apt install -y libopenni2-dev        # ⚠️ sudo: 시스템 패키지 설치
python -m pip install openni              # 확인 버전: openni 2.3.0

# 2) Orbbec 드라이버(.so). 저장소를 빌드하지 않고 미리 컴파일된 redist 폴더만 쓴다
cd ~ && git clone https://github.com/orbbec/ros2_astra_camera.git
find ~/ros2_astra_camera -name "libOpenNI2.so"   # .../openni2_redist/x64 를 사용 (arm, arm64 무시) → local.yaml의 camera.openni.redist

# 3) USB 권한 (udev 규칙). 2bc5 = Orbbec 제조사 ID
sudo bash -c 'cat > /etc/udev/rules.d/99-orbbec.rules << EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE:="0666"
EOF'                                      # ⚠️ sudo: 시스템 설정 변경
sudo udevadm control --reload-rules && sudo udevadm trigger
# 그다음 카메라를 뽑았다가 다시 꽂는다
```

카메라 연결 확인:

```bash
lsusb | grep -i orbbec
for d in /dev/video*; do echo -n "$d : "; cat /sys/class/video4linux/$(basename $d)/name; done
```

리눅스에서 카메라 하나는 `/dev/video` 번호를 두 개(영상용 + 메타데이터용) 차지한다. 측정 컴퓨터에서는 `video0·1`이 내장 카메라, `video2·3`이 Astra였고, RGB는 **2번**이었다.

### 설치 확인

```bash
python -c "import pac_vision, sys; print(pac_vision.__version__, pac_vision.__file__, sys.prefix)"
# 기대: 0.1.0 <REPO>/pac_vision/src/pac_vision/__init__.py <가상환경 경로>
env -u PYTHONPATH -u AMENT_PREFIX_PATH python -c "import pac_vision, sys; print('ok', [m for m in ('rclpy', 'cv2', 'openni') if m in sys.modules])"
# 기대: ok []   (핵심 모듈은 rclpy·OpenCV·openni를 불러오지 않는다)
```

`git status`에 `*.egg-info`, `config/cam2base.yaml`, `config/local.yaml`, `config/pose.yaml`, `data/sessions/*`가 보이지 않아야 한다 (`pac_vision/.gitignore`가 제외).

### Jazzy(Ubuntu 24.04, Python 3.12)에서 달라질 수 있는 곳

코드에는 해당 줄에 ⚠️ 주석을 달았다. 아직 팀원 컴퓨터에서 확인하지 않았다.

| 항목 | 이 컴퓨터 | 확인할 것 |
|---|---|---|
| OpenNI redist 경로, `openni` 설치 | `~/ros2_astra_camera/.../x64` | 다르면 `config/local.yaml`의 `camera.openni.redist` (`check_setup.py`가 확인) |
| RGB 장치 번호 | 자동 탐색 → 2 | 장치 이름에 `Astra`가 들어가는지 (`cat /sys/class/video4linux/*/name`). 다르면 `local.yaml`의 `camera.rgb.device` |
| OpenCV | 5.0.0 | `numpy<2` + opencv-contrib 4.11. 창 백엔드(Qt/GTK)에 따라 Enter 키 코드(13/10 둘 다 처리)·마우스 이벤트가 다를 수 있음 |
| ultralytics 설치 | 문제 없음 | opencv-python을 끌어오면 위 ⚠️ 절차로 재설치 |
| PyYAML | 5.4.1 | 6.x (safe_load만 사용, 6.0.3에서 테스트 통과) |
| pytest | 6.2.5 | 7.4 이상이면 `pythonpath` 경고 없음 (9.1.1에서 테스트 통과) |
| 한글 글꼴·Pillow | Noto Sans CJK, Pillow 12 | 없으면 영어 문구로 대체 |
| lap | 0.5.13 | 없으면 추적 없이 검출 + 최근접 대체 (자동 설치 안 함) |
| ROS 2 | `source /opt/ros/humble/setup.bash` | `source /opt/ros/jazzy/setup.bash`. venv가 `--system-site-packages`여야 rclpy가 보임 |

### 설치 함정 (실제로 겪음)

| 증상 | 원인 | 해결 |
|---|---|---|
| `build backend is missing the 'build_editable' hook` | Ubuntu 22.04에서 새로 만든 venv에는 setuptools 59.6 / pip 22.0.2가 들어 있어 pyproject만 있는 패키지의 editable 설치(PEP 660)를 못 함 | venv 안에서 `python -m pip install --upgrade pip "setuptools>=68,<80"` 후 재설치. 시스템 파이썬은 건드리지 않음 |
| `colcon-core ... requires setuptools<80, but you have setuptools 8x` (pip 경고) | `setuptools>=68`만 쓰면 venv에 최신 setuptools(2026-10-15: 84.0.0)가 설치되고, `--system-site-packages`로 보이는 ROS colcon-core와 버전 요구가 맞지 않음 | `python -m pip install "setuptools>=68,<80"` (2026-10-15: 79.0.1로 editable 설치·테스트 통과) |
| `Unknown config option: pythonpath` (pytest 경고) | pytest 7 미만(Ubuntu 22.04 시스템 pytest 6.2.5) | 무시해도 됨 (editable 설치라 import 경로 문제 없음) |
| `RGB 카메라를 열지 못했습니다` | 다른 프로그램(이전에 띄운 스크립트 포함)이 RGB 장치를 쓰는 중 | `fuser -v /dev/video*`로 확인 후 그 프로그램 종료 |

## 실행 상세

모든 명령은 같은 준비 뒤에 실행한다.

```bash
cd ~/pac_ws/src/26_project_manipulator/pac_vision
source ~/pac_ws/.venv/bin/activate
```

### 사전 점검 `check_setup.py` (받은 뒤·현장에서 먼저)

```bash
python scripts/check_setup.py              # 카메라 열기까지
python scripts/check_setup.py --no-camera  # 카메라를 다른 프로그램이 쓰는 중일 때
# 기대 마지막 줄: [요약] 모두 ✅   (⚠️ 항목마다 '→ 해결 방법'이 한 줄씩 붙는다. 캘리브레이션 전에는 cam2base.yaml 없음 ⚠️)
```

버전(Python·numpy·OpenCV·PyYAML·ultralytics·openni·Pillow·lap·rclpy), git 브랜치·커밋, 추적되는 config의 로컬 수정 여부, config 파일과 덮어쓰기 파일 키 검사,
`pose.yaml` 유무, OpenNI 경로, 글꼴, 보정 파일(날짜·보정 때 정합 계수 = 지금 설정인지), RGB·depth 장치 열기를 확인한다.

### 캘리브레이션 `calibrate.py` (빈 책상, 현장마다 3-5분)

`config/pose.yaml`(정합 계수)이 없으면 실행을 거부한다 (시연용 `--allow-default-pose`). 카메라 실물 1이면 빠른 시작 4번의 프리셋 복사, 다른 실물이면 아래 "정합 계수 측정".

준비 (기준 종이, 줄자 불필요):
- A4 두 장을 **긴 변끼리 끝을 맞춰 뒤에서 테이프로** 이어 붙인다 (틈·어긋남 없이). 두 장의 짧은 변이 일직선인지 자로 확인한다.
- 한 장에 굵은 마커로 대각선 두 개를 긋는다. **교차점 = base (0, 0, 0)**. 교차점을 지나 긴 변과 평행하게 X 화살표(로봇 앞쪽)를 그린다. +Y는 맞댄 종이 쪽(왼쪽).
- 바깥 꼭짓점 번호를 종이 안쪽에 적는다 (기본 `config/a4_layout.yaml`): 1 (−0.1485, −0.105), 2 (0.1485, −0.105), 3 (0.1485, 0.315), 4 (−0.1485, 0.315) m, 위에서 볼 때 반시계.
- 종이는 변 가운데만 테이프로 책상에 고정한다 (꼭짓점을 가리지 않게).
- 저장 뒤 로봇은 교차점 위에 앞면을 X 화살표 방향으로 놓는다. 종이를 치울 거면 먼저 교차점에 테이프 십자를 붙인다.

```bash
python scripts/calibrate.py                  # --depth-mode point: 평면 맞춤 대신 클릭 점 주변 depth
```

1. 배치 점검 화면: 화면 아래쪽(가까운 쪽) 거리 ≥ 0.6 m, 위쪽(먼 쪽) ≲ 1.5 m, depth 유효율 확인 → **Enter** (30프레임 평균 정지 화면)
2. 확대경을 보며 교차점(base) → 꼭짓점 1 → 2 → 3 → 4 클릭 (`u` 되돌리기, `r` 처음부터, `b` 배치 점검으로, `q` 종료)
3. 결과 화면: ○(계산으로 되돌려 그린 점)가 ●(클릭) 위에 겹치는가, 화살표(빨강 X, 초록 Y, 파랑 Z(위쪽))에서 빨강 X가 로봇 앞쪽인가, 다시 그린 A4 외곽선이 종이와 겹치는가.
   검사: base 점 홀드아웃(꼭짓점 4점으로 구한 변환으로 base 점 예측), A4 변 길이(변마다 허용 max(0.5 cm, 그 변 방향 2 px)),
   반사(평면 법선 기준 꼭짓점 순서 + 카메라가 책상 위인가), A4 화면 크기(경고만), 책상면 base z(경고만)
4. **s** 저장 → `config/cam2base.yaml`, `data/sessions/날짜_시각.npz`, 재점검용 테이프 안내 (검사 실패 상태에서는 s를 한 번 더 눌러야 저장)

```bash
python scripts/calibrate.py --replay data/sessions/<파일>.npz --no-gui     # 카메라 없이 같은 계산
# 기대 마지막 줄: [재현] 저장 때 결과와 같음 ✅ (R·t·점 좌표 최대 차이 ...)
python scripts/calibrate.py --replay data/sessions/<파일>.npz --use-current-config   # 현재 pose.yaml(dx·dy)로 재계산·저장
python scripts/calibrate.py --check          # 테이프 자리 클릭 → 오차, 이어서 다른 점 클릭 → base 좌표
```

- 창은 `q`로 닫는다. 창 닫기 버튼으로 닫으면 프로그램이 계속 실행돼 다음 실행에서 카메라가 열리지 않는다.
- `--check`로 이음매 끝 같은 검증점을 볼 때는 점을 찍지 말고 **이음매를 따라 테두리까지 짧은 선을 긋고, 선이 테두리와 만나는 점을 클릭**한다 (칠한 점은 중심이 어긋나기 쉽다).

### 좌표 확인 `show_base_coords.py`

```bash
python scripts/show_base_coords.py                      # 카메라 + config/cam2base.yaml (없으면 실행 거부)
python scripts/show_base_coords.py --config-dir <폴더>   # 그 폴더의 cam2base.yaml·pose.yaml
python scripts/show_base_coords.py --session tests/data/sample_session.npz   # 카메라 없이 (창 있음)
python scripts/show_base_coords.py --session tests/data/sample_session.npz --no-gui --click 320,240   # 창 없이 좌표만
```

- 화면을 클릭하면 점 번호와 함께 P(책상면, z = 0)와 D(depth, x·y·z)가 화면과 터미널에 나온다 (단위 cm). D는 depth가 없는 곳에서 "depth 없음".
- 키: `c` 점 지우기, `g` 책상면 격자(10 cm), `f` 화면 정지/재개, `s` 저장(`data/sessions/demo_<날짜_시각>.png`·`.csv`), `q` 종료.
- 화면 위 상태줄에 보정 파일 경로·생성 시각·검사 결과와 정합 계수 출처가 나온다. 검사 줄이 "기존 기준 미통과"여도 실행은 된다 ("알려진 한계").
- 설정: `thresholds.yaml`의 `demo:` 절 (격자 간격, 화면에 남길 클릭 수).

### 정합 계수 측정 (프리셋이 없는 카메라, 약 30-50분)

```bash
python scripts/check_camera.py
```

- depth 창 클릭: 30프레임 측정 → 카메라 X·Y·Z와 표준편차. 카메라 뒤에서 카메라와 같은 방향을 볼 때 오른쪽에 둔 물체가 **X 양수**여야 한다.
  Z는 줄자로 "카메라 하우징 앞면 → 물체 앞면"을 잰 값과 비교한다.
- RGB 창 클릭 → **바로 다음** depth 창 클릭 = 한 쌍(dx, dy). 밝은 무광 상자의 **윗면 바깥 꼭짓점**(depth 창에서 옆이 검게 비지 않은 쪽)을 찍는다.
  - 화면 좌·우·위·아래 끝, 가까운 쪽·먼 쪽, 기준 종이 둘레에 고루 15-25쌍
  - 같은 화면 높이에서 높이만 다른 쌍 2-3개 (높은 상자를 앞에, 낮은 상자를 뒤에 놓아 꼭짓점이 화면에서 같은 높이에 오게)
  - 같은 꼭짓점을 연속 두 번 찍은 쌍 2-3개 (클릭 반복성)
  - `q`로 끝내면 `data/sessions/camera_check_pairs_<시각>.csv`로 저장된다.
- 계수 맞춤 (카메라 불필요):

```bash
python tools/fit_rgb_to_depth.py data/sessions/camera_check_pairs_<시각>.csv [같은 자세의 다른 쌍 CSV ...] --write-pose
# 출력 끝: config/camera.yaml rgb_to_depth에 넣을 값:
#   dx: {a: ..., b: ..., c: ..., d: ...}
#   dy: {a: ..., b: ..., c: ..., d: ...}
# [저장] .../config/pose.yaml  (기존 파일은 pose.yaml.bak)
```

  규칙: Z ≤ 1.5 m 쌍만, 반복 클릭 차이가 3 px를 넘는 꼭짓점은 제외, dx는 거리 식, dy는 거리 식이 고정값보다 leave-one-out 0.3 px 이상 좋을 때만 거리 식.
  출력의 leave-one-out RMS가 3 px 안팎이면 충분하다 (2026-10-14: dx 2.65, dy 1.59 px). `--write-pose`가 계수를 `config/pose.yaml`에 저장하면 캘리브레이션을 한다.
  여러 설치 자세의 쌍을 합쳐 카메라 실물 하나의 계수를 만들 때는 `--pose 이름=CSV[,CSV] ... --fragment <파일>` (예: [`config/pose_presets/README.md`](config/pose_presets/README.md)의 재계산 명령).

### 물체 좌표 `detect_objects.py` (현재 시나리오에서는 쓰지 않음)

```bash
python scripts/detect_objects.py                          # 보기만 (config/cam2base.yaml이 없으면 실행하지 않음)
python scripts/detect_objects.py --model ~/경로/yolo26n.pt   # 받아 둔 가중치 사용
```

물체 클릭: 선택(노란 굵은 테두리, 추적), `c`/빈 곳 클릭: 해제, `z`: 책상면 z 점검(빈 책상에서, 화면 전체 기준이라 책상 밖은 보라색이어도 정상), `q`: 종료.
물체는 카메라에서 광축 방향 거리(Z) 0.65 m 이상인 곳에 둔다 (depth는 0.6 m 안쪽을 재지 못한다). 좌표의 알려진 한계는 아래 "알려진 한계" 절.
좌표는 윗면 중심 (x, y, z_top), base_ref, m. 책상면 위 1 cm 이하 픽셀은 버리므로 납작한 물체, 검정·광택 물체는 "depth 부족"(회색)으로 나오고 발행되지 않는다.

ROS 2 발행 (터미널 두 개):

```bash
source /opt/ros/humble/setup.bash            # ⚠️ Jazzy: /opt/ros/jazzy/setup.bash
cd ~/pac_ws/src/26_project_manipulator/pac_vision && source ~/pac_ws/.venv/bin/activate
python scripts/detect_objects.py --ros
```

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /pac/target_point            # 선택한 물체 (geometry_msgs/PointStamped, frame_id base_ref)
ros2 topic echo /pac/objects                 # 전체 물체 (geometry_msgs/PoseArray)
```

#### ⚠️ 주의: dx·dy 오프셋은 R·t로 보정되지 않는다

물체 좌표는 RGB 박스를 dx·dy로 옮긴 depth 픽셀에서 **직접** 계산한다. 캘리브레이션 때 dx·dy가 밀려 있으면 그 밀림은 R·t에 흡수되지 않고
**물체 좌표의 일정한 오프셋**으로 남는다. 캘리브레이션 검사(홀드아웃, 변 길이, `--check`)는 모두 같은 RGB 클릭 → depth 경로를 쓰므로,
큰 밀림은 A4 변 검사에 일부 걸릴 수 있지만 수 px의 작은 밀림은 통과한다. `--check`의 줄자 비교도 A4 밖 외삽 오차만 보여 준다.
`show_base_coords.py`의 D 좌표도 같은 경로라 같은 영향을 받는다.

**오프셋은 물체 좌표(또는 D 좌표)를 줄자로 잰 위치와 비교해서만 확인할 수 있다.**
방법: 캘리브레이션 때 붙인 테이프 표시(좌표를 아는 점)에 컵 바닥 가운데를 맞춰 놓고, 기대값 (표시 x, 표시 y, 컵 높이)와 출력 (x, y, z_top)을 비교한다.
가까운 쪽·먼 쪽 모두 같은 방향으로 어긋나면 dx·dy를 의심한다. 같은 자세에서 dx·dy만 다시 쟀다면 `--replay --use-current-config`로 재클릭 없이 갱신한다.

2026-10-14 한 설치 자세에서 확인: 테이프 표시에 놓은 컵의 x·z가 두 자리 모두 3 mm 안이고 방향도 일정하지 않아 **일정한 오프셋은 없었다** (측정 12절).
y(카메라에서 멀어지는 방향)의 치우침은 정합이 아니라 물체 좌표 계산 방식의 한계다 ("알려진 한계").

### 테스트 (카메라 없음)

```bash
python -m pytest -q tests
# 기대 (2026-10-14, numpy 2.2.6 / Python 3.10.12): 87 passed
```

합성 장면(55° 기울인 카메라, 책상 평면, 원기둥 물체, 벽)으로 R·t 복원, 기울어진 평면 맞춤, 반사 감지, 홀드아웃·변 길이 검사,
16:9 화면 변환, 윗면 중심, 세션 재계산 일치, RGB→depth 거리 식(옛 고정값 설정과의 호환 포함), 클릭 → base 좌표(P·D)를 확인한다.
`test_sample_session_replays_identically`는 실제 캘리브레이션 세션(2026-10-14)을 다시 계산해 저장 때와 같은지 본다.
`test_pose_preset_copies_as_pose_yaml`은 정합 프리셋을 `pose.yaml`로 복사했을 때 설정 키 검사를 통과하는지 본다.
`test_measurements.py`·`test_fit_rgb_to_depth.py`는 아래 측정 수치를 `data/measurements/` 원자료에서 다시 계산해 확인한다.

## 알려진 한계 (2026-10-14, 팀 공유 필요)

- 정합 프리셋은 **카메라 실물 1 전용**이다. 다른 실물에서는 좌표가 수 cm 어긋날 수 있다 (실물마다 RGB와 depth 센서의 상대 위치가 다름). 다른 실물은 정합 쌍을 잰다.
- 정합 계수를 맞출 때 쓰지 않은 설치 자세에서 A4 변 오차가 최대 약 1 cm라, calibrate 결과 화면의 변 검사(기준 5 mm)는 ❌로 나온다. 지금은 그 상태로 저장(s 두 번)해서 쓴다. 팀 허용 오차는 🔲.
  2026-10-15 다시 설치한 자세에서도 +11.9 mm였다 (`config/pose_presets/README.md`, 원자료 `data/measurements/2026-10-15_pose_calib.csv`).
- base_ref는 종이·테이프로 만든 임시 기준이다. 실제 로봇 base_link와의 관계는 🔲.
- `show_base_coords.py`는 동작만 확인했고 점별 오차 측정은 아직 없다.
- Jazzy 미확인 항목: OpenNI 설치·redist 경로, RGB 장치 번호, 창 백엔드별 키·마우스, numpy 1.x.

물체 좌표(`detect_objects.py`) 관련:

| 한계 | 근거 | 지금 대응 | 상태 |
|---|---|---|---|
| **물체 좌표가 카메라 쪽으로 치우침.** 이 설치 자세(광축이 수평에서 약 21° 아래)에서 물체 x·y가 **카메라 쪽으로 물체 깊이 크기의 약 절반** 치우친다. 원인: 낮은 카메라에서 수평 윗면의 depth가 일부만 나오고(윗면 depth 유효 12-38 %: 컵 종이 뚜껑 12 %, 골판지 상자 38 %), 남은 점도 카메라 쪽에 몰린다 | 12절 원인 확인 시험(외삽점 a, 30프레임 평균 2회): **뚜껑 컵 −37 mm, 상자 −43 ~ −50 mm**. 물체 좌표 측정 전체로는 뚜껑 컵 −25 ~ −38 mm. 알려진 위치로 만든 박스로 계산해도 같아 YOLO 박스 탓이 아님 | **받는 쪽(경로 계획)은 접근 여유를 둔다.** x·z는 3 mm 안 | 🔲 계산 방식 개선은 팀 안건. 실측 비교(12절)에서 `bbox_top`(depth 앞 끝 + YOLO 박스 위 끝 광선)이 컵 치우침을 −34 → −3 ~ +5 mm로 줄였다 (화면 끝에서 YOLO 위 끝 오차 약 15 mm). 폭으로 깊이를 추정하는 대칭 보정은 상쇄에 의한 통과라 비권장. 기본값은 `top_band` 유지. 카메라 설치 각도 기준도 함께 |
| **입구가 열린 용기**는 그 높이에 테두리만 있어 카메라 쪽 테두리로 반지름만큼 치우친다 | 12절: 열린 컵 −5 cm | 좌표 정의가 막힌 윗면 전제임을 공유 | 🔲 |
| depth는 카메라에서 약 0.6 m 안쪽을 재지 못한다 | 가장 가까운 테이프(외삽점 d)의 컵은 좌표 없음 | 물체는 광축 거리 0.65 m 이상인 곳에 둔다 | 안내 |
| `--check`의 클릭 주변 평면 맞춤이 책상 끝 근처에서 책상 너머 바닥을 잡을 수 있다 | 12절 외삽점 b (z −42 cm) | 2026-10-14 수정: 보정 책상면 ±2 cm 안의 점만으로 평면을 맞춘다 (`thresholds.yaml`의 `check.desk_band_m`) | 수정함 (합성 장면 테스트) |
| detect `z` 점검과 캘리브레이션 결과의 책상면 z 분포가 책상 밖 픽셀을 포함한다 | 낮은 카메라라 화면 위쪽 절반이 책상 밖 | 2026-10-14 수정: 통계는 책상면 ±2 cm 안의 점만 쓰고, 1 cm 넘게 뜬 점의 비율을 함께 출력 (`objects.desk_band_m`). 보라색 덧칠은 그대로 | 수정함 |
| 광택 책상은 depth 유효율이 낮다 (가까운 쪽 27 %) | 기준 영상 | 평면 맞춤은 동작 (잔차 1.3 mm) | 무광 깔개를 쓰면 나아질 수 있음 |

## 문제 해결 (실제로 겪은 것)

| 증상 | 원인 | 해결 |
|---|---|---|
| `OpenNIError` (`open_any`) | USB 권한 | udev 규칙 추가 후 카메라 재연결 |
| `sudo`로 실행하면 `No module named 'openni'` | 패키지가 사용자 경로에 설치됨 | `sudo`로 실행하지 말고 udev 규칙으로 권한 해결 |
| `no attribute 'destroy'` | 이 openni 버전의 종료 함수는 `unload()` | `openni2.unload()` |
| depth가 0 mm | 첫 프레임이 빔 / 0.6 m 이내 / 검은 물체 | 연속으로 읽기, 거리·타깃 바꾸기 |
| 30번 측정의 표준편차가 정확히 0.0 mm | 같은 프레임을 반복해서 읽음 (`prototype/measure_3d.py`에서 겪음) | 읽기 사이에 `time.sleep(0.04)`. 이 패키지는 새 프레임까지 기다리는 `read_frame()` + 프레임 번호가 같으면 버리는 방식으로 막는다 (`AstraDepth.read_average`, `check_camera.py`, `calibrate.py` 정지 화면) |
| `VIDEOIO ... can't open camera by index` | 없는 번호 또는 메타데이터 장치 | 위 "카메라 연결 확인" 명령으로 번호 확인 |
| 캘리브레이션 RMS가 매우 큼 | 이상치 점 | 점별 오차를 보고 제외 (7절의 점6) |
| `det(R) < 0` (반사) | 좌우반전 처리 누락 | X 부호·depth 반전 확인 후 다시 수집 |
| ROS를 source한 터미널에서 import 오류 의심 | ROS의 `PYTHONPATH` | `env -u PYTHONPATH -u AMENT_PREFIX_PATH python 파일.py`로 비교 |
| `USB events thread - failed to set priority` | 경고 | 무시 |
| 창을 닫았는데 다음 실행에서 `RGB 카메라를 열지 못했습니다` | 창 닫기 버튼으로 닫으면 OpenCV 프로그램은 화면 없이 계속 실행된다 | 창은 `q`로 닫는다. `fuser -v /dev/video*`로 남은 프로그램을 찾아 종료 |
| 확대경 오류 `(-215:Assertion failed) !ssize.empty()` | 커서가 영상 밖(문구 패널)에 있을 때 확대경을 그림 | 2026-10-14 수정 (`_ui.magnifier`) |
| calibrate가 정합 계수가 없다며 시작하지 않음 | `config/pose.yaml` 없음 | 빠른 시작 4번 |
| `show_base_coords.py`가 "보정 파일 없음"으로 종료 | calibrate에서 s 저장을 안 함 | 빠른 시작 5번 |
| show_base_coords가 `demo` 설정이 없다고 안내 | 예전에 복사해 둔 config 폴더 | 저장소 `config/thresholds.yaml`의 `demo:` 절을 그 폴더에 복사 |
| RGB 카메라를 열지 못함 | calibrate·detect·경로 편집기 등이 카메라를 쓰는 중 | `fuser -v /dev/video*`로 확인 후 종료 |

설치 중 문제는 "설치 상세 → 설치 함정".

## Claude Code로 따라 하기 (팀원용)

VS Code에서 저장소 폴더를 열고 Claude Code에 아래처럼 요청하면 설치·확인을 같이 진행할 수 있다.

```text
pac_vision/README.md의 "빠른 시작"을 0번부터 순서대로 실행해 줘.
내 환경은 Ubuntu 24.04 + ROS 2 Jazzy, Python 3.12야.
- 명령마다 결과를 README의 "기대" 출력과 비교하고, 다르면 멈추고 알려 줘.
- sudo가 필요한 명령, config/ 안의 파일을 고치는 일, git commit/push는 실행 전에 물어봐.
- 카메라 화면에서 클릭하는 단계(5, 6번)는 내가 직접 할 테니 명령만 알려 줘.
- 막히면 README "문제 해결"과 CALIBRATION_GUIDE를 먼저 확인해 줘.
```

끝나면 [`CALIBRATION_GUIDE.md`](CALIBRATION_GUIDE.md) 끝의 "Jazzy 확인 체크리스트" 결과를 윤수에게 보내 준다.

## 측정 결과와 재현 방법

모든 측정 항목은 같은 형식으로 적는다: **무엇**(잰 값), **기준**(좌표계·기준점), **조건**, **데이터**(원자료 위치), **재계산**(명령과 기대 출력), **한계**, **상태**.

- 원자료는 [`data/measurements/`](data/measurements/)에 있다 (목록과 출처는 그 폴더의 README). 현장 캘리브레이션 세션(`data/sessions/`)은 git에 올리지 않는다.
- 재계산 명령은 카메라가 필요 없다. 모든 절을 한 번에: `python tools/recompute_measurements.py`.
  같은 수치를 `tests/test_measurements.py`가 테스트로 고정한다 (테스트 이름에 `raw`(원자료로 재계산) / `arithmetic`(산술 검산(측정 재현 아님))를 붙였다).
- 상태는 세 가지로 나눈다.
  - **원자료로 재계산**: 측정 스크립트가 저장한 원본 파일에서 다시 계산해 기록값과 같음을 확인.
  - **산술 검산(측정 재현 아님)**: README 표를 옮겨 적은 값으로 계산식만 다시 확인. 원 측정 로그가 없으므로 측정이 맞았다는 확인은 아니다.
  - **기록만**: 원자료나 재계산 수단이 없어 기록을 옮긴 것.
- 1-8절은 2026-07-15 ~ 07-25, 다른 카메라 설치 위치에서 쟀다 (Orbbec Astra Pro LeTMC-520, depth 640×480 uint16 mm, 실내·커튼, 삼각대 고정).
  10·11절은 2026-10-02, 12절은 2026-10-14, 지금 저장소의 스크립트로 쟀다.

### 1. 카메라 특성 (2026-07)

| 항목 | 내용 |
|---|---|
| 무엇 | 화각, depth 최소 거리, 잘 안 재지는 표면, 거리별 노이즈 |
| 기준 | depth 원본 (mm) |
| 조건 | 평판 타깃 중앙, 같은 점 30프레임 |
| 데이터 | `data/measurements/2026-07_noise_by_distance.csv` (요약값만) |
| 재계산 | 없음 |
| 한계 | 원 측정 로그가 남아 있지 않다. 정면 평판 기준이라 비스듬한 면은 다를 수 있다 |
| 상태 | 기록만 |

- 화각: 가로 58.6°, 세로 45.6° (OpenNI로 조회). depth 최소 측정 거리 약 0.6 m (그 안은 0).
- 잘 안 재지는 표면: 검은색(IR 흡수), 유리·금속·광택, 작고 얇은 물체, 물체 가장자리, 밝은 햇빛. 둥근 물체(물병)는 볼록면을 읽어 수 cm 어긋났다 (1.05 m → 0.98 m).
- 거리별 노이즈 (30프레임 표준편차): 0.8-1.5 m ±0.5-1.0 mm / 2.0 m ±23.5 mm (유효 28/30) / 3.5 m 유효 14/30 (불안정).

### 2. intrinsics

| 항목 | 내용 |
|---|---|
| 무엇 | fx, fy, cx, cy (depth 640×480) |
| 기준 | depth 광학 좌표계 |
| 조건 | 1절의 화각에서 계산 (렌즈 왜곡 보정 없음) |
| 데이터 | 화각 두 값 (위 1절, 기록값) |
| 재계산 | `python tools/recompute_measurements.py --section 2` |
| 한계 | 왜곡 보정이 없어 화면 가장자리는 오차가 커질 수 있다 |
| 상태 | 산술 검산(측정 재현 아님) (2026-10-13) |

```text
[2] fx = 320/tan(58.6°/2) = 570.2, fy = 240/tan(45.6°/2) = 570.9  (사용값 570)
```

사용값: fx = fy = 570, cx = 320, cy = 240 (`config/camera.yaml`). 역투영 (depth를 좌우반전한 뒤):

```text
Z = depth_mm/1000 - Z_OFFSET
X = (u - cx) * Z / fx      # 카메라 오른쪽 +
Y = (v - cy) * Z / fy      # 아래 +
```

### 3. 좌우반전 판정

| 항목 | 내용 |
|---|---|
| 무엇 | depth를 좌우반전한 뒤 X 부호가 맞는가 |
| 기준 | **물체를 실제로 카메라 오른쪽에 두면 X가 양수**. 화면이 어떻게 보이는지는 무시한다 |
| 조건 | 2026-07: 두 방식(아래) 모두 실측 / 2026-10-02: `check_camera.py`, 상자를 카메라 오른쪽에 둠 |
| 데이터 | `data/measurements/2026-10-02_poseA_points_run1.csv` (u 465, v 249 행) |
| 재계산 | `python tools/recompute_measurements.py --section 11` 의 첫 줄 |
| 한계 | 카메라 고유 성질이라 설치 자세가 바뀌어도 유효 |
| 상태 | 2026-10-02: 원자료로 재계산 (2026-10-13, `check_camera.py`가 저장한 CSV 원본) / 2026-07: 기록만 |

```text
[11] 좌우반전: 오른쪽 상자 X = +0.276 m
```

`prototype/measure_3d.py`는 depth를 뒤집지 않고 `X = -(u-CX)*Z/FX`, `yolo_3d.py` 등은 depth를 먼저 뒤집고 `X = +(u-CX)*Z/FX`.
두 방식은 1픽셀 이내로 같다. 이 패키지는 **뒤집고 + 부호** 하나로 통일했다. 2026-07 캘리브레이션(7절)에서 반사(`det R < 0`)가 나오지 않은 것도 같은 확인이다.

### 4. Z 원점 보정 (`Z_OFFSET`)

| 항목 | 내용 |
|---|---|
| 무엇 | depth 원점과 줄자 기준점의 차이 |
| 기준 | **카메라 하우징 앞면 → A4 평판 타깃 앞면** (줄자) |
| 조건 | 0.8 / 1.0 / 1.5 / 2.0 m, 타깃 중앙, 30프레임 중앙값, `Z_OFFSET` = 0으로 둔 `measure_3d.py` 복사본 (2026-07) |
| 데이터 | `data/measurements/2026-07_z_offset.csv` (README 표를 옮김) |
| 재계산 | `python tools/recompute_measurements.py --section 4` |
| 한계 | 원 측정 로그 없음 (표를 옮김). 정면 측정 |
| 상태 | 산술 검산(측정 재현 아님) (2026-10-13) |

| 실제 (m) | 측정 Z (보정 전) | 차이 | 표준편차 |
|---|---|---|---|
| 0.8 | 0.781 | −0.019 | ±0.5 mm |
| 1.0 | 0.982 | −0.018 | ±1.0 mm |
| 1.5 | 1.478 | −0.022 | ±1.0 mm |
| 2.0 | 1.976 | −0.024 | ±23.5 mm (유효 28/30) |

```text
[4] 차이 [-0.019, -0.018, -0.022, -0.024] m → 평균 -0.0208 m (사용값 -0.021), 0.8→2.0 m 측정 변화 / 실제 변화 = 0.9958
```

차이가 거리와 거의 무관하므로 스케일 오차가 아니라 원점 차이다 → `Z_OFFSET = −0.021 m`, 보정 후 ±3 mm. 스케일 오차는 0.4 %.
⚠️ **기준점을 반드시 같이 적는다.** 처음에는 기준점 없이 재서 +0.044~+0.048 m(`Z_OFFSET = 0.046`)가 나왔고 폐기했다. depth 원점은 하우징 앞면이 아니라 렌즈 안쪽의 광학 원점이다.

### 5. fx 검증

| 항목 | 내용 |
|---|---|
| 무엇 | 타깃을 옆으로 옮긴 실제 거리와 측정 X 변화량의 비 |
| 기준 | 카메라 광학 좌표계 X (변화량만 사용) |
| 조건 | 카메라 고정, 거리 약 1.0 m 유지, 옆으로 0.30 m 이동 (2026-07) |
| 데이터 | `data/measurements/2026-07_fx_check.csv` (README 본문을 옮김) |
| 재계산 | `python tools/recompute_measurements.py --section 5` |
| 한계 | 한 번만 쟀다. ΔX/실제 = 0.96 (보정식 `fx_new = 570 × 측정 변화량 / 실제 변화량`대로면 fx ≈ 547). 570은 화각 계산값(2절)과 일치한다. 이동 거리 측정 오차와 fx 오차를 구분할 기록이 없다. 배율 오차는 R·t가 흡수하지 못하므로 캘리브레이션의 A4 변 길이로 현장에서 확인한다 — 카메라 가로 방향 변만 일관되게 약 4 % 짧으면 fx를 다시 검토한다 |
| 상태 | 산술 검산(측정 재현 아님) (2026-10-13) |

```text
[5] ΔX 0.288 m / 실제 0.30 m = 0.960 → 보정식을 적용하면 fx 547 (사용값 570 유지)
```

결과: X +0.032 → +0.320 m (Z 1.007 → 1.003). fx 값은 새 설치 자세에서 다시 확인할 항목이다.

### 6. RGB ↔ depth 정합 오프셋 (2026-07)

| 항목 | 내용 |
|---|---|
| 무엇 | 같은 물리 지점의 RGB 픽셀과 depth 픽셀 차이 (dx, dy) |
| 기준 | 640×480 RGB → 좌우반전한 depth |
| 조건 | `prototype/align_check.py`, RGB 클릭 **직후** depth 클릭한 쌍만 유효, 여러 지점·거리 |
| 데이터 | 없음 |
| 재계산 | 없음 |
| 한계 | 원자료가 없다. 당시 RGB를 어떤 해상도로 받아 640×480으로 줄였는지 기록이 없다 (10절 참고) |
| 상태 | 기록만. 지금 설치 자세의 값은 11절 |

- 결과: dx ≈ +25 px (−9 ~ +40 흔들림), dy ≈ 0. 흔들림 원인: 클릭 정밀도, 시차(가까울수록 커짐).
- OpenNI 자동 정합(`set_image_registration_mode`)은 Astra Pro에서 쓸 수 없다 (RGB가 UVC라 OpenNI 밖).
- ⚠️ **dx·dy가 틀리면 R·t로 보정되지 않는다.** 위 "주의" 절 참고. 큰 밀림은 A4 변 검사에 일부 걸릴 수 있지만 수 px의 작은 밀림은 통과해
  물체 좌표의 일정한 오프셋으로 남는다. `calibrate.py --check`의 줄자 비교도 같은 경로라 A4 밖 외삽 오차(오프셋 제외)만 보여 준다.
  오프셋은 `detect_objects.py`의 물체 좌표를 줄자로 잰 위치와 비교해서만 확인할 수 있다.

### 7. 카메라 → 기준 좌표 변환 (R, t) (2026-07)

| 항목 | 내용 |
|---|---|
| 무엇 | `P_base = R @ P_cam + t`, 사용점 RMS, 검증용 점(계산에 안 쓴 점) 오차 |
| 기준 | 바닥에 표시한 임시 원점 (밝은 무광 상자의 한 점, 테이프 십자·X축 앞·Y축 왼쪽, 직각은 3:4:5) |
| 조건 | 대응점 9개(높이 섞음, 2.5 m 이내), 줄자로 기준 좌표, `measure_3d.py` 30프레임 중앙값으로 카메라 좌표 |
| 데이터 | `data/measurements/2026-07-25_calib_points.csv`, `2026-07-25_calib_holdout.csv` (README 표를 옮김. 사용한 7점은 `prototype/make_transform.py`에 적힌 값과 같음) |
| 재계산 | `python tools/recompute_measurements.py --section 7` (원본 `prototype/make_transform.py` 계산과 같음, `tests/test_kabsch.py`도 같은 값을 확인) |
| 한계 | 당시 설치 전용 (지금 설치에는 무효). 검증용 점의 출력값은 카메라 측정이라 원자료는 표뿐 |
| 상태 | 산술 검산(측정 재현 아님) (2026-10-13, numpy 2.2.6 / Python 3.10.12) |

| 점 | 기준 좌표 (X, Y, Z) m | 카메라 좌표 (X, Y, Z) m | σ (mm) | 사용 |
|---|---|---|---|---|
| 1 | (0.98, −0.485, 0.805) | (0.230, −0.750, 2.623) | 0.0 | 사용 |
| 2 | (−0.34, 0.46, 0.50) | (−0.639, −0.092, 1.375) | 1.6 | 사용 |
| 3 | (−0.628, 0, 0) | (−0.215, 0.470, 1.306) | 0.7 | 사용 |
| 4 | (0, 0.361, 0) | (−0.627, 0.241, 1.860) | 3.0 | 사용 |
| 5 | (0.40, −0.764, 0.78) | (0.506, −0.540, 2.121) | 0.0 | 사용 |
| 6 | (0.60, −0.41, 0) | (0.155, 0.266, 1.919) | 1.8 | 제외: 오차 52.5 cm, 측정·기록 오류로 추정 |
| 7 | (0.57, 0.584, 0) | (−0.903, 0.033, 2.360) | 5.5 | 사용 |
| 8 | (1.718, 0.226, 0.17) | (−0.604, −0.449, 3.552) | 6.2 | 제외: 3.5 m, 유효 14/30 |
| 9 | (−0.312, −0.422, 0.10) | (0.194, 0.295, 1.599) | 1.5 | 사용 |

```text
[7] 9점: RMS 19.38 cm, det(R) +1
[7] 점6 제외: RMS 4.17 cm, det(R) +1
[7] 점6·8 제외(채택): RMS 2.26 cm, det(R) +1
[7] 점3·6·8 제외: RMS 2.42 cm, det(R) +1
[7] 채택 R =
[[-0.08364 -0.31945  0.9439 ]
 [-0.99135 -0.0695  -0.11137]
 [ 0.10118 -0.94505 -0.31088]]
[7] 채택 t = [-1.72318 -0.03662  0.87503]
[7] 검증용 점 RMS: 3D 8.80 cm, 수평 5.73 cm
```

| 검증용 점 실제 (X, Y, Z) | 출력 (X, Y, Z) | 3D 오차 | 수평 오차 |
|---|---|---|---|
| (0.30, −0.40, 0) | (0.316, −0.405, 0.098) | 9.9 cm | 1.7 cm |
| (−0.20, −0.20, 0) | (−0.193, −0.262, 0.054) | 8.3 cm | 6.2 cm |
| (0.26, 0.34, 0.16) | (0.334, 0.354, 0.131) | 8.1 cm | 7.5 cm |

- 점3을 빼도 개선이 없으므로(2.42 cm) 점3은 정상이다. 카메라는 기준 원점에서 약 1.72 m 뒤, 0.88 m 위였다.
- 높이(Z)가 약점이다: 비스듬히 내려다봐 depth 오차가 기준 Z로 번지고, 높이 있는 대응점이 3개뿐이었다.
- `Z_OFFSET`은 카메라 Z 방향 이동이라 t에 흡수될 수 있으므로 두 번 빼지 않는다 (위 데이터는 `Z_OFFSET` 적용 상태로 수집).
- 원본 스크립트로도 재현할 수 있다 (저장 경로만 바꾼 복사본, 2026-10-01 확인, 출력 R·t·RMS가 위와 같음):
  `sed "s#/home/user/vision_test/cam2robot.npz#/tmp/cam2robot_rerun.npz#" prototype/make_transform.py > /tmp/mt.py && python /tmp/mt.py`

### 8. YOLO 정적 검증, 사람 (2026-07)

| 항목 | 내용 |
|---|---|
| 무엇 | 기준 좌표를 아는 자리에 선 사람의 검출 위치 오차 (수평) |
| 기준 | 7절의 기준 원점 |
| 조건 | `prototype/yolo_verify.py`, 사람(class 0)만, 몇 초씩 서 있기 |
| 데이터 | [`prototype/data/verify_log.csv`](prototype/data/verify_log.csv) (2026-07-25 12:56:43 ~ 12:57:18, 274행, 스크립트가 쓴 원본) |
| 재계산 | 없음 — 로그에 자리를 옮기는 구간이 섞여 있고, **안정 구간을 고른 시각 범위를 기록하지 않아** 아래 평균을 같은 값으로 다시 뽑을 수 없다 |
| 한계 | 박스 중심은 발이 아니라 몸통, depth는 옷 표면까지 거리 |
| 상태 | 기록만 (원 로그는 있음) |

| 위치 | 실제 (X, Y) | 측정 (X, Y) | 위치 오차 |
|---|---|---|---|
| A | (0, −0.30) | (0.033, −0.360) | 6.9 cm |
| B | (−0.40, 0) | (−0.349, −0.015) | 5.3 cm |
| C | (−0.40, −0.30) | (−0.358, −0.417) | 12.5 cm |

한 자리에서의 흔들림은 ±1 cm 이내. COCO에 있는 물체(bottle 39, cup 41 등)만 바로 검출된다. 멀리 있는 작은 물체는 거의 검출되지 않았다.

### 9. 카메라를 옮기면 다시 재야 하는 값

| 값 | 성격 | 카메라를 옮기면 |
|---|---|---|
| fx, fy, cx, cy | 카메라 고유값 | 유지 (5절 한계 참고) |
| `Z_OFFSET` | 카메라 고유값 | 유지 (한 점으로 재확인 권장) |
| depth 좌우반전, RGB 해상도 모드 | 카메라 고유 특성 | 유지 |
| RGB↔depth 정합 식 계수 (dx·dy) | 설치 자세·거리·화면 위치에 따라 변함 | 재측정 ("실행 상세 → 정합 계수 측정". 카메라 실물 1은 프리셋) |
| R, t | 설치 위치 | **재측정** |

### 10. RGB 해상도와 종횡비 (2026-10-02)

| 항목 | 내용 |
|---|---|
| 무엇 | Astra Pro RGB가 받는 크기, 640×480이 16:9 영상과 어떤 관계인가 |
| 기준 | 640×480 RGB 영상 |
| 조건 | `/dev/video2`, OpenCV(V4L2) 요청. 사람이 없는 정지 장면에서 1920×1080(MJPG)과 640×480을 연속 촬영, 정규화 상호상관(NCC) |
| 데이터 | `data/measurements/2026-10-02_rgb_mode_requests.csv` (요청 → 실제 크기, 터미널 출력을 옮김). **비교 영상은 보존되지 않았다** |
| 재계산 | 없음. 재측정(카메라 필요): `python tools/rgb_modes.py` — 같은 비교를 하고 영상을 `data/measurements/`에 저장한다 |
| 한계 | NCC 값은 측정 당시 출력의 기록이다. 첫 시도는 장면이 움직여(기준 NCC 0.729) 버렸다 |
| 상태 | 기록만 (재측정 도구는 있음) |

- 받을 수 있는 크기: 1920×1080(MJPG일 때만), 1280×720, 640×480, 320×240. 사이 크기를 요청하면 가까운 모드로 바뀐다 (예: 640×360 → 640×480).
- 640×480은 **4:3 화면 전체**다. 16:9 모드는 그 4:3 화면의 **위아래를 잘라낸 가운데 띠**다.

| 640×480 영상과 비교한 것 | NCC |
|---|---|
| 기준: 1920×1080 두 번 촬영끼리 (장면이 고정됐는지) | 0.998 |
| A 16:9 전체를 640×480으로 찌그러뜨린 것 (늘림) | 0.739 |
| B 16:9의 가운데 4:3을 잘라 줄인 것 (잘림) | 0.492 |
| **C 640×480의 가운데 640×360 띠 = 16:9 전체** (위아래 60 px씩 잘림) | **0.994** |

→ 이 패키지는 RGB를 640×480으로 받고, RGB→depth 배율은 1에서 시작한다. 16:9 화면(예: 경로 편집기의 1280×720) 픽셀은
"가로 비율로 축소, 세로는 같은 비율로 축소한 뒤 +60 px(640×480 기준)"로 640×480 픽셀에 대응시킨다 (`CameraModel.screen_to_ref`).

### 11. 좌우반전·Z·정합·노이즈, 한 설치 자세 (2026-10-02)

| 항목 | 내용 |
|---|---|
| 무엇 | (1) X 부호 (2) 상자까지 거리 vs 줄자 (3) RGB→depth 정합 dx·dy (4) 책상면 depth 노이즈 |
| 기준 | 카메라 광학 좌표계. Z 비교는 **하우징 앞면 → 상자 앞면** 줄자. 정합은 640×480 RGB 픽셀 → 좌우반전한 depth 픽셀 |
| 조건 | 위에서 비스듬히 책상을 보는 한 자세(이후 카메라를 옮김). `scripts/check_camera.py`, 상자 꼭짓점 쌍 20개(Z 0.86-1.11 m), 같은 점 30프레임 |
| 데이터 | `data/measurements/2026-10-02_poseA_pairs_run1.csv`(12쌍, 스크립트 원본), `..._pairs_run2_transcribed.csv`(8쌍, **터미널 출력을 옮김**), `..._points_run1.csv`, `..._tape.csv` |
| 재계산 | `python tools/recompute_measurements.py --section 11` |
| 한계 | 한 자세라 화면 세로 위치 v와 거리 Z가 함께 변해(상관 −0.90) dx·dy 변화의 원인(배율 차이 / 시차)을 구분할 수 없다. 화면 아래쪽 끝은 재지 않았다. 카메라를 옮겨 위치 식은 무효 |
| 상태 | 원자료로 재계산 (2026-10-13): 1회차 12쌍·점·노이즈는 CSV 원본. 2회차 8쌍은 터미널 출력을 옮긴 값이라 그 부분은 옮겨 적기가 맞다는 전제. 측정 자체는 1회 |

```text
[11] 좌우반전: 오른쪽 상자 X = +0.276 m
[11] Z: 상자 Z 1.101 m, 직선거리 1.107 m, 줄자 1.12 m (차이 -1.3 cm)
[11] 정합 20쌍: dx 17-30 (평균 23.1), dy 0-14 (평균 8.1), Z 0.86-1.11 m, u 27-523, v 68-347
[11] 잔차 RMS: 고정값 dx 3.7 / dy 3.9 px, 위치 1차식 dx 2.2 / dy 2.7 px, v·Z 상관 -0.90
[11] 거리 식: dx ≈ -4.3 -0.0052·u +28.3/Z (잔차 2.2 px)
[11] 책상면 아래쪽 끝(가까운 쪽)/가운데: Z 0.880 m, 픽셀 하나 30프레임 σ 0.0 mm (유효 30/30)
[11] 책상면 위쪽 끝(먼 쪽)/가운데: Z 1.149 m, 픽셀 하나 30프레임 σ 0.0 mm (유효 30/30)
```

- 좌우반전·Z·노이즈는 카메라 고유 성질이라 자세가 바뀌어도 유효하다.
- 정합: 같은 자리를 두 번 찍은 쌍의 차이는 ±1 px (반복성). 고정값 하나로는 dx 잔차 3.7 px, 위치의 1차식(a + b·u + c·v)으로는 2.2 px.
  이 20쌍 평균(dx 23, dy +8)은 다음 자세를 재기 전까지 쓴 임시값이었다. 지금 설정은 12절의 거리 식 계수다.
- 책상면 노이즈: 같은 픽셀 30프레임 표준편차가 가까운 쪽·먼 쪽 모두 0.0 mm. 흔들림이 depth 값의 단계보다 작다 **(새 프레임 읽기 확인됨)**:
  `read_frame()`은 OpenNI2 `oniStreamReadFrame`(새 프레임까지 기다림)을 부르고, `check_camera.py`는 프레임 번호(`frameIndex`)가 직전과 같으면 버린다.
  그래서 책상면 노이즈는 시간 표준편차보다 캘리브레이션 결과 화면의 **평면 맞춤 잔차(공간 노이즈)**로 본다. 물체 모서리 점은 5×5 중앙값 기준 최대 26 mm로 크다.

### 12. 한 설치 자세의 정합 식·캘리브레이션·외삽·fx·물체 좌표 (2026-10-14)

| 항목 | 내용 |
|---|---|
| 무엇 | (1) RGB→depth 정합 식 계수 (2) 캘리브레이션 2회와 반복성 (3) 재점검·이음매·A4 밖 외삽 오차 (4) fx 판정 (5) 물체 좌표 vs 테이프 표시점 |
| 기준 | base_ref (기준 종이 대각선 교차점 원점, +X 화살표, +Y 맞댄 종이 쪽). 정합은 640×480 RGB 픽셀 → 좌우반전 depth 픽셀, Z는 광학 Z(Z_OFFSET 적용 후) |
| 조건 | 카메라가 책상면 위 51 cm, 광축이 수평에서 약 21° 아래 (낮고 비스듬함). 광택 책상(평면 맞춤 영역 depth 유효 47-48 %). 기준 종이 = A4 두 장. 평면 모드, 정지 화면 30프레임 평균. YOLO yolo26n CPU |
| 데이터 | `2026-10-14_pose_pairs.csv`·`_pairs_run2.csv`(원본), `_a4corners.csv`, `_calib_rounds.csv`(세션에서 뽑음), `_check.csv`·`_objects.csv`(터미널 출력을 옮김), 1회차 세션 원본 `tests/data/sample_session.npz` |
| 재계산 | `python tools/recompute_measurements.py --section 12`, `python tools/fit_rgb_to_depth.py data/measurements/2026-10-14_pose_pairs.csv data/measurements/2026-10-14_pose_pairs_run2.csv` |
| 한계 | 한 설치 자세. 2회차 세션은 git 제외(CSV만). `--check`와 물체 좌표는 터미널 출력을 옮긴 값. 테이프 표시는 자로 종이 변을 연장해 붙임(±2-3 mm) |
| 상태 | 정합 계수·1회차 캘리브레이션·fx 판정: 원자료로 재계산 (2026-10-14). 2회차·`--check`·물체 좌표: 산술 검산(측정 재현 아님) |

```text
[12] 정합 첫 측정 14쌍 (Z ≤ 1.5 m): LOO 고정 dx 8.98, 거리 식 dx 4.96 / dy 1.31 px, 제외 규칙 적용 거리 식 dx 2.82 px
[12] 정합 두 측정 24쌍 → 맞춤 20쌍 (제외 [3, 4]): LOO 거리 식 dx 2.65 / dy 1.59 px, 고정 dx 6.24 / dy 2.21 px
[12] dx = +10.337 -0.020835·u +0.025303·v +13.064/Z,  dy = +13.039 -0.005141·u -0.032642·v +6.587/Z
[12] c–d 상관 -0.81 (d를 물리적 시차로 해석하지 않음), 직접 시차 [26.6] px·m
[12] 캘리브레이션 round1: base 0.7 mm, 변 오차 [2.6, -3.8, 0.4, -1.5] mm, 5점 RMS 1.6 mm
[12] 캘리브레이션 round2: base 0.7 mm, 변 오차 [3.2, -1.2, 1.8, 6.7] mm, 5점 RMS 2.7 mm
[12] fx 판정 (1회차): fx 570 가로 +0.50 % / 깊이 -0.63 %, fx 547 가로 +4.70 % / 깊이 -0.06 %
```

- **정합:** 이 자세에서는 A4 안에서도 dx가 약 19-31 px로 변해 고정값 하나로는 leave-one-out 6-9 px가 남는다 (낮은 카메라라 화면 안 거리 차가 큼).
  거리 식 a + b·u + c·v + d/Z로 2.65 / 1.59 px. 같은 화면 높이에서 거리만 다른 두 쌍의 dx 변화(직접 시차)는 26.6 px·m.
  v와 Z의 상관(−0.83)이 커서 맞춘 d는 c와 섞이므로 물리적 시차 계수로 해석하지 않는다. 다른 설치 자세 자료(11절)와 합친 계수는 [`config/pose_presets/README.md`](config/pose_presets/README.md) (카메라 실물 1 프리셋, 맞춤에 쓰지 않은 자세에서 A4 변 오차 최대 약 1 cm, 원자료는 기록만).
- **캘리브레이션:** 1회차는 모든 검사 통과. 2회차는 변 4-1이 허용(6.6 mm)을 0.1 mm 넘었다 (꼭짓점 세로 1 px 클릭 차). 두 회차의 R 차이 0.021°, t 차이 0.3 mm,
  같은 화면 점 8곳의 책상면 좌표 차이 0.2-0.3 mm (클릭 + 정지 화면 반복성). 최종은 1회차.
- **외삽점** (종이 변을 자로 연장해 테이프 십자, base m): a = 꼭짓점 3에서 +X 20 cm (0.3485, 0.315), b = 꼭짓점 2에서 +X 20 cm (0.3485, −0.105),
  c = 꼭짓점 1에서 −X 15 cm (−0.2985, −0.105), d = 꼭짓점 2에서 −Y 15 cm (0.1485, −0.255). 이음매 끝 = (±0.1485, 0.105).
- **재점검·이음매·A4 밖** (`--check`, 1회차 변환): 재점검(꼭짓점 4) 3.2 mm, 이음매 앞쪽 끝 3.4 mm. 이음매 뒤쪽 끝은 13 mm였는데, 칠한 점의 중심이 이음매 끝보다 종이 안쪽에 있었다 (표시 문제).
  A4 밖 외삽 오차(dx·dy 오프셋 제외)는 외삽점 a 3.6, 외삽점 c 8.5, 외삽점 d 6.2 mm. 외삽점 b는 클릭 주변 평면 맞춤이 책상 너머 바닥을 잡아 실패했고, 책상면 교점으로는 8.5 mm.
  종이 밖 점이 바깥쪽으로 2-3 % 길게 재지는 모양이지만, 모두 1 cm 안이고 자 연장·테이프 중심·클릭 1 px의 불확실성과 같은 크기라 원인을 정하지 않았다.
- **fx:** 가로 변이 짧게 재지는 패턴은 없다 (fx 570에서 가로 +0.5 / +0.8 %, 깊이 방향 −0.6 / +0.7 %). fx를 547로 바꾸면 가로 변만 +4.7 / +5.0 % 길어져 검사에 실패한다 → **fx 570 유지**.
  5절의 이동 측정(0.96)은 그 측정 쪽 오차로 본다.
- **물체 좌표** (detect, 컵 위에 종이 뚜껑, h 72 mm, 테이프 표시에 컵 바닥 가운데):

| 자리 | x 차이 | y 차이 (카메라 쪽 −) | z 차이 |
|---|---|---|---|
| 외삽점 a (선택값 2회 평균 / 30프레임 평균 2회) | +0.5 / +2.3 mm | −33.5 / −33.6 mm | +1.5 / −0.3 mm |
| 외삽점 c | +2.5 / +5.2 mm | −25.0 / −29.0 mm | 0.0 / +1.1 mm |
| (참고) 입구 열린 컵, 외삽점 a | −4.5 mm | −50 mm | −8 mm |
| (참고) 상자 48 mm, 외삽점 a, 알려진 위치로 만든 박스 | +8.1 mm | −46.4 mm | +1.7 mm |

  x·z는 3 mm 안이고 방향이 일정하지 않아 **dx·dy 오프셋은 없다**. y는 카메라 쪽으로 2.5-5 cm 치우친다 → 아래 "알려진 한계".
  윗면 띠를 4 mm로, 박스 위쪽 줄이기를 0으로 바꾼 설정도 같은 프레임으로 비교했으나 y가 컵 1.5-4.8 mm, 상자 15 mm만 줄어 기본값을 유지했다.
- **물체 중심 계산 방식 비교** (`2026-10-14_pose_center_methods.csv`, 같은 30프레임 평균 프레임 2회, y 차이 mm):

| 물체·자리 | top_band (지금, 기본값) | symmetric | bbox_top |
|---|---|---|---|
| 뚜껑 컵 외삽점 a (YOLO 박스) | −33.6 | −15.6 | **−2.8** |
| 뚜껑 컵 외삽점 c (YOLO 박스) | −33.8 | −12.2 | **+1.4** |
| 상자 긴 변 X (알려진 위치로 만든 박스, 상한) | −47.6 | +2.4 | −11.6 |
| 상자 긴 변 Y (같음, 참고) | −87.6 | −56.2 | −10.6 |

  - `top_band`: 지금 방식 (윗면 띠 점의 중앙값).
  - `symmetric`: 시선과 직각인 폭으로 깊이 크기를 추정하고, 앞 끝 + 폭/2를 중심으로 본다 (좌우 대칭 가정).
  - `bbox_top`: depth로 찾은 앞 끝과, YOLO 박스 위 끝 픽셀의 광선이 윗면 높이 평면과 만나는 먼 끝의 가운데.
  - **bbox_top이 컵의 카메라 쪽 치우침을 −34 mm → −3 ~ +5 mm로 줄였다** (x·z는 나빠지지 않음). 화면 끝 근처(외삽점 c)에서는 YOLO 박스 위 끝이 알려진 위치보다 3.5-3.7 px 위라 먼 끝이 시선 방향으로 약 15 mm 멀리 잡혔다 (외삽점 a에서는 0.3-0.4 px, 약 3 mm).
  - symmetric은 depth로 잰 폭이 실제보다 6-17 mm 작고, 비스듬한 시선에서 앞면 모서리가 앞 끝으로 잡혀, 상자 X의 +2 mm는 두 오차가 서로 상쇄한 결과다 (설계 때 예측 +35, 긴 변 Y는 −56). 그래서 권장하지 않는다.
  - 상자는 놓인 위치를 depth 앞면으로 확인했다 (기대보다 카메라 쪽 6 mm). 이 비교 동안 카메라 이동은 없었다 (기준 영상 대비 RGB 0.94 px, depth −0.8 mm).
  - **기본값은 `top_band`를 유지한다.** 바꿀지는 팀 안건이다 (아래 "알려진 한계").

## prototype/ 스크립트 (측정 1-8절에 쓴 원본)

`pac_vision` 모듈을 만들기 전에 단독으로 실행해 측정에 쓴 스크립트다. **위 측정 1-8절은 이 파일들로 얻었으므로 수정하지 않고 보존한다.**
모듈로 옮길 때는 `src/pac_vision/`에 새로 작성하고, 상수는 `config/`로 뺀다.

카메라 드라이버 준비와 연결 확인은 "설치 상세 → 카메라 드라이버"와 같다.

YOLO를 쓰는 스크립트(`yolo_3d.py`, `yolo_verify.py`)는 추가로 필요하다 (CPU 전용 예):

```bash
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install ultralytics
# ultralytics가 opencv-python을 함께 설치하면 contrib판과 cv2가 충돌하므로 하나만 남긴다
python -m pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python
python -m pip install opencv-contrib-python
```

- 모델 `yolo26n.pt`는 첫 실행 때 ultralytics가 자동으로 받는다 (COCO 80클래스, 학습 불필요).
- 팀 `path 편집/`은 `opencv-contrib-python==4.11.0.86`, `numpy<2`를 고정한다. 같은 가상환경에 설치한다면 그 버전을 따른다.
- 2026-10-01 이 컴퓨터의 버전: numpy 2.2.6, OpenCV 5.0.0 (ArUco 포함), ultralytics 8.4.104, torch 2.13.0+cpu, openni 2.3.0.
  측정 당시(2026-07)의 정확한 버전은 기록하지 않았다.

### 실행 전에 바꿔야 하는 상수 (컴퓨터마다 다름)

스크립트에 측정 컴퓨터의 절대 경로가 직접 적혀 있다. 실행하려면 복사본을 만들어 아래 값만 바꾼다 (원본은 그대로 둔다).

| 상수 / 경로 | 들어 있는 파일 | 원래 값 | 의미 |
|---|---|---|---|
| `REDIST` | measure_3d, align_check, yolo_3d, yolo_verify | `/home/user/ros2_astra_camera/astra_camera/openni2_redist/x64` | "설치 상세 → 카메라 드라이버" 2)에서 찾은 redist 폴더 |
| `RGB_INDEX` / `VideoCapture(2)` | yolo_3d, yolo_verify / align_check | `2` | Astra RGB 장치 번호 |
| `cam2robot.npz` 경로 | measure_3d, yolo_3d, yolo_verify (읽기) / make_transform (쓰기) | `/home/user/vision_test/cam2robot.npz` | R, t 저장 파일 |
| `verify_log.csv` 경로 | yolo_verify (쓰기, **덮어씀**) | `/home/user/vision_test/verify_log.csv` | 검증 기록 |

### 스크립트별 설명

공통 상수 (측정 당시 값):

```python
FX = FY = 570.0; CX, CY = 320.0, 240.0   # intrinsics (depth 640×480)
Z_OFFSET = -0.021                        # m, 아래 "Z 원점 보정" 참고
FLIP_DEPTH = True; DX, DY = 25, 0        # RGB 픽셀을 depth 픽셀로 옮길 때만 사용
```

| 파일 | 하는 일 | 입력 → 출력 | 조작 |
|---|---|---|---|
| `measure_3d.py` | depth 영상을 띄우고, 클릭한 점 주변 5×5의 0 제외 중앙값을 0.04 s 간격으로 30번 읽어 다시 중앙값 → 카메라 좌표로 역투영. npz가 있으면 기준 좌표도 출력 | 클릭 (u, v) → `카메라: X Y Z`, `base: X Y Z`, 유효 프레임 수, 표준편차(mm) | 클릭, `q` 종료 |
| `make_transform.py` | 코드에 적힌 대응점 7쌍(기준 좌표, 카메라 좌표)으로 강체 변환 R, t 계산 (Kabsch/SVD, `det(R)<0`이면 반사 보정). 사용점 RMS 출력 | → `cam2robot.npz` (R, t) | 카메라 불필요 |
| `align_check.py` | RGB(640×480으로 resize)와 depth(좌우반전) 창을 띄우고, 같은 지점을 각각 클릭하면 픽셀 차이 출력 | 클릭 두 번 → `dx, dy` | `f` depth 반전 전환, `q` 종료 |
| `yolo_3d.py` | RGB에서 YOLO 추적 → 박스 중심을 depth 픽셀로 옮김(`×(depth폭/RGB폭) + DX`) → 박스 크기에 비례한 영역(반경 `clip(0.15×짧은 변, 3, 25)` px) depth 중앙값 → 역투영 → R, t로 기준 좌표 | 화면 → 물체별 카메라·기준 좌표, 수평거리 | `q` 종료 |
| `yolo_verify.py` | `yolo_3d.py`와 같은 계산, 사람(class 0)만, 매 검출을 CSV에 기록 | → `verify_log.csv` (`time, cam_X..Z, base_X..Z, dist_xy`) | 아는 자리에 서서 몇 초 유지, `q` 종료 |

⚠️ **좌우반전 처리가 스크립트마다 다르다.** `measure_3d.py`는 depth를 뒤집지 않고 `X = -(u-CX)*Z/FX`,
`yolo_3d.py`·`yolo_verify.py`·`align_check.py`는 depth를 먼저 뒤집고(`cv2.flip(d, 1)`) `X = +(u-CX)*Z/FX`.
두 방식은 1픽셀 이내로 같은 결과다. 모듈에서는 **뒤집고 + 부호** 방식 하나로 통일한다.
경로 편집기도 화면을 반전(`mirror=True`)하므로, 섞으면 반전이 두 번 걸리거나 빠진다.

## 재현성 점검 (2026-10-14 기준)

| 항목 | 다시 실행해 확인 | 기록만 있음 |
|---|---|---|
| `pip install -e`, import (Ubuntu 22.04 / Python 3.10) | ✅ | |
| 카메라 없는 테스트 (`pytest`) | ✅ 87 passed (2026-10-14, numpy 2.2.6, ROS 경로 있음·없음 둘 다) / 29 passed (2026-10-03, numpy 1.26.4 + PyYAML 6.0.3 + pytest 9.1.1) | |
| 측정 2·4·5·7절 (intrinsics, Z_OFFSET, fx, R·t·RMS·검증용 점) | ✅ 산술 검산(측정 재현 아님): `tools/recompute_measurements.py` (README 표를 옮긴 값) | 측정 자체는 2026-07, 원 로그 없음 |
| 측정 3·11절 (좌우반전, Z, 정합 20쌍, 책상면 노이즈) | ✅ 원자료로 재계산 (2026-10-02 CSV 원본, 2회차 8쌍만 터미널 옮김) | 측정 자체는 1회 |
| 측정 12절 정합 계수, 1회차 캘리브레이션, fx 판정 | ✅ 원자료로 재계산 (쌍 CSV 원본, `tests/data/sample_session.npz`) | |
| 측정 12절 2회차, `--check`, 물체 좌표 | ✅ 산술 검산 (세션·터미널 출력을 옮긴 CSV) | 측정 자체는 1회 |
| 측정 1·6·8·10절 | | ✅ 원자료 또는 재계산 수단 없음 (8절은 원 로그는 있으나 구간 기록 없음, 10절은 `tools/rgb_modes.py`로 재측정 가능) |
| `calibrate.py --replay --no-gui` | ✅ 실제 세션 "같음" (테스트로 고정) | |
| `ros_pub.py` 발행 → `ros2 topic echo` 수신 (Humble) | ✅ 실측 좌표, frame_id `base_ref` (2026-10-14) | |
| 새 clone에서 빠른 시작 1–4번 (카메라 없음) | ✅ 2026-10-15, Ubuntu 22.04 / Python 3.10.12, 커밋된 파일만 받은 clone + 새 venv: 87 passed, 빠른 시작 2번 기대 출력 일치, `check_setup.py --no-camera` `[요약] ⚠️ 1개`(cam2base.yaml 없음). 사용자 site(`~/.local`)를 끈 상태(`PYTHONNOUSERSITE=1`, apt의 numpy 1.21.5 · OpenCV 4.5.4 · PyYAML 5.4.1 · pytest 6.2.5)에서도 87 passed | Jazzy(Python 3.12, numpy 1.26)는 미확인 |
| 정합 프리셋 `astra_unit1.yaml` | ✅ 커밋된 쌍 CSV에서 다시 만들어 값 일치 (2026-10-14, `config/pose_presets/README.md`) | 맞춤에 쓰지 않은 자세의 확인(A4 변 최대 약 1 cm)은 원자료 미커밋 |
| `show_base_coords.py --session tests/data/sample_session.npz --no-gui` | ✅ 빠른 시작 2번 기대 출력 (2026-10-14) | 카메라 클릭 동작은 확인만, 점별 오차 없음 |
| 캘리브레이션·물체 좌표 실측 | ✅ 한 설치 자세 (2026-10-14, 12절) | 맞춤에 쓰지 않은 다른 자세 1곳의 캘리브레이션(2026-10-14)은 기록만 (원자료 미커밋). 물체 좌표는 다른 자세 없음 |
| Ubuntu 24.04 / Jazzy / Python 3.12 | | 미확인 |

## 팀 코드 조사 결과 (2026-10-01)

`work/webcam-path-editor`(20873db), `urdf`(b2b4038), `work/gemini-live-voice`(921c46e)를 읽고 연결에 필요한 점만 정리했다. 자세한 약속은 [`INTERFACE.md`](INTERFACE.md).

- **경로 편집기의 손 입력:** `webcam.py`의 `hand_to_editor()`가 MediaPipe 정규화 좌표를 가상 좌표로 바꾼다. 경로 편집기 `DESIGN.md`에 "향후 depth 어댑터는 이 함수만 교체"라고 되어 있어, 이곳을 depth 연결 지점으로 제안한다.
- **화면 반전:** `prepare_frame()`이 프레임을 한 번 좌우반전하고, 손 인식·커서·경로 선택이 모두 반전된 화면을 쓴다.
- **화면 크기:** 기본 1280×720. Astra RGB는 640×480이므로 픽셀 변환 때 화면 크기를 받아야 한다.
- **좌표계 이름:** 경로 편집기는 `robot_base`와 `virtual_edit_frame`, URDF의 최상위 링크는 `base_link`. 통합 때 하나로 정해야 한다.
- **ROS 이름:** 팀은 `/manipulator/edit_command`(service), `/manipulator/edit_state`(topic)를 쓴다. 이 패키지는 지금 `/pac/objects`, `/pac/target_point`를 쓴다 (`config/frames.yaml`). `/manipulator/` 아래로 맞출지는 팀 통합 때 정한다 (`INTERFACE.md`).
- 참고: 경로 편집기 `ros_bridge.py`는 `EditCommand`의 `mode`, `deadline_unix_ns`, `revision`, `validation_status` 필드를 쓰는데,
  GitHub의 `work/gemini-live-voice` 브랜치 `EditCommand.srv`(2026-09-28)에는 이 필드가 없다. 아직 올라오지 않은 변경이 있는 것으로 보인다.

## 다음 작업

1. Ubuntu 24.04 / Jazzy 컴퓨터에서 빠른 시작 0–6번과 `CALIBRATION_GUIDE.md` 끝 체크리스트 확인.
2. `show_base_coords.py`로 점별 오차 측정 (알려진 점 클릭 → P·D와 기대 좌표 차이 표).
3. 정합 계수를 카메라 고유 계수로 `camera.yaml`에 넣고 `pose.yaml` 없이 시작하는 방식 (지금 프리셋 복사는 그 전까지의 임시 경로).
4. A4 변 검사 기준과 팀 허용 오차, base_ref ↔ 로봇 base_link 관계를 팀과 정함.
5. 좌표계·토픽 이름을 팀과 확정 (`INTERFACE.md`). 물체 좌표 계산 방식은 시나리오에서 다시 쓰게 되면 정함 ("알려진 한계").
6. (선택) static TF 발행과 RViz 확인.
