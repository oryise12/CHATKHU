# 핀치·쿠옹이·MuJoCo·실물 팔 시연: 설치부터 실행까지

이 문서만 따라 하면 다른 노트북에서도 같은 시연을 재현할 수 있게 썼다. 명령은 2026-10-23 연구실 노트북(Ubuntu 24.04.5, Python 3.12.3, ROS 2 Jazzy 설치됨, NVIDIA RTX 5060 + Wayland)에서 실행해 본 것이다. 그 노트북에서 확인하지 못한 단계는 **미확인**으로 따로 표시했다(맨 아래 "확인 범위").

- **빠른 길:** 저장소 맨 위 [README.md](../README.md)의 `./setup.sh`와 `./run_demo.sh`. 이 문서는 그 스크립트가 하는 일을 한 단계씩 풀어 쓴 것과 보정·문제 해결이다.
- 범위: 시뮬레이터만으로도 시연한다. 실물 팔(`--real-arm`, 7-1절)은 2026-10-28에 처음 실기로 돌렸다(가는 길·돌아오는 길 DONE, `manipulator_control_ws/CURRENT_STATE.md`).
- **현장 보정값은 저장소에 들어 있다** (2026-10-28 사용자 결정): `pac_vision/config/{cam2base,pose,markers,robot_pose,local}.yaml`은 지금 시연 배치(책상·Astra·5번 표지·로봇 판)의 결과다. 같은 배치로 옮기면 다른 컴퓨터에서도 그대로 쓴다. **카메라·표지·로봇을 옮겼으면 6절대로 다시 보정해서 덮어쓴다** (시뮬레이터는 시작할 때와 실행 중에 표지로 정합을 다시 확인하고, 어긋나면 3D 핀치를 막는다).
- 설계와 결정 근거: [PINCH_KHU_INTEGRATION_PLAN.md](PINCH_KHU_INTEGRATION_PLAN.md) (계획서 원문, 구현 중 코드에 맞춰 바로잡은 점은 이 문서 "계획과 달라진 점"에 있다).

## 0. 시연 흐름 (사용자가 보는 것)

1. 터미널 1에서 시뮬레이터: MuJoCo 3D 창 + **전체 화면 모니터**(브라우저, 2026-10-28 Figma 디자인 "아이로봇", 흰색): 실제 Astra 영상(경로 · 손가락 표시) 위에 왼쪽 메뉴 줄과 Chat KHU 대화 패널, 위쪽 4단계 진행 메뉴(경로설정 → 수정 → 검증 → 이동)와 상태 칩, 높이 그래프 버튼, 오른쪽 위 i-ROBOT 로고와 상태 배지(정합 · 쿠옹이 · 실물), 아래쪽 단계별 버튼. 예전 OpenCV 모니터 창은 `--ui cv`.
2. 터미널 2에서 쿠옹이. **"쿠옹아, 경로 수정할래"** → 시뮬레이터가 실제로 수정 모드가 된 뒤 쿠옹이가 "경로 수정 후 수정 완료를 말해 주세요."라고 안내한다. 음성 없이 할 때는 **E 키**.
3. 검지로 경로 안쪽 점을 0.4초 가리키면 노란 점으로 고르고, 엄지·검지로 집어 손을 움직이면 그 점과 주변이 손의 3D 이동만큼 따라온다. 손을 펴면 확정. 깊이를 못 읽으면 잡지 않고 이유를 화면에 보여 준다(높이 수정으로 몰래 바뀌지 않는다). 손 입력 경고는 말로 하지 않는다.
4. "쿠옹아, 이전으로" → 직전 수정 하나 되돌리기 (손·마우스·말 수정이 같은 기록).
5. **"수정 완료" / "이 경로로 할게"** → 쿠옹이가 **"검증 후 이동할까요?"** → **"어"**(호출어 없이) → 검증 → 실패면 빨간 점과 이유, 수정 모드로 복귀 → 통과면 "검증을 통과했어요. 이동을 시작할게요."와 함께 그 경로로 이동. "아니"면 수정 모드 그대로. 언제든 **X 키** 또는 "멈춰".
6. (별도 단계) **G 키** 또는 "쿠옹아, 이 물체로 목표 바꿔 줘" → 카메라가 본 책상 위 물체 위로 목표가 바뀌고 새 경로가 생긴다.

## 1. 저장소

```bash
git clone https://github.com/jaemin0319/26_project_manipulator.git      # 비공개: gh auth login 먼저. --depth를 쓰지 않는다
cd 26_project_manipulator
```

- 어느 폴더에 받아도 된다. 저장소 안의 모든 경로는 저장소 기준이다(실물 제어 `manipulator_control_ws/`, Astra 드라이버 `vendor/`).
- **얕은 클론(`--depth`)은 쓰지 않는다:** 실물 제어의 모델 사본은 이 저장소의 커밋(`model.UPSTREAM_COMMIT`)에 묶여 있고, 시험이 그 커밋과 지금 모델을 비교한다.
- 2026-10-28부터 `main` 하나만 둔다(그 전 브랜치들은 정리됨).
- 아래 명령의 `$REPO`는 받은 저장소 폴더다: `export REPO=$(pwd)` (저장소 폴더에서).

## 2. 시스템 패키지 (한 번, 관리자 비밀번호 필요)

```bash
sudo apt install -y git python3.12-venv python3-pip libportaudio2 fonts-noto-cjk x11-utils x11-xserver-utils \
                    v4l-utils mesa-utils libopenni2-dev
```

| 패키지 | 왜 |
|---|---|
| python3.12-venv, python3-pip | 가상환경. Ubuntu 기본 파이썬에는 ensurepip가 없다 |
| libportaudio2 | 마이크·스피커(sounddevice). 없으면 쿠옹이 음성 모드가 `PortAudio library not found`로 멈춘다 |
| fonts-noto-cjk | `--ui cv` 모니터 창 한글 (웹 모니터는 저장소의 Pretendard 글꼴) |
| x11-utils, x11-xserver-utils | 창 배치, GUI 스모크 시험(xwininfo, xwd) |
| v4l-utils, mesa-utils | 카메라·그래픽 확인(v4l2-ctl, glxinfo) |
| libopenni2-dev | OpenNI2 (Astra 깊이). Orbbec 드라이버는 3절에서 따로 받는다 |

관리자 권한이 없어 `python3.12-venv`를 못 깔 때(이 노트북에서 실제로 쓴 방법): `python3 -m venv --without-pip .venv`로 만든 뒤 PyPI의 pip wheel(sha256 확인)로 `python <pip.whl>/pip install <pip.whl>` 해서 pip를 넣는다.

## 3. Astra 드라이버와 USB 권한

`./setup.sh`가 드라이버를 받고, `./setup.sh --system`이 udev·그룹까지 한다. 손으로 할 때:

```bash
# 드라이버 파일만 씀 (빌드 안 함). 고정 커밋 f7e71d9 (2026-10-23에 받은 것)
mkdir -p vendor/ros2_astra_camera && cd vendor/ros2_astra_camera && git init -q
git remote add origin https://github.com/orbbec/ros2_astra_camera.git
git fetch --depth 1 origin f7e71d9ce806e788cb48d8580aac2c778fba4214 && git checkout FETCH_HEAD && cd ../..
# 설정은 $PINCH_KHU_ROOT/vendor/ros2_astra_camera/astra_camera/openni2_redist/x64 를 가리킨다 ($PINCH_KHU_ROOT = 저장소 폴더)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-orbbec.rules
sudo udevadm control --reload-rules && sudo udevadm trigger      # 그다음 카메라를 뽑았다 다시 꽂는다
groups | grep -q plugdev || echo "plugdev 그룹이 아님: sudo usermod -aG plugdev $USER 후 다시 로그인"
```

- 우분투 `libopenni2-0`의 드라이버는 PS1080(키넥트류)뿐이라 Astra에는 Orbbec redist(`liborbbec.so`)가 필요하다.
- 드라이버를 다른 곳에 두었으면 `pac_vision/config/local.yaml`의 `camera.openni.redist`를 그 폴더로 바꾼다.

## 4. 파이썬 환경 세 개 (시뮬레이터 / 음성 / 실물 제어)

`./setup.sh`가 세 환경을 lock 파일의 정확한 버전으로 만들고 전체 시험까지 돌린다. 아래는 그 내용이다.

ROS 2를 `~/.bashrc`에서 source하면 `PYTHONPATH`에 ROS 패키지가 섞인다. 각 `env.sh`가 그 경로를 빼고 자기 가상환경을 켠다. 환경을 섞지 않는다(시뮬레이터는 numpy 1.26, 음성은 numpy 2.x).

```bash
cd mujoco_sim                                 # 저장소 폴더에서
python3 -m venv .venv
source scripts/env.sh                         # ROS PYTHONPATH 제거 + .venv 켜기
pip install -r requirements.lock              # mujoco 3.15.0, numpy 1.26.4, opencv-python 4.11
python -m pinch_sim.build_scene               # 로봇 장면 생성 (저장소의 생성본과 같아야 함: git status 깨끗)
pip install --no-deps mediapipe==0.10.21      # 이 두 줄 그대로 (그냥 깔면 numpy 2가 들어와 깨진다)
pip install -r requirements-hand.txt          # "mediapipe requires jax / opencv-contrib" 경고는 정상
pip install -r requirements-astra.txt         # pac_vision(-e) + openni 2.3.0
python -m pytest -q                           # 174 passed (약 4분, 실물 연결 e2e 포함: 실물 제어 환경이 먼저 있어야 함)
(cd ../pac_vision && python -m pytest -q)     # 131 passed
(cd "../path 편집" && python -m pytest -q)    # 110 passed
deactivate
cd ..

python3 -m venv .venv-voice
.venv-voice/bin/pip install -r voice/requirements.lock
(source voice/env.sh; python -m pytest -q voice/tests)    # 16 passed

cd manipulator_control_ws
python3 -m venv .venv && source scripts/env.sh
pip install -r requirements.lock && python -m pytest -q   # 152 passed (가짜 서보)
```

ChatKHU 키는 2026-10-28 저장소 주인 결정으로 저장소의 `voice/chatkhu_key.env`에 있다(비공개 저장소). 다른 키를 쓰려면 개인 파일에 넣는다. 개인 파일이 있으면 그쪽이 먼저다(값을 터미널 기록에 남기지 않도록 편집기로 넣는다):

```bash
mkdir -p ~/.config/pinch_khu && install -m 600 /dev/null ~/.config/pinch_khu/env
nano ~/.config/pinch_khu/env          # 한 줄: BAZE_API_KEY=발급받은_키
```

`source voice/env.sh`가 개인 파일 → 저장소 파일 순서로 읽는다(개인 파일 권한이 600이 아니면 경고). 키가 없으면 쿠옹이는 `환경변수 BAZE_API_KEY가 비어 있습니다`로 멈추고 키 값은 출력하지 않는다.

마이크 입력 크기를 확인한다 (키 없이, 소리는 저장하지 않음):

```bash
source voice/env.sh
python -m voice --list-mics          # 마이크 목록
python -m voice.mic_check            # 배경 잡음·DC·잘림. 잘림이 0 %가 아니면 아래 amixer
python -m voice.mic_check --echo     # 스피커 → 마이크 에코 (1초 삐 소리)
amixer -c 1 sset 'Internal Mic Boost' 0     # 노트북 내장 마이크 증폭 낮추기 (카드 번호: arecord -l)
```

2026-10-23 이 노트북(ALC257 내장 마이크)은 처음에 Internal Mic Boost +30 dB와 Capture +30 dB가 모두 최대라 입력의 28 %가 잘리고 DC가 +0.31이었다(음성 인식이 될 수 없는 상태). `Internal Mic Boost 0`으로 낮춘 뒤 배경 −37 ~ −45 dBFS, 잘림 0 %. 이 설정은 사용자 권한의 ALSA 믹서 값이라 재부팅·오디오 설정 앱에서 다시 바뀔 수 있으니 시연 전에 `mic_check`로 다시 본다.

## 5. 카메라 확인

```bash
cd "$REPO/pac_vision" && source ../mujoco_sim/scripts/env.sh
lsusb | grep -i 2bc5                                    # Orbbec 장치
for d in /sys/class/video4linux/video*; do echo "$d: $(cat $d/name)"; done
python scripts/check_setup.py                           # 보정 전에는 pose.yaml·cam2base.yaml ⚠️가 정상
```

장치 번호를 어디에도 고정하지 않는다. 시뮬레이터 `--video auto`와 pac_vision 스크립트는 이름(`camera.yaml`의 `rgb.name_match: Astra`)으로 찾는다. 같은 이름의 노드가 둘이면 프레임이 나오는 쪽을 쓴다.

**카메라를 위아래로 뒤집어 달았으면**(거치대에 거꾸로 매단 경우) `pac_vision/config/local.yaml`의 `camera:` 아래에 `rotate_180: true`를 넣는다. RGB와 깊이를 둘 다 180° 돌려 읽어 화면·클릭·손 인식이 바로 선다. 정합 계수는 돌린 좌표의 값이어야 하므로, 돌리기 전에 잰 `pose.yaml`(또는 프리셋)은 `python tools/rotate_pose_180.py [--from <파일>]`로 바꾼다. 돌린 뒤에 정합 쌍을 새로 재면 그대로 맞다. 보정(`cam2base.yaml`)은 회전 설정과 함께 저장되고, 시뮬레이터는 회전 설정이 다른 보정을 거부하며 영상도 그 설정대로 돌린다.

카메라는 **한 프로그램만** 연다. 시뮬레이터와 pac_vision 스크립트가 같은 잠금(`$XDG_RUNTIME_DIR/pac_camera.lock`)을 써서, 다른 쪽이 쓰는 중이면 누가 쓰는지 알려 주고 열지 않는다.

## 6. 보정 (카메라를 놓거나 옮길 때마다)

순서: ① 정합(RGB↔depth) → ② A4 보정 → ③ 고정 표지로 독립 확인 → ④ 로봇 위치. 자세한 화면 안내는 `pac_vision/CALIBRATION_GUIDE.md`.

### ① RGB↔depth 정합 (이 카메라, 이 설치 자세)

```bash
python scripts/check_camera.py          # RGB 창 클릭 → depth 창 클릭 = 한 쌍. 화면 여기저기 20쌍쯤, q로 끝 (쌍 CSV 저장)
python tools/fit_rgb_to_depth.py <저장된 쌍 CSV> --write-pose     # config/pose.yaml (git 제외) 생성
```

- 팀의 `pose_presets/astra_unit1.yaml`은 "카메라 실물 1" 전용이고, 그 카메라로도 다른 설치 자세에서 A4 변 오차가 약 1 cm라 변 검사(5 mm)를 못 넘었다는 기록이 있다(`pose_presets/README.md`). 기본값으로 복사하지 않는다.

### ② A4 보정

```bash
python scripts/calibrate.py --layout aruco   # ArUco 보정판(권장): 배치 점검 → Enter → 마커 4개 자동 검출 → 검사 5가지 → 모두 ✓이면 s
python scripts/calibrate.py --layout single  # 빈 A4 한 장: 꼭짓점 1 좌상 → 2 우상 → 3 좌하 → 4 우하 클릭
python scripts/calibrate.py                  # 팀 기본(A4 두 장): 원점·꼭짓점 1~4 클릭
```

- 보정판 인쇄물: `python tools/make_aruco_board.py ~/aruco_board.pdf` (A4 가로, 실제 크기 100 %). 놓는 법과 근거는
  [`pac_vision/CALIBRATION_GUIDE.md`](../pac_vision/CALIBRATION_GUIDE.md) 5-2절. 손 클릭은 2026-10-26에 3–5 px 오차로 변 검사를
  자주 넘겼다. 보정판은 그 단계를 자동 검출로 바꾼다.

- `cam2base.yaml`에 결과와 함께 **카메라 신원**(장치 이름, USB 포트 by-path)과 그때의 카메라 설정 사본이 들어간다.
- 실패한 결과를 `s s`로 강제 저장하면(`passed: false`) 시뮬레이터가 거부한다. 리허설 때만 `--allow-unverified-calib`(겹침 미검증 배지, 3D 핀치 꺼짐).

### ③ 고정 표지로 독립 확인 (보정에 쓰지 않은 점)

`passed: true`는 보정 순간 A4 5점이 맞았다는 뜻일 뿐이다. 카메라가 밀리거나 돌았는지는 고정 표지로 본다.

```bash
cp config/markers.example.yaml config/markers.yaml
python scripts/check_markers.py --sheet ~/markers_sheet.png     # 인쇄용 (기본 12 cm 표지 2개, A4 한 장)
```

1. 100 % 크기로 인쇄하고 자로 한 변이 `size_m`인지 확인한다.
2. 보정 종이 밖, 로봇·손이 가리지 않는 책상 위 두 곳 이상에 평평하게 붙인다. 화면에서 한 변이 30 px 이상이어야 한다(비스듬히 보면 짧아진다).
3. 각 표지 **중심**의 base_ref 좌표(m, A4 원점, +X = A4 화살표 방향, +Y = 왼쪽)를 줄자로 재서 `config/markers.yaml`에 적는다.
4. 확인:

```bash
python scripts/check_markers.py         # 표지별 이미지 오차(px)·책상면 오차(mm), 결과는 config/markers.check.yaml (git 제외)
```

- 허용오차 출발값: 8 px, 10 mm(`markers.yaml`의 `max_pixel_px`, `max_desk_mm`). **현장에서 카메라를 건드리지 않고 5회 이상 잰 분포**를 보고 정한다(아래 8절 시험 A).
- 시뮬레이터는 같은 폴더의 `markers.yaml`로 시작할 때와 0.5초마다 다시 확인한다. 두 번 연속 어긋나면 "카메라가 움직였어요 - 다시 보정하세요"(빨간 배지), 표지가 5초 넘게 안 보이면 "미검증". **초록(확인됨)일 때만** 실제 영상 겹침을 검증된 것으로 보고 3D 핀치·카메라 목표를 허용한다. 그렇지 않으면 마우스·말로 고치는 대체 시연이 된다.
- `markers.yaml`이 없으면 "정합 미검증: 기준 표지 설정 없음" — 3D 핀치는 꺼진다.
- **지금 현장 설정 (2026-10-28, 사용자 결정):** 표지 하나, 4x4 **5번 50 mm**(`~/Downloads`의 5–8번 시트에서 잘라 씀). 중심 (−0.115, 0.065, 0) = 로봇 판 앞 왼쪽 모서리 (−9, 9) cm에 표지 모서리를 맞춘 자리(판 바깥, 책상 위), `min_visible: 1`. 표지가 하나라 그 표지를 축으로 한 회전은 못 잡고, 5초 넘게 가리면 3D 핀치가 멈춘다. 이전 0·1번(12 cm, 붙인 적 없음)은 `markers.yaml.bak`.

### ④ 로봇 위치 (보정 원점 = 로봇이라고 가정하지 않는다)

로봇 바닥 중심(base_link 원점)의 base_ref 좌표와 로봇 앞이 +X에서 반시계로 몇 도인지 자로 재서 `pac_vision/config/robot_pose.yaml`(git 제외)에 적는다:

```yaml
x_m: -0.150      # 로봇 바닥 중심, A4 원점 기준 (m)
y_m: 0.000
yaw_deg: 0.0     # 로봇 앞(그리퍼가 뻗는 쪽)이 +X에서 반시계로
measured: 2026-10-23 줄자
```

시뮬레이터는 `--robot-pose X,Y,YAW`가 있으면 그것을, 없으면 이 파일을 쓴다(시작할 때 어느 것을 썼는지 출력). 변환:

```
p_cam  = Z · K⁻¹ [u_d, v_d, 1]           (u_d, v_d) = RGB 픽셀의 depth 픽셀 (pose.yaml 정합), Z = depth_mm/1000 − z_offset
p_base = R · p_cam + t                    cam2base.yaml (base_ref: A4 원점, +X 앞, +Y 왼쪽, +Z 위, 책상 z = 0)
p_world = R_z(90° − yaw) · (p_base − (x, y, 0))   MuJoCo world (x, y = 로봇 바닥 원점, 로봇 앞 = +Y; z = 책상면)
```

확인: 시뮬레이터를 켜고 모니터의 A(시작점) 표시가 실제 로봇 TCP 근처에 오는지, 기준축이 종이 원점·화살표와 겹치는지 본다.

**로봇 판 (2026-10-28):** 로봇은 180 × 180 × 20 mm 판 위에 선다(판 가운데 = J1 축, 앞 변 = 종이의 로봇 쪽 변, 옆 변 = ±9 cm 선). 시뮬레이터 world z = 0은 그대로 **책상면**이고(카메라 보정·목표·카메라 모두 이 기준), 로봇만 0.02 m 올라가 있다(`configs/demo.json`의 `mount`, 장면에 회색 판 `base_plate`). 관절각은 같아서, 같은 자세의 TCP가 전보다 20 mm 높다(ready TCP z 0.27). 바닥 여유 검사는 책상면과 판 둘 다로 잰다. 실물 제어 프로그램은 계속 로봇 바닥(= 판 윗면) 기준이다.

실물 팔(7-1절)은 모델 −Y 쪽에서 일하므로 `yaw_deg: -90`, `y_m: 0.003`이다(J1 축이 base_link 원점보다 모델 +Y로 2.984 mm).

## 7. 실행 (터미널 두 개)

**터미널 1: 시뮬레이터**

```bash
cd "$REPO/mujoco_sim" && source scripts/env.sh

# (가) Astra 전부: 실제 영상 + 손(가리키기·3D 핀치) + 음성 + 카메라 목표(G)
python -m pinch_sim.app --calib ../pac_vision/config/cam2base.yaml --video auto --hand --depth --voice-gateway \
  --start 0,0,0.25 --goal 0,0.20
#   (--robot-pose를 쓰려면 음수는 = 로 붙인다: --robot-pose=-0.15,0,0)

# (나) 카메라 없이: 시뮬레이터 카메라 화면 + 마우스 + 음성
python -m pinch_sim.app --target-r 0.275 --target-deg 10 --voice-gateway

# (다) 깊이 없는 웹캠 손 입력 = 명시적 2D 모드 (높이만, 화면에 "2D 높이 모드"), 흰 바탕
python -m pinch_sim.app --target-r 0.275 --target-deg 10 --canvas --hand 0 --voice-gateway
```

`voice gateway: .../voice.sock` 줄이 뜬 뒤 터미널 2를 켠다.

**모니터 (`--ui web`, 기본):** 시뮬레이터가 127.0.0.1의 빈 포트에 작은 웹 서버를 열고(실행마다 새 토큰), 전용 프로필의 Chrome 창(없으면 Firefox)을 전체 화면으로 띄운다. 그 창을 닫으면 시뮬레이터도 끝난다. `--browser none`이면 창을 띄우지 않고 주소를 출력한다. 화면 요소는 `mujoco_sim/src/pinch_sim/web/`(HTML/CSS/JS, 빌드 없음), 무엇을 보여 줄지(단계 · 칩 · 대화상자 · 알림 카드)는 `webui.py`가 정한다.
- 영상 위 마우스 끌기 = 경로 수정(예전과 같음), 키(N S V Y H X U R O E G Q)도 그대로 받는다.
- 진행 메뉴 칸을 누르면 그 단계를 시작한다(경로설정 = 새 목표, 수정 = 수정 모드, 검증 = 선택 + 검증, 이동 = 실행).
- 아래 버튼: 수정 모드면 되돌리기 · 다시하기 · 처음 경로 · **수정 완료**("검증 후 이동할까요?" 확인 뒤 검증하고 통과하면 이동), 검증 통과면 **실행하기**, 이동 · 검증 중에는 **정지**(X와 같음).
- 쿠옹이가 앱 질문("검증 후 이동할까요?", "계속 갈까요, 돌아갈까요?" 등)을 하면 대화 패널에 답 버튼이 붙는다(음성 명령과 같은 처리기 · 같은 검사).
- 검증 실패는 이유별 대화상자(충돌 위험 감지 · 관절 범위 초과 · 작업 범위 초과 · 경로 검증 실패), 거절 · 손 입력 경고는 오른쪽 알림 카드로 보인다.
- 저장된 경로 · 최근 대화 · 새 대화 · 설정 · 글자 입력창은 디자인만 있고 누르면 "준비 중"이라고 알린다.

**터미널 2: 쿠옹이**

```bash
cd "$REPO" && source voice/env.sh
python -m voice --sim             # 마이크
python -m voice --sim --text      # 키보드로 문장 입력 (같은 ChatKHU, 같은 함수)
python -m voice --list-mics       # 마이크 목록, --mic 번호로 고름
```

예전 ROS 쿠옹이(`manipulator_voice`, work/integration-all 브랜치)가 실행 중이면 새 쿠옹이는 시작하지 않는다. 시뮬레이터도 두 번째 음성 클라이언트는 거부한다(같은 소켓·같은 호출어 충돌 방지).

### 조작

| 하고 싶은 것 | 말 (쿠옹이) | 키 / 버튼 (모니터) | 손 / 마우스 |
|---|---|---|---|
| 수정 모드 켜기 | 쿠옹아, 경로 수정할래 | E | 마우스 끌기도 수정 모드를 켠다 |
| 점 고르기 | (고른 뒤) 쿠옹아, 여기 위로 조금 | — | 검지 0.4초 가리키기 / 클릭 |
| 점 옮기기 | 여기 위로 2센티, 가운데 왼쪽으로 많이 | — | 핀치로 집어 옮기기(3D) / 마우스 끌기(높이) |
| 되돌리기 | 쿠옹아, 이전으로 | U (R 다시, O 처음 경로) | — |
| 검증 → 자동 이동 | 쿠옹아, 수정 완료 → (검증 후 이동할까요?) 어 | 아래 **수정 완료**(또는 검증 후 이동) → 검증 후 이동, 진행 메뉴의 검증·이동 칸, 키 S → V → Y | 쿠챗의 답 버튼 |
| 정지 | 멈춰 (호출어 없이도) | **X** (확실한 정지 수단) | — |
| 카메라 물체를 목표로 | 쿠옹아, 이 물체로 목표 바꿔 줘 (여러 개면 검지로 가리킨 채) | G | — |
| 새 목표 / 홈 | 새 목표 / (도착 후 질문에 답) | N / H | — |

호출어 규칙: "쿠옹아"를 부른 뒤 30초 동안 호출어 없이 말해도 된다. 이 시간은 명령이 실제로 실행되거나 손으로 고칠 때만 늘어난다. 쿠옹이(앱)가 질문하면("계속 갈까요, 돌아갈까요?") 그 질문의 답 **한 번**은 호출어 없이 받는다. 답이 될 수 있는 명령만 실행하고(예: 계속 가 / 돌아가), "아니"나 엉뚱한 말에는 아무것도 움직이지 않는다. 멈춤은 언제나 받는다.

## 7-1. 실물 팔로 실행 (`--real-arm`, `manipulator_control_ws/` 연동)

시뮬이 검증한 궤적을 실물이 **같은 시간표로** 따라간다. 실물은 저장소 안 `manipulator_control_ws/`의 제어 프로그램(별도 venv)이 맡고, 시뮬은 소켓으로 궤적을 보낸다. `./run_demo.sh real`(검사만: `check`)이 아래 순서대로 창을 띄운다.

**준비 (한 번)**
- 로봇 캘리브레이션: `python -m manipulator_control calibrate` (그 작업공간 README). 12 V 전원을 다시 켰으면 다시 한다.
- `pac_vision/config/robot_pose.yaml`: `x_m: 0.0`, `y_m: 0.003`, `yaw_deg: -90`.
  - 실물은 모델 −Y 쪽에서 일한다. 실물 HOME이 시뮬 ready의 좌우 반대이고, 보정 자세에서 아래팔이 종이 쪽을 가리킨다.
  - yaw가 −90이 아니면 `--real-arm`이 거부한다.
- 카메라 보정(6절)은 그대로 쓴다.

**실행 순서**
1. 터미널 A (실물, 팔 옆에서, 12 V 스위치에 손):
   ```bash
   cd manipulator_control_ws && source scripts/env.sh
   python -m manipulator_control calibration-status   # 저장된 보정을 쓸 수 있는지 (읽기 전용)
   python -m manipulator_control pinch                # 보정 검증 → HOME 미리보기 y → HOME → "시뮬 연결 대기"
   ```
2. 터미널 B (시뮬):
   ```bash
   cd mujoco_sim && source scripts/env.sh
   python -m pinch_sim.app --real-arm --calib ../pac_vision/config/cam2base.yaml --video auto --hand --depth --voice-gateway
   ```
3. 터미널 A에 "시뮬 연결됨 … y = 무장"이 뜨면 실물 주변을 확인하고 **y**를 누른다. 이후로는 시뮬 승인(Y 키, "수정 완료" → "어")이 실물을 움직인다.
4. 터미널 C: 쿠옹이 (7절과 같음).

**시뮬만 쓸 때와 다른 점**
- 시뮬 팔이 실물 측정 자세를 그대로 보여 준다(물리 계산 없음). 모니터 아래에 "실물: ARMED …" 배지가 뜬다.
- 시작 자세, 가동 범위, 목표 위치
  - 시작 자세는 실물 HOME, 가동 범위는 실물 `robot.yaml`을 시뮬이 직접 읽는다.
  - 목표는 로봇 −Y 쪽 r 0.25–0.30 m, θ −15°~55°이다. J1 = 0이 θ ≈ +20°이다.
- 속도는 시뮬과 같다.
  - 관절 최대 21.4°/s(= 22.5 × 0.95), 가속 1.2 rad/s² 이하.
  - 실물 쪽 `--speed-scale 0.5`를 주면 같은 경로를 절반 속도로 간다.
- V·Y는 실물이 무장(ARMED) 상태이고 상태 정보가 최신일 때만 된다. 실물이 거부하면(예: 시작 자세 차이) 이유가 뜨고 경로를 다시 만든다.
- 실물도 받은 궤적을 다시 검사한다(가동 범위·제동점·속도·가속·모델 충돌). 시뮬 결과를 믿지 않는다.

**정지**

| 수단 | 결과 |
|---|---|
| 시뮬 X / "멈춰" | 실물이 감속 정지하고 무장은 유지된다(이어 가기·되돌아가기 가능). 시뮬은 실물이 정지를 확인할 때까지 "실행 중"이다 |
| 터미널 A s/space, 패널 ■ | 정지하고 무장이 풀린다(다시 y) |
| 시뮬 종료 또는 시뮬 멈춤(이동 중 1 s 넘게 응답 없음) | 실물이 스스로 감속 정지하고 연결 대기로 간다 |
| 12 V 스위치 | 유일한 하드웨어 차단이다(팔이 처진다) |

PC와 U2D2 사이 통신이 끊기면, 서보는 마지막 목표까지 가서 멈춘다. 마지막 목표는 궤적에서 최대 0.25 s 앞(5.6° 이하)이다.

**첫 실기 순서 (권장)**
1. `python -m manipulator_control pinch --check-only`로 시나리오를 한 바퀴 돈다. 모든 궤적이 실물 검사를 통과하는지 확인한다(이동 없음).
2. 실제로 실행한다. 손은 12 V 스위치에 두고, 각 정지 수단을 한 번씩 써 본다.
3. 실물 쪽 로그(`logs/sessions/*_pinch/events.jsonl`)의 `traj_done` metrics(`max_tracking_deg`, `max_following_deg`)로 임계값을 확인한다.

**리허설 (실물 없이 같은 프로필)**

`python -m pinch_sim.app --real-arm --rehearse --calib … --video auto`. 시뮬이 평소처럼 실행하고, 아무것도 움직이지 않는다.

**시험**

`python -m pytest tests/test_real_arm.py tests/test_arm_link_e2e.py`. e2e는 실물 프로그램을 가짜 서보로 실행하며 약 1분 걸린다.

## 8. 현장 시험 (Astra·마이크가 있을 때, 결과를 기록)

| 시험 | 방법 | 기록할 값 | 출발 기준 |
|---|---|---|---|
| A 표지 허용오차 | 카메라를 건드리지 않고 `check_markers.py` 5회 | px·mm 최대값 분포 | 최대값 + 여유로 `markers.yaml` 조정 |
| B 카메라 이동 감지 | 시뮬레이터 실행 중 카메라를 1~2 cm 밀거나 2° 돌림 | 빨간 배지가 뜨는지, 걸린 시간 | 1초 안 |
| C 화면 겹침 | 줄자로 잰 책상 위 점 3개를 모니터에서 클릭해 base 좌표 비교(`scripts/show_base_coords.py`) | 점별 오차 | ≤ 10 mm |
| D 손 3 cm | 핀치로 잡고 자를 따라 3 cm 옮기기 10회 | 경로점 변위의 중앙값·최대 | 중앙값 오차 ≤ 1 cm |
| E RGB↔depth 시간차 | 시뮬레이터 종료 시 출력 `rgb_depth_skew_ms` (p50, p95, max) | 분포 | p95 < 50 ms (`DepthConfig.max_skew_s`). **2026-10-23 이 Astra 실측(10초): RGB 25 fps, 깊이 15.4 fps(간격 최대 97 ms), 맞춘 시간차 p50 8.3 / p95 22.7 / 최대 31.4 ms → 153/153 사용 가능. 기준 50 ms·깊이 나이 150 ms 유지** |
| F 깊이 끊김 | 핀치 중 손을 카메라 0.6 m 안으로 / 다른 손으로 가림 | 1초 뒤 취소·원복되는지, 쿠옹이 안내 | — |
| G 음성 정지·에코 | 아래 절차 | 자기 음성 오작동 수, "멈춰" 성공 수 | 오작동 0 |
| H 카메라 목표 | 물체 3종 × 위치 3곳에서 G, 줄자 비교 | 윗면 중심 오차 | ≤ 2 cm (기울어 보면 카메라 쪽 치우침 알려짐) |

**시험 G (음성 정지와 스피커 에코)** — 기본 설정은 `mute_mic_while_speaking: true`, `listen_while_moving: false`(쿠옹이가 말하는 동안 마이크는 무음)라, 쿠옹이가 "이동을 시작할게요."라고 말하는 약 2초 동안의 "멈춰"는 들리지 않는다. 그동안은 **X 키**가 정지 수단이다.
1. 기본 설정으로: 이동 시작 직후(쿠옹이 말 끝난 뒤) "멈춰" 5회 → 멈춘 횟수.
2. `voice/config.yaml`에서 `listen_while_moving: true`로 바꾸고 스피커 볼륨 3단계에서: (a) 아무 말 없이 이동 5회 → 쿠옹이 자기 말이 끊기거나 명령으로 들어간 횟수, (b) 쿠옹이가 말하는 중 "멈춰" 5회 → 멈춘 횟수.
3. (a)가 0이고 (b)가 모두 성공한 설정(볼륨, 이어폰 여부)일 때만 `listen_while_moving: true`를 쓴다. 아니면 false로 두고 X 키를 안내한다. 결과를 이 문서에 적는다.

예비 측정 (2026-10-23 이 노트북, 내장 스피커·내장 마이크, 시스템 출력 볼륨 48 %, AI 없이 `voice.mic_check --echo`): 쿠옹이 목소리 크기(0.3)의 1 kHz 음이 마이크에 **−17.8 dBFS**로 들어온다(배경 −44 dBFS). 사람이 말하는 크기와 비슷해서, 내장 스피커로 `listen_while_moving`을 켜면 쿠옹이가 자기 말을 들을 가능성이 크다 → **이 노트북 내장 스피커로는 false 유지**, 켜려면 이어폰으로 위 1~3을 해 본다. ChatKHU로 한 실제 시험(1~3)은 API 키가 없어 아직 못 했다.

## 9. 장치·키가 없을 때의 대체 시연

| 없는 것 | 대체 |
|---|---|
| Astra | 7절 (나): 시뮬레이터 카메라 + 마우스 끌기(높이) + 말 |
| 보정 / 표지 | (가)를 `--allow-unverified-calib`로 리허설 (배지 빨강, 3D 핀치·카메라 목표 꺼짐) 또는 (나) |
| 마이크·스피커·API 키 | `python -m voice --sim --text`(키 필요) 또는 키 없이 `python -m voice.sim_dryrun` |
| 사람 손 | 마우스 |

`voice.sim_dryrun`은 AI 없이 쿠옹이 함수를 직접 불러 시뮬레이터의 반응과 쿠옹이가 말할 문장을 보여 준다:

```bash
source voice/env.sh
python -m voice.sim_dryrun --script "0.5:start_edit, 3:undo_edit, 5:finish_edit, 2:validate_and_move, 30:"
python -m voice.sim_dryrun        # 한 줄에 하나: start_edit / move_region region=middle direction=up amount=small / finish_edit / validate_and_move / stop_motion
```

## 10. 문제 해결

- **시뮬레이터: `the calibration is not usable for this camera`** — 출력의 `CALIBRATION:` 줄이 이유다. `passed: false` → 다시 보정. `camera setting '...' differs` → `pose.yaml`/`camera.yaml`을 보정 뒤에 바꿨다 → 다시 보정(또는 `calibrate.py --replay <세션> --use-current-config`). `camera identity` → 보정 때와 다른 카메라 또는 다른 USB 포트 → 같은 포트에 꽂거나 다시 보정.
- **`--calib with a real pac_vision calibration needs --video`** — 실제 보정을 시뮬레이터 카메라 영상에 겹칠 수는 없다. `--video auto`를 붙이거나 (나)를 쓴다.
- **`--video auto: no camera named like 'Astra' gives frames`** — 5절. 다른 프로그램이 카메라를 쓰는지 `fuser -v /dev/video*`.
- **`카메라를 pinch_sim (simulator) (PID …)이(가) 쓰고 있습니다`** — 그 프로그램을 끈다(카메라는 한 프로그램만).
- **Astra 깊이가 안 열림 (`no devices found`)** — 3절 udev, 다시 꽂기, `local.yaml`의 redist 경로. `python scripts/check_setup.py`.
- **배지 "카메라가 움직였어요"** — 카메라나 표지가 밀렸다. 표지가 그대로면 6절 ②~③을 다시. 로봇 위치를 바꿨으면 ④.
- **핀치가 안 잡힘** — 커서 옆 문구를 본다: "수정 모드가 아니에요"(E 또는 말), "정합 미검증"(표지), "깊이를 못 읽었어요"(손을 카메라에서 0.6 m 넘게), "깊이와 영상 시각이 안 맞아요"(시험 E).
- **쿠옹이: `시뮬레이터가 연결을 받지 않았습니다 (another voice client …)`** — 다른 쿠옹이·텍스트 클라이언트를 끈다.
- **쿠옹이: `PortAudio library not found`** — 2절 `libportaudio2`.
- **"쿠옹아"를 못 알아듣거나 말이 엉뚱하게 적힘** — `python -m voice.mic_check`로 잘림을 본다. 잘림이 있으면 `amixer -c 1 sset 'Internal Mic Boost' 0`(4절). 그다음 터미널 2의 `[사용자]` 줄을 보고 자주 틀리는 표기는 `voice/config.yaml`의 `wake_aliases`에 더한다.
- **ROS 패키지와 충돌(numpy 오류 등)** — 반드시 `source mujoco_sim/scripts/env.sh` / `source voice/env.sh`로 켠다.
- **MuJoCo 창 / 그래픽** — Wayland에서는 창 자동 배치가 안 된다(손으로 옮김). 3D 창이 안 뜨면 `MUJOCO_GL=egl` 상태에서 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia python -m pinch_sim.app ...`.
- **음수 인수** — `--robot-pose=-0.15,0,0`처럼 `=`로 붙인다.

## 11. git에 올리지 않는 것

개인 키 파일(`~/.config/pinch_khu/env`, 저장소 밖 — 저장소 키 `voice/chatkhu_key.env`는 올린다), 보정 원자료와 확인 결과(`pac_vision/data/sessions/`, `markers.check.yaml`, `*.bak` — 카메라 영상·장치 번호), 실물 서보 보정(`manipulator_control_ws/runtime/` — 12 V를 껐다 켜면 무효라 컴퓨터마다 `calibrate`), 실행 기록(`mujoco_sim/logs/`, `voice/logs/` — 대화 내용 포함, `manipulator_control_ws/logs/`), 저장 경로(`mujoco_sim/saved_paths/`), 가상환경, `vendor/`(setup.sh가 받음). 커밋 전 `git status`로 확인한다.

지금 시연 배치의 현장 값(`pac_vision/config/{cam2base,pose,markers,robot_pose,local}.yaml`)은 2026-10-28부터 **올린다**(맨 위 참고).

## 12. 계획과 달라진 점 (코드 기준으로 바로잡음)

- 카메라 이동 감지: 계획의 "책상면 z 중앙값 15 mm"는 수평 이동·회전을 놓칠 수 있어, 보정에 쓰지 않은 **ArUco 고정 표지**의 이미지 오차·책상면 오차로 바꿨다(합성 시험: 0.5° 회전, 2 cm 밀림부터 감지).
- `edit_mode` 직접 대입 9곳: 모두 상태 이벤트와 함께 일어나 revision·통지가 이미 되므로 바꾸지 않았다. 대신 손 입력만 `edit_mode`로 막고, 수정 모드를 켜면 자동 이동 예약을 취소한다.
- 자동 이동 예약은 이미 검증 토큰·경로 지문에 묶여 있었고 편집·X·새 경로·연결 끊김·실패에 취소됐다. 그대로 쓰고, 수정 모드 켜기를 취소 조건에 더했다. 로봇 상태는 실행 직전 `cmd_approve`가 다시 확인한다.
- 깊이 동기화: 최신 깊이 한 장 대신 최근 8장 중 **RGB 촬영 시각에 가장 가까운 장**을 쓴다(MediaPipe 지연 때문에 최신 장은 늘 30~60 ms 어긋남).
- 구형 ROS 음성과의 충돌: 클라이언트 hello가 프로토콜에 없어, 두 번째 음성 연결을 거부하는 방식으로 바꿨다.
- 비전 목표: YOLO(ultralytics) 대신 보정된 책상면 위로 솟은 깊이 점을 묶어 pac_vision `top_center`로 윗면 중심을 구한다(카메라를 다시 열지 않음).

## 13. 확인 범위 (2026-10-23, 연구실 노트북)

**확인함 (이 노트북, 카메라·마이크 없이):**
- 자동 테스트: mujoco_sim 148, pac_vision 103, path 편집 110, voice 13 통과.
- GUI(카메라 없음): 마우스 끌기 → S → V(12/12) → Y → 이동 완료; 쿠옹이 함수(`voice.sim_dryrun`)로 수정 모드 켜기 → 끌기 → 이전으로 → 끌기 → 수정 완료 → 검증 통과 → 묻지 않고 이동 → 도착 질문; 검증 실패 시 빨간 경로·실패 지점·"검증 실패 - 수정하세요".
- 녹화된 Astra 영상(팀 샘플 세션)에 합성 표지를 넣은 영상으로: 확인됨(0.4 px / 0.7 mm) / 20 px 밀린 영상은 "카메라가 움직였어요"(20.2 px / 39 mm); 쓸 수 없는 보정 파일·옵션 조합 거부.
- OpenNI2가 Orbbec redist로 초기화됨(장치 없음 보고), MediaPipe Hands 동작(빈 영상 약 18 ms/프레임).
- 2절 패키지 설치됨(libportaudio2 포함). 쿠옹이가 마이크를 인식하고(`--list-mics`), 내장 마이크 증폭을 낮춘 뒤 잘림 0 %. 스피커 → 마이크 에코 예비 측정 −17.8 dBFS(8절 시험 G 아래).

- **실제 ChatKHU(gemini-3.8-live, BAZE) 키보드 모드 + 시뮬레이터 (2026-10-23 23:11~23:19):** 호출어 없는 "경로 수정할래" → 막힘·"먼저 쿠옹아라고 불러 주세요" / "쿠옹아, 경로 수정할래" → 시뮬레이터 수정 모드 켜진 뒤 안내 문장 그대로 / "가운데를 위로 조금" 2 cm, "5센티" 5 cm / "아니 하지 마" → 함수 없음 / "이전으로" → 되돌림 / "이 경로로 할게" → 검증 12/12 → 묻지 않고 이동 → 도착 질문 / 잡담 → 함수 없음 / 호출어 창이 닫힌 뒤 도착 질문에 호출어 없이 "그냥 반대로 돌아가" → 질문 창으로 실행 / 귀가 질문에 "아니" → 경로 저장 / 질문 시간이 지난 뒤 호출어 없는 명령 → 막힘. 처음 시험에서 질문 알림(«…?»)을 AI가 그대로 읽지 못해(2/2) 알림 메시지에 '그대로 읽기'를 넣고 답할 시간을 질문을 다 말한 뒤부터 세게 고쳤다(고친 뒤 2/2 그대로 읽음).
- Astra 연결됨(2bc5:0403 깊이, 2bc5:0502 RGB = /dev/video2), udev 규칙, RGB·깊이 열림. 시험 E 실측: 맞춘 RGB↔깊이 시간차 p95 22.7 ms(8절 표). **카메라 배치는 보정용이 아님**: 책상 높이에서 수평으로 벽을 봐서 책상면 깊이가 거의 없다(아래쪽 유효 5 %) → 지금 배치로는 6절 보정을 할 수 없다.

**미확인:** 정합·보정·표지 오차(시험 A~C, 카메라 배치 필요), 실제 손 핀치 3 cm 오차·깊이 끊김(D·F), 음성(마이크) 모드의 실제 발화·음성 정지·에코(G의 1~3, 사람 필요), 카메라 물체 목표 오차(H). 합성 데이터로 확인한 것을 실제 장치 결과로 쓰지 않았다.

이전 노트북 기록(oryise12, 2026-10-18, `chatkhu/voice-mujoco-path-edit`): 실제 Astra Pro 보정 통과(원점 0.36 cm, 변 0.25 cm 이하), 실제 ChatKHU 음성 수정·되돌리기, 핀치 3D 수정 반영. 이 노트북에서 다시 확인한 것은 아니다.
