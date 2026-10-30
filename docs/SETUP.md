# 설치 · 보정 · 실행 안내

심사용 소개는 [README.md](../README.md)에 있습니다. 이 문서는 같은 시연을 다른 컴퓨터에서 다시 해 보려는 분을 위한 안내입니다.
명령은 저장소 폴더(`CHATKHU`)에서 실행합니다.

## 1. 준비물

| 무엇 | 시연에서 쓴 것 |
|---|---|
| 컴퓨터 | Ubuntu 24.04, Python 3.12, OpenGL이 되는 그래픽, 화면이 있는 데스크톱 (터미널 창은 gnome-terminal로 띄움) |
| 로봇팔 | ROBOTIS XM430-W210-T × 6 (ID n = 관절 n), U2D2 (1 Mbps), 12 V 전원과 **손이 닿는 전원 스위치** |
| 로봇 받침 | 180 × 180 × 20 mm 판. 판 가운데가 첫째 관절(J1) 축 |
| 깊이 카메라 | Orbbec Astra Pro. 거꾸로 매달아 작업 영역을 0.6 m 넘게 떨어져 내려다봄 |
| ArUco 마커 | A4 보정판(1~4번, 보정할 때만), 5번(카메라가 밀렸는지 확인), 6·7·8번(설거지 바구니 S · 국그릇 B · 밥그릇 C). 모두 4×4 사전(DICT_4X4_50) |
| 음성 | 마이크와 스피커, 인터넷, ChatKHU API 키 |
| 브라우저 | Google Chrome (없으면 Firefox) |

U2D2 장치 이름이 다르면 `manipulator_control_ws/config/robot.yaml`의 `bus.port`를 `ls /dev/serial/by-id/`에 나온 이름으로 바꿉니다.

## 2. 설치 (처음 한 번)

```bash
git clone https://github.com/oryise12/CHATKHU.git      # main 브랜치
cd CHATKHU
./setup.sh --system   # apt 패키지, Astra USB 권한, dialout·plugdev 그룹, Chrome (sudo). 이어서 아래 작업까지 함
                      # 그룹이 새로 들어갔으면 로그아웃 후 다시 로그인
./setup.sh            # 가상환경 3개(고정 버전) + Astra 드라이버 + 전체 시험
```

`setup.sh`가 만드는 것: `mujoco_sim/.venv`(시뮬레이터·손 인식·카메라), `.venv-voice`(음성), `manipulator_control_ws/.venv`(실물 제어), `vendor/ros2_astra_camera`(Astra 드라이버 파일).
끝에 나오는 시험 결과가 모두 `passed`면 정상입니다(시뮬레이터 179, 실물 제어 153, pac_vision 131, 경로 편집 110, 음성 16). 하드웨어 없이 도는 시험입니다.
설치 중 `mediapipe requires jax / jaxlib / opencv-contrib-python` 줄이 나오면 정상입니다(일부러 뺀 의존성).

**ChatKHU API 키.** 저장소에는 넣지 않습니다. 개인 파일에 한 줄로 넣습니다.

```bash
mkdir -p ~/.config/pinch_khu && install -m 600 /dev/null ~/.config/pinch_khu/env
nano ~/.config/pinch_khu/env        # 한 줄: BAZE_API_KEY=여기에_키
```

`voice/env.sh`가 이미 설정된 환경변수 `BAZE_API_KEY` → 이 파일 순서로 키를 찾습니다. 다른 키가 셸에 설정돼 있으면 그 키가 먼저 쓰입니다.

마이크 확인(키 없이): `source voice/env.sh && python -m voice.mic_check`

## 3. 현장 준비와 보정

배치(카메라, 로봇 받침, 마커)를 바꾸면 보정을 다시 합니다.

### 3-1. 로봇 보정 (12 V를 껐다 켤 때마다)

```bash
./run_demo.sh calibrate
```

- 3D 창의 기준 자세에 관절 1부터 6까지 하나씩 맞추고 Enter를 누릅니다. 기준 자세는 팔을 위로 곧게 뻗고 관절 3 = −90°, 관절 5 = +90°입니다.
- 6개가 모두 고정되면 q로 끝냅니다(토크 유지). 3D 창은 닫지 않습니다. 닫으면 프로그램이 끝납니다.
- 서보 6개가 보이는지 먼저 확인하려면: `cd manipulator_control_ws && source scripts/env.sh && python -m manipulator_control scan`
- 저장된 보정을 쓸 수 있는지 확인: `python -m manipulator_control calibration-status`
- USB 연결이 끊기거나 12 V를 껐다 켜면 저장된 보정이 지워집니다. 그때는 다시 보정합니다.

### 3-2. 카메라 보정 (카메라나 로봇 받침을 옮길 때마다)

카메라는 한 프로그램만 열 수 있습니다. 시뮬레이터를 먼저 끕니다.

```bash
cd pac_vision && source ../mujoco_sim/scripts/env.sh
python scripts/check_setup.py                        # 카메라·설정 점검
python tools/make_aruco_board.py ~/aruco_board.pdf   # 보정판이 없으면: A4 가로, 실제 크기 100 %로 인쇄
python scripts/calibrate.py --layout aruco           # 배치 점검 → Enter → 마커 4개 자동 검출 → 검사 5가지가 모두 ✓이면 s
python scripts/check_markers.py                      # 5번 마커로 확인: 8 px, 10 mm 이내면 통과
```

- **보정판 놓기:** 인쇄된 "로봇 쪽" 화살표가 로봇을 보게 가로로 놓고, 종이의 로봇 쪽 긴 변 가운데가 로봇 J1 축에서 9 cm 앞에 오게 합니다. 이 자리를 기준으로 좌표가 정해집니다(`pac_vision/config/a4_layout_aruco.yaml`).
- `check_markers.py` 창은 **영상 창을 클릭한 뒤 아무 키**나 눌러 닫습니다. 열어 둔 채 시뮬레이터를 켜면 카메라를 못 엽니다.
- 5번 마커의 위치(로봇 기준 좌표)는 `pac_vision/config/markers.yaml`에 적혀 있습니다. 마커를 다른 자리에 붙였다면 그 좌표를 실제 자리로 고칩니다.
- 카메라를 거꾸로 달지 않았다면 `pac_vision/config/local.yaml`의 `rotate_180`을 `false`로 바꾼 뒤 보정합니다. 다른 Astra를 쓰면 컬러·깊이 정합(`pac_vision/config/pose.yaml`)부터 다시 잽니다(`docs/PINCH_KHU_SETUP.md` 6절).
- 보정이 끝나면 보정판(1~4번)은 치워도 됩니다. 카메라와 로봇 받침은 움직이지 않습니다.
- 마커 위치를 눈으로 확인하려면: `python scripts/show_base_coords.py` (화면을 클릭하면 그 점의 좌표가 나옴, q로 종료)

### 3-3. 지점 마커 (6·7·8번)

- 6번 = S 설거지 바구니, 7번 = B 국그릇, 8번 = C 밥그릇 (`pac_vision/config/stations.yaml`).
- 시뮬레이터가 켜질 때 카메라로 **한 번** 위치를 읽습니다. 마커를 옮겼으면 시뮬레이터를 다시 켭니다.
- 로봇에서 약 25~30 cm, 로봇 앞쪽에 놓습니다. 시뮬레이터 창에 `stations located:`와 함께 각 지점의 좌표·거리가 나오고, 너무 멀면 경고가 붙습니다. 실제로 갈 수 있는지는 검증이 판단합니다.
- 목표 높이는 세 지점 모두 책상 위 12 cm, 그릇·바구니에 도착하면 5 cm 내려가서 그대로 있습니다(`height_m`, `dip_m`).

## 4. 실행

```bash
./run_demo.sh check     # 실물은 궤적을 검사만 하고 움직이지 않음 (처음엔 이것부터)
./run_demo.sh real      # 실물 이동
./run_demo.sh rehearse  # 실물 없이 카메라 영상 위에서 시뮬만 움직이며 전체 흐름 연습
```

`real` 순서:
1. **실물 제어 창:** HOME 미리보기 → **y** → HOME 도착 → "시뮬 연결 대기"
2. **시뮬레이터 창과 웹 화면**, 이어서 **음성 창**이 자동으로 뜹니다.
3. **무장:** 실물 제어 창에서 **y**를 누르거나, 화면 위에 뜨는 작은 **"실물 무장"** 창에서 y(또는 버튼)를 누릅니다. 팔 주변을 확인하고 12 V 스위치에 손을 둡니다.
4. 쿠옹이: "설거지 바구니로 이동할까요?" → "응" → 검증 후 바구니로 이동
5. "설거지 바구니에 도착했어요. 어디로 갈까요?" → "국그릇으로 가자"(또는 "B", "밥그릇", "C") → 바구니에서 그릇까지 직선 경로가 그려지고 수정 모드가 켜짐
6. 경로를 고친 뒤 "수정 완료" → "검증 후 이동할까요?" → "응" → 검증을 통과하면 이동
7. "B 국그릇에 도착했어요. 돌아오는 길을 수정할까요, 가던 길 그대로 반대로 돌아갈까요?" → 고르기 → 바구니로 돌아옴
8. "남은 C 밥그릇으로 갈까요?" → 같은 방식 → "두 곳 모두 다녀왔어요. 다시 이동할까요?"

- `check` 모드에서는 실물이 바구니로 가지 않으므로 그다음 단계(그릇 고르기)로 넘어가지 않습니다. 전체 흐름 연습은 `rehearse`를 씁니다.
- 옵션: `--no-voice`(음성 창 없이 화면 버튼으로), `--speed-scale 0.5`(real·check: 실물 속도 절반)
- `gnome-terminal`이 없으면 세 창을 직접 띄웁니다.
  ```bash
  # 창 1: 실물 제어
  cd manipulator_control_ws && source scripts/env.sh && python -m manipulator_control pinch
  # 창 2: 시뮬레이터 ("시뮬 연결 대기"가 된 뒤)
  cd mujoco_sim && source scripts/env.sh && python -u -m pinch_sim.app --real-arm --calib ../pac_vision/config/cam2base.yaml \
    --video auto --hand --depth --voice-gateway --swap-left-right --stations
  # 창 3: 쿠옹이 (시뮬레이터 창에 "voice gateway:" 줄이 뜬 뒤)
  source voice/env.sh && python -m voice --sim
  ```

## 5. 쓰는 법

**말 (호출어 없이 바로 말함)**

| 하고 싶은 것 | 예시 |
|---|---|
| 그릇 고르기 | "국그릇으로 가자", "B", "밥그릇", "C" |
| 점 옮기기 (검지로 가리킨 뒤) | "여기 위로 조금", "앞쪽으로 2센티", "왼쪽으로 많이" |
| 방향 | 위 · 아래 · 왼쪽 · 오른쪽 · 앞 · 뒤 (왼쪽·오른쪽은 노트북 화면 기준) |
| 거리 | 조금 2 cm · 보통 5 cm · 많이 8 cm, 또는 숫자(한 번에 15 cm까지). 말하지 않으면 조금 |
| 양 조절 · 되돌리기 | "조금만 더", "너무 갔어", "이전으로", "처음으로" |
| 끝내기 | "수정 완료", "이 경로로 할게" → "검증 후 이동할까요?"에 "응" |
| 돌아오는 길 | "돌아오는 길 수정할래", "반대로 돌아가" |
| 멈춤 | "멈춰" → 5초 뒤 "계속 갈까요, 돌아갈까요?" → "계속 가" / "돌아가" |

**손**
- 검지로 경로 위 점을 0.4초 가리키면 그 점이 노란 점으로 골라집니다.
- 엄지와 검지로 경로를 집어(핀치) 옮기면 손이 움직인 만큼 3차원으로 옮겨지고, 손을 놓는 순간 반영됩니다. 카메라 정합이 확인됐고(화면 오른쪽 위 초록 배지) 손이 카메라에서 0.6 m 넘게 떨어져 있을 때만 됩니다.

**화면**
- 대화창: 쿠옹이가 물을 때 답 버튼이 뜹니다(예: "B 국그릇", "C 밥그릇", "이동"). 대화창을 접으면 "이렇게 말해 보세요"와 함께 지금 할 수 있는 말이 보입니다.
- 오른쪽 아래 3차원 화면: 마우스로 끌면 경로 가운데를 중심으로 돌아갑니다.
- 아래 버튼: 수정 중에는 되돌리기 · 다시하기 · 처음 경로 · 수정 완료.

**멈추기:** "멈춰", 화면의 정지 버튼, 실물 제어 창의 s(또는 스페이스), 12 V 전원 스위치.

**끝내기**
1. 실물 제어 창에서 q (토크 유지). 창을 X로 닫지 않습니다.
2. 웹 화면 창을 닫으면 시뮬레이터가 끝납니다.
3. 음성 창은 Ctrl+C.

**토크 끄기:** `./run_demo.sh torque-off` → y → READY → 팔을 손으로 받치고 x → y → q

## 6. 문제 해결

| 증상 | 원인과 해결 |
|---|---|
| 시뮬레이터가 바로 꺼짐: "카메라를 … check_markers.py(…)이(가) 쓰고 있습니다" | 다른 카메라 프로그램(보정·확인 창)이 켜져 있음. 그 창을 클릭하고 아무 키로 닫은 뒤 다시 실행 |
| "실물 로봇팔이 아직 무장 전이에요" | 무장(y)을 안 한 상태. 실물 제어 창이나 "실물 무장" 창에서 y |
| 시뮬레이터 창에 `stations: looking for markers - not yet: …` | 그 마커가 카메라에 안 보임. 가리는 것을 치우고, 작으면 더 크게 인쇄 |
| 웹 화면 오른쪽 위가 빨간 배지 "카메라가 움직였어요" | 카메라나 5번 마커가 움직임. 카메라 보정(3-2)을 다시 하고, 5번 마커 좌표를 확인 |
| 핀치가 자주 끊김(`depth_lost`, `no_depth`) | 손이 카메라에서 0.6 m보다 가까움. 카메라를 더 높이·멀리 |
| 저장된 보정을 쓸 수 없다며 거부 | 12 V를 껐다 켰거나 USB가 끊김. `./run_demo.sh calibrate` |
| 12 V LED가 깜빡이고 일부 서보가 안 보임 | 케이블·커넥터 단락. `python -m manipulator_control scan`으로 보이는 ID 확인 |
| 쿠옹이: "ChatKHU에 연결하지 못했습니다 … HTTP 402 insufficient_quota" | 그 키의 이번 달 크레딧 소진. 키 계정에 크레딧을 충전 |
| 쿠옹이: "… HTTP 401 Invalid API key" | 키가 아님(예: 충전용 쿠폰 코드를 넣음). ChatKHU에서 발급한 API 키를 넣음 |
| "BAZE_API_KEY is not set" | 2절의 개인 키 파일을 만듦 |
| `check` 모드에서 시뮬이 "중단됨" | 정상. 실물이 움직이지 않아 시뮬이 멈춘 것으로 처리 |

더 자세한 보정·좌표계·정지 동작 설명은 [PINCH_KHU_SETUP.md](PINCH_KHU_SETUP.md)에 있습니다.
