# 카메라 보정·핀치 경로 수정·쿠옹이 음성·MuJoCo 통합 계획

## Context
- **목표:** 연구실 노트북에서 다음 시연을 끝까지 실행한다.
  - MuJoCo 6축 로봇과 실제 Astra 영상을 겹쳐 보여 준다.
  - 음성("쿠옹아, 경로 수정할래")으로 수정 모드에 들어간다.
  - 검지로 경로점을 고르고, 엄지·검지 핀치로 잡아 3D로 수정한다.
  - "수정 완료"라고 하면 검증하고, 통과하면 **시뮬레이터에서 자동으로 이동**한다.
- **현재 상황:**
  - 기능 대부분은 `chatkhu/voice-mujoco-path-edit`(c4058e4)에 이미 있다.
  - 하지만 수정 모드 진입이 실제로 연결돼 있지 않고, 깊이를 못 받으면 조용히 2D로 바뀐다.
  - 검증 뒤 "이동할까요?"를 한 번 더 묻는 등 합의한 흐름과 다르다.
  - 보정 파일 검사가 없고, 남의 카메라 정합 값과 고정된 장치 번호를 전제로 한다.
- **범위:** 시뮬레이터 전용이다. 실물 로봇 실행은 범위 밖이다.
- **사용자 결정 (2026-10-23):**
  1. 깊이를 잃으면 **3D 편집을 거부하고 안내**한다. 2D로 자동 대체하지 않는다.
  2. 손 입력은 **수정 모드에서만** 받는다. 마우스와 버튼은 지금처럼 항상 동작한다.
- **조사 방법과 상태:**
  - Plan 모드에서 읽기 전용으로 조사했다. c4058e4는 bare 클론을 `git show`로 읽었고, 나머지 브랜치는 `gh api`로 읽었다.
  - 영역별 조사 3건(비전·좌표, 시뮬 런타임, 음성)은 완료했다. 핵심 주장은 코드 줄 단위로 직접 다시 확인했다.
  - 함수 단위 설계를 교차검증하던 에이전트는 세션이 끝나면서 중단됐다. 그래서 S3–S5의 세부(특히 자동 실행 예약 바인딩)는 구현 착수 때 코드로 다시 확인한다.
- **계정 변경 반영:** 조사 중에 노트북 계정이 robotics에서 jammin으로 바뀌었다. 이 계획의 `~`는 이제 `/home/jammin`이다. `~/pinch_khu_ws`는 아직 없고, `~/.bashrc:118`의 ROS Jazzy source와 plugdev 그룹은 그대로다.
- **승인 후 첫 동작:** 이 계획을 `~/Downloads/PINCH_KHU_INTEGRATION_PLAN.md`로 저장하고 멈춘다. 구현·커밋·푸시는 따로 지시가 있을 때 S0부터 시작한다.

## 0. 사용자에게 보이는 시연 흐름 (완성 후)
1. **준비:** 터미널 1에서 시뮬레이터를 켜면 MuJoCo 창, 카메라 모니터 창(실제 Astra 영상), 경로가 뜬다. 모니터에는 로봇 기준축, A(시작점), B(목표점), 3D 경로가 겹쳐 그려진다. 터미널 2에서 쿠옹이를 켠다. 기존 버튼(N/S/V/Y/X/U)과 마우스 조작은 그대로 동작한다.
2. **수정 모드 진입:** "쿠옹아, 경로 수정할래"라고 말하면 시뮬레이터가 실제로 수정 모드가 되고, 화면에 "수정 중 · 손 입력 켜짐"이 뜬다. 그다음 쿠옹이가 "네. 손가락으로 경로를 수정해 주세요."라고 말한다. 음성 없이 할 때는 E 키로 같은 일을 한다.
3. **점 고르기와 잡아 옮기기:** 검지로 경로 안쪽 점을 0.4초 가리키면 그 점이 노랗게 표시된다. 엄지와 검지로 집어서 손을 3D로 움직이면 그 점과 주변 경로가 함께 따라 움직이고, 놓으면 확정된다.
   - 깊이를 못 읽으면 잡지 않는다. 화면과 음성으로 "깊이를 못 읽었어요. 손을 카메라에서 조금 떨어뜨려 주세요"라고 안내한다.
   - 움직이는 중에 깊이가 끊기면 미리보기를 멈추고, 1초가 넘으면 이번 동작을 취소해 원래 경로로 되돌린다.
4. **되돌리기:** "쿠옹아, 이전으로"라고 하면 직전 수정 한 번을 되돌린다(U 키도 같다).
5. **검증과 자동 이동:** "이 경로로 할게" 또는 "수정 완료"라고 하면 검증을 시작한다.
   - **실패하면:** 실패한 지점이 빨간 점으로 남고, 쿠옹이가 이유를 말한 뒤 수정 모드로 돌아간다.
   - **통과하면:** 쿠옹이가 "검증을 통과했어요. 이동을 시작할게요."라고 말하고, 시뮬레이터가 그 검증된 경로로 자동 이동한다.
   - "멈춰" 또는 X 키로 언제든 정지한다. 이후 동작(목표 도착 → 돌아오는 길)은 지금과 같다.
6. **(별도 단계) 비전 목표:** "쿠옹아, 이 물체로 목표 바꿔 줘"라고 하면 카메라가 찾은 물체 위로 목표점이 바뀌고, 새 초기 경로가 생성된다.

---

## 1. 브랜치·HEAD·구성요소 (2026-10-23 `git ls-remote`로 재확인, 조사 당시 값과 동일)
| 브랜치 | HEAD | 날짜 | 담긴 것 | 판정 |
|---|---|---|---|---|
| main | be843c0 | 09-30 | README(경로편집 revert됨) | 직접 쓰지 않음 |
| urdf | b2b4038 | 09-28 | 6dof_description 초판 | 구형(그리퍼 모델 등 3개 파일 없음) |
| work/webcam-path-editor | 4ace515 | 10-02 | `path 편집/` | 기반과 19개 파일 blob **동일** |
| work/gemini-live-voice | 921c46e | 09-28 | ROS2 Gemini 음성 노드 | 구형 → `voice/`로 대체 |
| work/integration-all | 5806df8 | 10-13 | 6dof+path편집+mujoco_sim+ROS 음성(`src/manipulator_voice`) | 기반이 이 커밋에서 출발 |
| work/astra-calibration | c1e355d | 10-15 | pac_vision | 기반과 77/78 동일, `calibrate.py`는 기반이 더 최신 |
| chatkhu/main | e240e3a | 10-10 | README | — |
| **chatkhu/voice-mujoco-path-edit** | **c4058e4** | 10-17 | 위 전부 + `voice/` + 손·깊이·보정 연동 | **통합 기반** |

**기반 선택의 근거**
- `chatkhu/*`는 다른 브랜치와 **공통 조상이 없다**(GitHub compare 404). 그래서 커밋 그래프가 아니라 파일 blob으로 비교했다.
- 6dof_description 24개, path 편집 19개, mujoco_sim 42개 파일이 integration-all과 같다. mujoco_sim은 10개 파일이 수정됐고 14개가 추가됐다(`canvas`, `pac_calib`, `hand_input`, `hand_depth`와 각 테스트).
- 1f6adc1의 `ui.py`는 5806df8과 8줄, 6544da0과 138줄 다르다. 즉 integration-all 최신에서 출발했다.
- pac_vision의 유일한 차이(`calibrate.py`)는 c4058e4가 안내 문구와 "번호 키로 그 점만 재클릭" 기능을 더한 것이다.
- 기반에만 없는 것: `src/manipulator_{voice,interfaces}`(ROS 음성), `docs/CURRENT_STATE.md`, `docs/FRAMES.md`, `.env.example`. 모두 구형 음성 경로용이라 **가져오지 않는다.**

**구성요소 현황 (코드 기준)**
| 기능 | 상태 | 근거 |
|---|---|---|
| pac_vision 보정(A4 5점, RGB↔depth 정합, Kabsch, `passed`) | 됨 | `pac_vision/src/pac_vision/cam2base.py:519-632`, `session.py:80-107` |
| 시뮬 모니터에 실영상 + 축·A·B·경로 겹치기 | 됨 | `pac_calib.py:72-91`, `canvas.py:75-104` |
| `--robot-pose X,Y,YAW` | 됨 | `app.py:285-300`, `canvas.py:36-41` |
| 검지 가리키기 선택 / 핀치 2D·3D 드래그 | 됨 | `hand_input.py:71-229`, `hand_depth.py` |
| 깊이 없을 때 2D로 **조용히 대체** | 고쳐야 함 | `hand_input.py:188-190` |
| RGB↔depth 시간차 검사 | 없음 | 스트림별로 나이 0.3 s만 봄(`app.py:587-592`) |
| 경로 편집 알고리즘(영향 영역, 끝점 고정, undo/redo) | 됨(원본 import) | `editing.py:31-39` → `path 편집/src/pac_path_editor/{core,editor}.py` |
| 음성: 호출어, 14개 함수, 실행 결과 피드백, 빠른 정지 | 됨 | `voice/tools.py:28-75`, `voice/sim_link.py` |
| 음성 `start_edit`(수정 모드 진입) | **없음**. 시뮬 쪽 명령만 있다 | `voice_gateway.py:113-118` |
| `edit_mode`가 손 입력을 막는 게이트인가 | **아님**(표시용) | `session.py:15-16`, 게이트는 `can_edit()` `:659-666` |
| 검증 후 동작 | 음성이 "이동할까요?"를 묻는다 / Y는 즉시 실행 | `sim_link.py:49-50,227-256`, `session.py:583-608` |
| 자동 실행 예약 기구 | **이미 있음** | `Session.arm_auto_run`/`watch`, `voice_gateway.py:188-212` |
| 시뮬이 보정 파일의 `passed`·현재 정합과 일치하는지 검사 | **없음** | `pac_calib.py:30-34,119-121` |
| 비전 물체 → 목표점 자동 입력 | **없음** | `mujoco_sim/`, `voice/`에 `objects`·`target_point` 참조 0건 |

---

## 2. 아키텍처와 데이터 흐름

```
[Astra RGB /dev/videoN] --(sim: hand-camera 스레드, MediaPipe)--> 손 랜드마크(px, t_rgb)
[Astra depth OpenNI]    --(sim: astra-depth 스레드)-----------> depth mm(t_depth)
             │ 동기 검사 |t_rgb − t_depth| ≤ 50 ms, 각 나이 ≤ 150 ms
             ▼
  손가락 px + depth → base_ref(m) → MuJoCo world(m) → Session(같은 객체, 같은 revision)
                                                        ▲            ▲             ▲
                                         마우스/버튼(main)  손(main)   음성 게이트웨이(main에서 drain)
  Session: 경로 편집(PathEdit → pac_path_editor) → 검증(워커 스레드) → PASSED → 자동 실행 예약 → EXECUTING(MuJoCo 토크 제어)
  쿠옹이 프로세스(.venv-voice): 마이크 → ChatKHU(Gemini Live) → 함수 → AF_UNIX 소켓 → 게이트웨이 → 결과·이벤트 → 음성 안내
```

| 단계 | 입력 | 출력 | 좌표계·단위 | 소유 프로세스·스레드 |
|---|---|---|---|---|
| 1. 정합 | RGB px (u,v) 640×480 | depth px (u_d,v_d) | `u_d = sx·u + a + b·u + c·v + d/Z` (`pose.yaml`) | sim 메인(투영 때) |
| 2. 역투영 | (u_d,v_d), depth mm | p_cam | `p_cam = Z·K⁻¹[u_d,v_d,1]`, `Z = mm/1000 − z_offset` (m, 광학) | sim |
| 3. 보정 | p_cam | p_base | `p_base = R·p_cam + t` (`cam2base.yaml`). base_ref: m, +X 앞(A4 화살표), +Y 왼쪽, +Z 위, 책상 z=0, 원점 = A4 대각선 교점 | sim |
| 4. 로봇 배치 | p_base | p_world | `p_world = R_z(90°−ψ)·(p_base − t_r)`, `t_r = (X,Y,0)` = base_link 원점의 base_ref 위치, ψ = 로봇 앞(+Y_world)이 +X_base에서 반시계로 이룬 각 (`canvas.py:31,75-80`) | sim |
| 5. 편집 | p_world 변위(3D 핀치) | 새 경로 revision | MuJoCo world(m, 로봇 앞 = +Y) | sim 메인 |
| 6. 검증 | 경로 + 로봇 상태 | 보고서(12개 검사, 실패 지점 where_m) | world | sim 검증 워커 |
| 7. 실행 | 검증된 궤적 | 관절 토크 | 관절 공간 | sim 메인(physics) |
| 8. 음성 | 마이크 16 kHz | 함수 호출 → 소켓 명령(`expected_rev`) | — | voice 프로세스 |
| 9. 화면 겹치기 | p_world | RGB px | 4→3→2→1의 역순 (`pac_calib.py:72-77`) | sim 메인 |

- **변환 전체:** `p_world = R_z(90°−ψ)·(R·p_cam + t − t_r)`
- **중요:** `--robot-pose`를 생략하면 "A4 원점 = 로봇 base_link 원점이고, 로봇 앞 = +X"라고 가정한다(`canvas.py:6-15`). base_link 원점은 J1 축에서 약 3 mm 떨어져 있다(`frames.py:16`). 계획에서는 이 가정을 **쓰지 않고** 현장에서 잰 값을 넣는다(§3 S2).

---

## 3. 단계별 구현 계획
- **작업 브랜치:** `origin/chatkhu/voice-mujoco-path-edit`에서 새 브랜치 `work/pinch-khu-demo`를 만든다. 원격 브랜치는 모두 그대로 둔다. `main`에 쓰지 않고, 여러 브랜치를 병합하지도 않는다.
- 단계마다 커밋하고, 푸시는 사용자가 승인한 뒤에만 한다.

### S0. 작업공간·환경 (의존성 없음)
**복제**
- `mkdir -p ~/pinch_khu_ws && cd ~/pinch_khu_ws`
- `git clone https://github.com/jaemin0319/26_project_manipulator.git`
- `git switch -c work/pinch-khu-demo origin/chatkhu/voice-mujoco-path-edit`
- 기존 디렉터리가 있으면 멈추고 사용자에게 보고한다.

**시스템 패키지 (사용자가 sudo로 실행)**
- `python3.12-venv python3-pip libportaudio2 fonts-noto-cjk x11-utils x11-xserver-utils v4l-utils mesa-utils`
- `libopenni2-dev`는 이미 있다.

**Astra 드라이버**
- 시스템 OpenNI2에는 Orbbec 드라이버가 없다(PS1080만 있음).
- Orbbec redist를 `~/pinch_khu_ws/vendor/ros2_astra_camera`에 받기만 하고 빌드하지 않는다.
- `pac_vision/config/local.yaml`의 `camera.openni.redist`에 그 경로를 적는다(gitignore 대상).
- udev 규칙 `99-orbbec.rules`: `idVendor=="2bc5"`, `GROUP="plugdev"`, `MODE="0660"`. 사용자가 plugdev 그룹에 있으므로 README의 0666보다 좁게 잡는다.

**파이썬 환경 두 개**
- 시뮬레이터 환경(`mujoco_sim/.venv`): `requirements.lock`(mujoco 3.15.0, numpy 1.26.4, opencv-python 4.11) → `pip install --no-deps mediapipe==0.10.21` → `requirements-hand.txt` → `requirements-astra.txt`(openni 2.3.0, `-e ../pac_vision`).
- 음성 환경(`.venv-voice`): `voice/requirements.txt`. 시뮬레이터 환경과 섞지 않는다.

**ROS 충돌**
- `~/.bashrc:118`이 Jazzy를 source해서 PYTHONPATH와 LD_LIBRARY_PATH가 오염된다.
- `mujoco_sim/scripts/env.sh`는 PYTHONPATH만 unset한다.
- `voice/env.sh`를 새로 만들어 PYTHONPATH를 unset하고 `.venv-voice`를 켠다. 두 env.sh 모두 ROS 경로의 LD_LIBRARY_PATH도 제거한다.
- miniforge base(Py 3.14)와 다른 프로젝트의 conda 환경은 쓰지 않는다.

**그래픽**
- 세션은 Wayland + XWayland이고, GPU는 NVIDIA prime on-demand다.
- `scripts/gui_smoke.sh`가 실패하면 `MUJOCO_GL=egl`을 쓰고, 그래도 안 되면 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`를 쓴다.

**검증과 완료 기준**
- 검증: `mujoco_sim`, `pac_vision`, `path 편집`에서 각각 `python -m pytest -q`를 돌린다(README 기준 mujoco_sim 126 passed). `voice`는 `python -m voice --list-mics`로 확인한다.
- 완료 기준: 카메라 없이 README 실행 예 (나)(시뮬 카메라 + 마우스)가 돈다. 음성은 텍스트 모드로 시뮬과 연결된다.

### S1. 카메라 인식·보정 재현 (S0 이후, Astra 필요)
**장치 찾기**
- **수정** `app.py`: `--video auto`를 추가해 `/sys/class/video4linux/*/name`에서 `camera.yaml`의 `name_match: Astra` 중 가장 작은 번호를 고른다(pac_vision `_ui.open_rgb` 로직과 같게).
- README와 예시에서 고정 번호 `--video 2`를 없앤다.

**정합 프리셋 적합성 (남의 값을 기본으로 쓰지 않음)**
- README 7단계의 "`astra_unit1.yaml`을 무조건 복사"하는 줄을 없앤다.
- 먼저 `scripts/check_camera.py`로 RGB/depth 클릭 쌍을 8개 이상 모은다.
- 그다음 `tools/fit_rgb_to_depth.py`로 판정한다:
  - unit1 프리셋의 잔차가 ≤ 2 px이면 그대로 쓴다.
  - 넘으면 `pose_presets/astra_<라벨>.yaml`로 새로 맞추고, `--write-pose`로 `pose.yaml`을 만든다.
- 카메라 실물에 라벨을 붙이고 프리셋 README 표에 적는다.

**보정**
- `python scripts/calibrate.py`를 실행한다(시뮬레이터는 닫아 둔 상태).
- `passed: true`일 때만 `s`로 저장한다. `s s`로 실패 결과를 강제 저장하는 것은 시연용으로 쓰지 않는다.

**보정 결과에 신원 정보 추가**
- **수정** `pac_vision/src/pac_vision/session.py:write_cam2base`: `camera_identity`를 기록한다. 내용은 v4l 장치 이름, `/dev/v4l/by-path` USB 포트 경로, 정합 프리셋 파일 이름과 sha256이다.
- OpenNI 시리얼 SN0001은 카메라마다 같아서 구별에 쓸 수 없다.

**현장 독립 검증**
- `calibrate.py --check` 줄자 검사(기존 기능, 허용 1 cm)를 한다. 보정에 쓰지 않은 점(A4 이음매 S1·S2, 줄자로 잰 점 2개 이상)을 쓴다.
- **수정:** 결과를 `config/cam2base.check.yaml`(gitignore)에 남긴다.

**완료 기준**
- `passed: true`, holdout ≤ 10 mm, 각 변 오차 ≤ 5 mm.
- 독립 검증점 오차 ≤ 10 mm.

### S2. 화면 정합·보정 파일 수용 검사 (S1 이후)
**`--calib` 불러오기를 엄격하게**
- **수정** `pac_calib.py:load_camera`:
  - `passed: false`, 예시 파일, MOCK 파일은 거부한다. 시연 리허설에서만 `--allow-unverified-calib`로 허용하고, 그때는 빨간 배너를 띄우고 3D 핀치를 막는다.
  - `pac_vision.session.calibration_matches_current`를 호출한다. 이 함수는 확장해 정합 계수에 더해 `flip`, `z_offset_m`, 내부 파라미터, 해상도까지 비교한다. 불일치하면 거부한다.
  - `camera_identity`가 지금 연결된 장치와 다르면 거부한다.
  - `$PAC_VISION_SRC`나 `~/research/...` 경로보다 **저장소 안의 `pac_vision`을 먼저** 쓰도록 순서를 바꾼다.

**카메라가 움직였는지 감지 (보정 무효화)**
- **새 함수** `pac_calib.desk_drift(depth_mm)`:
  - 보정 때 A4가 있던 영역의 책상 픽셀을 현재 depth로 base_ref로 바꿔, z의 중앙값을 구한다.
  - 시작할 때와 수정 모드에 들어갈 때 검사한다. |z| > 15 mm이면 "카메라가 움직였어요. 다시 보정해 주세요"를 띄운다.
  - 그러면 겹쳐 그리기와 3D 핀치를 끄고, 시뮬 카메라 + 마우스 대체 시연으로 바꾼다.

**로봇 배치**
- 로봇 base_link 중심의 base_ref 위치 (X, Y)와 앞 방향 ψ를 자로 잰다.
- 그다음 시뮬 TCP의 초기 자세 점을 카메라 화면에 투영하고, 실제 로봇 TCP나 표지와의 픽셀 차이로 확인한다.
- 원점 = 로봇이라는 가정은 측정해서 확인됐을 때만 쓴다.
- **수정:** `--robot-pose`를 `cam2base.yaml` 옆의 `robot_pose.yaml`에도 저장할 수 있게 한다. CLI 값이 그 파일보다 우선한다.

**조합 오류 방지**
- `--calib`만 있고 `--video`가 없으면, 실제 보정을 시뮬 side_cam에 투영하게 된다(`app.py:397-398`). 이 조합은 거부한다.

**검증**
- `tests/test_pac_calib.py`에 추가한다: passed:false 거부, 정합 불일치 거부, identity 불일치 거부.
- 현장 확인: 경로를 그린 화면에서 줄자 점 3개의 재투영 오차가 ≤ 8 px(640×480)인지 사진으로 남긴다.

**완료 기준**
- 축·A·B·경로가 실제 종이와 로봇 위에 8 px 이내로 겹친다.
- 카메라를 2 cm 옮기면 무효 안내가 뜬다.

### S3. 손 핀치 (S2 이후. 마우스 경로는 S0에서 이미 동작)
**수정 모드 게이트**
- **수정** `hand_input.py`의 가리키기 선택과 `_grab`: `session.edit_mode`가 꺼져 있으면 무시하고, "수정 모드가 아니에요(E 키 / '경로 수정할래')"를 한 번만 보여 준다.
- `Session`에서 `edit_mode`를 직접 대입하는 곳(178, 418, 441, 529, 549, 563, 596, 685, 801행)을 모두 `set_edit_mode()`로 바꾼다. 그래야 revision이 오르고 이벤트가 나간다.
- 키 E로 수정 모드를 켜고 끈다.

**타임스탬프와 동기화**
- **수정** `hand_input.py` HandCamera: `cap.read()` 직후의 시각(촬영 시각)을 저장한다. 지금은 MediaPipe 처리가 끝난 뒤의 시각을 쓴다(`:346-352`).
- `hand_depth.AstraDepthSource`: 읽은 시각과 함께 OpenNI 프레임 timestamp도 보관한다.
- `app.py`: 각 프레임의 나이가 ≤ 150 ms이고 |t_rgb − t_depth| ≤ 50 ms일 때만 깊이를 "유효"로 넘긴다.

**깊이 정책 (사용자 결정: 거부 + 안내)**
- **잡을 때:**
  - `_grab`에서 깊이가 유효하지 않거나 손가락 깊이를 못 재면 2D로 바꾸지 않는다.
  - `edit_refused: no_depth` 이벤트를 내고, 화면 문구와 함께 음성으로도 안내한다.
  - `--depth`를 켜지 않은 웹캠 모드(README 실행 예 (다))만 처음부터 "2D 높이 모드"로 시작하고, 화면에 늘 표시한다.
- **드래그 중:**
  - 깊이가 무효가 되면 미리보기를 멈추고 "깊이 끊김"을 표시한다. 0.3 s 안에 돌아오면 이어서 한다.
  - 1.0 s를 넘으면 `edit_cancel`로 되돌린다.
  - 끊긴 동안 손을 놓으면 확정하지 않고 취소한다.
  - 8 cm 넘게 튄 값을 0.3 s 뒤 그냥 받아들이는 동작(`hand_depth.py:113-130`)은 없앤다. 대신 드래그를 일시정지한다.

**겹친 경로점 고르기**
- **수정** `screen.pick_index`: 반경 안 후보를 인덱스가 이어지는 묶음끼리 나눈다.
  - 깊이가 유효하면 손가락 3D 점과 가장 가까운 묶음을 고른다.
  - 깊이가 없으면 카메라에 가장 가까운 묶음을 고른다.
  - 고르기 전에 강조 링으로 어느 점인지 보여 준다(기존 hover).
- 손이 경로를 가리는 문제는 손가락 패치 깊이의 25백분위로 처리한다(현행). 표시는 경로를 손 위에 덧그린다(현행).

**입력 동시성**
- 드래그에 `owner`(mouse/hand)를 기록하고, 다른 입력이 그 드래그를 움직이거나 끝내지 못하게 한다.

**테스트**
- `test_hand_input.py`, `test_hand_depth.py`: 깊이가 없을 때 2D로 바뀌지 않고 거부하는지(기존 `:156-166` 기대값을 반대로 수정), 끊김 → 일시정지 → 취소, 시간차가 클 때 거부, 수정 모드가 꺼져 있을 때 무시, 겹친 점 고르기.

**현장 검증**
- 손을 3 cm 움직였을 때 경로점 변위를 10회 잰다(S7 표의 "미결정" 항목).

**완료 기준**
- 위 테스트가 통과한다.
- 현장에서 3 cm 이동의 오차 중앙값이 ≤ 1 cm다.

### S4. 음성 진입·안내 (S3과 병행 가능. 게이트는 S3 필요)
**`start_edit` 함수 추가**
- **수정** `voice/tools.py`: `start_edit`을 추가한다. 예시 표현은 "경로 수정할래", "고칠래", "수정 모드".
- `voice/sim_link.py`: 시뮬 명령 `start_edit`으로 보낸다.
- 시뮬이 받아들인 결과(`applied`)가 왔을 때만 "네. 손가락으로 경로를 수정해 주세요."라고 말한다. 거부되면 이유를 말한다. 안내만 하는 척하지 않는다.
- `edit_return_path`와 `keep_editing` 설명에서 "수정할래"와 겹치는 예시를 정리한다.

**질문에 묶인 후속 답**
- **수정** `voice/wake_word.py`, `voice/session.py`: 30 s로 계속 연장되는 창을 다음처럼 바꾼다.
  - (a) 호출어를 들으면 창을 연다. 연장은 **함수가 실제로 실행됐을 때만** 한다. 사람 말이 끝날 때마다 연장하지 않는다.
  - (b) 시뮬레이터가 질문하면 그 질문이 **허용하는 함수 목록**과 함께 열고, 답 한 번으로 닫는다. 시뮬 revision이 바뀌면 버린다(구형 `sim_dialog.py:125-152`의 설계를 되살림).
  - 목록 밖의 함수가 호출되면 `blocked/not_an_answer`로 막는다.
- "야" 한 마디로 창이 열리는 것(`call_words`)은 기본값에서 끈다.

**프롬프트**
- 새 파일 `voice/prompts/system_v6.md`를 v5에서 만든다. 수정 모드 진입, 자동 이동(아래 S5), 깊이 거부 안내 문장을 넣는다.
- `--sim`이면 v6를 쓴다(`voice/app.py:44-47`).

**충돌 방지**
- 시뮬 게이트웨이의 `hello`에 `client: "chatkhu-voice"`를 요구한다. 구형 ROS `kuong_sim`은 같은 소켓 경로를 쓰고 1초마다 다시 접속해 새 음성 앱을 밀어내기 때문에, 이렇게 해서 거부한다.
- 음성 앱이 시작할 때 `pgrep -f manipulator_voice`로 구형이 돌고 있으면 경고하고 끝낸다.

**검증**
- `voice/` 텍스트 모드(`scripts/voice_text_client.py`)에서 문장별로 확인한다: 호출어 없는 말, "아니", 엉뚱한 말이 함수를 실행하지 않는지. `start_edit` 뒤 `edit_mode` 이벤트가 나오는지.
- `voice/tests/`를 새로 만들어 wake 창, 질문별 허용 목록, 실행 결과에 따른 안내 분기를 테스트한다.

**완료 기준**
- §4 시나리오 표의 음성 항목이 텍스트 모드에서 모두 기대한 결과를 낸다.

### S5. 검증 → 자동 이동 (S4 이후)
**"수정 완료"에 자동 실행 예약을 붙임**
- **수정** `voice/sim_link.py` finish_edit: `use_current_path(auto_run=True)`로 보낸다. 시뮬의 기존 예약 기구(`voice_gateway.py:188-212` → `arm_auto_run` → `watch` → `cmd_approve`의 재확인)를 그대로 쓴다.
- `ASK_PASSED`("이동할까요?"), `_ask_fp`, `approve_move`, `keep_editing`은 이 흐름에서 뺀다. 함수 선언은 남겨 두되 프롬프트에서는 쓰지 않게 한다.

**예약을 묶는 것**
- **수정** `Session`: 예약에 검증 토큰, 경로 fingerprint, revision을 함께 기록한다.
- `watch`가 실행하기 직전 이 셋과 로봇 관절 drift(기존 0.005 rad 기준)를 다시 확인한다.
- 다음 경우 예약을 취소한다: 편집, 상태 변경, `edit_mode` 변경, 게이트웨이 연결 끊김(기존), FAILED(기존), X 키.

**revision 확인을 우회하는 재시도 막기**
- `sim_link._command_fresh`가 STALE_REVISION에 자동으로 다시 보내는 것(`:152-157`)을 `use_current_path`에서는 금지한다.
- 명령에 `expected_path_fp`를 붙이고, 게이트웨이가 비교한다.

**실패를 보여 주기**
- **수정** `session.py`, `overlay.py`: 실패 보고서를 `session.last_failure`에 보관하고, FAILED 상태인 동안 계속 그린다. 지금은 `events[-1]`만 읽어서 바로 뒤에 다른 이벤트가 오면 사라진다(`overlay.py:62-66`).
- 쿠옹이는 실패한 검사 이름과 위치(가는 길 몇 %)를 말한 뒤 "다시 손가락으로 고쳐 주세요"라고 하고, 수정 모드로 돌아간다(`set_edit_mode(True)`).

**정지**
- 게이트웨이의 `stop` 명령은 deadline 검사에서 뺀다(`voice_gateway.py:92-94`). 메인 루프가 멈칫하면 REQUEST_EXPIRED로 거부될 수 있기 때문이다.
- 실행 중(EXECUTING)에는 `mute_mic_while_speaking`을 끄고, 실행 중 안내는 짧게 한다. 그래야 쿠옹이가 말하는 동안에도 "멈춰"를 듣는다. 스피커 소리로 스스로 멈추는지는 현장에서 시험한다.
- X 키는 늘 우선한다.

**수동 경로 유지**
- V(검증만), Y(승인), X(정지)는 바꾸지 않는다. Y는 PASSED에서 바로 실행한다(현행).

**테스트**
- `test_voice_gateway.py`, `test_validation_session.py`, `test_click_voice_edit.py`에 추가한다:
  - 예약 뒤 편집하면 실행되지 않는다.
  - 예약 뒤 연결이 끊기면 취소된다.
  - edit → undo로 같은 fingerprint가 돼도 revision이 달라 취소된다.
  - FAILED 뒤에도 빨간 점이 계속 남는다.
  - stop은 deadline이 지나도 실행된다.

**완료 기준**
- "수정 완료" 한 마디로 검증 → 자동 이동까지 이어진다.
- 위의 모든 무효화 경우에 움직이지 않는다.

### S6. 비전 목표 좌표 (고정 좌표 시연을 마친 뒤의 별도 단계)
**연결 방식: 직접 Python (ROS 2 아님)**
- 근거:
  - 시뮬레이터가 이미 pac_vision을 import하고 RGB와 depth를 혼자 소유한다.
  - ROS를 쓰면 카메라를 두 번 열게 되고, PYTHONPATH가 충돌한다.
  - 음성과 시뮬 어느 쪽도 ROS를 쓰지 않는다.
- `ros_pub.py`는 선택 기능으로 남긴다.

**구현**
- **새 함수** `pinch_sim/vision_goal.py`: 시뮬 프로세스가 이미 가진 최신 depth 프레임에 `pac_vision.objects`(물체 윗면 중심, base_ref)를 적용한다. 카메라를 따로 열지 않는다.
- 목표 고르기는 물체가 하나면 그 물체, 여럿이면 검지로 가리킨 물체로 한다.
- 키 G 또는 음성 함수 `set_target_from_camera`로 실행하고, 결과는 `cmd_ab`로 새 초기 경로를 만든다.

**좌표 정의 (명시적으로 나눔)**
| 이름 | 정의 |
|---|---|
| desk_point | (x, y, 0) |
| object_top | (x, y, z_top) |
| tcp_goal | object_top + work_height |
| approach | object_top + 0.12 m |

- 지금 코드는 목표가 책상 z=0에 있다고 가정한다(`--goal X,Y`, `session.py:326-327`). 그래서 `cmd_ab`와 검증 `goal_matches_target`에 목표 z를 받도록 확장한다.
- 물체 윗면 중심은 카메라 쪽으로 2.5–5 cm 치우치는 것이 알려져 있다(`INTERFACE.md:47-48`). 현장에서 재서 보정한다.

**카메라 중복 열기 방지**
- `$XDG_RUNTIME_DIR/pinch_sim/camera.lock`에 fcntl 잠금을 건다. 시뮬과 pac_vision 스크립트(`_ui.open_rgb`) 양쪽에 넣는다.

**완료 기준**
- 물체 3개 위치에서 tcp_goal 오차 ≤ 2 cm다.
- 그 목표로 검증을 통과하고 자동 이동한다.

### S7. 재현 문서·README 정리 (모든 단계와 병행, 마지막에 확정)
- 새 문서 `docs/PINCH_KHU_SETUP.md`를 `~/pinch_khu_ws` 기준으로 쓴다(§6).
- `README.md`를 고친다:
  - `oryise12/CHATKHU`, `~/CHATKHU`를 팀 저장소와 `work/pinch-khu-demo`로 바꾼다.
  - `--video 2`를 `auto`로 바꾼다.
  - unit1 프리셋을 무조건 복사하는 줄을 없앤다.
  - 로봇 배치 설명이 앞뒤로 다른 것(원점 배치 vs `--robot-pose=-0.15,0,0`)을 정리한다.
  - `check_setup` 기대 결과("⚠️ 3개" vs "1개")를 맞춘다.
- `voice/README.md`의 오래된 설명(자비스, 함수 5개)을 고친다.
- 코드 안의 `~/research/manipulator/...` 기본 경로(`pac_calib.py:40`, `test_pac_calib.py:10`)를 저장소 상대경로로 바꾼다.
- `.gitignore`에 `.env.*`, `*.wav`, `*.log`, `cam2base.check.yaml`, `robot_pose.yaml`, `*.key`를 추가한다.

---

## 4. 시나리오 표 (기대 결과)
| 상황 | 입력 | 기대 결과 |
|---|---|---|
| 호출어 없는 말 | "경로 수정할래" | 함수를 실행하지 않는다. 시뮬 상태도 그대로다(필요하면 쿠옹이가 "쿠옹아라고 불러 주세요"라고 한다) |
| 수정 모드 진입 | "쿠옹아, 경로 수정할래" | 시뮬 `edit_mode=True`가 되고 화면에 "수정 중 · 손 입력 켜짐"이 뜬 **뒤에** "네. 손가락으로 경로를 수정해 주세요." |
| 부정·엉뚱한 답 | 질문 뒤 "아니", "잠깐", 옆 사람 대화 | 함수를 실행하지 않는다. "네, 기다릴게요." |
| 수정 모드가 아닐 때 손 | 핀치 | 무시하고 "수정 모드가 아니에요" 표시. 마우스는 동작한다 |
| 검지 선택 | 경로 내부 점을 0.4 s 가리킴 | 노란 점으로 표시된다. 끝점(0, N-1)은 고를 수 없다 |
| 핀치 3D 수정 | 잡고 손을 3 cm 이동 | 그 점과 영향 영역(0.12 m)이 3D로 따라 움직이고, 끝점은 고정된다. 놓으면 새 revision이 된다 |
| 잡을 때 깊이 없음 | 깊이 무효로 핀치 | 잡지 않는다. "깊이를 못 읽었어요…"를 화면과 음성으로 알린다. 2D로 대체하지 않는다 |
| 드래그 중 깊이 상실 | 1 s 넘게 무효 | 일시정지했다가 취소하고 원래 경로로 돌아간다. 확정하지 않는다 |
| RGB·depth 시간차 | > 50 ms | 깊이 무효로 취급한다(위와 같다) |
| 되돌리기 | "쿠옹아, 이전으로" / U | 직전 편집 하나를 되돌리고 revision이 오른다. 자동 실행 예약이 있으면 취소한다 |
| 검증 실패 | "수정 완료" → FAILED | 빨간 실패 지점이 계속 남는다. 이유를 말하고 수정 모드로 돌아간다 |
| 검증 성공 | "이 경로로 할게" → PASSED | "통과했어요. 이동을 시작할게요." → 자동으로 EXECUTING |
| 예약 중 편집 | VALIDATING 중 손으로 수정 | 예약을 취소한다. 통과해도 움직이지 않는다 |
| 실행 중 정지 | "멈춰" / X | 즉시 ABORTED가 되고 자세를 유지한다. 그 뒤 "계속 갈까요, 돌아갈까요?" |
| 음성 연결 끊김 | 음성 앱 종료 | 예약을 취소한다. 화면의 버튼은 동작한다 |
| 카메라 이동 | 보정 뒤 카메라를 옮김 | 책상 z가 15 mm 넘게 어긋나면 "다시 보정하세요"가 뜨고 겹치기와 3D 핀치가 꺼진다 |
| 잘못된 보정 파일 | `passed:false`, 정합·장치 불일치 | 시작할 때 거부하고 이유를 출력한다 |
| 버튼 대체 조작 | E, 마우스 드래그, V, Y, X, U | 음성 없이 같은 Session과 같은 revision으로 시연 전 과정을 진행한다(V → PASSED → Y) |
| 구형 ROS 음성 동시 실행 | `kuong_sim` 실행 | 게이트웨이가 거부하고, 새 음성 앱은 경고 후 끝낸다 |

## 5. 미결정·현장 의존 사항 (추측으로 통과 처리하지 않음)
| 우선순위 | 항목 | 확인 절차 |
|---|---|---|
| 1 | Astra 모델·실물 번호(현재 노트북에 **미연결**) | 연결 뒤 `lsusb`(2bc5:…), `/sys/class/video4linux/*/name`, `check_setup.py`. 실물 라벨을 기록한다 |
| 1 | unit1 정합 프리셋이 이 카메라에 맞는지 | S1의 클릭 쌍 8개 이상. 잔차 ≤ 2 px면 그대로 쓰고, 아니면 새로 맞춘다 |
| 1 | `BAZE_API_KEY`(현재 셸과 `~/.bashrc`에 **없음**) | 사용자가 `~/.config/pinch_khu/env`(권한 600, 저장소 밖)에 넣고 `voice/env.sh`가 source한다. 값은 출력하지 않는다 |
| 2 | 보정 허용오차(holdout 10 mm, 변 5 mm, 독립점 10 mm, 재투영 8 px) | 첫 보정 3회의 분포를 보고 확정한다. 넘으면 카메라 배치(0.6 m 이상)부터 다시 한다 |
| 2 | 로봇 배치 (X, Y, ψ) | 자로 재고 TCP를 투영해 확인한다. 원점에 배치한다는 가정을 쓰지 않는다 |
| 2 | 손 3 cm 이동의 실측 오차 | 같은 동작을 10회 반복해 변위의 중앙값과 최대값을 기록한다(목표 ≤ 1 cm) |
| 2 | 동기화 기준(50 ms / 150 ms), 끊김 1 s | 현장 로그(t_rgb, t_depth)의 분포로 조정한다 |
| 3 | 실행 중 마이크를 켰을 때 스피커 소리로 스스로 멈추는지 | 스피커 볼륨별로 5회씩 시험한다. 문제가 되면 이어폰을 쓰거나 X 키를 기본으로 한다 |
| 3 | 물체 목표의 정의와 치우침(2.5–5 cm) | S6에서 물체 3종 × 3개 위치를 잰다 |
| 3 | Wayland와 NVIDIA에서 MuJoCo 창 | S0 gui_smoke로 확인한다 |
| 미확인 경계 | 카메라가 없거나 API 키가 없을 때 | 해당 현장 검증을 "미확인"으로 분리해 보고한다. 마우스와 텍스트 대체 시연만 확인한다 |

## 6. 재현 문서 계획 (`docs/PINCH_KHU_SETUP.md`)
1. **복제·환경:** §3 S0의 명령을 그대로 적는다(`~/pinch_khu_ws/26_project_manipulator`, `work/pinch-khu-demo`, apt, vendor redist, udev, 두 venv, env.sh).
2. **카메라 확인:** `check_setup.py`, 장치 이름 목록.
3. **정합과 보정:** `check_camera.py` → `fit_rgb_to_depth.py` → `calibrate.py` → `calibrate.py --check`. 각 단계의 통과 기준을 적는다.
4. **실행 (터미널 2개):**
   - 터미널 1: `source mujoco_sim/scripts/env.sh && python -m pinch_sim.app --calib ../pac_vision/config/cam2base.yaml --video auto --robot-pose=<측정값> --start … --goal … --hand --depth --voice-gateway`
   - 터미널 2: `source voice/env.sh && python -m voice --sim`
5. **대체 시연:**
   - 보정 파일이나 카메라가 없을 때: `python -m pinch_sim.app --target-r 0.275 --target-deg 10 --voice-gateway`(시뮬 카메라 + 마우스).
   - 마이크나 API 키가 없을 때: `scripts/voice_text_client.py`로 텍스트 명령을 보낸다.
6. **문제 해결:**
   - 카메라를 두 번 열었을 때: `fuser /dev/videoN`.
   - redist 경로, udev 다시 꽂기.
   - ROS PYTHONPATH.
   - `MUJOCO_GL`, prime offload.
   - 음수 인수는 `=`로 붙여 쓴다.
7. **테스트 결과 기록:** 각 패키지의 pytest 통과 수, 현장 측정값(보정, 3 cm, 재투영), 미확인 항목.

## 7. 검증 요약 (구현 후 끝까지 확인)
1. **자동 테스트:** `mujoco_sim`, `pac_vision`, `path 편집`, `voice`의 pytest가 모두 통과한다. 새 테스트는 §3 각 단계에 적은 것이다.
2. **카메라 없는 통합:** 텍스트 클라이언트와 마우스로 §4 표의 음성·버튼 항목을 시험한다. 이때 "수정 완료 → 자동 이동"과 무효화 경우를 포함한다.
3. **현장 (Astra와 마이크):** 다음 순서로 진행하고 각각 기준을 기록한다. 보정 → 독립 검증점 → 재투영 → 손 3 cm → 음성 전 과정 → 카메라를 옮겨 무효화 확인.
4. 구현·커밋 뒤 푸시는 사용자 승인을 받은 다음 `work/pinch-khu-demo`로만 한다.
