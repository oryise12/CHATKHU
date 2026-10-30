# XM430 매니퓰레이터 제어 준비 루틴 — 최종 설계

- 갱신: 2026-10-26
- 상태: **소프트웨어 구현, 가상 검증, 실기 읽기 전용 조회(scan·watch, 모터 쓰기 0회) 완료. 실기 구동(토크 ON·HOME)은 미실행**([IMPLEMENTATION_REPORT.md](IMPLEMENTATION_REPORT.md) 참고)
- 근거 요청서:
  - `MANIPULATOR_CONTROL_PLAN_REQUEST (1).md` (PLAN 단계)
  - `MANIPULATOR_CONTROL_EXECUTION_REQUEST.md` (구현 단계, 보완사항 우선)
- 참조 원본(읽기 전용): `/home/jammin/pinch_khu_ws/26_project_manipulator` (`work/pinch-khu-demo` @ `eeaca7f`, 작업 트리 clean)
- 공식 사양: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/

목표 동작은 다음 순서예요.

1. 사용자가 관절 마킹을 기준 자세에 맞춰요.
2. `prepare`를 실행하고 Enter를 눌러요.
3. 이번 세션의 원점을 등록하고, 실제 STATE를 MuJoCo에 표시해요.
4. 현재 자세를 유지한 채 토크를 켜요.
5. HOME으로 부드럽게 이동하고, 실제 도착을 확인한 뒤 READY가 돼요.

이번 범위는 제어 준비 루틴까지예요. 작업 궤적(Pick & Place, 경로 편집, 비전, 음성)은 구현하지 않았고, READY 이후 연결 지점만 정했어요(§7.5).

---

## 1. 조사 사실

### 1.1 PLAN 단계 조사와 구현 단계 재확인 (둘 다 2026-10-26)

| 항목 | 사실 | 구현 단계 재확인 |
|---|---|---|
| OS·ROS·Python | Ubuntu 24.04.5, ROS 2 Jazzy(`~/.bashrc:118`), Python 3.12.3 | 같음 |
| 전용 환경 | `~/manipulator_control_ws/.venv`(system site 미포함). dynamixel-sdk 4.0.3, pyserial 3.5, mujoco 3.15.0, numpy 1.26.4, PyYAML 6.0.3, pytest 9.1.1 (`requirements.lock`) | 구현 단계에서 생성 |
| 장치 | U2D2(FT232H) `/dev/ttyUSB0`. 고정 경로 `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HROZ-if00-port0`. latency_timer 16 ms. jammin은 dialout 그룹. ModemManager active | 같음 |
| 버스 점유 | PLAN 단계 내내 DYNAMIXEL Wizard(PID 45245)가 포트를 점유했고, 이 작업공간 도구는 포트를 열지 않았음 | `scan`이 PortBusy로 중단하는 것을 확인. 15:21 사용자가 Wizard를 닫은 뒤 읽기 전용 `scan`·`watch` 실행 |
| 서보 | (Wizard 화면) 1 Mbps, Protocol 2.0, ID 1~6 모두 XM430-W210(1030). ID1: FW 50, Drive Mode 0, mode 4, Homing Offset 0 | **`scan` 실측 (6축 모두 같음):** FW 50, mode 4, Drive Mode 0, Startup 0, Bus Watchdog 0, Homing Offset 0, HW Error 0, SRL 2, Return Delay 500 µs, Secondary ID 255, Velocity Limit 330, PV/PA 0/0, 기본 게인, 12.2~12.3 V. 서보 설정 때문에 prepare가 거부될 사유 없음 |
| 사용자 확인 | ID n = Revolute_n, 6축 모두 추가 16:1, 그리퍼 모터 없음. 토크 OFF에서 손으로 돌아가지만 **자중으로 처짐**. 정렬 중에는 손·지그로 받침. 종료 기본 = 자세 유지. 표시는 MuJoCo만 | — |
| 참조 저장소 | 시뮬레이터 전용. 다이나믹셀 코드는 이 노트북 어디에도 없음 | 같음 |
| 모델 | 필요한 모델 파일 28개 모두 `eeaca7f`에 추적됨. 미커밋 변경 없음 | `tools/copy_model.py`: blob 해시 일치, 원본 고정 해시 12/12 일치 |

### 1.2 모델 (`eeaca7f`)

- 모델 파일 5개(`6dof.urdf`, `6dof.xacro`, `6dof_gripper.urdf`, `6dof_gripper.xacro`, `mujoco/6dof_mj.urdf`)의 관절 기구학이 완전히 같아요.
  - 관절 6개(Revolute_1..6) 모두 revolute이고, 한계는 ±3.14159(임시값), velocity는 3.0이에요. mimic이나 fixed 관절은 없어요.
  - 차이는 xacro의 materials include, 그리퍼 버전의 housing_1 블록(gripper.stl, 0.22 kg 가정), `6dof_mj.urdf`의 `<mujoco>` meshdir뿐이에요.
  - `tools/build_viz_model.py`가 관절 단위로 비교했고 차이가 없었어요.
- `eeaca7f`는 관절 6개의 원점을 각 관절축 방향으로 2 mm씩 옮긴 커밋이에요.
- 축 방향은 R1·R5 = +z, R2·R4 = +x, R3·R6 = −x예요. q = 0이면 팔이 위로 곧게 서요.
- 원본의 `build_scene`은 원본 `urdf/6dof_gripper.{urdf,xacro}`를 덮어써요. 이 작업공간에서는 실행하지 않아요.
- 표시용 측정 팔의 body/joint 프레임은 `6dof_control.xml`, MuJoCo가 컴파일한 `6dof_mj.urdf`, `pinch_scene.xml` 모두와 같아요.
- ready 자세의 TCP FK는 (0.094294, 0.100206, 0.25) m로 원본과 일치해요.

### 1.3 XM430-W210 e-manual에서 확인한 사항

- **Present Position:** 전원 인가·재부팅 시, 그리고 mode 3으로 바꾸거나 mode 3에서 토크를 켤 때 1회전 안으로 리셋돼요. 토크 OFF에서는 signed 32-bit 연속값이에요.
- **Operating Mode 변경:** 게인이 그 모드의 기본값이 되고, PV/PA는 0이 돼요.
- **mode 4:** Min/Max Position Limit을 쓰지 않아요. Goal 범위는 ±1,048,575(±256 rev)예요.
- **PV·PA:** PV 1 = 0.229 rpm, PA 1 = 214.577 rev/min²이고, 0은 둘 다 무한이에요. 가속 시간 t1 ≈ 64·PV/PA [ms]예요.
- **이동 중 Goal 변경:** 현재 속도에서 새 프로파일을 만들어 이어가요.
- **Moving Status(123):** bit0 In-Position, bit1 Profile Ongoing, bit3 Following Error예요.
- **Velocity Trajectory(136) / Position Trajectory(140):** 서보 내부 목표 궤적이에요. 실제 위치·속도가 아니에요.
- **Drive Mode(10):** bit0 역방향, bit2 시간 기반 프로파일(FW 42+), bit3 Goal 갱신으로 토크 ON(FW 45+)이에요.
- **Startup Configuration(60, FW 45+):** bit0 Startup Torque On, bit1 RAM Restore예요.
- **Bus Watchdog(98, FW 38+):**
  - 단위는 20 ms이고 0이면 비활성이에요. 토크 ON일 때만 감시해요.
  - 어떤 Instruction Packet이든 받으면 타이머가 다시 시작돼요.
  - 시간이 초과되면 PV/PA를 0으로 적용해 정지하고, 값이 −1이 돼요. 그 뒤 Goal 쓰기는 Data Range Error로 거부되고, 0을 써야 풀려요.
- **EEPROM:** 토크 OFF일 때만 쓸 수 있어요.
- **mode 4의 출력 제한:** Current Limit/Goal Current는 전류 상한으로 적용되지 않아요. 출력을 제한하는 것은 PWM Limit뿐이에요.

## 2. 확정 / 구현 선택 / 확인 필요

| 구분 | 내용 |
|---|---|
| 확정 | 포트(by-id)·1 Mbps·Protocol 2.0, ID n = Revolute_n, g = 16(6축), 목표 Operating Mode 4, 모델 `eeaca7f`, 그리퍼는 고정 메시 |
| 구현 선택 | ROS 없는 독립 Python 패키지 `manipulator_control` + 전용 venv · HOME 궤적은 서보 내부 프로파일이 만들고 PC는 계획·감시(§7) · 터미널(한글 안내·키) + MuJoCo 운동학 뷰어(영문 오버레이) · 단일 버스 소유 스레드 · 종료 기본 = 정지 확인 후 자세 유지 |
| 확인 필요 | 관절별 부호 s, 기준 자세 각도 q_ref, 실물 가동 범위, HOME 후보의 실물 안전성과 전체 경로, 손으로 받친 상태의 정렬 흔들림, 백래시·마킹 반복 오차. (서보 레지스터·통신 주기·정지 잡음은 `scan`으로 확인됨: Sync Read p50 16.0 ms / p99 16.2 ms, 정지 잡음 1 count) |

`config/robot.yaml`에서 확인 필요 값은 모두 `null`이고, 프로그램은 `null`을 0으로 해석하지 않아요.

## 3. 시스템 구성과 데이터 흐름

```
U2D2 (by-id 경로) ⇄ bus.DxlBus  ── 유일한 포트 소유자 (점유 검사·flock·TIOCEXCL·명령어 가드)
        │ Sync Read 108..146 매 주기(20 Hz 목표): PA, PV, Goal, Tick, Moving, Moving Status, PWM, Current,
        │   Velocity, Position, Velocity Trajectory, Position Trajectory, 전압, 온도 (+ 상태 패킷 alert 비트)
        │ Sync Read 10..98 2 Hz(+요청 시): 모드·Drive·Homing Offset·Startup·Shutdown·Torque·SRL·HW Error·게인·Watchdog
        ▼
runner.Runner (버스 스레드)  1주기 = 읽기 → supervisor.step → 명령 실행 → View 갱신 → 샘플 로그(솎음)
        │                     쓰기는 이 스레드에서만. 명령이 실제로 나간 시각을 supervisor에 알림
        ▼
supervisor.Supervisor  상태 머신 (calibration·planner 사용), 결정론적: step(now, fast, slow, events) → 명령
        ├──► viz.StateViewer (메인 스레드, 30 Hz): 측정 q → qpos(관절명 매핑) → mj_kinematics (물리 없음)
        ├──► terminal (입력 스레드): 키 이벤트 + 시각, 상태 진입 시 입력 버퍼 비우기
        └──► logs.SessionLog: 이벤트 전부, 샘플은 이동 중 10 Hz / 대기 1 Hz
```

- **버스 소유:**
  - 시작할 때 `/proc/*/fd`를 확인하고, 다른 프로세스(Wizard 등)가 포트를 잡고 있으면 거부해요. 그 프로세스를 종료하지는 않아요.
  - 포트를 연 뒤 `TIOCEXCL`을 걸고, 이 프로그램의 중복 실행은 flock으로 막아요.
  - UI나 뷰어가 지연되더라도 버스 스레드의 감시와 정지는 멈추지 않아요.
- **쓰기 가드:** 주소 단위(`WritePolicy`)와, 직렬 포트로 나가기 직전 패킷의 명령어 단위(`InstructionGuard`) 두 겹이에요.
  - read_only: PING, READ, SYNC_READ만 나가요.
  - control: WRITE와 SYNC_WRITE가 추가돼요. 쓸 수 있는 주소는 64(토크), 65(LED), 108..119(PA/PV/Goal)이고, 설정이 허용할 때만 11(Operating Mode)도 쓸 수 있어요.
  - REBOOT, FACTORY_RESET, CLEAR는 어떤 모드에서도 나가지 않아요.
- **SIM 표시:** 전용 모델 `models/derived/viz_scene.xml`을 써요.
  - 회색 실선: 측정 팔(엔코더에서 계산한 각도). 미보정이면 하늘색으로 그리고 "UNCALIBRATED"를 표시해요.
  - 주황 반투명: 서보 내부 목표(Position Trajectory). 토크 ON 상태에서만 보여요.
  - 초록 반투명: 목표 자세. 대기 중에는 기준 자세, 이동 중에는 HOME이에요.
  - 고스트와 장식용 바닥은 충돌하지 않고, 충돌 판단의 근거로도 쓰지 않아요. `--no-cmd-ghost`, `--no-target-ghost`로 끌 수 있어요.
- **ROS:** 쓰지 않아요(사용자 선택). 나중에 `/joint_states`를 발행한다면 View 스냅샷을 받는 별도 프로세스로 만들고, `joint_state_publisher_gui`와 동시에 실행하지 않아요.

## 4. 관절-모터-SIM 매핑

| URDF 관절 | 구동 | 모터 ID | 추가 감속 | 부호 s | 기준 자세 q_ref | HOME(후보) | 관절 범위 | SIM 대응 |
|---|---|---|---|---|---|---|---|---|
| Revolute_1 (+z) | 구동 | 1 | 16 | 사용자 측정 필요 | 사용자 측정 필요 | 0.000° | 사용자 확인 필요 (URDF ±180°는 임시값) | 이름 `Revolute_1` → `jnt_qposadr` (측정 0, cmd_ 6, tgt_ 12) |
| Revolute_2 (+x) | 구동 | 2 | 16 | 〃 | 〃 | 36.743° | 〃 | `Revolute_2` (1 / 7 / 13) |
| Revolute_3 (−x) | 구동 | 3 | 16 | 〃 | 〃 | 75.193° | 〃 | `Revolute_3` (2 / 8 / 14) |
| Revolute_4 (+x) | 구동 | 4 | 16 | 〃 | 〃 | −51.550° | 〃 | `Revolute_4` (3 / 9 / 15) |
| Revolute_5 (+z) | 구동 | 5 | 16 | 〃 | 〃 | −90.000° | 〃 | `Revolute_5` (4 / 10 / 16) |
| Revolute_6 (−x) | 구동 | 6 | 16 | 〃 | 〃 | −8.008° | 〃 | `Revolute_6` (5 / 11 / 17) |
| gripper (housing_1) | 고정·비구동 | 없음 | — | — | — | — | — | 메시만 (Revolute_6 자식) |

- HOME 후보는 시뮬 ready 자세(`demo.json` `ready.q_rad` @ `eeaca7f`)예요. 원본에서 값과 모델을 다시 확인했고 TCP FK도 일치해요. 다만 시뮬레이터용으로 고른 값이라 `home.confirmed: false`로 두었어요.
- qpos 주소는 관절 이름으로 찾아요. 표의 숫자는 현재 빌드 결과일 뿐, 코드가 이 숫자에 의존하지 않아요.

## 5. 캘리브레이션과 좌표 변환

```text
q_rad  = q_ref_rad + s * (P - P_ref) * 2*pi / (4096*g)
P_goal = P_ref + s * (q_goal_rad - q_ref_rad) * (4096*g) / (2*pi)      (반올림, Goal 범위 ±1,048,575 검사)
```

- **단위:**
  - 카운트는 signed 32-bit이고, `% 4096`으로 자르지 않아요.
  - 내부 단위는 rad·rad/s이고, deg는 설정 입력과 UI에서만 써요.
  - g = 16이고, 서보 내부의 212.6:1은 곱하지 않아요. 4096 count = 관절 22.5°예요.
- **부호 s:** 서보가 보고하는 raw Present Position을 기준으로 정의해요.
  - Drive Mode 역방향 비트나 감속기 방향 반전을 따로 보정하지 않고 s 하나로만 다뤄요.
  - Drive Mode가 설정의 기대값과 다르면 거부해요.
  - Homing Offset은 읽고 기록만 하며 쓰지 않아요. P_ref가 이 값을 그대로 흡수해요.
- **Enter의 의미:** "지금 마킹에 맞춰져 있다"는 선언이에요. 알려진 q_ref와 이번 세션의 카운트 P_ref를 연결할 뿐, 자세를 자동으로 인식하거나 모르는 q_ref를 알아내지 않아요. 마킹을 정렬할 때 생긴 오차는 그대로 관절 영점 오차가 돼요.
- **등록 판정:** 진입 후 연속 20개의 완전한 샘플(20 Hz에서 1 s)이 다음을 모두 만족해야 해요. P_ref는 중앙값으로 정해요. 실패하면 이유를 보여주고 다시 Enter를 기다려요(FAULT 아님).
  - 관절별 max−min ≤ 8 count(0.044°), |Present Velocity| ≤ 2 raw
  - 상태 패킷 경보 없음, 토크 OFF, mode 4, Drive Mode 변화 없음
  - 전압 11.4~12.8 V, 온도 ≤ 55 °C
- **저장과 재사용 (2026-10-26 18:00 변경, 사용자 요청):** 원래는 세션 한정이었어요(P_ref를 다음 실행에서 읽지 않음). 지금은 최근 calibrate 하나만 `runtime/calibration_state.json`에 저장해요.
  - `Calibration`은 두 경우에만 만들어져요. 하나는 판정을 통과한 실시간 창이고, 다른 하나는 아래 근거를 통과한 저장 기록(`Calibration.resume`)이에요.
  - 근거는 다음과 같아요.
    - 서보마다 PA/PV 읽기값이 이 프로그램이 마지막으로 보낸 값과 정확히 같아요(PV/PA ≠ 0). Goal은 비교하지 않아요(토크 OFF 동안 서보가 Goal = Present로 유지, 실측). 재부팅하면 PV/PA = 0, 토크 OFF가 되고, 이 프로그램은 0을 쓰지 않아요.
    - 관절 ID·감속비·부호·q_ref가 보정할 때와 같아요.
    - Operating/Drive Mode와 Homing Offset이 같아요.
    - 토크 ON인 축은 목표에서 0.5° 이내예요.
  - calibrate는 시작할 때 기록을 지우고, 6개 관절을 고정하면 저장해요.
  - 정지 확인 뒤 유지(HOLDING/READY)와 종료 때 근거를 다시 저장해요.
  - 재설정 근거(INVALIDATED, MODE)나 통신 단절(COMM_LOST, 사용자 규칙: 끊기면 다시 calibrate)이 나오면 지워요.
  - 토크 ON 시 서보가 Goal = Present로 바꾸는 동작(실물에서 관찰)은 토크 ON 확인 뒤 읽기값으로 근거를 갱신해서 처리해요.
  - prepare / teleop은 정렬 없이 [RESUME_HOLD: 토크 OFF 관절은 y 뒤 그 자리 고정] → HOME 미리보기 + y 순서예요.
- **무효화:** 다음 중 하나라도 생기면 보정을 무효로 하고, 자동으로 다시 등록하지 않아요.
  - Realtime Tick 불연속(>150 ms), 위치 점프
  - 보낸 PV/PA와 읽은 값의 불일치(재부팅하면 0으로 돌아감)
  - 예기치 않은 토크 OFF
  - mode·Drive Mode·Homing Offset 변화, 새로 생긴 HW 오류·경보
  - 완전한 읽기 공백 > 0.5 s

## 6. 초기화 순서와 상태 전이

`CONNECTED_CHECK → [CONFIRM_RELEASE] → [MODE_PREP] → WAIT_REFERENCE → CALIBRATING → HOLD_CURRENT → COUNTDOWN → HOMING → READY`

| 상태 | 하는 일 | 다음으로 | 실패 |
|---|---|---|---|
| (시작 전) | `prepare`가 설정의 차단 사유를 검사. 하나라도 있으면 **포트를 열지 않고** 종료(코드 2). 실기는 대화형 터미널에서만 실행(무인 자동 시작 금지) | 차단 사유 없음 | — |
| 연결 | 점유 검사 → 포트 열기 → ID 1~6 ping → 설정·상태 읽기 | 6축 모두 응답 | 누락 ID를 알리고 종료(다른 ID로 대신하지 않음) |
| CONNECTED_CHECK | 모델 1030, Protocol 2, Drive Mode bit2/bit3 = 0, Drive Mode = 기대값, Startup = 0, SRL = 2, Watchdog = 0, HW Error = 0 | 통과 | FAULT `SERVO_SETTINGS`/`CONFIG` (자동 수정 없음) |
| CONFIRM_RELEASE | 토크가 켜진 축이 있을 때만. "팔을 받친 뒤 y" | y → 토크 OFF를 읽어서 확인 | 확인 실패 → FAULT |
| MODE_PREP | mode ≠ 4이고 `allow_operating_mode_write: true`일 때만. y → 토크 OFF → Operating Mode 4 쓰기 → 읽어서 확인. 게인·PV/PA 초기화를 기록 | 모든 축 mode 4 | 허용 안 됨·실패 → FAULT `MODE` |
| WAIT_REFERENCE | 진입할 때 터미널 입력 버퍼를 비우고, 진입 뒤 새로 누른 Enter만 받음. 토크 OFF(손·지그로 받침) | Enter | — |
| CALIBRATING | §5의 판정 | 통과 → P_ref 등록 | 거부 → WAIT_REFERENCE |
| HOLD_CURRENT | ① 최신 카운트로 [PA 3, PV 29, Goal = Present]를 한 패킷으로 쓰고 읽어서 확인 ② 토크 ON, 6축 ON 확인 ③ 0.6 s 동안 점프 ≤ 91 count(0.5°), 끝의 \|P − Goal\| ≤ 36 count(0.2°) | 통과 → 곧바로 HOME 계획(§7). 계획이 거부되면 이동 없이 FAULT `PLAN_REFUSED` | Goal 확인 실패 → 토크를 켜지 않고 FAULT. 점프·일부 축만 토크 ON → 취소 정지 후 FAULT |
| COUNTDOWN | 같은 Enter 루틴 안에서 3 s 동안 "손을 떼세요"를 안내. s/space를 누르면 취소하고 HOLDING. 두 번째 Enter는 필요 없음 | 3 s 경과 | — |
| HOMING | §7 계획대로 [PA, PV, Goal]을 한 번 쓰고, 읽어서 확인한 뒤 감시 | 모든 축이 \|q − q_home\| ≤ 0.2°, Moving = 0, Profile Ongoing = 0, 위치 퍼짐 ≤ 8 count를 1.0 s 연속 유지 → READY, `home_report.json` 기록 | §8 |
| READY / HOLDING | 위치 유지 감시(드리프트 > 0.5° → FAULT `DRIFT`). 중복 Start는 무시 | q → 종료(유지), x → 받친 뒤 토크 OFF | — |

- `prepare --hold-only`는 HOLD_CURRENT를 확인한 뒤 HOLDING에서 멈춰요. 토크 ON 시험용이에요.
- 모드·프로파일 준비처럼 카운트 기준을 바꿀 수 있는 조작은 모두 원점 등록 전에 끝나요.
- HOME 중에 들어온 Enter(재보정·재시작 요청)는 거부하고 로그에 남겨요.

## 7. HOME 궤적: 서보 내부 velocity-based 프로파일

### 7.1 역할

- PC는 공동 이동 시간과 가속 구간을 계산하고, 축별 [PA, PV, Goal]을 Sync Write 한 패킷으로 **한 번** 보낸 뒤 감시만 해요.
- 궤적은 서보 내부 프로파일이 만들어요. PC가 목표를 연속으로 보내지 않으니 두 프로파일이 충돌하지 않아요.
- 이렇게 정한 이유: FTDI 지연 16 ms와 Return Delay 500 µs × 6축 환경에서 PC가 목표를 연속 송신하면, 프로파일을 0으로 두면 계단식 진동이 생기고 프로파일을 켜면 재계획 지연이 생겨요.

### 7.2 공동 시간축과 정수화

1. **축별 상한 raw:** PV_cap = min(262, floor(v_home / 0.085875 °/s)), PA_cap = floor(a_home / 1.3411 °/s²)예요.
   - 기본값 v_home 2.5 °/s는 PV 29예요.
   - a_home 5 °/s²는 **PA ≤ 3 (4.02 °/s²)**예요. PA 4는 5.36 °/s²라 상한을 넘으므로 쓰지 않아요.
   - PA_cap < 1이면 거부해요.
2. **공동 시간:** 상한에서 가장 오래 걸리는 축이 공동 시간 T와 가속 시간 t_acc를 정해요.
3. **나머지 축:**
   - 속도는 v_i = |ΔP_i|/(T − t_acc)이고, PV_i는 1 ≤ PV_i ≤ PV_cap 범위에서 반올림해요.
   - PA_i는 1..PA_cap 중에서 예측 도착 시각이 T에 가장 가까운 값을 골라요.
   - 한 축이 한계에 걸리면 T 전체를 늘려요. 축마다 따로 잘라서 상대 진행률을 바꾸지 않아요.
   - 이동량이 0인 축은 Goal = 현재 위치, PV/PA = 유지값이에요(PV = 0 금지).
4. **실제로 보낼 정수 PV/PA/Goal로 다시 계산:** 축별 사다리꼴/삼각 궤적, 도착 시각, 최대 속도·가속도, 이상적인 공동 궤적과의 관절각 차이, 진행률 차이, TCP 경로 차이예요.
5. **동기화 편차는 경고만:** 양자화로 생긴 편차가 2°를 넘으면 경고하지만 FAULT로 처리하지 않아요. 이 기준은 실제 추종 오류의 임계값과 따로 관리해요.
   - 작은 축이 먼저 도착하더라도 아래 검사를 모두 통과하면 그대로 진행해요.
   - 예: 가상 설정 2.5 °/s에서 T 37.0 s, R6이 5.7 s 먼저 도착, 최대 편차 1.16°, TCP 경로 차이 3.8 mm(`preview-home --fake`)였어요.
   - 추가 검토의 "1° 이동, 10 s에서 약 4 s 먼저 도착"은 상수가 아니라 예시예요. 실제로 고른 PV/PA로 매번 다시 계산해요.
6. **거부 조건:** 다음은 단순한 동기화 경고가 아니라 이동을 거부해요.
   - Goal이 레지스터 범위 밖
   - 현재 자세나 HOME이 확정된 가동 범위 밖
   - 예측 경로나 제동 지점이 범위 밖
   - 정수화한 뒤 속도·가속 상한 초과
   - 모델 기준 자기충돌, 또는 장착면 아래로 내려감(참고 검사)
   - 원래 계획에 있던 "[min(q_ref, q_home) − 5°, max + 5°] 임시 허용 범위"는 **삭제했어요**. 범위가 확정되기 전에는 실기 이동이 없어요.
7. **모델 경로 검사의 한계:** STL을 볼록 껍질로 다루는 모델 기준 검사라 책상·케이블·사람은 검사하지 못해요. 그래서 사용자가 실물 경로를 확인한 경우에만 `home.path_confirmed: true`로 설정해요.

### 7.3 가감속의 한계

- 서보 프로파일은 사다리꼴(또는 삼각형)이에요. 위치와 속도는 연속이지만, 구간 경계에서 가속도가 0 ↔ ±a로 바뀌어요(저크 무제한).
- 가속도를 낮춰 충격을 줄일 뿐, 저크를 제한하는 구현은 아니에요.
- 연속 가속도나 저크 제약이 필요하면 PC의 S-curve/minimum-jerk와 연속 목표 송신을 후속 과제로 분리해요.
- FTDI latency 1 ms와 100 Hz 이상은 그 방식의 성능 목표 후보일 뿐이에요. 실제 지연·jitter를 측정해야 하고, sudo가 필요한 udev 변경은 이번 구현의 필수 조건이 아니에요.

### 7.4 감시 (HOMING 중 매 주기)

- **명령 확인:** 보낸 [PA, PV, Goal]을 읽어서 같은지 확인해요. 다르면 취소 정지 후 FAULT `HOME_CMD_MISMATCH`예요.
- **실제 추종 오류:** |Present − Position Trajectory| > 2° 또는 Following Error 비트예요. 해당 축을 취소 정지(§8)하고 FAULT `TRACKING`이에요.
- **전송 프로파일 예측과의 차이:** > 3°이면 감속 정지 후 FAULT `PREDICTION`이에요. 예측의 기준 시각은 명령이 실제로 나간 시각이에요.
- **범위·방향·속도:** 다음 중 하나면 감속 정지 후 FAULT예요.
  - 가동 범위 이탈, 또는 이동 구간 [q0, q_home] ± 1° 이탈
  - 반대 방향 이동 > 0.5° (부호·ID 오류 의심)
  - 과속: > 1.15 × 계획 속도 + 0.5 °/s, 또는 > 1.1 × 22.5 °/s
- **시간 초과:** 1.5 × 예측 도착 시간 + 3 s를 넘기면 도착하지 못한 축을 취소 정지하고 FAULT `TIMEOUT`이에요.
- **도착 판정:** §6의 READY 조건으로만 판정해요. 경과 시간이나 명령 전송 완료만으로 성공 처리하지 않아요.

### 7.5 후속 궤적 연결 지점

- READY와 HOLDING은 보정이 유효하고 정지가 확인된 상태예요.
- 작업 궤적 실행기를 붙인다면 이 두 상태에서만 시작하고, 궤적 소유 방식(서보 프로파일 / PC 연속 송신)을 그 실행기에서 명시하는 형태로 계획해요. 이번에는 구현하지 않았어요.
- **2026-10-26 Pinch 실행기(`pinch`):** READY에서 시작해요(시뮬 연결 → 터미널 y → ARMED).
  - 궤적 소유: **PC 연속 송신**이에요. 20 Hz로 Goal = 궤적의 다음 0.25 s 안 값, PV = 필요한 속도 + 보정, PA 고정이에요.
  - 서보 프로파일은 앞섬 구간만 채워요. 통신이 끊기면 그 구간 끝(최대 5.6°)에서 멈춰요.
  - §7.1/§7.3의 연속 송신 경고는 앞섬 조건(lead ≥ 2/read_hz + v_cap/2a)과 틱마다의 추종·구간·속도·명령 읽기값 감시로 다뤄요.
  - 자세한 내용은 README "Pinch 시나리오"를 봐요.

## 8. 정지 · FAULT · 통신 단절 · 종료 (원래 PLAN의 "목표 갱신 중지 → 자세 유지"를 교체)

내부 프로파일로 이미 진행 중인 HOME은 목표 갱신을 멈춘다고 멈추지 않아요. 그래서 모든 정지는 **명시적 쓰기 → 읽어서 확인 → 실제 정지 확인 → HOLDING** 순서를 거쳐요. 상태를 바꿨다는 것만으로 실제로 멈췄다고 판단하지 않아요.

### 8.1 통신이 가능할 때

- **정상 감속 정지:** 정지 키, 예측 불일치, 범위·방향·과속 위반, Ctrl+C, 창 닫기, HOMING 중 일반 예외일 때 써요.
  - 같은 시점의 신선한 스냅샷에서 Position Trajectory(140)와 Velocity Trajectory(136)를 읽어 계산해요.
  - 정지 지점 = x + sign(v)·⌈v²/(2a) + |v|·δ + 1.5·a·δ² + 2⌉ (count, δ = 쓰기 지연 여유 0.1 s)이에요. 현재 Goal을 넘지 않게 하고, 각 축의 현재 PA를 써요.
  - 계획에서 t_acc가 모든 축에 공통이라 축들이 거의 동시에 서고, 관절 공간에서의 방향이 유지돼요.
  - 제동 지점이 가동 범위를 넘으면 더 강한 감속(상한 15 °/s²)을 시도해요. 그래도 넘으면 범위에서 잘라내지 않고 "범위 여유 부족" 경고와 함께 보내요. 경계에서 잘라냈다고 정지가 보장된다고 보지 않기 때문이에요.
  - 이 식은 모델에 기반한 후보예요. 역행이나 오버슈트가 없다고 보장하지 않으며, 가상 시험에서만 역행이 없었어요.
- **취소 정지:** 걸림, 시간 초과, 큰 추종 오차일 때 써요.
  - 해당 축에 Goal = 최신 Present Position을 써서 이동 목표를 취소해요. 앞쪽으로 목표를 늘리지 않아요. 나머지 축은 감속 정지해요.
  - 이 쓰기로 전류나 접촉력이 0이 된다고 보지 않아요. 실제 정지 여부와 잔여 오차(`residual_counts`)를 기록해요.
- **정지 확인:**
  - 정지 명령의 Goal을 읽어서 확인한 뒤, |v| ≤ 1 raw, Moving = 0, Profile Ongoing = 0이 0.3 s 연속이면 HOLDING이에요.
  - 확인에 실패하거나 일부 축이 응답하지 않으면 `FAULT / 상태 미확인`과 미확인 축 ID를 표시해요. 이때는 STOPPING이나 HOLDING으로 표시하지 않아요.
- **토크 OFF 분기:**
  - 자동으로 끄지 않아요. FAULT·HOLDING·READY에서 사용자가 x를 누르고 "팔을 받친 뒤 y"를 입력했을 때만 꺼요(자중 처짐 때문).
  - 걸림이 계속되거나 출력을 해제해야 할 때 쓰는 경로예요. 경미한 편차나 단순 종료에는 쓰지 않아요.
- **서보 자체 보호 정지:** 과부하 등으로 서보가 스스로 토크를 끈 축은 "유지 중이 아님"으로 표시해요. 오류 해제, 토크 ON, 보정 복원은 자동으로 하지 않아요.

### 8.2 통신이 끊기거나 프로세스가 멈췄을 때

- **설정을 자동으로 바꾸지 않아요.** Bus Watchdog은 `scan`으로 조회해 기록만 해요. 값이 0이 아니면 이 구현과 맞지 않으므로 `prepare`를 거부하고, 켜거나 끄지 않아요.
- **Watchdog 비활성(기본값)일 때 서보는 계속 갈 수 있어요:**
  - HOME 명령이 이미 전달됐다면, 전원이 유지되고 다른 오류가 없는 한 서보는 통신이 끊겨도 남은 내부 프로파일대로 HOME까지 가서 멈출 수 있어요.
  - PC가 표시하는 `COMM_LOST`는 PC의 판단일 뿐, 모터가 그 자리에서 멈췄다는 뜻이 아니에요.
  - UI에는 `COMM LOST / MOTION UNKNOWN`과 12 V 전원 스위치 안내를 띄우고, 자세 유지나 HOME 도착을 선언하지 않아요.
  - 이 동작은 가상 시험으로 재현했어요.
- **PC와 독립적인 출력 차단:**
  - 현재 구성에서는 **손 닿는 곳의 12 V 전원 스위치**뿐이에요. 터미널 정지 키는 통신이 살아 있을 때만 통하는 소프트 정지예요.
  - 전원을 끄면 토크가 사라져 팔이 자중으로 처져요. 검증된 비상정지 회로는 없어요.
- **복구:**
  - 통신이 돌아와도 COMM_LOST를 유지하고 보정은 무효예요. 자동 재보정이나 자동 HOME 재개는 없어요.
  - 다시 시작하려면 n(새 세션) → 설정 확인을 거쳐야 해요. calibrate는 재정렬부터, prepare / teleop은 저장된 보정을 서보 읽기값으로 다시 검증한 뒤 HOME 미리보기 + y부터예요.
  - 서보가 이전 HOME을 계속하는 것과, 다시 연결된 PC가 새 HOME을 보내는 것은 다른 일이에요. 후자는 새 Enter 없이 일어나지 않아요.
- **Watchdog과 응답 실패의 차이:**
  - Watchdog은 **수신된 모든 Instruction Packet** 사이의 공백을 봐요. 응답만 사라지고 요청은 계속 도착하는 경우(부분 응답 실패)에는 타임아웃이 나지 않아요.
  - 가상 서보로 완전 단절, 응답 손실, 부분 응답 실패를 각각 시험했어요.
- **나중에 Watchdog을 쓰는 선택 기능**은 별도 기능으로 구분해요. Goal 쓰기 제한, 사용자 복구, 프로파일 재확인, 과거 HOME의 자동 재개 금지를 함께 다뤄야 해요. 이번 기본 구성에는 넣지 않았어요.

### 8.3 종료와 유지

- **"유지"의 의미:** 이동이 멈춘 것을 확인한 뒤 최종 위치를 유지하는 것이에요.
  - 이동 중에 q, Ctrl+C, 창 닫기가 들어오면 감속 정지 → HOLDING 확인 → 종료 순서로 처리해요.
  - 이동 중에 포트를 닫아 HOME이 계속 진행되는 상태를 "유지 성공"으로 보고하지 않아요.
  - Ctrl+C를 두 번 누르면 정지 확인 없이 종료하고, "최종 상태 미확인"으로 보고해요.
- **종료 후:**
  - 토크가 ON인 서보는 마지막 Goal을 유지해요. PC는 더 이상 감시하지 않고, Watchdog은 건드리지 않아 0 그대로예요.
  - 종료할 때 축별 실제 최종 상태(토크 ON/OFF, 정지/이동, 미확인)를 터미널과 로그에 표시해요.
- 다음 실행 때 토크가 켜져 있으면 CONFIRM_RELEASE(받친 뒤 토크 OFF)부터 시작해요.

## 9. 실행 전 거부 설정과 출력 관련 주의

| 설정 | 처리 | 이유 |
|---|---|---|
| Drive Mode bit2 = 1 | prepare 거부 | 시간 기반 프로파일이라 PV/PA 단위가 다름 (그 모드 자체가 비정상은 아님) |
| Drive Mode bit3 = 1 | prepare 거부 | Goal 갱신만으로 토크가 자동 ON되면 "현재 위치 설정 → 토크 ON" 순서를 보장하지 못함 |
| Drive Mode ≠ 설정 기대값 | prepare 거부 | 부호 s를 정의한 기준이 바뀜 |
| Startup Configuration ≠ 0 | prepare 거부, 비트 의미 표시 | Startup Torque On = 0, RAM Restore = 0을 전제함 (0이 아니라고 재부팅 감지가 반드시 불가능하다고 단정하지는 않음) |
| Bus Watchdog ≠ 0 | prepare 거부 | §8.2 |
| HW Error ≠ 0, SRL ≠ 2, Protocol ≠ 2, 모델 ≠ 1030 | prepare 거부 | 자동으로 지우거나 고치지 않음 |
| Operating Mode ≠ 4 | MODE_PREP (허용 설정 + 사용자 확인이 있을 때만) | 이번 구현의 유일한 EEPROM 쓰기 |

- 위 설정은 자동으로 고치지 않아요. `scan`과 `watch`는 계속 읽기 전용으로 동작하며, 거부 사유와 확인할 레지스터를 알려줘요.
- **출력 관련 주의:**
  - mode 4에서는 Current Limit/Goal Current가 전류 상한으로 적용되지 않아요. "Current Limit을 읽었으니 그 이하 토크"라고 처리하지 않아요.
  - 전류, PWM, 오류는 관찰만 하고, Current/PWM/Shutdown 설정은 바꾸지 않아요.
  - 정지 토크 3 N·m × 16 = 48 N·m는 손실 없는 정지 상태를 가정한 단순 계산값이에요. 실제 가용 토크·연속 토크·감속기 허용 토크가 아니에요.
  - 저속으로 제어해도 최대 출력이 제한되지 않아요.
  - 중력 하중 비율 같은 수치는 질량·무게중심·효율 근거가 없는 한 미검증 추정으로 남겨요.

## 10. 설정과 로그

- **`config/robot.yaml`** (실기, 버전 관리): 포트·baud, 서보 기대값, 모델 경로, 관절(ID, g, s, q_ref, limit), HOME 후보와 `confirmed`/`path_confirmed`가 들어 있어요. 확인이 필요한 값은 모두 `null`이에요.
- **`config/motion.yaml`** (공통): 주기와 stale 기준, 속도·가속 상한, HOME 판정, 보정 판정, 유지·정지 기준이에요.
- **`config/fake_robot.yaml`**: 가상 시험 전용으로 지어낸 값이고 "FAKE"로 표시했어요. 실기 실행은 이 파일을 거부하고, 가상 실행은 실기 파일을 거부해요.
- **로그 위치:** 실기는 `logs/sessions/<시각>_<목적>/`, 가상은 `logs/fake/`, 스캔은 `logs/scans/`예요.
- **로그 내용:** 상태 전이, 송신 명령(값 포함), 설정·모델 sha, 서보 설정 스냅샷, P_ref, 측정·명령 각도와 오차(솎아서 기록), 통신 오류, 판정 근거(`home_report.json`), 최종 상태예요. 세션 P_ref는 기록만 하고 복원하지 않아요.

## 11. 검증 단계와 완료 기준

| 단계 | 상태 | 방법 / 통과 기준 |
|---|---|---|
| 모델 확보·검증 | **완료** | 커밋 blob 해시, 원본 고정 해시 12/12, URDF/xacro 5종 관절 동일, 표시 모델 프레임 = 원본 3종, TCP FK 일치 |
| 하드웨어 없는 시험 | **완료** | pytest 81개 통과 (항목은 IMPLEMENTATION_REPORT) |
| 가상 전체 루틴 | **완료** | fake_bus 실시간: 2.5 °/s에서 READY(38.0 s = 예측 37.0 s + 유지 판정 1 s), 정지·Ctrl+C·통신 단절·뷰어 창 |
| H0 포트 확인 | **완료** | Wizard가 점유하던 동안 거부(포트 미개방) 확인 → Wizard 종료 후 포트가 비어 있음 |
| H1 읽기 전용 조회 | **완료** | `scan`: 6축 1030, 서보 설정 거부 사유 0, Sync Read 313회 실패 0(p99 16.15 ms), 정지 잡음 1 count, 모터 쓰기 0회. 실기 `watch`: 약 20 Hz, 실패 0, 쓰기 0 |
| H2 부호·감속비 | 미실행 (사용자 손동작 필요) | `watch`로 + 방향 회전 → s 확정, 각도기 90° → 약 16384 count (CALIBRATION_GUIDE) |
| H3 q_ref·범위·HOME 확정 | 미실행 (사용자 측정 필요) | CALIBRATION_GUIDE 기록표, `preview-home`, 실물 경로 확인 |
| H4 유지 시험 | 미실행 | `prepare --hold-only` 3회: 점프 ≤ 0.5°, 잔여 오차 ≤ 0.2°, SIM과 실물 자세 일치 |
| H5 저속 HOME | 미실행 | 2.5 → 5 → 11.25 °/s: 실제 도착 조건으로만 READY, 오차·전류·온도 기록 |
| H6 정지·재현성 | 미실행 | HOME 중 s → 역행 없는 정지 확인. 전원 재인가 후 같은 마킹으로 5회: HOME에서 외부 기준 대비 관절별 ≤ ±0.5° (마킹 ±0.3° + 백래시 가정, 실측으로 조정). 케이블을 뽑는 시험 대신 가상 오류 주입 결과를 근거로 함 |

엔코더는 감속기 앞에 있어서 READY 판정(0.2°)은 "서보가 목표에 도착했다"는 것만 보장해요. 실제 관절 각도 오차는 마킹 오차와 백래시를 합친 값이고, 외부 측정으로만 확인할 수 있어요.

## 12. 실행 명령 (모두 구현돼 있음)

```bash
cd ~/manipulator_control_ws && source scripts/env.sh
python -m manipulator_control scan                       # 읽기 전용 점검 (Wizard는 닫아 둔다)
python -m manipulator_control watch                      # 읽기 전용 STATE (미보정: 상대 변화만)
python -m manipulator_control pose --sweep 2             # 하드웨어 없음: 관절 2의 + 방향
python -m manipulator_control preview-home               # 하드웨어 없음: 기준 -> HOME 계획 재생
python -m manipulator_control prepare --hold-only        # 실기: 토크 ON 유지 시험 (값 확정 후)
python -m manipulator_control prepare                    # 실기: 전체 루틴 (값 확정 후)
python -m manipulator_control prepare --fake             # 가상 전체 루틴 (쓰기는 가상 서보에만)
python -m pytest -q                                      # 시험
```

## 13. 구현을 막는 미확인 값 (사용자가 줄 최소 정보)

1. **관절별 부호 s:** `watch`를 켜고 각 관절을 + 방향으로 돌려 확인해요(손동작 필요).
2. **기준 자세 q_ref:** URDF 영점 기준의 관절 상대각이에요. 정하는 방법은 CALIBRATION_GUIDE에 있어요.
3. **실물 가동 범위 limit_deg:** 기구 간섭과 케이블을 기준으로 정해요.
4. **HOME 확정:** 후보(시뮬 ready) 자세와, 기준 자세 → HOME 전체 경로가 실물에서 안전한지 확인한 뒤 `home.confirmed`와 `home.path_confirmed`를 설정해요.
5. (완료) 서보 레지스터와 통신 주기는 `scan`으로 확인했어요. 6축 모두 이미 mode 4라 EEPROM 쓰기(`allow_operating_mode_write`)는 필요 없어요. 실기 실행 중에는 Wizard를 닫아 두세요.

---

## 원래 PLAN에서 교체한 내용 (모두 해당 절에서 직접 교체함)

- **§8:** "정지 = 목표 갱신 중지 → 자세 유지"를 다음 내용으로 교체했어요.
  - 서보 프로파일 정지 절차 (쓰기 → 확인 → 정지 확인 → HOLDING)
  - 취소 정지와 토크 OFF 분기
  - 통신 단절 시 "서보가 계속 갈 수 있음"
  - 종료 시 "유지"의 의미
- **§7.2:** 임시 허용 범위 `[min(q_ref, q_home) − 5°, max + 5°]`를 삭제했어요. 범위가 확정되기 전에는 실기 이동이 없어요.
- **§7.2:** "a = 5 °/s² → PA 4"를 PA ≤ 3(4.02 °/s²)으로 바로잡았어요. PA 4는 5.36 °/s²예요.
- **§7.3:** 사다리꼴 프로파일에서 가속도가 불연속이고 저크가 무제한임을 명시했어요.
- **CALIBRATION_GUIDE:** "스마트폰 경사계로 수평축 관절 각도를 잰다"를 "부모·자식 링크의 상대 자세와 URDF 영점으로 정한다"로 교체했어요. 링크가 세계 기준으로 기운 각도는 관절각이 아니에요.
- **§9:** mode 4의 전류 제한 오해, 48 N·m의 의미, 미검증 하중 비율에 대한 주의를 추가했어요.
