# 구현 보고서 — XM430 제어 준비 루틴

- 작성: 2026-10-26
- 범위: `MANIPULATOR_CONTROL_EXECUTION_REQUEST.md` 1~10절
- 1~10절은 15:30 기준 기록이에요. 이후 변경은 11절(보정 저장·재사용, 조작 패널), 12절(SIM 기즈모, 관절 3 진단), 13절(보정 근거 PA/PV, 통신 단절 시 재보정, 22.5 °/s)에 있어요.

## 1. 결과 요약

| 항목 | 결과 |
|---|---|
| 소프트웨어 (scan · watch · prepare · prepare --hold-only · --fake · pose · preview-home) | **구현 완료** |
| 하드웨어 없는 시험 | **81개 통과** (`python -m pytest -q`, 4.6 s) |
| 가상 전체 루틴 (fake_bus, 실시간, 실제 MuJoCo 창 포함) | **성공**. 2.5 °/s에서 READY, 정지·Ctrl+C·통신 단절 시나리오 확인 |
| 실기 읽기 전용 조회 (`scan`, `watch`) | **성공** (15:21, 사용자가 Wizard를 닫은 뒤). 6축 응답, 서보 설정 때문에 prepare가 거부될 사유 0, Sync Read 실패 0, **모터 쓰기 0회**. Wizard가 점유하던 동안에는 점유를 감지하고 포트를 열지 않음(§9.1) |
| 실기 동작 (토크 ON·HOME) | **미실행**. `config/robot.yaml`의 s·q_ref·limit·HOME 확정값이 없음. `prepare`는 차단 사유 20개를 출력하고 포트를 열기 전에 거부(코드 2) |
| 지금 실제 HOME 이동이 가능한가 | **아니요.** (1) 부호·기준각·가동 범위·HOME·경로가 미확정이라 프로그램이 거부해요. (2) 유지 시험과 저속 HOME의 실측 기록이 없어요. 서보 설정과 통신 쪽에는 막는 사유가 없어요(§9.1) |

## 2. 환경과 의존성 (전역 변경 없음)

- 전용 venv: `~/manipulator_control_ws/.venv` (`python3.12 -m venv`, system site 미포함)
- 설치 패키지(`requirements.lock`): dynamixel-sdk 4.0.3, pyserial 3.5, mujoco 3.15.0 (glfw 2.11.0, PyOpenGL 3.1.10 포함), numpy 1.26.4, PyYAML 6.0.3, pytest 9.1.1 외 의존성
  - mujoco와 numpy는 원본 시뮬 venv와 같은 버전이에요.
  - dynamixel-sdk는 ROS Jazzy apt 후보와 같은 4.0.3을 골랐어요. PyPI에는 4.1.0도 있어요.
- `scripts/env.sh`: ROS의 PYTHONPATH를 비우고, pip·XDG 캐시와 임시 파일을 작업공간 안에 두고, venv를 활성화해요.
- 하지 않은 것: sudo, apt, udev(latency_timer·ModemManager), `~/.bashrc`, ROS, 원본 시뮬 venv, 사용자 전역 Python 변경.
- 서보 설정 변경: 없음. 이 작업공간 도구는 실기 포트를 한 번도 열지 않았어요.

## 3. 모델 확보와 검증

| 확인 | 결과 |
|---|---|
| 필요한 파일이 `eeaca7f`에 추적되는가 | 28개 모두 추적됨, 미커밋 변경 없음 |
| 사본 (`tools/copy_model.py`, `git archive eeaca7f`) | `models/upstream_eeaca7f/`, 커밋 blob 해시(SHA-1) 28/28 일치, `models/manifest.sha256` |
| 원본 고정 해시 (`original_model_sha256.json`) | 12/12 일치 |
| URDF/xacro 5종 관절 비교 | 차이 없음 (origin·axis·type·limit·parent/child·mimic) |
| 표시 모델 (`tools/build_viz_model.py`) | 측정 팔 프레임 = `6dof_control.xml` = MuJoCo가 컴파일한 `6dof_mj.urdf` = `pinch_scene.xml`. 고스트 2개도 같은 프레임. qpos는 이름으로 매핑(측정 0~5, cmd 6~11, tgt 12~17). ready 자세 TCP FK = (0.094294, 0.100206, 0.25) m로 원본·`demo.json`과 일치. 모델 경로 검사: q=0·ready 모두 접촉 없음, 팔꿈치를 170°로 접으면 접촉을 감지 |
| 표시 모델에서 뺀 것 | 책상, 카메라, ready 키프레임, 토크 actuator, 센서, 물리 옵션 |
| 원본 저장소 | 이 작업공간은 읽기만 했어요. 작업 중 다른 세션이 `14e7eac`(카메라 180° 회전, 15:17)와 `b30ab4e`(pac_vision A4 한 장 배치, 15:22)를 커밋했지만 모델 경로는 바뀌지 않았어요(`git diff eeaca7f HEAD -- 모델 경로`가 빈 결과). 사본은 여전히 유효해요 |

## 4. 추가한 파일

| 경로 | 역할 |
|---|---|
| `src/manipulator_control/control_table.py` | XM430 컨트롤 테이블(주소·크기·부호·EEPROM), 읽기 블록, 비트 해석 |
| `units.py` | count↔rad (`JointMap`, signed, `%4096` 없음), PV/PA raw↔관절 단위, Goal 범위 |
| `config.py` | 설정 로드, 구조 검사, `motion_blockers`(토크 ON·HOME 차단 사유) |
| `bus.py` | `DxlBus`(dynamixel-sdk): 포트 점유 검사, flock, TIOCEXCL, Sync Read의 ID별 응답 분리, `WritePolicy`와 명령어 가드, 통계 |
| `fake_bus.py` | 가상 XM430 체인. §5의 동작을 모델링 |
| `state.py` | 읽기 스냅샷(시각·누락 ID·신선도) |
| `calibration.py` | 정지 창 판정, `Calibration`(실시간 창에서만 생성, 로더 없음) |
| `planner.py` | 서보 프로파일 모델, 공동 시간축 HOME 계획(정수 PV/PA 기준 재계산·동기화 경고·거부), 감속 정지 목표 |
| `supervisor.py` | 상태 머신, 정지·FAULT·통신 단절·종료 정책, 서보 설정 검사 |
| `runner.py` | 버스 루프(유일한 송수신 지점), 예외 처리, 샘플 로그 |
| `viz.py` | MuJoCo passive 뷰어(운동학만): 측정 팔, 명령 고스트, 목표 고스트, 영문 오버레이, 종료 처리 |
| `terminal.py` | 키 입력(cbreak), 입력 버퍼 비우기, 한글 상태 출력 |
| `logs.py` | 세션 로그 (events, samples, session, home_report) |
| `scan.py` | 읽기 전용 점검과 주기·잡음 측정 |
| `model.py` | 표시 모델 로드와 관절명 매핑, URDF 비교, 모델 경로 검사 |
| `__main__.py` | CLI (scan, watch, prepare, pose, preview-home) |
| `pngio.py` | 안내서 그림용 PNG 저장 |
| `tools/copy_model.py`, `tools/build_viz_model.py` | 모델 사본·검증, 표시 모델 생성·교차 검증 |
| `config/robot.yaml`, `config/motion.yaml`, `config/fake_robot.yaml` | 실기 설정(미확정은 null), 공통 기준, 가상 전용 설정 |
| `tests/*.py` | 시험 81개 |
| `docs/*.md`, `docs/img/*.png`, `README.md`, `CURRENT_STATE.md`, `AGENTS.md` | 문서와 안내 그림 |

## 5. 가상 서보(fake_bus)가 재현하는 것

가상 서보는 모델일 뿐 실제 서보가 아니에요. 다음 동작은 e-manual을 근거로 재현했어요.

- 다회전 signed 카운트, 그리고 전원 인가 위치가 1회전 안으로 돌아가는 동작
- velocity-based 프로파일(PV·PA 단위, 0 = 무한)과 이동 중 Goal 변경 시 현재 속도에서의 재계획
- Position/Velocity Trajectory 레지스터, 토크 ON 시점의 현재 위치에서 프로파일 시작
- 토크 ON 중 EEPROM 쓰기 거부, 모드 변경 시 PV/PA 초기화
- Bus Watchdog: 토크 ON일 때만 감시, 수신 패킷마다 타이머 재시작, 시간 초과 시 정지하고 −1로 바뀜, 그 뒤 Goal 쓰기는 Data Range Error
- 재부팅, HW 오류에 따른 자체 토크 OFF

주입할 수 있는 고장은 링크 단절, 응답 손실, 특정 ID 무응답, 걸린 관절, 토크 OFF 상태의 처짐이에요. 해석 예측과의 차이는 내부 궤적 기준 3 count 이내, 실제 위치 기준 약 0.33° 이내예요(추종 지연 30 ms를 모델에 넣었기 때문).

## 6. 시험 (`python -m pytest -q` → 81 passed)

| 파일 | 개수 | 검사 내용 |
|---|---|---|
| `test_units_config.py` | 22 | count↔rad 왕복(음수, 4095 초과, 1,000,000, 부호 ±), **4096 count = 22.5°**, signed 해석, PV 262 = 59.998 rpm, 306 = 70.07 rpm, PA 1 = 1.3411 °/s², PA 4 = 5.3644 °/s², 5 °/s² → PA 3, Goal 범위, src 안에 `% 4096` 없음, 실기 설정의 미확정 값이 None인지(0으로 해석하지 않음), `prepare`·`--hold-only`가 포트를 열기 전에 거부하는지, `--auto-start`는 가상에서만, 구조 오류, `preview-home` |
| `test_planner.py` | 12 | e-manual 공식(t1 = 64·PV/PA), 가상 서보와 예측 일치, 정수화 뒤 모든 축 속도·가속 상한(PA ≤ 3), 공동 시간(늘리기, 자르지 않음), 속도 비율, 이동량 0·아주 작은 이동량(PV 0 금지), 양자화 편차는 경고만, 2° 경고 기준, 가동 범위 밖 경로 거부, 모델 자기충돌 거부, PA 최소값보다 낮은 가속 상한 거부, 정지 목표(역행 없음, 현재 Goal을 넘지 않음, 범위 여유 부족 표시), 속도 연속 |
| `test_supervisor_flow.py` | 18 | 정상 경로 → READY(다회전·음수 P_ref, Goal = 현재 위치를 토크 ON 전에 쓰기, 명령 순서, 도착 조건으로만 READY), hold-only, **미리 입력된 Enter 폐기**, 보정 중 움직임 거부·재시도, 모터 누락 시 연결 거부, Drive Mode bit2·bit3·Startup 1/2·Watchdog·Drive Mode 1·HW Error 시 prepare 거부와 watch 허용, mode 4 전류 제한 오해 방지 문구, MODE_PREP 확인·허용 필요, 토크 ON 상태로 시작하면 받친 뒤 OFF, 카운트다운 취소, HOME 중 중복 Start 거부, 범위 밖 HOME 거부 |
| `test_stop_fault_comm.py` | 13 | **HOME 중 정지**(STOPPING → 실측 정지 → HOLDING, HOME까지 가지 않고 역행 없음, 제동 거리 이내), q 종료(정지 확인 후 유지), **예외 경로** 감속 정지, 걸린 관절은 Goal = Present로 취소(앞쪽 연장 없음, 자동 토크 OFF 없음, 잔여 오차 기록), 토크 OFF는 명시적 확인 후에만, **통신 단절 + Watchdog 비활성: 서보가 HOME까지 계속 이동**하고 PC는 `COMM LOST / MOTION UNKNOWN` 표시, 재연결 후 자동 재개 없음, 새 세션은 토크 OFF 확인과 새 Enter가 필요, 부분 응답 실패는 상태 미확인, stale 표시, READY 중 재부팅 → 보정 무효, 서보 자체 보호 정지는 유지로 보고하지 않음(오류 유지), HOME 중 재부팅, **Watchdog: 읽기 요청이 타이머를 살리고 링크 단절은 시간 초과를 일으킴**, 종료 보고 |
| `test_bus_view_model.py` | 16 | WritePolicy(read_only는 전부 거부, control은 RAM 항목만), **명령어 가드**(read_only에서 WRITE·SYNC_WRITE·REBOOT·FACTORY_RESET 등 송신 차단, control에서도 REBOOT 차단), 가상 scan 쓰기 0회, watch 읽기 전용 보정, 미보정 상대 변화(22.5°) 표시, 포트 점유 프로세스 감지, 점유 포트 거부, **영문 오버레이·한글 메시지**, 측정·명령·목표 고스트의 출처 분리, 관절명 매핑·데이터 없는 고스트 숨김, **창 닫기 → 정상 종료 경로**, 모델 사본·표시 모델 해시, eeaca7f 관절표, 표시 모델에 시뮬 가정 없음, 참조 저장소 모델 경로 불변 |

## 7. 가상 실행 기록 (`logs/fake/`)

| 로그 | 내용 | 결과 |
|---|---|---|
| `20261026_150550_home` | `prepare --fake`, 기본값 2.5 °/s, 5 °/s² 상한 | READY 38.0 s (예측 도착 37.0 s + 유지 판정 1 s), 최종 오차 ≤ 0.003°(가상), 양자화 편차 경고 없음 |
| `20261026_145117_home` | 11.25 °/s | READY, 양자화 편차 2.14° 경고(R3, 도착 시각 차 2.93 s). 경고만 하고 진행 |
| `20261026_145828_home` | HOME 중 실제 SIGINT(Ctrl+C) | HOMING → STOPPING → HOLDING → EXITED (토크 ON 유지) |
| `20261026_150402_home`, `150419_home` | 실제 MuJoCo 창, HOME 중 q | STOPPING → HOLDING → EXITED, 창 정상 종료 |
| `20261026_151533_watch` 외 | watch 가상, 실제 창 | 읽기 전용 보정 → 종료. 반복 8회 모두 정상 종료 |
| `logs/sessions/20261026_152220_watch` | **실기** watch(읽기 전용) | 6축 연결, 약 20 Hz, 실패 0, 쓰기 0 |
| `144410`~`144814`, `150018`~`150230` | 개발 중 실행 (버그 수정 전) | 아래 8절의 버그를 찾은 기록 |

## 8. 구현 중 발견하고 고친 문제

- **로그 함수 인자 충돌:** 로그 함수의 인자 이름이 이벤트 필드 이름(`kind`)과 겹쳐 입력 이벤트가 유실됐어요. 위치 전용 인자로 바꿨어요.
- **HOME 예측 기준 시각:** HOME 계획(모델 경로 검사 포함, 2.5 °/s에서 0.69 s)이 버스 루프 안에서 실행돼서 실제 출발이 늦어졌고, 그만큼 예측 불일치로 잘못 판정됐어요.
  - 계획 시점을 토크 ON 확인 직후(카운트다운 전)로 옮겼어요.
  - 경로 검사 간격을 관절 변화 0.5°마다로 바꿨어요.
  - 예측 기준 시각을 명령 실제 송신 시각으로 바꿨어요.
- **예측 불일치 처리:** 예측 불일치를 취소 정지(Goal = Present)로 처리하면 움직이던 축이 역행했어요. 감속 정지로 바꿨어요. 취소 정지의 확인 시간에는 되돌아오는 시간을 포함했어요.
- **watch의 HW 오류 처리:** 읽기 전용 watch가 연결 시점부터 있던 HW 오류 때문에 FAULT로 멈췄어요. 이제 새로 생긴 오류만 무효화 증거로 보고, watch에서는 보정만 해제하고 계속 동작해요.
- **뷰어 종료 시 멈춤:** 이 노트북(GNOME Wayland)에서 MuJoCo 창을 프로그램이 닫으면, 렌더 스레드가 GLFW 이벤트 대기에 남아 있다가 `glfw.terminate`(atexit)에서 멈췄어요(4번 중 1번).
  - 텍스트 전달 스레드를 먼저 멈추고, 빈 GLFW 이벤트로 렌더 루프를 깨우고, 그래도 남으면 정리 후 종료하도록 고쳤어요.
  - 고친 뒤 8번 중 8번 정상 종료됐고, 대체 종료 경로는 한 번도 쓰이지 않았어요.

## 9. 실기 읽기 전용 조회 결과와 남은 항목

### 9.1 `scan` (2026-10-26 15:21, `logs/scans/scan_20261026_152149.json`)

**Wizard가 포트를 점유하던 동안:** `scan`·`watch`는 "포트를 다른 프로그램이 쓰고 있습니다: DynamixelWizard (PID 45245)"를 출력하고 포트를 열지 않은 채 종료했어요(코드 3). `prepare`는 차단 사유 20개를 출력하고 포트를 열지 않았어요(코드 2).

**사용자가 Wizard를 닫은 뒤 `scan`:**

| 항목 | ID 1~6 (모두 같음) |
|---|---|
| 응답·모델·FW | 6축 응답, 누락·예상 밖 ID 없음(ping 0~20), model 1030, FW 50 |
| 통신 | 1 Mbps, Protocol 2, Return Delay 500 µs, Status Return Level 2, Secondary ID 255(해제) |
| Operating Mode / Drive Mode | 4 (Extended Position) / 0 (정방향, 속도 기반, Goal 갱신 시 토크 ON 없음) |
| Startup Configuration / Bus Watchdog | 0 / 0 (비활성) |
| Homing Offset / Velocity Limit | 0 / 330 (75.6 rpm, PV 상한 262보다 큼) |
| Torque / HW Error / Shutdown | 0 / 0 / 52 (과부하·전기 충격·과열 시 보호 정지, 기본값) |
| PV / PA | 0 / 0 (무한). 토크 ON 전에 이 프로그램이 [PA, PV, Goal]을 쓰고 읽어서 확인함 |
| 게인 | Position P 800, I 0, D 0, Velocity P 100, I 1920 (기본값) |
| PWM / Current Limit | 885 / 1193 (mode 4에서는 Current Limit이 전류 상한으로 동작하지 않음) |
| 전압·온도 | 12.2~12.3 V, 33~35 °C |
| Present Position | 13945, 9968, 3409, −5479, 3701, 210 count. 전원 인가 뒤 움직여서 1회전 범위를 벗어난 다회전 값(음수 포함)이며, 원점과는 관계없음 |

- **prepare를 막는 서보 설정: 없음.**
- **연속 Sync Read(6축 × 39 byte):** 313회 중 실패 0회. 주기 p50 15.96 ms, p99 16.15 ms, 최대 17.87 ms로, FTDI latency 16 ms가 주기를 정해요(p99 기준 약 62 Hz).
- **정지 잡음:** 3 s 동안 관절별 max−min 1 count, 표준편차 0.07~0.48 count예요. 보정 판정 기준(8 count)의 여유는 손으로 받친 상태의 흔들림이 결정해요.
- **송신:** PING 21, READ 6, SYNC_READ 313, 쓰기 0이에요.
- **실기 `watch --no-viewer`** (읽기 전용, 약 6.5 s, `logs/sessions/20261026_152220_watch`):
  - 빠른 읽기 129회(약 20 Hz), 실패 0회. 읽기 시간 p50 16.0 ms, p99 24.4 ms(2 Hz 느린 블록 포함).
  - 송신은 PING과 SYNC_READ뿐이었어요.
  - 그래서 `motion.yaml`의 20 Hz, stale 0.25 s, 통신 단절 0.5 s는 그대로 둬요(최악 읽기 시간의 약 10배 여유).
- `config/robot.yaml`의 `servo_expect`(mode 4, Drive Mode 0, Startup 0, SRL 2, Watchdog 0)는 6축 실측과 일치해요. 6축이 모두 이미 mode 4라서 `allow_operating_mode_write`는 `false` 그대로 둬도 돼요.

### 9.2 남은 항목과 이어서 할 명령

1. **부호·감속비 확인** (사용자 손동작): `watch` + CALIBRATION_GUIDE 2절
2. **q_ref·가동 범위·HOME·경로 확정** (사용자 측정): CALIBRATION_GUIDE 3~5절, `preview-home`
3. **유지 시험:** `prepare --hold-only` (3회)
4. **저속 HOME:** `prepare` (2.5 °/s, 한 손은 12 V 스위치)
5. **정지 시험과 재현성:** HOME 중 s, 전원 재인가 5회

## 10. 알려진 한계

- HOME은 서보의 사다리꼴/삼각 프로파일이라 가속도가 불연속이에요(저크 무제한). 가속도를 낮춰 완화할 뿐이에요.
- 양자화 때문에 이동량이 작은 축은 먼저 도착할 수 있어요. 편차를 계산해 경고하고 진행해요.
- 정지 목표 식과 가상 서보는 모델이에요. 실제 서보에서 역행 없이 정지하는지는 H6 시험에서 확인해야 해요.
- 통신이 끊긴 동안에는 PC가 서보를 멈출 수 없어요(Watchdog 미사용이 기본). 독립적인 출력 차단 수단은 12 V 전원 스위치뿐이에요.
- 모델 경로 검사는 볼록 껍질 기준이라 책상·케이블·사람을 검사하지 못해요.
- 보정 판정 기준(8 count, 2 raw)은 "손으로 받친 상태"를 가정한 초기값이에요. `scan`의 정지 잡음과 실제 정렬 결과로 조정해야 해요.
- 실기 읽기 주기는 FTDI latency 16 ms가 결정해요(실측 p50 16 ms). PC가 목표를 연속으로 보내는 방식을 쓰려면 latency 1 ms 설정(sudo udev)과 재측정이 필요해요.
- mode 4에서는 전류 상한이 적용되지 않아요. 저속 제어가 최대 토크를 제한하지 않아요.

## 11. 추가 변경 (2026-10-26 18:00): 보정 저장·재사용, 조작 패널

사용자 요청은 세 가지예요.

- teleop은 관절별 정렬 없이, 보정된 결과로 현재 자세를 불러와 시작하고, 시작 전에 HOME으로 가요.
- 이동·회전은 Ctrl+드래그 대신 클릭으로 좌표나 축 기준으로 해요.
- 보정은 최근 것만 남기고, 새로 요청하기 전까지 그걸 기준으로 운용해요.

| 항목 | 내용 |
|---|---|
| 저장 파일 | `runtime/calibration_state.json` 하나예요(가상 실행은 `fake_calibration_state.json`). 매핑, P_ref, 보정 당시 서보 설정(mode, Drive Mode, Homing Offset), 서보별 마지막으로 보낸 PA/PV/Goal(근거)이 들어 있어요. 임시 파일을 쓴 뒤 교체해요 |
| 생성·삭제 | calibrate 시작 시 지우고, 6축 고정(HOLDING) 시 저장해요. HOLDING/READY 진입과 종료 때 근거를 다시 저장해요. 재설정 근거(INVALIDATED, MODE)나 근거 불일치 상태로 종료하면 지워요. 통신이 없으면 그대로 둬요. watch는 건드리지 않아요 |
| 재사용 검사 (`Calibration.resume`) | PA/PV/Goal 읽기값 = 저장값(정확히, PV/PA ≠ 0), ID·감속비·부호·q_ref 일치, 서보 설정 일치, 토크 ON 축은 목표 ±0.5° 이내. 하나라도 어긋나면 FAULT NO_CALIBRATION이 되고 쓰기는 0회예요 |
| 시작 흐름 (prepare / teleop) | CONNECTED_CHECK → 검사 통과 → [RESUME_HOLD: 토크 OFF 축은 y 뒤 Goal = 현재 위치, 토크 ON, 0.6 s 확인] → HOME_PREVIEW(① 회색 팔 = 실물인지 ② 경로 확인, y) → COUNTDOWN → HOMING → READY → TELEOP |
| 토크 ON 시 Goal 변경 | 실물에서 관찰했어요. 17:29에 ID 2에 Goal 17194를 보냈고, 토크 ON 전에 읽기로 확인했는데 17:54에는 17193을 읽었어요. 이후 다른 쓰기는 없었어요. 매뉴얼에는 없는 동작이에요. 토크 ON 확인 뒤 Goal 읽기값이 보낸 값 ±36 count 안이면 그 값을 근거로 삼고, 벗어나면 정지 + FAULT GOAL_CHANGED예요. 가상 서보에도 이 동작을 넣었어요 |
| 이전 보정 가져오기 | `tools/import_session_calibration.py`(1회용)로 17:28 calibrate 로그에서 기록을 만들었어요. 그 뒤로 다른 제어 세션은 없었어요. `--adopt-servo-goals`로 서보를 읽기만 했고(SYNC_READ 2회), ID 2의 Goal을 17193으로 반영했어요 |
| 읽기 전용 확인 | 새 명령 `calibration-status`예요. 시작할 때와 같은 검사를 쓰기 없이 해요. 실기 17:54 결과는 **사용 가능**, 관절각 [−0.01, 0.02, −90.02, 0.01, 89.99, 0.00]°, 6축 토크 ON이에요 |
| 조작 패널 (`panel.py`, Tk) | 버튼은 베이스/툴 좌표 기준 이동 ±(1/5/10/20 mm)와 회전 ±(1/5/10/15°)예요. 좌표 직접 입력(X/Y/Z mm, RPY °, R = Rz·Ry·Rx), 현재 목표 불러오기, 목표 = 실제 자세, HOME, 재개, 정지 버튼이 있어요. 창을 닫으면 STOP이에요. 이벤트만 입력 큐에 넣고, 판단은 버스 루프가 해요. Tk 객체는 패널 스레드에서 해제해요(다른 스레드에서 해제되면 Tcl_AsyncDelete로 abort됐어요. 고친 뒤 재현 안 됨) |
| 목표 안전 검사 추가 | IK 해가 현재 자세에서 어떤 관절이든 30° 넘게 돌면 거부(`teleop.max_jump_deg`)해요. 현재 → IK 해 관절 직선 경로는 2° 간격으로 모델 검사(자기충돌·바닥)해요(검사 1회 0.74 ms) |
| 없앤 것 | MuJoCo 창의 마커 드래그(Ctrl+드래그) 추종이에요. 마커는 매 프레임 목표 위치로 다시 써요. 텔레옵 중에는 바닥에 베이스 X/Y/Z 축을 표시해요 |

**시험:** 114개 통과(`python -m pytest -q`, 약 12.7 s). 추가·변경된 시험은 다음과 같아요.

- `test_stored_calibration.py`(10개): 전원 재인가 후 거부·삭제, 새 calibrate 시작 시 삭제, 3회 연속 teleop 실행에서 같은 보정 유지·근거 갱신, 다른 프로그램의 Goal 쓰기 거부, q_ref 변경 거부, 토크 OFF 축은 y 뒤 그 자리 고정, watch 불간섭, 통신 단절 시 보존, 파일 왕복, 토크 ON 시 Goal 변경 반영
- `test_teleop.py`: 좌표 직접 입력 추종, 툴 좌표 이동·회전, 큰 이동 거부와 "목표를 실제 자세로", 패널 함수
- 정렬 시험은 calibrate로, HOME 시험은 "calibrate → 종료 → 새 실행" 헬퍼로 바꿨어요

**가상 실행:**

- `teleop --fake --fake-calibrated`(CLI, 터미널 없이 스크립트 입력): 저장된 보정 → HOME → TELEOP → 조작 → 종료 시 근거 갱신. 송신은 SYNC_WRITE 4회(HOME, teleop 목표, 정지 포함)였어요.
- 같은 명령에서 `--fake-calibrated`를 빼면(새 가상 버스 = 전원 재인가) NO_CALIBRATION으로 거부되고, 쓰기 0회, 저장 파일 삭제예요.
- MuJoCo 창과 조작 패널을 함께 띄운 실행은 exit 0이고 오류 출력이 없었어요. 패널 캡처로 TELEOP 상태와 목표 표시를 확인했어요.

**실기:** 읽기 전용(`calibration-status`, 가져오기 도구의 서보 읽기)만 실행했어요. 실기 HOME·teleop은 아직 실행하지 않았어요.

## 12. 추가 변경 (2026-10-26 21:15): SIM 기즈모, 패널 토크 OFF, 관절 3 진단

| 항목 | 내용 |
|---|---|
| 요청 | "SIM 안에서 EE에 대해 xyz축 직선운동과 로테이션을 클릭앤드래그로, 한 번에 하나씩", "패널에 토크 오프 버튼" |
| SIM 창 (`viz.py`) | MuJoCo passive viewer는 마우스 클릭을 프로그램에 넘기지 않아서, GLFW 창 + `mjr_render` 루프를 직접 만들었어요(60 Hz, 메인 스레드). 빈 곳 드래그는 MuJoCo 뷰어와 같은 카메라 조작이에요(`mjv_moveCamera`). 창을 닫으면 정상 종료 경로예요. 이 GLFW 빌드는 Wayland만 지원해요(X11 미지원 확인). libdecor가 있어 창 테두리가 생겨요 |
| 기즈모 (`gizmo.py`) | 화살표 3개(0.2~1.0 × 크기)와 링 3개(반지름 1.25 × 크기, 화살표와 교차 없음)예요. 크기 = 0.07 × 카메라 거리예요 |
| 선택과 드래그 | 마우스와 투영된 핸들 사이의 화면 거리가 10 px 이내면 잡혀요. 이동은 광선과 축 직선의 최근접점, 회전은 광선과 링 평면의 교점 각도(누적, ±π 펼침)로 계산해요. 화면과 거의 평행한 링(\|cos\| < 0.15)은 잡지 않아요. 드래그 목표는 30 Hz TARGET 이벤트로 보내고, 놓은 뒤 0.3 s는 드래그한 자세를 그대로 보여요 |
| 표시 | 기즈모를 카메라 쪽으로 절반 당기고 같은 비율로 줄여서 그려요. 화면에서는 같은 모습이지만 팔에 가리지 않아요. MuJoCo 3.15에서 `mjv_connector` 화살표는 길이의 절반만 그려져서(픽셀 측정: 기대 72 px, 실제 35 px), 길이를 2배로 넘겨 보정했어요(보정 후 229~299 px, 기대 227.6~300). 베이스 기준일 때는 바닥에 12 cm 축을 그려요(월드 프레임 표시는 Z축이 팔 기둥을 따라 올라와 헷갈려서 쓰지 않아요) |
| 쫓아가기 | 거부 방식(목표가 30° 넘게 앞서면 거부)을 바꿨어요. 이제 명령은 측정 자세에서 IK 해 방향의 관절 직선을 따라 최대 30°까지만 보내요. 경로는 현재 → IK 해 전체를 2° 간격으로 모델 검사해요 |
| 조작 패널 | 이동·회전 버튼, 단계, 좌표 입력을 뺐어요. 기즈모 축 기준(베이스/툴), 정지, HOME, 재개, 목표 = 실제 자세, 토크 OFF가 남았어요 |
| 패널 토크 OFF | 확인 창("팔을 받쳤나요?") 뒤 `TORQUE_OFF_NOW`를 보내요. HOMING/TELEOP은 감속 정지 확인 → 6축 토크 OFF → RELEASED, STOPPING은 정지 확인 뒤 OFF, HOLDING/READY/HOME_PREVIEW/COUNTDOWN/RESUME_HOLD/FAULT/ALIGN은 바로 OFF예요(통신 정상일 때). 보정은 남아요(카운트 유지) |

**관절 3 진단:** 사용자 보고는 "토크 ON인데 힘을 못 버팀, 분해했지만 부품 문제 아님, 식혀도 같음"이에요. 읽기 전용 확인만 했어요.

- scan(19:42): ID 3은 26 °C, HW Error 0, 전압 12.3 V예요. 게인·PWM/전류 제한·모드가 다른 5축과 같아요.
- 같은 HOME 이동(관절 3: −90° → −75.2°) 비교:
  - 17:21과 18:03: 전류 중앙 207 mA(최대 293), 추종 지연 −15 count, HOME 유지 126~129 mA
  - 18:18(분해 후): 전류 32 mA(최대 75), 지연 −4 count, 유지 0 mA
- 서보는 명령 위치를 ±1~2 count로 지키지만 관절 부하를 받지 않아요. 서보 출력축과 관절 사이 전달부가 미끄러지는 것으로 판단해요.
- 원래 전류 수준(≤ 0.3 A, 정지 전류 2.3 A)으로는 과열 근거도 없어요.

**시험:** 121개 통과. 추가한 시험은 다음과 같아요.

- 투영 = mjv_select
- 화살표 드래그는 그 축만, 링 드래그는 그 축 회전만 바뀌는지(베이스/툴 각각)
- 빈 곳 드래그는 카메라만 바뀌고 목표는 안 보내는지
- 기즈모 드래그를 실물이 따라가는지
- 멀리 끈 목표의 앞섬 ≤ 30°와 도달
- 패널 토크 OFF(이동 중: 정지 후 OFF, 미리보기 중: 바로 OFF)
- 가짜 GLFW로 창 닫기 → 종료 요청

**가상 실행:** 실제 창과 패널을 띄운 `teleop --fake --fake-calibrated`가 exit 0, 오류 출력 없이 끝났어요. 마우스 드래그는 처리 함수 시험으로만 확인했어요(자동 마우스 입력 도구 없음). **실기 기즈모 조작은 미실행**이에요.

## 13. 추가 변경 (2026-10-26 23:15): 보정 근거 PA/PV, 통신 단절 시 재보정, 관절 속도 22.5 °/s

| 항목 | 내용 |
|---|---|
| 요청 | "매번 캘리브레이션 하지 말고 가장 최근 값 사용, 통신이 끊어진 적이 있으면 다시", "관절 최대 속도 22.5°/s(모터 60 rpm), 가장 빠른 관절 기준 제한, 다른 관절은 비례 감소" |
| 원인 | 토크 OFF로 끝낸 세션마다 "PA/PV/Goal 읽기값 ≠ 보낸 값"으로 보정이 지워졌어요(21:14, 22:53). 22:59 읽기 전용 실측 결과 토크 OFF 동안 Goal = Present였어요(6축 차이 0). PV/PA는 보낸 값 그대로였어요 |
| 근거 변경 | `Calibration.resume`과 저장(`_hold_record`) 모두 PA/PV만 비교하고(정확히, 0 아님), Goal은 기록만 해요. 종료할 때 토크 OFF여도 PA/PV가 맞으면 저장해요 |
| 통신 단절 | COMM_LOST가 되면 바로 `DiscardCalibration`을 보내고 메시지로 다시 calibrate하라고 알려요. 통신 없이 종료하면 지워요. 실행하지 않는 동안의 전원 재인가는 PV/PA = 0으로 걸러져요 |
| 토크 ON 전 확인 | align의 hold_goal과 RESUME_HOLD의 goal 단계는 PA/PV가 정확히 같고 Goal이 보낸 값 ±36 count 안이면 통과해요. 토크 OFF 동안 Goal이 현재 위치를 따라가기 때문이에요 |
| 가상 서보 | 토크 OFF 동안 `goal_position = round(pos)`예요 |
| 22:51 보정 복원 | 22:53 세션의 `calibration_discarded` 기록(P_ref 등)을 썼어요. 그 세션에서 마지막으로 보낸 PA/PV가 지금 읽기값과 6축 일치했고(읽기 전용 SYNC_READ 2회), 이후 세션의 쓰기는 `torque_off_user`뿐이라 저장 파일을 되살렸어요. `calibration-status`는 사용 가능이에요 |
| HOME 거부 수정 | 22:54에 IK가 관절 3을 하한 −135°에 붙여서 측정값이 −135.011°가 됐고, HOME 계획이 "현재 자세 범위 밖"으로 거부됐어요. 이제 IK 범위는 한계에서 0.2° 안쪽이에요. 계획은 시작 자세가 한계를 1° 이내로 넘으면 허용하고, 경로 검사의 범위를 시작점까지 넓혀요(HOME은 범위 안쪽) |
| 속도 | `motion.yaml`의 `home.speed_deg_s`와 `teleop.speed_deg_s`를 22.5로 올렸어요(PV 262 = 59.998 rpm). 공동 시간축 계획이라 이동량이 가장 큰 관절이 상한에 맞춰지고, 나머지는 같은 시간에 끝나도록 비례해요. 가속 상한은 그대로라, 보정 자세 → HOME에서는 관절 4가 14.4 °/s로 최고, 관절 2는 9.9, 관절 6은 2.2 °/s예요(7.4 s) |

**시험:** 123개 통과. 추가·변경한 시험은 다음과 같아요.

- Goal 변경은 허용하고 PV 변경은 거부
- 토크 OFF로 종료한 뒤 Goal이 현재 위치를 따라가도 보정 유지(RESUME_HOLD)
- 통신 단절 시 보정 삭제 → 다음 실행 거부
- 범위를 0.3° 넘은 시작은 허용, 2° 넘은 시작은 거부
