# manipulator_control_ws — XM430 6축 팔 제어 준비 루틴

- **calibrate:** 관절별 정렬 → Enter → 원점 등록 → 그 자리 토크 ON 고정이에요. 결과는 저장되고, 최근 것 하나만 남아요.
- **prepare / teleop:** 저장된 보정을 서보 읽기값으로 검증한 뒤 현재 자세를 불러와요(정렬 없음). 그다음 HOME 미리보기 + y → HOME 이동(서보 내부 프로파일) → 실제 도착 확인 → READY예요. teleop은 이어서 SIM 창의 EE 기즈모를 끌어서 EE를 움직여요(화살표 = 축 이동, 링 = 축 회전).

- 하드웨어: ROBOTIS XM430-W210-T ×6 (ID n = Revolute_n), 각 관절 추가 16:1, 12 V, U2D2, 1 Mbps, Protocol 2.0, Extended Position(mode 4).
- 현재 상태와 다음 할 일: [CURRENT_STATE.md](CURRENT_STATE.md)
- 설계: [docs/MANIPULATOR_CONTROL_PLAN.md](docs/MANIPULATOR_CONTROL_PLAN.md)
- 값 정하기: [docs/CALIBRATION_GUIDE.md](docs/CALIBRATION_GUIDE.md)
- 구현·시험 결과: [docs/IMPLEMENTATION_REPORT.md](docs/IMPLEMENTATION_REPORT.md)

> **보정은 하나만 저장해요:** `runtime/calibration_state.json`에 가장 최근 calibrate 결과만 있어요.
> - calibrate를 새로 시작하면 이전 것을 지우고, 6개 관절이 모두 고정되면 새 것을 저장해요.
> - 다음 calibrate 전까지는 prepare / teleop이 이 보정을 계속 써요. 토크를 끄고 끝내도 그대로 써요.
> - **통신이 끊기면 다시 calibrate해야 해요.** 실행 중 통신 단절(COMM_LOST)이 감지되면 그 자리에서 저장된 보정을 지워요.
> - 실행하지 않는 동안 12 V를 껐다 켠 경우도 걸러져요. 재부팅하면 PV/PA가 0이 되고, 이 프로그램은 PV/PA에 0을 쓰지 않아요. 그래서 서보의 PV/PA가 마지막으로 보낸 값 그대로일 때만 이 보정을 써요.
> - 근거가 맞지 않으면 아무것도 쓰지 않고 거부하며, calibrate를 안내해요.
> - 지금 쓸 수 있는지는 `calibration-status`(읽기 전용)로 확인해요.

이 폴더는 저장소 `26_project_manipulator` 안에 있어요(2026-10-28부터). 이 컴퓨터의 `~/manipulator_control_ws`는 이 폴더를 가리키는 링크예요.

## 설치 (저장소 맨 위 `./setup.sh`가 대신해요, 손으로 할 때)

```bash
cd manipulator_control_ws                  # 저장소 폴더 안
python3.12 -m venv .venv
source scripts/env.sh                      # ROS의 PYTHONPATH를 비우고 .venv 활성화 (전역 설정은 바꾸지 않음)
pip install -r requirements.lock           # dynamixel-sdk 4.0.3, mujoco 3.15.0, numpy 1.26.4, pyserial 3.5, PyYAML, pytest
python tools/copy_model.py                 # 참조 저장소 @ model.UPSTREAM_COMMIT(지금 849f2e1)에서 모델 복사·검증 (기존 사본은 재검증만)
python tools/build_viz_model.py            # 표시 모델 생성 + 기구학 교차 검증
python -m pytest -q                        # 하드웨어 없는 시험
```

새 터미널에서는 항상 이 폴더에서 `source scripts/env.sh`부터 실행해요. 직렬 포트(U2D2)는 `dialout` 그룹이 필요해요(`./setup.sh --system`). 그 밖에 `~/.bashrc` 변경은 필요 없어요.

> `tools/copy_model.py`를 다시 실행하면 `models/PROVENANCE.md`를 새로 써요. 확인만 할 때는 `python -m pytest -q`로 충분해요(사본 해시·원본 커밋 비교 시험).

## 명령

| 명령 | 하드웨어 | 쓰기 | 용도 |
|---|---|---|---|
| `python -m manipulator_control scan` | 실기 | 없음 (PING/READ/SYNC_READ만) | ID·모든 레지스터·비트 해석, Sync Read 주기, 정지 잡음, prepare 거부 사유. 결과는 `logs/scans/` |
| `python -m manipulator_control watch` | 실기 | 없음 | 실제 STATE를 MuJoCo에 표시. 미보정이면 상대 변화만(s=+1 가정). s·q_ref가 있으면 Enter = 읽기 전용 기준 등록 |
| `python -m manipulator_control pose [--sweep N] [--q-deg ...]` | 없음 | — | URDF 자세와 관절 N의 + 방향 보기 |
| `python -m manipulator_control preview-home` | 없음 | — | 기준 자세 → HOME 계획(prepare와 같은 정수 PV/PA)을 출력하고 재생 |
| `python -m manipulator_control calibration-status` | 실기 | 없음 (SYNC_READ 2회) | 저장된 보정을 지금 쓸 수 있는지, prepare / teleop이 시작할 때와 같은 검사를 해요. 쓸 수 있으면 현재 관절각도 보여줘요 |
| `python -m manipulator_control calibrate` | 실기 | 토크·PV/PA/Goal | 관절 1→6을 하나씩 진행해요: (부호 미확정이면 방향 확인) → SIM 기준 자세에 맞추고 Enter → 그 관절 원점 등록 + 그 자리 토크 ON 고정 → HOLDING(이동 없음). **시작할 때 이전 보정을 지우고, 끝나면 새 보정을 저장해요.** 확인한 부호는 y로 robot.yaml에 저장해요. `prepare --hold-only`와 같아요 |
| `python -m manipulator_control prepare [--home-speed-deg-s 22.5]` | 실기 | 토크·PV/PA/Goal | 저장된 보정 검증 → 현재 자세 표시 → HOME 경로 SIM 미리보기 + y → HOME → READY (정렬 없음) |
| `python -m manipulator_control teleop [--home-speed-deg-s 5]` | 실기 | 토크·PV/PA/Goal | prepare와 같고, READY 뒤에는 SIM 창의 **EE 기즈모**(화살표 = 축 이동, 링 = 축 회전)를 끌어서 EE 목표를 움직여요. 실물이 IK로 실시간으로 따라가요. 조작 패널에는 정지·HOME·토크 OFF가 있어요 |
| `python -m manipulator_control pinch [--speed-scale S] [--check-only]` | 실기 | 토크·PV/PA/Goal | prepare와 같고, READY 뒤에 Pinch 시뮬(`pinch_sim.app --real-arm`)과 연결해요. **터미널 y로 무장**하면 시뮬이 검증해서 보낸 관절 궤적을 여기서 **다시 검사**한 뒤 실물이 같은 시간표로 따라가요(아래 "Pinch 시나리오") |
| 위 명령 + `--fake` | 가상 서보 | 가상 서보에만 | 같은 코드를 fake_bus와 `config/fake_robot.yaml`(지어낸 값)로 실행해요. 로그는 `logs/fake/`, 보정은 `runtime/fake_calibration_state.json`이에요 |

- 공통 옵션: `--no-viewer`, `--no-cmd-ghost`, `--no-target-ghost`
- teleop 전용: `--no-panel` (조작 패널 없이 써요. 기즈모 축은 베이스 기준)
- 가상 실행 전용 옵션:
  - `--auto-start S`
  - `--fake-event confirm@T|jog@T:+x|stop@T|quit@T|linkdown@T|linkup@T|rxloss@T|silent@T:ID|reboot@T:ID|stuck@T:ID|hwerror@T:ID`
  - `--fake-calibrated`: 가상 버스는 매번 전원을 막 켠 상태로 시작해요. 그래서 prepare / teleop --fake에는 이 옵션이 필요해요. 가상 서보를 calibrate 직후 상태로 만들고 그에 맞는 저장 보정을 넣어요.
- 실기 명령은 대화형 터미널에서만 실행돼요. `--auto-start`와 `--fake-*` 옵션은 실기에서 거부돼요.

## 실기 조작 순서

기준 자세는 팔을 위로 곧게 뻗은 자세에서 **관절 3 = −90°(케이블 길이 때문), 관절 5 = +90°**예요(`config/robot.yaml`의 `q_ref_deg`). 그림은 `docs/img/ref_pose.png`에 있어요.

1. DYNAMIXEL Wizard를 닫아요. 포트를 잡고 있으면 이 프로그램은 포트를 열지 않고, Wizard를 종료하지도 않아요.
2. 12 V 전원을 켜고, 전원 스위치에 손이 닿게 해요.
3. `python -m manipulator_control calibrate`를 실행해요. 서보 설정 검사에 걸리면 거부 사유를 보여주고, 자동으로 고치지 않아요.
4. 관절 1부터 6까지 하나씩 진행해요. SIM에서 노란 링크가 지금 맞출 관절, 회색이 이미 고정된 관절, 초록이 기준 자세예요.
   1. **방향 확인** (부호가 아직 없을 때만): 초록 팔이 그 관절의 + 방향으로 흔들려요. 실물 관절을 그쪽으로 조금(10~30°) 돌리면 프로그램이 방향을 감지하고, 노란 링크가 실물을 따라 움직여요. 같은 방향, 비슷한 크기로 움직이면 **y**를 눌러요. 반대로 돌렸거나 크기가 다르면 **r**로 다시 해요.
   2. **영점:** 그 관절을 SIM 기준 자세와 같게 맞추고, 움직이지 않은 채 **Enter**를 눌러요. 1초 정지 판정 → 그 관절 원점 등록 → 현재 위치를 목표로 설정 → 그 관절만 토크 ON 고정 순서예요. 이동은 없어요.
   3. 미리 눌러 둔 Enter는 버려져요. 움직이면 이유를 보여주고 다시 Enter를 기다려요.
5. 시작할 때 이미 토크가 켜진 관절(예: 이전에 q로 끝낸 경우)은 끄지 않고 그대로 둬요. 그 관절 차례에 기준 자세와 맞으면 Enter만 누르면 그 자리에서 다시 등록돼요. 맞지 않으면 팔을 받친 뒤 x → y를 누르면 모든 토크가 꺼지고 관절 1부터 다시 시작해요.
6. 6개를 모두 고정하면 HOLDING이에요. 이 보정이 저장돼요. 이번에 확인한 부호가 있으면 **y**로 `config/robot.yaml`에 저장해요.
7. q로 종료해요(토크 유지). 이후 `prepare` / `teleop`은 관절별 정렬 없이 저장된 보정으로 시작해요. 다시 맞추고 싶을 때만 calibrate를 다시 실행해요. 그러면 이전 보정은 버려져요.

## 저장된 보정으로 시작 (prepare / teleop)

1. 서보 읽기값으로 저장된 보정을 검증해요. 검사 항목은 다음과 같아요.
   - 관절마다 PA/PV가 마지막으로 보낸 값과 정확히 같은지(0이 아님). Goal은 비교하지 않아요. 토크가 꺼져 있는 동안 서보가 Goal을 현재 위치로 계속 바꾸기 때문이에요(실측)
   - 관절 ID·감속비·부호·기준각이 보정할 때와 같은지
   - Operating/Drive Mode와 Homing Offset이 같은지
   - 토크가 켜진 관절이 목표 위치에 있는지
2. 하나라도 맞지 않으면 **아무것도 쓰지 않고 거부**하고, 이유를 보여주고 calibrate를 안내해요. 대표적인 경우가 12 V 전원을 다시 켠 경우예요.
3. 통과하면 SIM 회색 팔이 **엔코더로 계산한 실물 현재 자세**예요. 실물과 같은지 눈으로 확인해요.
4. 토크가 꺼진 관절이 있으면(지난번에 x → y로 끈 경우) 팔을 받친 뒤 **y**를 눌러요. 그 자리 그대로 토크 ON 고정되고 이동은 없어요.
5. HOME 미리보기에서 초록 팔이 지금 자세 → HOME 경로를 반복 재생해요. 실물 주변이 안전하면 **y**를 눌러요. "손을 떼세요" 3초 뒤 HOME → READY 순서예요.
6. 정지 확인 뒤 유지하거나 종료할 때마다(토크 ON/OFF 상관없이) 근거(지금 읽은 PA/PV)를 다시 저장해요. 그래서 다음 실행도 같은 보정으로 시작해요.
   - 재부팅 등 근거가 무효인 상황이나 **통신 단절**에서는 저장된 보정을 지워요.
   - 프로그램이 강제 종료되면(kill -9) 마지막 저장 이후에 보낸 PA/PV가 근거에 반영되지 않아서 다음 실행이 거부할 수 있어요. 그때는 calibrate를 다시 해요.
7. 시작 자세가 가동 범위를 1° 넘게 벗어나 있으면 HOME 계획이 거부돼요(예: 토크 OFF로 처진 팔). 토크가 꺼진 관절은 고정(y) 전에 손으로 범위 안으로 옮겨요. 1° 이내는 허용해요(IK가 한계에 붙인 자세).

## EE 조작 (teleop)

1. `teleop`을 실행하면 위의 "저장된 보정으로 시작" 순서대로 HOME까지 가요. READY가 되면 자동으로 EE 조작 모드가 돼요.
2. **SIM 창에서 EE 기즈모를 마우스로 끌어요.** 한 번 끌 때는 핸들 하나만 움직여요.
   - **화살표(빨강 X · 초록 Y · 파랑 Z):** 클릭한 채 끌면 그 축 방향으로만 직선 이동해요.
   - **링(같은 색):** 클릭한 채 끌면 그 축을 중심으로만 회전해요(목표점 기준). 두 링이 겹치는 점은 어느 링이 잡힐지 애매하니 피해서 잡아요. 화면과 거의 평행하게 누운 링은 잡히지 않으니 카메라를 돌려요.
   - 마우스를 올리면 그 핸들이 굵어지고, 끄는 동안에는 노란색이에요.
   - **빈 곳:** 왼쪽 드래그는 카메라 회전, 오른쪽 드래그는 이동, 휠·가운데 드래그는 확대/축소예요(Shift는 수평 변형).
   - 축 기준은 조작 패널에서 골라요. **베이스**는 로봇 바닥의 X/Y/Z로, 바닥에 작은 축이 표시돼요. **툴**은 EE 자신의 X/Y/Z예요.
3. 실물은 IK 해를 따라 실시간으로 움직여요. 관절 속도 상한은 22.5 °/s(모터 60 rpm)이고, 가속 상한은 10 °/s²예요. 가장 많이 도는 관절이 상한에 맞춰지고, 다른 관절은 같은 시간에 도착하도록 비례해서 느려져요. HOME도 같은 방식이에요(22.5 °/s, 가속 5 °/s²).
   - 목표를 멀리 끌면 "따라가는 중"으로 표시돼요. 명령은 실물보다 최대 30°만 앞서게 보내고, 실물이 움직이는 만큼 계속 따라가요.
   - 지금 자세 → 목표의 관절 경로를 2° 간격으로 모델 검사(자기 충돌·바닥)해요.
4. 가동 범위 밖이거나 손이 닿지 않는 목표, 경로 검사에 걸리는 목표는 기즈모 가운데 공이 **빨갛게** 바뀌고 보내지 않아요. 조금 되돌리거나 패널의 "목표를 실제 자세로"를 눌러요.
5. **조작 패널**(작은 창, 항상 위)에는 기즈모 축 기준, ■ 정지, HOME 미리보기(h), 조작 재개(t), 목표를 실제 자세로, **토크 OFF**가 있어요.
   - **토크 OFF**는 "팔을 받쳤나요?" 확인 창에서 예를 눌러야 실행돼요. 움직이는 중이면 먼저 감속 정지를 확인한 뒤 6축 토크를 꺼요. 저장된 보정은 남아요.
   - 패널 창을 닫으면 정지(유지)예요. y(확인)와 q(종료)는 터미널에서만 받아요.
6. 터미널 키도 그대로 써요. `i/k` = 베이스 +X/−X, `j/l` = +Y/−Y, `u/o` = +Z/−Z, `1/2`·`3/4`·`5/6` = X·Y·Z축 회전이고, `[`·`]`로 단계를 바꿔요(기본 1 cm·5°).
7. **s/space**는 정지예요. 정지를 확인한 뒤 HOLDING이 되고, **t**로 다시 조작하고 **h**로 HOME 미리보기를 다시 해요. **q**는 정지 후 종료(토크 유지)예요.

가동 범위(`config/robot.yaml`, 2026-10-28 사용자 지정, 실측 아님): 6축 모두 −180~180°(시뮬 모델과 같음). 범위 안이라도 모델상 자기 충돌·바닥 아래가 되는 자세(예: 관절 2 +90°, 관절 3 ±160°)는 경로 검사에서 막혀요. 모델은 케이블·책상을 모르니 실물 주변은 직접 확인해요.

## Pinch 시나리오 (시뮬 연동)

같은 저장소 `mujoco_sim/`의 Pinch 시뮬(경로 생성 → 마우스·손·음성 수정 → 12개 검증 → 승인)이 정한 궤적을 실물이 따라가요.

1. `calibration-status` → `pinch`를 실행해요. 저장된 보정 검증 → HOME 미리보기 + y → HOME → READY → **시뮬 연결 대기(LINK_WAIT)** 순서예요.
2. 다른 터미널에서 시뮬을 `python -m pinch_sim.app --real-arm ...`으로 실행해요.
   - 연결할 때 두 프로그램이 서로 확인해요: `robot.yaml`(sha·가동 범위·HOME), 모델(sha), 검증 기준(시뮬이 이 프로그램보다 엄격해야 함).
   - 다르면 연결을 거부해요.
3. **ARM_CONFIRM**에서 실물 주변을 확인하고 터미널에서 **y**를 눌러요(ARMED).
   - 그 뒤로는 시뮬의 승인(Y 키, 음성 "이 경로로 할게")이 실물을 움직여요.
   - 시뮬·패널·연결에서 온 y는 확인으로 치지 않아요.
4. 궤적을 받으면 다시 검사해요(TRAJ_CHECK, 별도 스레드, 3 s 안). 하나라도 걸리면 거부하고 이유를 시뮬에 보내요. 이동은 없어요.
   - **시작:** 시작 자세가 지금 실물과 0.35° 안이에요.
   - **가동 범위:** 모든 표본이 가동 범위 ±2.5° 안이고, 제동점도 범위 안이에요.
   - **속도·가속:** 22.5°/s(PV 262) 이하, 가속 70°/s² 이하, 표본 사이 도약이 없어요.
   - **형식:** 정지 → 정지 궤적이고, 표본 간격은 2–20 ms, 길이는 30 s 이하예요.
   - **모델 검사:** 관절 0.5°마다 자기 충돌·바닥을 확인해요.
5. 통과하면 **TRAJ_MOVING**이에요. PC가 궤적을 소유하고 20 Hz로 보내요.
   - Goal = 다음 0.25 s 안에서 그 관절이 가장 멀리 가는 값, PV = 필요한 속도 + 작은 위치 보정, PA = 100.6°/s²예요.
   - 앞섬 0.25 s ≥ 2/read_hz + v_cap/2a라서, 통신이 끊겨도(Watchdog 0) 팔은 궤적의 최대 0.25 s 앞(5.6° 이하)까지만 가서 멈춰요.
6. 틱마다 감시해요. 걸리면 감속 정지 후 FAULT예요.
   - **추종:** 측정과 서보 내부 목표의 차이 > 2° 또는 Following Error 비트, 측정과 궤적 q(t)의 차이 > 3°
   - **구간·범위·속도:** 궤적 구간 ±1° 이탈, 가동 범위 이탈, 과속
   - **명령 읽기값 불일치(유실):** STREAM_WRITE_LOST로 멈추고, 이때 보정은 지우지 않아요
7. 끝: 0.2° 안 + 0.3 s 정지 → 결과 DONE → ARMED예요.

| 정지 수단 | 결과 |
|---|---|
| 터미널 s/space, 패널 ■ | 감속 정지 → HOLDING, 무장 해제 (y로 다시 무장) |
| 시뮬 X / 음성 "멈춰" | 감속 정지 → ARMED 유지 (시뮬에서 이어 가기·되돌아가기) |
| 시뮬 종료·멈춤 (이동·검사 중 ping 1 s, 대기 중 5 s 없음) | 감속 정지 → LINK_WAIT |
| 12 V 스위치 | 유일한 하드웨어 차단 |

- 옵션: `--speed-scale 0.5`(같은 경로를 절반 속도로), `--check-only`(검사만, 이동 없음 - 첫 실기 확인용), `--link`(소켓 경로)
- 설정: `config/motion.yaml`의 `pinch:` 절
- 코드: `src/manipulator_control/pinch_traj.py`(검사·스트리밍 계산), `link.py`(소켓)
- 시험: `tests/test_pinch.py`. 시뮬 쪽 끝에서 끝까지 시험은 `mujoco_sim/tests/test_arm_link_e2e.py`(이 프로그램을 `--fake`로 실행해요)

## 정지 · 종료 · 비상 · 전원

- **키는 터미널에서 눌러요.** SIM 창에서는 teleop 때 기즈모를 끌 수 있고, 그 밖에는 카메라만 움직여요. 오버레이는 영문만 나와요. 창을 닫으면 q와 같아요. 정지와 토크 OFF는 조작 패널에서도 돼요.
- **s / space (정지):**
  - 서보 내부 궤적에서 감속 정지 목표를 계산해서 써요.
  - 실제 정지를 확인해야 HOLDING이에요(토크 유지). 확인에 실패하면 `FAULT / 상태 미확인`과 해당 축을 표시해요.
- **q / Ctrl+C / 창 닫기 (종료):**
  - 이동 중이면 먼저 정지를 확인한 뒤 종료해요.
  - 기본은 **자세 유지(토크 ON)**로 종료하고, 서보가 마지막 목표를 계속 유지해요(PC 감시 없음).
  - Ctrl+C를 두 번 누르면 확인 없이 종료하고, 상태 미확인으로 보고해요.
- **x → y (토크 OFF):** 팔을 받친 뒤에만 써요. 끄면 팔이 자중으로 처져요.
- **통신이 끊기면** `COMM LOST / MOTION UNKNOWN`이 표시돼요.
  - Watchdog이 꺼져 있으면(기본값), HOME 명령을 이미 받은 서보는 HOME까지 **계속 이동할 수 있어요**. PC는 그 이동을 멈출 수 없어요.
  - PC와 상관없이 출력을 끊는 수단은 **12 V 전원 스위치**뿐이에요(끄면 팔이 처져요).
  - 통신이 돌아와도 자동 재보정이나 자동 HOME은 없어요. n(새 세션)을 누르면 다시 시작해요. calibrate는 재정렬부터, prepare / teleop은 저장된 보정을 서보 읽기값으로 다시 검증하고 HOME 미리보기 + y부터예요.
- **전원을 다시 켜거나 서보가 재부팅되면** 다회전 카운트가 1회전 안으로 초기화돼서 저장된 원점(P_ref)이 맞지 않아요.
  - 이때는 PV/PA가 0이 되고 토크가 꺼지므로, prepare / teleop이 이를 감지하고 거부해요(저장된 보정도 지워요).
  - 전원을 다시 켠 뒤에는 calibrate를 다시 해야 해요.
- **토크 OFF(x → y)는 카운트를 바꾸지 않아요.** 그래서 보정은 남고, 다음 prepare / teleop이 그 자리에서 다시 고정(y)해요.

## 구성

```
config/robot.yaml        실기 기구 설정 (null = 미확정)        config/motion.yaml   속도·가속·판정 기준
config/fake_robot.yaml   가상 전용 (지어낸 값)
models/upstream_d654a11/ 참조 저장소 @ d654a11 사본 (수정 금지, 쓰는 사본은 model.UPSTREAM_COMMIT)  models/PROVENANCE.md, manifest.sha256
models/upstream_eeaca7f/ 이전 사본 (2026-10-27까지, 지우지 않음)
models/derived/          표시 모델 viz_scene.xml (측정 팔 + 명령/목표 고스트) + manifest.json
src/manipulator_control/ bus(단일 소유·쓰기 가드) fake_bus state calibration(저장·검증) planner supervisor
                         runner viz(SIM 창, GLFW) gizmo(EE 기즈모) panel(조작 패널) ik terminal logs scan config
                         units control_table model __main__
runtime/                 calibration_state.json (최근 calibrate 하나), fake_calibration_state.json (git 제외)
tools/                   copy_model.py, build_viz_model.py, record_positions.py,
                         import_session_calibration.py (저장 기능 이전 calibrate 로그 -> 저장 보정, 1회용)
tests/                   pytest (하드웨어 없음)
logs/                    sessions/, scans/, fake/ (git 제외)
```

모델 원본은 이 폴더를 담은 저장소예요(`manipulator_control.REF_REPO`). 이 작업공간 쪽 도구는 원본을 읽기만 해요(`git archive`). 모델을 바꾸면 시뮬 쪽에서 커밋한 뒤 `tools/copy_model.py --commit <새 커밋>` → `model.UPSTREAM_COMMIT` → `tools/build_viz_model.py` 순서로 사본을 갱신해요.
