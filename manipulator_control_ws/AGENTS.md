# AGENTS.md — manipulator_control_ws 작업 규칙

- 이 작업공간이 코드·환경·모델 사본·로그·문서의 기준 위치예요.
- 2026-10-28부터 이 작업공간은 저장소 `26_project_manipulator` 안(`manipulator_control_ws/`)에 있어요. 모델 원본은 그 저장소의 커밋(`model.UPSTREAM_COMMIT`)이고, 이 작업공간의 도구는 원본을 읽기만 해요(`git archive`). 이 작업공간 일을 하면서 시뮬 쪽 파일·venv는 바꾸지 않아요. 원본의 `build_scene`도 실행하지 않아요(원본 URDF를 덮어써요).
- 실제 모터를 움직이는 명령(`calibrate`, `prepare`, `teleop`)은 사용자가 팔 옆에서 직접 실행하고 확인(Enter/y)할 때만 진행해요. 무인 자동 토크 ON이나 자동 HOME 시험은 하지 않아요.
- 서보 EEPROM과 설정(Drive Mode, Homing Offset, Startup, Watchdog, Shutdown, PWM, Return Delay, ID, Baud)은 바꾸지 않아요. 유일한 예외는 Operating Mode 4로, 설정에서 허용하고 사용자가 확인한 경우에만 바꿔요.
- 다른 프로그램(Wizard 등)이 포트를 쓰고 있으면 그 프로그램을 종료하지 않아요. 사용자에게 닫아 달라고 해요.
- `config/robot.yaml`의 `null`은 "미확정"이에요. 측정 없이 값을 채우지 않아요.
- 보정은 `runtime/calibration_state.json` 하나(최근 calibrate)만 둬요. 손으로 만들거나 고치지 않아요. 쓸 수 있는지는 서보 읽기값(마지막으로 보낸 PA/PV)으로만 판단해요(`calibration-status`). 통신이 끊기면 지우고 다시 calibrate해요.
- 가상 실행 값은 `config/fake_robot.yaml`에만 둬요.
- 변경한 뒤에는 `source scripts/env.sh && python -m pytest -q`를 실행해요. 상태가 바뀌면 `CURRENT_STATE.md`를 갱신해요.
- 실제로 실행하지 않은 시험은 완료로 기록하지 않아요.
- Pinch 연동(2026-10-26, 사용자 요청): `pinch` 명령이 Pinch 시뮬(`pinch_sim.app --real-arm`)과 소켓으로 연결돼요.
  - `config/robot.yaml`은 시뮬도 읽어요(읽기만). 가동 범위·HOME을 바꾸면 연결할 때 두 쪽 hello 비교에 바로 반영돼요.
  - 시뮬 쪽 코드는 참조 저장소의 `mujoco_sim/src/pinch_sim/real_arm.py`, `arm_link.py`에 있어요. 이 작업공간에서 그 저장소를 고치지는 않아요.
  - 무장(y)은 터미널에서 받아요. 2026-10-29 사용자 결정으로 이 프로그램이 직접 띄우는 작은 "실물 무장" 확인 창(panel.py, 출처 `arm_dialog`)의 y도 받아요. 이 창은 무장(ARM_CONFIRM)만 확인하고 HOME·토크·시작은 확인하지 못해요. 시뮬·브라우저·연결·패널의 다른 버튼에서 온 확인은 여전히 운용자 확인이 아니에요.
  - 받은 궤적은 시뮬 검증을 믿지 않고 여기서 다시 검사해요(`pinch_traj.check`).
