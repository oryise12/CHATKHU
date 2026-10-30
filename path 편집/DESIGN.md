# 설계

## 경계와 좌표계

`core.py`, `editor.py`, `validation.py`, `storage.py`는 UI와 입력 장치에 독립적이다. 기존 `viewer.py`는 마우스 검증 UI이고, 신규 `webcam.py`는 웹캠·MediaPipe·핀치·OpenCV만 담당한다. `ros_bridge.py`는 ROS 콜백이 Editor를 직접 만지지 못하도록 명령 큐를 제공한다.

기존 경로 편집은 `robot_base` m 단위 프로토타입이다. 웹캠 모드는 같은 수치 단위를 편집기의 가상 길이로 사용하지만 저장 frame을 반드시 `virtual_edit_frame`으로 둔다. normalized landmark XYZ는 `hand_to_editor`에서 축별 gain/sign을 적용한 상대 입력이다. 카메라 depth나 절대 미터 위치로 해석하지 않는다. 향후 depth 어댑터는 이 함수만 교체할 수 있다.

## correction field와 작은 재수정

원본 대응좌표 `xi`는 최초 폴리라인 호 길이에서 정하고 다시 정의하지 않는다. 각 correction은 scalar local profile `w(xi)`, 중심 벡터 `delta`, vector profile `profile_residual(xi)`를 가진다.

```text
component_i(xi) = w_i(xi) * delta_i + profile_residual_i(xi)
visible(xi) = S(sum_i gamma_i * frozen_boundary_component_i(xi))
path(xi) = original(xi) + visible(xi)
```

이 확장이 필요한 이유는 기존 field가 서로 다른 방향과 서로 다른 profile의 합일 수 있기 때문이다. 그것을 중심의 단일 `delta`와 새 단일 profile로 흡수하면 중심은 맞아도 주변 곡선이 달라진다. 실제 재현 사례에서는 1 µm 재수정이 주변을 약 6.319322 mm 바꿨다.

새 드래그는 드래그 시작 직전의 보이는 offset 전체를 새 profile knot에서 샘플링하고, 그 위에 이번 `boundary * weight * delta`를 더한다. 흡수하지 않고 남은 field를 뺀 벡터 계수를 `delta + profile_residual`로 분해한다. 따라서 기존 곡선과 서로 다른 방향 성분은 knot 전체에서 보존되고 새 입력만 국소적으로 추가된다. 1 µm 입력의 전체 최대 변화는 약 1 µm로 수렴한다. deadband로 문제를 숨기거나 임계값 위에서 profile을 바꾸지 않는다.

끝점 감쇠도 correction 생성 당시 knot에서 동결한다. 새 knot 삽입이 기존 표시 곡선을 다시 맞추게 하지 않는다. 총 offset은 최종 벡터 합에 norm saturation을 적용한다.

## 끝점

`Editor.begin_drag`와 `build_drag_candidate`는 `xi == 0` 또는 `xi == 1`을 거부/무변경 처리한다. 따라서 UI를 우회한 core 호출도 memory를 바꾸지 않는다. 끝점 안쪽은 원본 길이에 대한 smoothstep 보호 가중치를 계속 사용하므로 영향이 부드럽게 0에서 증가한다. `both`에도 같은 규칙을 적용한다.

## 시간 정책

각 `PathData`에는 원본 길이 `L0`, 원본 기준시간 `T0`, 현재 편집 기하의 의미상 길이 `timing_length`를 보존한다. `never_shorten=true`일 때:

```text
T_length = T0 * max(1, L / L0)
T_speed  = 1.875 * L * speed_safety / max_ee_speed
T        = ceil(max(T_length, T_speed) / sample_dt) * sample_dt
```

실행용 재샘플링은 폴리라인 코너를 약간 잘라 실제 샘플 배열 길이를 바꿀 수 있다. 그래서 동일 실행을 다시 검사할 때는 저장한 `timing_length`를 사용한다. 검사 반복으로 시간이 계속 늘거나 기준시간이 덮어써지지 않는다.

## 핀치 상태

화면 반전 후의 한 프레임을 추적과 표시가 공유한다. 커서는 landmark 4·8의 화면 중점이고, 편집 입력은 같은 두 landmark XYZ의 평균이다. 화면 거리는 width/height를 적용한 픽셀 단위로 계산해 영상 종횡비를 반영한다. 핀치 거리를 손목-중지 MCP와 검지-소지 MCP 크기로 정규화한다.

open 관측 후에만 다음 close가 선택을 시작한다. 선택은 close 전이 순간 한 번만 하고 경로/xi를 해제까지 고정한다. 필터된 동일 샘플을 시작 기준과 첫 업데이트에 사용하므로 시작 점프가 없다. 추적 상실은 release가 아니다. 마지막 preview를 동결하고 재인식만으로 갱신하지 않으며, timeout 뒤 취소한다. 다시 open을 확인한 뒤에만 새 핀치를 허용한다.

## ROS 권한과 순서

ROS 콜백은 `Pending`을 bounded queue에 넣고 결과를 기다릴 뿐이다. OpenCV owner thread가 frame 사이에 `drain()`하여 `Editor.execute_command`를 순차 호출한다. deadline이 지난 요청은 commit 전에 취소되어 나중에 적용되지 않는다. `request_id`는 프로세스 수명 동안 캐시에 남아 같은 내용이면 같은 결과를 돌려주고, 다른 내용 재사용은 거부한다. 이로써 undo/redo 중복 실행을 막는다.

서비스 응답과 상태 토픽에는 실제 editing state, mode, revision, validation status, 최근 명령 수락/거부와 이유가 들어간다. 상태 토픽은 표시용이며 다시 명령으로 실행하지 않는다. 통합 모드에서는 `ros_bridge.py`만 서비스 서버다. 기존 `state_node.py`는 Editor가 없는 독립 음성 시험 시뮬레이터라 동시에 실행하지 않는다.

## 저장 schema

schema 2는 frame, `profile_residual`, `reference_length`, `reference_duration`, `timing_length`를 필수로 검사한다. schema 1을 schema 2로 조용히 읽지 않는다. `migrate_v1_session`은 구 schema의 scalar profile을 zero residual로 정확히 옮기고 원본에서 시간 참조를 생성한 뒤, 현재 loader와 validator로 전체 재검사하고 원자적으로 저장한다.

## 검사의 범위

프로토타입 검사는 유한 기하, xi 대응, 끝점, 총변형, 마지막 실효 변화, 일정 시간 간격과 EE 속도를 검사한다. 실제 카메라 보정, depth, IK, 관절 한계, 충돌, 특이점, 동역학, 토크, 로봇 실행은 범위 밖이며 명시적으로 `NOT_EVALUATED`다.
