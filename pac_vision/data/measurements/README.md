# data/measurements — 측정 원자료

`pac_vision/README.md`의 "측정 결과와 재현 방법" 절에 적은 수치의 원자료다. 다시 계산하는 명령:

```bash
python tools/recompute_measurements.py          # 카메라 불필요 (numpy, 12절은 pac_vision도 사용)
python -m pytest -q tests/test_measurements.py  # 같은 수치를 테스트로 고정
```

현장 캘리브레이션 세션(`data/sessions/`)은 설치 위치마다 바뀌어 git에 올리지 않는다. 이 폴더는 README 수치의 근거라 올린다.

⚠️ "README 표를 옮김" 파일로 한 계산은 **산술 검산(측정 재현 아님)**이다. 원 측정 로그가 없어 계산식만 다시 확인한다.
스크립트가 저장한 원본 파일(2026-10-02 `..._run1.csv`)로 한 계산만 **원자료로 재계산**이다.

| 파일 | README 절 | 내용 | 출처 |
|---|---|---|---|
| `2026-07_noise_by_distance.csv` | 1 | 거리별 30프레임 표준편차 | README 표를 옮김 (원 측정 로그 없음) |
| `2026-07_z_offset.csv` | 4 | 줄자 거리 vs 보정 전 Z | README 표를 옮김 (원 측정 로그 없음) |
| `2026-07_fx_check.csv` | 5 | 옆으로 0.30 m 이동 전후 X, Z | README 본문을 옮김 (원 측정 로그 없음) |
| `2026-07-25_calib_points.csv` | 7 | 대응점 9개 (기준 좌표, 카메라 좌표, σ, 사용 여부) | README 표를 옮김. `prototype/make_transform.py`의 7점과 같음 |
| `2026-07-25_calib_holdout.csv` | 7 | 검증용 점 3개 (실제, 출력) | README 표를 옮김 (원 측정 로그 없음) |
| `../../prototype/data/verify_log.csv` | 8 | 사람 검출 정적 검증 로그 274행 | 스크립트가 쓴 원본 (prototype에 보존) |
| `2026-10-02_rgb_mode_requests.csv` | 10 | 요청 크기 → 실제 받은 크기·FOURCC | 측정 터미널 출력을 옮김. 비교 영상은 보존되지 않음 |
| `2026-10-02_poseA_pairs_run1.csv` | 11 | 정합 쌍 12개 (RGB u_ref·v_ref, depth u_d·v_d, dx, dy, Z) | `scripts/check_camera.py`가 저장한 원본 |
| `2026-10-02_poseA_pairs_run2_transcribed.csv` | 11 | 정합 쌍 8개 (u, v, dx, dy, Z) | **터미널 출력을 옮겨 적음** (원본 파일 없음) |
| `2026-10-02_poseA_points_run1.csv` | 11 | depth 클릭 점 15개의 카메라 좌표와 30프레임 표준편차 | `scripts/check_camera.py`가 저장한 원본 |
| `2026-10-02_poseA_tape.csv` | 11 | Z 비교에 쓴 줄자 값 (하우징 앞면 → 상자 앞면) | 손 기록을 옮김 |
| `2026-10-14_pose_pairs.csv` | 12 | 정합 쌍 14개 (책상 12, 배경 2), 열은 run1과 같음 | `scripts/check_camera.py`가 저장한 원본 (측정 중 이상해서 지운 쌍 4개는 들어 있지 않음) |
| `2026-10-14_pose_pairs_run2.csv` | 12 | 같은 자세의 정합 쌍 10개 (A4 주변, 반복 2쌍 포함) | `scripts/check_camera.py`가 저장한 원본 (잘못 클릭해 지운 쌍 1개는 들어 있지 않음) |
| `2026-10-14_pose_a4corners.csv` | 12 | 같은 자세의 A4(두 장) 바깥 꼭짓점 1-4의 RGB 픽셀과 Z (책상 평면 교점) | 측정 중 임시 도구로 기록한 원본 |
| `2026-10-14_pose_calib_rounds.csv` | 12 | 캘리브레이션 2회의 클릭 5점(RGB 픽셀), 카메라 좌표, 점별 오차, base 점 오차, 변 오차, 평면 잔차 | 세션 파일(`data/sessions/`, git 제외)에서 뽑음. 1회차 세션 원본은 `tests/data/sample_session.npz` |
| `2026-10-14_pose_check.csv` | 12 | `calibrate.py --check` 클릭 8개(재점검, 이음매 S1·S2, A4 밖 ext_a~d)의 RGB 픽셀, 측정 좌표, 기대 좌표, 오차 | 터미널 출력을 옮김 (`--check`는 클릭을 파일로 남기지 않음). 첫 클릭 픽셀은 측정값에서 되돌린 근사 |
| `2026-10-14_pose_objects.csv` | 12 | 물체 좌표(컵·상자) vs 테이프 표시점: detect 선택값, 설정 비교, 윗면 depth 유효율 | 터미널 출력을 옮김 |
| `2026-10-15_pose_calib.csv` | `config/pose_presets/README.md` "확인된 범위와 한계" | 정합 프리셋 `astra_unit1.yaml`로 한 캘리브레이션 1회: 클릭 5점(RGB 픽셀), 카메라 좌표, 점별 오차, base 점 오차, 변 오차와 변별 허용값, 평면 잔차, 기울기, 카메라 높이. 열은 `2026-10-14_pose_calib_rounds.csv`와 같고 뒤에 `tol*_mm`·`a4_side_tol_m_setting`·`tilt_deg`·`cam_height_m`을 더함 | 세션 파일(`data/sessions/`, git 제외)을 `replay_session`으로 다시 계산해 뽑음 (저장 결과와 차이 0) |
| `2026-10-14_pose_center_methods.csv` | 12 | 물체 중심 계산 방식 비교 (top_band / symmetric / bbox_top): 뚜껑 컵 외삽점 a·c, 상자 긴 변 X·Y, 각 2회 | 저장한 30프레임 평균 프레임(`data/sessions/`, git 제외)에서 다시 계산. 상자 X 첫 시도(15 mm 어긋나게 놓음)도 `attempt`로 구분해 남김 |

열 단위: 길이는 m, 표준편차는 mm, 픽셀은 640×480 기준. `where` 열은 `check_camera.py`가 붙인 화면 위치 이름(한글).

## 2026-10-14 정합 쌍: RGB→depth 계수 맞춤 규칙 (맞춤 전에 정함)

`tools/fit_rgb_to_depth.py`가 이 규칙을 그대로 따른다.

- Z는 카메라 광학 Z(m) = depth_mm/1000 − z_offset_m (Z_OFFSET 적용 후). `check_camera.py`가 쌍 CSV에 남긴 Z, 책상 평면 교점의 Z,
  물체 점의 Z가 모두 이 정의다.
- 맞춤에는 Z ≤ 1.5 m 쌍(책상 영역)만 쓴다. 더 먼 배경 쌍은 거리 식 참고 계산에만 쓴다.
- 연속한 두 쌍의 RGB 클릭이 5 px 안이면 같은 꼭짓점을 반복해 찍은 것으로 본다. 그 두 쌍의 dx 또는 dy 차이가 3 px를 넘으면
  그 꼭짓점의 쌍은 모두 맞춤에서 뺀다 (클릭 흔들림이 식의 차이보다 큰 점). 2026-10-14 자료(두 파일, 24쌍)에서는 화면 왼쪽 끝 꼭짓점의 두 쌍(dx 44 / 39)이 해당한다.
- dx는 거리 식 a + b·u + c·v + d/Z로 맞춘다. dy는 고정값과 거리 식의 leave-one-out RMS 차이가 0.3 px 미만이면 고정값을 쓴다.
- ⚠️ 이 자세에서는 화면 세로 위치 v와 Z의 상관이 커서(−0.86) c와 d가 서로 바뀌어 맞을 수 있다. 맞춘 d를 물리적 시차 계수로 해석하지 않는다.
  시차는 같은 화면 높이에서 Z만 다른 쌍(직접 시차)으로 따로 본다.

## 설치 자세 이름과 정합 쌍 파일 (여러 자세 합친 맞춤)

파일 이름은 측정 날짜 기준이다. 같은 날 두 번째 자세는 `<날짜>_<시각>_pose_pairs.csv`로 짓는다.
자세 이름은 파일 이름에 넣지 않고 `tools/fit_rgb_to_depth.py --pose 이름=CSV[,CSV]` 입력과 이 표에 적는다.

| 자세 | 날짜 | 카메라 설치 | 쌍 파일 | 쌍 (맞춤에 쓴 쌍) |
|---|---|---|---|---|
| A | 2026-10-02 | 위에서 비스듬히 책상을 봄 (README 11절, 높이·각도 수치 기록 없음). 쌍 Z 0.86-1.11 m | `2026-10-02_poseA_pairs_run1.csv`, `2026-10-02_poseA_pairs_run2_transcribed.csv` (터미널 출력을 옮김) | 20 (20) |
| B | 2026-10-14 | 책상면 위 51 cm, 광축 약 21° 아래 | `2026-10-14_pose_pairs.csv`, `2026-10-14_pose_pairs_run2.csv` | 24 (20: 반복 차이 > 3 px 꼭짓점 2쌍, 배경 Z > 1.5 m 2쌍 제외) |

```bash
python tools/fit_rgb_to_depth.py \
    --pose A=data/measurements/2026-10-02_poseA_pairs_run1.csv,data/measurements/2026-10-02_poseA_pairs_run2_transcribed.csv \
    --pose B=data/measurements/2026-10-14_pose_pairs.csv,data/measurements/2026-10-14_pose_pairs_run2.csv
python tools/recompute_measurements.py --section 13
```

- 규칙은 위 "계수 맞춤 규칙"과 같다 (파일마다 Z ≤ 1.5 m, 반복 쌍 제외). 가중치 없음. 그대로 둔 반복 쌍은 쌍 2개로 들어가고,
  그 영향은 꼭짓점 단위 leave-one-out(반복 쌍을 함께 뺌)으로 따로 출력한다.
- 출력의 정리 표에 자세, 파일 안 쌍 번호, 반복 표시(꼭짓점 번호 뒤 `R`), 맞춤 사용 여부와 제외 사유가 나온다.
- 상태: 시험 중. 합친 계수는 아직 설정에 넣지 않았다.
