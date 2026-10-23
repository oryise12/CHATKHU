# config/pose_presets — RGB→depth 정합 계수 프리셋

카메라 실물마다 미리 맞춘 정합 계수다. 형식은 `config/pose.yaml`과 같고(`camera.rgb_to_depth`만), 그대로 복사해 쓴다.

```bash
cd ~/pac_ws/src/26_project_manipulator/pac_vision
cp config/pose_presets/astra_unit1.yaml config/pose.yaml
```

- `config/pose.yaml`은 git 제외 파일이다. 복사해도 저장소는 바뀌지 않는다. 이미 있으면 덮어쓰므로 먼저 사본을 남긴다.
- `config/pose.example.yaml`은 형식 예시(다른 설치 자세 한 곳의 값)다. `pose.yaml`로 쓰지 않는다.
- 프리셋 파일은 고치지 않는다. 계수를 바꾸려면 아래 명령으로 다시 만들고 sha256을 이 문서에 갱신한다.

## astra_unit1.yaml

| 항목 | 값 |
|---|---|
| 카메라 | Orbbec Astra Pro **실물 1** (몸체 스티커로 구분. 스티커 문구: ⚠️ 미기입). USB 일련번호(RGB `SN0001`)로는 실물을 구분할 수 없다 |
| 계수 | dx: 거리 식 `a + b·u + c·v + d/Z` (a 8.580472, b −0.012502, c 0.01603, d 14.56444) / dy: 고정 8.475 px |
| 만든 방법 | 설치 자세 2곳(A: 2026-10-02, B: 2026-10-14)의 정합 쌍을 합쳐 맞춤, 자세마다 20쌍 사용 |
| 원자료 | `data/measurements/2026-10-02_poseA_pairs_run1.csv`, `2026-10-02_poseA_pairs_run2_transcribed.csv`, `2026-10-14_pose_pairs.csv`, `2026-10-14_pose_pairs_run2.csv` (자세 설명은 [`data/measurements/README.md`](../../data/measurements/README.md)) |
| 자세별 leave-one-out RMS | dx A 2.56 / B 2.72 px, dy A 4.05 / B 2.19 px (파일 머리 주석과 같음) |
| sha256 | `2fe20aab9706f0d7f6753ca7c9808e7a94eead91071887c9ba17183023a2e2f6` |
| 만든 날짜 | 2026-10-14 (파일 머리 주석의 시각) |

### 다시 만들기 (카메라 불필요)

```bash
cd ~/pac_ws/src/26_project_manipulator/pac_vision
python tools/fit_rgb_to_depth.py \
  --pose A=data/measurements/2026-10-02_poseA_pairs_run1.csv,data/measurements/2026-10-02_poseA_pairs_run2_transcribed.csv \
  --pose B=data/measurements/2026-10-14_pose_pairs.csv,data/measurements/2026-10-14_pose_pairs_run2.csv \
  --dy fixed --fragment config/pose_presets/astra_unit1.yaml
```

파일 머리 주석의 `명령:` 줄과 같은 명령이다. 다시 만들면 첫 줄의 생성 시각이 바뀌어 sha256도 바뀐다. 계수 값은 같아야 한다.
2026-10-14 확인: 이 명령으로 만든 계수 값이 먼저 만들어 고정해 둔 파일(sha256 `7d79914a…0b33`)과 같다. 두 파일은 머리 주석의 생성 시각과 `--fragment` 경로만 다르다.

### 확인된 범위와 한계

- 맞춤에 쓰지 않은 설치 자세 1곳(기준 종이를 놓은 면에서 렌즈까지 높이 약 82 cm, 광축이 수평에서 약 53° 아래, 2026-10-14)에서
  이 계수로 캘리브레이션했을 때 A4 변 길이 오차는 최대 약 1 cm(+11 mm, 두 번째 실행 +8 mm)였다. 그래서 `calibrate.py`의 변 검사(기준 5 mm)는 ❌로 표시된다.
  이 확인의 원자료는 저장소에 없다(기록만).
- 2026-10-15, 카메라를 옮겨 다시 설치한 자세(책상면 위 카메라 높이 0.812 m, 광축이 수평에서 약 53° 아래 — 위 2026-10-14 자세와 높이·각도가 비슷함)에서 이 계수로 캘리브레이션 1회:

  | 항목 | 값 |
  |---|---|
  | base 점 오차 (꼭짓점 4점 변환으로 예측) | 5.4 mm |
  | A4 변 오차 1-2 / 2-3 / 3-4 / 4-1 | +4.0 / −0.7 / +9.4 / **+11.9** mm |
  | 5점 RMS / 평면 맞춤 잔차 | 5.7 / 2.3 mm |

  변 오차 최대 +11.9 mm로 2026-10-14와 같은 수준(약 1 cm)이다. 변 검사 기준 5 mm(`config/thresholds.yaml`)로는 ❌이고,
  이 실행은 따로 만든 설정 폴더(`--config-dir`)에서 `a4_side_tol_m`만 0.015로 둔 임시 기준으로 통과·저장했다. 저장소 `thresholds.yaml` 값은 바꾸지 않았다(팀 허용 오차 🔲).
  원자료: [`data/measurements/2026-10-15_pose_calib.csv`](../../data/measurements/2026-10-15_pose_calib.csv) (세션 파일에서 뽑음, 세션은 git 제외라 다시 계산은 세션이 있는 컴퓨터에서만).
  같은 자세의 `--check`·`show_base_coords` 점별 확인은 기록 없음.
- **다른 Astra 실물에는 쓰지 않는다.** 실물마다 RGB와 depth 센서의 상대 위치가 달라 계수가 다를 수 있고, 좌표가 수 cm 어긋날 수 있다.
  다른 실물이면 `CALIBRATION_GUIDE.md`의 정합 쌍 측정으로 그 카메라의 `pose.yaml`을 만든다.
- 테스트: `tests/test_config_overlay.py::test_pose_preset_copies_as_pose_yaml` (이 파일을 `pose.yaml`로 복사했을 때 설정 로더의 키 검사 통과, 계수 값).
