# pac_vision 인터페이스 (임시안 v0.4)

> 작성: 윤수, 2026-09-30 / 갱신: 2026-10-03 (물체 토픽·좌표 정의, 화면 크기 규칙, 실제 함수 시그니처). 팀 통합 회의 전 임시안이다. 이름은 바뀔 수 있다.
> 모든 이름은 `config/frames.yaml`에서 관리하며, 코드에는 직접 쓰지 않는다.
> 표기: ✅ 확정 / 💡 제안 / ⚠️ 주의 / 🔲 미정

## 공통 규약
- 단위: m (depth 원본 영상만 mm) ✅
- 축 방향: REP-103 ✅
  - 기준·로봇 좌표계: +X 앞, +Y 왼쪽, +Z 위. **책상면 z = 0**
  - 카메라 광학 좌표계: +X 오른쪽, +Y 아래, +Z 카메라 정면 (depth 좌우반전 보정 후)
- 변환: `P_base = R @ P_cam + t` (`config/cam2base.yaml`, 캘리브레이션으로 생성)

## 좌표계 (TF frame)
| 이름 | 의미 | 발행 | 상태 |
|---|---|---|---|
| `base_ref` | 빈 책상에 표시한 base 십자 (원점), X 화살표 방향이 +X. 로봇 앞면을 화살표에 맞춰 놓는다 | 루트. 물체 토픽의 `frame_id` | 💡 |
| `base_link` | 실제 로봇 base | `base_ref`→`base_link` static TF, 로봇 도착 전까지 항등 변환 | 💡 / 🔲 측정 |
| `camera_depth_optical_frame` | Astra depth 광학 좌표계 (좌우반전 보정 후) | `base_ref`→카메라 static TF (캘리브레이션 결과). 아직 발행하지 않음 | 💡 |

```
base_ref
 ├── base_link                    (로봇 쪽, 로봇 모델이 이 아래로 이어짐)
 └── camera_depth_optical_frame   (캘리브레이션 결과)
```

## 토픽 (`detect_objects.py --ros`)
| 토픽 | 타입 | 내용 | 주기 | 상태 |
|---|---|---|---|---|
| `/pac/objects` | `geometry_msgs/PoseArray` | 인식된 물체 전체의 윗면 중심. `pose.position` = (x, y, z_top), `orientation` = 단위 쿼터니언 (방향 정보 없음). 좌표를 못 구한 물체(depth 부족)는 빠진다 | 카메라 프레임마다 | 💡 구현, Humble 실측 (2026-10-14) |
| `/pac/target_point` | `geometry_msgs/PointStamped` | 화면에서 클릭으로 선택한 물체의 윗면 중심. 선택이 없거나 놓치면(약 1 s) 발행하지 않는다 | 선택 중 프레임마다 | 💡 구현, Humble 실측: frame_id `base_ref`, 값이 화면과 같음 (2026-10-14) |

- 두 토픽 모두 `header.frame_id` = `base_ref` (`frames.yaml`의 `base_frame`), `header.stamp` = 발행 시각.
- 받는 쪽 확인: `ros2 topic echo /pac/target_point`
- 💡 **`/pac` vs `/manipulator`:** v0.3에서는 팀 코드의 `/manipulator/edit_command`, `/manipulator/edit_state`에 맞춰 `/manipulator/target_point`를 제안했다.
  지금 구현은 실행 시나리오대로 `/pac/objects`, `/pac/target_point`다. 이름은 `config/frames.yaml` 한 곳만 바꾸면 되므로 팀 통합 때 정한다.
- 카메라 영상 토픽(`/camera/color/image_raw` 등)은 아직 발행하지 않는다. 카메라를 여러 프로그램이 같이 써야 하면 그때 다시 정한다.

## 물체 좌표 정의 💡 (경로 생성 담당과 합의 🔲)
- **base_ref 기준 물체 윗면 중심 (x, y, z_top)**, m. 책상면이 z = 0이므로 z_top이 곧 물체 높이다.
- 계산: YOLO 박스(테두리 10 % 안쪽) 안 depth 픽셀 → base 좌표 → 책상면 위 1 cm ~ 0.5 m인 점만 남김
  → `z_top` = 남은 점 높이의 90 백분위, `(x, y)` = z_top 아래 0.8 cm 두께(윗면 띠) 점들의 중앙값.
  카메라가 비스듬히 보면 옆면이 같이 보여, 단순 중앙값은 윗면보다 낮고 카메라 쪽으로 치우치기 때문이다. 기준값은 `config/thresholds.yaml`의 `objects`.
- 그리퍼 접근 오프셋(윗면에서 얼마나 위/옆에서 접근할지)은 받는 쪽에서 더한다.
- 납작한 물체(약 1 cm 이하), 검정·유리·광택 물체는 depth가 없어 좌표가 나오지 않을 수 있다.
- ⚠️ RGB↔depth 정합 오프셋(dx, dy)이 틀리면 R·t로 보정되지 않고 물체 좌표의 일정한 오프셋으로 남는다. 줄자로 잰 위치와 비교해 확인한다 (README "주의" 절).
- ⚠️ **알려진 한계 (2026-10-14 실측, 팀 협의 필요 🔲):** 카메라가 낮게(광축 약 20° 아래) 보면 수평 윗면의 depth가 일부만 나와 (x, y)가 **카메라 쪽으로 물체 깊이 크기의 약 절반**
  치우친다 (컵 2.5-3.8 cm, 상자 4.6 cm). x·z는 3 mm 안. 입구가 열린 용기는 카메라 쪽 테두리로 반지름만큼 치우친다. 받는 쪽은 접근 여유를 두고,
  계산 방식(예: 박스 아래 끝·윤곽 광선과 책상면 교점)과 카메라 설치 각도 기준은 통합 때 정한다 (README "알려진 한계").

## 화면 픽셀 규칙 (RGB 화면 → depth 픽셀) ✅ 측정 2026-10-02
- Astra RGB 640×480은 4:3 화면 전체다. 16:9 모드(1280×720, 1920×1080)는 그 위아래를 잘라낸 **가운데 띠**다 (README 측정 10절).
- 그래서 크기 `rgb_size = (W, H)`인 화면의 픽셀 (u, v)는 640×480 기준 픽셀로 이렇게 옮긴다:
  ```text
  s  = 640 / W
  u' = u * s
  v' = v * s + (480 - H * s) / 2        # 1280x720이면 0.5배 후 +60 px
  ```
  좌우반전 화면(`rgb_mirrored=True`)은 먼저 `u = (W - 1) - u`.
- 그다음 depth 픽셀 (거리 식, `config/camera.yaml`의 `rgb_to_depth`):
  ```text
  u_d = sx * u' + (ax + bx * u' + cx * v' + dx / Z)
  v_d = sy * v' + (ay + by * u' + cy * v' + dy / Z)
  ```
  Z는 그 점의 카메라 광학 Z(m, Z_OFFSET 적용 후). b = c = d = 0이면 예전의 고정 dx·dy와 같고, 옛 형식(`dx: 23`)도 읽는다.
  거리 항(d)이 있으면 Z가 필요하다: 책상 위 점은 책상 평면 교점의 Z로 고정점 반복하고(`rgb_to_depth_on_plane`, `Cam2Base.pixel_to_desk`),
  `pixel_to_base`는 그 자리 depth의 Z로 반복한다. Z를 모르면 `z_ref_m`(기본 1.0 m)을 쓰고 경고를 남긴다.
  계수는 설치 자세마다 `tools/fit_rgb_to_depth.py`로 맞춘다 (README "3) 카메라 확인").
- 경로 편집기는 1280×720 화면을 쓰므로 `rgb_size=(1280, 720)`, `rgb_mirrored=True`로 넘기면 된다. 이 규칙은 함수 안에서 처리한다.

## 모듈 인터페이스 (실제 시그니처, v0.1.0)
핵심 모듈은 numpy·PyYAML만 쓴다 (`import pac_vision`이 rclpy·cv2·openni를 불러오지 않음).

```python
from pac_vision.config import load_config
from pac_vision import AstraDepth                      # openni는 open() 할 때 불러온다
from pac_vision.session import require_cam2base

cfg   = load_config("config", ("camera",))             # {"camera": {...}}
depth = AstraDepth.from_config(cfg["camera"]).open()   # 또는 with AstraDepth.from_config(...) as depth:
conv  = require_cam2base("config/cam2base.yaml")       # 없거나 예시 파일이면 MissingCalibration ("먼저 calibrate.py를 실행하세요")

frame = depth.read()            # np.ndarray uint16 (480, 640), mm, 좌우반전 보정 완료. 새 프레임까지 기다림
frame = depth.latest()          # read()와 같고 실패하면 None
avg, frac = depth.read_average(30)   # 서로 다른 30프레임 평균 (float32 mm, 픽셀별 유효 비율)

# 화면 RGB 픽셀 → base_ref 좌표 (m)
r = conv.pixel_to_base(u, v, frame, rgb_mirrored=False, rgb_size=(640, 480), half=2)
r.valid     # False면 쓰지 않는다
r.xyz       # np.array([x, y, z]), base_ref, m (valid가 False면 nan)
r.reason    # "out_of_image" / "no_depth" / "too_close" / "too_far"

# base_ref 좌표 → 화면 RGB 픽셀 (경로를 영상 위에 그릴 때). 입력 (3,) 또는 (N, 3), 출력 (..., 2), 카메라 뒤는 nan
uv = conv.base_to_pixel(xyz, rgb_mirrored=False, rgb_size=(640, 480))

conv.camera_to_base(p_cam)      # (..., 3) → (..., 3)
conv.base_to_camera(p_base)
conv.cam                        # CameraModel (보정할 때 쓴 카메라 설정 사본으로 만든다)
depth.close()
```

그 밖의 공개 함수:

| 함수 | 모듈 | 하는 일 |
|---|---|---|
| `CameraModel.from_config(camera_cfg)` | `cam2base` | intrinsics, Z_OFFSET, rgb_to_depth를 담은 카메라 모델 |
| `CameraModel.screen_to_ref(u, v, rgb_size=None, mirrored=False)` / `ref_to_screen` | `cam2base` | 화면 픽셀 ↔ 640×480 기준 픽셀 (위 규칙) |
| `CameraModel.rgb_to_depth(u, v, rgb_size=None, mirrored=False, z_m=None)` / `depth_to_rgb(ud, vd, rgb_size=None, mirrored=False, z_m=None)` | `cam2base` | 화면 픽셀 ↔ depth 픽셀 (거리 식이면 z_m 필요) |
| `CameraModel.rgb_to_depth_on_plane(u, v, plane, rgb_size=None, mirrored=False)` | `cam2base` | 평면 위 점: 평면 교점 Z로 고정점 반복 → (ud, vd, 카메라 좌표) |
| `Cam2Base.desk_plane()` / `Cam2Base.pixel_to_desk(u, v, rgb_mirrored=False, rgb_size=None, plane=None)` | `cam2base` | 책상면(base z = 0)의 카메라 평면 / 화면 픽셀 → 책상면 위 base 좌표 |
| `CameraModel.backproject(ud, vd, depth_mm)` / `project(p_cam)` | `cam2base` | depth 픽셀+mm ↔ 카메라 좌표 |
| `solve_calibration(depth_mm, clicks_rgb, cam, layout, thr, mode="plane", rgb_size=None)` | `cam2base` | 클릭 5점(base, 꼭짓점 1-4)으로 R, t와 검사 결과 (`CalibResult`) |
| `fit_plane(points, iters, inlier_m, max_points, seed)` / `intersect_ray_plane(dirs, plane)` / `kabsch(a, b)` | `cam2base` | 평면 맞춤(RANSAC+최소제곱), 광선-평면 교차, 강체 변환 |
| `top_center(points_base, min_height_m, max_height_m, top_percentile, top_band_m, min_pixels)` | `objects` | base 좌표 점들 → 윗면 중심 (`TopCenter.valid`, `.xyz`) |
| `box_to_depth_roi(box_xyxy, cam, rgb_size=None, shrink=0.1, mirrored=False)` | `objects` | 화면 박스 → depth 픽셀 사각형 |
| `load_cam2base(path)` / `write_cam2base(...)` / `save_session(...)` / `replay_session(path, mode=None, camera_cfg=None)` | `session` | 보정 결과·세션 저장과 읽기, 재계산 |
| `ObjectPublisher(frames)` → `.publish_objects(xyz_list)`, `.publish_target(xyz)`, `.close()` | `ros_pub` | ROS 2 발행 (rclpy는 이 모듈에서만) |

- v0.3에 있던 `pixels_to_base`(손가락 두 점 중 손에 해당하는 depth 고르기)는 구현하지 않았다. 손 3D 연결은 지금 요구사항 밖이며, 필요해지면 다시 정한다.
- ⚠️ RGB와 depth는 따로 읽으므로 두 프레임 사이에 짧은 시간차가 있다. 빨리 움직이는 대상은 오차가 커질 수 있다.

### 경로 편집기 연결 지점 (💡 제안, 수정은 경로 편집기 담당)
- 경로 편집기 `DESIGN.md`에 "향후 depth 어댑터는 `hand_to_editor` 함수만 교체할 수 있다"고 되어 있어 이 함수를 교체 지점으로 제안한다.
- 반전된 화면 픽셀과 화면 크기를 그대로 `conv.pixel_to_base(u, v, depth.latest(), rgb_mirrored=True, rgb_size=(1280, 720))`에 넘기면 된다.
  현재 `hand_to_editor`는 화면 크기를 받지 않으므로 인자 추가가 필요하다.

## 설정 파일
| 파일 | 내용 | git |
|---|---|---|
| `config/camera.yaml` | 장치 이름(자동 탐색), OpenNI 경로 기본값, intrinsics, Z_OFFSET, 좌우반전, rgb_to_depth 옛 형식 기본값, 화면 글꼴 | ✅ |
| `config/local.yaml` (git 제외, 예시 `local.example.yaml`) | 컴퓨터별 값만: `camera.rgb.device`, `camera.openni.redist`, `camera.ui.font`, `detect.model`, `detect.device` | ✅ 2026-10-14 |
| `config/pose.yaml` (git 제외, 예시 `pose.example.yaml`) | 설치 자세의 정합 계수만: `camera.rgb_to_depth` (sx, sy, z_ref_m, dx·dy의 a-d) | ✅ 2026-10-14 |
| `config/a4_layout.yaml` | base 점과 A4 꼭짓점 1-4의 base 좌표, 재점검 꼭짓점 | ✅ |
| `config/frames.yaml` | 좌표계·토픽·노드 이름 | ✅ |
| `config/thresholds.yaml` | 검사·판정 기준값 (임시) | ✅ |
| `config/detect.yaml` | YOLO 모델 이름, conf, 클래스, 추적기 | ✅ |
| `config/cam2base.example.yaml` | 보정 결과 형식 예시 (숫자는 합성 값, 실행에 쓰지 않음) | ✅ |
| `config/cam2base.yaml` | 현장 보정 결과 (R, t, 검사 결과, 카메라 설정 사본) | ❌ 현장마다 생성 |

## 팀 코드와 대조한 결과 (2026-10-01, `work/webcam-path-editor` 20873db · `urdf` b2b4038 기준)
| 항목 | 이 문서 | 팀 코드 | 조치 |
|---|---|---|---|
| 로봇 기준 좌표계 | `base_ref` → `base_link` | urdf 브랜치 `base_link` ✅ / 경로 편집기 세션 메타데이터 `robot_base` ⚠️ | 통합 회의에서 하나로. `frames.yaml`로 흡수 가능 |
| 화면 크기 | `rgb_size`로 받음 | 경로 편집기 기본 1280×720 (`WebcamConfig.width/height`) | 위 "화면 픽셀 규칙"(가운데 띠)으로 처리 |
| 카메라 여는 곳 | RGB(UVC)와 depth(OpenNI)를 나눠서 연다 | 경로 편집기 `webcam.py`가 `cv2.VideoCapture`로 RGB를 엶 | `detect_objects.py`도 RGB를 열므로 같은 카메라로 동시에 실행하면 충돌 🔲 협의 |

✅ 경로 편집기는 순수 파이썬(ROS는 `--ros` 선택)으로 확인됨 (10-01). 그래서 핵심 인터페이스는 ROS 토픽이 아니라 import해서 쓰는 파이썬 함수이고, 토픽 발행은 선택이다.

## 바꾸기 쉬운 것과 어려운 것
| 항목 | 변경 비용 | 이유 |
|---|---|---|
| 토픽·좌표계 이름 | 낮음 | `frames.yaml` 한 곳 수정, 또는 launch에서 이름 바꾸기(remap) |
| 기준값 (검사, 물체 높이·띠) | 낮음 | `thresholds.yaml` |
| 메시지 타입 | 중간 | 보내는 쪽과 받는 쪽 코드 모두 수정 |
| 물체 좌표 정의 (윗면 중심) | 중간 | `objects.top_center` 한 곳이지만 받는 쪽 접근 방식이 따라 바뀜 |
| 단위·축 방향 규약 | 높음 | 모든 계산에 퍼져 있고, 틀려도 에러 없이 조용히 틀린 값이 나옴 |
