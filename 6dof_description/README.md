# 6dof_description

6자유도(6-DOF) 로봇팔의 모델 패키지입니다. ROS 2 RViz2 시각화와 MuJoCo 시뮬레이션을 지원합니다.

- 테스트 환경: Ubuntu 24.04, ROS 2 Jazzy, MuJoCo (Python)
- 관절: `Revolute_1` ~ `Revolute_6` (모두 revolute, ±π rad)

## 폴더 구조

```
6dof_description/
├── launch/display.launch.py   # RViz2 실행 launch 파일
├── rviz/display.rviz          # RViz2 설정
├── meshes/                    # STL 메쉬 (RViz2 / MuJoCo 공용)
├── urdf/
│   ├── 6dof.xacro             # 로봇 모델 원본
│   ├── materials.xacro
│   └── 6dof.urdf              # xacro 변환 결과 (참고용)
└── mujoco/
    ├── scene.xml              # MuJoCo 실행 파일 (바닥/조명 + 로봇)
    ├── 6dof_control.xml       # 모터 6개 포함 모델
    ├── 6dof.xml               # URDF → MJCF 변환 원본
    └── 6dof_mj.urdf           # MuJoCo 변환용 URDF
```

## 1. RViz2로 보기

### 설치 (최초 1회)

ROS 2 Jazzy가 설치되어 있어야 합니다.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b urdf https://github.com/jaemin0319/26_project_manipulator.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select 6dof_description
```

### 실행

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch 6dof_description display.launch.py
```

- `joint_state_publisher_gui` 창의 슬라이더로 각 관절을 움직일 수 있습니다.
- 슬라이더 없이 실행: `ros2 launch 6dof_description display.launch.py gui:=false`
- `rviz/`, `urdf/`, `meshes/` 파일을 수정한 뒤에는 `colcon build`를 다시 해야 반영됩니다.

## 2. MuJoCo로 보기

ROS 없이 Python만 있으면 됩니다.

### 설치 (최초 1회)

```bash
sudo apt install python3-venv
python3 -m venv ~/mjenv
source ~/mjenv/bin/activate
pip install mujoco
```

### 실행

```bash
source ~/mjenv/bin/activate
python3 -m mujoco.viewer --mjcf=$HOME/ros2_ws/src/26_project_manipulator/6dof_description/mujoco/scene.xml
```

- 오른쪽 **Control** 패널에서 모터 6개(`Revolute_1_mot` ~ `Revolute_6_mot`)를 조작할 수 있습니다.
- `GLFWError: Wayland: The platform does not provide the window position` 경고는 무시해도 됩니다.
- 창이 뜨지 않으면 X11로 실행해 보세요:
  `PYGLFW_LIBRARY_VARIANT=x11 python3 -m mujoco.viewer --mjcf=...`

## 참고

- Python venv(mjenv)가 활성화된 터미널에서 ROS 2를 실행하면 충돌할 수 있으니, RViz2는 별도 터미널에서 실행하세요 (`deactivate`).
- 메쉬 경로: URDF/xacro는 `package://6dof_description/meshes/`, MuJoCo는 `../meshes/` (상대경로)를 사용합니다.

## 그리퍼 변형 (브랜치 `work/integration-all`에서 추가)

원본 파일(`urdf/6dof.xacro`, `urdf/6dof.urdf`, `mujoco/*.xml`, 기존 `meshes/*.stl`)은 브랜치 `urdf`@b2b4038 그대로이고, 수정하지 않았습니다. 아래 파일만 추가했습니다.

| 파일 | 내용 |
|---|---|
| `meshes/gripper.stl` | 고정 그리퍼 메시(mm, 사용자 제공 `그리퍼URDF용.stl`의 이름만 바꿈) |
| `urdf/6dof_gripper.xacro`, `urdf/6dof_gripper.urdf` | **생성물.** 원본에서 `housing_1` 링크 블록만 바꿨다(`housing_1.stl` 제거, 그리퍼 메시·관성). `Revolute_6`은 revolute로 그대로다. 직접 고치지 말고 `mujoco_sim`에서 `python -m pinch_sim.build_scene`으로 다시 만든다. |

- 손가락 개폐 관절은 없습니다(고정 그리퍼).
- 장착 변환, 질량 가정, TCP는 [docs/FRAMES.md](../docs/FRAMES.md)에 있습니다.
- `display.launch.py`는 원본 `6dof.xacro`를 띄웁니다. 그리퍼 변형은 `xacro`와 `check_urdf`로 파싱만 확인했고, RViz 표시는 확인하지 않았습니다.

## License

[LICENSE](LICENSE) 파일을 참고하세요.
