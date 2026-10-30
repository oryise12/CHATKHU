#!/usr/bin/env bash
# 26_project_manipulator 한 번에 설치 (Ubuntu 24.04, Python 3.12). 저장소를 어디에 받아도 된다.
#
#   ./setup.sh             가상환경 3개(lock 파일의 정확한 버전) + Astra 드라이버(고정 커밋) + 키 파일 자리 + 전체 시험
#   ./setup.sh --system    먼저 시스템 준비까지 (sudo 필요, 컴퓨터마다 한 번):
#                          apt 패키지, Astra udev 규칙, dialout·plugdev 그룹, (없으면) Google Chrome
#   ./setup.sh --no-test   시험 생략
#
# 만드는 것 (모두 git 제외):
#   mujoco_sim/.venv              시뮬레이터 + 손(mediapipe) + Astra(openni, pac_vision)
#   .venv-voice                   음성(쿠옹이)
#   manipulator_control_ws/.venv  실물 팔 제어
#   vendor/ros2_astra_camera      Orbbec OpenNI2 redist (빌드하지 않고 .so만 쓴다)
# ChatKHU 키는 저장소의 voice/chatkhu_key.env (개인 파일 ~/.config/pinch_khu/env가 있으면 그쪽이 먼저)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3.12}"
ASTRA_REPO=https://github.com/orbbec/ros2_astra_camera.git
ASTRA_COMMIT=f7e71d9ce806e788cb48d8580aac2c778fba4214      # 2026-10-23에 받아 쓴 그 커밋
APT_PKGS=(git python3.12-venv python3-pip libportaudio2 fonts-noto-cjk x11-utils x11-xserver-utils
          v4l-utils mesa-utils libopenni2-dev gnome-terminal)
SYSTEM=0
TESTS=1
for a in "$@"; do
  case "$a" in
    --system) SYSTEM=1 ;;
    --no-test) TESTS=0 ;;
    -h|--help) sed -n 2,15p "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $a (./setup.sh --help)" >&2; exit 2 ;;
  esac
done

step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
# ROS 2를 source한 셸이면 PYTHONPATH에 ROS 패키지가 섞인다: 설치·시험은 늘 PYTHONPATH 없이
pyrun() { env -u PYTHONPATH "$@"; }

# ------------------------------------------------------------------------------------------------ system (sudo)
if [ "$SYSTEM" = 1 ]; then
  step "시스템 패키지 (sudo)"
  sudo apt-get update
  sudo apt-get install -y "${APT_PKGS[@]}"
  step "Astra USB 권한 (udev) · 직렬 포트(U2D2)·카메라 그룹"
  echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-orbbec.rules >/dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger
  sudo usermod -aG dialout,plugdev "$USER"
  echo "  그룹을 새로 넣었다면 로그아웃 후 다시 로그인해야 적용돼요. 카메라는 뽑았다 다시 꽂아요."
  if ! command -v google-chrome >/dev/null && ! command -v chromium >/dev/null && ! command -v chromium-browser >/dev/null; then
    step "Google Chrome (웹 모니터를 앱 창으로 띄울 때 씀, 없으면 Firefox로 열림)"
    tmp="$(mktemp -d)"
    wget -q -O "$tmp/chrome.deb" https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
    sudo apt-get install -y "$tmp/chrome.deb"
    rm -rf "$tmp"
  fi
fi

# ------------------------------------------------------------------------------------------------ checks
step "확인: Python 3.12, venv"
command -v "$PY" >/dev/null || { echo "$PY 없음: sudo apt install python3.12 python3.12-venv (또는 ./setup.sh --system)" >&2; exit 1; }
"$PY" -c 'import sys; assert sys.version_info[:2] == (3, 12), sys.version' \
  || { echo "Python 3.12가 필요해요 (시험한 버전 3.12.3)" >&2; exit 1; }
"$PY" -c 'import ensurepip' 2>/dev/null \
  || { echo "ensurepip 없음: sudo apt install python3.12-venv (또는 ./setup.sh --system)" >&2; exit 1; }
echo "  $("$PY" -V)"
"$PY" -c 'import ctypes.util, sys; sys.exit(0 if ctypes.util.find_library("portaudio") else 1)' \
  || echo "  경고: PortAudio 없음 (마이크·스피커): sudo apt install libportaudio2 (또는 ./setup.sh --system)"

mkvenv() {   # $1 = venv folder
  if [ ! -x "$1/bin/python" ]; then
    "$PY" -m venv "$1"
  fi
}

# ------------------------------------------------------------------------------------------------ simulator
step "시뮬레이터 가상환경: mujoco_sim/.venv"
mkvenv "$ROOT/mujoco_sim/.venv"
(
  cd "$ROOT/mujoco_sim"
  P=.venv/bin/python
  pyrun $P -m pip install -q --disable-pip-version-check -r requirements.lock        # mujoco 3.15.0, numpy 1.26.4, opencv 4.11
  pyrun $P -m pip install -q --disable-pip-version-check --no-deps mediapipe==0.10.21 # 그냥 깔면 numpy 2가 들어와 깨진다
  pyrun $P -m pip install -q --disable-pip-version-check -r requirements-hand.txt
  pyrun $P -m pip install -q --disable-pip-version-check -r requirements-astra.txt   # -e ../pac_vision + openni 2.3.0
  echo "  (위에 'mediapipe requires jax / jaxlib / opencv-contrib-python' 줄이 나오면 정상이에요: requirements-hand.txt)"
)

step "음성 가상환경: .venv-voice"
mkvenv "$ROOT/.venv-voice"
pyrun "$ROOT/.venv-voice/bin/python" -m pip install -q --disable-pip-version-check -r "$ROOT/voice/requirements.lock"

step "실물 제어 가상환경: manipulator_control_ws/.venv"
mkvenv "$ROOT/manipulator_control_ws/.venv"
pyrun "$ROOT/manipulator_control_ws/.venv/bin/python" -m pip install -q --disable-pip-version-check \
  -r "$ROOT/manipulator_control_ws/requirements.lock"

# ------------------------------------------------------------------------------------------------ Astra driver
step "Astra 드라이버: vendor/ros2_astra_camera @ ${ASTRA_COMMIT:0:7}"
A="$ROOT/vendor/ros2_astra_camera"
if [ "$(git -C "$A" rev-parse HEAD 2>/dev/null || true)" != "$ASTRA_COMMIT" ]; then
  rm -rf "$A"
  mkdir -p "$A"
  git -C "$A" init -q
  git -C "$A" remote add origin "$ASTRA_REPO"
  git -C "$A" fetch -q --depth 1 origin "$ASTRA_COMMIT"
  git -C "$A" checkout -q FETCH_HEAD
fi
so="$A/astra_camera/openni2_redist/x64/libOpenNI2.so"
[ -f "$so" ] && echo "  $so" || { echo "libOpenNI2.so가 없어요: $so" >&2; exit 1; }

# ------------------------------------------------------------------------------------------------ key file
step "ChatKHU 키 (값은 출력하지 않아요)"
envf="${PINCH_KHU_ENV:-$HOME/.config/pinch_khu/env}"
repo_key="$ROOT/voice/chatkhu_key.env"
chmod 600 "$repo_key" 2>/dev/null || true          # git은 권한 600을 기억하지 않는다
if [ -f "$envf" ] && grep '^BAZE_API_KEY=.\+' "$envf" >/dev/null; then
  chmod 600 "$envf"
  echo "  개인 키 파일을 써요: $envf (저장소 키보다 먼저)"
elif [ -f "$repo_key" ] && grep '^BAZE_API_KEY=.\+' "$repo_key" >/dev/null; then
  echo "  저장소의 키를 써요: voice/chatkhu_key.env"
else
  echo "  키 없음: voice/chatkhu_key.env 또는 $envf 에 BAZE_API_KEY=... 한 줄 (없으면 음성 대신 키보드·마우스·버튼으로 시연)"
fi

# ------------------------------------------------------------------------------------------------ tests
if [ "$TESTS" = 1 ]; then
  step "시험: 실물 제어 (가짜 서보, 모델 사본 해시·원본 커밋 비교)"
  (cd "$ROOT/manipulator_control_ws" && source scripts/env.sh && python -m pytest -q | tail -2)
  step "시험: 시뮬레이터 (실물 연결 e2e 포함, 약 4분)"
  (cd "$ROOT/mujoco_sim" && source scripts/env.sh && python -m pytest -q | tail -2)
  step "시험: pac_vision, 경로 편집"
  (cd "$ROOT/mujoco_sim" && source scripts/env.sh && cd ../pac_vision && python -m pytest -q | tail -2)
  (cd "$ROOT/mujoco_sim" && source scripts/env.sh && cd "../path 편집" && python -m pytest -q | tail -2)
  step "시험: 음성"
  (cd "$ROOT" && source voice/env.sh 2>/dev/null && python -m pytest -q voice/tests | tail -2)
fi

step "끝. 실행: ./run_demo.sh --help   (처음이면 README.md의 '처음 한 번' 순서대로)"
