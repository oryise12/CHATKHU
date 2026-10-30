#!/usr/bin/env bash
# 시연 실행. 터미널 창을 순서대로 띄운다 (gnome-terminal). 각 창의 키 안내는 README.md "실행".
#
#   ./run_demo.sh real        실물 이동: 실물 제어(pinch) → 시뮬(--real-arm, 카메라·손·깊이) → 음성
#   ./run_demo.sh check       위와 같지만 실물은 궤적을 검사만 하고 움직이지 않는다 (pinch --check-only)
#   ./run_demo.sh rehearse    실물 없이 실물 프로필로 카메라 영상 위에서 연습 (시뮬만 움직임) → 음성
#   ./run_demo.sh sim         카메라·실물 없이 시뮬레이터 화면 + 마우스 → 음성
#   ./run_demo.sh calibrate   실물 팔 보정 (calibrate, 3D 창의 기준 자세에 관절별로 맞춤)
#   ./run_demo.sh torque-off  실물 토크 끄기 (prepare --no-viewer: y → READY → 팔을 받치고 x → y → q)
#
# 옵션: --no-voice (음성 창 없이), --speed-scale S (real/check: 실물 속도 배율 ≤ 1)
#
# 실물 쪽 규칙: 팔은 터미널의 y(HOME 확인, 무장)로만 움직인다. 끌 때는 실물 제어 창에서 q (창을 닫지 않는다).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARM_WS="${MANIPULATOR_CONTROL_WS:-$ROOT/manipulator_control_ws}"
CALIB="../pac_vision/config/cam2base.yaml"
# 음성 "오른쪽"·"왼쪽"만 맞바꾼다 (pinch_sim --swap-left-right). 2026-10-28 사용자 결정: 노트북 화면을 보고 말하면
# 기본 방향에서 좌우만 반대였다. 위·아래·멀리·가까이는 그대로.
VOICE_DIR="--swap-left-right"
MODE="${1:-}"
[ $# -gt 0 ] && shift
VOICE=1
SPEED=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-voice) VOICE=0 ;;
    --speed-scale) SPEED=(--speed-scale "$2"); shift ;;
    *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
  esac
  shift
done

usage() { sed -n 2,14p "$0"; }
command -v gnome-terminal >/dev/null || { echo "gnome-terminal이 필요해요 (없으면 README의 명령을 터미널마다 직접 실행)" >&2; exit 1; }
for v in "$ROOT/mujoco_sim/.venv" "$ARM_WS/.venv"; do
  [ -x "$v/bin/python" ] || { echo "가상환경 없음: $v  → ./setup.sh 먼저" >&2; exit 1; }
done

LOGDIR="$ROOT/mujoco_sim/logs"
mkdir -p "$LOGDIR"
TS="$(date +%Y%m%d_%H%M%S)"
STAMP="$(mktemp)"
trap 'rm -f "$STAMP"' EXIT

# 창 하나 띄우기: $1 제목, $2 작업 폴더, $3 명령 (끝나면 Enter로 닫는 창)
term() {
  gnome-terminal --title "$1" --geometry=120x40 -- bash -c "cd \"$2\" && $3; echo; echo \"[끝났어요. Enter를 누르면 이 창이 닫혀요]\"; read" >/dev/null 2>&1
}

arm_cmd() { echo "source scripts/env.sh && python -m manipulator_control $*"; }

# 새 실물 제어 세션이 연결 대기(LINK_WAIT)가 될 때까지 (HOME 미리보기에서 사용자가 y를 눌러야 한다)
wait_link_wait() {
  local f="" d
  echo "실물 제어 창에서 HOME 미리보기를 확인하고 y를 눌러 주세요. 연결 대기가 되면 시뮬을 띄워요..."
  while :; do
    if [ -z "$f" ]; then
      d="$(find "$ARM_WS/logs/sessions" -maxdepth 1 -name '*_pinch' -newer "$STAMP" 2>/dev/null | sort | tail -1)"
      [ -n "$d" ] && [ -f "$d/events.jsonl" ] && f="$d/events.jsonl"
    fi
    if [ -n "$f" ]; then
      grep -q '"to": "LINK_WAIT"' "$f" && { echo "연결 대기: $(basename "$(dirname "$f")")"; return 0; }
      if grep -q '"to": "\(EXITED\|FAULT\)"' "$f"; then
        echo "실물 제어가 연결 대기 전에 끝났어요 ($f). 창의 안내를 확인해 주세요." >&2; return 1
      fi
    fi
    sleep 0.5
  done
}

# 시뮬이 음성 게이트웨이를 연 뒤 음성 창
start_voice() {
  [ "$VOICE" = 1 ] || return 0
  local log="$1" i
  for i in $(seq 1 240); do
    grep -q "^voice gateway:" "$log" 2>/dev/null && break
    sleep 0.5
  done
  grep -q "^voice gateway:" "$log" 2>/dev/null || { echo "시뮬의 음성 게이트웨이가 열리지 않았어요: $log" >&2; return 1; }
  term "음성 (쿠옹이)" "$ROOT" "source voice/env.sh && python -m voice --sim"
  echo "음성 창을 띄웠어요. \"쿠옹아\"라고 부른 뒤 말하세요."
}

sim() {   # $1 제목, $2 시뮬 인수
  local log="$LOGDIR/run_demo_${TS}.log"
  # -u: 파이프(tee)여도 줄마다 바로 기록 (음성 창은 "voice gateway:" 줄을 보고 띄운다)
  term "$1" "$ROOT/mujoco_sim" "source scripts/env.sh && python -u -m pinch_sim.app $2 2>&1 | tee \"$log\""
  echo "시뮬 창을 띄웠어요 (기록 $log). 웹 화면 창을 닫으면 시뮬이 끝나요."
  start_voice "$log"
}

case "$MODE" in
  real|check)
    only=""; [ "$MODE" = check ] && only=" --check-only"
    term "실물 제어 (pinch${only})" "$ARM_WS" "$(arm_cmd pinch$only "${SPEED[@]}")"
    wait_link_wait
    sim "시뮬 (실물 연결)" "--real-arm --calib $CALIB --video auto --hand --depth --voice-gateway $VOICE_DIR --stations"
    echo
    echo "시뮬이 연결되면 실물 제어 창에 무장 확인이 나와요: 팔 주변을 확인하고 y."
    [ "$MODE" = real ] && echo "무장 뒤에는 승인(검증 후 이동)마다 실물이 움직여요. 12 V 스위치에 손을 두세요." \
                       || echo "검사 전용: 승인해도 실물은 움직이지 않아요 (시뮬은 '중단됨'으로 끝나요)."
    ;;
  rehearse)
    sim "시뮬 (실물 프로필 연습, 실물 없음)" "--real-arm --rehearse --calib $CALIB --video auto --hand --depth --voice-gateway $VOICE_DIR --stations"
    ;;
  sim)
    sim "시뮬 (카메라 없음)" "--target-r 0.275 --target-deg 10 --voice-gateway"
    ;;
  calibrate)
    term "실물 보정 (calibrate)" "$ARM_WS" "$(arm_cmd calibrate)"
    echo "보정 창을 띄웠어요. 3D 창의 기준 자세에 관절을 하나씩 맞추고 창의 안내대로 Enter. 3D 창은 닫지 마세요."
    ;;
  torque-off)
    term "실물 토크 OFF (prepare --no-viewer)" "$ARM_WS" "$(arm_cmd prepare --no-viewer)"
    echo "y → READY → 팔을 손으로 받치고 x → y (토크 OFF) → q. 보정은 그대로 남아요."
    ;;
  -h|--help|"") usage ;;
  *) echo "알 수 없는 모드: $MODE" >&2; usage; exit 2 ;;
esac
