# Source this file from anywhere: `source mujoco_sim/scripts/env.sh`
# - drops ROS's PYTHONPATH (a sourced ROS 2 shell would mix its packages into the venv)
# - puts mujoco_sim/src on PYTHONPATH and activates mujoco_sim/.venv (or $PINCH_VENV if set)
# - keeps pip/XDG caches and temp files inside mujoco_sim/.cache, .tmp (git-ignored)
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH
export PIP_CACHE_DIR="$_ws/.cache/pip" XDG_CACHE_HOME="$_ws/.cache/xdg" TMPDIR="$_ws/.tmp"
export PYTHONPATH="$_ws/src"
mkdir -p "$TMPDIR" "$PIP_CACHE_DIR" "$XDG_CACHE_HOME"
source "${PINCH_VENV:-$_ws/.venv}/bin/activate"
