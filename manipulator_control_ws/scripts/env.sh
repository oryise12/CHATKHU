# Source this file from anywhere: `source <repo>/manipulator_control_ws/scripts/env.sh`
# - drops ROS's PYTHONPATH (a sourced ROS 2 shell would mix its packages into the venv)
# - puts src/ on PYTHONPATH and activates the workspace .venv
# - keeps pip/XDG caches and temp files inside the workspace (.cache, .tmp)
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH
export PIP_CACHE_DIR="$_ws/.cache/pip" XDG_CACHE_HOME="$_ws/.cache/xdg" TMPDIR="$_ws/.tmp"
export PYTHONPATH="$_ws/src"
mkdir -p "$TMPDIR" "$PIP_CACHE_DIR" "$XDG_CACHE_HOME"
source "$_ws/.venv/bin/activate"
