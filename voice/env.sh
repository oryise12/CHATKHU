# Source this file from anywhere: `source voice/env.sh`, then `python -m voice --sim`
# - drops ROS's PYTHONPATH (a sourced ROS 2 shell would mix its packages, numpy among them, into the venv)
# - activates .venv-voice at the repository root (or $VOICE_VENV) and puts the root on PYTHONPATH
# - BAZE_API_KEY: if it is not set yet, it is read from ~/.config/pinch_khu/env (or $PINCH_KHU_ENV), a personal
#   file with one line `BAZE_API_KEY=...`, mode 600; if that has no key, from voice/chatkhu_key.env in this
#   (private) repository (2026-10-28 owner decision). The key is never printed.
_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH
source "${VOICE_VENV:-$_root/.venv-voice}/bin/activate"
export PYTHONPATH="$_root"
_envf="${PINCH_KHU_ENV:-$HOME/.config/pinch_khu/env}"
if [ -z "${BAZE_API_KEY:-}" ] && [ -f "$_envf" ]; then
  if [ "$(stat -c %a "$_envf")" != "600" ]; then
    echo "warning: $_envf should be readable only by you: chmod 600 $_envf" >&2
  fi
  set -a; . "$_envf"; set +a
fi
if [ -z "${BAZE_API_KEY:-}" ] && [ -f "$_root/voice/chatkhu_key.env" ]; then
  set -a; . "$_root/voice/chatkhu_key.env"; set +a
fi
if [ -z "${BAZE_API_KEY:-}" ]; then
  echo "BAZE_API_KEY is not set: put it in $_envf or voice/chatkhu_key.env (see docs/PINCH_KHU_SETUP.md)" >&2
fi
unset _root _envf
