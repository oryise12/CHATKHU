#!/usr/bin/env bash
# Unattended GUI smoke test on the real X display: runs the two-window app with scripted keys
# (same Session.handle_key path as real key presses) and captures both windows with xwd.
# Usage: scripts/gui_smoke.sh <name> <capture_at_s> -- <app args...>
# Real keyboard input is NOT exercised by this script.
set -euo pipefail
name=$1; cap=$2; shift 3
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ws/scripts/env.sh"
out=$ws/logs/gui_smoke/$name
rm -rf "$out"; mkdir -p "$out"
cd $ws
timeout 90 python -m pinch_sim.app "$@" --snapshot-dir logs/gui_smoke/$name > "$out/app.out" 2>&1 &
pid=$!
sleep "$cap"
xwininfo -root -tree | grep -E '"(MuJoCo : 6dof scene|side_cam \()' | grep -v mutter | tee "$out/windows.txt"
while read -r wid rest; do
  tag=$(echo "$rest" | grep -q MuJoCo && echo main || echo side_cam)
  xwd -silent -id "$wid" -out "$out/$tag.xwd" && python scripts/xwd2png.py "$out/$tag.xwd" "$out/window_$tag.png" && rm "$out/$tag.xwd"
done < "$out/windows.txt"
wait $pid || true
tail -n 25 "$out/app.out"
