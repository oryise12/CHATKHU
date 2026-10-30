"""XM430 6-DOF arm control preparation: session calibration, real-state view, hold, HOME, READY."""
from pathlib import Path

WS = Path(__file__).resolve().parents[2]
# the model's reference repo: the repository this workspace lives in (<repo>/manipulator_control_ws next to
# mujoco_sim); a workspace kept outside it falls back to the old location
REF_REPO = WS.parent if (WS.parent / "mujoco_sim").is_dir() else Path.home() / "pinch_khu_ws" / "26_project_manipulator"
