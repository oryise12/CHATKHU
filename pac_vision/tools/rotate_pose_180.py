"""카메라를 위아래로 뒤집어 달았을 때(camera.rotate_180: true), 돌리기 전에 잰 정합 계수를 돌린 좌표의 값으로 바꾼다.

python tools/rotate_pose_180.py                      config/pose.yaml을 바꾼다 (기존 파일은 pose.yaml.bak)
python tools/rotate_pose_180.py --from config/pose_presets/astra_unit1.yaml   프리셋을 바꿔 config/pose.yaml로

카메라 불필요. 수식은 pac_vision.cam2base.rotate_rgb_to_depth_180. 한 번만 적용한다(두 번 하면 원래대로 돌아간다):
바꾼 파일 머리에 '# rotate_180 변환됨' 줄을 남기고, 그 줄이 있으면 다시 바꾸지 않는다.
돌린 뒤에 check_camera.py로 정합 쌍을 새로 재면 그 계수는 이미 돌린 좌표라 이 도구가 필요 없다.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from datetime import datetime
from pathlib import Path

import yaml

from pac_vision.cam2base import rotate_rgb_to_depth_180
from pac_vision.config import load_config

MARK = "# rotate_180 변환됨"
CONFIG = Path(__file__).resolve().parents[1] / "config"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", help="바꿀 계수 파일 (기본 config/pose.yaml)")
    ap.add_argument("--out", default=str(CONFIG / "pose.yaml"))
    a = ap.parse_args(argv)
    src, out = Path(a.src or a.out), Path(a.out)
    text = src.read_text(encoding="utf-8")
    if MARK in text:
        sys.exit(f"{src}는 이미 rotate_180 좌표로 바꾼 파일입니다 (다시 바꾸면 원래대로 돌아갑니다)")
    m = (yaml.safe_load(text) or {})["camera"]["rgb_to_depth"]
    cam = load_config(CONFIG, ("camera",), overlays=False)["camera"]
    rot = rotate_rgb_to_depth_180(m, (cam["rgb"]["width"], cam["rgb"]["height"]),
                                  (cam["depth"]["width"], cam["depth"]["height"]))
    if out.exists():
        shutil.copy(out, out.with_suffix(".yaml.bak"))
    head = f"{MARK} ({datetime.now():%Y-%m-%d %H:%M}, 원본: {src.name}, tools/rotate_pose_180.py)\n"
    out.write_text(head + yaml.safe_dump({"camera": {"rgb_to_depth": rot}}, allow_unicode=True, sort_keys=False),
                   encoding="utf-8")
    print(f"[저장] {out}: dx a {m['dx']['a'] if isinstance(m['dx'], dict) else m['dx']} -> {rot['dx']['a']}, "
          f"dy a {m['dy']['a'] if isinstance(m['dy'], dict) else m['dy']} -> {rot['dy']['a']} (d 부호 반전)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
