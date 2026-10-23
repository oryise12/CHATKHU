"""사전 점검: 받아서 바로 실행할 수 있는지 항목마다 ✅/⚠️와 해결 방법 한 줄로 출력한다.

python scripts/check_setup.py              # 전체 (카메라 열기 포함)
python scripts/check_setup.py --no-camera  # 카메라를 열지 않음 (다른 프로그램이 카메라를 쓰는 중일 때)

항상 종료 코드 0 (점검만 하고 고치지 않는다).
"""
from __future__ import annotations

import argparse
import importlib
import importlib.util
import platform
import subprocess
import sys
from datetime import datetime
from pathlib import Path

PV_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_DIR = PV_DIR / "config"
PACKAGES = (("numpy", "numpy"), ("cv2", "OpenCV"), ("yaml", "PyYAML"), ("ultralytics", "ultralytics"),
            ("openni", "openni"), ("PIL", "Pillow"), ("lap", "lap"))
FIX = {
    "numpy": "pip install -e pac_vision (venv 안에서)",
    "cv2": "pip install opencv-contrib-python==4.11.0.86 (README 설치 절)",
    "yaml": "pip install -e pac_vision (venv 안에서)",
    "ultralytics": "detect_objects.py에 필요: pip install ultralytics 후 opencv-contrib 재설치 (README)",
    "openni": "depth에 필요: pip install openni + Orbbec redist (README prototype 절)",
    "PIL": "없으면 화면 문구가 영어로 나옴: pip install Pillow",
    "lap": "없으면 추적 없이 검출 + 최근접 대체: pip install lap",
}


def item(ok, name, detail, fix=""):
    return (bool(ok), name, str(detail), fix)


def check_versions():
    out = [item(True, "Python", f"{platform.python_version()} ({sys.prefix})")]
    for mod, label in PACKAGES:
        try:
            m = importlib.import_module(mod)
            out.append(item(True, label, getattr(m, "__version__", "버전 정보 없음")))
        except Exception as e:                      # ImportError 외에 라이브러리 초기화 오류도 있음
            out.append(item(False, label, f"불러오지 못함 ({type(e).__name__})", FIX[mod]))
    has_rclpy = importlib.util.find_spec("rclpy") is not None
    out.append(item(True, "rclpy", "있음" if has_rclpy else "없음 (--ros를 쓸 때만 필요: source /opt/ros/<배포판>/setup.bash)"))
    return out


def _git(args, cwd):
    return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, timeout=10)


def check_git(repo_dir=PV_DIR):
    try:
        br = _git(["rev-parse", "--abbrev-ref", "HEAD"], repo_dir)
        if br.returncode != 0:
            return [item(False, "git", "저장소가 아님", "git clone으로 받은 폴더에서 실행")]
        head = _git(["log", "-1", "--format=%h %cd", "--date=format:%Y-%m-%d %H:%M"], repo_dir).stdout.strip()
        changed = _git(["status", "--porcelain", "--", "config"], repo_dir).stdout.splitlines()
    except (OSError, subprocess.SubprocessError) as e:
        return [item(False, "git", f"실행 실패 ({type(e).__name__})", "git 설치 확인")]
    out = [item(True, "git", f"브랜치 {br.stdout.strip()}, 커밋 {head}")]
    tracked = [c for c in changed if not c.startswith("??")]
    out.append(item(not tracked, "추적되는 config 로컬 수정",
                    "없음" if not tracked else ", ".join(c[3:] for c in tracked),
                    "커밋된 config는 고치지 말고 local.yaml·pose.yaml로 덮어쓰기 (git diff config로 확인)"))
    return out


def check_configs(cfg_dir=DEFAULT_CONFIG_DIR):
    from pac_vision.config import CONFIG_NAMES, ConfigError, has_pose, load_overlays
    d = Path(cfg_dir)
    out = []
    missing = [n for n in CONFIG_NAMES if not (d / f"{n}.yaml").exists()]
    out.append(item(not missing, "config 파일", "모두 있음" if not missing else f"없음: {missing}", "저장소를 다시 받기"))
    try:
        ov = load_overlays(d)
        out.append(item(True, "local.yaml / pose.yaml 키 검사",
                        "통과 (" + (", ".join(ov) if ov else "덮어쓰기 파일 없음") + ")"))
    except ConfigError as e:
        out.append(item(False, "local.yaml / pose.yaml 키 검사", e, "파일이 받는 키만 남기기 (*.example.yaml 참고)"))
    out.append(item(True, "local.yaml (이 컴퓨터 값)",
                    "있음" if (d / "local.yaml").exists() else "없음 (camera.yaml 기본값과 장치 자동 탐색 사용)",
                    "경로·장치가 다르면 local.example.yaml을 local.yaml로 복사해 고치기"))
    out.append(item(has_pose(d), "pose.yaml (이 설치 자세의 정합 계수)",
                    "있음" if has_pose(d) else "없음 → calibrate.py가 실행을 거부함",
                    "check_camera.py로 쌍을 잰 뒤 python tools/fit_rgb_to_depth.py <쌍 CSV> --write-pose"))
    return out


def check_paths(camera_cfg):
    from pac_vision.config import expand_path
    out = []
    redist = Path(expand_path(camera_cfg["openni"]["redist"]))
    so = redist / "libOpenNI2.so"
    out.append(item(so.exists(), "OpenNI redist", f"{redist}" + ("" if so.exists() else " (libOpenNI2.so 없음)"),
                    "config/local.yaml의 camera.openni.redist를 libOpenNI2.so가 있는 폴더로"))
    font = Path(expand_path(camera_cfg["ui"]["font"]))
    out.append(item(font.exists(), "한글 글꼴", font, "없으면 영어 문구로 대체됨. local.yaml의 camera.ui.font로 지정"))
    return out


def check_cam2base(cfg_dir, camera_cfg):
    from pac_vision.session import calibration_matches_current, load_cam2base
    p = Path(cfg_dir) / "cam2base.yaml"
    if not p.exists():
        return [item(False, "cam2base.yaml (보정 결과)", "없음", "python scripts/calibrate.py로 캘리브레이션")]
    conv = load_cam2base(p)
    created = str(conv.info.get("created", "?")).replace("T", " ")[:16]
    out = [item(True, "cam2base.yaml (보정 결과)",
                f"{created}, base 점 오차 {conv.info.get('holdout_m', float('nan')) * 100:.1f} cm, 세션 {conv.info.get('session_file', '?')}")]
    same = calibration_matches_current(conv, camera_cfg)
    out.append(item(same, "보정 때 정합 계수 = 지금 설정", "같음" if same else "다름",
                    "정합을 다시 쟀다면 calibrate.py --replay <세션> --use-current-config로 보정 갱신"))
    return out


def check_devices(camera_cfg):
    sys.path.insert(0, str(PV_DIR / "scripts"))
    import _ui
    out = []
    r = camera_cfg["rgb"]
    cands = _ui.rgb_device_candidates(str(r.get("name_match", "Astra")))
    out.append(item(bool(cands) or r.get("device") is not None, "RGB 장치 후보",
                    "; ".join(([f"설정 rgb.device {r['device']}"] if r.get("device") is not None else [])
                              + ["이름 일치: " + (", ".join(f"video{n} ({name})" for n, name in cands) if cands else "없음")]),
                    "카메라 연결, 장치 이름 확인 (cat /sys/class/video4linux/*/name), 또는 local.yaml에 rgb.device"))
    try:
        import cv2
        dev = int(r["device"]) if r.get("device") is not None else (cands[0][0] if cands else None)
        ok = False
        for n in ([dev] if r.get("device") is not None else [c[0] for c in cands]):
            cap = cv2.VideoCapture(n, cv2.CAP_V4L2)
            ok = cap.isOpened() and cap.read()[0]
            cap.release()
            if ok:
                dev = n
                break
        out.append(item(ok, "RGB 열기", f"/dev/video{dev} 프레임 읽힘" if ok else "실패",
                        "다른 프로그램이 쓰는 중인지 fuser -v /dev/video* 로 확인"))
    except Exception as e:
        out.append(item(False, "RGB 열기", f"{type(e).__name__}: {e}", "OpenCV 설치 확인"))
    try:
        from pac_vision import AstraDepth
        dev = AstraDepth.from_config(camera_cfg).open()
        d = dev.read()
        dev.close()
        out.append(item(True, "depth 열기", f"{d.shape[1]}x{d.shape[0]}, 유효 {100 * (d > 0).mean():.0f} %"))
    except Exception as e:
        out.append(item(False, "depth 열기", f"{type(e).__name__}: {e}",
                        "redist 경로, udev 규칙(README), 카메라 재연결, 다른 프로그램 사용 여부 확인"))
    return out


def run(cfg_dir=DEFAULT_CONFIG_DIR, camera=True):
    from pac_vision.config import ConfigError, load_config
    items = check_versions() + check_git() + check_configs(cfg_dir)
    try:
        cam_cfg = load_config(cfg_dir, ("camera",))["camera"]
    except (ConfigError, FileNotFoundError) as e:
        return items + [item(False, "설정 읽기", e, "위 config 항목부터 해결")]
    items += check_paths(cam_cfg) + check_cam2base(cfg_dir, cam_cfg)
    if camera:
        items += check_devices(cam_cfg)
    return items


def main():
    ap = argparse.ArgumentParser(description="pac_vision 사전 점검")
    ap.add_argument("--config-dir", default=str(DEFAULT_CONFIG_DIR))
    ap.add_argument("--no-camera", action="store_true", help="카메라를 열지 않음")
    args = ap.parse_args()
    print(f"[pac_vision 사전 점검] {datetime.now():%Y-%m-%d %H:%M}  config: {args.config_dir}")
    items = run(args.config_dir, camera=not args.no_camera)
    for ok, name, detail, fix in items:
        print(f"{'✅' if ok else '⚠️'} {name}: {detail}" + ("" if ok or not fix else f"\n    → {fix}"))
    bad = sum(1 for i in items if not i[0])
    print(f"[요약] ⚠️ {bad}개" if bad else "[요약] 모두 ✅")


if __name__ == "__main__":
    main()
