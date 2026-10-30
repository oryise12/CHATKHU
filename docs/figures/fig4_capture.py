"""그림 4용 실제 화면 캡처 (시뮬레이션 모드: 실물 팔은 움직이지 않음).

실행 (저장소 폴더에서):
    mujoco_sim/.venv/bin/python docs/figures/fig4_capture.py            # 카메라 없이: 흰 바탕 + 보정 좌표 격자
    mujoco_sim/.venv/bin/python docs/figures/fig4_capture.py --camera   # Astra가 꽂혀 있으면: 실제 카메라 영상 위

하는 일: pinch_sim을 실물 프로필 연습 모드(--real-arm --rehearse)와 --stations로 띄우고, 쿠옹이 쪽 연결(voice/sim_link.py)로
"설거지 바구니로 이동할까요?" → 응 → 바구니 도착 → "국그릇으로 가자"까지 진행한 뒤, 브라우저(headless Chrome)로 화면을 찍는다.
대화 말풍선은 AI 없이 앱이 실제로 말하는 문장(sim_link.py의 알림)과 사용자 말을 그대로 넣는다.
결과: docs/figures/fig4_raw_open.png (대화 펼침), fig4_raw_folded.png (대화 접음: 추천 말)
카메라 없이 찍을 때 S·B·C 위치는 가장 최근 실행 기록에서 카메라로 읽은 값(fig1_layout.stations_from_log)을 쓴다.
"""
from __future__ import annotations

import argparse
import queue
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HERE))


def said(ed, until: str, timeout: float = 60.0) -> list[str]:
    out, t0 = [], time.time()
    while time.time() - t0 < timeout:
        try:
            x = ed.announcements.get(timeout=0.2)
        except queue.Empty:
            continue
        out.append(x)
        if until in x:
            return out
    raise SystemExit(f"timeout waiting for {until!r} (got {out})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", action="store_true", help="use the Astra camera (markers read live)")
    a = ap.parse_args()
    tmp = Path(tempfile.mkdtemp(prefix="fig4_"))
    args = ["--real-arm", "--rehearse", "--calib", str(REPO / "pac_vision/config/cam2base.yaml"), "--browser", "none",
            "--voice-gateway", "--swap-left-right"]
    if a.camera:
        args += ["--video", "auto", "--stations"]
    else:
        from fig1_layout import stations_from_log
        st, _ = stations_from_log()
        yml = tmp / "stations.yaml"
        cfg = (REPO / "pac_vision/config/stations.yaml").read_text(encoding="utf-8")
        for k, (_, marker, x, y) in st.items():
            cfg = re.sub(rf"(\s{k}: \{{marker: {marker}, [^}}]*?)\}}", rf"\1, base_m: [{x}, {y}]}}", cfg)
        yml.write_text(cfg, encoding="utf-8")
        args += ["--canvas", "--stations", str(yml)]
    log = open(tmp / "app.log", "w")
    app = subprocess.Popen([str(REPO / "mujoco_sim/.venv/bin/python"), "-u", "-m", "pinch_sim.app", *args],
                           cwd=REPO / "mujoco_sim", stdout=log, stderr=subprocess.STDOUT,
                           env={**__import__("os").environ, "PYTHONPATH": str(REPO / "mujoco_sim/src")})
    try:
        url = None
        for _ in range(240):
            text = (tmp / "app.log").read_text(errors="replace")
            m = re.search(r"open (http://127\.0\.0\.1:\d+/\?t=[\w-]+)", text)
            if m and "voice gateway:" in text:
                url = m.group(1)
                break
            time.sleep(0.5)
        if url is None:
            raise SystemExit("the simulator did not start: " + str(tmp / "app.log"))
        from voice.sim_link import SimEditor
        ed = SimEditor({"small": 2.0, "medium": 5.0, "large": 8.0})
        ask = said(ed, "이동할까요?", 90)[-1]
        ed.chat("kuong", ask)
        ed.chat("user", "응, 이동해.")
        print("->", ed.handle("validate_and_move", {}))
        arrive = said(ed, "어디로 갈까요?", 90)[-1]
        ed.chat("kuong", arrive)
        ed.chat("user", "국그릇으로 가자.")
        r = ed.handle("go_to_station", {"station": "B"})
        print("->", r)
        ed.chat("kuong", r.get("say", ""))
        time.sleep(2.0)
        for name, extra in (("fig4_raw_open.png", ""), ("fig4_raw_folded.png", "&open=sidebar")):
            subprocess.run(["google-chrome", "--headless=new", "--disable-gpu", "--no-first-run",
                            f"--user-data-dir={tmp}/chrome_{name}", "--hide-scrollbars", "--window-size=1920,1200",
                            "--timeout=6000", f"--screenshot={HERE / name}", url + extra], capture_output=True,
                           timeout=60)
            print(HERE / name)
        ed.close()
    finally:
        app.terminate()
        try:
            app.wait(10)
        except subprocess.TimeoutExpired:
            app.kill()


if __name__ == "__main__":
    main()
