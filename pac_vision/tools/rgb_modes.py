"""Astra RGB(UVC) 해상도·종횡비 측정 (카메라 필요). README 측정 10절의 재측정 도구.

python tools/rgb_modes.py [장치번호]      # 기본 2. 화면에서 비켜 서고, 끝날 때까지 장면을 움직이지 않는다

1) 후보 해상도를 요청해 실제로 받은 크기·FOURCC를 출력
2) 같은 장면을 16:9 원본과 640x480으로 촬영하고, 640x480 영상을 세 가설과 정규화 상호상관(NCC)으로 비교
   A 늘림(16:9 전체를 4:3으로 찌그러뜨림) / B 잘림(16:9의 가운데 4:3) / C 가운데 띠(16:9 = 640x480의 가운데 640x360)
3) 16:9를 두 번 찍어 장면이 고정됐는지(기준 NCC) 확인. 기준이 0.95 미만이면 결과를 쓰지 않는다
4) 비교 영상을 data/measurements/rgb_modes_<날짜>_*.png로 저장 (원자료)
"""
from __future__ import annotations

import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

DEV = int(sys.argv[1]) if len(sys.argv) > 1 else 2
OUT = Path(__file__).resolve().parents[1] / "data" / "measurements"
CANDIDATES = [(1920, 1080), (1280, 720), (1280, 960), (1024, 768), (800, 600), (640, 480), (640, 360), (320, 240)]
BASE_MIN = 0.95   # 장면 고정 기준
NCC_LOW = 0.8     # 모든 가설이 이보다 낮으면 판단하지 않음


def grab(w, h, fourcc=None, warm=20, n=8):
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if not cap.isOpened():
        return None, None
    for _ in range(warm):
        cap.read()
    frames = [f.astype(np.float32) for ok, f in (cap.read() for _ in range(n)) if ok]
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    info = "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
    cap.release()
    time.sleep(0.3)
    return (np.mean(frames, axis=0).astype(np.uint8) if frames else None), info


def ncc(a, b):
    a = cv2.GaussianBlur(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(np.float32), (5, 5), 0)
    b = cv2.GaussianBlur(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).astype(np.float32), (5, 5), 0)
    a -= a.mean()
    b -= b.mean()
    return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum()))


def main():
    print(f"장치 /dev/video{DEV}\n요청 크기 -> 실제 받은 크기 (FOURCC)")
    got = {}
    for w, h in CANDIDATES:
        for fcc in (None, "MJPG"):
            img, info = grab(w, h, fcc, warm=5, n=1)
            shape = None if img is None else (img.shape[1], img.shape[0])
            print(f"  {w}x{h:<5} {fcc or '기본':>4} -> {shape} ({info})")
            if shape:
                got.setdefault(shape, fcc)
    wide = [s for s in got if abs(s[0] / s[1] - 16 / 9) < 0.02]
    if not wide or (640, 480) not in got:
        sys.exit("16:9 모드나 640x480을 받지 못해 비교할 수 없습니다")
    native = max(wide)
    print(f"\n비교에 쓸 16:9: {native}. 화면 밖으로 비켜 주세요.")
    for k in range(5, 0, -1):
        print(f"  촬영까지 {k}초", flush=True)
        time.sleep(1)
    w1, _ = grab(*native, got[native])
    vga, _ = grab(640, 480, got[(640, 480)])
    w2, _ = grab(*native, got[native])
    small = lambda im, size: cv2.resize(im, size, interpolation=cv2.INTER_AREA)
    base = ncc(small(w1, (640, 480)), small(w2, (640, 480)))
    a = ncc(vga, small(w1, (640, 480)))
    cw = int(round(native[1] * 4 / 3))
    x0 = (native[0] - cw) // 2
    b = ncc(vga, small(w1[:, x0:x0 + cw], (640, 480)))
    c = ncc(vga[60:420], small(w1, (640, 360)))
    print(f"\n기준 (16:9 두 장끼리): {base:.3f}\nA 늘림: {a:.3f}\nB 가운데 4:3 잘림: {b:.3f}\nC 가운데 띠: {c:.3f}")
    if base < BASE_MIN:
        print(f"⚠️ 기준 {base:.3f} < {BASE_MIN}: 촬영 중 장면이 움직였습니다. 결과를 쓰지 마세요.")
    elif max(a, b, c) < NCC_LOW:
        print(f"⚠️ 모든 가설이 {NCC_LOW} 미만: 판단하지 않습니다.")
    else:
        print("판정:", {a: "A 늘림", b: "B 잘림", c: "C 가운데 띠"}[max(a, b, c)])
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M")
    OUT.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT / f"rgb_modes_{stamp}_640x480.png"), vga)
    cv2.imwrite(str(OUT / f"rgb_modes_{stamp}_{native[0]}x{native[1]}.png"), w1)
    print(f"저장: {OUT}/rgb_modes_{stamp}_*.png")


if __name__ == "__main__":
    main()
