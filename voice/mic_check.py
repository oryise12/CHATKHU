"""마이크 입력 크기와 스피커 → 마이크 에코를 잰다 (AI·API 키 없이). 소리는 저장하지 않고 숫자만 출력한다.

  python -m voice.mic_check            조용히 2초: 배경 잡음(dBFS), DC, 잘림(%)
  python -m voice.mic_check --echo     1 kHz 음을 1초 틀며 녹음: 스피커 소리가 마이크에 들어오는 크기 (삐 소리가 난다)

판단 (docs/PINCH_KHU_SETUP.md 8절 시험 G):
- 잘림이 0 %가 아니거나 DC가 크면 마이크 증폭이 너무 높다. 노트북 내장 마이크(ALC257 등):
  amixer -c <카드> sset 'Internal Mic Boost' 0   (카드 번호는 arecord -l)
- 에코가 사람 말 크기(약 -30 ~ -15 dBFS)에 가까우면 listen_while_moving을 켜지 않는다(쿠옹이가 자기 말을 듣는다).
  이어폰을 쓰거나, 정지는 X 키로 한다.
"""
from __future__ import annotations

import argparse

import numpy as np


def _db(x) -> float:
    return float(20 * np.log10(max(float(np.std(x - np.mean(x))), 1e-9)))


def main(argv=None) -> int:
    import sounddevice as sd
    ap = argparse.ArgumentParser(prog='python -m voice.mic_check')
    ap.add_argument('--echo', action='store_true', help='1 kHz 음을 틀어 스피커 → 마이크 에코를 잰다')
    ap.add_argument('--amp', type=float, default=0.3, help='시험 음 크기 (0~1, 쿠옹이 목소리 정도 = 0.3)')
    ap.add_argument('--mic', type=int, help='입력 장치 번호 (python -m voice --list-mics)')
    a = ap.parse_args(argv)
    fs = 48000
    print(f"입력: {sd.query_devices(a.mic, 'input')['name']}, 출력: {sd.query_devices(kind='output')['name']}")
    x = sd.rec(int(fs * 2.0), samplerate=fs, channels=1, dtype='float32', device=a.mic)
    sd.wait()
    x = x[int(fs * 0.3):, 0]
    clip = float((np.abs(x) > 0.99).mean() * 100)
    print(f"조용할 때: 배경 {_db(x):.1f} dBFS, DC {x.mean():+.3f}, 잘림 {clip:.2f} %"
          + ("  ← 마이크 증폭이 너무 높음 (위 설명의 amixer)" if clip > 0.1 or abs(x.mean()) > 0.05 else ""))
    if a.echo:
        t = np.arange(fs) / fs
        sig = np.concatenate([np.zeros(fs // 2), a.amp * np.sin(2 * np.pi * 1000 * t), np.zeros(fs // 2)])
        r = sd.playrec(sig.astype(np.float32)[:, None], samplerate=fs, channels=1, dtype='float32',
                       device=(a.mic, None))
        sd.wait()
        on, quiet = r[int(fs * 0.6):int(fs * 1.4), 0], r[int(fs * 0.05):int(fs * 0.45), 0]
        print(f"스피커 음(크기 {a.amp}) 중 마이크: {_db(on):.1f} dBFS (배경 {_db(quiet):.1f} dBFS, "
              f"잘림 {(np.abs(on) > 0.99).mean() * 100:.1f} %)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
