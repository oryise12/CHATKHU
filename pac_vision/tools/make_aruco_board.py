"""ArUco 보정판 인쇄물(PDF)을 배치 파일(config/a4_layout_aruco.yaml)의 값으로 만든다.

python tools/make_aruco_board.py board.pdf                   A4 가로 한 장 (벡터, matplotlib 필요)
python tools/make_aruco_board.py board.pdf --png board.png   확인용 그림도 (300 dpi)

카메라 불필요. 마커 위치·크기·번호는 배치 파일의 corners·aruco·sheet에서 계산하므로, 인쇄물과 캘리브레이션이 같은 숫자를 쓴다.
실제 크기(100 %)로 인쇄하고, 가운데 50 mm 눈금과 마커 한 변을 자로 확인한다.
마커끼리 붙이면 검은 테두리가 합쳐져 하나도 검출되지 않는다: 마커 사이·종이 가장자리에 흰 여백이 없으면 만들지 않는다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

LAYOUT = Path(__file__).resolve().parents[1] / "config" / "a4_layout_aruco.yaml"
MIN_EDGE_MM = 3.0        # 종이 가장자리까지 최소 흰 여백 (프린터가 못 찍는 테두리)
MIN_GAP_MM = 5.0         # 마커 사이 최소 흰 여백. 2026-10-26 실제 카메라 화면 합성: 0 mm(붙임) 0개 검출, 10 mm 4개 모두
FONTS = ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf",)   # 한글 라벨 (없으면 영어)


def board_geometry(layout: dict) -> dict:
    """인쇄물 좌표(mm, 원점 = 종이 꼭짓점 1, u = 꼭짓점 1 → 2, v = 꼭짓점 1 → 3)의 마커 왼쪽 위 자리.

    반환: {"page_mm": (W, H), "side_mm": 한 변, "markers": {id: (u0, v0)}}. 여백이 모자라면 ValueError."""
    a, sh = layout["aruco"], layout["sheet"]
    side = float(a["size_m"]) * 1000
    W, H = (float(x) * 1000 for x in sh["size_m"])
    c1 = np.asarray(sh["corner1_m"], float)[:2]
    ud, vd = np.asarray(sh["u_dir"], float), np.asarray(sh["v_dir"], float)
    markers = {}
    for mid, p in zip(a["ids"], layout["corners"]):
        d = (np.asarray(p, float)[:2] - c1) * 1000
        markers[int(mid)] = (float(d @ ud) - side / 2, float(d @ vd) - side / 2)
    for mid, (u0, v0) in markers.items():
        if min(u0, v0, W - u0 - side, H - v0 - side) < MIN_EDGE_MM:
            raise ValueError(f"마커 {mid}이(가) 종이 가장자리에 너무 가깝습니다 (최소 {MIN_EDGE_MM} mm)")
    ids = sorted(markers)
    for i in ids:
        for j in ids:
            if i < j:
                (ui, vi), (uj, vj) = markers[i], markers[j]
                gap = max(abs(ui - uj), abs(vi - vj)) - side
                if gap < MIN_GAP_MM:
                    raise ValueError(f"마커 {i}과 {j} 사이 흰 여백이 {gap:.1f} mm입니다 (최소 {MIN_GAP_MM} mm, "
                                     "붙으면 검은 테두리가 합쳐져 검출되지 않음)")
    return {"page_mm": (W, H), "side_mm": side, "markers": markers}


def _marker_cells(dictionary: str, mid: int) -> np.ndarray:
    import cv2
    n = int(dictionary[5]) + 2                                            # DICT_4X4_50 -> 6 칸 (검은 테두리 포함)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return cv2.aruco.generateImageMarker(d, int(mid), n) > 127          # True = 흰 칸


def render(layout: dict, px_per_mm: float = 4.0) -> np.ndarray:
    """인쇄물을 회색조 그림으로 (마커만, 라벨 없음). 시험·미리보기용."""
    import cv2
    g = board_geometry(layout)
    W, H = g["page_mm"]
    page = np.full((int(round(H * px_per_mm)), int(round(W * px_per_mm))), 255, np.uint8)
    s = int(round(g["side_mm"] * px_per_mm))
    dic = layout["aruco"].get("dictionary", "DICT_4X4_50")
    for mid, (u0, v0) in g["markers"].items():
        x, y = int(round(u0 * px_per_mm)), int(round(v0 * px_per_mm))
        cells = _marker_cells(dic, mid).astype(np.uint8) * 255
        page[y:y + s, x:x + s] = cv2.resize(cells, (s, s), interpolation=cv2.INTER_NEAREST)
    return page


def write_pdf(layout: dict, out: Path) -> None:
    """A4 벡터 PDF: 마커, 로봇 쪽 화살표, ID 라벨, 50 mm 눈금."""
    import matplotlib
    matplotlib.use("pdf")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.patches import FancyArrow, Rectangle

    g = board_geometry(layout)
    W, H = g["page_mm"]
    side, dic = g["side_mm"], layout["aruco"].get("dictionary", "DICT_4X4_50")
    font = next((p for p in FONTS if Path(p).exists()), None)
    ko = FontProperties(fname=font) if font else None
    t = (lambda k, e: k) if font else (lambda k, e: e)
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis("off")
    for mid, (u0, v0) in g["markers"].items():
        cells = _marker_cells(dic, mid)
        c = side / cells.shape[0]
        ax.add_patch(Rectangle((u0, v0), side, side, facecolor="black", edgecolor="none", antialiased=False))
        for r, k in zip(*np.nonzero(cells)):
            ax.add_patch(Rectangle((u0 + k * c, v0 + r * c), c, c, facecolor="white", edgecolor="none",
                                   antialiased=False))
        right = u0 + side / 2 > W / 2
        ax.text(W / 2 + (20 if right else -20), v0 + side / 2, f"ID {mid}", ha="center", va="center", fontsize=10)
    cx = W / 2
    ax.text(cx, 16, t("로봇 쪽 (변 1-2)", "robot side (edge 1-2)"), ha="center", va="center", fontproperties=ko,
            fontsize=11)
    ax.add_patch(FancyArrow(cx, 30, 0, -7, width=0.6, head_width=4, head_length=3, color="black"))
    ax.text(cx, H / 2 - 17, f"ArUco {dic[5]}x{dic[5]} ({dic})\n" + t(f"마커 한 변 {side:.0f} mm", f"marker side {side:.0f} mm"),
            ha="center", va="center", fontproperties=ko, fontsize=8, linespacing=1.6)
    y = H / 2 + 7
    ax.plot([cx - 25, cx + 25], [y, y], color="black", lw=0.8)
    for i in range(6):
        ax.plot([cx - 25 + 10 * i] * 2, [y, y - (3 if i % 5 == 0 else 1.5)], color="black", lw=0.6)
    ax.text(cx, y + 5, "50 mm", ha="center", va="center", fontsize=8)
    ax.text(cx, y + 16, t("실제 크기(100%)로 인쇄", "print at 100 % (actual size)"), ha="center", va="center",
            fontproperties=ko, fontsize=8, color="0.3")
    fig.savefig(out)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="ArUco 보정판 인쇄물 만들기")
    ap.add_argument("out", help="PDF 경로")
    ap.add_argument("--layout", default=str(LAYOUT), help="배치 파일 (기본: config/a4_layout_aruco.yaml)")
    ap.add_argument("--png", help="확인용 그림 경로 (300 dpi, 라벨 없음)")
    a = ap.parse_args()
    layout = yaml.safe_load(Path(a.layout).read_text(encoding="utf-8"))
    try:
        g = board_geometry(layout)
    except ValueError as e:
        sys.exit(f"[보정판] {e}")
    write_pdf(layout, Path(a.out))
    print(f"[보정판] {a.out}: A4 {g['page_mm'][0]:.0f}x{g['page_mm'][1]:.0f} mm, 마커 {g['side_mm']:.0f} mm, "
          f"ID {sorted(g['markers'])}. 실제 크기(100 %)로 인쇄하세요")
    if a.png:
        import cv2
        cv2.imwrite(a.png, render(layout, 300 / 25.4))
        print(f"[보정판] {a.png}")


if __name__ == "__main__":
    main()
