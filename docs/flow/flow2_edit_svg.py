"""경로 수정 흐름도 (좌표를 직접 정해 그리는 SVG). 실행: python3 flow2_edit_svg.py → flow2_edit.svg"""
from pathlib import Path

W, H = 1560, 1590
FONT = "Noto Sans CJK KR"
STYLE = {
    "main":  ("#E8F1FB", "#2B5797"),
    "voice": ("#EEEBFB", "#5B4BB7"),
    "pinch": ("#E6F4EE", "#2E7D5B"),
    "dec":   ("#FFFFFF", "#888888"),
    "pill":  ("#F2F0EA", "#9A968C"),
    "note":  ("#F7F6F2", "#BDBAB0"),
}
out = []


def box(cx, cy, w, h, lines, kind="main", dashed=False, size=17, bold=True, r=14, width=1.6):
    fill, stroke = STYLE[kind]
    dash = ' stroke-dasharray="7 5"' if dashed else ""
    out.append(f'<rect x="{cx - w/2}" y="{cy - h/2}" width="{w}" height="{h}" rx="{r}" '
               f'fill="{fill}" stroke="{stroke}" stroke-width="{width}"{dash}/>')
    lh = size * 1.35
    y0 = cy - lh * (len(lines) - 1) / 2 + size * 0.35
    weight = "700" if bold else "400"
    for i, t in enumerate(lines):
        out.append(f'<text x="{cx}" y="{y0 + i*lh}" font-size="{size}" font-weight="{weight}" '
                   f'text-anchor="middle" font-family="{FONT}" fill="#1a1a1a">{t}</text>')


def pill(cx, cy, text, w=None):
    w = w or (len(text) * 15 + 34)
    box(cx, cy, w, 36, [text], "pill", size=15, r=18, width=1.2)


def hop_line(pts, hop_x, r=11, color="#777777"):
    """점선 꺾은선. 마지막 수평 구간이 x = hop_x 세로선을 지날 때 작은 반원으로 넘는다 (이어진 선이 아님을 표시)."""
    *head, (x1, y1), (x2, y2) = pts
    d = " ".join(f"{'M' if i == 0 else 'L'}{x},{y}" for i, (x, y) in enumerate(head + [(x1, y1)]))
    d += f" L{hop_x + r},{y2} A{r},{r} 0 0 0 {hop_x - r},{y2} L{x2},{y2}"
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.6" stroke-dasharray="7 5" marker-end="url(#ah)"/>')


def line(pts, arrow=True, dashed=False, color="#777777"):
    d = " ".join(f"{'M' if i == 0 else 'L'}{x},{y}" for i, (x, y) in enumerate(pts))
    dash = ' stroke-dasharray="7 5"' if dashed else ""
    mk = ' marker-end="url(#ah)"' if arrow else ""
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.6"{dash}{mk}/>')


def label(x, y, text, anchor="start"):
    out.append(f'<text x="{x}" y="{y}" font-size="14" font-family="{FONT}" fill="#777" text-anchor="{anchor}">{text}</text>')


# ---------------------------------------------------------------- 위: 고치는 방법
CX = 780
box(CX, 50, 470, 62, ["① 수정 모드에서 고치는 방법 선택 (쿠옹아, 경로 수정할래 / E)"], "dec", dashed=True, size=16)
VX, PX = 515, 1125                      # 말(왼쪽) / 핀치(오른쪽) 기둥
BW = 390                                # 두 기둥 상자 너비
line([(CX - 45, 81), (CX - 45, 98), (VX, 98), (VX, 100)], arrow=False)
line([(CX + 45, 81), (CX + 45, 98), (PX, 98), (PX, 100)], arrow=False)
pill(VX, 118, "검지로 가리키기 + 말로", 200)
pill(PX, 118, "핀치로", 70)
line([(VX, 136), (VX, 156)])
line([(PX, 136), (PX, 156)])

# 말로 고치기 (왼쪽)
box(VX, 195, BW, 78, ["사용자: 경로 위 점을 검지로 0.4초 가리킴", "→ 노란 동그라미로 표시"], "voice")
line([(VX, 234), (VX, 266)])
box(VX, 305, BW, 78, ["② 사용자: 점을 보고 말함", "예: 쿠옹아, 여기 위로 조금 올려줘"], "voice")
line([(VX, 344), (VX, 375)])
box(VX, 410, 300, 70, ["쿠옹이: 정보가 충분한가?", "(아래 ※ 참고)"], "dec", dashed=True)
# 부족 → 되묻기 (왼쪽으로 수평), 되물은 뒤 사용자가 다시 말함 → ②
line([(VX - 150, 410), (362, 410)], arrow=False)
pill(336, 410, "부족", 52)
line([(310, 410), (306, 410)])
box(187, 410, 235, 100, ["쿠옹이가 되물음", "예: 위로 넘어갈까요,", "옆으로 돌아갈까요?"], "voice", size=16)
line([(187, 360), (187, 305), (VX - BW / 2, 305)], dashed=True)
label(195, 296, "다시 ②로")
# 충분 → 말한 거리만큼 옮길 수 있나? → 고침 / 못 옮김
line([(VX, 445), (VX, 457)], arrow=False)
pill(VX, 475, "충분", 52)
line([(VX, 493), (VX, 507)])
box(VX, 538, 300, 62, ["말한 거리만큼 옮길 수 있나?"], "dec", dashed=True, size=16)
line([(VX - 150, 538), (362, 538)], arrow=False)
pill(336, 538, "불가능", 70)
line([(301, 538), (306, 538)])
box(187, 538, 235, 92, ["쿠옹이: 너무 많이 이동해서", "더 수정할 수 없어요.", "(경로는 그대로)"], "voice", size=16)
line([(187 - 117.5, 538), (50, 538), (50, 305), (187, 305)], arrow=False, dashed=True)   # → 되물음 선과 합쳐 ②로
line([(VX, 569), (VX, 579)], arrow=False)
pill(VX, 597, "가능", 52)
line([(VX, 615), (VX, 626)])
box(VX, 665, BW, 78, ["쿠옹이: 여기를 위로 2cm 올렸어요.", "(수정 전 경로를 회색으로 겹쳐 표시)"], "voice")

# 핀치로 고치기 (오른쪽)
box(PX, 195, BW, 78, ["사용자: 경로를 핀치로 집어 옮김", "(손을 놓는 순간 반영)"], "pinch")
line([(PX, 234), (PX, 626)])
box(PX, 665, BW, 78, ["쿠옹이: 경로를 수정했어요.", "(수정 전 경로를 회색으로 겹쳐 표시)"], "pinch")

# 두 갈래 → ③
line([(VX, 704), (VX, 735), (CX - 45, 735), (CX - 45, 763)])
line([(PX, 704), (PX, 735), (CX + 45, 735), (CX + 45, 763)])

# ---------------------------------------------------------------- ③ 다음 입력과 줄기
box(CX, 800, 295, 70, ["③ 다음 입력", "(입력이 올 때까지 기다림)"], "dec", dashed=True)
line([(CX, 835), (CX, 1342)], arrow=False)          # 가운데 줄기
ROWS = (935, 1060, 1170)
LX_P, LX_B = 650, 310                               # 왼쪽 말 / 결과
RX_P, RX_B = 910, 1250                              # 오른쪽 말 / 결과
LBUS, RBUS = 62, 1490                               # 결과 → ③ 점선 버스
branches = [
    (ROWS[0], "L", "조금만 더 · 아직 모자라", ["직전 수정을 한 단계 크게 다시", "조금 2 → 보통 5 → 많이 8cm", "(많이에서는: 이미 가장 많이 옮겼어요)"]),
    (ROWS[0], "R", "너무 갔어 · 좀 덜", ["직전 수정을 한 단계 작게 다시", "많이 8 → 보통 5 → 조금 2cm", "(조금에서는: 이미 가장 조금 옮겼어요)"]),
    (ROWS[1], "L", "아 별로다, 이전으로", ["쿠옹이: 직전 수정을 되돌렸어요."]),
    (ROWS[1], "R", "그대로 둬 · 하지 마", ["쿠옹이: 네, 그대로 둘게요.", "(아무것도 움직이지 않음)"]),
    (ROWS[2], "L", "처음부터 다시 · 초기화", ["쿠옹이: 처음 경로로 되돌렸어요.", "(지금까지 한 수정을 모두 취소)"]),
]
for y, side, said, result in branches:
    bh = 34 + 24 * len(result)
    if side == "L":
        line([(CX, y), (LX_P + 106, y)], arrow=False)
        pill(LX_P, y, said, 210)
        line([(LX_P - 105, y), (LX_B + 211, y)])
        box(LX_B, y, 420, bh, result, size=16)
        line([(LX_B - 210, y), (LBUS, y)], arrow=False, dashed=True)
    else:
        line([(CX, y), (RX_P - 96, y)], arrow=False)
        pill(RX_P, y, said, 190)
        line([(RX_P + 95, y), (RX_B - 211, y)])
        box(RX_B, y, 420, bh, result, size=16)
        line([(RX_B + 210, y), (RBUS, y)], arrow=False, dashed=True)
line([(LBUS, ROWS[2]), (LBUS, 805), (CX - 148, 805)], dashed=True)       # 왼쪽 버스 → ③
line([(RBUS, ROWS[1]), (RBUS, 800), (CX + 148, 800)], dashed=True)       # 오른쪽 버스 → ③
label(LBUS + 8, ROWS[0] - 62, "③으로")
label(RBUS - 8, ROWS[0] - 62, "③으로", "end")

# 다른 점 가리키기 → 가리키기 상자 / 다른 점 핀치 → 핀치 상자
ROW4 = 1265
line([(CX, ROW4), (437 + 71, ROW4)], arrow=False)
pill(437, ROW4, "다른 점 가리키기", 140)
line([(437 - 70, ROW4), (30, ROW4), (30, 195), (VX - BW / 2, 195)])
label(40, 178, "새 점으로 ②부터")
line([(CX, ROW4), (1140 - 56, ROW4)], arrow=False)
pill(1140, ROW4, "다른 점 핀치", 110)
line([(1140 + 55, ROW4), (1530, ROW4), (1530, 195), (PX + BW / 2, 195)])
label(1520, 178, "새 점을 핀치로", "end")

# 수정 완료 → ④ (가운데 아래로 곧게)
pill(CX, 1360, "이 경로로 할게 · 수정 완료", 230)
line([(CX, 1378), (CX, 1400)])
box(CX, 1435, 440, 66, ["④ 수정 끝 → 쿠옹이: 검증 후 이동할까요?", "(어 → 검증, 통과하면 이동 · 전체 흐름 ②로)"], "main", width=2.6)

# ※ 조건 (왼쪽 아래)
fill, stroke = STYLE["note"]
out.append(f'<rect x="55" y="1345" width="470" height="211" rx="14" fill="{fill}" stroke="{stroke}"/>')
for i, (t, b) in enumerate([("※ 정보가 충분한 조건", True), ("· 어디: 검지로 가리킨 점 (또는 시작·가운데·끝)", False),
                            ("· 방향: 위·아래·왼쪽·오른쪽·내 쪽·저쪽", False), ("· 거리: 조금 2 · 보통 5 · 많이 8cm", False),
                            ("(말하지 않으면 묻지 않고 조금)", False), ("· 호출어(쿠옹아)는 처음 한 번만", False)]):
    out.append(f'<text x="75" y="{1378 + i*31}" font-size="{17 if b else 16}" font-weight="{700 if b else 400}" '
               f'font-family="{FONT}" fill="#1a1a1a">{t}</text>')

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">'
       '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
       '<path d="M0,0 L10,5 L0,10 z" fill="#777777"/></marker></defs>'
       f'<rect width="{W}" height="{H}" fill="white"/>' + "".join(out) + "</svg>")
Path(__file__).with_name("flow2_edit.svg").write_text(svg, encoding="utf-8")
print("flow2_edit.svg")
