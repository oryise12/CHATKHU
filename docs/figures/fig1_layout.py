"""그림 1. 시연 배치도 (위에서 본 그림 + 옆에서 본 그림). 실행: mujoco_sim/.venv/bin/python docs/figures/fig1_layout.py

좌표는 보정 좌표계(base_ref, m): 원점 = 로봇 J1 축의 책상면, +x = 로봇의 오른쪽, +y = 로봇 앞(종이 쪽), +z = 위.
값의 출처
  카메라 위치·보는 방향 : pac_vision/config/cam2base.yaml (t, R의 셋째 열 = 광축), 화각 = 2·atan(320/fx)
  5번 표지              : pac_vision/config/markers.yaml
  S·B·C 지점            : 가장 최근 시뮬 실행 기록(mujoco_sim/logs/run_demo_*.log)의 "stations located" (카메라로 읽은 값)
  목표 높이·내려가기     : pac_vision/config/stations.yaml (height_m, dip_m)
  로봇 받침판            : 180 x 180 x 20 mm (README.md 준비물)
  A (HOME 그리퍼 끝)     : 실물 연결 시 화면의 A 표시 (-0.09, 0.10, 0.22) m
추정(코드에 없음): 식탁 크기, 사용자·노트북 자리, 로봇 팔 모양(개략)
"""
from __future__ import annotations

import math
import re

import numpy as np
import yaml

from _style import COL, GRAY, INK, REPO, mpl_setup, save_mpl

CFG = REPO / "pac_vision" / "config"
A = np.array([-0.09, 0.10, 0.22])


def stations_from_log():
    logs = sorted((REPO / "mujoco_sim" / "logs").glob("run_demo_*.log"))
    pat = re.compile(r"^\s+([SBC]) (\S+(?: \S+)?) \(marker (\d+)\): \(([+-][\d.]+), ([+-][\d.]+)\) m")
    for f in reversed(logs):
        found = {}
        for line in f.read_text(encoding="utf-8", errors="replace").splitlines():
            m = pat.match(line)
            if m:
                found[m.group(1)] = (m.group(2), int(m.group(3)), float(m.group(4)), float(m.group(5)))
        if len(found) == 3:
            return found, f.name
    raise SystemExit("no 'stations located' in mujoco_sim/logs/run_demo_*.log")


def main():
    plt = mpl_setup()
    from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle, Wedge

    cam = yaml.safe_load((CFG / "cam2base.yaml").read_text(encoding="utf-8"))
    t, R = np.array(cam["t"]), np.array(cam["R"])
    axis = R[:, 2]
    fx = float(cam["camera"]["depth"]["fx"])
    fov = math.degrees(2 * math.atan(320.0 / fx))
    min_m = float(cam["camera"]["depth"]["min_m"])
    m5 = yaml.safe_load((CFG / "markers.yaml").read_text(encoding="utf-8"))["markers"][5]["base_m"]
    stc = yaml.safe_load((CFG / "stations.yaml").read_text(encoding="utf-8"))
    h = {k: float(v["height_m"]) for k, v in stc["stations"].items()}
    dip = float(stc.get("dip_m", 0.0))
    st, logname = stations_from_log()

    fig = plt.figure(figsize=(21, 10.5))
    fig.suptitle("그림 1. 시연 배치도", fontsize=30, fontweight="bold", x=0.02, ha="left", y=0.975)
    ax = fig.add_axes([0.03, 0.08, 0.50, 0.80])
    sx = fig.add_axes([0.56, 0.10, 0.43, 0.76])
    colors = {"S": "#1E6F9F", "B": "#E07B1F", "C": "#3C9A3C"}

    # ---------------------------------------------------------------- top view
    ax.set_title("위에서 본 그림", fontsize=22, fontweight="bold", loc="left", pad=12)
    ax.add_patch(Rectangle((-0.62, -0.30), 1.10, 0.92, fc="#F4EBDD", ec="#C9B79C", lw=2, zorder=0))
    ax.text(0.46, 0.60, "식탁 (크기 추정)", ha="right", va="top", fontsize=15, color="#8A7556")
    ax.add_patch(Wedge((0, 0), 0.30, 0, 360, width=0.05, fc="#2B5797", alpha=0.07, zorder=1))
    ax.text(0.0, -0.285, "팔이 닿기 쉬운 범위 25~30 cm", ha="center", fontsize=13, color="#2B5797", zorder=4)
    ax.add_patch(Rectangle((-0.09, -0.09), 0.18, 0.18, fc="#9E9E9E", ec="#616161", lw=2, zorder=2))
    ax.add_patch(Circle((0, 0), 0.045, fc="white", ec="#616161", lw=2, zorder=3))
    ax.text(0.10, -0.075, "로봇팔\n(받침 18×18 cm)", fontsize=15, fontweight="bold", va="center", zorder=4)
    # paths
    pts = {k: np.array(v[2:]) for k, v in st.items()}
    ax.annotate("", xy=pts["S"], xytext=A[:2], arrowprops=dict(arrowstyle="-|>", lw=3, color="#5B4BB7",
                                                              mutation_scale=22), zorder=5)
    for k in ("B", "C"):
        ax.annotate("", xy=pts[k], xytext=pts["S"], arrowprops=dict(arrowstyle="-|>", lw=3, ls=(0, (6, 4)),
                                                                   color=colors[k], mutation_scale=22), zorder=5)
    ax.text(*(A[:2] + pts["S"]) / 2 + [0.0, 0.03], "① 처음 이동", fontsize=14, color="#5B4BB7", ha="center",
            fontweight="bold")
    ax.add_patch(Circle(A[:2], 0.014, fc="#8E3FA8", ec="white", lw=2, zorder=6))
    ax.text(A[0] - 0.02, A[1] + 0.02, "A 출발 자세\n(HOME)", fontsize=15, fontweight="bold", color="#8E3FA8",
            ha="right", zorder=6)
    for k, (label, marker, x, y) in st.items():
        ax.add_patch(Rectangle((x - 0.025, y - 0.025), 0.05, 0.05, fc="white", ec=colors[k], lw=3, zorder=6))
        ax.text(x, y, k, ha="center", va="center", fontsize=16, fontweight="bold", color=colors[k], zorder=7)
        dx, ha = {"S": (0.035, "left"), "B": (-0.035, "right"), "C": (0.0, "center")}[k]
        ax.text(x + dx, y + (0.04 if k == "C" else 0.0), f"{k} {label}\n({marker}번 마커)", ha=ha,
                va="bottom" if k == "C" else "center", fontsize=15, fontweight="bold", color=colors[k], zorder=7)
    ax.add_patch(Rectangle((m5[0] - 0.025, m5[1] - 0.025), 0.05, 0.05, fc="white", ec="#555", lw=2, zorder=6))
    ax.text(m5[0], m5[1], "5", ha="center", va="center", fontsize=13, fontweight="bold", zorder=7)
    ax.text(m5[0] - 0.035, m5[1] - 0.035, "5번 마커: 카메라가\n밀렸는지 확인", fontsize=12, color="#555", ha="right",
            va="top", zorder=7)
    # camera
    d = axis[:2] / np.linalg.norm(axis[:2])
    ang = math.degrees(math.atan2(d[1], d[0]))
    ax.add_patch(Wedge(t[:2], 1.05, ang - fov / 2, ang + fov / 2, fc="#5B4BB7", alpha=0.05, zorder=1))
    ax.add_patch(Rectangle((t[0] - 0.035, t[1] - 0.02), 0.07, 0.04, angle=0, fc="#37474F", ec="black", zorder=8))
    ax.add_patch(FancyArrowPatch(t[:2], t[:2] + 0.22 * d, arrowstyle="-|>", mutation_scale=25, lw=3, color="#37474F",
                                 zorder=8))
    ax.text(t[0], t[1] + 0.05, f"뎁스 카메라\n(높이 {t[2] * 100:.0f} cm, 화각 약 {fov:.0f}°)", ha="center",
            fontsize=15, fontweight="bold", zorder=8)
    # user and laptop (estimated)
    lap = np.array([-0.50, -0.20])
    ax.add_patch(Rectangle(lap - [0.07, 0.045], 0.14, 0.09, fc="#CFD8DC", ec="#37474F", lw=2, zorder=6))
    ax.text(*lap, "노트북", ha="center", va="center", fontsize=14, fontweight="bold", zorder=7)
    usr = np.array([-0.50, -0.40])
    ax.add_patch(Circle(usr, 0.05, fc=COL["user"][0], ec=COL["user"][1], lw=3, zorder=6, clip_on=False))
    ax.text(usr[0] + 0.08, usr[1], "사용자 (자리 추정)\n화면을 보며 말·손으로 수정", fontsize=14, va="center",
            color=COL["user"][1], fontweight="bold", clip_on=False)
    o = np.array([0.20, -0.42])
    ax.annotate("", xy=o + [0.10, 0], xytext=o, arrowprops=dict(arrowstyle="-|>", color=GRAY, lw=2), annotation_clip=False)
    ax.annotate("", xy=o + [0, 0.10], xytext=o, arrowprops=dict(arrowstyle="-|>", color=GRAY, lw=2), annotation_clip=False)
    ax.text(o[0] + 0.11, o[1], "x 로봇 오른쪽", fontsize=12, color=GRAY, va="center")
    ax.text(o[0], o[1] + 0.115, "y 로봇 앞", fontsize=12, color=GRAY, ha="center")
    ax.set_xlim(-0.95, 0.50)
    ax.set_ylim(-0.48, 0.66)
    ax.set_aspect("equal")
    ax.axis("off")

    # ---------------------------------------------------------------- side view (looking along -x, y to the right)
    sx.set_title("옆에서 본 그림 (높이)", fontsize=22, fontweight="bold", loc="left", pad=12)
    sx.add_patch(Rectangle((-0.20, -0.04), 0.98, 0.04, fc="#F4EBDD", ec="#C9B79C", lw=2))
    sx.text(-0.19, -0.03, "식탁 z = 0", ha="left", va="center", fontsize=13, color="#8A7556")
    sx.add_patch(Rectangle((-0.09, 0.0), 0.18, 0.02, fc="#9E9E9E", ec="#616161", lw=2))
    arm = np.array([[0.0, 0.02], [0.0, 0.14], [0.06, 0.26], [A[1], A[2] + 0.04], [A[1], A[2]]])
    sx.plot(arm[:, 0], arm[:, 1], color="#BDBDBD", lw=14, solid_capstyle="round", zorder=2)
    sx.plot(arm[:, 0], arm[:, 1], color="#757575", lw=2, zorder=3)
    sx.add_patch(Circle((A[1], A[2]), 0.012, fc="#8E3FA8", ec="white", lw=2, zorder=6))
    sx.text(A[1] + 0.02, A[2] + 0.015, f"A HOME 그리퍼 끝 {A[2] * 100:.0f} cm", fontsize=14, fontweight="bold",
            color="#8E3FA8")
    yS, yC = st["S"][3], st["C"][3]
    zc = h["C"]
    sx.plot([yS, yC], [zc, zc], color=COL["algo"][1], lw=3, ls=(0, (6, 4)))
    sx.annotate("", xy=(yC, zc - dip), xytext=(yC, zc), arrowprops=dict(arrowstyle="-|>", lw=3, color="#C0661A"))
    for k in ("S", "C"):
        y = st[k][3]
        sx.add_patch(Rectangle((y - 0.025, 0.0), 0.05, 0.006, fc=colors[k], ec=colors[k]))
        sx.text(y + (-0.01 if k == "S" else 0.01), -0.075, f"{k} {st[k][0]}", ha="right" if k == "S" else "left",
                fontsize=14, fontweight="bold", color=colors[k])
    sx.text((yS + yC) / 2, zc + 0.015, f"수정할 직선 경로: 높이 {zc * 100:.0f} cm", ha="center", fontsize=14,
            color=COL["algo"][1], fontweight="bold")
    sx.text(yC + 0.02, zc - dip - 0.02, f"도착하면 {dip * 100:.0f} cm 내려가서\n그대로 있음 (높이 {(zc - dip) * 100:.0f} cm)",
            fontsize=13, color="#C0661A", va="top", fontweight="bold")
    # camera in the side view: y = its y, z = its height, the optical axis in the y-z plane
    cy, cz = t[1], t[2]
    dz = np.array([axis[1], axis[2]])
    dz = dz / np.linalg.norm(dz)
    pitch = math.degrees(math.atan2(-axis[2], math.hypot(axis[0], axis[1])))
    sx.add_patch(Rectangle((cy - 0.03, cz - 0.02), 0.06, 0.04, fc="#37474F", ec="black", zorder=8))
    sx.add_patch(FancyArrowPatch((cy, cz), (cy + 0.25 * dz[0], cz + 0.25 * dz[1]), arrowstyle="-|>",
                                 mutation_scale=25, lw=3, color="#37474F", zorder=8))
    sx.plot([cy, cy], [0, cz - 0.02], color="#90A4AE", lw=2, ls=":")
    sx.text(cy + 0.05, cz, f"뎁스 카메라\n높이 {cz * 100:.0f} cm\n약 {pitch:.0f}° 내려다봄", fontsize=14,
            fontweight="bold", va="center")
    sx.text(-0.19, 0.74, f"카메라에서 {min_m * 100:.0f} cm보다 가까운 곳은 깊이를 잴 수 없어요\n"
            "→ 손으로 집는 높이도 카메라에서 충분히 떨어지게 둡니다", fontsize=13, color=GRAY, va="top")
    sx.set_xlim(-0.20, 0.80)
    sx.set_ylim(-0.12, 0.76)
    sx.set_aspect("equal")
    sx.axis("off")

    fig.text(0.02, 0.025, f"위치: 카메라 보정값(cam2base.yaml), 마커는 카메라로 읽은 값({logname}), 높이는 stations.yaml. "
             "식탁 크기·사용자와 노트북 자리·팔 모양은 개략(추정).", fontsize=13, color=GRAY)
    out = save_mpl(fig, "fig1_layout.png")
    print(out)


if __name__ == "__main__":
    main()
