"""Shared look of the KHU AI Challenge figures (docs/figures/): colours, Noto Sans CJK KR, PNG output.

Colour roles (same in every figure):
    algo   blue    algorithm / simulator / checks          ai     purple  AI models (ChatKHU, MediaPipe Hands)
    user   green   what the user does / says                robot  orange  the real arm, stops
    dec    dashed  a decision                               pill   grey    an answer / condition label
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
FONT = "Noto Sans CJK KR"
MIN_WIDTH = 1600

COL = {                      # (fill, line)
    "algo": ("#E8F1FB", "#2B5797"),
    "ai": ("#EEEBFB", "#5B4BB7"),
    "user": ("#E6F4EE", "#2E7D5B"),
    "robot": ("#FDF0E6", "#C0661A"),
    "dec": ("#FFFFFF", "#888888"),
    "pill": ("#F2F0EA", "#9A968C"),
    "stop": ("#FBE3D0", "#C0661A"),
    "warn": ("#FDECEC", "#C62828"),
}
INK, GRAY, EDGE = "#1a1a1a", "#666666", "#777777"


# ------------------------------------------------------------------ matplotlib with the KR face of the Noto CJK collection
def _kr_face(ttc: Path, out: Path) -> Path:
    """The Noto Sans CJK KR face of a .ttc collection as its own .otf (matplotlib reads only a collection's first face)."""
    if out.exists():
        return out
    from fontTools.ttLib import TTCollection
    for font in TTCollection(str(ttc)).fonts:
        family = font["name"].getDebugName(1) or ""
        if family.startswith(FONT):
            out.parent.mkdir(parents=True, exist_ok=True)
            font.save(str(out))
            return out
    raise RuntimeError(f"{FONT} not found in {ttc}")


def mpl_setup():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    cache = Path.home() / ".cache" / "pinch_figures"
    base = Path("/usr/share/fonts/opentype/noto")
    for weight in ("Regular", "Bold"):
        fm.fontManager.addfont(str(_kr_face(base / f"NotoSansCJK-{weight}.ttc", cache / f"NotoSansCJKkr-{weight}.otf")))
    plt.rcParams.update({"font.family": FONT, "axes.unicode_minus": False, "savefig.facecolor": "white",
                         "figure.facecolor": "white"})
    return plt


def save_mpl(fig, name: str, dpi: int = 100) -> Path:
    out = HERE / name
    fig.savefig(out, dpi=dpi, facecolor="white")
    _check_width(out)
    return out


# ------------------------------------------------------------------ graphviz
def dot_png(src: str, name: str) -> Path:
    """Render a dot source to docs/figures/<name>.png (and keep the .dot), at least MIN_WIDTH px wide."""
    if shutil.which("dot") is None:
        raise RuntimeError("graphviz 'dot' is needed (sudo apt install graphviz)")
    dot = HERE / f"{name}.dot"
    dot.write_text(src, encoding="utf-8")
    out = HERE / f"{name}.png"
    dpi = 110
    for _ in range(6):
        subprocess.run(["dot", "-Tpng", f"-Gdpi={dpi}", str(dot), "-o", str(out)], check=True)
        w = png_size(out)[0]
        if w >= MIN_WIDTH:
            return out
        dpi = int(dpi * MIN_WIDTH / w) + 2
    raise RuntimeError(f"{out} narrower than {MIN_WIDTH}px")


def dot_header(rankdir: str = "TB", nodesep: float = 0.45, ranksep: float = 0.45) -> str:
    return (f'  graph [fontname="{FONT}", bgcolor="white", rankdir={rankdir}, nodesep={nodesep}, ranksep={ranksep}, '
            f'pad=0.35, newrank=true, splines=ortho];\n'
            f'  node [fontname="{FONT} Bold", fontsize=17, shape=box, style="rounded,filled", penwidth=1.6, '
            f'margin="0.30,0.14", fontcolor="{INK}"];\n'
            f'  edge [color="{EDGE}", arrowsize=0.8, penwidth=1.4, fontname="{FONT}", fontsize=14];\n')


def style(kind: str, dashed: bool = False, **extra) -> str:
    fill, line = COL[kind]
    st = "rounded,filled,dashed" if dashed else "rounded,filled"
    attrs = {"fillcolor": fill, "color": line, "style": st, **extra}
    return ", ".join(f'{k}="{v}"' for k, v in attrs.items())


def pill(kind: str = "pill") -> str:
    return style(kind, fontsize=14, margin="0.14,0.05", penwidth=1.1) + f', fontname="{FONT} Bold"'


# ------------------------------------------------------------------ checks
def png_size(path: Path) -> tuple[int, int]:
    import struct
    with open(path, "rb") as f:
        head = f.read(24)
    return struct.unpack(">II", head[16:24])


def _check_width(path: Path) -> None:
    w, _ = png_size(path)
    if w < MIN_WIDTH:
        raise RuntimeError(f"{path} is {w}px wide (< {MIN_WIDTH})")
