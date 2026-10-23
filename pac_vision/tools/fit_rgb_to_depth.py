"""RGB→depth 정합 쌍 CSV로 dx·dy 식(고정값 / 위치 1차식 / 거리 식)을 맞추고 config/camera.yaml에 넣을 계수를 낸다.

python tools/fit_rgb_to_depth.py data/measurements/2026-10-14_pose_pairs.csv [같은 자세의 쌍 CSV ...] \
    [--corners data/measurements/2026-10-14_pose_a4corners.csv] \
    [--other data/measurements/2026-10-02_poseA_pairs_run1.csv data/measurements/2026-10-02_poseA_pairs_run2_transcribed.csv]
    [--write-pose [config/pose.yaml]]   # 채택 계수를 이 설치 자세의 pose.yaml로 저장 (기존 파일은 .bak로)

여러 설치 자세를 합쳐 카메라 고유 계수를 시험할 때 (자세 이름 = 쌍 CSV, 같은 자세의 파일은 쉼표로):
python tools/fit_rgb_to_depth.py \
    --pose A=data/measurements/2026-10-02_poseA_pairs_run1.csv,data/measurements/2026-10-02_poseA_pairs_run2_transcribed.csv \
    --pose B=data/measurements/2026-10-14_pose_pairs.csv,data/measurements/2026-10-14_pose_pairs_run2.csv \
    [--dy {auto,dist4,fixed}] [--fragment PATH]
    → 정리 표(자세, 쌍 번호, 반복 표시, 제외 여부·사유), 합친 계수, 자세별 leave-one-out(쌍 단위 / 꼭짓점 단위),
      자세 하나를 빼고 맞춘 식의 예측(교차 예측), 길이 환산(fx는 config/camera.yaml), 계수 YAML 조각(pose.yaml 형식)

카메라 불필요, numpy만. 규칙은 data/measurements/README.md "계수 맞춤 규칙" 절과 같다.
- Z: 카메라 광학 Z(m) = depth_mm/1000 - z_offset_m (Z_OFFSET 적용 후). check_camera.py가 쌍 CSV에 남긴 값 그대로.
- 맞춤에는 Z <= --zmax(1.5 m) 쌍만. 같은 꼭짓점 반복(연속 두 쌍의 RGB 클릭 5 px 안)의 dx 또는 dy 차이가 3 px를 넘으면 그 쌍들은 뺀다.
- dx: 거리 식 a + b*u + c*v + d/Z. dy: 고정값과 거리 식의 leave-one-out 차이가 0.3 px 미만이면 고정값.
- 맞춘 d는 v·Z 상관이 큰 자세에서 c와 섞이므로 물리적 시차로 해석하지 않는다. 시차는 '직접 시차'(같은 화면 높이, 다른 Z)로 본다.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path

import numpy as np

MODELS = ("fixed", "linear", "dist4")
REPEAT_RADIUS_PX = 5.0      # 연속 두 쌍의 RGB 클릭이 이 안이면 같은 꼭짓점 반복
REPEAT_MAX_DIFF_PX = 3.0    # 반복 쌍의 dx 또는 dy 차이가 이보다 크면 그 꼭짓점 제외
DY_FIXED_MARGIN_PX = 0.3    # dy: 거리 식이 이만큼 이상 좋아야 거리 식을 쓴다


def load_pairs(path) -> np.ndarray:
    """쌍 CSV → (N, 5) [u, v, dx, dy, Z]. 열 이름은 u_ref·v_ref(check_camera 원본) 또는 u·v(옮겨 적은 파일)."""
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    uk, vk = ("u_ref", "v_ref") if "u_ref" in rows[0] else ("u", "v")
    a = np.array([[float(r[uk]), float(r[vk]), float(r["dx"]), float(r["dy"]), float(r["Z"])] for r in rows])
    return a[np.isfinite(a[:, 4]) & (a[:, 4] > 0)]


def design(p, model) -> np.ndarray:
    u, v, z = p[:, 0], p[:, 1], p[:, 4]
    one = np.ones_like(u)
    if model == "fixed":
        return one[:, None]
    if model == "linear":
        return np.c_[one, u, v]
    if model == "dist4":
        return np.c_[one, u, v, 1.0 / z]
    raise ValueError(model)


def rms(x) -> float:
    return float(np.sqrt(np.mean(np.asarray(x, dtype=float) ** 2)))


def fit(p, y, model):
    return np.linalg.lstsq(design(p, model), y, rcond=None)[0]


def residual_and_loo(p, y, model):
    """(잔차 RMS, leave-one-out 오차 배열)."""
    X = design(p, model)
    res = y - X @ np.linalg.lstsq(X, y, rcond=None)[0]
    loo = []
    for i in range(len(y)):
        m = np.arange(len(y)) != i
        loo.append(y[i] - X[i] @ np.linalg.lstsq(X[m], y[m], rcond=None)[0])
    return rms(res), np.array(loo)


def coef_correlation(p, model="dist4") -> np.ndarray:
    """최소제곱 계수 사이의 상관 (X^T X)^-1을 정규화. 1에 가까우면 두 계수가 서로 바뀌어 맞을 수 있다."""
    X = design(p, model)
    cov = np.linalg.inv(X.T @ X)
    s = np.sqrt(np.diag(cov))
    return cov / np.outer(s, s)


def repeat_groups(p):
    """연속한 두 쌍의 RGB 클릭이 REPEAT_RADIUS_PX 안이면 같은 꼭짓점 반복. 반환 [(i, j, dx 차, dy 차), ...]."""
    out = []
    for i in range(len(p) - 1):
        if np.hypot(*(p[i, :2] - p[i + 1, :2])) <= REPEAT_RADIUS_PX:
            out.append((i, i + 1, abs(p[i, 2] - p[i + 1, 2]), abs(p[i, 3] - p[i + 1, 3])))
    return out


def load_many(paths):
    """같은 설치 자세의 쌍 CSV 여러 개 → (쌍 (N, 5), 파일 번호 (N,)). 반복 쌍은 파일 안에서만 찾는다."""
    parts = [load_pairs(f) for f in paths]
    return np.concatenate(parts), np.concatenate([np.full(len(a), k) for k, a in enumerate(parts)])


def select_many(p, src, zmax=1.5):
    """파일마다 select_for_fit. 반환 (mask, [(i, j, dx 차, dy 차) 전체 번호], 제외 번호)."""
    mask = np.zeros(len(p), bool)
    reps, excluded = [], []
    for k in np.unique(src):
        idx = np.flatnonzero(src == k)
        m, r, e = select_for_fit(p[idx], zmax)
        mask[idx] = m
        reps += [(idx[i], idx[j], a, b) for i, j, a, b in r]
        excluded += [int(idx[i]) for i in e]
    return mask, reps, excluded


def select_for_fit(p, zmax=1.5):
    """맞춤용 마스크와 제외 이유. 반환 (mask, 반복 목록, 제외 번호 목록)."""
    mask = p[:, 4] <= zmax
    reps = repeat_groups(p)
    excluded = []
    for i, j, ddx, ddy in reps:
        if max(ddx, ddy) > REPEAT_MAX_DIFF_PX:
            mask[i] = mask[j] = False
            excluded += [i, j]
    return mask, reps, excluded


def direct_parallax(p, dv_max=20.0, du_max=30.0, dz_min=0.15):
    """같은 화면 높이(|Δv| ≤ dv_max, |Δu| ≤ du_max)에서 Z만 다른 쌍의 Δdx / Δ(1/Z) (px·m)."""
    out = []
    for i in range(len(p)):
        for j in range(i + 1, len(p)):
            if abs(p[i, 1] - p[j, 1]) <= dv_max and abs(p[i, 0] - p[j, 0]) <= du_max and abs(p[i, 4] - p[j, 4]) >= dz_min:
                k = (p[i, 2] - p[j, 2]) / (1.0 / p[i, 4] - 1.0 / p[j, 4])
                out.append((i, j, float(k)))
    return out


def fit_report(p, mask):
    """맞춤 자료 하나에 대한 모델별 잔차·leave-one-out (dx, dy)."""
    q = p[mask]
    rep = {}
    for k, name in ((2, "dx"), (3, "dy")):
        for model in MODELS:
            r, loo = residual_and_loo(q, q[:, k], model)
            rep[(name, model)] = {"resid": r, "loo": rms(loo), "loo_max": float(np.abs(loo).max())}
    return rep


def choose(p, mask):
    """정한 규칙으로 dx·dy 계수 (a, b, c, d)."""
    q = p[mask]
    rep = fit_report(p, mask)
    cx = fit(q, q[:, 2], "dist4")
    use_dist_dy = rep[("dy", "fixed")]["loo"] - rep[("dy", "dist4")]["loo"] >= DY_FIXED_MARGIN_PX
    cy = fit(q, q[:, 3], "dist4") if use_dist_dy else np.array([q[:, 3].mean(), 0.0, 0.0, 0.0])
    return tuple(float(x) for x in cx), tuple(float(x) for x in cy), use_dist_dy, rep


def predict(coef, u, v, z):
    a, b, c, d = coef
    return a + b * np.asarray(u, float) + c * np.asarray(v, float) + d / np.asarray(z, float)


def write_pose(path, cx, cy, pairs, rep, n_fit, use_dist_dy):
    """채택 계수를 pose.yaml로 (rgb_to_depth 키만). 기존 파일은 .bak로 남긴다. 쓴 뒤 config의 키 검사를 통과하는지 확인."""
    import yaml
    from pac_vision.config import POSE_KEYS, check_overlay, load_yaml
    path = Path(path)
    if path.exists():
        path.replace(path.with_name(path.name + ".bak"))
    head = [f"# 이 설치 자세의 RGB→depth 정합 계수 (git 제외). tools/fit_rgb_to_depth.py --write-pose가 만듦, {datetime.now():%Y-%m-%d %H:%M}",
            "# 쌍 CSV: " + ", ".join(str(f) for f in pairs),
            f"# 맞춤 {n_fit}쌍, leave-one-out RMS dx {rep[('dx', 'dist4')]['loo']:.2f} px / dy "
            f"{rep[('dy', 'dist4' if use_dist_dy else 'fixed')]['loo']:.2f} px (dy {'거리 식' if use_dist_dy else '고정값'})",
            "# 카메라를 옮기면 다시 잰다."]
    doc = {"camera": {"rgb_to_depth": {"dx": dict(zip("abcd", [round(float(v), 6) for v in cx])),
                                       "dy": dict(zip("abcd", [round(float(v), 6) for v in cy]))}}}
    path.write_text("\n".join(head) + "\n" + yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    check_overlay(load_yaml(path), POSE_KEYS, path.name)
    return path


# --- 여러 설치 자세 합친 맞춤 (카메라 고유 계수 시험) ---
# 규칙은 한 자세와 같다: 파일마다 Z <= zmax와 반복 쌍 제외(select_for_fit). 가중치 없음. 그대로 둔 반복 쌍은 쌍 2개로 들어간다
# (꼭짓점 단위 leave-one-out은 반복 쌍을 함께 빼서 그 영향을 따로 본다).

LENGTH_Z_M = (0.8, 1.0, 1.5)       # 픽셀 오차 → 길이 환산을 보여 줄 거리
LOO_DIFF_WARN_PX = 0.3             # 쌍 / 꼭짓점 단위 leave-one-out 차이가 이보다 크면 ⚠️


def parse_pose_args(specs):
    """['A=f1,f2', 'B=f3'] → [(이름, [Path, ...]), ...] (입력 순서 유지)."""
    out = []
    for s in specs:
        name, sep, files = s.partition("=")
        if not sep or not name or not files:
            raise SystemExit(f"--pose 형식은 이름=CSV[,CSV...] 입니다 (받은 값: {s})")
        if name in [n for n, _ in out]:
            raise SystemExit(f"--pose 이름이 겹칩니다: {name}")
        out.append((name, [Path(f) for f in files.split(",")]))
    return out


def corner_groups(p, src) -> np.ndarray:
    """같은 꼭짓점을 반복해 찍은 쌍(repeat_groups 기준, 파일 안에서만)에 같은 꼭짓점 번호. (N,) 0부터."""
    g = np.arange(len(p))
    for k in np.unique(src):
        idx = np.flatnonzero(src == k)
        for i, j, _, _ in repeat_groups(p[idx]):
            g[idx[j]] = g[idx[i]]
    return np.unique(g, return_inverse=True)[1]


def load_poses(poses, zmax=1.5) -> dict:
    """[(이름, [CSV, ...]), ...] → 이어 붙인 쌍과 정리 정보.

    p (N, 5) [u, v, dx, dy, Z], pose (N,) 자세 번호, src (N,) 파일 번호(전체), num (N,) 파일 안 쌍 번호(1부터),
    group (N,) 꼭짓점 번호, desk (N,) 맞춤 자료(Z <= zmax + 반복 규칙), full (N,) 반복 규칙만, reason [str] 제외 사유."""
    parts, pose, src, num, files = [], [], [], [], []
    for pi, (_, paths) in enumerate(poses):
        for f in paths:
            a = load_pairs(f)
            parts.append(a)
            pose.append(np.full(len(a), pi))
            src.append(np.full(len(a), len(files)))
            num.append(np.arange(1, len(a) + 1))
            files.append(Path(f))
    p, src = np.concatenate(parts), np.concatenate(src)
    desk = np.zeros(len(p), bool)
    full = np.zeros(len(p), bool)
    reason = [""] * len(p)
    for k in np.unique(src):
        idx = np.flatnonzero(src == k)
        m, reps, _ = select_for_fit(p[idx], zmax)
        desk[idx] = m
        full[idx] = select_for_fit(p[idx], np.inf)[0]
        for i in idx[p[idx, 4] > zmax]:
            reason[i] = f"배경 (Z {p[i, 4]:.2f} > {zmax} m)"
        for i, j, ddx, ddy in reps:
            if max(ddx, ddy) > REPEAT_MAX_DIFF_PX:
                for q in (idx[i], idx[j]):
                    reason[q] = "; ".join(x for x in (reason[q], f"반복 차이 dx {ddx:.0f} / dy {ddy:.0f} px > {REPEAT_MAX_DIFF_PX:.0f}") if x)
    return {"names": [n for n, _ in poses], "files": files, "p": p, "pose": np.concatenate(pose), "src": src,
            "num": np.concatenate(num), "group": corner_groups(p, src), "desk": desk, "full": full, "reason": reason}


def _as4(model, c):
    return tuple(float(x) for x in c) if model == "dist4" else (float(c[0]), 0.0, 0.0, 0.0)


def pooled_fit(q, y, model, groups):
    """합친 자료 맞춤: (계수 4개, 잔차, 쌍 단위 leave-one-out 오차, 꼭짓점 단위 leave-one-out 오차)."""
    X = design(q, model)
    c = np.linalg.lstsq(X, y, rcond=None)[0]
    _, pair = residual_and_loo(q, y, model)
    corner = np.empty(len(y))
    for g in np.unique(groups):
        m = groups != g
        corner[~m] = y[~m] - X[~m] @ np.linalg.lstsq(X[m], y[m], rcond=None)[0]
    return _as4(model, c), y - X @ c, pair, corner


def _stat(e):
    e = np.asarray(e, float)
    return {"rms": rms(e), "max": float(np.abs(e).max()), "mean": float(e.mean()), "n": int(len(e))}


def analyze_poses(d: dict, mask_key="desk") -> dict:
    """한 자료 집합(desk 또는 full)에 대해 합친 맞춤·자세별 leave-one-out·자세 하나 빼고 예측."""
    m = d[mask_key]
    q, pose, groups = d["p"][m], d["pose"][m], d["group"][m]
    names = d["names"]
    out = {"n": {n: int((pose == i).sum()) for i, n in enumerate(names)}, "models": {}, "lopo": {},
           "z_range": {n: (float(q[pose == i, 4].min()), float(q[pose == i, 4].max())) for i, n in enumerate(names)},
           "corr_v_z": {n: float(np.corrcoef(q[pose == i, 1], q[pose == i, 4])[0, 1]) for i, n in enumerate(names)},
           "coef_corr": coef_correlation(q)}
    for k, name in ((2, "dx"), (3, "dy")):
        for model in ("fixed", "dist4"):
            c, res, pair, corner = pooled_fit(q, q[:, k], model, groups)
            out["models"][(name, model)] = {
                "coef": c, "resid": rms(res),
                "pair": {n: _stat(pair[pose == i]) for i, n in enumerate(names)},
                "corner": {n: _stat(corner[pose == i]) for i, n in enumerate(names)}}
            if len(names) < 2:
                continue
            for i, n in enumerate(names):          # 자세 n을 빼고 맞춘 식으로 n을 예측 (2자세면 교차 예측)
                tr, te = q[pose != i], q[pose == i]
                c2 = fit(tr, tr[:, k], model)
                zs, zt = (float(tr[:, 4].min()), float(tr[:, 4].max())), out["z_range"][n]
                out["lopo"][(name, model, n)] = {**_stat(te[:, k] - design(te, model) @ c2), "coef": _as4(model, c2),
                                                 "src": "+".join(x for x in names if x != n), "src_z": zs,
                                                 "extrapolate": zt[0] < zs[0] - 0.01 or zt[1] > zs[1] + 0.01}
    return out


def choose_dy(rep: dict, mode="auto") -> str:
    """dy 모델. auto: 자세별 쌍 leave-one-out의 최악값이 거리 식에서 DY_FIXED_MARGIN_PX 이상 좋으면 dist4, 아니면 fixed."""
    if mode != "auto":
        return mode
    worst = {m: max(s["rms"] for s in rep["models"][("dy", m)]["pair"].values()) for m in ("fixed", "dist4")}
    return "dist4" if worst["fixed"] - worst["dist4"] >= DY_FIXED_MARGIN_PX else "fixed"


def make_fragment(d: dict, rep: dict, dy_model: str, cmd: str) -> str:
    """합친 계수(맞춤 자료)를 pose.yaml 형식 조각으로. 출처는 주석으로 (키는 rgb_to_depth.dx·dy만)."""
    import yaml
    names = d["names"]
    cx = rep["models"][("dx", "dist4")]["coef"]
    cy = rep["models"][("dy", dy_model)]["coef"]
    head = [f"# 여러 설치 자세를 합친 RGB→depth 정합 계수 (카메라 고유 후보). tools/fit_rgb_to_depth.py, {datetime.now():%Y-%m-%d %H:%M}",
            f"# 명령: {cmd}"]
    for i, n in enumerate(names):
        fs = [str(f) for k, f in enumerate(d["files"]) if np.any((d["src"] == k) & (d["pose"] == i))]
        head.append(f"# 자세 {n}: {', '.join(fs)} — 맞춤 {rep['n'][n]}쌍")
    for name, model in (("dx", "dist4"), ("dy", dy_model)):
        r = rep["models"][(name, model)]
        head.append(f"# {name} ({'거리 식' if model == 'dist4' else '고정값'}) 자세별 leave-one-out RMS 쌍 / 꼭짓점 단위: "
                    + ", ".join(f"{n} {r['pair'][n]['rms']:.2f} / {r['corner'][n]['rms']:.2f} px" for n in names))
    doc = {"camera": {"rgb_to_depth": {"dx": dict(zip("abcd", [round(v, 6) for v in cx])),
                                       "dy": dict(zip("abcd", [round(v, 6) for v in cy]))}}}
    return "\n".join(head) + "\n" + yaml.safe_dump(doc, allow_unicode=True, sort_keys=False)


def print_table(d: dict):
    print("[정리 표] u_depth = u_rgb + dx, v_depth = v_rgb + dy (sx = sy = 1, 원본의 u_d·v_d와 같음). "
          "쌍 번호는 파일 안 번호(Z 없는 줄 제외), 꼭짓점이 같으면 반복")
    print("  자세 파일                                        쌍  u_rgb v_rgb u_depth v_depth     Z    dx  dy  꼭짓점 맞춤 사유")
    rep_groups = {g for g in np.unique(d["group"]) if (d["group"] == g).sum() > 1}
    for i in range(len(d["p"])):
        u, v, dx, dy, z = d["p"][i]
        g = d["group"][i]
        print(f"  {d['names'][d['pose'][i]]:<4} {d['files'][d['src'][i]].name:<44} {d['num'][i]:>2}  {u:5.0f} {v:5.0f} {u + dx:7.0f} {v + dy:7.0f}"
              f" {z:5.3f} {dx:+4.0f} {dy:+3.0f}  {g + 1:>4}{'R' if g in rep_groups else ' '}  "
              f"{'사용' if d['desk'][i] else '제외'} {d['reason'][i]}")


def print_poses(d: dict, reps: dict, fx: float, dy_model: str, zmax: float):
    names = d["names"]
    print_table(d)
    for key, label in (("desk", f"맞춤 자료 (파일마다 Z ≤ {zmax} m + 반복 규칙)"), ("full", "전체 (반복 규칙만, 배경 포함, 참고)")):
        rep = reps[key]
        print(f"\n===== {label}: " + ", ".join(f"자세 {n} {rep['n'][n]}쌍" for n in names) + " =====")
        print("  Z 범위 " + ", ".join(f"{n} {a:.2f}-{b:.2f} m" for n, (a, b) in rep["z_range"].items())
              + " | v·Z 상관 " + ", ".join(f"{n} {c:+.2f}" for n, c in rep["corr_v_z"].items())
              + f" | 합친 자료의 c–d 계수 상관 {rep['coef_corr'][2, 3]:+.2f}")
        for name in ("dx", "dy"):
            for model in ("fixed", "dist4"):
                r = rep["models"][(name, model)]
                a, b, c, dd = r["coef"]
                coef = f"{a:+.2f}" if model == "fixed" else f"{a:+.2f} {b:+.5f}·u {c:+.5f}·v {dd:+.2f}/Z"
                print(f"  [합친] {name} {model:<6} = {coef} | 잔차 {r['resid']:.2f} px")
                for n in names:
                    s, t = r["pair"][n], r["corner"][n]
                    flag = " ⚠️ 차이 > %.1f px" % LOO_DIFF_WARN_PX if abs(s["rms"] - t["rms"]) > LOO_DIFF_WARN_PX else ""
                    print(f"         자세 {n} leave-one-out: 쌍 단위 RMS {s['rms']:.2f} (최대 {s['max']:.1f}) / "
                          f"꼭짓점 단위 RMS {t['rms']:.2f} (최대 {t['max']:.1f}) px{flag}")
        if rep["lopo"]:
            print("  [자세 하나 빼고 예측] 나머지 자세로 맞춘 식 → 뺀 자세의 쌍 (RMS / 최대 / 평균 px)")
            for name in ("dx", "dy"):
                for model in ("fixed", "dist4"):
                    for n in names:
                        r = rep["lopo"][(name, model, n)]
                        ex = " ⚠️ 외삽(맞춤 자료의 Z 범위 밖, 표시만)" if r["extrapolate"] else ""
                        print(f"    {name} {model:<6} {r['src']} → {n}: {r['rms']:5.2f} / {r['max']:5.1f} / {r['mean']:+5.2f}"
                              f" (맞춤 Z {r['src_z'][0]:.2f}-{r['src_z'][1]:.2f} m){ex}")

    rep = reps["desk"]
    print(f"\n[길이 환산] 픽셀 오차 × Z / fx (fx {fx:.0f}, config/camera.yaml) — 맞춤 자료, 쌍 단위 leave-one-out, mm")
    for name, model in (("dx", "dist4"), ("dy", "fixed"), ("dy", "dist4")):
        for n in names:
            s = rep["models"][(name, model)]["pair"][n]
            print(f"    {name} {model:<6} 자세 {n}: RMS {s['rms']:.2f} px → "
                  + ", ".join(f"Z {z} m {s['rms'] * z / fx * 1000:.1f}" for z in LENGTH_Z_M)
                  + f" | 최대 {s['max']:.1f} px → " + ", ".join(f"{s['max'] * z / fx * 1000:.1f}" for z in LENGTH_Z_M))
    print(f"\n[dy 모델] 조각에 넣은 것: {'거리 식' if dy_model == 'dist4' else '고정값'} "
          f"(auto 규칙: 자세별 최악 leave-one-out이 거리 식에서 {DY_FIXED_MARGIN_PX} px 이상 좋으면 거리 식)")


def main_poses(args):
    import shlex
    import sys
    poses = parse_pose_args(args.pose)
    d = load_poses(poses, args.zmax)
    reps = {"desk": analyze_poses(d, "desk"), "full": analyze_poses(d, "full")}
    dy_model = choose_dy(reps["desk"], args.dy)
    from pac_vision.config import load_config
    fx = float(load_config(args.config_dir, ("camera",), overlays=False)["camera"]["depth"]["fx"])
    print_poses(d, reps, fx, dy_model, args.zmax)
    frag = make_fragment(d, reps["desk"], dy_model, "python tools/fit_rgb_to_depth.py " + " ".join(shlex.quote(a) for a in sys.argv[1:]))
    print("\n[계수 YAML 조각] (pose.yaml 형식: 키 camera.rgb_to_depth.dx·dy)\n" + frag)
    if args.fragment:
        from pac_vision.config import POSE_KEYS, check_overlay, load_yaml
        out = Path(args.fragment)
        out.write_text(frag, encoding="utf-8")
        check_overlay(load_yaml(out), POSE_KEYS, out.name)
        print(f"[저장] {out}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="RGB→depth dx·dy 식 맞춤")
    ap.add_argument("pairs", nargs="*", help="이 설치 자세의 쌍 CSV (check_camera.py 원본, 여러 개면 이어 붙임)")
    ap.add_argument("--pose", action="append", default=[], metavar="이름=CSV[,CSV]",
                    help="여러 설치 자세 합친 맞춤 (반복해서 줌). 주면 위치 인자·--other·--corners·--write-pose 대신 이 모드로 동작")
    ap.add_argument("--dy", choices=("auto", "dist4", "fixed"), default="auto", help="--pose: 조각에 넣을 dy 모델")
    ap.add_argument("--fragment", metavar="PATH", help="--pose: 계수 YAML 조각을 이 파일로도 저장 (pose.yaml 형식)")
    ap.add_argument("--config-dir", default=str(Path(__file__).resolve().parents[1] / "config"),
                    help="--pose: 길이 환산의 fx를 읽을 config 폴더")
    ap.add_argument("--corners", help="A4 꼭짓점 CSV (corner, u, v, Z): 꼭짓점에서의 예측값 출력")
    ap.add_argument("--other", nargs="*", default=[], help="다른 자세의 쌍 CSV (두 자세 합친 계수 참고 계산)")
    ap.add_argument("--zmax", type=float, default=1.5)
    ap.add_argument("--write-pose", nargs="?", const=str(Path(__file__).resolve().parents[1] / "config" / "pose.yaml"),
                    default=None, metavar="PATH", help="채택 계수를 pose.yaml로 저장 (기본 config/pose.yaml, 기존 파일은 .bak)")
    args = ap.parse_args()
    if args.pose:
        if args.pairs or args.other or args.corners or args.write_pose:
            ap.error("--pose는 위치 인자·--other·--corners·--write-pose와 함께 쓰지 않습니다")
        return main_poses(args)
    if not args.pairs:
        ap.error("쌍 CSV(위치 인자) 또는 --pose가 필요합니다")

    p, src = load_many(args.pairs)
    desk = p[:, 4] <= args.zmax
    mask, reps, excluded = select_many(p, src, args.zmax)
    for k, f in enumerate(args.pairs):
        print(f"[자료] 파일 {k + 1} {f}: {int((src == k).sum())}쌍")
    print(f"       합계 {len(p)}쌍 (번호는 이어 붙인 순서), Z ≤ {args.zmax} m {int(desk.sum())}쌍, 맞춤에 쓰는 쌍 {int(mask.sum())}")
    print("       Z = 카메라 광학 Z (depth_mm/1000 - z_offset_m, check_camera.py가 남긴 값 그대로)")
    for i, j, ddx, ddy in reps:
        tag = "제외" if max(ddx, ddy) > REPEAT_MAX_DIFF_PX else "유지"
        print(f"[반복] 쌍 {i + 1}·{j + 1}: dx 차 {ddx:.0f}, dy 차 {ddy:.0f} px → {tag} (기준 {REPEAT_MAX_DIFF_PX:.0f} px)")
    print(f"       v·Z 상관 (맞춤 자료) {np.corrcoef(p[mask, 1], p[mask, 4])[0, 1]:+.2f}")

    for label, m in (("Z 제한만 (제외 규칙 적용 전)", desk), ("맞춤 자료 (제외 규칙 적용)", mask)):
        rep = fit_report(p, m)
        print(f"\n[{label}] {int(m.sum())}쌍 — 잔차 RMS / leave-one-out RMS (최대) px")
        for name in ("dx", "dy"):
            for model in MODELS:
                r = rep[(name, model)]
                print(f"    {name} {model:<6}: {r['resid']:5.2f} / {r['loo']:5.2f} ({r['loo_max']:4.1f})")

    cx, cy, use_dist_dy, rep = choose(p, mask)
    corr = coef_correlation(p[mask])
    names = ("a", "b(u)", "c(v)", "d(1/Z)")
    print("\n[계수 상관] 거리 식, 맞춤 자료 (|값|이 1에 가까우면 두 계수가 서로 바뀌어 맞을 수 있음)")
    for i in range(4):
        print("    " + names[i].ljust(7) + " ".join(f"{corr[i, j]:+.2f}" for j in range(4)))
    print(f"    c–d 상관 {corr[2, 3]:+.2f}")
    print("    ⚠️ 이 자세의 d는 v·Z 상관 때문에 c와 섞인다. 물리적 시차 계수로 해석하지 않는다.")

    dp = direct_parallax(p[desk])
    idx = np.flatnonzero(desk)
    print("\n[직접 시차] 같은 화면 높이(|Δv| ≤ 20, |Δu| ≤ 30 px)에서 Z만 0.15 m 이상 다른 쌍: Δdx / Δ(1/Z)")
    for i, j, k in dp:
        a, b = p[idx[i]], p[idx[j]]
        print(f"    쌍 {idx[i] + 1}·{idx[j] + 1}: Z {a[4]:.3f} / {b[4]:.3f} m, dx {a[2]:+.0f} / {b[2]:+.0f} → {k:.1f} px·m")
    if not dp:
        print("    (해당 쌍 없음)")

    print("\n[채택] dx 거리 식, dy " + ("거리 식" if use_dist_dy else "고정값")
          + f" (dy leave-one-out 고정 {rep[('dy', 'fixed')]['loo']:.2f} / 거리 식 {rep[('dy', 'dist4')]['loo']:.2f} px,"
          f" 기준 차이 {DY_FIXED_MARGIN_PX} px)")
    print("config/camera.yaml rgb_to_depth에 넣을 값:")
    print(f"  dx: {{a: {cx[0]:.3f}, b: {cx[1]:.6f}, c: {cx[2]:.6f}, d: {cx[3]:.3f}}}")
    print(f"  dy: {{a: {cy[0]:.3f}, b: {cy[1]:.6f}, c: {cy[2]:.6f}, d: {cy[3]:.3f}}}")
    if args.write_pose:
        out = write_pose(args.write_pose, cx, cy, args.pairs, rep, int(mask.sum()), use_dist_dy)
        print(f"[저장] {out}  (calibrate.py·detect_objects.py가 camera.yaml 대신 이 계수를 씁니다)")

    if args.corners:
        with open(args.corners, encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        u = np.array([float(r["u"]) for r in rows]); v = np.array([float(r["v"]) for r in rows])
        z = np.array([float(r["Z"]) for r in rows])
        px, py = predict(cx, u, v, z), predict(cy, u, v, z)
        print("\n[A4 꼭짓점] 채택 식의 예측")
        for r, a, b in zip(rows, px, py):
            print(f"    꼭짓점 {r['corner']}: RGB ({float(r['u']):.0f}, {float(r['v']):.0f}) Z {float(r['Z']):.3f} m → dx {a:+.2f}, dy {b:+.2f}")

    if args.other:
        o = np.concatenate([load_pairs(f) for f in args.other])
        both = np.r_[o, p[mask]]
        tag = np.r_[np.zeros(len(o)), np.ones(int(mask.sum()))]
        print(f"\n[참고: 두 자세 합친 거리 식] 다른 자세 {len(o)}쌍 + 이 자세 맞춤 자료 {int(mask.sum())}쌍 (채택하지 않음)")
        for k, name in ((2, "dx"), (3, "dy")):
            c = fit(both, both[:, k], "dist4")
            _, loo = residual_and_loo(both, both[:, k], "dist4")
            print(f"    {name} = {c[0]:+.2f} {c[1]:+.5f}·u {c[2]:+.5f}·v {c[3]:+.2f}/Z | leave-one-out 다른 자세 "
                  f"{rms(loo[tag == 0]):.2f}, 이 자세 {rms(loo[tag == 1]):.2f} px")


if __name__ == "__main__":
    main()
