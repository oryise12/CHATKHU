"""Copy the robot model from the reference repo at a fixed commit into models/upstream_<commit>/.

Read-only on the reference repo: only `git rev-parse / ls-tree / status / archive` are run there. The copy is
taken from the commit (not the working tree), checked blob-by-blob against the commit, hashed into
models/manifest.sha256 and compared with the reference's own pinned hashes (original_model_sha256.json).
An existing copy is never deleted or overwritten; re-running only re-verifies it.

    python tools/copy_model.py            # defaults: the repo around this workspace @ model.UPSTREAM_COMMIT
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))
from manipulator_control import REF_REPO as DEFAULT_REF  # noqa: E402
PATHS = ["6dof_description", "mujoco_sim/models/generated", "mujoco_sim/tests/original_model_sha256.json",
         "mujoco_sim/configs/demo.json"]
REQUIRED = ["6dof_description/urdf/6dof.urdf", "6dof_description/urdf/6dof.xacro",
            "6dof_description/urdf/6dof_gripper.urdf", "6dof_description/urdf/6dof_gripper.xacro",
            "6dof_description/mujoco/6dof_mj.urdf", "6dof_description/mujoco/6dof_control.xml",
            "6dof_description/mujoco/scene.xml", "6dof_description/meshes/gripper.stl",
            "mujoco_sim/models/generated/pinch_scene.xml"]


def _upstream_commit() -> str:
    sys.path.insert(0, str(WS / "src"))
    from manipulator_control.model import UPSTREAM_COMMIT
    return UPSTREAM_COMMIT


def git(ref: Path, *args: str, binary: bool = False):
    out = subprocess.run(["git", "-C", str(ref), *args], check=True, capture_output=True)
    return out.stdout if binary else out.stdout.decode().strip()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def blob_sha1(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ref", type=Path, default=DEFAULT_REF)
    ap.add_argument("--commit", default=_upstream_commit())
    a = ap.parse_args()

    commit = git(a.ref, "rev-parse", "--verify", f"{a.commit}^{{commit}}")
    short = commit[:7]
    dest = WS / "models" / f"upstream_{short}"
    tree = {}
    for line in git(a.ref, "ls-tree", "-r", commit, "--", *PATHS).splitlines():
        meta, path = line.split("\t", 1)
        tree[path] = meta.split()[2]
    missing = [p for p in REQUIRED if p not in tree]
    if missing:
        print(f"commit {short} does not track: {missing}", file=sys.stderr)
        return 1

    archive = git(a.ref, "archive", "--format=tar", commit, "--", *PATHS, binary=True)
    files = {}
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        for member in tar.getmembers():
            if member.isfile():
                files[member.name] = tar.extractfile(member).read()
    bad = [p for p, d in files.items() if blob_sha1(d) != tree.get(p)]
    if bad or set(files) != set(tree):
        print(f"archive does not match the commit tree: {bad or set(files) ^ set(tree)}", file=sys.stderr)
        return 1

    existed = dest.exists()
    for path, data in files.items():
        out = dest / path
        if out.exists():
            if out.read_bytes() != data:
                print(f"existing copy differs from {short}:{path} (not overwritten)", file=sys.stderr)
                return 1
            continue
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(data)

    manifest = {p: sha256(d) for p, d in sorted(files.items())}
    (WS / "models" / "manifest.sha256").write_text(
        "".join(f"{h}  upstream_{short}/{p}\n" for p, h in manifest.items()))

    pinned = json.loads(files["mujoco_sim/tests/original_model_sha256.json"])
    pin_check = {rel: manifest.get(f"6dof_description/{rel}") == h for rel, h in pinned.items()}
    dirty = git(a.ref, "status", "--porcelain", "--", *PATHS)
    head = git(a.ref, "rev-parse", "HEAD")
    branch = git(a.ref, "rev-parse", "--abbrev-ref", "HEAD")
    write_provenance(dest, a.ref, commit, head, branch, dirty, manifest, pin_check, existed)
    print(f"{'verified' if existed else 'copied'} {len(files)} files from {a.ref} @ {short} -> {dest}")
    print(f"pinned hashes (original_model_sha256.json): {sum(pin_check.values())}/{len(pin_check)} match")
    return 0 if all(pin_check.values()) else 1


def write_provenance(dest, ref, commit, head, branch, dirty, manifest, pin_check, existed) -> None:
    now = dt.datetime.now().astimezone().isoformat(timespec="seconds")
    lines = [
        "# 모델 출처 (PROVENANCE)",
        "",
        f"- 생성: `tools/copy_model.py`, {now}{' (기존 사본 재검증)' if existed else ''}",
        f"- 원본 저장소(읽기 전용): `{ref}`",
        f"- 원본 브랜치·HEAD: `{branch}` @ `{head}`",
        f"- 복사한 커밋: `{commit}` (`git archive`로 커밋 내용 그대로, 작업 트리가 아님)",
        f"- 복사 시점 원본 작업 트리의 해당 경로 변경: {'없음' if not dirty else '있음 — ' + dirty}",
        f"- 사본 위치: `models/{dest.name}/` (원본의 상대 배치 유지 → `pinch_scene.xml`의 "
        "`meshdir=\"../../../6dof_description/meshes/\"`가 수정 없이 사본 메시를 가리킴)",
        "- 변환: 없음. 사본 파일은 수정하지 않는다. 표시용 모델은 `tools/build_viz_model.py`가 "
        "`models/derived/`에 따로 만든다.",
        "- 원본 `build_scene`·xacro는 실행하지 않았다 (원본 `urdf/6dof_gripper.*`를 덮어쓰는 생성기).",
        "- 검증: 사본의 모든 파일이 커밋의 blob 해시(git SHA-1)와 일치.",
        "",
        "## 원본이 고정한 해시와 대조 (`mujoco_sim/tests/original_model_sha256.json`)",
        "",
        "| 파일 (6dof_description/) | 일치 |",
        "|---|---|",
        *[f"| `{rel}` | {'예' if ok else '**아니오**'} |" for rel, ok in pin_check.items()],
        "",
        "## 파일별 sha256",
        "",
        "전체 목록: `models/manifest.sha256` (`cd models && sha256sum -c manifest.sha256`로 확인).",
        "",
        "| 파일 | sha256 |",
        "|---|---|",
        *[f"| `{p}` | `{h[:16]}…` |" for p, h in manifest.items()],
        "",
    ]
    (WS / "models" / "PROVENANCE.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
