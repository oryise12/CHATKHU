# 모델 출처 (PROVENANCE)

- 생성: `tools/copy_model.py`, 2026-10-28T12:56:28+09:00
- 원본 저장소(읽기 전용): `/home/jammin/pinch_khu_ws/26_project_manipulator`
- 원본 브랜치·HEAD: `work/pinch-khu-demo` @ `849f2e17004a9fdde2ca494119b41fc09cdb758d`
- 복사한 커밋: `849f2e17004a9fdde2ca494119b41fc09cdb758d` (`git archive`로 커밋 내용 그대로, 작업 트리가 아님)
- 복사 시점 원본 작업 트리의 해당 경로 변경: 없음
- 사본 위치: `models/upstream_849f2e1/` (원본의 상대 배치 유지 → `pinch_scene.xml`의 `meshdir="../../../6dof_description/meshes/"`가 수정 없이 사본 메시를 가리킴)
- 변환: 없음. 사본 파일은 수정하지 않는다. 표시용 모델은 `tools/build_viz_model.py`가 `models/derived/`에 따로 만든다.
- 원본 `build_scene`·xacro는 실행하지 않았다 (원본 `urdf/6dof_gripper.*`를 덮어쓰는 생성기).
- 검증: 사본의 모든 파일이 커밋의 blob 해시(git SHA-1)와 일치.

## 원본이 고정한 해시와 대조 (`mujoco_sim/tests/original_model_sha256.json`)

| 파일 (6dof_description/) | 일치 |
|---|---|
| `mujoco/scene.xml` | 예 |
| `mujoco/6dof_control.xml` | 예 |
| `meshes/base_link.stl` | 예 |
| `meshes/r_a_1.stl` | 예 |
| `meshes/link_1.stl` | 예 |
| `meshes/link__1__1.stl` | 예 |
| `meshes/r_a__1__1.stl` | 예 |
| `meshes/r_a__2__1.stl` | 예 |
| `meshes/housing_1.stl` | 예 |
| `urdf/6dof.xacro` | 예 |
| `urdf/6dof.urdf` | 예 |
| `urdf/materials.xacro` | 예 |
| `meshes/link_1_engraved.stl` | 예 |
| `meshes/link_1_engraving.stl` | 예 |
| `meshes/link__1__1_engraved.stl` | 예 |
| `meshes/link__1__1_engraving.stl` | 예 |
| `meshes/link_1_ring.stl` | 예 |
| `meshes/link__1__1_ring.stl` | 예 |
| `meshes/gripper_body.stl` | 예 |
| `meshes/gripper_fingers.stl` | 예 |

## 파일별 sha256

전체 목록: `models/manifest.sha256` (`cd models && sha256sum -c manifest.sha256`로 확인).

| 파일 | sha256 |
|---|---|
| `6dof_description/.gitignore` | `00cc23688fcd47f9…` |
| `6dof_description/CMakeLists.txt` | `f900f98442b68230…` |
| `6dof_description/LICENSE` | `25843f9143b800e5…` |
| `6dof_description/README.md` | `9a3e0c2cf8b1f5f6…` |
| `6dof_description/launch/display.launch.py` | `ac5f14b02e53a2f1…` |
| `6dof_description/meshes/base_link.stl` | `3bc640575627f9e7…` |
| `6dof_description/meshes/gripper.stl` | `de8b75f01a2fcd93…` |
| `6dof_description/meshes/gripper_body.stl` | `f870328015f6a39a…` |
| `6dof_description/meshes/gripper_fingers.stl` | `86b93fa799db2611…` |
| `6dof_description/meshes/housing_1.stl` | `a6d9dba509929dae…` |
| `6dof_description/meshes/link_1.stl` | `7f24e5d1ef43c28b…` |
| `6dof_description/meshes/link_1_engraved.stl` | `552401b905066416…` |
| `6dof_description/meshes/link_1_engraving.stl` | `3b7e0fb56a458c56…` |
| `6dof_description/meshes/link_1_ring.stl` | `2c1cfb495015e4a3…` |
| `6dof_description/meshes/link__1__1.stl` | `95d388eeae5f99b4…` |
| `6dof_description/meshes/link__1__1_engraved.stl` | `a615bef98ca300a7…` |
| `6dof_description/meshes/link__1__1_engraving.stl` | `0d26337f471326a1…` |
| `6dof_description/meshes/link__1__1_ring.stl` | `273d4a2ade2adec3…` |
| `6dof_description/meshes/r_a_1.stl` | `d73a3419ec13334f…` |
| `6dof_description/meshes/r_a__1__1.stl` | `28526e462a07e790…` |
| `6dof_description/meshes/r_a__2__1.stl` | `e7158912836a46d2…` |
| `6dof_description/mujoco/6dof.xml` | `751719ad57446120…` |
| `6dof_description/mujoco/6dof_control.xml` | `8fc24243633a3401…` |
| `6dof_description/mujoco/6dof_mj.urdf` | `d9b81295c1626f77…` |
| `6dof_description/mujoco/scene.xml` | `bec1ff0a4e1fb224…` |
| `6dof_description/package.xml` | `e87819667ea7e44b…` |
| `6dof_description/rviz/display.rviz` | `e06f3410eff8f82d…` |
| `6dof_description/urdf/6dof.urdf` | `d09438d90290fc18…` |
| `6dof_description/urdf/6dof.xacro` | `b3a2ee9230c1a486…` |
| `6dof_description/urdf/6dof_gripper.urdf` | `a8d4ccd0f564bd4e…` |
| `6dof_description/urdf/6dof_gripper.xacro` | `842db2415ffddebe…` |
| `6dof_description/urdf/materials.xacro` | `9caa339ffa0a6814…` |
| `mujoco_sim/configs/demo.json` | `ade0facdd4b8ce06…` |
| `mujoco_sim/models/generated/desk_wood.png` | `8d257bdd04b02b05…` |
| `mujoco_sim/models/generated/pinch_scene.xml` | `7ea30c3d53aadcba…` |
| `mujoco_sim/tests/original_model_sha256.json` | `35f84fdee1867f98…` |
