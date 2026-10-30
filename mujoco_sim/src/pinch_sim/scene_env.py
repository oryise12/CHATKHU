"""Scene environment added by build_scene: a light wooden desk under the robot, a grey floor and the plate the
robot stands on.

World frame: z = 0 is the desk top (the table; targets, the camera calibration and the cameras use it). The desk TOP
SURFACE is placed at z = 0 and the room floor (the original `floor` plane) is lowered to z = -desk.height_m.
Without a mount the robot's base_link is the world origin. With `mount` (add_mount) the robot stands on a plate:
base_link and everything above it are raised by mount.height_m (base bottom at z = height_m), the plate fills
z 0..height_m around the J1 axis, and x, y, the joint angles and the table frame stay as they are.

The desk and the plate are their own bodies without joints (welded to the world): MuJoCo does not collide static
bodies with each other (base mesh <-> plate <-> desk top, legs <-> floor), while every robot link <-> desk / plate
geom pair is collided normally. No contact exclusion is needed. Validation's "floor" clearance is measured to the
desk top and the plate (RobotIndex.support_geoms).

The wood texture is generated here (deterministic, low-contrast grain) next to the generated scene.
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

DESK_TEXTURE = "desk_wood.png"


def wood_texture(path: Path, width: int = 512, height: int = 256, seed: int = 7) -> Path:
    """Light wood with gentle grain along the texture's u axis (contrast about +-4 %)."""
    import cv2
    rng = np.random.default_rng(seed)
    v, u = np.mgrid[0:height, 0:width].astype(np.float32)
    warp = 3.0 * np.sin(u / 48.0 + 1.3) + 1.5 * np.sin(u / 20.0) + 1.0 * np.sin(u / 6.5 + v / 105.0)
    rings = np.sin((v + warp) / height * 2 * np.pi * 9.0)
    fine = np.sin((v + 0.6 * warp) / height * 2 * np.pi * 47.0)
    noise = cv2.GaussianBlur(rng.normal(0, 1, (height, width)).astype(np.float32), (0, 0), sigmaX=5, sigmaY=1.0)
    shade = 1.0 + 0.025 * rings + 0.008 * fine + 0.008 * noise / (noise.std() + 1e-9)
    base = np.array([0.78, 0.63, 0.44], np.float32)            # light oak (RGB); brighter washes out under the lights
    rgb = np.clip(base[None, None, :] * shade[..., None], 0, 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), (rgb[..., ::-1] * 255).round().astype(np.uint8))
    return path


def add_desk_and_floor(spec: mujoco.MjSpec, dc: dict, texture_dir: Path) -> dict:
    """Add the desk body, lower and grey the floor. Returns a summary for the build report."""
    x0, x1 = dc["top_x_m"]
    y0, y1 = dc["top_y_m"]
    z_top, t, h = float(dc["top_z_m"]), float(dc["thickness_m"]), float(dc["height_m"])
    z_floor = z_top - h
    tex_path = wood_texture(texture_dir / DESK_TEXTURE)
    spec.add_texture(name="desk_wood", type=mujoco.mjtTexture.mjTEXTURE_2D, file=str(tex_path))
    mat = spec.add_material(name="desk_wood", texrepeat=dc.get("texrepeat", [1.0, 1.0]), texuniform=False,
                            rgba=dc.get("top_tint_rgba", [0.82, 0.82, 0.82, 1.0]),   # dims the texture: the
                            reflectance=0.0, specular=0.05, shininess=0.1)              # scene lights saturate it
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "desk_wood"
    spec.add_material(name="desk_frame", rgba=dc["frame_rgba"], reflectance=0.0, specular=0.1)

    # floor: same plane geom, moved down to the room floor and made grey
    floor = spec.geom("floor")
    floor.pos = [0.0, 0.0, z_floor]
    gt = spec.texture("groundplane")
    if gt is not None:
        gt.rgb1, gt.rgb2 = dc["floor_rgb"], dc["floor_rgb2"]
        gt.markrgb = dc["floor_rgb2"]

    body = spec.worldbody.add_body(name="desk", pos=[0.0, 0.0, 0.0])
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
    body.add_geom(name="desk_top", type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, t / 2],
                  pos=[cx, cy, z_top - t / 2], material="desk_wood")
    leg, inset = float(dc["leg_size_m"]), float(dc["leg_inset_m"])
    leg_h = h - t
    for sx in (-1, 1):
        for sy in (-1, 1):
            px = cx + sx * (hx - inset - leg / 2)
            py = cy + sy * (hy - inset - leg / 2)
            body.add_geom(name=f"desk_leg_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
                          type=mujoco.mjtGeom.mjGEOM_BOX, size=[leg / 2, leg / 2, leg_h / 2],
                          pos=[px, py, z_top - t - leg_h / 2], material="desk_frame")
    ah, aw = float(dc["apron_height_m"]), 0.012          # apron (frame under the top)
    za = z_top - t - ah / 2
    ix, iy = hx - inset - leg / 2, hy - inset - leg / 2
    body.add_geom(name="desk_apron_front", type=mujoco.mjtGeom.mjGEOM_BOX, size=[ix, aw, ah / 2],
                  pos=[cx, cy + iy, za], material="desk_frame")
    body.add_geom(name="desk_apron_back", type=mujoco.mjtGeom.mjGEOM_BOX, size=[ix, aw, ah / 2],
                  pos=[cx, cy - iy, za], material="desk_frame")
    body.add_geom(name="desk_apron_left", type=mujoco.mjtGeom.mjGEOM_BOX, size=[aw, iy, ah / 2],
                  pos=[cx - ix, cy, za], material="desk_frame")
    body.add_geom(name="desk_apron_right", type=mujoco.mjtGeom.mjGEOM_BOX, size=[aw, iy, ah / 2],
                  pos=[cx + ix, cy, za], material="desk_frame")
    return {"desk_top_x_m": [x0, x1], "desk_top_y_m": [y0, y1], "desk_top_z_m": z_top, "floor_z_m": z_floor,
            "texture": DESK_TEXTURE}


def add_mount(spec: mujoco.MjSpec, mc: dict) -> dict:
    """Raise the robot (the world-body base_link geoms and the J1 body r_a_1) by mc.height_m and add the plate under
    it, centred on the J1 axis. Returns a summary for the build report."""
    h = float(mc["height_m"])
    base = [g for g in spec.worldbody.geoms if g.meshname == "base_link"]
    arm = spec.worldbody.bodies
    if len(base) != 2 or [b.name for b in arm] != ["r_a_1"]:
        raise RuntimeError(f"expected 2 base_link geoms and body r_a_1 on the world body, got {len(base)}, "
                           f"{[b.name for b in arm]}")
    for el in base + arm:
        el.pos = [el.pos[0], el.pos[1], el.pos[2] + h]
    cx, cy = float(arm[0].pos[0]), float(arm[0].pos[1])                  # J1 axis
    sx, sy = (float(v) for v in mc["plate_size_m"])
    body = spec.worldbody.add_body(name="base_plate", pos=[0.0, 0.0, 0.0])
    body.add_geom(name="base_plate", type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx / 2, sy / 2, h / 2],
                  pos=[cx, cy, h / 2], rgba=mc["plate_rgba"])
    return {"height_m": h, "plate_size_m": [sx, sy], "plate_centre_m": [cx, cy]}


def relativize_texture(xml: str, texture_dir: Path) -> str:
    """The texture was added with an absolute path (the spec compiles from the source scene's folder); in the
    generated file it must be relative to that file (it is written next to it)."""
    old = f'file="{texture_dir / DESK_TEXTURE}"'
    if old not in xml:
        return xml
    return xml.replace(old, f'file="{DESK_TEXTURE}"')
