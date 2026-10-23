"""Monitor pixels <-> world through a calibrated `CameraModel` (OpenCV conventions).

The same functions work for the sim camera (model derived from MuJoCo) and a real calibrated
camera. Pixel (u, v) is in the camera image (integer = pixel centre); the window may show it
scaled, the caller divides by the display scale first. No pixel-to-metre scaling is assumed.

    ray(u, v):        o = camera centre, r = R_cw^T * undistort(u, v)  (unit)
    plane z = z_i:    o + t r,  t = (z_i - o_z) / r_z,  t > 0
    vertical line through (x, y), closest to the ray:
        b = r_z, w0 = (x, y, 0) - o,  den = 1 - b^2,
        z = (b (r.w0) - w0_z) / den,  s = ((r.w0) - b w0_z) / den  (s > 0: in front of the camera)
"""
from __future__ import annotations

import numpy as np

from .camera_model import CameraModel


def ray_plane_z(origin, direction, z: float, min_cos: float = 0.05):
    """Intersection with the horizontal plane z, or None if (nearly) parallel or behind the camera."""
    if abs(direction[2]) < min_cos:
        return None
    t = (z - origin[2]) / direction[2]
    if not np.isfinite(t) or t <= 0:
        return None
    return origin + t * direction


def pixel_to_plane(cam: CameraModel, u, v, z, radial_max: float | None = None):
    """World point under pixel (u, v) at height z; None if invalid or outside radial_max (m from base)."""
    o, r = cam.pixel_ray(u, v)
    p = ray_plane_z(o, r, z)
    if p is None or (radial_max is not None and np.hypot(p[0], p[1]) > radial_max):
        return None
    return p


def pixel_to_vertical(cam: CameraModel, u, v, xy, z_range=None, min_sin: float = 0.05):
    """Point (x, y, z) on the vertical line through world (x, y) closest to the ray of pixel (u, v)."""
    o, r = cam.pixel_ray(u, v)
    b = r[2]
    den = 1.0 - b * b
    if den < min_sin ** 2:
        return None
    w0 = np.array([xy[0], xy[1], 0.0]) - o
    e = float(r @ w0)
    t = (b * e - w0[2]) / den
    s = (e - b * w0[2]) / den
    if s <= 0 or not np.isfinite(t) or (z_range is not None and not z_range[0] <= t <= z_range[1]):
        return None
    return np.array([xy[0], xy[1], t])


def pick_index(points, cam: CameraModel, u, v, radius_px: float) -> int | None:
    """Nearest interior path point within radius_px (camera pixels) of (u, v); endpoints are not pickable."""
    uv, front = cam.project(points)
    dist = np.hypot(uv[:, 0] - u, uv[:, 1] - v)
    dist[~front] = np.inf
    dist[[0, -1]] = np.inf
    k = int(np.argmin(dist))
    return k if dist[k] <= radius_px else None
