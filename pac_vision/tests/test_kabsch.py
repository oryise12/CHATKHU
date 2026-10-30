"""Kabsch(강체 변환) — 합성 R·t 복원과 기존 측정 데이터 재현."""
import numpy as np
import pytest

from pac_vision.cam2base import kabsch, rms


def random_rotation(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@pytest.mark.parametrize("coplanar", [False, True])
def test_recovers_synthetic_transform(coplanar):
    rng = np.random.default_rng(1)
    R, t = random_rotation(rng), rng.normal(size=3)
    a = rng.uniform(-0.5, 0.5, size=(5, 3))
    if coplanar:                       # 캘리브레이션처럼 5점이 한 평면 (z=0)
        a[:, 2] = 0.0
    b = a @ R.T + t
    R2, t2 = kabsch(a, b)
    assert np.linalg.det(R2) == pytest.approx(1.0)
    assert np.allclose(R2, R, atol=1e-9)
    assert np.allclose(t2, t, atol=1e-9)


# 2026-07-25 측정 데이터 (README 7절): 기준 좌표, 카메라 좌표
P_BASE = np.array([[0.98, -0.485, 0.805], [-0.34, 0.46, 0.50], [-0.628, 0.00, 0.00], [0.0, 0.361, 0.00],
                   [0.40, -0.764, 0.78], [0.60, -0.41, 0.0], [0.57, 0.584, 0.00], [1.718, 0.226, 0.17],
                   [-0.312, -0.422, 0.10]])
P_CAM = np.array([[0.230, -0.750, 2.623], [-0.639, -0.092, 1.375], [-0.215, 0.470, 1.306],
                  [-0.627, 0.241, 1.860], [0.506, -0.540, 2.121], [0.155, 0.266, 1.919],
                  [-0.903, 0.033, 2.360], [-0.604, -0.449, 3.552], [0.194, 0.295, 1.599]])


def fit_rms_cm(exclude):
    keep = [i for i in range(9) if i + 1 not in exclude]
    R, t = kabsch(P_CAM[keep], P_BASE[keep])
    err = np.linalg.norm(P_CAM[keep] @ R.T + t - P_BASE[keep], axis=1)
    return R, t, rms(err) * 100


def test_reproduces_recorded_calibration():
    R, t, rms_cm = fit_rms_cm(exclude={6, 8})
    print(f"\n7점 RMS {rms_cm:.2f} cm")
    assert round(rms_cm, 2) == 2.26
    assert np.allclose(R, [[-0.08364, -0.31945, 0.9439], [-0.99135, -0.0695, -0.11137],
                           [0.10118, -0.94505, -0.31088]], atol=1e-5)
    assert np.allclose(t, [-1.72318, -0.03662, 0.87503], atol=1e-5)
    assert round(fit_rms_cm(set())[2], 2) == 19.38
    assert round(fit_rms_cm({6})[2], 2) == 4.17
    assert round(fit_rms_cm({3, 6, 8})[2], 2) == 2.42


def test_reproduces_recorded_holdout():
    actual = np.array([[0.30, -0.40, 0], [-0.20, -0.20, 0], [0.26, 0.34, 0.16]])
    output = np.array([[0.316, -0.405, 0.098], [-0.193, -0.262, 0.054], [0.334, 0.354, 0.131]])
    err3 = np.linalg.norm(output - actual, axis=1)
    err2 = np.linalg.norm((output - actual)[:, :2], axis=1)
    assert round(rms(err3) * 100, 2) == 8.80
    assert round(rms(err2) * 100, 2) == 5.73
