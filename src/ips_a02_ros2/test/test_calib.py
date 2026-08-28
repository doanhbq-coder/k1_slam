"""Kiểm tra rigid_2d bằng dữ liệu tự sinh: quay + tịnh tiến đã biết, có nhiễu."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ips_a02_ros2.calib import apply_2d, residuals, rigid_2d  # noqa: E402


def _make(theta, tx, ty, n=6, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    src = rng.uniform(-5, 5, size=(n, 2))
    dst = np.array([apply_2d(p, theta, tx, ty) for p in src])
    return src, dst + rng.normal(0, noise, dst.shape)


def test_khong_nhieu():
    th, tx, ty = math.radians(30), -2.4, 7.9
    s, d = _make(th, tx, ty)
    e_th, e_tx, e_ty, rms = rigid_2d(s, d)
    assert abs(e_th - th) < 1e-9 and abs(e_tx - tx) < 1e-9 and abs(e_ty - ty) < 1e-9
    assert rms < 1e-9


def test_co_nhieu():
    th, tx, ty = math.radians(-115), 3.0, -1.25
    s, d = _make(th, tx, ty, n=8, noise=0.05, seed=7)
    e_th, e_tx, e_ty, rms = rigid_2d(s, d)
    assert abs(e_th - th) < math.radians(2.0)
    assert math.hypot(e_tx - tx, e_ty - ty) < 0.1
    assert rms < 0.12


def test_hai_diem_va_loi():
    s, d = _make(math.radians(90), 1.0, 2.0, n=2)
    assert abs(rigid_2d(s, d)[0] - math.radians(90)) < 1e-9
    for bad in (([[0, 0]], [[1, 1]]), ([[0, 0], [1, 1]], [[0, 0]])):
        try:
            rigid_2d(*bad)
        except ValueError:
            continue
        raise AssertionError("phải ném ValueError")


def test_khong_lat_truc():
    """Dữ liệu bị lật gương -> vẫn phải trả phép quay thuần (det = +1), rms lớn để người dùng biết."""
    s, _ = _make(0.0, 0.0, 0.0, n=5)
    d = s.copy()
    d[:, 1] *= -1.0                      # lật trục y
    _, _, _, rms = rigid_2d(s, d)
    assert rms > 0.5


def test_residuals_chi_ra_diem_hong():
    th, tx, ty = math.radians(45), 1.0, 1.0
    s, d = _make(th, tx, ty, n=5)
    d[2] += np.array([1.0, 0.0])         # một điểm chụp sai 1 m
    e = rigid_2d(s, d)
    r = residuals(s, d, *e[:3])
    assert r.index(max(r)) == 2


if __name__ == "__main__":
    for fn in [v for k, v in sorted(globals().items()) if k.startswith("test_")]:
        fn()
        print("PASS", fn.__name__)
