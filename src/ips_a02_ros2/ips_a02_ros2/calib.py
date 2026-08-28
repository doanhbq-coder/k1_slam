"""Khớp hệ toạ độ IPS vào frame `map` của Nav2.

Hai hệ chỉ lệch nhau một phép quay + tịnh tiến trong mặt phẳng (3 tham số: theta, tx, ty).
Không có scale (cả hai đều là mét thật) và không có lật trục — nếu tính ra det < 0 thì
chắc chắn dữ liệu nhập sai. Dùng Umeyama/Kabsch 2D, có chặn phản xạ gương.
"""

import math

import numpy as np


def rigid_2d(src, dst):
    """src, dst: (N,2) — trả (theta, tx, ty, rms) sao cho R(theta) @ src + t ≈ dst."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    if src.ndim != 2 or src.shape[1] != 2 or src.shape != dst.shape:
        raise ValueError("src và dst phải cùng dạng (N,2)")
    if len(src) < 2:
        raise ValueError("cần ít nhất 2 cặp điểm")
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))          # chặn phản xạ gương
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = cd - R @ cs
    res = (src @ R.T + t) - dst
    rms = float(np.sqrt((res ** 2).sum(1).mean()))
    return math.atan2(R[1, 0], R[0, 0]), float(t[0]), float(t[1]), rms


def residuals(src, dst, theta, tx, ty):
    """Sai số từng điểm (m) — để chỉ ra điểm nào chụp hỏng."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return np.linalg.norm(src @ R.T + np.array([tx, ty]) - dst, axis=1).tolist()


def apply_2d(pt, theta, tx, ty):
    """Đưa một điểm từ hệ IPS sang hệ map."""
    c, s = math.cos(theta), math.sin(theta)
    return (c * pt[0] - s * pt[1] + tx, s * pt[0] + c * pt[1] + ty)
