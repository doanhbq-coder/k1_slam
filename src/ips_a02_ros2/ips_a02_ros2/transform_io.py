"""Đọc / ghi `ips_map_transform.yaml` — dùng chung cho ips_calib, ips_align, ips_map_tf."""

import math
import os
import time

import yaml


def load(path):
    """-> dict(parent_frame, child_frame, x, y, yaw, ...); ném RuntimeError kèm lời khuyên."""
    if not os.path.isfile(path):
        raise RuntimeError(
            f"chưa có file hiệu chuẩn: {path}\n"
            "  Hệ IPS chưa được khớp vào bản đồ. Chạy một trong hai cách:\n"
            "    ros2 run ips_a02_ros2 ips_align                       # bấm 2 điểm trên RViz\n"
            "    ros2 run ips_a02_ros2 ips_calib --ros-args -p calib_tag:=T1   # lái robot\n"
            "  (xem NAV2_INTEGRATION.md mục B1)")
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    node = doc.get("ips_map_tf") or doc.get("/ips_map_tf") or {}
    cfg = node.get("ros__parameters") or {}
    missing = [k for k in ("x", "y", "yaw") if not isinstance(cfg.get(k), (int, float))]
    if missing:
        raise RuntimeError(f"{path} thiếu hoặc sai kiểu trường: {', '.join(missing)}")
    cfg.setdefault("parent_frame", "map")
    cfg.setdefault("child_frame", "ips_map")
    return cfg


def write(path, theta, tx, ty, *, parent="map", child="ips_map", source="", rms=None,
          n_points=None, raw=(), note=""):
    """Ghi file hiệu chuẩn. `raw`: các dòng dữ liệu thô, ghi dạng comment để tính lại được."""
    lines = [
        "# Sinh tự động — ĐỪNG sửa tay (chạy lại ips_align hoặc ips_calib).",
        f"# {time.strftime('%Y-%m-%d %H:%M:%S')} · nguồn: {source or '?'}"
        + (f" · {n_points} điểm" if n_points is not None else "")
        + (f" · rms {rms:.3f} m" if isinstance(rms, (int, float)) else ""),
        "# Hiệu chuẩn lại khi: quét lại bản đồ SLAM · di chuyển anchor · đổi gốc trong IPS Tool.",
    ]
    if note:
        lines += [f"# {ln}" for ln in note.splitlines()]
    lines += [
        "ips_map_tf:",
        "  ros__parameters:",
        f'    parent_frame: "{parent}"',
        f'    child_frame: "{child}"',
        f"    x: {tx:.6f}",
        f"    y: {ty:.6f}",
        f"    yaw: {theta:.6f}          # radian = {math.degrees(theta):.3f} deg",
    ]
    if isinstance(rms, (int, float)):
        lines.append(f"    rms: {rms:.4f}            # chỉ để tham khảo, không dùng lúc chạy")
    if n_points is not None:
        lines.append(f"    n_points: {n_points}")
    lines.append(f'    source: "{source}"')
    if raw:
        lines.append("# Dữ liệu thô, giữ lại để tính lại được:")
        lines += [f"#   {ln}" for ln in raw]
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(tmp, path)          # ghi nguyên tử: đọc dở file đang ghi ra TF sai thì khổ
