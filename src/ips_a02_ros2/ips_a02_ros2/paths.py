"""Chỗ đọc/ghi file hiệu chuẩn `ips_map_transform.yaml`.

File này do ips_calib sinh ra lúc chạy, không có sẵn lúc build. Nếu workspace build bằng
`--symlink-install` thì ghi thẳng vào thư mục config trong `src/` — nếu không, kết quả hiệu
chuẩn nằm trong `install/` và sẽ mất khi `rm -rf install/`.
"""

import os

from ament_index_python.packages import get_package_share_directory

FILENAME = "ips_map_transform.yaml"
_PROBE = "ips_params.yaml"          # file có sẵn lúc build -> dùng để dò kiểu install


def config_dir():
    share = os.path.join(get_package_share_directory("ips_a02_ros2"), "config")
    probe = os.path.join(share, _PROBE)
    if os.path.islink(probe):
        src = os.path.dirname(os.path.realpath(probe))
        if os.path.isdir(src) and os.access(src, os.W_OK):
            return src
    return share


def transform_file():
    return os.path.join(config_dir(), FILENAME)
