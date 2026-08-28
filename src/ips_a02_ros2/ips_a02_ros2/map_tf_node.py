"""ips_map_tf — phát static TF `map -> ips_map` từ kết quả hiệu chuẩn (ips_align / ips_calib).

Đọc config/ips_map_transform.yaml. Chưa hiệu chuẩn thì báo lỗi rõ ràng rồi thoát,
KHÔNG phát TF identity: im lặng sai còn tệ hơn chết ồn ào — robot sẽ chạy tới nhầm chỗ.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from . import transform_io
from .paths import transform_file


class IpsMapTf(Node):
    def __init__(self):
        super().__init__("ips_map_tf")
        self.declare_parameter("transform_file", "")   # để trống = file trong share
        path = self.get_parameter("transform_file").value or transform_file()
        cfg = transform_io.load(path)                     # lỗi -> ném ra main, thoát mã 1

        yaw = float(cfg["yaw"])
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id = str(cfg["parent_frame"])
        tr.child_frame_id = str(cfg["child_frame"])
        tr.transform.translation.x = float(cfg["x"])
        tr.transform.translation.y = float(cfg["y"])
        tr.transform.translation.z = 0.0               # bản đồ Nav2 là 2D
        tr.transform.rotation.z = math.sin(yaw / 2.0)
        tr.transform.rotation.w = math.cos(yaw / 2.0)

        self.bc = StaticTransformBroadcaster(self)     # giữ tham chiếu, không thì TF biến mất
        self.bc.sendTransform(tr)

        rms = cfg.get("rms")
        extra = f"  (hiệu chuẩn {cfg.get('n_points', '?')} điểm, rms {rms:.3f} m)" if \
            isinstance(rms, (int, float)) else ""
        self.get_logger().info(
            f"static TF {tr.header.frame_id} -> {tr.child_frame_id}: "
            f"x={tr.transform.translation.x:.3f} y={tr.transform.translation.y:.3f} "
            f"yaw={math.degrees(yaw):.2f}°{extra}")
        if isinstance(rms, (int, float)) and rms > 0.3:
            self.get_logger().warn(
                f"rms {rms:.3f} m là khá lớn — vị trí tag trên bản đồ sẽ lệch tương ứng. "
                "Nên hiệu chuẩn lại với các điểm trải rộng hơn.")


def main():
    rclpy.init()
    try:
        node = IpsMapTf()
    except RuntimeError as e:
        print(f"[ips_map_tf] LỖI: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
