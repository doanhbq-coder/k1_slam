"""ips_calib — hiệu chuẩn hệ toạ độ IPS vào bản đồ Nav2 (chạy một lần, khi lắp xong anchor).

Cách dùng: gắn một tag lên robot, lái robot tới vài điểm trải rộng trong vùng phủ anchor,
mỗi điểm dừng hẳn vài giây rồi gọi service `capture`. Node ghép cặp
(vị trí tag trong hệ IPS, vị trí robot trong `map` lấy từ TF) rồi giải Umeyama 2D.

    ros2 run ips_a02_ros2 ips_calib --ros-args -p calib_tag:=T1
    ros2 service call /ips_calib/capture std_srvs/srv/Trigger    # lặp lại ở >= 4 điểm
    ros2 service call /ips_calib/compute std_srvs/srv/Trigger

Không có tag rảnh để gắn lên robot thì dùng đường thủ công: bấm `Publish Point` trong RViz
vào đúng chân từng anchor, rồi nạp các cặp bằng tham số `manual_pairs`
(phẳng: ips_x, ips_y, map_x, map_y, ips_x, ...) và gọi `compute`.
"""

import math
import os
import statistics
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from . import transform_io
from .calib import residuals, rigid_2d
from .paths import transform_file


class IpsCalib(Node):
    def __init__(self):
        super().__init__("ips_calib")
        defaults = {}

        def d(name, value):
            defaults[name] = value
            self.declare_parameter(name, value)

        d("calib_tag", "T1")            # tên tag gắn trên robot (đúng deviceName trong IPS Tool)
        d("pose_topic", "")             # để trống = ips/<calib_tag>/pose
        d("map_frame", "map")
        d("base_frame", "base_footprint")
        d("settle_time", 2.0)           # cửa sổ gom mẫu, tính ngược từ lúc gọi capture
        d("min_samples", 8)             # số mẫu IPS tối thiểu trong cửa sổ
        d("still_threshold", 0.05)      # robot phải đứng yên: biên độ TF trong cửa sổ (m)
        d("min_points", 3)
        d("max_rms", 0.5)               # rms lớn hơn ngần này = hỏng, từ chối ghi file
        d("manual_pairs", [])           # đường thủ công: [ips_x, ips_y, map_x, map_y, ...]
        d("output", "")                 # để trống = config/ips_map_transform.yaml trong share

        # ROS 2 không suy được kiểu của LIST RỖNG trong file yaml: `allowed_tags: []` làm tham số
        # thành NOT_SET và get_parameter() ném ParameterUninitializedException ngay lúc khởi động.
        # Nên đọc tham số qua hàm này, hỏng thì lùi về đúng giá trị mặc định đã khai.
        def g(name):
            try:
                v = self.get_parameter(name).value
            except Exception:
                return defaults.get(name)
            return defaults.get(name) if v is None else v

        self._get = g
        self.tag = g("calib_tag")
        self.map_frame, self.base_frame = g("map_frame"), g("base_frame")
        self.settle = float(g("settle_time"))
        self.min_samples = int(g("min_samples"))
        self.still = float(g("still_threshold"))
        self.out = g("output") or transform_file()

        topic = g("pose_topic") or f"ips/{self.tag}/pose"
        self.ips_buf = deque(maxlen=600)      # (t, x, y) trong hệ IPS
        self.map_buf = deque(maxlen=600)      # (t, x, y) trong hệ map, lấy từ TF
        self.pairs = []                       # [(ips_x, ips_y, map_x, map_y)]

        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self)
        self.create_subscription(PoseStamped, topic, self.on_pose, qos_profile_sensor_data)
        self.create_timer(0.1, self.sample_tf)
        for name, cb in (("capture", self.srv_capture), ("compute", self.srv_compute),
                         ("reset", self.srv_reset), ("undo", self.srv_undo),
                         ("list", self.srv_list)):
            self.create_service(Trigger, f"~/{name}", cb)

        self.get_logger().info(
            f"hiệu chuẩn tag '{self.tag}' — nghe {topic}, TF {self.map_frame}->{self.base_frame}\n"
            f"  lái robot tới điểm 1, dừng hẳn {self.settle:.0f}s rồi:\n"
            f"  ros2 service call /ips_calib/capture std_srvs/srv/Trigger")

    # ------------------------------------------------------------------ thu mẫu
    # Cửa sổ cắt theo timestamp TRONG message, không theo lúc nhận: TransformListener trả
    # "mới nhất" có thể trễ vài trăm ms, tính theo giờ nhận sẽ vớ phải mẫu của điểm trước.
    @staticmethod
    def _stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        self.ips_buf.append((self._stamp(msg.header), msg.pose.position.x, msg.pose.position.y))

    def sample_tf(self):
        try:
            tr = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        t = self._stamp(tr.header)
        if self.map_buf and self.map_buf[-1][0] == t:
            return                                    # lookup trả lại đúng mẫu cũ -> bỏ
        self.map_buf.append((t, tr.transform.translation.x, tr.transform.translation.y))

    def _window(self, buf):
        t0 = self._now() - self.settle
        return [(x, y) for t, x, y in buf if t >= t0]

    # ------------------------------------------------------------------ service
    def srv_capture(self, _req, res):
        ips = self._window(self.ips_buf)
        mp = self._window(self.map_buf)
        if len(ips) < self.min_samples:
            res.success, res.message = False, (
                f"chỉ có {len(ips)} mẫu IPS trong {self.settle:.1f}s (cần {self.min_samples}). "
                f"Tag '{self.tag}' có đang phát không? Kiểm tra: ros2 topic hz /ips/{self.tag}/pose")
            return res
        if len(mp) < 2:
            age = self._now() - self.map_buf[-1][0] if self.map_buf else None
            res.success, res.message = False, (
                f"TF {self.map_frame}->{self.base_frame} cũ {age:.1f}s, không dùng được"
                if age is not None else
                f"không đọc được TF {self.map_frame}->{self.base_frame}. "
                "Nav2/AMCL đã chạy và robot đã localize chưa?")
            return res

        spread = max(math.hypot(x - mp[0][0], y - mp[0][1]) for x, y in mp)
        if spread > self.still:
            res.success, res.message = False, (
                f"robot đang di chuyển ({spread:.3f} m > {self.still:.3f} m trong cửa sổ). "
                "Dừng hẳn rồi chụp lại.")
            return res

        pair = (statistics.median(p[0] for p in ips), statistics.median(p[1] for p in ips),
                statistics.median(p[0] for p in mp), statistics.median(p[1] for p in mp))
        self.pairs.append(pair)
        res.success = True
        res.message = (f"điểm #{len(self.pairs)}: ips=({pair[0]:.3f}, {pair[1]:.3f})  "
                       f"map=({pair[2]:.3f}, {pair[3]:.3f})  [{len(ips)} mẫu IPS]")
        self.get_logger().info(res.message)
        return res

    def _all_pairs(self):
        """Cặp đã chụp + cặp nhập tay qua tham số manual_pairs (đọc lại tại thời điểm gọi)."""
        out = list(self.pairs)
        raw = list(self._get("manual_pairs") or [])
        if len(raw) % 4:
            raise ValueError(f"manual_pairs có {len(raw)} số, phải chia hết cho 4")
        out += [tuple(float(v) for v in raw[i:i + 4]) for i in range(0, len(raw), 4)]
        return out

    def srv_compute(self, _req, res):
        try:
            pairs = self._all_pairs()
        except ValueError as e:
            res.success, res.message = False, str(e)
            return res
        n_min = int(self._get("min_points"))
        if len(pairs) < n_min:
            res.success, res.message = False, f"mới có {len(pairs)} điểm, cần ít nhất {n_min}"
            return res

        src = [(p[0], p[1]) for p in pairs]
        dst = [(p[2], p[3]) for p in pairs]
        theta, tx, ty, rms = rigid_2d(src, dst)
        per = residuals(src, dst, theta, tx, ty)
        detail = "  ".join(f"#{i + 1}:{r:.3f}" for i, r in enumerate(per))
        head = (f"n={len(pairs)}  yaw={math.degrees(theta):.2f}°  "
                f"tx={tx:.3f}  ty={ty:.3f}  rms={rms:.3f} m")
        self.get_logger().info(f"{head}\n  sai số từng điểm (m): {detail}")

        max_rms = float(self._get("max_rms"))
        if rms > max_rms:
            worst = per.index(max(per)) + 1
            res.success, res.message = False, (
                f"{head} — rms vượt ngưỡng {max_rms} m, KHÔNG ghi file. "
                f"Điểm #{worst} lệch nhiều nhất ({max(per):.3f} m), bỏ nó bằng `undo` "
                "hoặc chụp lại các điểm cho trải rộng hơn.")
            self.get_logger().error(res.message)
            return res

        try:
            self._write(theta, tx, ty, rms, pairs, per)
        except OSError as e:
            res.success, res.message = False, f"không ghi được {self.out}: {e}"
            return res
        res.success = True
        res.message = (f"{head}\nđã ghi {os.path.realpath(self.out)}\n"
                       "chạy: ros2 run ips_a02_ros2 ips_map_tf")
        self.get_logger().info(res.message)
        return res

    def _write(self, theta, tx, ty, rms, pairs, per):
        raw = ["ips_x ips_y map_x map_y | sai số (m)"] + [
            f"{p[0]:8.3f} {p[1]:8.3f} {p[2]:8.3f} {p[3]:8.3f} | {r:.3f}"
            for p, r in zip(pairs, per)]
        transform_io.write(self.out, theta, tx, ty, parent=self.map_frame, child="ips_map",
                           source=f"ips_calib tag '{self.tag}'", rms=rms, n_points=len(pairs),
                           raw=raw)

    def srv_reset(self, _req, res):
        n, self.pairs = len(self.pairs), []
        res.success, res.message = True, f"đã xoá {n} điểm"
        return res

    def srv_undo(self, _req, res):
        if not self.pairs:
            res.success, res.message = False, "chưa có điểm nào"
            return res
        p = self.pairs.pop()
        res.success = True
        res.message = f"đã bỏ điểm cuối: ips=({p[0]:.3f}, {p[1]:.3f}) map=({p[2]:.3f}, {p[3]:.3f})"
        return res

    def srv_list(self, _req, res):
        try:
            pairs = self._all_pairs()
        except ValueError as e:
            res.success, res.message = False, str(e)
            return res
        res.success = True
        res.message = f"{len(pairs)} điểm:\n" + "\n".join(
            f"  #{i + 1}  ips=({p[0]:8.3f}, {p[1]:8.3f})  map=({p[2]:8.3f}, {p[3]:8.3f})"
            for i, p in enumerate(pairs))
        return res


def main():
    rclpy.init()
    node = IpsCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
