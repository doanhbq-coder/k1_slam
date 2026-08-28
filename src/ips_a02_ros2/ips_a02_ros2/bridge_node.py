"""ips_bridge — Nooploop IPS Tool -> ROS2.

Nhận message định vị theo MQTT / UDP / Webhook (chọn bằng tham số `source`), publish:

    /ips/<tag>/pose            geometry_msgs/PoseStamped
    /ips/<tag>/pose_cov        geometry_msgs/PoseWithCovarianceStamped   (covariance từ posVar)
    /ips/<tag>/twist           geometry_msgs/TwistStamped
    /ips/tag_poses             geometry_msgs/PoseArray      (tất cả tag, tiện xem nhanh)
    /ips/tags /ips/anchors /ips/fences   visualization_msgs/MarkerArray  (hiển thị RViz2)
    /ips/events                ips_a02_msgs/IpsEvent  (nút bấm trên tag: Call/CancelCall/Ack)
    /diagnostics               diagnostic_msgs/DiagnosticArray (Hz, mất gói, nhiễu, tuổi mẫu)
    TF: map -> <tag_frame_prefix><tag>   và map -> base_link cho tag gắn trên robot

Toạ độ lấy thẳng từ IPS Tool (mét), trục z là độ cao — trùng quy ước REP-103 nên không cần xoay.
"""

import math
import re
import threading
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (PoseArray, PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped, TwistStamped)
from ips_a02_msgs.msg import IpsEvent
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from . import markers as MK
from .openapi import IpsOpenApi
from .transports import make_transport

SAFE = re.compile(r"[^0-9a-zA-Z_]")


class TagState:
    __slots__ = ("name", "uid", "x", "y", "z", "vx", "vy", "vz", "noise", "var", "rssi",
                 "fences", "cnt", "dev_time", "last", "first", "count", "gaps", "rate",
                 "trail", "yaw", "online", "pubs")

    def __init__(self, name, uid, trail_len):
        self.name, self.uid = name, uid
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.noise = 0.0
        self.var = [0.0, 0.0, 0.0]
        self.rssi = 0.0
        self.fences = []
        self.cnt = None
        self.dev_time = 0
        self.last = 0.0
        self.first = time.time()
        self.count = 0
        self.gaps = 0
        self.rate = 0.0
        self.yaw = 0.0
        self.online = False
        self.trail = deque(maxlen=trail_len)
        self.pubs = {}

    def update(self, data, now, use_opt, yaw_from_vel):
        pos = data.get("pos") or [0.0, 0.0, 0.0]
        if use_opt:
            opt = data.get("optPoses")
            if isinstance(opt, list) and opt and isinstance(opt[0], list):
                pos = opt[0]
        self.x, self.y = float(pos[0]), float(pos[1])
        self.z = float(pos[2]) if len(pos) > 2 else float(data.get("z", 0.0))
        v = data.get("vel") or [0.0, 0.0, 0.0]
        self.vx, self.vy, self.vz = float(v[0]), float(v[1]), float(v[2] if len(v) > 2 else 0.0)
        self.noise = float(data.get("noiseRadius", 0.0) or 0.0)
        pv = data.get("posVar") or []
        if len(pv) >= 3:
            self.var = [float(pv[0]), float(pv[1]), float(pv[2])]
        elif self.noise > 0:
            self.var = [self.noise ** 2] * 3
        self.rssi = float(data.get("rxRssi", 0.0) or 0.0)
        self.fences = [f.get("name", "") for f in (data.get("fences") or []) if isinstance(f, dict)]
        t = data.get("time")
        if isinstance(t, (int, float)) and t > 0:
            self.dev_time = int(t)

        cnt = data.get("cnt")
        if isinstance(cnt, (int, float)):
            cnt = int(cnt)
            if self.cnt is not None:
                d = cnt - self.cnt
                if 1 < d < 500:
                    self.gaps += d - 1
            self.cnt = cnt

        if self.last:
            dt = now - self.last
            if 0.001 < dt < 5.0:
                inst = 1.0 / dt
                self.rate = inst if self.rate == 0 else self.rate * 0.8 + inst * 0.2
        self.last = now
        self.count += 1
        self.trail.append((self.x, self.y, self.z))

        if yaw_from_vel and math.hypot(self.vx, self.vy) > 0.15:
            self.yaw = math.atan2(self.vy, self.vx)


class IpsBridge(Node):
    def __init__(self):
        super().__init__("ips_bridge")
        d = self.declare_parameter
        d("source", "udp")                       # udp | mqtt | webhook
        d("udp.port", 9600)
        d("udp.bind", "0.0.0.0")
        d("mqtt.broker", "127.0.0.1")
        d("mqtt.port", 1883)
        d("mqtt.username", "")
        d("mqtt.password", "")
        d("mqtt.topics", ["ips/#"])
        d("mqtt.topic", "")          # dạng chuỗi (ngăn cách bằng dấu phẩy), ưu tiên hơn mqtt.topics
        d("mqtt.client_id", "ips_a02_ros2")
        d("webhook.port", 8091)
        d("webhook.path", "/ingest")

        d("frame_id", "map")
        d("tag_frame_prefix", "ips_")
        d("robot_tag", "")                       # tag gắn trên robot -> TF map->base_link
        d("base_frame", "base_link")
        d("publish_tf", True)
        d("use_opt_poses", False)                # dùng optPoses (đã ràng buộc tường) thay pos raw
        d("yaw_from_velocity", True)
        d("max_noise", 0.0)                      # >0: bỏ mẫu có noiseRadius lớn hơn
        d("offline_after", 3.0)
        d("event_kinds", ["Call", "CancelCall", "Ack"])   # loại sự kiện đẩy ra /ips/events
        d("trail_length", 300)
        d("marker_rate", 10.0)
        d("tag_size", 0.28)
        d("velocity_scale", 0.7)
        d("show_trail", True)
        d("show_noise", True)
        d("show_velocity", True)
        d("show_coords", False)        # thêm dòng toạ độ vào nhãn tag trong RViz
        d("label_size", 0.26)

        d("openapi.enabled", True)
        d("openapi.base_url", "http://192.168.68.142:8080")
        d("openapi.poll_period", 30.0)

        g = lambda n: self.get_parameter(n).value
        self.frame = g("frame_id")
        self.prefix = g("tag_frame_prefix")
        self.robot_tag = g("robot_tag")
        self.base_frame = g("base_frame")
        self.pub_tf = g("publish_tf")
        self.use_opt = g("use_opt_poses")
        self.yaw_vel = g("yaw_from_velocity")
        self.max_noise = float(g("max_noise"))
        self.event_kinds = set(g("event_kinds") or [])
        self.offline_after = float(g("offline_after"))
        self.trail_len = int(g("trail_length"))
        self.mopts = {"tag_size": float(g("tag_size")), "vel_scale": float(g("velocity_scale")),
                      "show_trail": g("show_trail"), "show_noise": g("show_noise"),
                      "show_velocity": g("show_velocity"), "show_coords": g("show_coords"),
                      "label_size": float(g("label_size"))}

        self.tags = {}
        self.order = []          # thứ tự ổn định -> màu & marker id không nhảy
        self.lock = threading.Lock()
        self.dropped = 0
        self.anchors, self.fences = [], []

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_tags = self.create_publisher(MarkerArray, "ips/tags", 1)
        self.pub_anchors = self.create_publisher(MarkerArray, "ips/anchors", latched)
        self.pub_fences = self.create_publisher(MarkerArray, "ips/fences", latched)
        self.pub_poses = self.create_publisher(PoseArray, "ips/tag_poses", 1)
        self.pub_diag = self.create_publisher(DiagnosticArray, "diagnostics", 1)
        # depth 10, RELIABLE: nút bấm là sự kiện rời rạc, mất một cái là mất hẳn một lệnh gọi
        self.pub_events = self.create_publisher(IpsEvent, "ips/events", 10)
        self.tf = TransformBroadcaster(self) if self.pub_tf else None

        # --- transport theo lựa chọn ---
        src = g("source")
        cfg = {"udp": {"port": g("udp.port"), "bind": g("udp.bind")},
               "mqtt": {"broker": g("mqtt.broker"), "port": int(g("mqtt.port")),
                        "username": g("mqtt.username"), "password": g("mqtt.password"),
                        "topics": ([t.strip() for t in g("mqtt.topic").split(",") if t.strip()]
                                   or list(g("mqtt.topics"))),
                        "client_id": g("mqtt.client_id")},
               "webhook": {"port": g("webhook.port"), "path": g("webhook.path")}}[src]
        self.transport = make_transport(src, cfg, self.get_logger())
        self.transport.start(self.on_ips_message)
        self.get_logger().info(f"nguồn dữ liệu: {src.upper()}  |  frame: {self.frame}")
        self._log_howto(src, cfg)

        self.create_timer(1.0 / max(1.0, float(g("marker_rate"))), self.publish_markers)
        self.create_timer(1.0, self.publish_diagnostics)
        if g("openapi.enabled"):
            self.api = IpsOpenApi(g("openapi.base_url"))
            self.create_timer(float(g("openapi.poll_period")), self.poll_openapi)
            threading.Thread(target=self.poll_openapi, daemon=True).start()

    def _log_howto(self, src, cfg):
        import socket as _s
        ip = "<ip-máy-này>"
        try:
            k = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
            k.connect(("8.8.8.8", 80))
            ip = k.getsockname()[0]
            k.close()
        except Exception:
            pass
        how = {"udp": f"UDP     address = {ip}:{cfg.get('port')}  (nhớ điền localAddress = IP card Wi-Fi của PC IPS Tool)",
               "mqtt": f"MQTT    address = mqtt://{cfg.get('broker')}:{cfg.get('port')}  topics = {cfg.get('topics')}",
               "webhook": f"Webhook URL = http://{ip}:{cfg.get('port')}{cfg.get('path')}  (POST)"}[src]
        self.get_logger().info("điền vào IPS Tool (cấu hình API): " + how)

    # ------------------------------------------------------------------ nhận dữ liệu
    def on_ips_message(self, obj, meta=""):
        name = obj.get("name")
        if name != "Pos":
            if name in self.event_kinds:
                self.publish_event(obj, name)
            elif name:
                self.get_logger().debug(f"bỏ qua message lạ: {name}")
            return
        data = obj.get("data") or {}
        if self.max_noise > 0 and float(data.get("noiseRadius", 0.0) or 0.0) > self.max_noise:
            self.dropped += 1
            if self.dropped % 50 == 1:
                self.get_logger().warn(
                    f"bỏ mẫu nhiễu cao (> {self.max_noise} m), tổng {self.dropped}")
            return

        uid = obj.get("uid") or obj.get("deviceName") or "?"
        dev = obj.get("deviceName") or uid
        now = time.time()
        with self.lock:
            t = self.tags.get(uid)
            if t is None:
                t = TagState(dev, uid, self.trail_len)
                self.tags[uid] = t
                self.order.append(uid)
                self.get_logger().info(f"phát hiện tag mới: {dev} ({uid})")
            t.name = dev
            t.update(data, now, self.use_opt, self.yaw_vel)
        self.publish_tag(t)

    def publish_event(self, obj, kind):
        """Nút bấm trên tag. Đính kèm luôn vị trí tag ngay lúc này: Call và Pos là hai message
        khác nhau trên cùng một luồng, node điều hướng tự đi đọc topic pose thì dễ vớ phải mẫu
        cũ. Đóng gói tại nguồn thì bên nhận có đúng một sự thật để làm việc."""
        uid = obj.get("uid") or obj.get("deviceName") or "?"
        ev = IpsEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.header.frame_id = self.frame
        ev.kind, ev.uid = kind, uid
        ev.device_name = obj.get("deviceName") or uid
        ev.req_id = str((obj.get("data") or {}).get("reqId", ""))
        ev.tag_pose.header = ev.header

        now = time.time()
        with self.lock:
            t = self.tags.get(uid)
            if t is not None:
                ev.device_name = t.name
                ev.tag_pose.pose.position.x = t.x
                ev.tag_pose.pose.position.y = t.y
                ev.tag_pose.pose.position.z = t.z
                ev.tag_pose.pose.orientation.z = math.sin(t.yaw / 2.0)
                ev.tag_pose.pose.orientation.w = math.cos(t.yaw / 2.0)
                ev.pose_valid = True
                ev.pose_age = float(now - t.last)
                ev.pose_noise = float(t.noise)
        if not ev.pose_valid:
            ev.tag_pose.pose.orientation.w = 1.0
        self.pub_events.publish(ev)

        where = (f"({ev.tag_pose.pose.position.x:.2f}, {ev.tag_pose.pose.position.y:.2f}) "
                 f"tuổi {ev.pose_age:.2f}s nhiễu {ev.pose_noise:.2f}m"
                 if ev.pose_valid else "CHƯA CÓ VỊ TRÍ (chưa nhận Pos nào của tag này)")
        self.get_logger().info(f"[{kind}] tag {ev.device_name} ({uid}) — {where}")

    # ------------------------------------------------------------------ publish theo tag
    def publish_tag(self, t):
        key = SAFE.sub("_", t.name)
        if not t.pubs:
            t.pubs = {
                "pose": self.create_publisher(PoseStamped, f"ips/{key}/pose", qos_profile_sensor_data),
                "cov": self.create_publisher(PoseWithCovarianceStamped, f"ips/{key}/pose_cov",
                                             qos_profile_sensor_data),
                "twist": self.create_publisher(TwistStamped, f"ips/{key}/twist",
                                               qos_profile_sensor_data),
            }
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(t.yaw / 2.0), math.cos(t.yaw / 2.0)

        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, self.frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = t.x, t.y, t.z
        p.pose.orientation.z, p.pose.orientation.w = qz, qw
        t.pubs["pose"].publish(p)

        pc = PoseWithCovarianceStamped()
        pc.header = p.header
        pc.pose.pose = p.pose
        cov = [0.0] * 36
        cov[0], cov[7], cov[14] = t.var[0] or 1e-3, t.var[1] or 1e-3, t.var[2] or 1e-3
        cov[21] = cov[28] = 1e6                     # UWB không đo được roll/pitch
        cov[35] = 0.5 if self.yaw_vel else 1e6      # yaw chỉ suy ra từ vận tốc
        pc.pose.covariance = cov
        t.pubs["cov"].publish(pc)

        tw = TwistStamped()
        tw.header = p.header
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = t.vx, t.vy, t.vz
        t.pubs["twist"].publish(tw)

        if self.tf:
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id = stamp, self.frame
            tr.child_frame_id = f"{self.prefix}{key}"
            tr.transform.translation.x, tr.transform.translation.y = t.x, t.y
            tr.transform.translation.z = t.z
            tr.transform.rotation.z, tr.transform.rotation.w = qz, qw
            out = [tr]
            if self.robot_tag and t.name == self.robot_tag:
                rb = TransformStamped()
                rb.header.stamp, rb.header.frame_id = stamp, self.frame
                rb.child_frame_id = self.base_frame
                rb.transform = tr.transform
                out.append(rb)
            self.tf.sendTransform(out)

    # ------------------------------------------------------------------ hiển thị
    def publish_markers(self):
        now = time.time()
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            ordered = [self.tags[u] for u in self.order if u in self.tags]
            for t in ordered:
                t.online = (now - t.last) < self.offline_after
        if not ordered:
            return
        self.pub_tags.publish(MK.tag_markers(ordered, self.frame, stamp, self.mopts))

        pa = PoseArray()
        pa.header.stamp, pa.header.frame_id = stamp, self.frame
        for t in ordered:
            p = PoseStamped().pose
            p.position.x, p.position.y, p.position.z = t.x, t.y, t.z
            p.orientation.z, p.orientation.w = math.sin(t.yaw / 2), math.cos(t.yaw / 2)
            pa.poses.append(p)
        self.pub_poses.publish(pa)

    def publish_diagnostics(self):
        now = time.time()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus(name="ips_bridge: transport", hardware_id="ips_a02")
        s = self.transport.status()
        st.level = DiagnosticStatus.OK if s["ready"] and s["count"] else DiagnosticStatus.WARN
        st.message = (f"{s['transport'].upper()} — {s['count']} message"
                      if s["ready"] else f"{s['transport'].upper()} chưa sẵn sàng: {s['error']}")
        st.values = [KeyValue(key=k, value=str(v)) for k, v in s.items()]
        arr.status.append(st)

        with self.lock:
            ordered = [self.tags[u] for u in self.order if u in self.tags]
        for t in ordered:
            age = now - t.last
            loss = 100.0 * t.gaps / max(1, t.gaps + t.count)
            d = DiagnosticStatus(name=f"ips_bridge: tag {t.name}", hardware_id=t.uid)
            if age > self.offline_after:
                d.level, d.message = DiagnosticStatus.ERROR, f"mất tín hiệu {age:.1f}s"
            elif loss > 5.0 or t.noise > 0.3:
                d.level, d.message = DiagnosticStatus.WARN, f"mất {loss:.1f}%, nhiễu {t.noise:.2f}m"
            else:
                d.level, d.message = DiagnosticStatus.OK, f"{t.rate:.1f} Hz"
            d.values = [
                KeyValue(key="x", value=f"{t.x:.3f}"), KeyValue(key="y", value=f"{t.y:.3f}"),
                KeyValue(key="z", value=f"{t.z:.3f}"), KeyValue(key="rate_hz", value=f"{t.rate:.2f}"),
                KeyValue(key="noise_m", value=f"{t.noise:.3f}"), KeyValue(key="rssi_dbm", value=f"{t.rssi:.1f}"),
                KeyValue(key="packets", value=str(t.count)), KeyValue(key="cnt_gaps", value=str(t.gaps)),
                KeyValue(key="loss_pct", value=f"{loss:.2f}"), KeyValue(key="age_s", value=f"{age:.2f}"),
                KeyValue(key="fences", value=",".join(t.fences) or "-"),
            ]
            arr.status.append(d)
        self.pub_diag.publish(arr)

    def poll_openapi(self):
        stamp = self.get_clock().now().to_msg()
        try:
            an = self.api.anchors()
            if an != self.anchors:
                self.anchors = an
                self.pub_anchors.publish(MK.anchor_markers(an, self.frame, stamp))
                self.get_logger().info(f"nạp {len(an)} anchor từ IPS Tool")
        except Exception as e:
            self.get_logger().warn(f"không đọc được anchor qua OpenAPI: {e}")
        try:
            fc = self.api.fences()
            if fc != self.fences:
                self.fences = fc
                self.pub_fences.publish(MK.fence_markers(fc, self.frame, stamp))
                if fc:
                    self.get_logger().info(f"nạp {len(fc)} vùng ảo từ IPS Tool")
        except Exception as e:
            self.get_logger().debug(f"không đọc được fence: {e}")

    def destroy_node(self):
        try:
            self.transport.stop()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = IpsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
