"""ips_align — khớp hệ IPS vào bản đồ bằng mắt, ngay trong RViz.

Cách dùng: bấm `Publish Point` vào đúng chân 2 anchor trên bản đồ (node chỉ rõ bấm anchor nào
trước, anchor nào sau). Hai điểm là đủ để xác định trọn vẹn phép quay + tịnh tiến, nên ngay sau
click thứ hai cả 4 anchor hiện lên đúng chỗ. Chưa vừa ý thì kéo / xoay khung bằng interactive
marker (mũi tên tròn + mặt phẳng ở gốc hệ IPS), nhìn 4 anchor chạy theo thời gian thực. Ưng thì:

    ros2 service call /ips_align/save std_srvs/srv/Trigger

hoặc chuột phải vào interactive marker -> "Lưu vào file".

Khác `ips_calib` (lái robot, chụp N điểm): cách này khớp theo **vị trí vật lý của anchor vẽ
trên bản đồ** nên nhìn được đúng/sai ngay, còn `ips_calib` khớp theo **chính dữ liệu UWB** nên
nuốt được cả sai lệch hệ thống của UWB. Không chạy hai node cùng lúc với `ips_map_tf` — cả ba
đều phát static TF `map -> ips_map`, chồng nhau.
"""

import math

import rclpy
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from interactive_markers import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

from . import transform_io
from .calib import apply_2d, rigid_2d
from .openapi import IpsOpenApi
from .paths import transform_file

SQ = 1.0 / math.sqrt(2.0)          # quaternion (w=1,y=1) chuẩn hoá -> trục điều khiển là Z


class IpsAlign(Node):
    def __init__(self):
        super().__init__("ips_align")
        d = self.declare_parameter
        d("map_frame", "map")
        d("ips_frame", "ips_map")
        d("openapi_url", "http://192.168.68.142:8080")
        d("anchors", [])              # dự phòng khi không đọc được OpenAPI: [x, y, x, y, ...]
        d("anchor_names", [])
        d("click_anchors", ["A1", "A2"])   # 2 anchor phải bấm; đặt 3-4 tên = bấm thêm để có rms
        d("output", "")
        d("load_existing", True)      # nạp file hiệu chuẩn cũ làm điểm xuất phát
        d("marker_size", 0.3)
        d("dist_tolerance", 0.15)     # lệch khoảng cách quá ngần này (tỉ lệ) -> cảnh báo bấm sai

        g = lambda n: self.get_parameter(n).value
        self.map_frame, self.ips_frame = g("map_frame"), g("ips_frame")
        self.out = g("output") or transform_file()
        self.size = float(g("marker_size"))
        self.tol = float(g("dist_tolerance"))

        self.anchors = self._load_anchors(g("openapi_url"), g("anchors"), g("anchor_names"))
        if len(self.anchors) < 2:
            raise RuntimeError(
                "cần ít nhất 2 anchor mà không lấy được. Bật IPS Tool + OpenAPI, hoặc truyền tay:\n"
                "  --ros-args -p anchors:='[0.0, 0.0, 3.6, 0.0, 3.6, -10.2, 0.0, -10.2]' "
                "-p anchor_names:='[A1, A2, A3, A4]'")
        self.targets = self._pick_targets(g("click_anchors"))

        self.theta = self.tx = self.ty = 0.0
        self.rms = None
        self.n_points = 0
        self.have_fit = False        # chưa khớp lần nào -> chưa vẽ anchor, tránh vẽ ở chỗ vô nghĩa
        self.source = "chưa khớp"
        if g("load_existing"):
            self._reload()

        self.bc = StaticTransformBroadcaster(self)
        self.pub_mk = self.create_publisher(MarkerArray, "ips/align/anchors", 1)
        self.clicks = []
        self.create_subscription(PointStamped, "clicked_point", self.on_click, 10)
        for name, cb in (("save", self.srv_save), ("reload", self.srv_reload),
                         ("clear_clicks", self.srv_clear), ("status", self.srv_status)):
            self.create_service(Trigger, f"~/{name}", cb)

        self.menu = MenuHandler()
        self.menu.insert("Lưu vào file", callback=lambda fb: self._log(self._save()))
        self.menu.insert("Nạp lại từ file", callback=lambda fb: self._log(self._reload()))
        self.menu.insert("Xoá click đang dở", callback=lambda fb: self._log(self._clear()))
        self.server = InteractiveMarkerServer(self, "ips_align")
        self._make_handle()

        self._send_tf()
        self.create_timer(0.2, self.publish_markers)
        self._log(self._instructions())

    # ------------------------------------------------------------------ dữ liệu anchor
    def _load_anchors(self, url, flat, names):
        try:
            got = IpsOpenApi(url).anchors()
            if got:
                self.get_logger().info(f"lấy {len(got)} anchor từ OpenAPI {url}")
                return [{"name": a["name"], "x": a["x"], "y": a["y"]} for a in got]
            self.get_logger().warn(f"OpenAPI {url} trả về 0 anchor")
        except Exception as e:
            self.get_logger().warn(f"không đọc được anchor qua OpenAPI: {e}")
        flat = list(flat or [])
        names = list(names or [])
        out = [{"name": names[i // 2] if i // 2 < len(names) else f"A{i // 2 + 1}",
                "x": float(flat[i]), "y": float(flat[i + 1])} for i in range(0, len(flat) - 1, 2)]
        if out:
            self.get_logger().info(f"dùng {len(out)} anchor từ tham số")
        return out

    def _pick_targets(self, want):
        """Thứ tự anchor phải bấm. Mặc định đúng 2 cái (A1, A2) -> bấm lại 2 điểm là ghi đè,
        không phải nhớ mình đang ở bước nào. Khai 3-4 tên thì bấm thêm để có rms."""
        by_name = {a["name"]: a for a in self.anchors}
        want = [w for w in (want or []) if w in by_name]
        if len(want) >= 2:
            return [by_name[w] for w in want]
        best, pair = -1.0, (self.anchors[0], self.anchors[1])
        for i, a in enumerate(self.anchors):
            for b in self.anchors[i + 1:]:
                d = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                if d > best:
                    best, pair = d, (a, b)
        self.get_logger().warn(
            f"click_anchors không khớp anchor nào có thật ({list(by_name)}), "
            f"tự chọn cặp xa nhau nhất: {pair[0]['name']}, {pair[1]['name']}")
        return list(pair)

    def _instructions(self):
        a, b = self.targets[0], self.targets[1]
        d = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
        # lệch tay khi bấm quy ra lệch góc của cả khung: càng 2 điểm gần nhau càng nhạy
        deg = math.degrees(math.atan2(0.05, max(d, 1e-6)))
        more = [t["name"] for t in self.targets[2:]]
        extra = (f"\n  Bấm tiếp {', '.join(more)} thì node giải lại bằng bình phương tối thiểu "
                 "và báo rms." if more else
                 "\n  Bấm lại 2 điểm bất cứ lúc nào để ghi đè.")
        return (f"Bấm `Publish Point` trong RViz vào chân anchor **{a['name']}** trước, "
                f"rồi **{b['name']}**.{extra}\n"
                f"  Hai anchor này cách nhau {d:.2f} m -> bấm lệch 5 cm ≈ khung lệch {deg:.1f}°.\n"
                f"  Xong 2 click là cả {len(self.anchors)} anchor + khung bao hiện lên; "
                "kéo/xoay interactive marker để chỉnh; lưu bằng:\n"
                "  ros2 service call /ips_align/save std_srvs/srv/Trigger")

    # ------------------------------------------------------------------ khớp bằng click
    def on_click(self, msg):
        if msg.header.frame_id not in ("", self.map_frame):
            self.get_logger().warn(
                f"điểm bấm ở frame '{msg.header.frame_id}', cần '{self.map_frame}'. "
                f"Đặt Fixed Frame của RViz = {self.map_frame}.")
            return
        if len(self.clicks) >= len(self.targets):
            self.clicks = []          # vòng trước đã đủ điểm -> click này mở vòng mới
        done = self.targets[len(self.clicks)]
        self.clicks.append((msg.point.x, msg.point.y))
        self.publish_markers()        # hiện dấu ngay từ click đầu, không đợi đủ 2 điểm
        if len(self.clicks) < 2:
            self.get_logger().info(
                f"đã ghi {done['name']} = ({msg.point.x:.2f}, {msg.point.y:.2f}) — "
                f"giờ bấm vào {self.targets[1]['name']}")
            return
        self._solve()

    def _solve(self):
        used = self.targets[:len(self.clicks)]
        src = [(a["x"], a["y"]) for a in used]
        dst = list(self.clicks)
        self.theta, self.tx, self.ty, rms = rigid_2d(src, dst)
        self.have_fit = True
        names = ", ".join(a["name"] for a in used)
        self.source = f"ips_align {len(used)} điểm ({names})"
        self.rms = rms if len(used) > 2 else None
        self.n_points = len(used)

        d_true = math.hypot(src[0][0] - src[1][0], src[0][1] - src[1][1])
        d_click = math.hypot(dst[0][0] - dst[1][0], dst[0][1] - dst[1][1])
        err = abs(d_click - d_true) / max(d_true, 1e-6)
        msg = (f"khớp {len(used)} điểm ({names}): yaw={math.degrees(self.theta):.2f}°  "
               f"x={self.tx:.3f}  y={self.ty:.3f}"
               + (f"  rms={rms:.3f} m" if self.rms is not None else "")
               + f"\n  khoảng cách {used[0]['name']}-{used[1]['name']}: thật {d_true:.2f} m, "
                 f"bấm {d_click:.2f} m (lệch {err * 100:.1f}%)")
        if err > self.tol:
            self.get_logger().error(
                msg + f"\n  LỆCH QUÁ {self.tol * 100:.0f}% — nhiều khả năng bấm nhầm chỗ hoặc "
                "nhầm thứ tự anchor. Gọi `clear_clicks` rồi bấm lại.")
        elif self.rms is not None and self.rms > 0.3:
            self.get_logger().warn(
                msg + f"\n  rms {self.rms:.3f} m hơi lớn — soi lại xem có điểm nào bấm lệch không.")
        else:
            self.get_logger().info(
                msg + ("\n  Bấm tiếp " + self.targets[len(self.clicks)]["name"] +
                       " để tinh chỉnh, hoặc kéo interactive marker rồi `save`."
                       if len(self.clicks) < len(self.targets)
                       else "\n  Chỉnh thêm bằng interactive marker rồi `save`."))
        self._apply()

    # ------------------------------------------------------------------ TF + marker
    def _apply(self):
        self._send_tf()
        self.server.setPose("ips_origin", self._handle_pose())
        self.server.applyChanges()
        self.publish_markers()

    def _send_tf(self):
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id, tr.child_frame_id = self.map_frame, self.ips_frame
        tr.transform.translation.x, tr.transform.translation.y = self.tx, self.ty
        tr.transform.rotation.z = math.sin(self.theta / 2.0)
        tr.transform.rotation.w = math.cos(self.theta / 2.0)
        self.bc.sendTransform(tr)

    def _handle_pose(self):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x, p.position.y = self.tx, self.ty
        p.orientation.z, p.orientation.w = math.sin(self.theta / 2), math.cos(self.theta / 2)
        return p

    def _make_handle(self):
        im = InteractiveMarker()
        im.header.frame_id = self.map_frame
        im.name, im.description = "ips_origin", "gốc hệ IPS — kéo để dịch, vòng để xoay"
        im.scale = max(1.0, self.size * 4)
        im.pose = self._handle_pose()

        ball = Marker(type=Marker.SPHERE)
        ball.scale.x = ball.scale.y = ball.scale.z = self.size
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.4, 0.9, 0.9
        grab = InteractiveMarkerControl(always_visible=True,
                                        interaction_mode=InteractiveMarkerControl.MENU)
        grab.markers.append(ball)
        im.controls.append(grab)

        for mode in (InteractiveMarkerControl.MOVE_PLANE, InteractiveMarkerControl.ROTATE_AXIS):
            c = InteractiveMarkerControl(interaction_mode=mode, always_visible=True)
            c.name = "move_xy" if mode == InteractiveMarkerControl.MOVE_PLANE else "rotate_yaw"
            c.orientation.w, c.orientation.y = SQ, SQ      # trục điều khiển = Z
            im.controls.append(c)

        self.server.insert(im, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, im.name)
        self.server.applyChanges()

    def on_feedback(self, fb):
        if fb.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        q = fb.pose.orientation
        self.tx, self.ty = fb.pose.position.x, fb.pose.position.y
        self.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.source.startswith("ips_align chỉnh tay"):
            self.source = "ips_align chỉnh tay trên RViz"
        self._send_tf()

    def publish_markers(self):
        """Vẽ thẳng trong frame map, không qua TF — kéo marker tới đâu thấy tới đó.

        Mở đầu bằng DELETEALL: số marker thay đổi theo số click, không xoá thì dấu cũ còn lại.
        """
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()
        nxt = (self.targets[len(self.clicks)]["name"]
               if len(self.clicks) < len(self.targets) else None)

        # 1. dấu tại các điểm vừa bấm — có ngay từ click đầu tiên
        for i, (cx, cy) in enumerate(self.clicks):
            name = self.targets[i]["name"]
            m = self._mk(Marker.CYLINDER, "click", i, stamp, cx, cy)
            m.scale.x = m.scale.y = self.size * 1.25
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.4, 0.9
            arr.markers.append(m)
            t = self._mk(Marker.TEXT_VIEW_FACING, "click_label", i, stamp, cx, cy, z=0.75)
            t.scale.z = self.size * 1.1
            t.color.g = t.color.b = t.color.r = t.color.a = 1.0
            t.text = f"{name} ✓"
            arr.markers.append(t)

        # 2. anchor — chỉ vẽ khi đã khớp, chưa khớp mà vẽ thì là vẽ ở chỗ vô nghĩa
        if not self.have_fit:
            if nxt and self.clicks:     # gợi ý ngay cạnh điểm vừa bấm, không thả lơ lửng ở gốc map
                cx, cy = self.clicks[-1]
                tip = self._mk(Marker.TEXT_VIEW_FACING, "hint", 0, stamp, cx, cy, z=1.15)
                tip.scale.z = self.size
                tip.color.r = tip.color.g = 1.0
                tip.color.b, tip.color.a = 0.3, 1.0
                tip.text = f"giờ bấm vào {nxt}"
                arr.markers.append(tip)
            self.pub_mk.publish(arr)
            return

        pts = [apply_2d((a["x"], a["y"]), self.theta, self.tx, self.ty) for a in self.anchors]
        for i, (a, (x, y)) in enumerate(zip(self.anchors, pts)):
            hit = a["name"] == nxt
            m = self._mk(Marker.CYLINDER, "anchor", i, stamp, x, y)
            m.scale.x = m.scale.y = self.size
            m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b = (1.0, 0.85, 0.1) if hit else (0.2, 0.9, 1.0)
            m.color.a = 0.95
            arr.markers.append(m)

            t = self._mk(Marker.TEXT_VIEW_FACING, "anchor_label", i, stamp, x, y, z=0.45)
            t.scale.z = self.size * 1.2
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = a["name"] + (" ← bấm điểm này" if hit else "")
            arr.markers.append(t)

        if len(pts) >= 3:                       # khung bao, xếp theo góc quanh tâm
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            ring = sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
            e = self._mk(Marker.LINE_STRIP, "anchor_ring", 0, stamp, 0.0, 0.0)
            e.scale.x = 0.04
            e.color.r, e.color.g, e.color.b, e.color.a = 0.2, 0.9, 1.0, 0.6
            e.points = [Point(x=p[0], y=p[1], z=0.0) for p in ring + [ring[0]]]
            arr.markers.append(e)
        self.pub_mk.publish(arr)

    def _mk(self, kind, ns, mid, stamp, x, y, z=0.0):
        m = Marker(type=kind, ns=ns, id=mid, action=Marker.ADD)
        m.header.frame_id, m.header.stamp = self.map_frame, stamp
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        return m

    # ------------------------------------------------------------------ service
    def _log(self, text):
        self.get_logger().info(text)
        return text

    def _save(self):
        try:
            transform_io.write(self.out, self.theta, self.tx, self.ty, parent=self.map_frame,
                               child=self.ips_frame, source=self.source, rms=self.rms,
                               n_points=max(2, self.n_points),
                               raw=[f"anchor {a['name']}: ips=({a['x']:.3f}, {a['y']:.3f})  "
                                    f"map=({m[0]:.3f}, {m[1]:.3f})"
                                    for a, m in zip(self.anchors,
                                                    [apply_2d((a["x"], a["y"]), self.theta,
                                                              self.tx, self.ty)
                                                     for a in self.anchors])])
        except OSError as e:
            return f"không ghi được {self.out}: {e}"
        return (f"đã lưu yaw={math.degrees(self.theta):.2f}° x={self.tx:.3f} y={self.ty:.3f}\n"
                f"  -> {self.out}\n  Tắt node này rồi chạy: ros2 run ips_a02_ros2 ips_map_tf")

    def _reload(self):
        try:
            cfg = transform_io.load(self.out)
        except RuntimeError as e:
            return str(e).splitlines()[0]
        self.theta, self.tx, self.ty = float(cfg["yaw"]), float(cfg["x"]), float(cfg["y"])
        self.rms = cfg.get("rms") if isinstance(cfg.get("rms"), (int, float)) else None
        self.n_points = int(cfg.get("n_points") or 0)
        self.have_fit = True
        self.source = str(cfg.get("source") or "file cũ")
        if hasattr(self, "server"):
            self._apply()
        return (f"đã nạp {self.out}: yaw={math.degrees(self.theta):.2f}° "
                f"x={self.tx:.3f} y={self.ty:.3f} (nguồn: {self.source})")

    def _clear(self):
        n, self.clicks = len(self.clicks), []
        return f"đã bỏ {n} click đang dở. " + self._instructions()

    def srv_save(self, _req, res):
        res.message = self._log(self._save())
        res.success = res.message.startswith("đã lưu")
        return res

    def srv_reload(self, _req, res):
        res.message = self._log(self._reload())
        res.success = res.message.startswith("đã nạp")
        return res

    def srv_clear(self, _req, res):
        res.success, res.message = True, self._log(self._clear())
        return res

    def srv_status(self, _req, res):
        res.success = True
        res.message = (f"yaw={math.degrees(self.theta):.3f}°  x={self.tx:.3f}  y={self.ty:.3f}\n"
                       f"nguồn: {self.source}\nfile: {self.out}\n"
                       f"anchor: " + ", ".join(f"{a['name']}({a['x']:.2f},{a['y']:.2f})"
                                               for a in self.anchors))
        return res


def main():
    rclpy.init()
    try:
        node = IpsAlign()
    except RuntimeError as e:
        print(f"[ips_align] LỖI: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
