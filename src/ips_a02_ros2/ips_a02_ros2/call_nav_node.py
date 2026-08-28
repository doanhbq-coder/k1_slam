"""ips_call_nav — nhận nút bấm trên tag UWB, đẩy goal cho Nav2.

    IpsEvent(Call) -> lọc -> tf2 ips_map->map -> lùi lại `standoff` -> dò ô free -> NavigateToPose
    IpsEvent(CancelCall) -> huỷ goal đang chạy

Không sửa gì bên Nav2: nối vào đúng một cửa là action `navigate_to_pose`.

Hành vi thật của tag A02 (đo 2026-08-26, quyết định toàn bộ phần debounce):
  - giữ nút ~2 s thì tag vào "chế độ call" và phát `Call` **lặp lại ~1 Hz**, không phải một lần;
  - `Pos` của tag về ~1 Hz, nên `pose_age` bình thường đã ~1.0 s.
Vì vậy: một lần bấm = một **phiên**, kết thúc khi luồng Call ngắt quãng `rearm_gap` giây.
Trong cùng phiên, các Call sau chỉ là nhịp giữ, không tạo goal mới.
"""

import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  — đăng ký kiểu PoseStamped cho tf2, BẮT BUỘC import
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from ips_a02_msgs.msg import IpsEvent
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .openapi import IpsOpenApi


class IpsCallNavigator(Node):
    def __init__(self):
        super().__init__("ips_call_navigator")
        defaults = {}

        def d(name, value):
            defaults[name] = value
            self.declare_parameter(name, value)

        d("map_frame", "map")
        d("ips_frame", "ips_map")
        d("robot_base_frame", "base_footprint")
        d("action_name", "navigate_to_pose")
        d("standoff", 0.9)             # dừng cách người bao nhiêu mét
        d("max_pose_age", 3.0)         # Pos về ~1 Hz -> 1.5 s là quá chặt, loại cả lệnh hợp lệ
        d("max_noise", 0.5)            # noiseRadius đo thực tế 0.11..0.28 m
        d("rearm_gap", 3.0)            # Call ngắt ngần này giây thì lần bấm sau tính là phiên mới
        d("allowed_tags", [])          # rỗng = cho tất cả tag; điền tên hoặc uid
        d("queue_policy", "preempt")   # preempt = goal mới huỷ goal cũ | ignore = bỏ qua
        d("costmap_topic", "/global_costmap/costmap")
        d("search_radius", 1.5)        # bán kính dò ô free quanh đích
        d("lethal_threshold", 50)      # cost >= ngần này coi như chặn
        d("allow_unknown", False)      # ô -1 (chưa biết) có coi là đi được không
        d("check_anchor_box", True)    # từ chối tag ở ngoài khung anchor
        d("anchor_box_margin", 1.0)
        d("follow", False)             # True = bám theo người trong suốt phiên gọi
        d("follow_min_move", 0.5)      # bám: tag phải dịch ngần này mét mới cập nhật goal
        d("send_ack", True)            # báo ngược về tag qua OpenAPI (tag rung/nháy)
        # Bấm Call/CancelCall trên web IPS Tool KHÔNG sinh uplink `Call`, mà sinh `Ack` kèm
        # reqId (đo 2026-08-27). Nhận Ack làm lệnh thì gọi được robot từ server, không cần
        # ai chạm vào tag.
        d("trigger_on_ack", True)
        d("ack_call_req_id", "CallAck")
        d("ack_cancel_req_id", "CancelCallAck")
        # Chính node này cũng gửi Ack(CallAck) xuống tag khi nhận lệnh. Nếu IPS Tool đẩy ngược
        # lên thì thành vòng lặp tự gọi -> bỏ qua Ack của tag vừa được gửi ack trong ngần này giây.
        d("ack_echo_window", 5.0)
        d("openapi_url", "http://192.168.68.142:8080")

        # ROS 2 không suy được kiểu của LIST RỖNG trong file yaml: `allowed_tags: []` làm tham số
        # thành NOT_SET và get_parameter() ném ParameterUninitializedException ngay lúc khởi động.
        # Nên đọc tham số qua hàm này, hỏng thì lùi về đúng giá trị mặc định đã khai.
        def g(name):
            try:
                v = self.get_parameter(name).value
            except Exception:
                return defaults.get(name)
            return defaults.get(name) if v is None else v

        self.map_frame, self.ips_frame = g("map_frame"), g("ips_frame")
        self.base_frame = g("robot_base_frame")
        self.standoff = float(g("standoff"))
        self.max_age, self.max_noise = float(g("max_pose_age")), float(g("max_noise"))
        self.rearm = float(g("rearm_gap"))
        self.allowed = {a for a in (g("allowed_tags") or []) if a}
        self.policy = g("queue_policy")
        self.search_radius = float(g("search_radius"))
        self.lethal, self.allow_unknown = int(g("lethal_threshold")), bool(g("allow_unknown"))
        self.follow, self.follow_min = bool(g("follow")), float(g("follow_min_move"))
        self.trigger_on_ack = bool(g("trigger_on_ack"))
        self.ack_call_id, self.ack_cancel_id = g("ack_call_req_id"), g("ack_cancel_req_id")
        self.ack_window = float(g("ack_echo_window"))
        self.ack_sent = {}             # uid -> (lúc node tự gửi Ack, reqId), để nhận ra tiếng vọng

        self.api = IpsOpenApi(g("openapi_url")) if g("send_ack") else None
        self.margin = float(g("anchor_box_margin"))
        self.box = self._anchor_box(g("openapi_url")) if g("check_anchor_box") else None

        self.costmap = None
        self.goal_handle = None
        self.active_uid = ""
        self.active_goal = None        # (x, y) đã gửi, để biết người đã đi xa chưa
        self.sessions = {}             # uid -> {"last": t, "handled": bool}

        from tf2_ros import Buffer, TransformListener
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self)

        cb = ReentrantCallbackGroup()
        self.nav = ActionClient(self, NavigateToPose, g("action_name"), callback_group=cb)
        self.create_subscription(IpsEvent, "ips/events", self.on_event, 10, callback_group=cb)
        self.create_subscription(          # costmap là transient_local, QoS mặc định không nhận được
            OccupancyGrid, g("costmap_topic"), self.on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST), callback_group=cb)
        self.pub_status = self.create_publisher(String, "ips/nav_status", 10)

        box = (f"khung anchor x[{self.box[0]:.1f},{self.box[1]:.1f}] "
               f"y[{self.box[2]:.1f},{self.box[3]:.1f}] (đã nới {self.margin} m)"
               if self.box else "không kiểm tra khung anchor")
        self.get_logger().info(
            f"chờ nút bấm trên /ips/events -> action '{g('action_name')}'\n"
            f"  standoff {self.standoff} m · tuổi mẫu tối đa {self.max_age} s · "
            f"nhiễu tối đa {self.max_noise} m · phiên mới sau {self.rearm} s im lặng\n"
            f"  {box} · bám theo người: {'CÓ' if self.follow else 'không'}")
        self.create_timer(5.0, self._check_server)
        self._warned_server = False

    # ------------------------------------------------------------------ hạ tầng
    def _anchor_box(self, url):
        try:
            an = IpsOpenApi(url).anchors()
        except Exception as e:
            self.get_logger().warn(f"không lấy được anchor để dựng khung: {e} — bỏ kiểm tra khung")
            return None
        if len(an) < 3:
            return None
        m = self.margin
        xs, ys = [a["x"] for a in an], [a["y"] for a in an]
        return (min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m)

    def _check_server(self):
        if self.nav.server_is_ready():
            if self._warned_server:
                self.get_logger().info("Nav2 đã sẵn sàng")
                self._warned_server = False
            return
        if not self._warned_server:
            self.get_logger().warn(
                f"chưa thấy action '{self.get_parameter('action_name').value}' — Nav2 đã chạy chưa?")
            self._warned_server = True

    def on_costmap(self, msg):
        self.costmap = msg

    def _status(self, text, level="info"):
        # rclpy nhớ mức log theo VỊ TRÍ DÒNG GỌI: gọi getattr(logger, level) từ cùng một dòng mà
        # đổi mức giữa info/warn sẽ ném ValueError('Logger severity cannot be changed between
        # calls') và giết node. Mỗi mức phải nằm ở một dòng riêng.
        log = self.get_logger()
        if level == "warn":
            log.warning(text)
        elif level == "error":
            log.error(text)
        else:
            log.info(text)
        self.pub_status.publish(String(data=text))

    # ------------------------------------------------------------------ sự kiện
    def on_event(self, ev):
        if ev.kind == IpsEvent.KIND_CALL:
            self.on_call(ev)
        elif ev.kind == IpsEvent.KIND_CANCEL:
            self.on_cancel(ev)
        elif ev.kind == IpsEvent.KIND_ACK:
            self.on_ack(ev)
        else:
            self.get_logger().debug(f"bỏ qua {ev.kind} từ {ev.device_name}")

    def on_ack(self, ev):
        """Ack là đường điều khiển từ WEB IPS Tool: bấm Call/CancelCall ở đó chỉ sinh Ack."""
        if not self.trigger_on_ack:
            return
        if ev.req_id not in (self.ack_call_id, self.ack_cancel_id):
            self.get_logger().debug(f"Ack lạ reqId='{ev.req_id}' từ {ev.device_name}")
            return
        # Chỉ chặn ĐÚNG reqId mình vừa gửi. Chặn cả cụm thì cú CancelCall bấm ngay sau Call
        # cũng bị nuốt — người bấm huỷ mà robot vẫn chạy tiếp là lỗi nguy hiểm.
        sent_at, sent_id = self.ack_sent.get(ev.uid, (0.0, ""))
        gap = time.time() - sent_at
        if sent_id == ev.req_id and gap < self.ack_window:
            self.get_logger().info(
                f"bỏ qua Ack '{ev.req_id}' của {ev.device_name}: tiếng vọng của Ack chính node "
                f"vừa gửi {gap:.1f}s trước")
            return
        if ev.req_id == self.ack_cancel_id:
            self.get_logger().info(f"[web] CancelCall cho {ev.device_name}")
            self.on_cancel(ev)
            return
        self.get_logger().info(f"[web] Call cho {ev.device_name}")
        self.on_call(ev)

    def on_call(self, ev):
        now = time.time()
        ses = self.sessions.get(ev.uid)
        fresh = ses is None or (now - ses["last"]) > self.rearm
        if fresh:
            ses = {"last": now, "handled": False}
            self.sessions[ev.uid] = ses
            self.get_logger().info(f"[Call] {ev.device_name}: phiên mới")
        ses["last"] = now

        if ses["handled"]:
            if self.follow and self.active_uid == ev.uid:
                self._maybe_follow(ev)
            return                      # nhịp giữ của cùng một lần bấm, không tạo goal mới

        why = self._reject_reason(ev)
        if why:
            ses["handled"] = True       # đã từ chối thì thôi, đừng spam mỗi giây một lần
            self._status(f"TỪ CHỐI {ev.device_name}: {why}", "warn")
            return

        goal = self._plan_goal(ev)
        if goal is None:
            ses["handled"] = True
            return
        if self.goal_handle is not None and self.policy == "ignore":
            ses["handled"] = True
            self._status(f"bỏ qua {ev.device_name}: đang bận chạy cho {self.active_uid}", "warn")
            return
        ses["handled"] = True
        self._send_goal(goal, ev)

    def on_cancel(self, ev):
        ses = self.sessions.get(ev.uid)
        if ses:
            ses["handled"] = True       # tag có thể còn phát Call sau khi huỷ -> chặn tái kích hoạt
            ses["last"] = time.time()
        if self.goal_handle is None or self.active_uid != ev.uid:
            self._status(f"[CancelCall] {ev.device_name}: không có goal nào của tag này đang chạy")
            return
        self._status(f"[CancelCall] {ev.device_name}: đang huỷ goal")
        self.goal_handle.cancel_goal_async()
        self._ack(ev.uid, "CancelCallAck")

    # ------------------------------------------------------------------ lọc & tính goal
    def _reject_reason(self, ev):
        if self.allowed and ev.device_name not in self.allowed and ev.uid not in self.allowed:
            return f"tag không nằm trong allowed_tags ({sorted(self.allowed)})"
        if not ev.pose_valid:
            return "chưa có vị trí của tag (chưa nhận Pos nào)"
        if ev.pose_age > self.max_age:
            return f"mẫu vị trí cũ {ev.pose_age:.1f}s > {self.max_age}s"
        if self.max_noise > 0 and ev.pose_noise > self.max_noise:
            return f"nhiễu {ev.pose_noise:.2f}m > {self.max_noise}m"
        if self.box:
            x, y = ev.tag_pose.pose.position.x, ev.tag_pose.pose.position.y
            if not (self.box[0] <= x <= self.box[1] and self.box[2] <= y <= self.box[3]):
                return (f"tag ở ({x:.1f}, {y:.1f}) ngoài khung anchor — ngoài vùng phủ UWB vẫn "
                        "trả toạ độ nhưng sai rất nhiều")
        if not self.nav.server_is_ready():
            return "Nav2 chưa sẵn sàng"
        return None

    def _robot_xy(self):
        try:
            tr = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self._status(f"không đọc được TF {self.map_frame}->{self.base_frame}: {e}", "warn")
            return None
        return tr.transform.translation.x, tr.transform.translation.y

    def _tag_in_map(self, ev):
        p = PoseStamped()
        p.header.frame_id = ev.tag_pose.header.frame_id or self.ips_frame
        p.pose = ev.tag_pose.pose
        try:
            # Time() = "mới nhất": TF map->ips_map là static, chỉ có đúng một mốc thời gian
            out = self.tfbuf.transform(p, self.map_frame, rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self._status(f"không đổi được {p.header.frame_id} -> {self.map_frame}: {e}. "
                         "Đã chạy ips_map_tf chưa?", "warn")
            return None
        return out.pose.position.x, out.pose.position.y

    def _plan_goal(self, ev):
        tag = self._tag_in_map(ev)
        rob = self._robot_xy()
        if tag is None or rob is None:
            return None
        dx, dy = rob[0] - tag[0], rob[1] - tag[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:                       # robot đứng ngay trên tag: giữ nguyên hướng hiện tại
            gx, gy = rob
        else:
            k = min(self.standoff, dist) / dist    # đừng lùi quá vị trí robot đang đứng
            gx, gy = tag[0] + dx * k, tag[1] + dy * k
        free = self._free_cell_near(gx, gy)
        if free is None:
            self._status(f"TỪ CHỐI {ev.device_name}: quanh ({gx:.2f}, {gy:.2f}) không có ô trống "
                         f"trong bán kính {self.search_radius} m", "warn")
            return None
        gx, gy = free
        yaw = math.atan2(tag[1] - gy, tag[0] - gx)     # quay mặt về phía người
        return gx, gy, yaw, tag

    # ------------------------------------------------------------------ costmap
    def _cell_free(self, x, y):
        cm = self.costmap
        if cm is None:
            return True                       # chưa có costmap thì để Nav2 tự từ chối
        info = cm.info
        i = int((x - info.origin.position.x) / info.resolution)
        j = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= i < info.width and 0 <= j < info.height):
            return False
        v = cm.data[j * info.width + i]
        if v < 0:
            return self.allow_unknown
        return v < self.lethal

    def _free_cell_near(self, x, y):
        """Dò xoắn ốc ra ngoài, trả ô free gần nhất — người hay đứng sát tường/đồ đạc."""
        if self._cell_free(x, y):
            return x, y
        if self.costmap is None:
            return x, y
        step = max(self.costmap.info.resolution * 2.0, 0.1)
        r = step
        while r <= self.search_radius:
            n = max(8, int(2 * math.pi * r / step))
            for k in range(n):
                a = 2 * math.pi * k / n
                cx, cy = x + r * math.cos(a), y + r * math.sin(a)
                if self._cell_free(cx, cy):
                    return cx, cy
            r += step
        return None

    # ------------------------------------------------------------------ gửi goal
    def _send_goal(self, goal, ev):
        gx, gy, yaw, tag = goal
        if self.goal_handle is not None:
            self._status(f"huỷ goal cũ của {self.active_uid} để nhận lệnh mới")
            self.goal_handle.cancel_goal_async()

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = self.map_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = gx, gy
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.active_uid, self.active_goal = ev.uid, (tag[0], tag[1])
        self._status(f"NHẬN {ev.device_name} ({ev.uid}): tag ({tag[0]:.2f}, {tag[1]:.2f}) "
                     f"-> goal ({gx:.2f}, {gy:.2f}) yaw {math.degrees(yaw):.0f}°")
        fut = self.nav.send_goal_async(msg)
        fut.add_done_callback(lambda f: self._on_accepted(f, ev))
        self._ack(ev.uid, "CallAck")

    def _on_accepted(self, fut, ev):
        try:
            gh = fut.result()
        except Exception as e:
            self._status(f"gửi goal lỗi: {e}", "error")
            return
        if not gh.accepted:
            self.active_uid, self.goal_handle = "", None
            self._status(f"Nav2 TỪ CHỐI goal của {ev.device_name}", "warn")
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_result(f, ev))

    def _on_result(self, fut, ev):
        try:
            st = fut.result().status
        except Exception as e:
            self._status(f"lỗi khi lấy kết quả: {e}", "error")
            st = None
        name = {GoalStatus.STATUS_SUCCEEDED: "TỚI NƠI", GoalStatus.STATUS_CANCELED: "đã huỷ",
                GoalStatus.STATUS_ABORTED: "Nav2 bỏ cuộc"}.get(st, f"kết thúc (status={st})")
        self._status(f"{name}: {ev.device_name}")
        self.goal_handle, self.active_uid, self.active_goal = None, "", None

    def _maybe_follow(self, ev):
        tag = self._tag_in_map(ev)
        if tag is None or self.active_goal is None:
            return
        if math.hypot(tag[0] - self.active_goal[0], tag[1] - self.active_goal[1]) < self.follow_min:
            return
        goal = self._plan_goal(ev)
        if goal is not None:
            self._send_goal(goal, ev)

    def _ack(self, uid, req_id):
        if self.api is None:
            return
        # ghi mốc TRƯỚC khi gửi: tiếng vọng có thể về trước cả khi urlopen trả kết quả
        self.ack_sent[uid] = (time.time(), req_id)
        try:
            self.api.ack(uid, req_id)
        except Exception as e:      # tag không phản hồi được thì kệ, robot vẫn phải chạy
            self.get_logger().warn(f"không gửi được {req_id} về tag {uid}: {e}")


def main():
    rclpy.init()
    node = IpsCallNavigator()
    # BẮT BUỘC nhiều luồng: gửi goal từ trong callback của subscription, chạy 1 luồng sẽ deadlock
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
