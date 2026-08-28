"""ips_sim — giả lập IPS Tool: sinh message Pos đúng định dạng hãng và đẩy qua đúng
phương thức đang chọn (udp/mqtt/webhook), để test toàn bộ chuỗi khi chưa có phần cứng.

Giả lập luôn nút bấm trên tag (không cần cầm tag thật):

    ros2 service call /ips_sim/press  std_srvs/srv/Trigger    # -> Call
    ros2 service call /ips_sim/cancel std_srvs/srv/Trigger    # -> CancelCall

Tag phát sự kiện là `press_tag` (mặc định T1), đổi bằng `ros2 param set /ips_sim press_tag T2`.
"""

import json
import math
import random
import socket
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class IpsSim(Node):
    def __init__(self):
        super().__init__("ips_sim")
        d = self.declare_parameter
        d("source", "udp")
        d("host", "127.0.0.1")
        d("udp.port", 9600)
        d("mqtt.broker", "127.0.0.1")
        d("mqtt.port", 1883)
        d("mqtt.topic", "ips/uplink")
        d("webhook.port", 8091)
        d("webhook.path", "/ingest")
        d("tags", 3)
        d("rate", 10.0)
        d("noise", 0.04)
        d("pattern", "lissajous")     # lissajous | circle | random_walk | static
        # vùng hoạt động, mặc định trùng khung 4 anchor A02 thật (3.6 x 10.2 m)
        d("area_min", [0.0, -10.2])
        d("area_max", [3.6, 0.0])
        d("height", 1.2)
        d("press_tag", "T1")          # tag nào "bấm nút" khi gọi service ~/press

        g = lambda n: self.get_parameter(n).value
        self.src = g("source")
        self.host = g("host")
        self.n = int(g("tags"))
        self.noise = float(g("noise"))
        self.pattern = g("pattern")
        self.lo, self.hi = list(g("area_min")), list(g("area_max"))
        self.zh = float(g("height"))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_port = int(g("udp.port"))
        self.wh_url = f"http://{self.host}:{g('webhook.port')}{g('webhook.path')}"
        self.mqtt = None
        if self.src == "mqtt":
            import paho.mqtt.client as m
            self.mqtt = m.Client()
            self.mqtt.connect(g("mqtt.broker"), int(g("mqtt.port")), 30)
            self.mqtt.loop_start()
            self.topic = g("mqtt.topic")

        self.t0 = time.time()
        self.cnt = [random.randint(1000, 5000) for _ in range(self.n)]
        self.ph = [random.random() * 6.283 for _ in range(self.n)]
        self.rw = [[random.uniform(self.lo[0], self.hi[0]),
                    random.uniform(self.lo[1], self.hi[1])] for _ in range(self.n)]
        self.prev = [None] * self.n
        self.rate = float(g("rate"))
        self.create_timer(1.0 / max(0.5, self.rate), self.tick)
        self.create_service(Trigger, "~/press", lambda q, r: self._event("Call", r))
        self.create_service(Trigger, "~/cancel", lambda q, r: self._event("CancelCall", r))
        self.get_logger().info(
            f"giả lập {self.n} tag @ {self.rate} Hz -> {self.src.upper()} "
            f"(vùng x {self.lo[0]}..{self.hi[0]}, y {self.lo[1]}..{self.hi[1]})")

    def _xy(self, i, t):
        cx, cy = (self.lo[0] + self.hi[0]) / 2, (self.lo[1] + self.hi[1]) / 2
        rx, ry = (self.hi[0] - self.lo[0]) / 2, (self.hi[1] - self.lo[1]) / 2
        if self.pattern == "circle":
            r = min(rx, ry) * 0.75
            a = 0.4 * t + self.ph[i]
            return cx + r * math.cos(a), cy + r * math.sin(a)
        if self.pattern == "static":
            return cx + (i - self.n / 2) * 0.6, cy
        if self.pattern == "random_walk":
            self.rw[i][0] = min(self.hi[0], max(self.lo[0], self.rw[i][0] + random.gauss(0, 0.05)))
            self.rw[i][1] = min(self.hi[1], max(self.lo[1], self.rw[i][1] + random.gauss(0, 0.05)))
            return self.rw[i]
        return (cx + rx * 0.8 * math.sin(0.31 * t + self.ph[i]),
                cy + ry * 0.8 * math.sin(0.23 * 1.7 * t + self.ph[i] * 1.3))

    def tick(self):
        t = time.time() - self.t0
        for i in range(self.n):
            self.cnt[i] += 1
            x, y = self._xy(i, t)
            x += random.gauss(0, self.noise)
            y += random.gauss(0, self.noise)
            z = self.zh + random.gauss(0, self.noise / 3)
            if self.prev[i]:
                dt = 1.0 / self.rate
                vx, vy = (x - self.prev[i][0]) / dt, (y - self.prev[i][1]) / dt
            else:
                vx = vy = 0.0
            self.prev[i] = (x, y)
            msg = {"deviceName": f"T{i + 1}", "name": "Pos", "uid": f"SIM{i + 1:012X}",
                   "data": {"cnt": self.cnt[i], "fences": [], "mapId": 1,
                            "noiseRadius": round(abs(random.gauss(self.noise, self.noise / 2)), 5),
                            "optPoses": [[round(x, 4), round(y, 4), round(z, 4)]],
                            "pos": [round(x, 5), round(y, 5), round(z, 5)],
                            "posVar": [round(self.noise ** 2, 8)] * 3,
                            "rxAnchorSn": f"SIMANCHOR{(i % 4) + 1}",
                            "rxRssi": round(random.gauss(-60, 4), 2),
                            "time": int(time.time() * 1000),
                            "vel": [round(vx, 4), round(vy, 4), 0.0],
                            "velVar": [0.01, 0.01, 0.001], "z": round(z, 5)}}
            self.send(json.dumps(msg))

    def _event(self, kind, res):
        """Bắn Call/CancelCall đúng phong bì của hãng: không có `data`, key là uid."""
        name = self.get_parameter("press_tag").value
        i = next((k for k in range(self.n) if f"T{k + 1}" == name), None)
        if i is None:
            res.success, res.message = False, (
                f"press_tag='{name}' không nằm trong {self.n} tag giả lập (T1..T{self.n})")
            return res
        self.send(json.dumps({"deviceName": name, "name": kind, "uid": f"SIM{i + 1:012X}"}))
        res.success, res.message = True, f"đã gửi {kind} từ {name}"
        self.get_logger().info(res.message)
        return res

    def send(self, raw):
        try:
            if self.src == "udp":
                self.sock.sendto(raw.encode(), (self.host, self.udp_port))
            elif self.src == "mqtt":
                self.mqtt.publish(self.topic, raw, qos=0)
            else:
                req = urllib.request.Request(self.wh_url, data=raw.encode(),
                                             headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=1).read()
        except Exception as e:
            self.get_logger().warn(f"gửi lỗi: {e}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = IpsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
