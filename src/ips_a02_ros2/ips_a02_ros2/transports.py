"""Ba phương thức nhận dữ liệu từ Nooploop IPS Tool, cùng một interface.

IPS Tool là client push (không phải broker): nó đẩy JSON ra theo MQTT, UDP hoặc Webhook.
Mỗi transport ở đây nhận JSON đó rồi gọi `on_message(obj, meta)`.

    tr = make_transport('udp', {'port': 9600}, logger)
    tr.start(cb); ... ; tr.stop()
"""

import json
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Transport:
    name = "base"

    def __init__(self, cfg, logger):
        self.cfg = cfg
        self.log = logger
        self.on_message = None
        self.count = 0
        self.bad = 0
        self.last_peer = ""
        self.ready = False
        self.error = ""

    def start(self, on_message):
        self.on_message = on_message
        self._start()

    def _start(self):
        raise NotImplementedError

    def stop(self):
        pass

    def status(self):
        return {"transport": self.name, "ready": self.ready, "count": self.count,
                "bad": self.bad, "peer": self.last_peer, "error": self.error}

    # --- dùng chung: text -> dict (IPS Tool có thể gửi 1 object hoặc mảng object) ---
    def _feed(self, text, meta=""):
        try:
            obj = json.loads(text)
        except Exception:
            self.bad += 1
            return
        items = obj if isinstance(obj, list) else [obj]
        for it in items:
            if isinstance(it, dict):
                self.count += 1
                if self.on_message:
                    self.on_message(it, meta)


class UdpTransport(Transport):
    """IPS Tool -> UDP: address = <ip máy chạy node>:<port>."""
    name = "udp"

    def _start(self):
        port = int(self.cfg.get("port", 9600))
        bind = self.cfg.get("bind", "0.0.0.0")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)
        try:
            self._sock.bind((bind, port))
        except Exception as e:
            self.error = str(e)
            self.log.error(f"[udp] không bind được {bind}:{port}: {e}")
            return
        self.ready = True
        self.log.info(f"[udp] đang nghe {bind}:{port}")
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._run:
            try:
                data, addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.last_peer = f"{addr[0]}:{addr[1]}"
            self._feed(data.decode("utf-8", "replace"), self.last_peer)
        self.ready = False

    def stop(self):
        self._run = False
        try:
            self._sock.close()
        except Exception:
            pass


class MqttTransport(Transport):
    """IPS Tool -> broker MQTT -> node này. Cần một broker chạy sẵn (mosquitto...)."""
    name = "mqtt"

    def _start(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            self.error = "thiếu paho-mqtt (pip install paho-mqtt)"
            self.log.error(self.error)
            return
        c = mqtt.Client(client_id=self.cfg.get("client_id") or "", clean_session=True)
        if self.cfg.get("username"):
            c.username_pw_set(self.cfg["username"], self.cfg.get("password") or None)
        topics = self.cfg.get("topics") or ["ips/#"]

        def on_connect(cli, u, f, rc):
            if rc == 0:
                self.ready = True
                self.error = ""
                for t in topics:
                    cli.subscribe(t, qos=0)
                self.log.info(f"[mqtt] đã kết nối {self.cfg['broker']}:{self.cfg['port']}, "
                              f"subscribe {topics}")
            else:
                self.ready = False
                self.error = f"rc={rc}"
                self.log.error(f"[mqtt] kết nối lỗi rc={rc}")

        def on_disconnect(cli, u, rc):
            self.ready = False
            if rc != 0:
                self.error = f"mất kết nối (rc={rc})"

        def on_message(cli, u, msg):
            self.last_peer = msg.topic
            self._feed(msg.payload.decode("utf-8", "replace"), msg.topic)

        c.on_connect, c.on_disconnect, c.on_message = on_connect, on_disconnect, on_message
        self._cli = c
        try:
            c.connect_async(self.cfg["broker"], int(self.cfg["port"]), keepalive=30)
            c.loop_start()
        except Exception as e:
            self.error = str(e)
            self.log.error(f"[mqtt] {e}")

    def stop(self):
        try:
            self._cli.loop_stop()
            self._cli.disconnect()
        except Exception:
            pass


class _QuietHTTPServer(ThreadingHTTPServer):
    """IPS Tool hay ngắt socket kiểu RST sau mỗi POST -> bỏ qua lỗi kết nối, giữ log sạch."""

    def handle_error(self, request, client_address):
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionResetError, BrokenPipeError, TimeoutError, socket.timeout)):
            return
        super().handle_error(request, client_address)


class WebhookTransport(Transport):
    """IPS Tool -> HTTP POST http://<ip máy chạy node>:<port><path>."""
    name = "webhook"

    def _start(self):
        port = int(self.cfg.get("port", 8091))
        path = self.cfg.get("path", "/ingest")
        outer = self

        class H(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *a):
                pass

            def handle_one_request(self):
                try:
                    super().handle_one_request()
                except (ConnectionResetError, BrokenPipeError, socket.timeout):
                    self.close_connection = True

            def do_POST(self):
                n = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(n).decode("utf-8", "replace") if n else ""
                ok = self.path.startswith(path)
                if ok:
                    outer.last_peer = self.client_address[0]
                    outer._feed(raw, outer.last_peer)
                body = b'{"ok":true}' if ok else b'{"ok":false}'
                self.send_response(200 if ok else 404)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        try:
            self._srv = _QuietHTTPServer(("0.0.0.0", port), H)
        except Exception as e:
            self.error = str(e)
            self.log.error(f"[webhook] không mở được cổng {port}: {e}")
            return
        self._srv.daemon_threads = True
        self.ready = True
        self.log.info(f"[webhook] đang nghe http://0.0.0.0:{port}{path}")
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()

    def stop(self):
        try:
            self._srv.shutdown()
        except Exception:
            pass


def make_transport(kind, cfg, logger):
    kinds = {"udp": UdpTransport, "mqtt": MqttTransport, "webhook": WebhookTransport}
    if kind not in kinds:
        raise ValueError(f"source phải là một trong {list(kinds)}, nhận được '{kind}'")
    return kinds[kind](cfg, logger)
