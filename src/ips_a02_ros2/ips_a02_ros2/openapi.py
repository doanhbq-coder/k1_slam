"""HTTP OpenAPI của IPS Tool: đọc anchor / vùng ảo (poll định kỳ), và gọi ngược về tag."""

import json
import urllib.error
import urllib.request


class IpsOpenApi:
    def __init__(self, base_url, timeout=5.0):
        self.base = (base_url or "").rstrip("/")
        self.timeout = timeout

    def _get(self, path):
        url = f"{self.base}/openapi/v1/{path.lstrip('/')}"
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode("utf-8", "replace"))

    def _put(self, path, body=None):
        url = f"{self.base}/openapi/v1/{path.lstrip('/')}"
        raw = json.dumps(body).encode() if body is not None else b""
        req = urllib.request.Request(url, data=raw, method="PUT",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            d = json.loads(r.read().decode("utf-8", "replace") or "{}")
        if d.get("code") not in (0, None):      # response luôn bọc code/message/data
            raise RuntimeError(f"OpenAPI code={d.get('code')} msg={d.get('message')}")
        return d.get("data")

    def ack(self, uid, req_id="CallAck"):
        """Báo lại cho tag là đã nhận lệnh — tag rung/nháy, người bấm biết robot đang tới."""
        return self._put(f"devices/{uid}/services/Ack", {"data": {"reqId": req_id}})

    def call(self, uid, duration=1):
        """Kêu ở tag. duration: giây, tối đa 64."""
        return self._put(f"devices/{uid}/services/Call",
                         {"data": {"duration": max(1, min(64, int(duration)))}})

    def cancel_call(self, uid):
        return self._put(f"devices/{uid}/services/CancelCall")

    def anchors(self):
        """[{name, uid, x, y, z, online}] — deviceType=4 là anchor."""
        d = self._get("devices?deviceType=4&pageSize=100")
        out = []
        for r in (d.get("data", {}) or {}).get("records", []) or []:
            c = (r.get("coordinate") or {}).get("coords")
            if not c:
                continue
            out.append({"name": r.get("name") or r.get("uid", "")[:6], "uid": r.get("uid", ""),
                        "x": float(c[0]), "y": float(c[1]),
                        "z": float(c[2]) if len(c) > 2 else 0.0,
                        "online": bool(r.get("online"))})
        return out

    def tags(self):
        d = self._get("devices?deviceType=5&pageSize=100")
        return [{"name": r.get("name"), "uid": r.get("uid"), "online": bool(r.get("online")),
                 "power": r.get("power"), "product": r.get("product")}
                for r in (d.get("data", {}) or {}).get("records", []) or []]

    def fences(self):
        """[{id, name, bounds:[[[x,y],...]]}] — vùng ảo đã khai trong IPS Tool."""
        d = self._get("fences?pageSize=100")
        data = d.get("data") or {}
        recs = data.get("records") if isinstance(data, dict) else None
        if recs is None:
            recs = data if isinstance(data, list) else []
        out = []
        for r in recs:
            if r.get("bounds"):
                out.append({"id": r.get("id"), "name": r.get("name") or f"fence-{r.get('id')}",
                            "bounds": r["bounds"], "enable": r.get("enable", True)})
        return out
