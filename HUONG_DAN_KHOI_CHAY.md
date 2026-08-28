# Hướng dẫn khởi chạy — IPS A02

Mỗi kịch bản ghi rõ **bên server (máy chạy IPS Tool) cần cài đặt gì** và **bên máy Linux/ROS cần
cài đặt gì**, kèm lệnh chạy và cách xác nhận đã thông.

> **File này viết cho workspace cũ `~/Documents/IPS_A02_MQTT/`** (có kèm web dashboard trong
> `web_test/`). Phần **kịch bản A/B/C và G** nói về web dashboard đó, không có trong repo
> `neo_k1`. Phần **cài đặt, cấu hình IPS Tool, bảng cổng, và các kịch bản ROS 2** vẫn đúng — chỉ
> thay đường dẫn workspace thành `~/Documents/neo_k1` và bỏ `ros2_ws/` khỏi đường dẫn.
>
> Phần tích hợp Nav2 (gọi robot tới chỗ người cầm tag) nằm ở
> [`NAV2_INTEGRATION.md`](NAV2_INTEGRATION.md) và
> [`src/ips_a02_ros2/README.md`](src/ips_a02_ros2/README.md).

## Bố cục thư mục

```
~/Documents/IPS_A02_MQTT/
├── HUONG_DAN_KHOI_CHAY.md      ← file này
├── web_test/                    phần web: hub + dashboard (giám sát, kiểm thử, nghiệm thu)
│   ├── ips_hub.py               chạy: cd web_test && python3 -u ips_hub.py
│   ├── config.json              cấu hình hub (nguồn dữ liệu, broker, anchor, bản đồ)
│   ├── mosquitto/ips.conf       mẫu cấu hình mở broker ra LAN
│   └── web/                     giao diện dashboard
└── ros2_ws/                     workspace ROS2
    └── src/ips_a02_ros2/        package: ips_bridge + ips_sim + RViz config
```

> Hub ghi `config.json` vào **thư mục đang đứng khi chạy**. Luôn `cd web_test` trước khi chạy
> `ips_hub.py`, nếu không sẽ sinh ra một `config.json` lạc chỗ và hub đọc nhầm cấu hình rỗng.

## Địa chỉ hệ thống hiện tại

**Cập nhật 28.08.2026 — địa chỉ đã đổi so với bản đầu:**

| Vai trò | Máy | Địa chỉ |
|---|---|---|
| Server định vị — **IPS Tool v1.1.36 chạy trên chính máy Linux này** (`~/Downloads/UWB/ips/`) | máy này | web UI + OpenAPI cổng `8080`, nghe trên mọi card |
| Máy chạy ROS 2 | máy này | có **2 IP**: `192.168.68.142` và `192.168.8.101` — OpenAPI trả lời trên cả hai |
| Broker MQTT (mosquitto) | máy này | `1883`, nghe `0.0.0.0` |
| Anchor A02 | 4 cái | A1 (0, 0) · A2 (3.6, 0) · A3 (3.6, −10.2) · A4 (0, −10.2), cao 2.9 m |
| Tag | T1…T9 online, T10 offline | UID dạng `1004011717xx`, đời T21M / T21H / T22C |

> Bản đầu của tài liệu ghi IPS Tool nằm trên **PC Windows `192.168.68.126`**. Địa chỉ đó **không
> còn tồn tại trên mạng** (ping mất 100% gói) — đây chính là nguyên nhân lỗi
> `không đọc được anchor qua OpenAPI: [Errno 113] No route to host`. Nếu gặp lại lỗi đó, kiểm
> tra `openapi.base_url` trước tiên.

> IPS Tool **là client push, không phải broker**. Nó chỉ đẩy JSON ra ngoài theo MQTT / UDP /
> Webhook. Broker (nếu dùng MQTT) là thứ mình phải tự dựng ở phía Linux.

---

## 0. Chọn kịch bản

| Bạn muốn | Kịch bản | Phương thức | Lý do |
|---|---|---|---|
| Chỉ xem web dashboard, dựng nhanh nhất | **A** | UDP | không cần broker, không cần `sudo` |
| Chỉ chạy ROS2 + RViz2 | **D** | UDP | như trên |
| Web dashboard **và** ROS2 chạy song song | **G** | MQTT | UDP/Webhook chỉ tới được **một** đích |
| Mạng chặn UDP, muốn debug bằng HTTP | **C** / **F** | Webhook | thấy rõ request, dễ bắt lỗi |
| ROS2 chạy trên robot (máy khác) | **H** | UDP hoặc MQTT | tuỳ có nhiều robot hay không |

## 1. Cài đặt một lần (mọi kịch bản đều cần)

### 1.1 Bên server — PC Windows chạy IPS Tool

| Việc | Cách làm | Đã xong chưa |
|---|---|---|
| IPS Tool chạy, 4 anchor online, đã set toạ độ | web UI `http://192.168.68.126:8080` | ✅ đã kiểm tra |
| **Bật OpenAPI** trong Settings | log không còn `openapi specification is disabled` | ✅ đã kiểm tra (đọc được 4 anchor) |
| IP tĩnh cho card Wi-Fi | `192.168.68.126` | ✅ |
| Firewall Windows: mở **inbound TCP 8080** | chỉ cần khi máy khác gọi OpenAPI | ✅ (Linux gọi được) |
| **Không** cần cài thêm phần mềm nào khác | — | — |

Ghi nhớ: PC này có **2 card mạng** (Wi-Fi `192.168.68.126` + card đi switch PoE dải `172.17.31.x`).
Với UDP **bắt buộc** điền `localAddress`, nếu không gói có thể ra sai card.

### 1.2 Bên máy Linux (192.168.68.142)

```bash
# --- cho web hub ---
pip install paho-mqtt                      # đã có sẵn trên máy này

# --- cho ROS2 ---
source /opt/ros/humble/setup.bash
cd ~/Documents/IPS_A02_MQTT/ros2_ws
colcon build --symlink-install --packages-select ips_a02_ros2
source install/setup.bash                  # phải source lại ở MỖI terminal mới

# --- tường lửa (chỉ khi `sudo ufw status` báo active) ---
sudo ufw allow 9600/udp        # UDP cho web hub
sudo ufw allow 9610/udp        # UDP cho ROS2
sudo ufw allow 8090/tcp        # dashboard + webhook của hub
sudo ufw allow 8091/tcp        # webhook của ROS2
sudo ufw allow 1883/tcp        # chỉ khi dùng MQTT
```

**IP động**: `192.168.68.142` đang lấy DHCP. Đổi IP là IPS Tool đẩy vào chỗ trống — đặt IP tĩnh
hoặc DHCP reservation theo MAC trên router.

### 1.3 Bảng cổng dùng trong hệ

| Cổng | Ai giữ | Dùng cho |
|---|---|---|
| `8080/tcp` | IPS Tool (Windows) | web UI + OpenAPI (anchor, fence, lệnh Call) |
| `8090/tcp` | `ips_hub.py` | web dashboard + webhook của hub (`/ingest`) |
| `9600/udp` | `ips_hub.py` | nhận dữ liệu UDP cho web |
| `9610/udp` | ROS2 `ips_bridge` | nhận dữ liệu UDP cho ROS2 (khác 9600 để không đụng hub) |
| `8091/tcp` | ROS2 `ips_bridge` | webhook của ROS2 |
| `1883/tcp` | mosquitto | broker MQTT |

---

## Kịch bản A — Web dashboard, nguồn UDP  *(nhanh nhất)*

**Bên server**: IPS Tool → icon cấu hình API → chọn **UDP**:

| Field | Giá trị |
|---|---|
| address | `192.168.68.142:9600` |
| localAddress | `192.168.68.126` |

**Bên Linux**:

```bash
cd ~/Documents/IPS_A02_MQTT/web_test
python3 -u ips_hub.py                 # hoặc ./run.sh để tự mở trình duyệt
```

Mở `http://192.168.68.142:8090` → tab **Cài đặt → ① Nguồn dữ liệu vào → UDP** → **Lưu & áp dụng**.

**Xác nhận thành công**: pill nguồn trên thanh tiêu đề chuyển xanh và đếm lên
`UDP :9600 · 1.2k gói`; tag hiện trên bản đồ.

```bash
curl -s localhost:8090/api/state | python3 -m json.tool | head -20   # kiểm tra bằng dòng lệnh
```

---

## Kịch bản B — Web dashboard, nguồn MQTT

**Bên Linux, làm trước** (broker mặc định chỉ nghe `127.0.0.1`, PC Windows không vào được):

```bash
sudo cp ~/Documents/IPS_A02_MQTT/web_test/mosquitto/ips.conf /etc/mosquitto/conf.d/ips.conf
sudo systemctl restart mosquitto
ss -ltn | grep 1883            # phải thấy 0.0.0.0:1883
```

**Bên server**: IPS Tool → cấu hình API → **MQTT**:

| Field | Giá trị |
|---|---|
| address | `mqtt://192.168.68.142:1883` (có auth: `mqtt://user:pass@192.168.68.142:1883`) |
| clientId | để trống hoặc `ips-tool` |
| topic | `ips/uplink` |

**Bên Linux**:

```bash
python3 -u ips_hub.py
```

Dashboard → **Cài đặt → ① Nguồn dữ liệu vào → MQTT**, topic subscribe `ips/#` → **Lưu & áp dụng**.

**Xác nhận**: pill nguồn `MQTT 127.0.0.1:1883 · N gói`. Soi trực tiếp trên broker:

```bash
mosquitto_sub -h 127.0.0.1 -t 'ips/#' -v | head -3
```

---

## Kịch bản C — Web dashboard, nguồn Webhook

**Bên server**: IPS Tool → cấu hình API → **Webhook**, URL `http://192.168.68.142:8090/ingest`, method POST.

**Bên Linux**: `python3 -u ips_hub.py`, dashboard → Cài đặt → **Webhook** → Lưu & áp dụng.

**Xác nhận**:

```bash
curl -X POST -d '{"deviceName":"TEST","name":"Pos","uid":"t","data":{"cnt":1,"pos":[1,1,1],"vel":[0,0,0],"z":1,"time":0,"mapId":1,"fences":[]}}' \
     http://192.168.68.142:8090/ingest
# {"ok": true}  -> tag TEST xuất hiện trên dashboard
```

Trả về `{"ok": false, ... webhook đang tắt}` nghĩa là dashboard đang chọn nguồn khác.

---

## Kịch bản D — ROS2 + RViz2, nguồn UDP

**Bên server**: IPS Tool → **UDP**, address `192.168.68.142:9610`, localAddress `192.168.68.126`.

> Dùng **9610**, không dùng 9600, để không đụng web hub (xem kịch bản G).

**Bên Linux**:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/IPS_A02_MQTT/ros2_ws/install/setup.bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=udp udp_port:=9610 robot_tag:=T1
```

**Xác nhận**: log node in `[udp] đang nghe 0.0.0.0:9610` và `nạp 4 anchor từ IPS Tool`;
RViz2 hiện anchor + tag chạy.

```bash
ros2 topic hz /ips/T1/pose          # ~10 Hz
ros2 topic echo /diagnostics --once | grep -E 'name:|message:'
```

---

## Kịch bản E — ROS2 + RViz2, nguồn MQTT

**Bên Linux, làm trước**: mở broker ra LAN như kịch bản B.

**Bên server**: IPS Tool → **MQTT**, address `mqtt://192.168.68.142:1883`, topic `ips/uplink`.

**Bên Linux**:

```bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py \
    source:=mqtt broker:=192.168.68.142 mqtt_topic:='ips/#' robot_tag:=T1
```

**Xác nhận**: log `[mqtt] đã kết nối 192.168.68.142:1883, subscribe ['ips/#']`.

---

## Kịch bản F — ROS2 + RViz2, nguồn Webhook

**Bên server**: IPS Tool → **Webhook**, URL `http://192.168.68.142:8091/ingest`.

**Bên Linux**:

```bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=webhook webhook_port:=8091
```

**Xác nhận**: log `[webhook] đang nghe http://0.0.0.0:8091/ingest`; thử bằng `curl` như kịch bản C
(đổi cổng thành 8091).

---

## Kịch bản G — Chạy **cả** web dashboard **và** ROS2 cùng lúc

IPS Tool chỉ đẩy tới **một** đích cho mỗi giao thức, nên có 2 cách:

### G1 (khuyến nghị) — MQTT, một nguồn phục vụ cả hai

Broker phát bản sao cho mọi client subscribe. Đã đo thực tế: bơm 60 message → ROS2 nhận 9.8 Hz,
web hub đếm đủ 60, không mất gói bên nào.

**Bên server**: IPS Tool → **MQTT** → `mqtt://192.168.68.142:1883`, topic `ips/uplink`.

**Bên Linux** (3 terminal):

```bash
# 1) broker mở ra LAN (một lần)
sudo cp ~/Documents/IPS_A02_MQTT/web_test/mosquitto/ips.conf /etc/mosquitto/conf.d/ips.conf
sudo systemctl restart mosquitto

# 2) web hub, nguồn = MQTT
cd ~/Documents/IPS_A02_MQTT/web_test && python3 -u ips_hub.py
#    dashboard -> Cài đặt -> Nguồn dữ liệu vào -> MQTT -> Lưu & áp dụng

# 3) ROS2, nguồn = MQTT
source /opt/ros/humble/setup.bash && source ~/Documents/IPS_A02_MQTT/ros2_ws/install/setup.bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=127.0.0.1 mqtt_topic:='ips/#'
```

### G2 — UDP, hai cổng khác nhau

IPS Tool chỉ điền được một địa chỉ UDP, nên chỉ **một trong hai** phía nhận được dữ liệu thật;
phía còn lại chạy simulator để test giao diện. Nếu vẫn muốn dùng UDP:

* web hub: cổng `9600` (mặc định)
* ROS2: `udp_port:=9610`

> ⚠️ **Không được để hai tiến trình cùng cổng UDP.** Đã thử: tiến trình thứ hai **bind thành công,
> không báo lỗi gì**, nhưng nó cướp toàn bộ gói — phía kia nhận 0/5 gói và im lặng. Triệu chứng là
> "dashboard đứng im dù IPS Tool vẫn đang đẩy". Luôn dùng hai cổng khác nhau.

---

## Kịch bản H — ROS2 chạy trên robot (máy khác trong LAN)

**Một robot**: IPS Tool → **UDP**, address = `<ip robot>:9610`, localAddress = `192.168.68.126`.
Trên robot: `ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=udp udp_port:=9610 rviz:=false robot_tag:=T1`
(`rviz:=false` để đỡ nặng; xem RViz từ máy trạm cùng `ROS_DOMAIN_ID`).

**Nhiều robot**: dùng MQTT. IPS Tool đẩy vào broker ở `192.168.68.142`, mỗi robot chạy:

```bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142 \
    mqtt_topic:='ips/#' rviz:=false robot_tag:=T3
```

Mỗi robot đặt `robot_tag` đúng tag gắn trên nó — node chỉ phát TF `map→base_link` cho tag đó,
các tag khác vẫn hiện đầy đủ để tránh va chạm.

Trên robot cần: ROS2 Humble, package đã build (`colcon build --symlink-install`), `paho-mqtt`,
cùng `ROS_DOMAIN_ID` với máy trạm.

---

## Chạy thử khi chưa nối phần cứng

```bash
# web:   dashboard -> Công cụ MQTT -> Simulator -> ▶ Chạy   (hoặc)
python3 -u ips_hub.py --sim

# ROS2:  sinh dữ liệu đúng định dạng hãng, đi đúng phương thức đang chọn
ros2 launch ips_a02_ros2 ips_bringup.launch.py sim:=true sim_tags:=4
```

---

## Không lên dữ liệu — kiểm tra theo thứ tự

| # | Kiểm tra | Lệnh / cách xem | Nếu sai |
|---|---|---|---|
| 1 | Hai máy thấy nhau | `ping 192.168.68.126` | sai mạng Wi-Fi hoặc IP đổi |
| 2 | IPS Tool còn sống, OpenAPI bật | `curl "http://192.168.68.126:8080/openapi/v1/devices?pageSize=5"` | bật OpenAPI trong Settings, mở firewall 8080 |
| 3 | Tag đang online trong IPS Tool | web UI của IPS Tool | sạc tag / kiểm tra anchor |
| 4 | Phía nhận có mở cổng không | `ss -lun \| grep 9600` · `ss -ltn \| grep 8090` | chạy lại hub/node, xem log lỗi bind |
| 5 | Gói có tới máy Linux không | `sudo tcpdump -n -i any udp port 9600` | sai IP/cổng đích, hoặc thiếu `localAddress`, hoặc ufw chặn |
| 6 | Nguồn đang chọn có khớp không | pill trên dashboard / log node | dashboard: Cài đặt → đổi nguồn; ROS2: `source:=` |
| 7 | Đúng topic MQTT | `mosquitto_sub -h 127.0.0.1 -t '#' -v` | sửa topic ở IPS Tool hoặc subscribe `#` |
| 8 | ROS2 có publish không | `ros2 topic hz /ips/T1/pose` | xem log `ips_bridge`, kiểm tra `max_noise` lọc quá tay |

Lỗi hay gặp nhất theo thứ tự: **quên `localAddress`** (UDP), **broker chỉ nghe 127.0.0.1** (MQTT),
**hai tiến trình cùng cổng UDP**, **quên `source install/setup.bash`** ở terminal mới.
