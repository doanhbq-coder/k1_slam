# ips_a02_ros2

**Nooploop IPS A02 (anchor UWB + tag) → ROS 2 → Nav2.** Nhận vị trí tag qua **MQTT / UDP /
Webhook**, hiển thị trên RViz2, và **bấm nút trên tag (hoặc bấm Call trên web IPS Tool) thì robot
tự chạy tới chỗ người cầm tag**.

📄 [`../../NAV2_INTEGRATION.md`](../../NAV2_INTEGRATION.md) — thiết kế chi tiết, số liệu đo, nhật ký
· [`../../GIAI_PHAP_GOI_ROBOT_UWB.md`](../../GIAI_PHAP_GOI_ROBOT_UWB.md) — báo cáo giải pháp
· [`../../HUONG_DAN_KHOI_CHAY.md`](../../HUONG_DAN_KHOI_CHAY.md) — khởi chạy theo từng kịch bản

```
                    ┌── UDP      (address = <ip máy chạy node>:9600)
IPS Tool (Windows) ─┼── MQTT     (qua broker, topic ips/#)     ──► ips_bridge ──┬─► RViz2
                    └── Webhook  (POST http://<ip>:8091/ingest)                 │
                              HTTP OpenAPI (anchor, vùng ảo, ack) ──────────►   │
                                                                                ▼
                                   ips_map_tf ──► TF map→ips_map ──► ips_call_navigator
                                        ▲                                       │
                                   ips_align (hiệu chuẩn 1 lần)     NavigateToPose ──► Nav2
```

## 1. Node trong package

| Executable | Node | Việc |
|---|---|---|
| `ips_bridge` | `ips_bridge` | IPS Tool → ROS 2: pose, twist, marker, `/ips/events`, diagnostics |
| `ips_align` | `ips_align` | **Khớp hệ toạ độ bằng cách bấm 2 điểm trên bản đồ trong RViz** |
| `ips_calib` | `ips_calib` | Khớp hệ toạ độ bằng cách lái robot tới N điểm (chính xác hơn, lâu hơn) |
| `ips_map_tf` | `ips_map_tf` | Phát static TF `map → ips_map` từ kết quả hiệu chuẩn |
| `ips_call_nav` | `ips_call_navigator` | Nút bấm → lọc → tính điểm dừng → goal cho Nav2 |
| `ips_sim` | `ips_sim` | Giả lập IPS Tool (kể cả nút bấm) khi chưa có phần cứng |

Kiểu message riêng nằm ở package `ips_a02_msgs` (ament_cmake, chỉ chứa `msg/IpsEvent.msg`).

## 2. Build

**Thứ tự bắt buộc lần đầu** — package msg phải có trước, và phải chạy **từ gốc workspace**:

```bash
cd ~/Documents/neo_k1
colcon build --packages-select ips_a02_msgs
source install/setup.bash
colcon build --packages-select ips_a02_ros2 --symlink-install
source install/setup.bash
```

> Chạy `colcon build` khi shell đang đứng trong `src/...` sẽ tạo `build/ install/ log/` lồng
> trong `src/`, rồi `source install/setup.bash` nạp nhầm overlay thiếu `ips_a02_msgs`. Luôn `cd`
> về gốc workspace.

> `--symlink-install` để sửa `config/*.yaml`, `rviz/*.rviz`, launch là có hiệu lực ngay, và để
> kết quả hiệu chuẩn được ghi vào `src/` thay vì `install/` (xoá `install/` là mất).

Cần `paho-mqtt` khi dùng MQTT: `sudo apt install python3-paho-mqtt`.

## 3. Chạy thử ngay, chưa cần phần cứng

```bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py sim:=true
ros2 service call /ips_sim/press  std_srvs/srv/Trigger    # giả lập bấm nút gọi
ros2 service call /ips_sim/cancel std_srvs/srv/Trigger
```

`ips_sim` sinh `Pos` đúng định dạng hãng trong khung 4 anchor thật (3.6 × 10.2 m), đẩy qua đúng
phương thức đang chọn.

## 4. Chạy thật — chỉ hiển thị

```bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py                                     # UDP
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=webhook webhook_port:=8091
```

Launch arg để trống thì **không** ghi đè yaml — `config/ips_params.yaml` là nguồn sự thật, chỉ
những gì bạn gõ trên dòng lệnh mới đè lên. Node in ra lúc khởi động đúng dòng cần điền vào IPS
Tool:

| `source` | Trong IPS Tool (icon cấu hình API) | Cần gì thêm |
|---|---|---|
| `udp` (mặc định) | **UDP**: address = `<ip máy chạy node>:9600`, localAddress = IP card Wi-Fi của PC IPS Tool | không |
| `mqtt` | **MQTT**: address = `mqtt://<ip broker>:1883`, topic = `ips/uplink` | broker chạy sẵn, nghe `0.0.0.0` |
| `webhook` | **Webhook**: URL = `http://<ip máy chạy node>:8091/ingest`, POST | không |

PC IPS Tool có 2 card mạng nên với UDP **phải** điền `localAddress`, không thì gói ra sai card.

## 5. Chạy thật — gọi robot tới chỗ người cầm tag

### 5.1. Hiệu chuẩn một lần (bấm 2 điểm trên bản đồ)

Cần Nav2 + bản đồ của robot đang chạy và robot đã localize.

```bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142 rviz:=false
ros2 run   ips_a02_ros2 ips_align
rviz2 -d $(ros2 pkg prefix ips_a02_ros2)/share/ips_a02_ros2/rviz/ips_nav2.rviz
```

Trong RViz dùng công cụ **Publish Point**: bấm chân anchor **A1** → hiện dấu `A1 ✓`; bấm chân
**A2** → cả 4 anchor + khung bao hiện lên. Chưa ưng thì bấm lại 2 điểm (ghi đè), hoặc kéo/xoay
interactive marker ở gốc hệ IPS để chỉnh tay. Lưu:

```bash
ros2 service call /ips_align/save std_srvs/srv/Trigger
```

A1–A2 chỉ cách nhau 3.6 m nên **bấm lệch 10 cm là khung lệch ~3.2°** — phóng to RViz hết cỡ khi
bấm. Muốn chắc hơn: `-p click_anchors:='[A1, A3]'` (đường chéo 10.82 m) hoặc bấm cả 4 anchor
`-p click_anchors:='[A1, A2, A3, A4]'` để node báo **rms**.

Service khác của `ips_align`: `~/reload`, `~/clear_clicks`, `~/status`.

**Tắt `ips_align` trước khi sang bước sau** — nó cũng phát static TF `map → ips_map`.

### 5.2. Chạy hàng ngày

```bash
ros2 launch ips_a02_ros2 ips_nav2.launch.py source:=mqtt broker:=192.168.68.142
ros2 topic echo /ips/nav_status
```

Gọi robot bằng **giữ nút trên tag ~2 giây**, hoặc **bấm `Call` trên web IPS Tool**. Huỷ bằng nút
huỷ trên tag hoặc `CancelCall` trên web.

## 6. Topic & TF

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `/ips/<tag>/pose` | `PoseStamped` | vị trí thô, frame `ips_map` |
| `/ips/<tag>/pose_cov` | `PoseWithCovarianceStamped` | covariance từ `posVar` — dùng cho EKF |
| `/ips/<tag>/twist` | `TwistStamped` | vận tốc từ IPS Tool |
| `/ips/tag_poses` | `PoseArray` | tất cả tag |
| `/ips/tags` | `MarkerArray` | cầu + nhãn + vệt + vòng nhiễu + mũi tên vận tốc |
| `/ips/anchors`, `/ips/fences` | `MarkerArray` **transient_local** | từ OpenAPI, latched |
| `/ips/align/anchors` | `MarkerArray` | xem trước lúc hiệu chuẩn (chỉ khi chạy `ips_align`) |
| `/ips/events` | `ips_a02_msgs/IpsEvent` | **nút bấm** + vị trí tag ngay lúc bấm |
| `/ips/nav_status` | `std_msgs/String` | `NHẬN…` `TỪ CHỐI… vì…` `TỚI NƠI` `đã huỷ` |
| `/diagnostics` | `DiagnosticArray` | Hz, mất gói, nhiễu, RSSI, tuổi mẫu, trạng thái transport |

TF: `map → ips_map` (static, từ hiệu chuẩn) → `ips_<tag>` cho mọi tag.

> `/ips/anchors` và `/ips/fences` chỉ publish **khi danh sách thay đổi**. Display RViz cho hai
> topic này **phải để `Durability Policy: Transient Local`**, để `Volatile` là RViz mở sau sẽ
> không thấy gì.

## 7. Tham số hay dùng (`config/ips_params.yaml`)

**`/ips_bridge`**

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `source` | `udp` | `udp` \| `mqtt` \| `webhook` |
| `frame_id` | `ips_map` | frame gốc của toạ độ IPS; `map` do `ips_map_tf` nối vào |
| `event_kinds` | `[Call, CancelCall, Ack]` | loại message đẩy ra `/ips/events` |
| `use_opt_poses` | `false` | `true` = dùng `optPoses` (đã ràng buộc tường) thay `pos` raw |
| `max_noise` | `0.0` | `>0`: bỏ mẫu có `noiseRadius` vượt ngưỡng (m) |
| `openapi.base_url` | `http://192.168.68.142:8080` | nơi lấy anchor + vùng ảo |

**`/ips_call_navigator`** — số mặc định lấy từ đo thật, xem `NAV2_INTEGRATION.md` mục B.3

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `standoff` | `0.9` | dừng cách người bao nhiêu mét (≥ bán kính robot + 0.5) |
| `max_pose_age` | `3.0` | Pos về ~1 Hz nên đừng đặt 1.5, sẽ loại cả lệnh hợp lệ |
| `max_noise` | `0.5` | đo thực tế 0.11–0.28 m |
| `rearm_gap` | `3.0` | tag lặp Call ~1 Hz → im ngần này giây mới tính lần bấm mới |
| `queue_policy` | `preempt` | `preempt` = lệnh mới huỷ lệnh cũ \| `ignore` |
| `check_anchor_box` | `false` | ngoài khung anchor vẫn cho gọi (đo thấy dùng được) |
| `follow` | `false` | `true` = bám theo người suốt phiên gọi |
| `send_ack` | `true` | báo ngược về tag → tag rung/nháy |
| `trigger_on_ack` | `true` | nhận lệnh từ **web IPS Tool** (xem mục 8) |

## 8. Điều khiển từ web IPS Tool

Bấm `Call` / `CancelCall` trên web của hãng **không** sinh uplink `Call`, mà sinh `Ack` kèm
`reqId` — đây là đường duy nhất đọc được lệnh từ server (OpenAPI không có endpoint sự kiện nào):

| Bấm trên web | Đẩy lên `ips/uplink` |
|---|---|
| Call | `{"name":"Ack","data":{"reqId":"CallAck"},…}` |
| CancelCall | `{"name":"Ack","data":{"reqId":"CancelCallAck"},…}` |

`ips_call_navigator` hiểu hai cái này như lệnh đi / lệnh huỷ. Vì chính node cũng gửi
`Ack(CallAck)` xuống tag, `ack_echo_window` (5 s) chặn tiếng vọng — chặn **đúng reqId vừa gửi**,
không chặn cả cụm, nếu không cú `CancelCall` bấm ngay sau `Call` cũng bị nuốt.

## 9. Ghép UWB vào bộ định vị của robot (tuỳ chọn, không dùng cho bài toán gọi robot)

Vị trí UWB là **phép đo tuyệt đối, ~1 Hz (đo thật), không có hướng**. Muốn dùng để định vị chính
robot thì fuse qua `robot_localization`, đừng lấy thẳng làm `map→odom`:

```yaml
# ekf.yaml — nhận UWB như một nguồn pose tuyệt đối
pose0: /ips/T1/pose_cov
pose0_config: [true,  true,  false,     # x, y, z
               false, false, false,     # roll, pitch, yaw  (UWB không đo được)
               false, false, false,
               false, false, false,
               false, false, false]
pose0_differential: false
pose0_relative: false
```

Với bài toán gọi robot thì **không cần**: robot tự định vị bằng lidar như cũ, UWB chỉ dùng để
biết người đang ở đâu. Đây là kiến trúc ít rủi ro nhất — giữ nguyên.

## 10. Trục trặc

| Hiện tượng | Xử lý |
|---|---|
| RViz trống, không có tag | `ros2 topic hz /ips/<tag>/pose`; không có ⇒ IPS Tool chưa push hoặc sai địa chỉ/cổng |
| Có tag nhưng **không có anchor** | (a) sai `openapi.base_url` — xem log `không đọc được anchor`; (b) display RViz để `Volatile` thay vì `Transient Local` |
| `ips_map_tf` thoát ngay, mã 1 | chưa hiệu chuẩn — chạy `ips_align` trước |
| Anchor lệch khỏi góc phòng | bản đồ đã đổi (SLAM chạy lại ở mapping mode) ⇒ hiệu chuẩn lại |
| Bấm nút mà robot không đi | xem `/ips/nav_status`, node luôn nói rõ lý do từ chối |
| `ParameterUninitializedException` lúc khởi động | yaml có **list rỗng** `[]` — ROS 2 không suy được kiểu; các node đã có fallback, nhưng nếu tự thêm tham số mới thì nhớ bẫy này |
| `Logger severity cannot be changed between calls` | gọi `getattr(logger, level)` từ cùng một dòng với nhiều mức khác nhau; mỗi mức phải một dòng riêng |
| Đổi `config/*.yaml` không ăn | build thiếu `--symlink-install`; xoá `build/ install/` rồi build lại |
| Tag nhảy hoặc lệch | tăng `max_noise` để lọc, hoặc bật `use_opt_poses`; xem `/diagnostics` |
| Cổng UDP bị chiếm | đổi `udp_port:=9610` |
