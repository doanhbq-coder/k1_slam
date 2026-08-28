# ips_a02_ros2 → Nav2: tài liệu bàn giao & kế hoạch tích hợp

> **File này dùng để làm gì**: copy vào workspace mới cùng với `src/ips_a02_ros2/`.
> Đọc hết file là biết (1) package hiện có gì, chạy ra sao, (2) mục tiêu cần đạt,
> (3) chính xác phải viết thêm node nào, sửa file nào, theo thứ tự nào, (4) nghiệm thu thế nào.
>
> Trạng thái: **Phần A chạy ổn định. B1 đã code xong + test bằng dữ liệu giả, chờ hiệu chuẩn
> thật trên robot. B2, B3 chưa có dòng code nào.**

---

## 0. Mục tiêu cuối cùng

```
4 anchor A02 (UWB) ──► IPS Tool (Windows) ──► ips_bridge ──► vị trí tag trong hệ IPS
                                                                    │
                                              [ calibration map ↔ ips ]
                                                                    ▼
                                                    vị trí tag trên bản đồ Nav2 (frame `map`)
                                                                    │
       người bấm nút trên tag ──► sự kiện `Call` ────────────────────┤
                                                                    ▼
                                            NavigateToPose goal ──► Nav2 ──► robot chạy tới chỗ người
       bấm lại / bấm huỷ  ──► `CancelCall` ──────────────────────► cancel goal
```

Ba việc phải làm, đúng thứ tự này (việc sau phụ thuộc việc trước):

| # | Việc | Trạng thái |
|---|---|---|
| B1 | Đưa 4 anchor + hệ toạ độ IPS **khớp** vào bản đồ Nav2 (`map`) | ⏳ code xong, chờ hiệu chuẩn thật |
| B2 | Bridge phát sự kiện nút bấm (`Call`/`CancelCall`) ra ROS topic | ❌ chưa có (đang chỉ `log`) |
| B3 | Node nhận `Call` → lấy vị trí tag → gửi goal cho Nav2 | ❌ chưa có |

---

# PHẦN A — Hiện trạng (đã chạy được)

## A.1. Phần cứng & địa chỉ mạng thực tế

| Vai trò | Máy | Địa chỉ |
|---|---|---|
| Server định vị — IPS Tool (Windows) | PC server | `192.168.68.126`, web UI + OpenAPI cổng `8080` |
| Máy Linux chạy ROS 2 Humble | máy dev | `192.168.68.142` |
| Anchor A02 | 4 cái | A1 (0, 0) · A2 (3.6, 0) · A3 (3.6, −10.2) · A4 (0, −10.2), cao **2.9 m** |
| Tag UWB | T1…T9 online | UID dạng `1004011717xx` |

**Quan trọng**: IPS Tool là **client push**, không phải broker. Nó tự đẩy JSON ra ngoài bằng
UDP / MQTT / Webhook. Broker MQTT (nếu dùng) là thứ mình phải tự dựng ở phía Linux.

Khung 4 anchor tạo ra một hệ toạ độ riêng của IPS: gốc ở A1, vùng phủ 3.6 × 10.2 m,
**y âm** (A3, A4 có y = −10.2). Hệ này **không trùng** với frame `map` của Nav2 → xem B1.

## A.2. Cấu trúc package hiện tại

```
ros2_ws/src/ips_a02_ros2/
├── package.xml                    ament_python, deps: rclpy geometry_msgs visualization_msgs
│                                  diagnostic_msgs std_msgs tf2_ros rviz2 python3-paho-mqtt
├── setup.py                       entry_points: ips_bridge, ips_sim
├── config/ips_params.yaml         toàn bộ tham số của ips_bridge
├── launch/ips_bringup.launch.py   bridge + sim (tuỳ chọn) + rviz
├── rviz/ips_a02.rviz              Grid, IPS Anchors, IPS Fences, IPS Tags, TF; view TopDownOrtho
└── ips_a02_ros2/
    ├── bridge_node.py   (394 dòng)  node chính
    ├── transports.py    (222 dòng)  UdpTransport / MqttTransport / WebhookTransport
    ├── openapi.py       ( 51 dòng)  đọc anchor + fence qua HTTP OpenAPI (chỉ GET)
    ├── markers.py       (153 dòng)  dựng MarkerArray cho RViz
    └── sim_node.py      (136 dòng)  giả lập IPS Tool khi chưa có phần cứng
```

## A.3. Build & chạy

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ips_a02_ros2
source install/setup.bash

# chạy thử không cần phần cứng
ros2 launch ips_a02_ros2 ips_bringup.launch.py sim:=true

# chạy thật
ros2 launch ips_a02_ros2 ips_bringup.launch.py                                    # UDP (mặc định)
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=webhook webhook_port:=8091
```

`--symlink-install` là bắt buộc nếu muốn sửa `config/*.yaml` mà không build lại.
Cần `paho-mqtt` khi dùng MQTT: `sudo apt install python3-paho-mqtt`.

Node in ra ngay khi khởi động dòng cần điền vào IPS Tool (icon cấu hình API):

| `source` | Điền vào IPS Tool | Cần thêm |
|---|---|---|
| `udp` | address = `<ip máy ROS>:9600`, **localAddress = IP card Wi-Fi của PC IPS Tool** | không |
| `mqtt` | address = `mqtt://<ip broker>:1883`, topic = `ips/uplink` | broker nghe `0.0.0.0` |
| `webhook` | URL = `http://<ip máy ROS>:8091/ingest`, POST | không |

> PC chạy IPS Tool có 2 card mạng (Wi-Fi + card đi switch PoE), nên với UDP **bắt buộc** điền
> `localAddress`, không thì gói ra sai card.

## A.4. Topic / TF hiện có

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `/ips/<tag>/pose` | `geometry_msgs/PoseStamped` | vị trí thô, frame = tham số `frame_id` |
| `/ips/<tag>/pose_cov` | `PoseWithCovarianceStamped` | covariance từ `posVar` — dùng cho EKF |
| `/ips/<tag>/twist` | `TwistStamped` | vận tốc từ IPS Tool |
| `/ips/tag_poses` | `PoseArray` | tất cả tag, xem nhanh |
| `/ips/tags` | `MarkerArray` | cầu + nhãn + vệt + vòng nhiễu + mũi tên vận tốc |
| `/ips/anchors`, `/ips/fences` | `MarkerArray` (transient_local) | lấy từ OpenAPI, latched |
| `/diagnostics` | `DiagnosticArray` | Hz, mất gói theo `cnt`, nhiễu, RSSI, tuổi mẫu, trạng thái transport |

TF: `map → ips_<tag>` cho mọi tag; thêm `map → base_link` nếu đặt `robot_tag`
(**cách này bỏ qua bộ lọc, chỉ để nhìn cho vui — xem A.7**).

## A.5. Tham số quan trọng (`config/ips_params.yaml`)

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `source` | `udp` | `udp` \| `mqtt` \| `webhook` |
| `frame_id` | `map` | frame gốc của toạ độ IPS — **sẽ đổi thành `ips_map` ở B1** |
| `tag_frame_prefix` | `ips_` | tiền tố frame con của mỗi tag |
| `robot_tag` | `""` | tag gắn trên robot → phát thêm TF `map→base_link` |
| `use_opt_poses` | `false` | `true` = dùng `optPoses` (đã nắn theo tường) thay `pos` raw |
| `yaw_from_velocity` | `true` | UWB không đo hướng; suy yaw từ vector vận tốc khi đang chạy |
| `max_noise` | `0.0` | `>0`: bỏ mẫu có `noiseRadius` vượt ngưỡng (m) |
| `offline_after` | `3.0` | quá ngần này giây không có mẫu ⇒ coi tag mất tín hiệu |
| `openapi.base_url` | `http://192.168.68.126:8080` | nơi lấy anchor + vùng ảo |
| `openapi.poll_period` | `30.0` | chu kỳ poll anchor/fence |

## A.6. Định dạng dữ liệu IPS Tool (tóm tắt — chi tiết ở `CAU_TRUC_JSON.md`)

Phong bì chung (UDP/MQTT/Webhook đều giống nhau, **có thể là 1 object hoặc 1 mảng object**):

```json
{ "deviceName": "T154", "name": "Pos", "uid": "2189E233324B", "data": { … } }
```

- `uid` là định danh thật → **dùng làm key**. `deviceName` đổi được, chỉ để hiển thị.
- `name` ∈ `Pos` | `Call` | `CancelCall` | `Ack`. `data` chỉ có ở `Pos` và `Ack`.

`data` của `Pos`, các field dùng được:

| Field | Kiểu | Ghi chú |
|---|---|---|
| `pos` | `[x,y,z]` m | toạ độ **thô** |
| `optPoses` | `[[x,y,z]]` m | đã nắn theo tường — **lồng 2 tầng**, lấy `optPoses[0]` |
| `posVar` | `[m²,m²,m²]` | **phương sai**, phải `sqrt()` mới ra mét |
| `noiseRadius` | m | chỉ số nhiễu tổng hợp → ngưỡng lọc đặt ở đây |
| `vel`, `velVar` | m/s | **không có hướng** trong payload |
| `rxRssi` | dBm | anchor thu mạnh nhất |
| `fences` | `[{id,name}]` | vùng ảo đang đứng trong đó — là **trạng thái**, không phải sự kiện |
| `cnt` | int | bộ đếm theo từng tag → đếm gói thiếu |
| `time` | ms epoch | theo đồng hồ PC server, cần NTP mới dùng được |

Ba message còn lại — **đây là chỗ nút bấm trên tag xuất hiện**:

```json
{ "deviceName": "T4", "name": "Call",       "uid": "ea22…" }   ← bấm nút gọi
{ "deviceName": "T4", "name": "CancelCall", "uid": "ea22…" }   ← bấm huỷ
{ "deviceName": "T4", "name": "Ack", "uid": "ea22…", "data": { "reqId": "CallAck" } }
```

HTTP OpenAPI `http://<server>:8080/openapi/v1`, response luôn bọc
`{ "code": 0, "message": "", "data": {…} }` — **kiểm tra `code == 0` trước khi đọc `data`**:

| Method | Endpoint | Ghi chú |
|---|---|---|
| GET | `/devices?deviceType=4` | anchor (toạ độ ở `coordinate.coords`) |
| GET | `/devices?deviceType=5` | tag |
| GET | `/fences` | vùng ảo, `bounds` lồng **3 tầng**, không tự khép kín |
| PUT | `/devices/{uid}/services/Call` | `{"data":{"duration":1}}` — giây, tối đa 64 |
| PUT | `/devices/{uid}/services/CancelCall` | không body |
| PUT | `/devices/{uid}/services/Ack` | `{"data":{"reqId":"CallAck"}}` ← **dùng ở B3 để phản hồi tag** |
| PUT | `/devices/{uid}/services/TransmitData` | `{"data":{"data":"123","codec":"utf8"}}` |

> OpenAPI phải **bật trong Settings** của IPS Tool, nếu chưa thì log báo
> `openapi specification is disabled`. Tài liệu hãng không mô tả cơ chế xác thực nào.

## A.7. Ghép UWB vào robot — nguyên tắc đúng

Vị trí UWB là **phép đo tuyệt đối, ~10 Hz, không có hướng, nhiễu vài chục cm**.
Đừng lấy thẳng làm `map→odom`. Cách đúng là fuse qua `robot_localization`:

```yaml
# ekf.yaml — UWB như một nguồn pose tuyệt đối
pose0: /ips/T1/pose_cov
pose0_config: [true,  true,  false,     # x, y, z
               false, false, false,     # roll, pitch, yaw — UWB không đo được
               false, false, false,
               false, false, false,
               false, false, false]
pose0_differential: false
pose0_relative: false
```

Bridge đã set sẵn covariance: `posVar` cho x/y/z, `1e6` cho roll/pitch, `0.5` cho yaw khi bật
`yaw_from_velocity` (đứng yên thì yaw vô nghĩa → nên tắt yaw như trên).

**Nhưng cho bài toán này (B1–B3): robot đã có Nav2 + AMCL/SLAM chạy ổn rồi, nên KHÔNG cần fuse
UWB vào robot.** UWB chỉ dùng để định vị **người/tag đi bộ**, còn robot vẫn tự định vị bằng
lidar như cũ. Đây là kiến trúc gọn nhất và ít rủi ro nhất — giữ nguyên như vậy.

---

# PHẦN B — Việc cần làm

## B.0. Kiến trúc đích

```
                         ┌──────────────────────────────────────────────┐
IPS Tool ──udp/mqtt/wh──►│ ips_bridge  (SỬA: thêm publish /ips/events)   │
                         └───────┬──────────────────────────┬───────────┘
                                 │ /ips/<tag>/pose          │ /ips/events
                                 │ (frame: ips_map)         │ (Call/CancelCall/Ack)
                                 ▼                          ▼
        ┌────────────────────────────────┐   ┌──────────────────────────────────┐
        │ ips_map_tf   (MỚI)             │   │ ips_call_navigator   (MỚI)       │
        │ static TF  map → ips_map       │   │ Call → chọn goal → NavigateToPose│
        │ đọc config/ips_map_transform   │   │ CancelCall → cancel goal         │
        └────────────┬───────────────────┘   │ + Ack ngược lại tag qua OpenAPI  │
                     │                       └──────────────┬───────────────────┘
                     │ tf2                                  │ action
                     ▼                                      ▼
        ┌────────────────────────────────┐   ┌──────────────────────────────────┐
        │ ips_map_calibrator (MỚI, chạy  │   │ Nav2 (bt_navigator, AMCL, …)     │
        │ 1 lần) → sinh file transform   │   │ đã chạy sẵn, KHÔNG sửa gì        │
        └────────────────────────────────┘   └──────────────────────────────────┘
```

Nguyên tắc: **không sửa gì bên Nav2 của robot.** Toàn bộ phần thêm nằm trong package này,
nối vào Nav2 chỉ qua đúng một cửa: action `/navigate_to_pose`.

## B.1. Khớp hệ toạ độ IPS vào bản đồ Nav2

### Vấn đề

Có 2 hệ toạ độ hoàn toàn độc lập:

- **`ips_map`** — gốc ở anchor A1, do người lắp anchor định nghĩa, đơn vị mét, y âm về phía A3/A4.
- **`map`** — gốc do SLAM đặt ở chỗ robot bắt đầu quét, hướng tuỳ lúc đó.

Chúng lệch nhau một phép **quay + tịnh tiến trong mặt phẳng** (3 tham số: `θ, tx, ty`).
Không có scale (cả hai đều là mét thật) và không có lật trục — nếu tính ra scale ≠ 1 hoặc
det < 0 thì chắc chắn đã nhập sai dữ liệu.

### Cách làm — khuyến nghị: hiệu chuẩn bằng chính robot (N điểm)

Gắn một tag lên robot (gọi là `calib_tag`), rồi:

1. Cho robot đứng yên ở một điểm trong vùng phủ anchor.
2. Ghi lại **cặp** (vị trí tag trong `ips_map`, vị trí robot trong `map` lấy từ TF `map→base_link`).
3. Lặp lại ở **≥ 4 điểm**, trải rộng ra 4 góc vùng phủ (càng xa nhau càng chính xác;
   4 điểm gần nhau cho kết quả rất tệ).
4. Giải bài toán Umeyama/Kabsch 2D → ra `θ, tx, ty` + sai số RMS.
5. Ghi ra `config/ips_map_transform.yaml`.

Cách này chính xác hơn hẳn việc đo tay vị trí anchor trên bản đồ, vì nó dùng đúng nguồn dữ
liệu sẽ dùng lúc chạy thật (bao gồm cả sai lệch hệ thống của UWB).

> **Fallback đo tay**: nếu chưa có tag rảnh để gắn lên robot — mở bản đồ trong RViz, dùng
> `Publish Point` bấm vào đúng chân 4 anchor, ghi lại toạ độ, ghép cặp với toạ độ anchor trong
> hệ IPS (0,0) (3.6,0) (3.6,−10.2) (0,−10.2) rồi giải cùng công thức. Sai số sẽ lớn hơn.

### Đã cài đặt

| File | Vai trò |
|---|---|
| `ips_a02_ros2/calib.py` | `rigid_2d` (Umeyama 2D, chặn phản xạ gương), `residuals`, `apply_2d` |
| `ips_a02_ros2/calib_node.py` | node `ips_calib` — lái robot, chụp cặp điểm, giải, ghi yaml |
| `ips_a02_ros2/align_node.py` | node `ips_align` — **bấm 2 điểm trên RViz**, kéo/xoay tinh chỉnh |
| `ips_a02_ros2/transform_io.py` | đọc/ghi `ips_map_transform.yaml` (ghi nguyên tử qua file .tmp) |
| `ips_a02_ros2/map_tf_node.py` | node `ips_map_tf` — đọc yaml, phát static TF `map → ips_map` |
| `ips_a02_ros2/paths.py` | quyết định chỗ đọc/ghi `ips_map_transform.yaml` |
| `test/test_calib.py` | 5 test cho `rigid_2d`, chạy trực tiếp bằng `python3` |
| `rviz/ips_nav2.rviz` | fixed frame `map`, thêm Map + Global Costmap + Path + LaserScan |

**Khác với bản kế hoạch ở trên** — ghi lại để khỏi ngạc nhiên:

- `base_frame` mặc định là **`base_footprint`** (không phải `base_link`): đó là frame robot K1
  đang dùng, xem `k1_slam/config/k1_nav2_mppi.yaml`.
- Node **không chặn** trong service `capture`. Nó chạy hai vòng đệm liên tục (mẫu IPS 
  và TF `map→base_footprint` lấy ở 10 Hz), lúc gọi `capture` chỉ việc lấy các mẫu trong
  `settle_time` giây gần nhất. Nhờ vậy không cần `MultiThreadedExecutor`.
- Cửa sổ mẫu cắt theo **timestamp trong message**, không theo lúc nhận. `TransformListener`
  trả "mới nhất" có thể trễ vài trăm ms; tính theo giờ nhận sẽ vớ phải mẫu của điểm trước
  và báo nhầm "robot đang di chuyển". (Đã dính lỗi này lúc test, đây là cách sửa.)
- Có thêm kiểm tra **robot đứng yên**: biên độ TF trong cửa sổ phải < `still_threshold`
  (5 cm), không thì từ chối chụp.
- Ngoài `capture` / `compute` / `reset` còn có `undo` (bỏ điểm cuối) và `list` (xem các điểm
  đã chụp) — cần khi một điểm chụp hỏng mà không muốn làm lại từ đầu.
- Đường thủ công không dùng `/clicked_point` mà dùng tham số `manual_pairs`
  (mảng phẳng `ips_x, ips_y, map_x, map_y, ...`), đọc lại tại thời điểm gọi `compute`
  nên `ros2 param set` xong gọi luôn được.
- File kết quả ghi vào **`src/ips_a02_ros2/config/`** nếu workspace build bằng
  `--symlink-install` (dò qua symlink của `ips_params.yaml`), không thì đành ghi vào
  `install/`. Ghi vào `install/` thì `rm -rf install/` là mất kết quả hiệu chuẩn.

## B1a. Cách khớp bằng mắt trên RViz — `ips_align` (khuyến nghị dùng trước)

Hai anchor là **đủ** để xác định trọn vẹn 3 tham số (θ, tx, ty). Nên quy trình gọn nhất là:
bấm `Publish Point` vào chân 2 anchor trên bản đồ, cả 4 anchor hiện lên ngay, sai thì kéo lại
bằng tay. Không phải lái robot, và quan trọng hơn: **nhìn thấy đúng/sai ngay lập tức**.

```bash
# Nav2 + bản đồ của robot đang chạy (chỉ cần /map và TF map, không cần localize)
ros2 run ips_a02_ros2 ips_align
ros2 run ips_a02_ros2 ips_align --ros-args -p click_anchors:='[A1, A3]'          # đường chéo 10.82 m, nhạy hơn 3 lần
ros2 run ips_a02_ros2 ips_align --ros-args -p click_anchors:='[A1, A2, A3, A4]'  # bấm cả 4, có rms


rviz2 -d install/ips_a02_ros2/share/ips_a02_ros2/rviz/ips_nav2.rviz
```

Node tự lấy toạ độ 4 anchor qua OpenAPI. Thao tác **luôn cố định: bấm A1 trước, A2 sau**
(tham số `click_anchors`, mặc định `["A1", "A2"]`).

| Việc | Cách làm | Thấy gì trên RViz |
|---|---|---|
| bấm A1 | `Publish Point` vào chân anchor A1 | dấu tròn xanh lá + nhãn `A1 ✓`, kèm chữ `giờ bấm vào A2` |
| bấm A2 | `Publish Point` vào chân anchor A2 | `A2 ✓` + **cả 4 anchor + khung bao** hiện lên |
| bấm lại | bấm A1, A2 lần nữa | vòng bấm mới ghi đè vòng cũ; khung cũ vẫn hiện trong lúc bấm dở, không bị mất hình |
| tinh chỉnh | kéo mặt phẳng / vòng xoay của interactive marker ở gốc hệ IPS | 4 anchor + khung chạy theo thời gian thực |
| lưu | `ros2 service call /ips_align/save std_srvs/srv/Trigger` hoặc chuột phải marker → "Lưu vào file" | |
| huỷ click đang dở | `ros2 service call /ips_align/clear_clicks std_srvs/srv/Trigger` | |
| xem / nạp lại | service `~/status`, `~/reload` | |

Trước khi khớp lần đầu node **không vẽ anchor** — vẽ khi chưa có phép biến đổi thì 4 chấm nằm ở
chỗ vô nghĩa, nhìn tưởng đã khớp rồi.

**Về độ chính xác của cặp A1–A2**: hai anchor này chỉ cách nhau 3.60 m (cạnh ngắn của khung),
trong khi vùng phủ dài 10.2 m. Sai số góc tỉ lệ nghịch với khoảng cách hai điểm bấm: đo thực tế
bấm lệch 10 cm → khung lệch **3.2°**, đầu xa của vùng phủ lệch ~0.6 m. Khắc phục, chọn một
trong ba:

- phóng to RViz hết cỡ khi bấm (lệch 2-3 cm thì khung chỉ lệch dưới 1°);
- bấm cặp đường chéo: `-p click_anchors:='[A1, A3]'` — cách nhau 10.82 m, nhạy hơn 3 lần;
- bấm cả 4: `-p click_anchors:='[A1, A2, A3, A4]'` — node giải bình phương tối thiểu sau mỗi
  click từ điểm thứ 2 và **báo rms**, là cách duy nhất biết mình bấm chuẩn tới đâu.

Kiểm tra tự động: sau 2 click node so **khoảng cách giữa 2 điểm vừa bấm** với khoảng cách thật
giữa 2 anchor. Lệch > 15% (`dist_tolerance`) là báo lỗi đỏ — gần như chắc chắn bấm nhầm chỗ.
Nhưng **phép kiểm tra này không bắt được lỗi bấm nhầm sang đường chéo kia** (hai đường chéo dài
bằng nhau), nên vẫn phải nhìn hình 4 anchor có trùng 4 góc phòng không.

Tham số: `openapi_url`, `anchors` + `anchor_names` (dự phòng khi OpenAPI hỏng, dạng phẳng
`[x, y, x, y, ...]`), `click_anchors` (tự chọn thứ tự bấm), `map_frame`, `ips_frame`,
`marker_size`, `dist_tolerance`, `load_existing`, `output`.

> `ips_align` **tự phát** static TF `map → ips_map` trong lúc chỉnh, nên **đừng chạy cùng lúc
> với `ips_map_tf`** — hai node cùng phát một TF thì RViz nhận cái đến sau, nhìn rất khó hiểu.
> Chỉnh xong, `save`, tắt `ips_align`, rồi mới chạy `ips_map_tf`.

### `ips_align` hay `ips_calib`?

| | `ips_align` (bấm RViz) | `ips_calib` (lái robot) |
|---|---|---|
| Khớp theo | vị trí **vật lý** của anchor vẽ trên bản đồ | chính **dữ liệu UWB** đo được |
| Cần | biết anchor nằm ở đâu trên bản đồ | tag gắn trên robot + robot đã localize |
| Nuốt được sai lệch hệ thống của UWB | không | có |
| Kiểm chứng | nhìn thấy ngay | phải chạy xong mới biết rms |
| Thời gian | ~1 phút | ~10 phút |

Làm `ips_align` trước cho nhanh và chắc. Nếu sau này thấy chấm tag lệch đều một hướng so với
thực tế (dấu hiệu UWB có bias hệ thống), chạy thêm `ips_calib` để so.

### Tham số của `ips_calib`

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `calib_tag` | `T1` | tên tag gắn trên robot (đúng `deviceName` trong IPS Tool) |
| `pose_topic` | `""` | để trống = `ips/<calib_tag>/pose` |
| `map_frame` / `base_frame` | `map` / `base_footprint` | |
| `settle_time` | `2.0` | cửa sổ gom mẫu, tính ngược từ lúc gọi `capture` |
| `min_samples` | `8` | số mẫu IPS tối thiểu trong cửa sổ |
| `still_threshold` | `0.05` | robot phải đứng yên trong ngần này (m) |
| `min_points` | `3` | ít hơn thì từ chối `compute` |
| `max_rms` | `0.5` | rms lớn hơn = hỏng, **không ghi file** |
| `manual_pairs` | `[]` | đường thủ công |
| `output` | `""` | để trống = tự chọn theo `paths.py` |

> **Gắn tag ở đâu trên robot**: càng gần trục quay (ngay trên `base_footprint`) càng tốt.
> Tag lệch tâm `d` mét thì mỗi lần robot quay đầu, cặp điểm lệch đi tới `d` — hiệu chuẩn sẽ
> có rms cỡ `d` dù mọi thứ khác đều đúng. Lệch không tránh được thì ít nhất giữ **cùng một
> hướng robot** ở tất cả các điểm chụp.

### Quy trình chạy

```bash
# terminal 1: Nav2 + AMCL của robot (đã có sẵn), localize xong, robot biết mình ở đâu
# terminal 2: bridge — frame_id đã mặc định là ips_map trong config, không cần truyền
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142 rviz:=false
# terminal 3:
ros2 run ips_a02_ros2 ips_calib --ros-args -p calib_tag:=T1

# lái robot tới điểm 1 (một góc vùng phủ anchor), dừng hẳn 3 giây, rồi:
ros2 service call /ips_calib/capture std_srvs/srv/Trigger
# lặp lại cho điểm 2, 3, 4 — TRẢI RỘNG ra 4 góc, 4 điểm gần nhau cho kết quả rất tệ
ros2 service call /ips_calib/compute std_srvs/srv/Trigger

# xong -> phát TF
ros2 run ips_a02_ros2 ips_map_tf
rviz2 -d install/ips_a02_ros2/share/ips_a02_ros2/rviz/ips_nav2.rviz
```

Kết quả `config/ips_map_transform.yaml`:

```yaml
# Sinh tự động bởi ips_calib — ĐỪNG sửa tay.
# 2026-08-26 16:32:32 · 4 điểm · rms 0.000 m · tag 'T1'
ips_map_tf:
  ros__parameters:
    parent_frame: "map"
    child_frame: "ips_map"
    x: -2.400252
    y: 7.899992
    yaw: 0.436352          # radian = 25.001 deg
    rms: 0.0005
    n_points: 4
# Dữ liệu thô, giữ lại để tính lại được: ips_x ips_y map_x map_y | sai số (m)
#      0.500   -0.500   -1.736    7.658 | 0.000
```

Dòng `#` cuối giữ lại dữ liệu thô — cần tính lại bằng tay hay soi điểm nào hỏng thì có sẵn.

### Đã sửa trong config

```yaml
frame_id: "ips_map"     # ĐỔI từ "map" — mọi pose/marker của IPS nằm trong ips_map
robot_tag: ""           # ĐỂ TRỐNG — robot tự định vị bằng lidar, không lấy TF từ UWB
```

`rviz/ips_a02.rviz` cũng đổi fixed frame sang `ips_map` để vẫn xem được khi chạy một mình
(không có Nav2, không có TF `map→ips_map`).

### Nghiệm thu B1

- `ros2 run tf2_ros tf2_echo map ips_map` ra đúng số trong yaml.
- Mở RViz với fixed frame `map`: 4 anchor phải nằm đúng 4 góc phòng **trên bản đồ occupancy grid**.
- Cầm tag đi bộ dọc một bức tường → chấm tag chạy song song tường trên bản đồ, lệch < 0.3 m.
- `rms` trong yaml < 0.15 m là tốt, 0.15–0.3 m tạm được, > 0.5 m là hỏng — chụp lại điểm.

---

## B.2. Bridge phát sự kiện nút bấm

Hiện tại `bridge_node.py:on_ips_message()` **vứt** `Call`/`CancelCall`/`Ack` đi, chỉ log:

```python
if name != "Pos":
    if name in ("Call", "CancelCall", "Ack"):
        self.get_logger().info(f"[{name}] từ tag {obj.get('deviceName')}")
    return
```

### Tạo package msg riêng: `ips_a02_msgs`

Tách ra package `ament_cmake` riêng (package Python không build được msg).

```
src/ips_a02_msgs/
├── package.xml          <buildtool_depend>ament_cmake</buildtool_depend>
│                        <depend>std_msgs</depend> <depend>geometry_msgs</depend>
│                        <build_depend>rosidl_default_generators</build_depend>
│                        <exec_depend>rosidl_default_runtime</exec_depend>
│                        <member_of_group>rosidl_interface_packages</member_of_group>
├── CMakeLists.txt       rosidl_generate_interfaces(${PROJECT_NAME} "msg/IpsEvent.msg"
│                            DEPENDENCIES std_msgs geometry_msgs)
└── msg/IpsEvent.msg
```

```
# msg/IpsEvent.msg — sự kiện không phải Pos từ tag UWB
std_msgs/Header header

string KIND_CALL="Call"
string KIND_CANCEL="CancelCall"
string KIND_ACK="Ack"

string kind             # Call | CancelCall | Ack
string uid              # định danh thật của tag — dùng cái này làm key
string device_name      # nhãn hiển thị, có thể đổi

string req_id           # chỉ có ở Ack: CallAck | CancelCallAck

# vị trí tag ngay lúc phát sự kiện — đính kèm luôn để tránh race với luồng Pos
geometry_msgs/PoseStamped tag_pose      # frame_id = ips_map
bool  pose_valid                        # false nếu chưa từng nhận Pos của tag này
float32 pose_age                        # giây, tuổi của mẫu Pos dùng ở trên
float32 pose_noise                      # noiseRadius của mẫu đó (m)
```

> Vì sao đính kèm pose vào chính message sự kiện: `Call` và `Pos` đến qua cùng một luồng
> transport nhưng là 2 message khác nhau. Nếu node navigator tự đi đọc topic pose lúc nhận
> `Call`, nó có thể vớ phải mẫu cũ 2 giây hoặc mẫu vừa nhảy. Đóng gói tại nguồn thì
> navigator có đúng một sự thật để làm việc, và có luôn `pose_age`/`pose_noise` để từ chối.

### Sửa `bridge_node.py`

1. `import` msg mới, thêm publisher trong `__init__`:
   ```python
   self.pub_events = self.create_publisher(IpsEvent, "ips/events", 10)
   ```
2. Trong `on_ips_message`, thay nhánh `name != "Pos"` bằng `self._publish_event(obj, name)`:
   - tra `self.tags` theo `uid` (dưới `self.lock`) để lấy `x, y, z, yaw, last, noise`;
   - `pose_valid = t is not None`, `pose_age = time.time() - t.last`;
   - `tag_pose.header.frame_id = self.frame`;
   - `req_id = (obj.get("data") or {}).get("reqId", "")`.
3. Thêm `<depend>ips_a02_msgs</depend>` vào `package.xml`.

### Bổ sung `openapi.py` — phần ghi (hiện chỉ có GET)

```python
def _put(self, path, body=None):
    url = f"{self.base}/openapi/v1/{path.lstrip('/')}"
    raw = json.dumps(body).encode() if body is not None else b""
    req = urllib.request.Request(url, data=raw, method="PUT",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=self.timeout) as r:
        d = json.loads(r.read().decode("utf-8", "replace"))
    if d.get("code") != 0:                       # response luôn bọc code/message/data
        raise RuntimeError(f"OpenAPI code={d.get('code')} msg={d.get('message')}")
    return d.get("data")

def ack(self, uid, req_id="CallAck"):
    return self._put(f"devices/{uid}/services/Ack", {"data": {"reqId": req_id}})

def call(self, uid, duration=1):
    """Rung/kêu ở tag để báo cho người biết robot đã nhận lệnh. duration: giây, tối đa 64."""
    return self._put(f"devices/{uid}/services/Call", {"data": {"duration": int(duration)}})
```

### Nghiệm thu B2

```bash
ros2 topic echo /ips/events
```
Bấm nút trên tag → thấy đúng một message `kind: "Call"` với `uid` đúng, `pose_valid: true`,
`pose_age` < 0.5 s. Bấm huỷ → `kind: "CancelCall"`.

> Chưa rõ tag phát `Call` **một lần** hay **lặp lại liên tục** khi giữ nút. Kiểm tra bằng lệnh
> trên **trước khi** viết B3, vì nó quyết định logic debounce. Nếu lặp lại → cần chặn ở B3.

---

## B.3. Node điều hướng: `ips_call_navigator`

File `ips_a02_ros2/call_nav_node.py`, entry point `ips_call_nav`.

### Luồng xử lý một lần bấm nút

```
IpsEvent(kind=Call, uid, tag_pose@ips_map)
   │
   ├─ 1. lọc: uid có trong allowlist? pose_valid? pose_age < max_pose_age? noise < max_noise?
   │        debounce: cùng uid vừa Call < debounce_sec giây → bỏ qua
   ├─ 2. tf2: transform tag_pose  ips_map → map   (dùng Time() = latest, TF này là static nên an toàn)
   ├─ 3. chọn điểm đích:
   │        - bỏ z (Nav2 là 2D)
   │        - lùi lại `standoff` mét trên đường thẳng từ robot tới tag (đừng đâm vào người)
   │        - yaw goal = hướng từ điểm đích nhìn về phía tag (robot quay mặt vào người)
   ├─ 4. kiểm tra khả thi: tra `/global_costmap/costmap` — ô đích phải free.
   │        nếu bị chặn → dò xoắn ốc ra tối đa `search_radius` m tìm ô free gần nhất
   ├─ 5. arbitration: đang có goal chạy dở?
   │        `queue_policy: preempt` → cancel goal cũ, nhận goal mới
   │        `queue_policy: ignore`  → bỏ qua Call mới cho tới khi xong
   ├─ 6. gửi NavigateToPose goal, giữ goal_handle
   └─ 7. phản hồi: openapi.ack(uid, "CallAck")  → tag rung/nháy báo "đã nhận"
         publish /ips/nav_status (String) để dashboard/RViz thấy

IpsEvent(kind=CancelCall, uid)
   └─ nếu goal đang chạy là của uid này → goal_handle.cancel_goal_async()
      + openapi.ack(uid, "CancelCallAck")
```

### Khung code

```python
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  — đăng ký kiểu PoseStamped cho tf2, BẮT BUỘC import

from ips_a02_msgs.msg import IpsEvent


class IpsCallNavigator(Node):
    def __init__(self):
        super().__init__("ips_call_navigator")
        d = self.declare_parameter
        d("map_frame", "map")
        d("robot_base_frame", "base_link")
        d("standoff", 0.9)            # dừng cách người bao nhiêu mét
        d("max_pose_age", 1.5)        # bỏ qua Call nếu mẫu vị trí cũ hơn ngần này
        d("max_noise", 0.4)           # bỏ qua Call nếu noiseRadius lớn hơn ngần này
        d("debounce_sec", 3.0)
        d("allowed_tags", [""])       # [""] hoặc rỗng = cho tất cả tag
        d("queue_policy", "preempt")  # preempt | ignore
        d("costmap_topic", "/global_costmap/costmap")
        d("search_radius", 1.5)       # bán kính dò ô free quanh đích
        d("send_ack", True)
        d("openapi_base_url", "http://192.168.68.126:8080")

        cb = ReentrantCallbackGroup()
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.create_subscription(IpsEvent, "ips/events", self.on_event, 10, callback_group=cb)
        self.create_subscription(
            OccupancyGrid, self.get_parameter("costmap_topic").value, self.on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.costmap = None
        self.goal_handle = None
        self.active_uid = ""
        self.last_call = {}

    # ... on_event / plan_goal / free_cell_near / send_goal / cancel ...
```

Những chỗ dễ sai, viết ra cho khỏi quên:

- **`import tf2_geometry_msgs`** — không import thì `tfbuf.transform(pose, "map")` ném
  `TypeException` khó hiểu. Import xong không dùng, nhưng bắt buộc phải có.
- **Callback group**: `ActionClient` và subscription phải cùng `ReentrantCallbackGroup`,
  và **phải chạy bằng `MultiThreadedExecutor`**, nếu không sẽ deadlock khi gửi goal
  từ trong callback.
- **QoS costmap** là `transient_local` — dùng QoS mặc định sẽ không nhận được gì.
- **Đừng dùng `nav2_simple_commander.BasicNavigator`** ở đây: nó block và tự tạo node riêng,
  hợp cho script chứ không hợp cho node chạy dài. Dùng `ActionClient` trực tiếp.
- **Timestamp khi transform**: dùng `rclpy.time.Time()` (= "mới nhất") chứ không phải stamp
  trong message, vì static TF `map→ips_map` chỉ có một mốc thời gian.
- **yaw của goal**: UWB không cho hướng người. Đặt yaw = hướng từ điểm đích nhìn về tag,
  tính từ vị trí robot hiện tại lúc gửi goal.

### Tham số mới — thêm vào `config/ips_params.yaml`

```yaml
/ips_call_navigator:
  ros__parameters:
    map_frame: "map"
    robot_base_frame: "base_link"
    standoff: 0.9
    max_pose_age: 1.5
    max_noise: 0.4
    debounce_sec: 3.0
    allowed_tags: [""]
    queue_policy: "preempt"
    costmap_topic: "/global_costmap/costmap"
    search_radius: 1.5
    send_ack: true
    openapi_base_url: "http://192.168.68.126:8080"

/ips_map_tf:
  ros__parameters:
    transform_file: ""      # rỗng = dùng config/ips_map_transform.yaml trong share
```

### Nghiệm thu B3

1. `ros2 topic echo /ips/nav_status` — bấm nút, thấy `accepted uid=… goal=(x, y)`.
2. Robot chạy tới, dừng cách tag đúng `standoff` ± 0.3 m, quay mặt về phía tag.
3. Bấm huỷ giữa đường → robot dừng trong < 1 s, `/ips/nav_status` báo `cancelled`.
4. Bấm nút khi tag đứng trong tường / ngoài vùng phủ anchor → node **từ chối có log rõ ràng**,
   không gửi goal rác cho Nav2.
5. Bấm nút liên tục 5 lần trong 2 giây → chỉ 1 goal được gửi (debounce hoạt động).
6. Tắt IPS Tool giữa chừng → không có exception, node vẫn sống.

---

## B.4. Launch file tổng: `launch/ips_nav2.launch.py`

```
ips_bringup.launch.py  (bridge, frame_id:=ips_map, rviz:=false)
  + ips_map_tf
  + ips_call_navigator
```

Không include Nav2 vào đây — Nav2 do bringup của robot lo, để hai thứ độc lập.
Thêm argument `rviz:=true` dùng `rviz/ips_nav2.rviz` (bản copy của `ips_a02.rviz`
nhưng thêm display Map + Global Costmap + Path).

## B.5. Cập nhật `setup.py`

```python
entry_points={
    'console_scripts': [
        'ips_bridge   = ips_a02_ros2.bridge_node:main',
        'ips_sim      = ips_a02_ros2.sim_node:main',
        'ips_calib    = ips_a02_ros2.calib_node:main',      # MỚI
        'ips_map_tf   = ips_a02_ros2.map_tf_node:main',     # MỚI
        'ips_call_nav = ips_a02_ros2.call_nav_node:main',   # MỚI
    ],
},
```

---

## B.6. Cấu trúc thư mục sau khi làm xong

```
ros2_ws/src/
├── ips_a02_msgs/                       ← MỚI (ament_cmake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/IpsEvent.msg
└── ips_a02_ros2/
    ├── config/
    │   ├── ips_params.yaml             ← SỬA: frame_id=ips_map + block navigator
    │   └── ips_map_transform.yaml      ← MỚI, sinh tự động bởi ips_calib
    ├── launch/
    │   ├── ips_bringup.launch.py
    │   └── ips_nav2.launch.py          ← MỚI
    ├── rviz/
    │   ├── ips_a02.rviz
    │   └── ips_nav2.rviz               ← MỚI
    └── ips_a02_ros2/
        ├── bridge_node.py              ← SỬA: publish /ips/events
        ├── openapi.py                  ← SỬA: thêm _put/ack/call
        ├── calib.py                    ← MỚI: rigid_2d
        ├── calib_node.py               ← MỚI
        ├── map_tf_node.py              ← MỚI
        ├── call_nav_node.py            ← MỚI
        ├── transports.py               (không đổi)
        ├── markers.py                  (không đổi)
        └── sim_node.py                 ← nên thêm: bắn Call/CancelCall để test không cần tag
```

## B.7. Quy trình chạy đầy đủ (sau khi xong)

```bash
# 1. Robot: Nav2 + bản đồ (bringup có sẵn của bạn)
ros2 launch <robot_bringup> nav2.launch.py map:=<map>.yaml
#    -> đặt 2D Pose Estimate trong RViz, đợi AMCL hội tụ

# 2. Một lần duy nhất, khi lắp anchor xong hoặc khi quét lại bản đồ:
# Nav2 + bản đồ của robot đang chạy (chỉ cần /map và TF map, không cần localize)
ros2 run ips_a02_ros2 ips_align
ros2 run ips_a02_ros2 ips_align --ros-args -p click_anchors:='[A1, A3]'          # đường chéo 10.82 m, nhạy hơn 3 lần
ros2 run ips_a02_ros2 ips_align --ros-args -p click_anchors:='[A1, A2, A3, A4]'  # bấm cả 4, có rms
rviz2 -d install/ips_a02_ros2/share/ips_a02_ros2/rviz/ips_nav2.rviz

# hoặc sử dụng
ros2 launch ips_a02_ros2 ips_bringup.launch.py frame_id:=ips_map rviz:=false
ros2 run ips_a02_ros2 ips_calib --ros-args -p calib_tag:=T1
#    lái robot tới 4-5 điểm, mỗi điểm: ros2 service call /ips_calib/capture …
ros2 service call /ips_calib/compute std_srvs/srv/Trigger


# 3. Chạy hàng ngày:
ros2 launch ips_a02_ros2 ips_nav2.launch.py source:=udp
#    -> bấm nút trên tag, robot chạy tới
```

**Khi nào phải hiệu chuẩn lại (B1)**: quét lại bản đồ SLAM · di chuyển bất kỳ anchor nào ·
đổi gốc toạ độ trong IPS Tool. Không cần hiệu chuẩn lại khi chỉ khởi động lại máy.

---

## B.8. Bẫy & lưu ý an toàn

**Về toạ độ**
- `posVar` là **m²** không phải m. `optPoses` lồng **2 tầng** → `optPoses[0]`.
- Payload có thể là **1 object hoặc 1 mảng** → luôn chuẩn hoá về list (transports.py đã làm).
- `deviceName` đổi được → key phải là `uid` (bridge đã đúng, giữ nguyên).
- Anchor cao 2.9 m nhưng tag ở ~1.2 m — Nav2 chỉ quan tâm x, y. Bỏ z ở bước tạo goal.
- Vùng phủ chỉ 3.6 × 10.2 m. **Ngoài vùng đó UWB vẫn trả toạ độ nhưng sai rất nhiều.**
  Nên kiểm tra tag có nằm trong bounding box anchor (nới ra ~1 m) trước khi nhận `Call`.

**Về sự kiện**
- `fences` trong `Pos` là **trạng thái**, không phải sự kiện vào/ra — muốn có sự kiện phải tự so
  hai mẫu liên tiếp. (Chưa dùng ở B1–B3, nhưng là hướng mở rộng tự nhiên.)
- Mất tín hiệu **không có message báo** — chỉ là message ngừng đến. Phải tự canh bằng đồng hồ
  của mình (`offline_after`).
- `name` lạ thì bỏ qua, đừng crash.

**Về an toàn vận hành**
- Robot đi tới chỗ **người đang đứng**. `standoff` phải ≥ bán kính robot + 0.5 m, và nên bật
  `nav2_collision_monitor` nếu robot có sensor phù hợp.
- Người **di chuyển**. Quyết định rõ ràng: goal là vị trí tag **lúc bấm nút** (đơn giản, an toàn,
  khuyến nghị cho phiên bản đầu) hay **bám theo liên tục** (phải cập nhật goal ~1 Hz và
  xử lý dao động). Đừng để trạng thái lấp lửng ở giữa.
- Phải có cách dừng khẩn ngoài `CancelCall`: e-stop phần cứng, hoặc
  `ros2 action send_goal /… ` cancel bằng tay. `CancelCall` đi qua UWB → không đáng tin cho an toàn.
- `debounce` không chỉ để chống bấm nhiều lần, mà còn chống tag tự phát lại `Call` — kiểm tra
  hành vi thật của tag ở bước nghiệm thu B2.

**Về ROS**
- Đổi `config/*.yaml` không thấy tác dụng ⇒ build thiếu `--symlink-install`; xoá `build/ install/`
  rồi build lại.
- Truyền số qua launch phải ép kiểu: `ParameterValue(x, value_type=int)`, không thì
  `InvalidParameterType`. Launch file hiện tại đã xử lý — theo đúng mẫu đó khi viết launch mới.
- Cổng UDP 9600 có thể bị `web_test/ips_hub.py` chiếm. Chỉ chạy một trong hai, hoặc đổi
  `udp_port:=9610`.
- Khi build package msg lần đầu: `colcon build --packages-select ips_a02_msgs` **trước**,
  `source install/setup.bash`, rồi mới build `ips_a02_ros2`.

---

## B.9. Checklist

**B1 — Khớp toạ độ**
- [x] `align_node.py` — bấm 2 điểm trên RViz + interactive marker + lưu file
- [x] `calib.py::rigid_2d` + unit test với dữ liệu tự sinh (quay 30°, tịnh tiến, thêm nhiễu)
- [x] `calib_node.py` + 5 service capture/compute/reset/undo/list
- [x] `map_tf_node.py` đọc yaml → static TF
- [x] `frame_id` đổi sang `ips_map` trong config
- [x] Chạy thử toàn chuỗi bằng dữ liệu giả: lấy lại đúng phép biến đổi đã biết
- [ ] Chụp ≥ 4 điểm **thật**, rms < 0.3 m
- [ ] Anchor hiện đúng vị trí trên bản đồ trong RViz

**B2 — Sự kiện nút bấm**
- [ ] Package `ips_a02_msgs` + `IpsEvent.msg`, build được
- [ ] `bridge_node.py` publish `/ips/events` kèm pose
- [ ] `openapi.py` có `_put` / `ack` / `call`
- [ ] Xác nhận hành vi thật của nút: bấm 1 lần ra mấy message?
- [ ] `sim_node.py` bắn được `Call`/`CancelCall` giả

**B3 — Điều hướng**
- [ ] `call_nav_node.py` với MultiThreadedExecutor
- [ ] Lọc: allowlist, pose_age, noise, bounding box anchor, debounce
- [ ] Standoff + yaw quay về phía tag
- [ ] Kiểm tra costmap + dò ô free gần nhất
- [ ] Cancel hoạt động < 1 s
- [ ] Ack ngược về tag qua OpenAPI
- [ ] `/ips/nav_status` cho dashboard

**B4 — Hoàn thiện**
- [ ] `ips_nav2.launch.py` + `ips_nav2.rviz`
- [ ] `setup.py` khai báo 3 entry point mới
- [ ] Cập nhật `README.md` của package
- [ ] Chạy thử liên tục 30 phút, 20 lần bấm nút, không crash

---

## B.10. Tài liệu tham khảo trong repo

| File | Nội dung |
|---|---|
| `CAU_TRUC_JSON.md` | đặc tả đầy đủ JSON của IPS Tool + toàn bộ OpenAPI + bẫy hay gặp |
| `HUONG_DAN_KHOI_CHAY.md` | từng kịch bản: server cần cài gì, ROS cần cài gì, xác nhận thông thế nào |
| `LUONG_DU_LIEU_MQTT.md` | luồng dữ liệu qua MQTT, cấu hình mosquitto |
| `DANH_GIA_PHUONG_THUC_TRUYEN.md` | so sánh UDP / MQTT / Webhook, chọn cái nào khi nào |
| `web_test/nooploop_ips_api_reference.md` | bản tham chiếu API gốc của hãng |
| `web_test/examples/ros2_ips_pose_node.py` | node ROS2 tối giản, đọc cho nhanh hiểu luồng |

Tài liệu hãng: https://support.nooploop.com/ips/api

---

# PHỤ LỤC — Nhật ký thực hiện

## 2026-08-26 · B1 — code xong, test bằng dữ liệu giả

**Đã sửa trước đó (không thuộc B1)**: địa chỉ OpenAPI mặc định trỏ sai máy
(`192.168.68.126` không tồn tại trên mạng → `Errno 113 No route to host`). IPS Tool thật
nằm ở **`192.168.68.142:8080`**, cùng máy với broker MQTT. Đồng thời sửa `ips_bringup.launch.py`
sang `OpaqueFunction`: launch arg để trống thì **không** ghi đè yaml nữa, nên
`config/ips_params.yaml` mới thật sự là nguồn sự thật (trước đó launch luôn đè yaml bằng
`default_value` của chính nó).

**Test đã chạy**

| Test | Cách chạy | Kết quả |
|---|---|---|
| `rigid_2d` | `python3 test/test_calib.py` | 5/5 PASS — không nhiễu sai < 1e-9; nhiễu 5 cm → sai góc < 2°, rms 0.12 m; dữ liệu lật gương → rms lớn (không im lặng trả kết quả sai); `residuals` chỉ đúng điểm hỏng |
| Toàn chuỗi B1 | node giả lập tag + TF `map→base_footprint` với phép biến đổi đã biết (yaw 25°, tx −2.4, ty 7.9), chụp 4 điểm rồi `compute` | lấy lại **yaw 25.001°, x −2.400, y 7.900, rms 0.0005 m** |
| `ips_map_tf` có file | `ros2 run … ips_map_tf` + `tf2_echo map ips_map` | đúng số trong yaml |
| `ips_map_tf` chưa hiệu chuẩn | xoá file rồi chạy | thoát mã 1 kèm hướng dẫn chạy `ips_calib`, không phát TF identity |

**Bẫy gặp phải khi test** — đáng ghi lại:

1. `TransformListener.lookup_transform(..., Time())` trả "mới nhất" nhưng **trễ**, và độ trễ
   tích luỹ nếu vòng spin bị chặn. Cắt cửa sổ mẫu theo giờ nhận ⇒ vớ phải mẫu của điểm trước
   ⇒ báo nhầm "robot đang di chuyển". Sửa: cắt cửa sổ theo **timestamp trong message**.
2. Máy dev đang chạy sẵn stack robot thật ở `ROS_DOMAIN_ID=76`. Node giả lập phát
   `map→base_footprint` **đụng với TF thật**, cho ra số vô nghĩa. Test giả lập phải chạy trong
   domain riêng: `export ROS_DOMAIN_ID=91`.
3. File hiệu chuẩn sinh lúc chạy nên **không có symlink sẵn** trong `install/` — mặc định
   ghi vào `install/`, mất khi xoá thư mục đó. `paths.py` dò symlink của `ips_params.yaml`
   để tìm ngược ra `src/` và ghi vào đó.

**Còn lại của B1**: hiệu chuẩn thật trên robot (≥ 4 điểm trải rộng, rms < 0.3 m) rồi xác nhận
4 anchor nằm đúng vị trí trên bản đồ occupancy trong RViz.

## 2026-08-26 · B1a — thêm cách khớp bằng mắt (`ips_align`)

Lý do: hiệu chuẩn bằng cách lái robot cho ra một bộ số mà **không nhìn được đúng hay sai** cho
tới khi chạy xong. Bấm 2 điểm trên bản đồ thì thấy kết quả ngay và sửa được bằng tay.

**Test đã chạy** (giả lập click, `ROS_DOMAIN_ID=91`, anchor thật lấy từ OpenAPI):

| Test | Kết quả |
|---|---|
| bấm 2 điểm đúng chỗ (yaw 25°, t = −2.4, 7.9) | lấy lại **yaw 25.00°, x −2.400, y 7.900** |
| bấm thêm điểm 3 và 4, có nhiễu tay ±8 cm | yaw 25.01°, x −2.395, y 7.895, **rms 0.070 m** |
| kéo interactive marker (`POSE_UPDATE`) | TF đổi tức thì đúng giá trị kéo (yaw 28.000°, x −1.900) |
| `save` → `reload` | đọc lại đúng số, đúng `n_points`, đúng `source` |
| bấm nhầm sang đường chéo kia | **kiểm tra khoảng cách KHÔNG bắt được** (2 đường chéo dài bằng nhau) — phải nhìn hình |

**Bẫy gặp phải**: `kill` tiến trình cha của `ros2 run` **không giết node con**. Ba node
`ips_align` cũ còn sống cùng nhận `/clicked_point` và cùng đăng ký service `/ips_align/save`,
làm kết quả lưu ra là của node khác. Dùng `setsid` khi chạy nền và `pkill -9 -f` lúc dọn.

## 2026-08-26 · B1a — chốt thao tác: bấm cố định A1 → A2

Theo yêu cầu: thao tác phải luôn giống nhau (A1 rồi A2), bấm A1 là thấy dấu ngay, bấm A2 là
thấy đủ 4 anchor + khung bao, và bấm lại được nếu chưa ưng.

| Thay đổi | Chi tiết |
|---|---|
| `click_anchors` mặc định `["A1", "A2"]` | bỏ cách tự chọn cặp xa nhất (chọn xong người dùng phải đọc log mới biết bấm cái nào) |
| marker hiện ngay từ click đầu | namespace `click` + `click_label` + `hint`, vẽ tại đúng điểm vừa bấm |
| chưa khớp lần nào thì không vẽ anchor | cờ `have_fit`; vẽ 4 chấm khi chưa có phép biến đổi là vẽ ở chỗ vô nghĩa |
| bấm lại ghi đè | đủ số điểm rồi mà còn bấm ⇒ mở vòng mới; khung cũ vẫn hiện trong lúc bấm dở |
| `DELETEALL` đầu mỗi MarkerArray | số marker thay đổi theo số click, không xoá thì dấu cũ đọng lại |

**Test** (giả lập click, `ROS_DOMAIN_ID=91`): đếm marker theo từng bước ra đúng như thiết kế —
chưa bấm `{}` → sau A1 `{click:1, click_label:1, hint:1}` → sau A2 `{click:2, anchor:4,
anchor_label:4, anchor_ring:1}` → bấm lại A1 `{click:1, anchor:4, …}` (khung cũ còn nguyên).
Khớp đúng chỗ cho lại **yaw 25.00°, x −2.400, y 7.900**; `/tf_static` và `tf2_echo map ips_map`
khớp số. Bấm A1 lệch 10 cm cho **yaw 28.23°** — đây là cơ sở của cảnh báo độ nhạy ở mục B1a.
