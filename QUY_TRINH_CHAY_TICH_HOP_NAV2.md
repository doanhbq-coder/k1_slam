# neo_k1

## Khởi chạy robot (SLAM localization + Nav2)

```bash
ros2 launch k1_bringup bringup_robot.launch.py map:=/home/neok1-01/k1_slam/src/k1_maps/map2/map2.yaml
```

Lưu bản đồ mới:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/neok1-01/k1_slam/src/k1_maps/map3/map3'}"
```

## Hiệu chuẩn IPS ↔ bản đồ (một lần, sau khi lắp anchor hoặc quét bản đồ mới)

```bash
cd ~/Documents/neo_k1 && source install/setup.bash
ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142 rviz:=false
ros2 run   ips_a02_ros2 ips_align
rviz2 -d $(ros2 pkg prefix ips_a02_ros2)/share/ips_a02_ros2/rviz/ips_nav2.rviz
```

Trong RViz bấm **Publish Point** vào chân anchor **A1** rồi **A2** → cả 4 anchor hiện lên.
Kéo/xoay interactive marker để chỉnh, rồi lưu:

```bash
ros2 service call /ips_align/save std_srvs/srv/Trigger
```

Tắt `ips_align` trước khi chạy bước dưới (cả hai cùng phát TF `map → ips_map`).

## Chạy IPS tích hợp Nav2 — bấm nút trên tag, robot chạy tới

```bash
cd ~/Documents/neo_k1 && source install/setup.bash
ros2 launch ips_a02_ros2 ips_nav2.launch.py source:=mqtt broker:=192.168.68.142
ros2 topic echo /ips/nav_status      # terminal khác
```

Gọi robot: **giữ nút trên tag ~2 giây**, hoặc bấm **Call** trên web IPS Tool (`:8080`).

## Tài liệu

| File | Nội dung |
|---|---|
| [GIAI_PHAP_GOI_ROBOT_UWB.md](GIAI_PHAP_GOI_ROBOT_UWB.md) | Báo cáo giải pháp — cho người không chuyên |
| [NAV2_INTEGRATION.md](NAV2_INTEGRATION.md) | Thiết kế, tham số, số liệu đo, nhật ký thực hiện |
| [src/ips_a02_ros2/README.md](src/ips_a02_ros2/README.md) | Package: node, topic, tham số, trục trặc |
| [HUONG_DAN_KHOI_CHAY.md](HUONG_DAN_KHOI_CHAY.md) | Cấu hình IPS Tool theo từng phương thức truyền |
