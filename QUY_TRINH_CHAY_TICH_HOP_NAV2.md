# neo_k1
ssh neok1-01@192.168.68.160
phenikaaneo@

## Khởi chạy bringup
runtime_k1
cd dev_ws
ros2 launch neo_manager bringup.launch.py

## lệnh khởi chạy slam trên robot
ros2 launch k1_bringup bringup_robot.launch.py map:=/home/neok1-01/k1_slam/src/k1_maps/map3/map3.yaml

### lệnh lưu map posegraph, data
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/neok1-01/k1_slam/src/k1_maps/map3/map3'}"

# PC server
## Gắn 4 anchor lên bản đồ (nav2) - Tạo ra file config
### Nav2 + bản đồ của robot đang chạy (chỉ cần /map và TF map, không cần localize)
ros2 run ips_a02_ros2 ips_align
ros2 run ips_a02_ros2 ips_align --ros-args -p click_anchors:='[A1, A3]'  
ros2 run ips_a02_ros2 ips_align --ros-args -p click_anchors:='[A1, A2, A3, A4]

rviz2 -d install/ips_a02_ros2/share/ips_a02_ros2/rviz/ips_nav2.rviz


## Khởi chạy ips tích hợp nav2
cd ~/Documents/neo_k1 && source install/setup.bash
ros2 launch ips_a02_ros2 ips_nav2.launch.py source:=mqtt broker:=192.168.68.142
ros2 topic echo /ips/nav_status      # terminal khác

