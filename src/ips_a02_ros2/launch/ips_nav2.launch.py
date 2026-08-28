"""Chạy đầy đủ: bridge + static TF map->ips_map + node điều hướng theo nút bấm.

    ros2 launch ips_a02_ros2 ips_nav2.launch.py source:=mqtt broker:=192.168.68.142
    ros2 launch ips_a02_ros2 ips_nav2.launch.py sim:=true            # test không cần tag thật
    ros2 launch ips_a02_ros2 ips_nav2.launch.py rviz:=true           # kèm RViz có Map + Costmap

KHÔNG include Nav2 vào đây — Nav2 do bringup của robot lo, để hai thứ độc lập. Phải chạy
`ips_align` (hoặc `ips_calib`) một lần trước, không thì ips_map_tf thoát ngay kèm hướng dẫn.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "ips_a02_ros2"
lc = LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory(PKG)
    params_file = os.path.join(share, "config", "ips_params.yaml")

    args = [
        DeclareLaunchArgument("source", default_value=""),
        DeclareLaunchArgument("broker", default_value=""),
        DeclareLaunchArgument("broker_port", default_value=""),
        DeclareLaunchArgument("mqtt_topic", default_value=""),
        DeclareLaunchArgument("udp_port", default_value=""),
        DeclareLaunchArgument("webhook_port", default_value=""),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="RViz của package này; robot thường đã có RViz riêng"),
        DeclareLaunchArgument("rviz_config",
                              default_value=os.path.join(share, "rviz", "ips_nav2_v2.rviz")),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "ips_bringup.launch.py")),
        launch_arguments={"source": lc("source"), "broker": lc("broker"),
                          "broker_port": lc("broker_port"), "mqtt_topic": lc("mqtt_topic"),
                          "udp_port": lc("udp_port"), "webhook_port": lc("webhook_port"),
                          "params_file": lc("params_file"), "sim": lc("sim"),
                          "rviz": "false"}.items(),
    )
    map_tf = Node(package=PKG, executable="ips_map_tf", name="ips_map_tf", output="screen",
                  parameters=[lc("params_file")])
    call_nav = Node(package=PKG, executable="ips_call_nav", name="ips_call_navigator",
                    output="screen", parameters=[lc("params_file")])
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                condition=IfCondition(lc("rviz")), arguments=["-d", lc("rviz_config")])
    return LaunchDescription(args + [bringup, map_tf, call_nav, rviz])
