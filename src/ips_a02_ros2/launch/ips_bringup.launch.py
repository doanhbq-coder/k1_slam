"""Khởi động cầu nối IPS -> ROS2 (+ RViz2, + node giả lập nếu chưa có phần cứng).

File config/ips_params.yaml là nguồn sự thật duy nhất cho mọi mặc định.
Tham số dòng lệnh dưới đây chỉ ghi đè khi được truyền, để trống = giữ nguyên yaml.

    ros2 launch ips_a02_ros2 ips_bringup.launch.py                     # đúng như yaml
    ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=mqtt broker:=192.168.68.142
    ros2 launch ips_a02_ros2 ips_bringup.launch.py source:=webhook
    ros2 launch ips_a02_ros2 ips_bringup.launch.py sim:=true           # tự sinh dữ liệu để xem thử
    ros2 launch ips_a02_ros2 ips_bringup.launch.py rviz:=false robot_tag:=T1
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "ips_a02_ros2"

# tên launch arg -> (tên tham số ROS, kiểu).  Để trống trên dòng lệnh = dùng giá trị trong yaml.
ARG_TO_PARAM = {
    "source": ("source", str),
    "udp_port": ("udp.port", int),
    "broker": ("mqtt.broker", str),
    "broker_port": ("mqtt.port", int),
    "mqtt_topic": ("mqtt.topic", str),
    "webhook_port": ("webhook.port", int),
    "openapi_url": ("openapi.base_url", str),
    "robot_tag": ("robot_tag", str),
    "frame_id": ("frame_id", str),
}

lc = LaunchConfiguration


def _load_yaml(path):
    """ros__parameters của node ips_bridge trong file yaml (rỗng nếu không đọc được)."""
    try:
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}
    node = doc.get("/ips_bridge") or doc.get("ips_bridge") or {}
    return node.get("ros__parameters") or {}


def _dig(cfg, dotted, default=None):
    cur = cfg
    for key in dotted.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def launch_setup(context, *_):
    cfg = _load_yaml(lc("params_file").perform(context))

    # chỉ những arg thực sự có giá trị mới lọt vào dict ghi đè
    overrides = {}
    for arg, (param, cast) in ARG_TO_PARAM.items():
        raw = lc(arg).perform(context).strip()
        if raw:
            overrides[param] = cast(raw)

    def value(param, default=None):
        """giá trị cuối cùng của một tham số: dòng lệnh > yaml > default."""
        return overrides[param] if param in overrides else _dig(cfg, param, default)

    bridge = Node(
        package=PKG, executable="ips_bridge", name="ips_bridge", output="screen",
        parameters=[lc("params_file"), overrides],
    )

    # node giả lập không đọc yaml của bridge -> tự lấy giá trị đã giải quyết ở trên
    topic = value("mqtt.topic") or (list(_dig(cfg, "mqtt.topics", [])) or ["ips/#"])[0]
    sim = Node(
        package=PKG, executable="ips_sim", name="ips_sim", output="screen",
        condition=IfCondition(lc("sim")),
        parameters=[{
            "source": value("source", "udp"),
            "udp.port": int(value("udp.port", 9600)),
            "mqtt.broker": value("mqtt.broker", "127.0.0.1"),
            "mqtt.port": int(value("mqtt.port", 1883)),
            "mqtt.topic": topic.replace("#", "uplink"),
            "webhook.port": int(value("webhook.port", 8091)),
            "tags": int(lc("sim_tags").perform(context)),
            "pattern": lc("sim_pattern").perform(context),
        }],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        condition=IfCondition(lc("rviz")),
        arguments=["-d", lc("rviz_config")],
    )
    return [bridge, sim, rviz]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    params_file = os.path.join(share, "config", "ips_params.yaml")
    rviz_file = os.path.join(share, "rviz", "ips_a02.rviz")

    args = [
        DeclareLaunchArgument("source", default_value="",
                              description="udp | mqtt | webhook (trống = theo yaml)"),
        DeclareLaunchArgument("udp_port", default_value=""),
        DeclareLaunchArgument("broker", default_value="", description="broker MQTT"),
        DeclareLaunchArgument("broker_port", default_value=""),
        DeclareLaunchArgument("mqtt_topic", default_value=""),
        DeclareLaunchArgument("webhook_port", default_value=""),
        DeclareLaunchArgument("openapi_url", default_value="",
                              description="URL IPS Tool để lấy anchor / vùng ảo"),
        DeclareLaunchArgument("robot_tag", default_value="",
                              description="tên tag gắn trên robot -> TF map->base_link"),
        DeclareLaunchArgument("frame_id", default_value=""),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_file),
        DeclareLaunchArgument("sim", default_value="false",
                              description="true = chạy node giả lập IPS Tool"),
        DeclareLaunchArgument("sim_tags", default_value="3"),
        DeclareLaunchArgument("sim_pattern", default_value="lissajous"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
