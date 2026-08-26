from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ukf_config = Path(get_package_share_directory('k1_odom')) / 'config' / 'ukf.yaml'

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[str(ukf_config)],
        ),
    ])
