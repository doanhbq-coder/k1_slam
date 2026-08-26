from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    ekf_config = Path(get_package_share_directory('k1_odom')) / 'config' / 'ekf_v2.yaml'

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[str(ekf_config)],
            remappings=[
                ('odometry/filtered', '/neo_robotics/K1_demo/V0_0_0/odom_filtered'),
            ],
        ),
    ])
