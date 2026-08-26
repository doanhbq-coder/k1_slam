"""
Full robot bringup for the K1: localization + navigation + base command relay.

Starts, in one shot:
  * cmd_vel_relay          - Twist on /cmd_vel -> TwistStamped for the base
  * k1_odom/ekf            - robot_localization EKF
  * k1_slam/k1_map_server  - map_server + map_saver + slam_toolbox (localization)
  * k1_slam/k1_navigation  - nav2 planner / controller / bt_navigator / ...

Joystick teleop is NOT included here; run bringup.launch.py (or
joy_teleop.launch.py) alongside this if you want manual override.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    k1_bringup_share = get_package_share_directory('k1_bringup')
    k1_odom_share = get_package_share_directory('k1_odom')
    k1_slam_share = get_package_share_directory('k1_slam')

    # k1_maps lives in the source tree, not in an installed package, so the
    # default map path is resolved the same way k1_map_server.launch.py does it.
    workspace_dir = os.path.abspath(os.path.join(k1_slam_share, '..', '..', '..', '..'))
    default_map = os.path.join(workspace_dir, 'src', 'k1_maps', 'map1', 'map1.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    robot_cmd_vel_topic = LaunchConfiguration('robot_cmd_vel_topic')
    frame_id = LaunchConfiguration('frame_id')
    use_relay = LaunchConfiguration('use_relay')
    use_ekf = LaunchConfiguration('use_ekf')
    use_map_server = LaunchConfiguration('use_map_server')
    use_navigation = LaunchConfiguration('use_navigation')

    declared_args = [
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the YAML map file to load',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel',
            description='Twist topic feeding the base, input of cmd_vel_relay',
        ),
        DeclareLaunchArgument(
            'robot_cmd_vel_topic',
            default_value='/neo_robotics/K1_demo/V0_0_0/cmd_vel',
            description='TwistStamped topic the robot base listens on',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='frame_id written into the TwistStamped header',
        ),
        DeclareLaunchArgument(
            'use_relay',
            default_value='true',
            description='Start the Twist -> TwistStamped relay to the base',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Start the robot_localization EKF (k1_odom)',
        ),
        DeclareLaunchArgument(
            'use_map_server',
            default_value='true',
            description='Start map_server / map_saver / slam_toolbox localization (k1_slam)',
        ),
        DeclareLaunchArgument(
            'use_navigation',
            default_value='true',
            description='Start the nav2 stack (k1_slam)',
        ),
    ]

    cmd_vel_relay = Node(
        package='k1_bringup',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        condition=IfCondition(use_relay),
        parameters=[{
            'input_topic': cmd_vel_out,
            'output_topic': robot_cmd_vel_topic,
            'frame_id': frame_id,
        }],
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(k1_odom_share, 'launch', 'ekf.launch.py')
        ),
        condition=IfCondition(use_ekf),
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(k1_slam_share, 'launch', 'k1_map_server.launch.py')
        ),
        condition=IfCondition(use_map_server),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(k1_slam_share, 'launch', 'k1_navigation.launch.py')
        ),
        condition=IfCondition(use_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(declared_args + [
        cmd_vel_relay,
        ekf,
        map_server,
        navigation,
    ])
