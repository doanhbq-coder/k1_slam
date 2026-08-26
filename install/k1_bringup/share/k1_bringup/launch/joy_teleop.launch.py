"""
Joystick teleoperation bringup for the K1 robot.

Pipeline:

    joy_node  --/joy-->  teleop_node  --/cmd_vel_joy--\
                                                       twist_mux --/cmd_vel-->
    (nav2)    --/cmd_vel_nav-------------------------/
    (magnetic line)  --/cmd_vel_mag-----------------/
    (collision monitor) --/cmd_vel_collision-------/

All parameters (joystick mapping, axis scales, mux priorities) live in
config/joy_teleop.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('k1_bringup'),
        'config',
        'joy_teleop.yaml',
    )

    config_file = LaunchConfiguration('config_file')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    use_twist_mux = LaunchConfiguration('use_twist_mux')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to the joy / teleop / twist_mux parameter file',
    )

    declare_cmd_vel_out = DeclareLaunchArgument(
        'cmd_vel_out',
        default_value='/cmd_vel',
        description='Output topic of twist_mux (the command actually sent to the base)',
    )

    declare_use_twist_mux = DeclareLaunchArgument(
        'use_twist_mux',
        default_value='true',
        description='Start twist_mux. Set to false to publish joystick commands only',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[config_file],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[config_file],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        condition=IfCondition(use_twist_mux),
        parameters=[config_file],
        remappings=[('/cmd_vel_out', cmd_vel_out)],
    )

    return LaunchDescription([
        declare_config_file,
        declare_cmd_vel_out,
        declare_use_twist_mux,
        joy_node,
        teleop_node,
        twist_mux_node,
    ])
