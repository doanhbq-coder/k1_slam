"""
Top level bringup for the K1 robot.

Starts joystick teleop + cmd_vel multiplexing and relays the resulting Twist
to the TwistStamped topic the base actually listens on:

    joy_node --/joy--> teleop_node --/cmd_vel_joy--\
    (nav2)         --/cmd_vel_nav-----------------  twist_mux --/cmd_vel-->
    (magnetic)     --/cmd_vel_mag-----------------/                  |
    (collision)    --/cmd_vel_collision----------/                   v
                                                            cmd_vel_relay
                                                                     |
                                          /neo_robotics/K1_demo/V0_0_0/cmd_vel
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
    pkg_share = get_package_share_directory('k1_bringup')

    config_file = LaunchConfiguration('config_file')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    robot_cmd_vel_topic = LaunchConfiguration('robot_cmd_vel_topic')
    frame_id = LaunchConfiguration('frame_id')
    use_joy = LaunchConfiguration('use_joy')
    use_twist_mux = LaunchConfiguration('use_twist_mux')
    use_relay = LaunchConfiguration('use_relay')

    declared_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, 'config', 'joy_teleop.yaml'),
            description='Full path to the joy / teleop / twist_mux parameter file',
        ),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel',
            description='Output topic of twist_mux, also the input of cmd_vel_relay',
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
            'use_joy',
            default_value='true',
            description='Start joy_node / teleop_node / twist_mux',
        ),
        DeclareLaunchArgument(
            'use_twist_mux',
            default_value='true',
            description='Start twist_mux. Set to false to publish joystick commands only',
        ),
        DeclareLaunchArgument(
            'use_relay',
            default_value='true',
            description='Start the Twist -> TwistStamped relay to the base',
        ),
    ]

    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joy_teleop.launch.py')
        ),
        condition=IfCondition(use_joy),
        launch_arguments={
            'config_file': config_file,
            'cmd_vel_out': cmd_vel_out,
            'use_twist_mux': use_twist_mux,
        }.items(),
    )

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

    return LaunchDescription(declared_args + [joy_teleop, cmd_vel_relay])
