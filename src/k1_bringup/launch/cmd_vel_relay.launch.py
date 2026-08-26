"""Standalone launch for the cmd_vel -> TwistStamped relay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/cmd_vel',
            description='Twist topic to subscribe to (output of twist_mux)',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/neo_robotics/K1_demo/V0_0_0/cmd_vel',
            description='TwistStamped topic the robot base listens on',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='frame_id written into the TwistStamped header',
        ),
        Node(
            package='k1_bringup',
            executable='cmd_vel_relay',
            name='cmd_vel_relay',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'frame_id': frame_id,
            }],
        ),
    ])
