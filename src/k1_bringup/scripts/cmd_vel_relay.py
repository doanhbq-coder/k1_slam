#!/usr/bin/env python3
"""
cmd_vel_stamper node
--------------------
Subscribes to a Twist topic (unstamped) and republishes it as TwistStamped,
adding the current timestamp and a configurable frame_id.

Parameters:
  ~input_topic  (str)  : source topic, default '/cmd_vel'
  ~output_topic (str)  : destination topic,
                         default '/neo_robotics/K1_demo/V0_0_0/cmd_vel_stamped'
  ~frame_id     (str)  : frame_id for the stamped header, default 'base_footprint'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):

    def __init__(self):
        super().__init__('cmd_vel_stamper')

        # --- parameters ---
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter(
            'output_topic',
            '/neo_robotics/K1_demo/V0_0_0/cmd_vel'
        )
        self.declare_parameter('frame_id', 'base_footprint')

        input_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- pub / sub ---
        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self._callback,
            10
        )
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)

        self.get_logger().info(
            f'cmd_vel_stamper started\n'
            f'  subscribe : {input_topic}\n'
            f'  publish   : {output_topic}\n'
            f'  frame_id  : {self.frame_id}'
        )

    def _callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()