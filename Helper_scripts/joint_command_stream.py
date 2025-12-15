#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointCommandStream(Node):
    def __init__(self):
        super().__init__('joint_command_stream')

        # Publisher to Isaac's command topic
        self.pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # 100 Hz to match Isaac / ros2_control rate
        self.timer = self.create_timer(0.01, self.tick)

        # Adjust as needed
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        # Start from current-ish pose
        self.positions = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0]

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointCommandStream()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
