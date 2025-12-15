#!/usr/bin/env python3
import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from rclpy.duration import Duration

from control_msgs.msg import JointJog


class JointJogTest(Node):
    def __init__(self, topic: str, joint: str, vel: float, hz: float):
        super().__init__("joint_jog_test")

        # DON'T declare 'use_sim_time' (it's already declared); just set it.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.pub = self.create_publisher(JointJog, topic, 10)
        self.joint = joint
        self.vel = float(vel)
        self.timer = self.create_timer(1.0 / hz, self._tick)

    def _tick(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()  # critical
        msg.joint_names = [self.joint]
        msg.velocities = [self.vel]
        msg.duration = 0.0
        self.pub.publish(msg)


def main():
    argv = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/servo_node/delta_joint_cmds")
    parser.add_argument("--joint", default="joint_2")
    parser.add_argument("--vel", type=float, default=0.15)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args(argv[1:])

    rclpy.init(args=sys.argv)
    node = JointJogTest(args.topic, args.joint, args.vel, args.hz)

    end_time = node.get_clock().now() + Duration(seconds=args.seconds)
    while rclpy.ok() and node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    # send a clean stop
    node.vel = 0.0
    for _ in range(5):
        node._tick()
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
