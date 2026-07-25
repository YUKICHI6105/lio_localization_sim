"""Deterministic stage-4 motion command for the Gazebo physics baseline."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class GazeboDriver(Node):
    def __init__(self):
        super().__init__('gazebo_driver_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        msg = Twist()
        # Bounded, continuously varying command; collisions remain governed by Gazebo.
        msg.linear.x = 0.65 + 0.25 * math.sin(0.31 * t)
        msg.angular.z = 0.42 * math.sin(0.23 * t) + 0.18 * math.sin(0.71 * t)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
