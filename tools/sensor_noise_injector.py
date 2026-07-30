#!/usr/bin/env python3
"""Inject one reproducible LiDAR measurement perturbation into a replay stream.

This is intentionally a ROS-side replay tool: it lets a single sensor-error
factor be tested against a fixed Unity recording without changing the motion,
ground truth, or any other already-recorded sensor stream.
"""

import copy
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNoiseInjector(Node):
    def __init__(self) -> None:
        super().__init__("scan_noise_injector")
        self.yaw_bias_rad = float(self.declare_parameter("scan_yaw_bias_rad", 0.0).value)
        self.range_bias_m = float(self.declare_parameter("scan_range_bias_m", 0.0).value)
        self.range_scale = float(self.declare_parameter("scan_range_scale", 1.0).value)
        self.range_noise_sigma_m = float(
            self.declare_parameter("scan_range_noise_sigma_m", 0.0).value)
        self.dropout_probability = float(
            self.declare_parameter("scan_dropout_probability", 0.0).value)
        seed = int(self.declare_parameter("random_seed", 20260729).value)
        if self.range_scale <= 0.0:
            raise ValueError("scan_range_scale must be positive")
        if not 0.0 <= self.dropout_probability <= 1.0:
            raise ValueError("scan_dropout_probability must be in [0, 1]")
        self.rng = random.Random(seed)
        self.scan_count = 0
        self.publisher = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, "/scan_raw", self.callback, qos_profile_sensor_data)
        self.get_logger().info(
            "scan noise injector: yaw_bias=%.6frad range_bias=%.4fm scale=%.6f "
            "range_sigma=%.4fm dropout=%.4f seed=%d"
            % (self.yaw_bias_rad, self.range_bias_m, self.range_scale,
               self.range_noise_sigma_m, self.dropout_probability, seed))

    def callback(self, message: LaserScan) -> None:
        output = copy.deepcopy(message)
        output.angle_min += self.yaw_bias_rad
        output.angle_max += self.yaw_bias_rad
        adjusted = []
        for measured_range in message.ranges:
            if not math.isfinite(measured_range):
                adjusted.append(measured_range)
                continue
            if self.rng.random() < self.dropout_probability:
                adjusted.append(float("inf"))
                continue
            value = self.range_scale * measured_range + self.range_bias_m
            value += self.rng.gauss(0.0, self.range_noise_sigma_m)
            adjusted.append(min(max(value, message.range_min), message.range_max))
        output.ranges = adjusted
        self.publisher.publish(output)
        self.scan_count += 1


def main() -> None:
    rclpy.init()
    node = ScanNoiseInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(f"forwarded {node.scan_count} scans")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
