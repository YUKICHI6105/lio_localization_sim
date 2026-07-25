#!/usr/bin/env python3
"""Count messages actually delivered, to locate where 1 kHz IMU samples are being lost.

The earlier rate measurement used a sensor-data QoS (depth 5, best effort), which a Python
subscriber can under-run on its own at 1 kHz -- so it could not distinguish "the transport
dropped them" from "my subscriber dropped them". This node subscribes with a deep reliable
queue instead, so whatever it misses was already missing upstream. Comparing its totals with
Unity's own send counters splits the pipeline at the bridge.

/clock is counted too because it is published on the same 1 kHz FixedUpdate as the IMU and
therefore doubles the message load through the same socket; if both arrive at the same
fraction, the bridge is saturated rather than any one topic being mishandled.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan


class TopicCountDiag(Node):
    def __init__(self):
        super().__init__('topic_count_diag')
        deep = QoSProfile(depth=5000, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.n_imu = 0
        self.n_scan = 0
        self.n_clock = 0
        self.first_stamp = None
        self.last_stamp = None

        self.create_subscription(Imu, '/imu/data', self.on_imu, deep)
        self.create_subscription(LaserScan, '/scan', self.on_scan, deep)
        self.create_subscription(Clock, '/clock', self.on_clock, deep)
        self.create_timer(10.0, self.report)
        self.get_logger().info('topic_count_diag started (deep reliable queues)')

    def on_imu(self, msg):
        self.n_imu += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = t
        self.last_stamp = t

    def on_scan(self, msg):
        self.n_scan += 1

    def on_clock(self, msg):
        self.n_clock += 1

    def report(self):
        if self.first_stamp is None:
            self.get_logger().info('no IMU yet')
            return
        # Elapsed is taken from message stamps, i.e. simulated time, so a Unity Editor that
        # runs slower than wall time does not show up as a phantom drop rate.
        span = max(self.last_stamp - self.first_stamp, 1e-9)
        self.get_logger().info(
            f'sim_span={span:.3f}s || imu n={self.n_imu} ({self.n_imu / span:.1f}Hz) '
            f'clock n={self.n_clock} ({self.n_clock / span:.1f}Hz) '
            f'scan n={self.n_scan} ({self.n_scan / span:.1f}Hz)')


def main():
    rclpy.init()
    node = TopicCountDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
