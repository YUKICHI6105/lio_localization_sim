"""main.md 6-A-1: 合成IMUデータ生成ノード。

Trajectoryの真値からバイアス・ノイズ付きの/imu/dataを1kHzで生成し、比較用に
/ground_truth_poseも配信する。バイアスをわざと大きめに乗せることで、③が
本当にバイアスを学習・補正できているかを検証できるようにしている。
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from lio_localization_sim.trajectory import Trajectory


class ImuSimNode(Node):
    def __init__(self):
        super().__init__('imu_sim_node')

        field_width = self.declare_parameter('field_width', 10.0).value
        field_height = self.declare_parameter('field_height', 10.0).value
        self.accel_noise_sigma = self.declare_parameter('accel_noise_sigma', 0.01).value
        self.gyro_noise_sigma = self.declare_parameter('gyro_noise_sigma', 0.001).value
        # main.md ①③ IMUバイアス学習の検証用: ノイズより大きめのバイアスをわざと乗せる
        self.bias_ax = self.declare_parameter('bias_accel_x', 0.05).value
        self.bias_ay = self.declare_parameter('bias_accel_y', -0.03).value
        self.bias_gz = self.declare_parameter('bias_gyro_z', 0.005).value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.map_frame_id = self.declare_parameter('map_frame_id', 'map').value

        pattern = self.declare_parameter('trajectory_pattern', 0).value
        self.traj = Trajectory(field_width, field_height, pattern=pattern)
        self.rng = random.Random(12345)

        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.gt_pub = self.create_publisher(Odometry, '/ground_truth_pose', 20)

        # main.md 6-A-1: imu_sim/lidar_sim/evaluatorが個別に自分のノード起動時刻を
        # t=0にすると、ノード起動タイミングのズレ(数十ms)が速度×ズレの恒常誤差
        # として現れる(fableによる敵対的レビューで指摘)。launchファイルが計算した
        # 単一のsim_start_time_secを全ノードで共有することでこれを解消する。
        # 未指定(0.0)の場合は従来通り自ノード起動時刻にフォールバックする。
        self.sim_start_time_sec = self.declare_parameter('sim_start_time_sec', 0.0).value
        if self.sim_start_time_sec <= 0.0:
            self.sim_start_time_sec = self.get_clock().now().nanoseconds * 1e-9

        # 第三段階センサ異常注入(検知+自己復帰の検証)。窓の間だけIMUを破綻させる。
        # 真値(gt)は常に配信し続けるので、破綻中の誤差と窓後の復帰を測定できる。
        # kind: none/dropout(配信停止)/noise(ノイズ激増)/stuck(直近値に固着)
        self.fault_kind = self.declare_parameter('imu_fault_kind', 'none').value
        self.fault_start = self.declare_parameter('imu_fault_start', 0.0).value
        self.fault_dur = self.declare_parameter('imu_fault_dur', 0.0).value
        self._last_msg = None

        self.timer = self.create_timer(0.001, self.on_timer)  # 1kHz

    def _fault_active(self, t):
        return (self.fault_dur > 0.0 and self.fault_start <= t
                < self.fault_start + self.fault_dur)

        self.get_logger().info(
            f'imu_sim_node started (bias_ax={self.bias_ax}, bias_ay={self.bias_ay}, '
            f'bias_gz={self.bias_gz})')

    def elapsed(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self.sim_start_time_sec

    def on_timer(self):
        # tとstampを同一のnow()から導出し、両者にズレが生じないようにする
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.sim_start_time_sec
        s = self.traj.state(t)

        fault = self.fault_kind if self._fault_active(t) else 'none'
        if fault == 'dropout':
            pass  # IMU配信を止める(真値は下で配信し続ける)
        else:
            an = self.accel_noise_sigma
            gn = self.gyro_noise_sigma
            if fault == 'noise':
                an *= 50.0  # ノイズ激増(振動・電磁ノイズ相当)
                gn *= 50.0
            msg = Imu()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.base_frame_id
            if fault == 'stuck' and self._last_msg is not None:
                msg = self._last_msg  # 直近値に固着(センサハング相当)
                msg.header.stamp = now.to_msg()
            else:
                msg.linear_acceleration.x = s.ax_body + self.bias_ax + self.rng.gauss(0.0, an)
                msg.linear_acceleration.y = s.ay_body + self.bias_ay + self.rng.gauss(0.0, an)
                msg.linear_acceleration.z = s.az_body + self.rng.gauss(0.0, an)
                msg.angular_velocity.x = self.rng.gauss(0.0, gn)
                msg.angular_velocity.y = self.rng.gauss(0.0, gn)
                msg.angular_velocity.z = s.omega + self.bias_gz + self.rng.gauss(0.0, gn)
            self.imu_pub.publish(msg)
            self._last_msg = msg

        gt = Odometry()
        gt.header.stamp = now.to_msg()
        gt.header.frame_id = self.map_frame_id
        gt.child_frame_id = self.base_frame_id
        gt.pose.pose.position.x = s.x
        gt.pose.pose.position.y = s.y
        half_yaw = s.yaw / 2.0
        gt.pose.pose.orientation.z = math.sin(half_yaw)
        gt.pose.pose.orientation.w = math.cos(half_yaw)
        gt.twist.twist.linear.x = s.vx
        gt.twist.twist.linear.y = s.vy
        gt.twist.twist.angular.z = s.omega
        self.gt_pub.publish(gt)


def main():
    rclpy.init()
    node = ImuSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
