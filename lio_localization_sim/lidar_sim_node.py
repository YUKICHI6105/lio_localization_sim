"""main.md 6-A-1: 合成LiDARスキャン生成ノード。

Trajectoryの真値姿勢に対して、config/field.yamlと同じ壁・固定円柱ジオメトリへ
レイキャストし、UTM-30LX相当の/scanを40Hzで生成する。1点ごとに真値のサブ
スキャン時刻の姿勢を使うため、②のDeskewingロジック自体も検証対象になる。
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from lio_localization_sim.trajectory import Trajectory

NUM_POINTS = 1081
FOV_DEG = 270.0
SCAN_PERIOD_SEC = 0.025
ACTIVE_FRACTION = 270.0 / 360.0  # main.md ②-1で確定した回転比率


class LidarSimNode(Node):
    def __init__(self):
        super().__init__('lidar_sim_node')

        field_width = self.declare_parameter('field_width', 10.0).value
        field_height = self.declare_parameter('field_height', 10.0).value
        self.hw = field_width / 2.0
        self.hh = field_height / 2.0

        cyl_ids = self.declare_parameter('cylinder_ids', [0]).value
        cyl_x = self.declare_parameter('cylinder_x', [0.0]).value
        cyl_y = self.declare_parameter('cylinder_y', [0.0]).value
        cyl_r = self.declare_parameter('cylinder_radius', [0.1]).value
        self.cylinders = list(zip(cyl_x, cyl_y, cyl_r)) if len(cyl_ids) > 0 else []

        self.range_min = self.declare_parameter('range_min', 0.1).value
        self.range_max = self.declare_parameter('range_max', 30.0).value
        self.range_noise_sigma = self.declare_parameter('range_noise_sigma', 0.03).value
        self.simulate_ball = self.declare_parameter('simulate_ball', True).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        pattern = self.declare_parameter('trajectory_pattern', 0).value
        self.traj = Trajectory(field_width, field_height, pattern=pattern)
        self.rng = random.Random(54321)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

        # imu_sim_nodeと同じ理由(fableのレビュー指摘): 各ノードが自分の起動時刻を
        # t=0にすると起動タイミングのズレが速度×ズレの恒常誤差になる。launchが
        # 計算した共通のsim_start_time_secを使う(未指定なら従来通り自ノード起点)。
        self.sim_start_time_sec = self.declare_parameter('sim_start_time_sec', 0.0).value
        if self.sim_start_time_sec <= 0.0:
            self.sim_start_time_sec = self.get_clock().now().nanoseconds * 1e-9

        # 第三段階センサ異常注入(検知+自己復帰の検証)。窓の間だけLiDARを破綻させる。
        # kind: none/dropout(スキャン停止)/outlier(点の一部を乱数距離に置換=遮蔽・
        # マルチパス・他ロボット乱入相当)
        self.fault_kind = self.declare_parameter('lidar_fault_kind', 'none').value
        self.fault_start = self.declare_parameter('lidar_fault_start', 0.0).value
        self.fault_dur = self.declare_parameter('lidar_fault_dur', 0.0).value

        self.timer = self.create_timer(SCAN_PERIOD_SEC, self.on_timer)

    def _fault_active(self, t):
        return (self.fault_dur > 0.0 and self.fault_start <= t
                < self.fault_start + self.fault_dur)

        self.get_logger().info(
            f'lidar_sim_node started ({len(self.cylinders)} cylinders, '
            f'field {field_width}x{field_height}m, ball={self.simulate_ball})')

    def elapsed(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self.sim_start_time_sec

    def ray_box_exit(self, px, py, dx, dy) -> float:
        t_best = float('inf')
        if dx > 1e-9:
            t = (self.hw - px) / dx
            if 0.0 < t < t_best:
                y_at = py + t * dy
                if -self.hh <= y_at <= self.hh:
                    t_best = t
        elif dx < -1e-9:
            t = (-self.hw - px) / dx
            if 0.0 < t < t_best:
                y_at = py + t * dy
                if -self.hh <= y_at <= self.hh:
                    t_best = t
        if dy > 1e-9:
            t = (self.hh - py) / dy
            if 0.0 < t < t_best:
                x_at = px + t * dx
                if -self.hw <= x_at <= self.hw:
                    t_best = t
        elif dy < -1e-9:
            t = (-self.hh - py) / dy
            if 0.0 < t < t_best:
                x_at = px + t * dx
                if -self.hw <= x_at <= self.hw:
                    t_best = t
        return t_best

    @staticmethod
    def ray_circle_hit(px, py, dx, dy, cx, cy, r):
        ox, oy = px - cx, py - cy
        b = 2.0 * (dx * ox + dy * oy)
        c = ox * ox + oy * oy - r * r
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / 2.0
        t2 = (-b + sqrt_disc) / 2.0
        if t1 > 1e-6:
            return t1
        if t2 > 1e-6:
            return t2
        return None

    def cast_ray(self, px, py, world_angle, ball_pos):
        dx, dy = math.cos(world_angle), math.sin(world_angle)
        dist = self.ray_box_exit(px, py, dx, dy)
        for (cx, cy, r) in self.cylinders:
            hit = self.ray_circle_hit(px, py, dx, dy, cx, cy, r)
            if hit is not None and hit < dist:
                dist = hit
        if ball_pos is not None:
            hit = self.ray_circle_hit(px, py, dx, dy, ball_pos[0], ball_pos[1], self.traj.ball_radius)
            if hit is not None and hit < dist:
                dist = hit
        return dist

    def on_timer(self):
        # stampはこのループ開始時点のnowを使う。ループ後にnow()を取り直すと、
        # 1081点のレイキャスト計算(実測数ms)ぶんstampが系統的に遅れ、
        # デスキュー・評価の両方に系統誤差が乗る(fableのレビュー指摘)。
        now = self.get_clock().now()
        t_start = now.nanoseconds * 1e-9 - self.sim_start_time_sec
        fault = self.fault_kind if self._fault_active(t_start) else 'none'
        if fault == 'dropout':
            return  # スキャン配信を止める(壁補正途絶→③が検知・復帰するはず)
        angle_min = -math.radians(FOV_DEG / 2.0)
        angle_increment = math.radians(FOV_DEG) / (NUM_POINTS - 1)
        time_increment = (SCAN_PERIOD_SEC * ACTIVE_FRACTION) / (NUM_POINTS - 1)

        ball_pos = self.traj.ball_position(t_start) if self.simulate_ball else None

        ranges = []
        for i in range(NUM_POINTS):
            t_i = t_start + i * time_increment
            s = self.traj.state(t_i)
            local_angle = angle_min + i * angle_increment
            world_angle = s.yaw + local_angle
            true_dist = self.cast_ray(s.x, s.y, world_angle, ball_pos)
            if not math.isfinite(true_dist) or true_dist > self.range_max:
                ranges.append(float('inf'))
                continue
            noisy = true_dist + self.rng.gauss(0.0, self.range_noise_sigma)
            if noisy < self.range_min:
                noisy = self.range_min
            # outlier注入: 窓中は一部の点を乱数距離に置換(遮蔽・マルチパス・
            # 他ロボット乱入相当)。半数を壊してICPを激しく攪乱する。
            if fault == 'outlier' and self.rng.random() < 0.5:
                noisy = self.rng.uniform(self.range_min, self.range_max)
            ranges.append(noisy)

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = angle_min
        msg.angle_max = -angle_min
        msg.angle_increment = angle_increment
        msg.time_increment = time_increment
        msg.scan_time = SCAN_PERIOD_SEC
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = LidarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
