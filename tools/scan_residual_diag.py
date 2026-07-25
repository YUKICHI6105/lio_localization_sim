#!/usr/bin/env python3
"""Measure how well the LiDAR scan matches the known wall map at the *ground-truth* pose.

This isolates map/geometry error from estimator error. If residuals at the true pose are
already large, no amount of ICP tuning can fix it -- the map or the sensor model is wrong.
If they are small, the map is fine and the estimator/association is at fault.

Reports, per scan, the distribution of point-to-nearest-segment distances and how many
points have no plausible wall association at all (candidate balls / unmapped objects).
"""
import bisect
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanResidualDiag(Node):
    def __init__(self):
        super().__init__('scan_residual_diag')
        self.declare_parameter('field_width', 5.572)
        self.declare_parameter('field_height', 3.310)
        self.declare_parameter('wall_thickness', 0.033)
        self.declare_parameter('wall_segment_x1', [0.0])
        self.declare_parameter('wall_segment_y1', [0.0])
        self.declare_parameter('wall_segment_x2', [0.0])
        self.declare_parameter('wall_segment_y2', [0.0])
        self.declare_parameter('report_every', 40)
        self.declare_parameter('outlier_threshold', 0.05)

        w = self.get_parameter('field_width').value
        h = self.get_parameter('field_height').value
        t = self.get_parameter('wall_thickness').value
        x1 = list(self.get_parameter('wall_segment_x1').value)
        y1 = list(self.get_parameter('wall_segment_y1').value)
        x2 = list(self.get_parameter('wall_segment_x2').value)
        y2 = list(self.get_parameter('wall_segment_y2').value)
        self.report_every = int(self.get_parameter('report_every').value)
        self.outlier_threshold = float(self.get_parameter('outlier_threshold').value)

        # Mirror laser_scan_matching_node.build_static_objects(): perimeter from
        # field_width/height (already the inner faces), interior segments contributing
        # BOTH faces at +/- half thickness. Labels let us attribute error per structure.
        hw, hh = w / 2.0, h / 2.0
        self.segments = [
            ((hw, -hh), (hw, hh), 'perimeter'),
            ((-hw, -hh), (-hw, hh), 'perimeter'),
            ((-hw, hh), (hw, hh), 'perimeter'),
            ((-hw, -hh), (hw, -hh), 'perimeter'),
        ]
        for i in range(len(x1)):
            dx, dy = x2[i] - x1[i], y2[i] - y1[i]
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            nx, ny = -dy / length, dx / length
            ht = t * 0.5
            label = f'interior[{i}]'
            self.segments.append((
                (x1[i] + nx * ht, y1[i] + ny * ht),
                (x2[i] + nx * ht, y2[i] + ny * ht), label))
            self.segments.append((
                (x1[i] - nx * ht, y1[i] - ny * ht),
                (x2[i] - nx * ht, y2[i] - ny * ht), label))

        self.get_logger().info(
            f'map: {len(self.segments)} segments (field {w:.3f}x{h:.3f}, thickness {t})')

        self.gt_t, self.gt = [], []
        self.scan_count = 0
        self.create_subscription(Odometry, '/ground_truth_pose', self.on_gt, 50)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_gt(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_t.append(t)
        self.gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                        yaw_from_quat(msg.pose.pose.orientation)))
        if len(self.gt_t) > 20000:
            del self.gt_t[:10000]
            del self.gt[:10000]

    def interp_gt(self, t):
        if not self.gt_t:
            return None
        i = bisect.bisect_left(self.gt_t, t)
        if i <= 0:
            return self.gt[0]
        if i >= len(self.gt_t):
            return self.gt[-1]
        t0, t1 = self.gt_t[i - 1], self.gt_t[i]
        a, b = self.gt[i - 1], self.gt[i]
        r = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * dyaw)

    @staticmethod
    def point_seg_distance(px, py, seg):
        (ax, ay), (bx, by), _ = seg
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-12:
            return math.hypot(px - ax, py - ay)
        u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return math.hypot(px - (ax + u * dx), py - (ay + u * dy))

    def on_scan(self, msg):
        self.scan_count += 1
        if self.scan_count % self.report_every:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.interp_gt(t)
        if pose is None:
            return
        px, py, yaw = pose
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        residuals, per_label, outliers, used = [], {}, 0, 0
        outlier_pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            used += 1
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            wx = px + cos_y * lx - sin_y * ly
            wy = py + sin_y * lx + cos_y * ly
            best, best_label = 1e9, None
            for seg in self.segments:
                d = self.point_seg_distance(wx, wy, seg)
                if d < best:
                    best, best_label = d, seg[2]
            if best > self.outlier_threshold:
                outliers += 1
                outlier_pts.append((wx, wy, best))
                continue
            residuals.append(best)
            per_label.setdefault(best_label, []).append(best)

        if not residuals:
            self.get_logger().warn(f'scan#{self.scan_count}: no inliers of {used} points')
            return
        residuals.sort()
        n = len(residuals)
        mean = sum(residuals) / n
        median = residuals[n // 2]
        p95 = residuals[min(int(n * 0.95), n - 1)]
        summary = '  '.join(
            f'{k}:n={len(v)},mean={1000 * sum(v) / len(v):.1f}mm'
            for k, v in sorted(per_label.items(), key=lambda kv: -len(kv[1]))[:6])
        # Where the unmapped returns are, so the offending structure can be identified by
        # location rather than guessed at. Coarse 0.5 m grid, biggest clusters first.
        cells = {}
        for wx, wy, d in outlier_pts:
            cells.setdefault((round(wx * 2) / 2, round(wy * 2) / 2), []).append(d)
        top = sorted(cells.items(), key=lambda kv: -len(kv[1]))[:5]
        cluster_txt = '  '.join(
            f'({cx:+.1f},{cy:+.1f}):n={len(v)},d={1000 * sum(v) / len(v):.0f}mm' for (cx, cy), v in top)

        self.get_logger().info(
            f'scan#{self.scan_count} pose=({px:.3f},{py:.3f},{math.degrees(yaw):.1f}deg) '
            f'pts={used} inlier={n} outlier={outliers} '
            f'| resid mean={1000 * mean:.1f}mm median={1000 * median:.1f}mm p95={1000 * p95:.1f}mm '
            f'|| {summary} || OUTLIERS {cluster_txt}')


def main():
    rclpy.init()
    node = ScanResidualDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
