#!/usr/bin/env python3
"""Measure the SIGNED offset between scan points and each individual wall, at ground truth.

The earlier nearest-segment diagnostic could only show |residual|, which hides a systematic
map/measurement mismatch: symmetric noise and a uniform 20mm offset look similar. Here each
point is attributed to one specific wall only if it is unambiguous (close to that wall and
far from every other), then the mean SIGNED perpendicular offset is reported per wall.

Sign convention: positive means the measured point lies BEYOND the mapped wall as seen from
the robot (the wall appears farther away than the map says).

A near-zero mean per wall means the map is right and residuals are pure sensor noise. A
consistent nonzero mean on some wall points at that wall being misplaced -- or at unmapped
objects (balls) sitting near it and being dragged into its correspondence set.
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


class WallBiasDiag(Node):
    def __init__(self):
        super().__init__('wall_bias_diag')
        self.declare_parameter('field_width', 5.572)
        self.declare_parameter('field_height', 3.310)
        self.declare_parameter('wall_thickness', 0.033)
        self.declare_parameter('wall_segment_x1', [0.0])
        self.declare_parameter('wall_segment_y1', [0.0])
        self.declare_parameter('wall_segment_x2', [0.0])
        self.declare_parameter('wall_segment_y2', [0.0])
        self.declare_parameter('report_every', 80)
        # Attribution gates: close enough to this wall, and clearly farther from all others.
        self.declare_parameter('attach_distance', 0.08)
        self.declare_parameter('exclusive_margin', 0.25)

        w = self.get_parameter('field_width').value
        h = self.get_parameter('field_height').value
        t = self.get_parameter('wall_thickness').value
        x1 = list(self.get_parameter('wall_segment_x1').value)
        y1 = list(self.get_parameter('wall_segment_y1').value)
        x2 = list(self.get_parameter('wall_segment_x2').value)
        y2 = list(self.get_parameter('wall_segment_y2').value)
        self.report_every = int(self.get_parameter('report_every').value)
        self.attach = float(self.get_parameter('attach_distance').value)
        self.margin = float(self.get_parameter('exclusive_margin').value)

        hw, hh = w / 2.0, h / 2.0
        self.segments = [
            ((hw, -hh), (hw, hh), 'perim_+x'),
            ((-hw, -hh), (-hw, hh), 'perim_-x'),
            ((-hw, hh), (hw, hh), 'perim_+y'),
            ((-hw, -hh), (hw, -hh), 'perim_-y'),
        ]
        for i in range(len(x1)):
            dx, dy = x2[i] - x1[i], y2[i] - y1[i]
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            nx, ny = -dy / length, dx / length
            ht = t * 0.5
            self.segments.append((
                (x1[i] + nx * ht, y1[i] + ny * ht),
                (x2[i] + nx * ht, y2[i] + ny * ht), f'int{i}+'))
            self.segments.append((
                (x1[i] - nx * ht, y1[i] - ny * ht),
                (x2[i] - nx * ht, y2[i] - ny * ht), f'int{i}-'))

        self.get_logger().info(f'map: {len(self.segments)} segments')
        self.gt_t, self.gt = [], []
        self.n = 0
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
        d = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * d)

    @staticmethod
    def perp(px, py, seg):
        """Perpendicular distance and unit normal of the infinite line through seg."""
        (ax, ay), (bx, by), _ = seg
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L <= 1e-12:
            return math.hypot(px - ax, py - ay), (0.0, 0.0), 0.0
        nx, ny = -dy / L, dx / L
        signed = (px - ax) * nx + (py - ay) * ny
        u = ((px - ax) * dx + (py - ay) * dy) / (L * L)
        return abs(signed), (nx, ny), u

    def on_scan(self, msg):
        self.n += 1
        if self.n % self.report_every:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.interp_gt(t)
        if pose is None:
            return
        px, py, yaw = pose
        cy, sy = math.cos(yaw), math.sin(yaw)

        stats = {}
        pts_by_label = {}
        wall_angle = {}
        for (ax, ay), (bx, by), label in self.segments:
            wall_angle.setdefault(label, math.atan2(by - ay, bx - ax))
        unattached = 0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            wx = px + cy * lx - sy * ly
            wy = py + sy * lx + cy * ly

            best = None
            second = 1e9
            for seg in self.segments:
                d, n, u = self.perp(wx, wy, seg)
                within = -0.02 <= u <= 1.02
                eff = d if within else 1e9
                if best is None or eff < best[0]:
                    if best is not None:
                        second = min(second, best[0])
                    best = (eff, seg, n)
                else:
                    second = min(second, eff)
            if best is None or best[0] > self.attach or second < best[0] + self.margin:
                unattached += 1
                continue
            # Signed along the outward direction seen from the robot.
            _, seg, (nx, ny) = best
            (ax, ay), _, label = seg
            signed = (wx - ax) * nx + (wy - ay) * ny
            to_wall = (ax - px) * nx + (ay - py) * ny
            if to_wall < 0:
                signed = -signed
            stats.setdefault(label, []).append(signed)
            pts_by_label.setdefault(label, []).append((wx, wy))

        parts = []
        for label, vals in sorted(stats.items(), key=lambda kv: -len(kv[1])):
            if len(vals) < 15:
                continue
            m = sum(vals) / len(vals)
            var = sum((v - m) ** 2 for v in vals) / len(vals)
            # Fit a line to the points that hit this wall and compare its direction to the
            # wall's mapped direction. A pure translation error leaves this at 0; a rotational
            # inconsistency between scan and map shows up here directly.
            pts = pts_by_label.get(label, [])
            tilt_txt = ''
            if len(pts) >= 30:
                mx = sum(p[0] for p in pts) / len(pts)
                my = sum(p[1] for p in pts) / len(pts)
                sxx = sum((p[0] - mx) ** 2 for p in pts)
                syy = sum((p[1] - my) ** 2 for p in pts)
                sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
                fit = 0.5 * math.atan2(2 * sxy, sxx - syy)
                wall_dir = wall_angle.get(label, 0.0)
                d = math.atan2(math.sin(fit - wall_dir), math.cos(fit - wall_dir))
                if d > math.pi / 2:
                    d -= math.pi
                if d < -math.pi / 2:
                    d += math.pi
                tilt_txt = f' tilt={math.degrees(d):+.3f}deg'
            parts.append(
                f'{label}: n={len(vals)} bias={1000 * m:+.1f}mm sd={1000 * math.sqrt(var):.1f}mm{tilt_txt}')
        self.get_logger().info(
            f'scan#{self.n} pose=({px:.3f},{py:.3f},{math.degrees(yaw):.1f}deg) '
            f'unattached={unattached} || ' + ' | '.join(parts))


def main():
    rclpy.init()
    node = WallBiasDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
