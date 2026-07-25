#!/usr/bin/env python3
"""Run point-to-line ICP starting AT the ground-truth pose and report where it converges.

This is the decisive split for a systematic localisation error:

  * If ICP stays at ~0 correction, the scan genuinely supports the true pose. The map and the
    sensor model are fine, and the error must be introduced downstream -- in the factor-graph
    fusion (IMU bias, prior weighting) rather than in scan matching.

  * If ICP walks away to ~the observed error, the scan itself supports a pose that far from
    truth. Then the map/sensor geometry is at fault and no amount of fusion tuning will help.

Correspondence gating mirrors the production planar matcher so the answer is representative.
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


class IcpFromTruth(Node):
    def __init__(self):
        super().__init__('icp_from_truth_diag')
        self.declare_parameter('field_width', 5.572)
        self.declare_parameter('field_height', 3.310)
        self.declare_parameter('wall_thickness', 0.033)
        self.declare_parameter('wall_segment_x1', [0.0])
        self.declare_parameter('wall_segment_y1', [0.0])
        self.declare_parameter('wall_segment_x2', [0.0])
        self.declare_parameter('wall_segment_y2', [0.0])
        self.declare_parameter('planar_correspondence_distance', 0.20)
        self.declare_parameter('planar_max_iterations', 20)
        self.declare_parameter('report_every', 80)

        w = self.get_parameter('field_width').value
        h = self.get_parameter('field_height').value
        t = self.get_parameter('wall_thickness').value
        x1 = list(self.get_parameter('wall_segment_x1').value)
        y1 = list(self.get_parameter('wall_segment_y1').value)
        x2 = list(self.get_parameter('wall_segment_x2').value)
        y2 = list(self.get_parameter('wall_segment_y2').value)
        self.corr = float(self.get_parameter('planar_correspondence_distance').value)
        self.iters = int(self.get_parameter('planar_max_iterations').value)
        self.report_every = int(self.get_parameter('report_every').value)

        hw, hh = w / 2.0, h / 2.0
        self.segs = [((hw, -hh), (hw, hh)), ((-hw, -hh), (-hw, hh)),
                     ((-hw, hh), (hw, hh)), ((-hw, -hh), (hw, -hh))]
        for i in range(len(x1)):
            dx, dy = x2[i] - x1[i], y2[i] - y1[i]
            L = math.hypot(dx, dy)
            if L <= 1e-9:
                continue
            nx, ny = -dy / L, dx / L
            ht = t * 0.5
            self.segs.append(((x1[i] + nx * ht, y1[i] + ny * ht), (x2[i] + nx * ht, y2[i] + ny * ht)))
            self.segs.append(((x1[i] - nx * ht, y1[i] - ny * ht), (x2[i] - nx * ht, y2[i] - ny * ht)))

        self.get_logger().info(f'map: {len(self.segs)} segments, corr={self.corr}m iters={self.iters}')
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

    def closest(self, px, py):
        """Nearest point on any segment plus that segment's unit normal."""
        best_d, best_pt, best_n = 1e9, None, None
        for (ax, ay), (bx, by) in self.segs:
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 <= 1e-12:
                continue
            u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            qx, qy = ax + u * dx, ay + u * dy
            d = math.hypot(px - qx, py - qy)
            if d < best_d:
                L = math.sqrt(L2)
                best_d, best_pt, best_n = d, (qx, qy), (-dy / L, dx / L)
        return best_d, best_pt, best_n

    def on_scan(self, msg):
        self.n += 1
        if self.n % self.report_every:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.interp_gt(t)
        if pose is None:
            return
        tx, ty, tyaw = pose

        local = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            local.append((r * math.cos(a), r * math.sin(a)))

        # Start exactly at ground truth; let point-to-line ICP move if the data wants it to.
        cx, cy, cyaw = tx, ty, tyaw
        used = 0
        for _ in range(self.iters):
            c, s = math.cos(cyaw), math.sin(cyaw)
            H = [[0.0] * 3 for _ in range(3)]
            g = [0.0, 0.0, 0.0]
            used = 0
            for lx, ly in local:
                wx = cx + c * lx - s * ly
                wy = cy + s * lx + c * ly
                d, q, nrm = self.closest(wx, wy)
                if q is None or d > self.corr:
                    continue
                nx, ny = nrm
                e = (wx - q[0]) * nx + (wy - q[1]) * ny
                # d(point)/d(yaw) for the rotated local offset
                dxdyaw = -s * lx - c * ly
                dydyaw = c * lx - s * ly
                J = (nx, ny, nx * dxdyaw + ny * dydyaw)
                for a_ in range(3):
                    g[a_] += J[a_] * e
                    for b_ in range(3):
                        H[a_][b_] += J[a_] * J[b_]
                used += 1
            if used < 40:
                break
            for a_ in range(3):
                H[a_][a_] += 1e-9
            try:
                delta = solve3(H, [-v for v in g])
            except ZeroDivisionError:
                break
            cx += delta[0]
            cy += delta[1]
            cyaw += delta[2]
            if math.hypot(delta[0], delta[1]) < 1e-6 and abs(delta[2]) < 1e-7:
                break

        ex, ey = cx - tx, cy - ty
        eyaw = math.atan2(math.sin(cyaw - tyaw), math.cos(cyaw - tyaw))
        self.get_logger().info(
            f'scan#{self.n} truth=({tx:.3f},{ty:.3f},{math.degrees(tyaw):.2f}deg) '
            f'corr_pts={used} || ICP moved dx={1000 * ex:+.1f}mm dy={1000 * ey:+.1f}mm '
            f'dyaw={math.degrees(eyaw):+.3f}deg |d|={1000 * math.hypot(ex, ey):.1f}mm')


def solve3(A, b):
    m = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-15:
            raise ZeroDivisionError
        m[col], m[piv] = m[piv], m[col]
        pv = m[col][col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / pv
            for k in range(col, 4):
                m[r][k] -= f * m[col][k]
    return [m[i][3] / m[i][i] for i in range(3)]


def main():
    rclpy.init()
    node = IcpFromTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
