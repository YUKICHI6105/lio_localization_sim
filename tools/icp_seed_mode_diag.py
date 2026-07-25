#!/usr/bin/env python3
"""Compare three ICP seeding strategies on one identical recorded scan sequence.

The production matcher lands ~14.8 mm from truth while the same scans, seeded from truth,
land within 1-5 mm. Two very different causes fit that observation and they call for opposite
fixes, so they have to be separated before anything is changed:

  truth   -- seeded at ground truth every frame. Not achievable in operation (it is handed the
             answer); it measures how much information the scan actually contains.
  offset  -- seeded at ground truth plus the observed bias every frame. If ICP walks back to
             truth, the geometry is well conditioned and the production error comes from a
             contaminated seed. If it sits still, ICP itself has a wide flat basin and the
             correspondence gate or the geometry is to blame.
  chained -- seeded from its own previous output, starting once at the initial pose. This is
             what "ICP only, no IMU" would actually do, which is the question that matters for
             deciding whether the IMU path is helping or hurting.

Scans are recorded first and processed afterwards rather than matched live: the chained mode
is only meaningful if it sees every scan in order, and a Python matcher cannot keep up with
40 Hz x 1081 points in real time. Recording removes that constraint entirely.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from lio_localization.msg import ScanMatchResult


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SegmentMap:
    """Wall segments as flat arrays so every point tests every segment in one vectorised pass."""

    def __init__(self, width, height, thickness, x1, y1, x2, y2):
        hw, hh = width / 2.0, height / 2.0
        segs = [((hw, -hh), (hw, hh)), ((-hw, -hh), (-hw, hh)),
                ((-hw, hh), (hw, hh)), ((-hw, -hh), (hw, -hh))]
        for i in range(len(x1)):
            dx, dy = x2[i] - x1[i], y2[i] - y1[i]
            L = math.hypot(dx, dy)
            if L <= 1e-9:
                continue
            nx, ny = -dy / L, dx / L
            ht = thickness * 0.5
            segs.append(((x1[i] + nx * ht, y1[i] + ny * ht), (x2[i] + nx * ht, y2[i] + ny * ht)))
            segs.append(((x1[i] - nx * ht, y1[i] - ny * ht), (x2[i] - nx * ht, y2[i] - ny * ht)))

        self.a = np.array([s[0] for s in segs], dtype=float)
        self.b = np.array([s[1] for s in segs], dtype=float)
        d = self.b - self.a
        self.d = d
        self.l2 = np.maximum((d * d).sum(axis=1), 1e-12)
        length = np.sqrt(self.l2)
        self.normal = np.stack([-d[:, 1] / length, d[:, 0] / length], axis=1)

    def closest(self, pts):
        """Nearest point on any segment for each input point, with that segment's normal."""
        rel = pts[:, None, :] - self.a[None, :, :]
        u = np.clip((rel * self.d[None, :, :]).sum(axis=2) / self.l2[None, :], 0.0, 1.0)
        q = self.a[None, :, :] + u[:, :, None] * self.d[None, :, :]
        diff = pts[:, None, :] - q
        d2 = (diff * diff).sum(axis=2)
        idx = np.argmin(d2, axis=1)
        rows = np.arange(pts.shape[0])
        return d2[rows, idx], q[rows, idx], self.normal[idx]


def icp(seed, local, smap, corr, iters=20, huber=None, clamp=None):
    """Point-to-line Gauss-Newton ICP.

    huber=None gives plain least squares. Passing the production delta matters: with delta
    below the sensor sigma the cost stops being convex, so where the solver ends up starts to
    depend on where it began -- which is exactly the behaviour under test here.
    """
    cx, cy, cyaw = seed
    for _ in range(iters):
        c, s = math.cos(cyaw), math.sin(cyaw)
        world = np.stack([cx + c * local[:, 0] - s * local[:, 1],
                          cy + s * local[:, 0] + c * local[:, 1]], axis=1)
        d2, q, nrm = smap.closest(world)
        keep = d2 <= corr * corr
        if keep.sum() < 40:
            break
        w = world[keep]
        e = ((w - q[keep]) * nrm[keep]).sum(axis=1)
        lx, ly = local[keep, 0], local[keep, 1]
        dxdyaw = -s * lx - c * ly
        dydyaw = c * lx - s * ly
        j = np.stack([nrm[keep, 0], nrm[keep, 1],
                      nrm[keep, 0] * dxdyaw + nrm[keep, 1] * dydyaw], axis=1)
        if huber is None:
            wt = np.ones_like(e)
        else:
            ae = np.abs(e)
            wt = np.where(ae <= huber, 1.0, huber / np.maximum(ae, 1e-12))
        h = (j * wt[:, None]).T @ j + np.eye(3) * 1e-9
        g = (j * wt[:, None]).T @ e
        try:
            delta = np.linalg.solve(h, -g)
        except np.linalg.LinAlgError:
            break
        if clamp is not None:
            delta[0] = np.clip(delta[0], -clamp[0], clamp[0])
            delta[1] = np.clip(delta[1], -clamp[0], clamp[0])
            delta[2] = np.clip(delta[2], -clamp[1], clamp[1])
        cx += delta[0]
        cy += delta[1]
        cyaw += delta[2]
        if math.hypot(delta[0], delta[1]) < 1e-6 and abs(delta[2]) < 1e-7:
            break
    return cx, cy, cyaw


class Stats:
    def __init__(self, name):
        self.name = name
        self.dx, self.dy, self.dyaw = [], [], []

    def add(self, dx, dy, dyaw):
        self.dx.append(dx)
        self.dy.append(dy)
        self.dyaw.append(dyaw)

    def text(self):
        if not self.dx:
            return f'{self.name}: (no data)'
        def ms(v):
            m = float(np.mean(v))
            return m, float(np.std(v))
        mx, sx = ms(self.dx)
        my, sy = ms(self.dy)
        ma, sa = ms(self.dyaw)
        rms = float(np.sqrt(np.mean(np.array(self.dx) ** 2 + np.array(self.dy) ** 2)))
        return (f'{self.name:<9} n={len(self.dx):<5} '
                f'dx={1000 * mx:+7.2f}+-{1000 * sx:5.2f}mm '
                f'dy={1000 * my:+7.2f}+-{1000 * sy:5.2f}mm '
                f'|bias|={1000 * math.hypot(mx, my):6.2f}mm RMSE={1000 * rms:6.2f}mm '
                f'dyaw={math.degrees(ma):+.4f}deg')


class IcpSeedModeDiag(Node):
    def __init__(self):
        super().__init__('icp_seed_mode_diag')
        self.declare_parameter('field_width', 5.572)
        self.declare_parameter('field_height', 3.310)
        self.declare_parameter('wall_thickness', 0.033)
        self.declare_parameter('wall_segment_x1', [0.0])
        self.declare_parameter('wall_segment_y1', [0.0])
        self.declare_parameter('wall_segment_x2', [0.0])
        self.declare_parameter('wall_segment_y2', [0.0])
        self.declare_parameter('planar_correspondence_distance', 0.20)
        self.declare_parameter('collect_sec', 55.0)
        self.declare_parameter('settle_sec', 10.0)
        # The bias measured at the matcher output, used as the deliberate seed perturbation.
        self.declare_parameter('offset_x', 0.00679)
        self.declare_parameter('offset_y', -0.01333)

        g = self.get_parameter
        self.smap = SegmentMap(
            g('field_width').value, g('field_height').value, g('wall_thickness').value,
            list(g('wall_segment_x1').value), list(g('wall_segment_y1').value),
            list(g('wall_segment_x2').value), list(g('wall_segment_y2').value))
        self.corr = float(g('planar_correspondence_distance').value)
        self.collect_sec = float(g('collect_sec').value)
        self.settle_sec = float(g('settle_sec').value)
        self.offset = (float(g('offset_x').value), float(g('offset_y').value))

        self.gt_t, self.gt = [], []
        self.scans = []
        self.t0 = None
        self.done = False

        # The production matcher's own answer, keyed by the stamp of the scan that produced it.
        # Recording it here rather than in a separate node is what puts both solvers on the
        # same footing: identical scan, identical instant, identical robot pose. Earlier
        # comparisons ran the offline solver on an open-loop session (no stack means the
        # controller falls back to ground truth and the robot stops somewhere else entirely),
        # so the two numbers described different geometry and could not be compared at all.
        self.prod = {}

        self.create_subscription(Odometry, '/ground_truth_pose', self.on_gt, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(ScanMatchResult, '/scan_match_result', self.on_match, 50)
        self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f'icp_seed_mode_diag: recording {self.collect_sec}s, '
            f'{len(self.smap.a)} segments, corr={self.corr}m')

    def on_gt(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_t.append(t)
        self.gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                        yaw_from_quat(msg.pose.pose.orientation)))

    def on_scan(self, msg):
        if self.done:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.scans.append((t, np.stack([r[ok] * np.cos(ang[ok]),
                                        r[ok] * np.sin(ang[ok])], axis=1)))

    def on_match(self, msg):
        if not msg.wall.valid:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.wall.corrected_pose
        self.prod[round(t, 6)] = (p.position.x, p.position.y,
                                  yaw_from_quat(p.orientation))

    def interp(self, t):
        ts = self.gt_t
        if len(ts) < 2 or t < ts[0] or t > ts[-1]:
            return None
        i = int(np.searchsorted(ts, t))
        if i <= 0:
            return self.gt[0]
        t0, t1 = ts[i - 1], ts[i]
        a, b = self.gt[i - 1], self.gt[i]
        r = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        d = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * d)

    def tick(self):
        if self.done or self.t0 is None:
            return
        if self.scans[-1][0] - self.t0 < self.collect_sec:
            return
        self.done = True
        self.analyse()

    def analyse(self):
        log = self.get_logger().info
        log(f'recorded {len(self.scans)} scans; running three seeding modes')

        # Production settings, so the seed test is run against the solver that is actually
        # deployed rather than the plain least-squares one used for the first pass.
        prod = dict(iters=10, huber=0.025, clamp=(0.10, 0.08))
        modes = ('production', 'truth', 'offset', 'chained', 'truth_prod', 'offset_prod')
        st = {k: Stats(k) for k in modes}
        matched = 0
        chain = None
        for t, local in self.scans:
            g = self.interp(t)
            if g is None:
                continue
            if chain is None:
                # A real ICP-only system knows where it was placed; give it that and nothing more.
                chain = g
            settled = (t - self.t0) >= self.settle_sec

            off_seed = (g[0] + self.offset[0], g[1] + self.offset[1], g[2])
            res = {
                'truth': icp(g, local, self.smap, self.corr),
                'offset': icp(off_seed, local, self.smap, self.corr),
                'truth_prod': icp(g, local, self.smap, self.corr, **prod),
                'offset_prod': icp(off_seed, local, self.smap, self.corr, **prod),
            }
            chain = icp(chain, local, self.smap, self.corr)
            res['chained'] = chain

            # Only count a scan the production matcher also answered on, so every mode's
            # statistics are drawn from exactly the same set of instants.
            prod_pose = self.prod.get(round(t, 6))
            if prod_pose is not None:
                res['production'] = prod_pose
                matched += 1

            if not settled:
                continue
            for k, (x, y, yaw) in res.items():
                st[k].add(x - g[0], y - g[1],
                          math.atan2(math.sin(yaw - g[2]), math.cos(yaw - g[2])))

        log('=' * 100)
        log(f'seed perturbation applied to "offset": '
            f'dx={1000 * self.offset[0]:+.2f}mm dy={1000 * self.offset[1]:+.2f}mm '
            f'(|{1000 * math.hypot(*self.offset):.2f}|mm)')
        log(f'production results matched by stamp: {matched} of {len(self.scans)} scans '
            f'({len(self.prod)} received)')
        log('plain = least squares, 20 iters. _prod = huber 0.025 / 10 iters / clamped, '
            'as deployed. "production" is the running matcher on these same scans.')
        for k in modes:
            log(st[k].text())
        log('=' * 100)


def main():
    rclpy.init()
    node = IcpSeedModeDiag()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
