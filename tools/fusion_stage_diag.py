#!/usr/bin/env python3
"""Localise a steady-state error by comparing every stage of the pipeline to ground truth.

The scan-only diagnostic (icp_from_truth_diag) showed the scan supports the true pose to
within a few mm while the delivered estimate sits ~14 mm away, so the loss happens somewhere
between the matcher and the published state. There are only three places it can be:

  * /scan_match_result wall.corrected_pose -- what the production planar ICP actually solved,
    seeded from the current estimate rather than from truth. If this is already off by the
    full error, the matcher is sitting in a seed-dependent fixed point and fusion is faithful.
  * /state_estimate -- the smoother's fused pose. Off here but not upstream means the wall
    prior is being outvoted or mis-weighted.
  * /odom_fast -- the IMU propagation the evaluator actually scores. Off here but not upstream
    means the error is added during propagation.

Each is compared against ground truth interpolated to that message's own stamp, so a constant
timestamp offset cannot masquerade as position error while the robot is moving. The stamp
skews between streams are reported alongside, since that was a real defect earlier in this
project and needs to be re-checked rather than assumed fixed.

Errors are reported as signed dx/dy, not magnitude: a constant offset and a noisy zero-mean
scatter have the same magnitude but completely different causes.
"""
import bisect
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from lio_localization.msg import ScanMatchResult, StateEstimate


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Acc:
    """Signed running stats; mean separates a bias from scatter, sd measures the scatter."""

    def __init__(self):
        self.dx, self.dy, self.dyaw = [], [], []

    def add(self, dx, dy, dyaw):
        self.dx.append(dx)
        self.dy.append(dy)
        self.dyaw.append(dyaw)

    def text(self, name):
        if not self.dx:
            return f'{name}: (no data)'
        def ms(v):
            m = sum(v) / len(v)
            sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
            return m, sd
        mx, sx = ms(self.dx)
        my, sy = ms(self.dy)
        ma, sa = ms(self.dyaw)
        mag = math.hypot(mx, my)
        return (f'{name}: n={len(self.dx)} '
                f'dx={1000 * mx:+.2f}+-{1000 * sx:.2f}mm '
                f'dy={1000 * my:+.2f}+-{1000 * sy:.2f}mm '
                f'|bias|={1000 * mag:.2f}mm '
                f'dyaw={math.degrees(ma):+.4f}+-{math.degrees(sa):.4f}deg')


class FusionStageDiag(Node):
    def __init__(self):
        super().__init__('fusion_stage_diag')
        # Skip the startup transient: the estimate takes ~2 s to reach its steady offset and
        # averaging that in would blur the very bias we are trying to measure.
        self.settle_sec = self.declare_parameter('settle_sec', 8.0).value
        self.window_sec = self.declare_parameter('window_sec', 0.0).value
        self.report_every_sec = self.declare_parameter('report_every_sec', 10.0).value

        # Independent replication of the backend's point-of-no-return test, evaluated on
        # ground truth rather than on the estimate. Deciding the collision windows here, from
        # data the estimator never sees, is what makes this a check on the mechanism instead
        # of a restatement of it: the backend could fire at the wrong moments and still look
        # self-consistent in its own logs. Comparing odom error inside vs outside these windows
        # then measures whether hard scan trust actually helps where it is aimed.
        self.collision_max_decel = self.declare_parameter('collision_max_decel', 4.905).value
        self.collision_robot_radius = self.declare_parameter('collision_robot_radius', 0.29).value
        self.collision_min_speed = self.declare_parameter('collision_min_speed', 0.3).value
        self.collision_hold_sec = self.declare_parameter('collision_hold_sec', 0.15).value
        fw = self.declare_parameter('field_width', 0.0).value
        fh = self.declare_parameter('field_height', 0.0).value
        wx1 = list(self.declare_parameter('wall_segment_x1', [0.0]).value)
        wy1 = list(self.declare_parameter('wall_segment_y1', [0.0]).value)
        wx2 = list(self.declare_parameter('wall_segment_x2', [0.0]).value)
        wy2 = list(self.declare_parameter('wall_segment_y2', [0.0]).value)
        self.coll_segments = []
        if fw > 0.0 and fh > 0.0:
            hw, hh = fw / 2.0, fh / 2.0
            self.coll_segments += [(hw, -hh, hw, hh), (-hw, -hh, -hw, hh),
                                   (-hw, hh, hw, hh), (-hw, -hh, hw, -hh)]
        for i in range(min(len(wx1), len(wy1), len(wx2), len(wy2))):
            self.coll_segments.append((wx1[i], wy1[i], wx2[i], wy2[i]))
        self.ponr_until = None
        self.ponr_fires = 0
        # Error magnitudes split by whether the instant falls in a collision-imminent window.
        self.coll_window_err = {'inside': [], 'outside': []}

        self.gt_t, self.gt = [], []
        self.t0 = None

        self.match = Acc()
        self.state = Acc()
        self.odom = Acc()

        self.last_imu_stamp = None
        self.imu_dt = []
        self.last_scan_stamp = None
        self.scan_dt = []
        # Stamp skew between the newest IMU and each arriving scan: the earlier defect in this
        # project was exactly this kind of constant offset between streams.
        self.scan_vs_imu = []
        self.gyro_bias_z = []
        self.accel_bias = []
        self.speed_bins = {}
        self.accel_bins = {}            # odom_fast (IMU-propagated), evaluated stream
        self.accel_bins_match = {}      # scan_match wall.corrected_pose
        self.accel_bins_state = {}      # fused state_estimate

        self.create_subscription(Odometry, '/ground_truth_pose', self.on_gt, 50)
        self.create_subscription(ScanMatchResult, '/scan_match_result', self.on_match, 20)
        self.create_subscription(StateEstimate, '/state_estimate', self.on_state, 20)
        self.create_subscription(Odometry, '/odom_fast', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.create_timer(self.report_every_sec, self.report)
        self.get_logger().info(f'fusion_stage_diag started (settle={self.settle_sec}s)')

    def on_gt(self, msg):
        t = stamp_sec(msg.header.stamp)
        if self.t0 is None:
            self.t0 = t
        self.gt_t.append(t)
        self.gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                        yaw_from_quat(msg.pose.pose.orientation)))
        if len(self.gt_t) > 40000:
            del self.gt_t[:20000]
            del self.gt[:20000]

    def interp(self, t):
        """Ground truth at t, or None when t falls outside the samples we hold.

        Returning None rather than clamping matters: clamping would silently convert a stamp
        that is ahead of the truth stream into a position error proportional to speed.
        """
        if len(self.gt_t) < 2 or t < self.gt_t[0] or t > self.gt_t[-1]:
            return None
        i = bisect.bisect_left(self.gt_t, t)
        if i <= 0:
            return self.gt[0]
        t0, t1 = self.gt_t[i - 1], self.gt_t[i]
        a, b = self.gt[i - 1], self.gt[i]
        r = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        d = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * d)

    def _settled(self, t):
        if self.t0 is None:
            return False
        rel = t - self.t0
        # window_sec bounds the far end so the moving phase can be isolated. The stationary
        # hold that follows dominates by sample count and would otherwise bury it.
        if self.window_sec > 0.0 and rel > self.window_sec:
            return False
        return rel >= self.settle_sec

    def _record(self, acc, t, x, y, yaw):
        if not self._settled(t):
            return
        g = self.interp(t)
        if g is None:
            return
        acc.add(x - g[0], y - g[1],
                math.atan2(math.sin(yaw - g[2]), math.cos(yaw - g[2])))

    def gt_velocity(self, t, h=0.02):
        """Ground-truth velocity by central difference, or None outside the samples."""
        a = self.interp(t - h)
        b = self.interp(t + h)
        if a is None or b is None:
            return None
        return ((b[0] - a[0]) / (2 * h), (b[1] - a[1]) / (2 * h))

    def gt_accel_mag(self, t, h=0.02):
        """Ground-truth |acceleration| by central second difference.

        A collision drives this far above the robot's own 4.9 m/s^2 control limit, so binning
        localisation error by this value isolates exactly the collision instants: if the error
        does not climb in the high-acceleration bins, the estimator is riding through the
        impacts rather than being thrown by them.
        """
        va = self.gt_velocity(t - h, h)
        vb = self.gt_velocity(t + h, h)
        if va is None or vb is None:
            return None
        return math.hypot((vb[0] - va[0]) / (2 * h), (vb[1] - va[1]) / (2 * h))

    def _record_speed_bin(self, t, ex, ey):
        """Split the error into along-track and cross-track, bucketed by speed.

        A pure timing offset between the scan and the pose history it is matched against can
        only displace the answer along the direction of travel, by speed x lag. So along-track
        error that grows with speed while cross-track stays flat identifies a lag, and
        along/speed gives its size directly. Any other cause has no reason to respect that
        decomposition.
        """
        v = self.gt_velocity(t)
        if v is None:
            return
        speed = math.hypot(v[0], v[1])
        if speed < 0.02:
            key = 'still'
        else:
            key = f'{0.2 * int(speed / 0.2):.1f}-{0.2 * int(speed / 0.2) + 0.2:.1f}'
        along = (ex * v[0] + ey * v[1]) / speed if speed > 1e-9 else 0.0
        cross = (-ex * v[1] + ey * v[0]) / speed if speed > 1e-9 else 0.0
        self.speed_bins.setdefault(key, []).append((speed, along, cross, math.hypot(ex, ey)))

    def on_match(self, msg):
        if not msg.wall.valid:
            return
        t = stamp_sec(msg.header.stamp)
        p = msg.wall.corrected_pose
        self._record(self.match, t, p.position.x, p.position.y,
                     yaw_from_quat(p.orientation))
        g = self.interp(t)
        if g is not None and self._settled(t):
            self._record_speed_bin(t, p.position.x - g[0], p.position.y - g[1])
            self._accel_bin(self.accel_bins_match, t, p.position.x - g[0], p.position.y - g[1])

    def on_state(self, msg):
        t = stamp_sec(msg.header.stamp)
        self._record(self.state, t, msg.pose.position.x, msg.pose.position.y,
                     yaw_from_quat(msg.pose.orientation))
        if self._settled(t):
            self.gyro_bias_z.append(msg.gyro_bias.z)
            self.accel_bias.append(math.hypot(msg.accel_bias.x, msg.accel_bias.y))
            g = self.interp(t)
            if g is not None:
                self._accel_bin(self.accel_bins_state, t,
                                msg.pose.position.x - g[0], msg.pose.position.y - g[1])

    def _forward_clearance(self, px, py, ux, uy):
        """Distance to the first wall along the heading, less the robot radius."""
        best = float('inf')
        for ax, ay, bx, by in self.coll_segments:
            ex, ey = bx - ax, by - ay
            denom = ux * ey - uy * ex
            if abs(denom) < 1e-12:
                continue
            wx, wy = ax - px, ay - py
            t = (wx * ey - wy * ex) / denom
            s = (wx * uy - wy * ux) / denom
            if t >= 0.0 and 0.0 <= s <= 1.0 and t < best:
                best = t
        return best - self.collision_robot_radius

    def _ponr_now(self, t):
        """True while a collision is kinematically unavoidable, judged on ground truth."""
        if not self.coll_segments:
            return False
        g = self.interp(t)
        v = self.gt_velocity(t)
        if g is None or v is None:
            return False
        speed = math.hypot(v[0], v[1])
        if speed >= self.collision_min_speed:
            clearance = self._forward_clearance(g[0], g[1], v[0] / speed, v[1] / speed)
            if math.isfinite(clearance):
                stopping = speed * speed / (2.0 * self.collision_max_decel)
                if stopping >= clearance:
                    if self.ponr_until is None or t > self.ponr_until:
                        self.ponr_fires += 1
                    self.ponr_until = t + self.collision_hold_sec
        return self.ponr_until is not None and t <= self.ponr_until

    def _accel_bin(self, bins, t, ex, ey):
        """Bin one stage's error magnitude by |ground-truth acceleration|.

        Splitting the scan, the fused state and the propagated odom this way is the direct test
        of the user's point: at a collision the IMU truly senses the impact (1 kHz = physics
        rate in Unity, no aliasing). If the scan pose stays accurate through the collision but
        the propagated odom does not, the impact information is present yet not being turned
        into a correction fast enough -- a fusion problem, not a sensing limit.
        """
        acc = self.gt_accel_mag(t)
        if acc is None:
            return
        if acc < 5.0:
            akey = f'{2.0 * int(acc / 2.0):.0f}-{2.0 * int(acc / 2.0) + 2.0:.0f}'
        else:
            akey = f'{5.0 * int(acc / 5.0):.0f}-{5.0 * int(acc / 5.0) + 5.0:.0f}(coll)'
        bins.setdefault(akey, []).append(math.hypot(ex, ey))

    def on_odom(self, msg):
        t = stamp_sec(msg.header.stamp)
        p = msg.pose.pose
        self._record(self.odom, t, p.position.x, p.position.y, yaw_from_quat(p.orientation))
        g = self.interp(t)
        if g is None or not self._settled(t):
            return
        ex, ey = p.position.x - g[0], p.position.y - g[1]
        self._accel_bin(self.accel_bins, t, ex, ey)
        key = 'inside' if self._ponr_now(t) else 'outside'
        self.coll_window_err[key].append(math.hypot(ex, ey))

    def on_imu(self, msg):
        t = stamp_sec(msg.header.stamp)
        if self.last_imu_stamp is not None:
            self.imu_dt.append(t - self.last_imu_stamp)
        self.last_imu_stamp = t

    def on_scan(self, msg):
        t = stamp_sec(msg.header.stamp)
        if self.last_scan_stamp is not None:
            self.scan_dt.append(t - self.last_scan_stamp)
        self.last_scan_stamp = t
        if self.last_imu_stamp is not None:
            self.scan_vs_imu.append(t - self.last_imu_stamp)

    @staticmethod
    def _dt_text(name, dts, nominal):
        if not dts:
            return f'{name}: (no data)'
        n = len(dts)
        m = sum(dts) / n
        sd = math.sqrt(sum((x - m) ** 2 for x in dts) / n)
        return (f'{name}: n={n} mean={1000 * m:.3f}ms (nominal {1000 * nominal:.3f}) '
                f'sd={1000 * sd:.3f}ms min={1000 * min(dts):.3f} max={1000 * max(dts):.3f}')

    def report(self):
        log = self.get_logger().info
        log('=' * 78)
        log('--- stage vs ground truth (signed, settled samples only) ---')
        log(self.match.text('scan_match wall.corrected_pose'))
        log(self.state.text('state_estimate (fused)      '))
        log(self.odom.text('odom_fast (evaluated)       '))
        if self.speed_bins:
            log('--- matcher error split by speed (along = direction of travel) ---')
            for key in sorted(self.speed_bins,
                              key=lambda k: -1.0 if k == 'still' else float(k.split('-')[0])):
                v = self.speed_bins[key]
                sp = sum(x[0] for x in v) / len(v)
                al = sum(x[1] for x in v) / len(v)
                cr = sum(x[2] for x in v) / len(v)
                rms = math.sqrt(sum(x[3] ** 2 for x in v) / len(v))
                mx = max(x[3] for x in v)
                lag = (al / sp * 1000.0) if sp > 0.02 else float('nan')
                log(f'  speed {key:>9} m/s: n={len(v):<5} mean_speed={sp:.3f} '
                    f'|err| rms={1000 * rms:6.2f}mm max={1000 * mx:6.2f}mm  '
                    f'along={1000 * al:+6.2f} cross={1000 * cr:+6.2f}mm lag={lag:+6.2f}ms')
        if self.accel_bins:
            log('--- error by |accel|: scan_match vs fused vs odom (collision test) ---')
            def akey_sort(k):
                return float(k.split('-')[0])
            def stage_stat(bins, key):
                v = bins.get(key)
                if not v:
                    return '   -  '
                return f'{1000 * (sum(v) / len(v)):5.1f}/{1000 * max(v):5.1f}'
            allkeys = set(self.accel_bins) | set(self.accel_bins_match) \
                | set(self.accel_bins_state)
            log(f'  {"accel m/s^2":>16}  {"n":>5}  {"scan mean/max":>13}  '
                f'{"fused mean/max":>13}  {"odom mean/max":>13}')
            for key in sorted(allkeys, key=akey_sort):
                n = len(self.accel_bins.get(key, []))
                log(f'  {key:>16}  {n:>5}  {stage_stat(self.accel_bins_match, key):>13}  '
                    f'{stage_stat(self.accel_bins_state, key):>13}  '
                    f'{stage_stat(self.accel_bins, key):>13}')
        if any(self.coll_window_err.values()):
            log(f'--- odom error in/out of collision-imminent windows '
                f'({self.ponr_fires} windows, judged on ground truth) ---')
            for key in ('inside', 'outside'):
                v = self.coll_window_err[key]
                if not v:
                    log(f'  {key:>8}: (none)')
                    continue
                s = sorted(v)
                p95 = s[min(int(len(s) * 0.95), len(s) - 1)]
                log(f'  {key:>8}: n={len(v):<6} mean={1000 * sum(v) / len(v):6.2f}mm '
                    f'p95={1000 * p95:6.2f}mm max={1000 * max(v):6.2f}mm')
        log('--- stamp regularity ---')
        log(self._dt_text('imu  stamp dt', self.imu_dt, 1.0 / 250.0))
        log(self._dt_text('scan stamp dt', self.scan_dt, 1.0 / 40.0))
        if self.scan_vs_imu:
            v = self.scan_vs_imu
            m = sum(v) / len(v)
            log(f'scan stamp - newest imu stamp: n={len(v)} mean={1000 * m:+.3f}ms '
                f'min={1000 * min(v):+.3f} max={1000 * max(v):+.3f}')
        if self.gyro_bias_z:
            g = self.gyro_bias_z
            a = self.accel_bias
            log(f'estimated bias: gyro_z mean={sum(g) / len(g):+.6f}rad/s '
                f'(last={g[-1]:+.6f})  |accel_xy| mean={sum(a) / len(a):.6f}m/s^2')


def main():
    rclpy.init()
    node = FusionStageDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
