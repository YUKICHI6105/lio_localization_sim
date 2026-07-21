"""main.md 6-A-1: 精度評価ノード。

/ground_truth_pose と /odom_fast を突き合わせて誤差を記録し、要件A
（定常誤差±10mm、ドリフト±20mm/3分）に対する合否と、スピンバースト・
ボール横切り期間中の誤差を個別に報告する。最後にグラフをPNGで保存する。
"""

import bisect
import math
import statistics

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray

from lio_localization_sim.trajectory import Trajectory


def yaw_from_quat(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Sample:
    __slots__ = ('t', 'x', 'y', 'yaw')

    def __init__(self, t, x, y, yaw):
        self.t = t
        self.x = x
        self.y = y
        self.yaw = yaw


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator_node')

        field_width = self.declare_parameter('field_width', 10.0).value
        field_height = self.declare_parameter('field_height', 10.0).value
        self.duration_sec = self.declare_parameter('duration_sec', 180.0).value
        self.settle_sec = self.declare_parameter('settle_sec', 5.0).value
        self.output_dir = self.declare_parameter('output_dir', '/tmp').value
        # imu_sim/lidar_simと共有する時刻基準。これで揃えないと、メッセージの
        # stampが表す「軌道上のt」と評価器がここで言う「t」がズレる。
        self.sim_start_time_sec = self.declare_parameter('sim_start_time_sec', 0.0).value

        pattern = self.declare_parameter('trajectory_pattern', 0).value
        self.traj = Trajectory(field_width, field_height, pattern=pattern)

        self.gt_samples: list[Sample] = []
        self.gt_times: list[float] = []
        self.errors: list[tuple] = []  # (t, pos_err, yaw_err_deg)
        self.ball_detections = 0

        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth_pose', self.gt_callback, 50)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_fast', self.odom_callback, 50)
        self.ball_sub = self.create_subscription(
            PoseArray, '/detected_balls', self.ball_callback, 10)

        self.report_timer = self.create_timer(5.0, self.periodic_report)
        self.finish_timer = self.create_timer(self.duration_sec, self.finish)

        self.get_logger().info(
            f'evaluator_node started: will run for {self.duration_sec}s then report')

    def _stamp_to_t(self, stamp) -> float:
        # 到着時刻(通信・処理遅延を含む)ではなく、送信側が付けたメッセージの
        # stampを使う。到着時刻で比較すると、ノード間の処理遅延の差がそのまま
        # 誤差に混入する(fableのレビュー指摘)。
        return (stamp.sec + stamp.nanosec * 1e-9) - self.sim_start_time_sec

    def gt_callback(self, msg: Odometry):
        t = self._stamp_to_t(msg.header.stamp)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.gt_samples.append(Sample(t, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
        self.gt_times.append(t)

    def ball_callback(self, msg: PoseArray):
        self.ball_detections = len(msg.poses)

    def _nearest_gt(self, t: float):
        if not self.gt_times:
            return None
        idx = bisect.bisect_left(self.gt_times, t)
        if idx <= 0:
            return self.gt_samples[0]
        if idx >= len(self.gt_times):
            return self.gt_samples[-1]
        before = self.gt_samples[idx - 1]
        after = self.gt_samples[idx]
        return before if (t - before.t) <= (after.t - t) else after

    def odom_callback(self, msg: Odometry):
        t = self._stamp_to_t(msg.header.stamp)
        gt = self._nearest_gt(t)
        if gt is None:
            return
        ex = msg.pose.pose.position.x - gt.x
        ey = msg.pose.pose.position.y - gt.y
        pos_err = math.hypot(ex, ey)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        yaw_err_deg = math.degrees(abs(wrap_angle(yaw - gt.yaw)))
        self.errors.append((t, pos_err, yaw_err_deg))

    def _in_any_window(self, t, windows):
        return any(t0 <= t <= t0 + dur for (t0, dur) in windows)

    def periodic_report(self):
        if not self.errors:
            self.get_logger().warning('no /odom_fast samples matched to ground truth yet')
            return
        recent = [e for e in self.errors if e[0] >= max(0.0, self.errors[-1][0] - 5.0)]
        pos_errs = [e[1] * 1000.0 for e in recent]
        self.get_logger().info(
            f't={self.errors[-1][0]:.1f}s  直近5秒: pos_err mean={statistics.mean(pos_errs):.2f}mm '
            f'max={max(pos_errs):.2f}mm  samples={len(self.errors)}  detected_balls={self.ball_detections}')

    def finish(self):
        self.finish_timer.cancel()
        self.get_logger().info('=== 評価終了、集計します ===')

        if not self.errors:
            self.get_logger().error('/odom_fast のサンプルが1件もありません。ノード構成を確認してください。')
            rclpy.shutdown()
            return

        steady = [e for e in self.errors if e[0] >= self.settle_sec]
        pos_mm = [e[1] * 1000.0 for e in steady]
        yaw_deg = [e[2] for e in steady]

        spin_pos_mm = [e[1] * 1000.0 for e in steady if self._in_any_window(e[0], self.traj.spin_bursts)]
        ball_window = [self.traj.ball_active_window]
        ball_pos_mm = [e[1] * 1000.0 for e in steady if self._in_any_window(e[0], ball_window)]
        non_ball_pos_mm = [e[1] * 1000.0 for e in steady if not self._in_any_window(e[0], ball_window)]

        def stats(vals):
            if not vals:
                return (float('nan'), float('nan'), float('nan'))
            rmse = math.sqrt(sum(v * v for v in vals) / len(vals))
            return (statistics.mean(vals), rmse, max(vals))

        mean_all, rmse_all, max_all = stats(pos_mm)
        mean_spin, rmse_spin, max_spin = stats(spin_pos_mm)
        mean_ball, rmse_ball, max_ball = stats(ball_pos_mm)
        mean_nonball, rmse_nonball, max_nonball = stats(non_ball_pos_mm)
        mean_yaw, rmse_yaw, max_yaw = stats(yaw_deg)

        steady_ok = rmse_all <= 10.0
        drift_ok = max_all <= 20.0
        ball_ok = (math.isnan(mean_ball) or math.isnan(mean_nonball) or
                   mean_ball <= mean_nonball * 3.0 + 5.0)

        report = []
        report.append('=' * 60)
        report.append('main.md 6-A-1 シミュレーション評価レポート')
        report.append('=' * 60)
        report.append(f'総サンプル数: {len(self.errors)}  (定常区間 t>={self.settle_sec}s: {len(steady)})')
        report.append('')
        report.append('--- 全体（要件A: 定常±10mm, 最大±20mm） ---')
        report.append(f'  位置誤差: mean={mean_all:.2f}mm  RMSE={rmse_all:.2f}mm  max={max_all:.2f}mm')
        report.append(f'  ヨー誤差: mean={mean_yaw:.3f}deg  RMSE={rmse_yaw:.3f}deg  max={max_yaw:.3f}deg')
        report.append(f'  判定: 定常誤差(RMSE<=10mm) {"PASS" if steady_ok else "FAIL"} / '
                       f'最大ドリフト(<=20mm) {"PASS" if drift_ok else "FAIL"}')
        report.append('')
        report.append('--- 高G旋回スピンバースト中 ---')
        report.append(f'  位置誤差: mean={mean_spin:.2f}mm  RMSE={rmse_spin:.2f}mm  max={max_spin:.2f}mm')
        report.append('')
        report.append('--- ボール横切り中 vs 非横切り中 ---')
        report.append(f'  横切り中   : mean={mean_ball:.2f}mm  RMSE={rmse_ball:.2f}mm  max={max_ball:.2f}mm')
        report.append(f'  非横切り中 : mean={mean_nonball:.2f}mm  RMSE={rmse_nonball:.2f}mm  max={max_nonball:.2f}mm')
        report.append(f'  判定: ボール横切りによる自己位置無影響 {"PASS" if ball_ok else "FAIL"}')
        report.append(f'  (参考) 横切り中の検出ボール数(最新): {self.ball_detections}')
        report.append('=' * 60)

        text = '\n'.join(report)
        self.get_logger().info('\n' + text)

        report_path = f'{self.output_dir}/sim_eval_report.txt'
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
        self.get_logger().info(f'レポートを保存しました: {report_path}')

        # 診断用一時計測(バイアス推定遅れ仮説の検証、原因特定後に削除予定)
        with open(f'{self.output_dir}/diag_raw_errors.csv', 'w', encoding='utf-8') as f:
            f.write('t,pos_err_mm,yaw_err_deg\n')
            for t, pos_err, yaw_err_deg in self.errors:
                f.write(f'{t:.6f},{pos_err * 1000.0:.4f},{yaw_err_deg:.4f}\n')

        self._save_plot()
        rclpy.shutdown()

    def _save_plot(self):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning('matplotlibが無いためグラフ保存をスキップします')
            return

        fig, axes = plt.subplots(2, 1, figsize=(10, 10))

        gx = [s.x for s in self.gt_samples]
        gy = [s.y for s in self.gt_samples]
        axes[0].plot(gx, gy, label='ground truth', linewidth=1, color='tab:blue')
        axes[0].set_xlabel('x [m]')
        axes[0].set_ylabel('y [m]')
        axes[0].set_title('Ground truth trajectory')
        axes[0].axis('equal')
        axes[0].legend()
        axes[0].grid(True)

        ts = [e[0] for e in self.errors]
        errs = [e[1] * 1000.0 for e in self.errors]
        axes[1].plot(ts, errs, linewidth=0.8, color='tab:red')
        axes[1].axhline(10.0, color='green', linestyle='--', label='10mm target')
        axes[1].axhline(20.0, color='orange', linestyle='--', label='20mm target')
        for (t0, dur) in self.traj.spin_bursts:
            axes[1].axvspan(t0, t0 + dur, color='purple', alpha=0.15)
        t0, t1 = self.traj.ball_active_window
        axes[1].axvspan(t0, t1, color='blue', alpha=0.15)
        axes[1].set_xlabel('time [s]')
        axes[1].set_ylabel('position error [mm]')
        axes[1].set_title('Position error over time (purple=spin burst, blue=ball crossing)')
        axes[1].legend()
        axes[1].grid(True)

        fig.tight_layout()
        plot_path = f'{self.output_dir}/sim_eval_plot.png'
        fig.savefig(plot_path, dpi=120)
        self.get_logger().info(f'グラフを保存しました: {plot_path}')


def main():
    rclpy.init()
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
