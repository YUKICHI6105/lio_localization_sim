#!/usr/bin/env python3
"""Consolidate a --record-stages bag into flat, analysis-ready CSVs.

Design principle (2026-07-31, per user direction): raw recordings should only carry
information that cannot be reconstructed by calculus (position/orientation, bias states,
node bookkeeping, sigma/covariance, discrete events). Velocity is NOT stored by the
recorder -- it is derived here, once, by central-difference numerical differentiation of
position, and saved alongside position in the same row so repeat analyses never need to
re-derive it. This also sidesteps the two frame-convention bugs hit today when a raw
twist/velocity field with an ambiguous frame (body vs world) was compared directly.

Usage: consolidate_stage_log.py <stage_derived_bag_dir> <out_dir>

Writes, under <out_dir>:
  ground_truth.csv, scan_match_wall.csv, state_estimate.csv, odom_fast.csv
    each with: t,x,y,yaw,vx,vy,yaw_rate (vx/vy/yaw_rate central-differenced from x,y,yaw)
    state_estimate.csv additionally carries accel_bias_x/y/z, gyro_bias_x/y/z, node_index,
    wall_factor_valid (none of these are derivable from position, so they are read directly)
  collisions.csv: t,name,relative_speed_mps (from /robot_collision, if present in the bag)
"""
import csv
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String as StringMsg
from lio_localization.msg import ScanMatchResult, StateEstimate


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    so = rosbag2_py.StorageOptions(uri=path, storage_id='mcap')
    co = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)
    type_map = {
        '/ground_truth_pose': Odometry,
        '/scan_match_result': ScanMatchResult,
        '/state_estimate': StateEstimate,
        '/odom_fast': Odometry,
        '/robot_collision': StringMsg,
    }
    out = {k: [] for k in type_map}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in type_map:
            continue
        msg = deserialize_message(data, type_map[topic])
        out[topic].append(msg)
    return out


def central_diff_series(rows):
    """rows: list of (t, x, y, yaw), sorted by t. Returns list of
    (t, x, y, yaw, vx, vy, yaw_rate) using central differences (one-sided at the ends)."""
    n = len(rows)
    out = []
    for i in range(n):
        t, x, y, yaw = rows[i]
        if n == 1:
            vx = vy = yaw_rate = 0.0
        elif i == 0:
            t1, x1, y1, yaw1 = rows[i + 1]
            dt = t1 - t
            vx = (x1 - x) / dt if dt > 0 else 0.0
            vy = (y1 - y) / dt if dt > 0 else 0.0
            yaw_rate = wrap(yaw1 - yaw) / dt if dt > 0 else 0.0
        elif i == n - 1:
            t0, x0, y0, yaw0 = rows[i - 1]
            dt = t - t0
            vx = (x - x0) / dt if dt > 0 else 0.0
            vy = (y - y0) / dt if dt > 0 else 0.0
            yaw_rate = wrap(yaw - yaw0) / dt if dt > 0 else 0.0
        else:
            t0, x0, y0, yaw0 = rows[i - 1]
            t1, x1, y1, yaw1 = rows[i + 1]
            dt = t1 - t0
            vx = (x1 - x0) / dt if dt > 0 else 0.0
            vy = (y1 - y0) / dt if dt > 0 else 0.0
            yaw_rate = wrap(yaw1 - yaw0) / dt if dt > 0 else 0.0
        out.append((t, x, y, yaw, vx, vy, yaw_rate))
    return out


def wrap(d):
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def write_pose_csv(path, rows_with_extra, extra_header, extra_getter):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate'] + extra_header)
        for row, extra in rows_with_extra:
            w.writerow(list(row) + extra_getter(extra))


def main():
    bag_path, out_dir = sys.argv[1], sys.argv[2]
    data = read_bag(bag_path)

    def gt_key(m):
        return (stamp_of(m), m.pose.pose.position.x, m.pose.pose.position.y,
                yaw_from_quat(m.pose.pose.orientation))
    gt_raw = sorted(data['/ground_truth_pose'], key=stamp_of)
    gt_rows = central_diff_series([gt_key(m) for m in gt_raw])
    write_pose_csv(f'{out_dir}/ground_truth.csv',
                   [(r, None) for r in gt_rows], [], lambda _: [])

    odom_raw = sorted(data['/odom_fast'], key=stamp_of)
    odom_rows = central_diff_series([gt_key(m) for m in odom_raw])
    write_pose_csv(f'{out_dir}/odom_fast.csv',
                   [(r, None) for r in odom_rows], [], lambda _: [])

    se_raw = sorted(data['/state_estimate'], key=stamp_of)

    def se_key(m):
        return (stamp_of(m), m.pose.position.x, m.pose.position.y,
                yaw_from_quat(m.pose.orientation))
    se_pos_rows = central_diff_series([se_key(m) for m in se_raw])
    write_pose_csv(
        f'{out_dir}/state_estimate.csv',
        list(zip(se_pos_rows, se_raw)),
        ['node_index', 'wall_factor_valid', 'accel_bias_x', 'accel_bias_y', 'accel_bias_z',
         'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z'],
        lambda m: [m.node_index, int(m.wall_factor_valid),
                   m.accel_bias.x, m.accel_bias.y, m.accel_bias.z,
                   m.gyro_bias.x, m.gyro_bias.y, m.gyro_bias.z])

    wall_raw = [m for m in sorted(data['/scan_match_result'], key=stamp_of) if m.wall.valid]

    def wall_key(m):
        return (stamp_of(m), m.wall.corrected_pose.position.x, m.wall.corrected_pose.position.y,
                yaw_from_quat(m.wall.corrected_pose.orientation))
    wall_pos_rows = central_diff_series([wall_key(m) for m in wall_raw])
    write_pose_csv(
        f'{out_dir}/scan_match_wall.csv',
        list(zip(wall_pos_rows, wall_raw)),
        ['correspondence_count', 'covariance_valid'],
        lambda m: [m.wall.correspondence_count, int(m.wall.covariance_valid)])

    with open(f'{out_dir}/collisions.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'name', 'relative_speed_mps'])
        for m in sorted(data['/robot_collision'], key=lambda m: m.data):
            # payload: "v=1;stamp_ns=...;name=...;relative_speed_mps=..."
            parts = dict(kv.split('=', 1) for kv in m.data.split(';') if '=' in kv)
            t = float(parts.get('stamp_ns', 0)) * 1e-9
            w.writerow([t, parts.get('name', ''), parts.get('relative_speed_mps', '')])

    print(f"wrote ground_truth.csv ({len(gt_rows)}), odom_fast.csv ({len(odom_rows)}), "
          f"state_estimate.csv ({len(se_pos_rows)}), scan_match_wall.csv ({len(wall_pos_rows)}), "
          f"collisions.csv ({len(data['/robot_collision'])}) to {out_dir}")


if __name__ == '__main__':
    main()
