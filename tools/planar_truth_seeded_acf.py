#!/usr/bin/env python3
"""Measure the Unity planar scan matcher without odometry/backend feedback.

Every LaserScan is solved from interpolated ground truth.  This makes the signed
registration-error autocorrelation a frontend-only diagnostic.  Use --start and
--count to process bounded chunks, then --summarize to combine their CSV files.
"""

import argparse
import bisect
import csv
import glob
import math
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def make_segments(params):
    hw, hh = params["field_width"] / 2.0, params["field_height"] / 2.0
    segments = [((hw, -hh), (hw, hh)), ((-hw, -hh), (-hw, hh)),
                ((-hw, hh), (hw, hh)), ((-hw, -hh), (hw, -hh))]
    half_thickness = params["wall_thickness"] * 0.5
    for x1, y1, x2, y2 in zip(
            params["wall_segment_x1"], params["wall_segment_y1"],
            params["wall_segment_x2"], params["wall_segment_y2"]):
        d = np.array([x2 - x1, y2 - y1], dtype=float)
        n = np.array([-d[1], d[0]], dtype=float) / np.linalg.norm(d)
        a, b = np.array([x1, y1]), np.array([x2, y2])
        segments.extend(((a + n * half_thickness, b + n * half_thickness),
                         (a - n * half_thickness, b - n * half_thickness)))
    a = np.asarray([x[0] for x in segments], dtype=float)
    b = np.asarray([x[1] for x in segments], dtype=float)
    d = b - a
    return a, d, np.sum(d * d, axis=1)


def nearest(world, segment_a, segment_d, segment_l2):
    rel = world[:, None, :] - segment_a[None, :, :]
    u = np.clip(np.sum(rel * segment_d[None, :, :], axis=2) /
                segment_l2[None, :], 0.0, 1.0)
    projection = segment_a[None, :, :] + u[:, :, None] * segment_d[None, :, :]
    delta = world[:, None, :] - projection
    d2 = np.sum(delta * delta, axis=2)
    idx = np.argmin(d2, axis=1)
    rows = np.arange(world.shape[0])
    tangent = segment_d[idx] / np.sqrt(segment_l2[idx])[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    return d2[rows, idx], projection[rows, idx], normal


def solve_point_to_line(local, pose_start, params, segment_a, segment_d, segment_l2):
    pose = pose_start.copy()
    corr2 = params["planar_correspondence_distance"] ** 2
    huber = params["planar_huber_delta"]
    n_inliers = 0
    mean_abs_residual = math.inf
    for _ in range(int(params["planar_max_iterations"])):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = np.column_stack((pose[0] + c * local[:, 0] - s * local[:, 1],
                                 pose[1] + s * local[:, 0] + c * local[:, 1]))
        d2, projection, normal = nearest(world, segment_a, segment_d, segment_l2)
        mask = d2 <= corr2
        world, projection, normal = world[mask], projection[mask], normal[mask]
        active_local = local[mask]
        n_inliers = len(world)
        if n_inliers < int(params["planar_min_inliers"]):
            break
        residual = np.sum(normal * (world - projection), axis=1)
        d_yaw = np.column_stack((-s * active_local[:, 0] - c * active_local[:, 1],
                                 c * active_local[:, 0] - s * active_local[:, 1]))
        jacobian = np.column_stack((normal, np.sum(normal * d_yaw, axis=1)))
        weights = np.minimum(1.0, huber / np.maximum(np.abs(residual), 1e-15))
        hessian = jacobian.T @ (weights[:, None] * jacobian)
        gradient = jacobian.T @ (weights * residual)
        hessian[np.diag_indices(3)] += 1e-6
        delta = -np.linalg.solve(hessian, gradient)
        pose[0] += np.clip(delta[0], -0.10, 0.10)
        pose[1] += np.clip(delta[1], -0.10, 0.10)
        pose[2] += np.clip(delta[2], -0.08, 0.08)
        mean_abs_residual = float(np.mean(np.abs(residual)))
        if np.linalg.norm(delta) < 1e-5:
            break
    return pose, n_inliers, mean_abs_residual


def solve_ray_to_wall(ranges, angles, pose_start, params, segment_a, segment_d, segment_l2):
    """Maximum-likelihood pose for range-Gaussian beams against finite map segments."""
    pose = pose_start.copy()
    normals = np.column_stack((-segment_d[:, 1], segment_d[:, 0])) / \
        np.sqrt(segment_l2)[:, None]
    a = segment_a
    huber = params["planar_huber_delta"]
    gate = 0.120  # Four sigma under the stress bag's 30 mm range-noise model.
    n_inliers = 0
    mean_abs_residual = math.inf
    unit_local = np.column_stack((np.cos(angles), np.sin(angles)))
    for _ in range(int(params["planar_max_iterations"])):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        direction = np.column_stack((c * unit_local[:, 0] - s * unit_local[:, 1],
                                     s * unit_local[:, 0] + c * unit_local[:, 1]))
        d_yaw = np.column_stack((-direction[:, 1], direction[:, 0]))
        denom = direction @ normals.T
        numer = np.sum((a - pose[:2]) * normals, axis=1)[None, :]
        predicted_all = numer / np.where(np.abs(denom) > 1e-9, denom, np.nan)
        # Intersection must lie on the finite segment, not merely on its supporting line.
        hit = pose[:2] + predicted_all[:, :, None] * direction[:, None, :]
        rel = hit - a[None, :, :]
        segment_u = np.sum(rel * segment_d[None, :, :], axis=2) / segment_l2[None, :]
        valid = ((predicted_all > 0.0) & (segment_u >= -1e-9) &
                 (segment_u <= 1.0 + 1e-9) & np.isfinite(predicted_all))
        predicted_all[~valid] = np.inf
        segment_index = np.argmin(predicted_all, axis=1)
        predicted = predicted_all[np.arange(len(ranges)), segment_index]
        residual = ranges - predicted
        mask = np.isfinite(predicted) & (np.abs(residual) <= gate)
        n_inliers = int(np.count_nonzero(mask))
        if n_inliers < int(params["planar_min_inliers"]):
            break
        active = segment_index[mask]
        n = normals[active]
        d = direction[mask]
        dd = d_yaw[mask]
        den = np.sum(n * d, axis=1)
        num = np.sum(n * (a[active] - pose[:2]), axis=1)
        # residual = measured range - predicted range
        jacobian = np.column_stack((n[:, 0] / den, n[:, 1] / den,
                                   num * np.sum(n * dd, axis=1) / (den * den)))
        active_residual = residual[mask]
        weights = np.minimum(1.0, huber / np.maximum(np.abs(active_residual), 1e-15))
        hessian = jacobian.T @ (weights[:, None] * jacobian)
        gradient = jacobian.T @ (weights * active_residual)
        hessian[np.diag_indices(3)] += 1e-6
        delta = -np.linalg.solve(hessian, gradient)
        pose[0] += np.clip(delta[0], -0.10, 0.10)
        pose[1] += np.clip(delta[1], -0.10, 0.10)
        pose[2] += np.clip(delta[2], -0.08, 0.08)
        mean_abs_residual = float(np.mean(np.abs(active_residual)))
        if np.linalg.norm(delta) < 1e-5:
            break
    return pose, n_inliers, mean_abs_residual


def load_bag(bag_uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_uri, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    msg_types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    gt, scans = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/ground_truth_pose":
            msg = deserialize_message(data, msg_types[topic])
            gt.append((stamp_sec(msg.header.stamp), msg.pose.pose.position.x,
                       msg.pose.pose.position.y, yaw_from_quat(msg.pose.pose.orientation)))
        elif topic == "/scan":
            msg = deserialize_message(data, msg_types[topic])
            timestamp = stamp_sec(msg.header.stamp)
            if 3.0 <= timestamp <= 62.0:
                scans.append((timestamp, msg))
    return gt, scans


def interpolate_gt(gt, t):
    times = [x[0] for x in gt]
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return np.asarray(gt[0][1:], dtype=float)
    if i >= len(gt):
        return np.asarray(gt[-1][1:], dtype=float)
    a, b = gt[i - 1], gt[i]
    ratio = (t - a[0]) / (b[0] - a[0])
    return np.asarray((a[1] + ratio * (b[1] - a[1]),
                       a[2] + ratio * (b[2] - a[2]),
                       a[3] + ratio * wrap(b[3] - a[3])), dtype=float)


def scan_to_local(msg, decimation):
    ranges = np.asarray(msg.ranges)[::decimation]
    indices = np.arange(len(msg.ranges))[::decimation]
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    ranges, indices = ranges[valid], indices[valid]
    angle = msg.angle_min + indices * msg.angle_increment
    return ranges, angle, np.column_stack((ranges * np.cos(angle), ranges * np.sin(angle)))


def run_chunk(args):
    with open(args.yaml) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    gt, scans = load_bag(args.bag)
    start = args.start
    end = min(start + args.count, len(scans))
    segment_a, segment_d, segment_l2 = make_segments(params)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(("scan_index", "time_sec", "dx_mm", "dy_mm", "dyaw_deg",
                         "position_error_mm", "inliers", "mean_abs_residual_mm"))
        for index in range(start, end):
            timestamp, msg = scans[index]
            truth = interpolate_gt(gt, timestamp)
            ranges, angles, local = scan_to_local(msg, max(int(params["scan_decimation"]), 1))
            if args.solver == "point_to_line":
                estimate, inliers, residual = solve_point_to_line(
                    local, truth, params, segment_a, segment_d, segment_l2)
            else:
                estimate, inliers, residual = solve_ray_to_wall(
                    ranges, angles, truth, params, segment_a, segment_d, segment_l2)
            dx, dy = (estimate[:2] - truth[:2]) * 1000.0
            dyaw = math.degrees(wrap(estimate[2] - truth[2]))
            writer.writerow((index, f"{timestamp:.9f}", f"{dx:.9f}", f"{dy:.9f}",
                             f"{dyaw:.9f}", f"{math.hypot(dx, dy):.9f}", inliers,
                             f"{residual * 1000.0:.9f}"))
    print(f"wrote {end - start} {args.solver} scans ({start}..{end - 1}) to {out}")


def acf(values, lag):
    if len(values) <= lag:
        return math.nan
    a, b = values[:-lag], values[lag:]
    a, b = a - np.mean(a), b - np.mean(b)
    denom = math.sqrt(float(np.dot(a, a) * np.dot(b, b)))
    return float(np.dot(a, b) / denom) if denom else math.nan


def summarize(pattern):
    rows = []
    for filename in glob.glob(pattern):
        with open(filename, newline="") as f:
            rows.extend(csv.DictReader(f))
    rows.sort(key=lambda x: int(x["scan_index"]))
    indexes = np.asarray([int(x["scan_index"]) for x in rows])
    if len(indexes) == 0 or not np.array_equal(indexes, np.arange(len(indexes))):
        raise RuntimeError("CSV chunks must cover every scan exactly once from index 0")
    print(f"truth-seeded planar registration: {len(rows)} scans")
    for field in ("dx_mm", "dy_mm", "dyaw_deg"):
        values = np.asarray([float(x[field]) for x in rows])
        print(f"  {field}: mean={np.mean(values):+.3f}, std={np.std(values):.3f}, "
              f"ACF " + ", ".join(f"{lag}:{acf(values, lag):+.3f}"
                                    for lag in (1, 2, 4, 8, 20, 40)))
    error = np.asarray([float(x["position_error_mm"]) for x in rows])
    print(f"  position: mean={np.mean(error):.3f}mm rmse={math.sqrt(np.mean(error**2)):.3f}mm "
          f"p99={np.percentile(error, 99):.3f}mm max={np.max(error):.3f}mm")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag")
    parser.add_argument("--yaml")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=320)
    parser.add_argument("--output")
    parser.add_argument("--solver", choices=("point_to_line", "ray_to_wall"),
                        default="point_to_line")
    parser.add_argument("--summarize")
    args = parser.parse_args()
    if args.summarize:
        summarize(args.summarize)
    elif args.bag and args.yaml and args.output:
        run_chunk(args)
    else:
        parser.error("provide --summarize PATTERN or --bag, --yaml, and --output")


if __name__ == "__main__":
    main()
