#!/usr/bin/env python3
"""Convert Unity's transport-independent sensor CSV recording into a rosbag2 MCAP."""

from __future__ import annotations

import argparse
import csv
import hashlib
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import rosbag2_py
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, LaserScan


NS_PER_SEC = 1_000_000_000
TOPICS = {
    "imu": ("/imu/data", "sensor_msgs/msg/Imu"),
    "scan": ("/scan", "sensor_msgs/msg/LaserScan"),
    "ground_truth": ("/ground_truth_pose", "nav_msgs/msg/Odometry"),
}


@dataclass
class ConvertedRecord:
    stamp_ns: int
    priority: int
    topic: str
    message: Any


def parse_scalar(value: str) -> float:
    normalized = value.strip().lower()
    if normalized == "inf":
        return math.inf
    if normalized == "-inf":
        return -math.inf
    if normalized == "nan":
        return math.nan
    return float(value)


def parse_array(value: str, expected_length: int | None = None) -> list[float]:
    values = [] if value == "" else [parse_scalar(item) for item in value.split(";")]
    if expected_length is not None and len(values) != expected_length:
        raise ValueError(f"expected {expected_length} values, got {len(values)}")
    return values


def stamp_from_ns(stamp_ns: int) -> Time:
    if stamp_ns < 0:
        raise ValueError(f"negative timestamp: {stamp_ns}")
    stamp = Time()
    stamp.sec = stamp_ns // NS_PER_SEC
    stamp.nanosec = stamp_ns % NS_PER_SEC
    return stamp


def set_header(header: Any, row: dict[str, str], stamp_ns: int) -> None:
    header.stamp = stamp_from_ns(stamp_ns)
    header.frame_id = row["frame_id"]


def imu_from_row(row: dict[str, str], stamp_ns: int) -> Imu:
    message = Imu()
    set_header(message.header, row, stamp_ns)
    message.orientation.x = parse_scalar(row["orientation_x"])
    message.orientation.y = parse_scalar(row["orientation_y"])
    message.orientation.z = parse_scalar(row["orientation_z"])
    message.orientation.w = parse_scalar(row["orientation_w"])
    message.orientation_covariance = parse_array(row["orientation_covariance"], 9)
    message.angular_velocity.x = parse_scalar(row["angular_velocity_x"])
    message.angular_velocity.y = parse_scalar(row["angular_velocity_y"])
    message.angular_velocity.z = parse_scalar(row["angular_velocity_z"])
    message.angular_velocity_covariance = parse_array(row["angular_velocity_covariance"], 9)
    message.linear_acceleration.x = parse_scalar(row["linear_acceleration_x"])
    message.linear_acceleration.y = parse_scalar(row["linear_acceleration_y"])
    message.linear_acceleration.z = parse_scalar(row["linear_acceleration_z"])
    message.linear_acceleration_covariance = parse_array(row["linear_acceleration_covariance"], 9)
    return message


def scan_from_row(row: dict[str, str], stamp_ns: int) -> LaserScan:
    message = LaserScan()
    set_header(message.header, row, stamp_ns)
    for name in (
        "angle_min", "angle_max", "angle_increment", "time_increment", "scan_time",
        "range_min", "range_max",
    ):
        setattr(message, name, parse_scalar(row[name]))
    message.ranges = parse_array(row["ranges"])
    message.intensities = parse_array(row["intensities"])
    return message


def odometry_from_row(row: dict[str, str], stamp_ns: int) -> Odometry:
    message = Odometry()
    set_header(message.header, row, stamp_ns)
    message.child_frame_id = row["child_frame_id"]
    pose = message.pose.pose
    pose.position.x = parse_scalar(row["position_x"])
    pose.position.y = parse_scalar(row["position_y"])
    pose.position.z = parse_scalar(row["position_z"])
    pose.orientation.x = parse_scalar(row["orientation_x"])
    pose.orientation.y = parse_scalar(row["orientation_y"])
    pose.orientation.z = parse_scalar(row["orientation_z"])
    pose.orientation.w = parse_scalar(row["orientation_w"])
    message.pose.covariance = parse_array(row["pose_covariance"], 36)
    twist = message.twist.twist
    twist.linear.x = parse_scalar(row["linear_x"])
    twist.linear.y = parse_scalar(row["linear_y"])
    twist.linear.z = parse_scalar(row["linear_z"])
    twist.angular.x = parse_scalar(row["angular_x"])
    twist.angular.y = parse_scalar(row["angular_y"])
    twist.angular.z = parse_scalar(row["angular_z"])
    message.twist.covariance = parse_array(row["twist_covariance"], 36)
    return message


ROW_CONVERTERS = {
    "imu": imu_from_row,
    "scan": scan_from_row,
    "ground_truth": odometry_from_row,
}


def iter_records(recording_dir: Path, kind: str, priority: int) -> Iterator[ConvertedRecord]:
    csv_path = recording_dir / {"imu": "imu.csv", "scan": "scan.csv", "ground_truth": "ground_truth.csv"}[kind]
    expected_sequence = 0
    previous_stamp = -1
    converter = ROW_CONVERTERS[kind]
    with csv_path.open("r", newline="", encoding="utf-8") as source:
        for line_number, row in enumerate(csv.DictReader(source), start=2):
            try:
                sequence = int(row["seq"])
                stamp_ns = int(row["stamp_ns"])
                if sequence != expected_sequence:
                    raise ValueError(f"sequence {sequence}, expected {expected_sequence}")
                if stamp_ns < previous_stamp:
                    raise ValueError(f"timestamp {stamp_ns} is before {previous_stamp}")
                message = converter(row, stamp_ns)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"{csv_path}:{line_number}: {error}") from error
            expected_sequence += 1
            previous_stamp = stamp_ns
            yield ConvertedRecord(stamp_ns, priority, TOPICS[kind][0], message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_manifest(recording_dir: Path, allow_incomplete: bool) -> dict[str, Any]:
    manifest_path = recording_dir / "manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {manifest_path}: {error}") from error
    if manifest.get("schema_version") != 1:
        raise ValueError(f"unsupported schema_version: {manifest.get('schema_version')}")
    if not manifest.get("complete", False) and not allow_incomplete:
        raise ValueError("recording manifest is incomplete; stop Unity cleanly or pass --allow-incomplete")
    return manifest


def convert(recording_dir: Path, output_bag: Path, allow_incomplete: bool) -> dict[str, Any]:
    manifest = read_manifest(recording_dir, allow_incomplete)
    if output_bag.exists():
        raise ValueError(f"output path already exists: {output_bag}")
    for name in ("imu.csv", "scan.csv", "ground_truth.csv"):
        if not (recording_dir / name).is_file():
            raise ValueError(f"missing recording file: {recording_dir / name}")

    custom_data = {
        "source": "unity_authoritative_sensor_csv",
        "run_id": str(manifest.get("run_id", "")),
        "manifest_sha256": sha256(recording_dir / "manifest.json"),
    }
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="mcap", custom_data=custom_data),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for topic_id, kind in enumerate(("imu", "scan", "ground_truth"), start=1):
        topic_name, topic_type = TOPICS[kind]
        writer.create_topic(rosbag2_py.TopicMetadata(topic_id, topic_name, topic_type, "cdr"))

    heap: list[tuple[int, int, int, ConvertedRecord, Iterator[ConvertedRecord]]] = []
    for priority, kind in enumerate(("imu", "ground_truth", "scan")):
        iterator = iter_records(recording_dir, kind, priority)
        try:
            record = next(iterator)
        except StopIteration:
            continue
        heapq.heappush(heap, (record.stamp_ns, record.priority, 0, record, iterator))

    counts = {kind: 0 for kind in TOPICS}
    previous_bag_stamp = -1
    sequence_number = 0
    while heap:
        stamp_ns, _, _, record, iterator = heapq.heappop(heap)
        if stamp_ns < previous_bag_stamp:
            raise AssertionError("merged stream is not timestamp ordered")
        writer.write(record.topic, serialize_message(record.message), stamp_ns)
        counts[next(kind for kind, topic in TOPICS.items() if topic[0] == record.topic)] += 1
        previous_bag_stamp = stamp_ns
        try:
            next_record = next(iterator)
        except StopIteration:
            continue
        sequence_number += 1
        heapq.heappush(heap, (next_record.stamp_ns, next_record.priority, sequence_number, next_record, iterator))
    writer.close()

    expected_counts = manifest.get("written_counts", {})
    for kind, count in counts.items():
        expected = expected_counts.get(kind)
        if expected is not None and count != expected:
            raise ValueError(f"{kind} row count {count} does not match manifest {expected}")

    conversion_manifest = {
        "schema_version": 1,
        "source_recording": str(recording_dir.resolve()),
        "source_manifest_sha256": custom_data["manifest_sha256"],
        "bag_storage": "mcap",
        "counts": counts,
        "start_stamp_ns": None if previous_bag_stamp < 0 else min(
            next(iter_records(recording_dir, kind, 0)).stamp_ns for kind in TOPICS if counts[kind]),
        "end_stamp_ns": None if previous_bag_stamp < 0 else previous_bag_stamp,
    }
    (output_bag / "unity_csv_conversion.json").write_text(
        json.dumps(conversion_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return conversion_manifest


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert Unity authoritative sensor CSV files to a rosbag2 MCAP directory.")
    parser.add_argument("recording_dir", type=Path, help="Unity SensorRecordings/<run_id> directory")
    parser.add_argument("output_bag", type=Path, help="new rosbag2 output directory")
    parser.add_argument("--allow-incomplete", action="store_true",
                        help="allow a recording whose Unity manifest was not finalized")
    arguments = parser.parse_args()
    try:
        result = convert(arguments.recording_dir, arguments.output_bag, arguments.allow_incomplete)
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
