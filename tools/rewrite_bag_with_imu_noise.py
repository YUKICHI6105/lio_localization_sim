#!/usr/bin/env python3
"""Write an MCAP copy with declared ICM-42688-P IMU perturbations added.

The transformation happens before playback, so a Python callback cannot create
timing pressure in the 1 kHz ROS localization path being measured.
"""

import argparse
import math
from pathlib import Path
import random

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sample-rate-hz", type=float, default=1000.0)
    parser.add_argument("--accel-density", type=float, default=0.000686466,
                        help="m/s^2/sqrt(Hz), ICM-42688-P conservative 70 ug/sqrt(Hz)")
    parser.add_argument("--gyro-density", type=float, default=0.0000488692,
                        help="rad/s/sqrt(Hz), ICM-42688-P 2.8 mdps/sqrt(Hz)")
    parser.add_argument("--seed", type=int, default=42688)
    parser.add_argument("--temperature-ramp-c", type=float, default=0.0,
                        help="Linear IMU temperature change over the recording (degC). "
                             "Uses ICM-42688-P worst-direction temperature coefficients: "
                             "accel zero-g 0.15 mg/degC, accel scale 0.005 %%/degC, "
                             "gyro offset 5 mdps/degC.")
    parser.add_argument("--vibration-accel-amplitude-mps2", type=float, default=0.0,
                        help="Peak sinusoidal acceleration added on the IMU X axis (m/s^2).")
    parser.add_argument("--vibration-frequency-hz", type=float, default=100.0,
                        help="Frequency for --vibration-accel-amplitude-mps2 (Hz).")
    parser.add_argument("--accel-scale-fraction", type=float, default=1.0,
                        help="Static multiplicative accelerometer scale factor.")
    parser.add_argument("--accel-cross-axis-fraction", type=float, default=0.0,
                        help="Static X<-Y accelerometer cross-axis coupling fraction.")
    parser.add_argument("--gyro-scale-fraction", type=float, default=1.0,
                        help="Static multiplicative gyroscope scale factor.")
    parser.add_argument("--gyro-cross-axis-fraction", type=float, default=0.0,
                        help="Static X<-Y gyroscope cross-axis coupling fraction.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f"refusing to overwrite existing output: {args.output}")
    if args.sample_rate_hz <= 0.0:
        raise ValueError("sample rate must be positive")
    if args.vibration_frequency_hz < 0.0:
        raise ValueError("vibration frequency must be non-negative")
    accel_sigma = args.accel_density * math.sqrt(args.sample_rate_hz)
    gyro_sigma = args.gyro_density * math.sqrt(args.sample_rate_hz)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.source), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    metadata = reader.get_all_topics_and_types()
    message_types = {item.name: get_message(item.type) for item in metadata}
    imu_duration_ns = 1
    if args.temperature_ramp_c != 0.0:
        # rosbag2 has no per-topic duration API.  Probe the immutable source once
        # before the transform, then reopen it below for the actual copy.
        probe = rosbag2_py.SequentialReader()
        probe.open(rosbag2_py.StorageOptions(uri=str(args.source), storage_id="mcap"),
                   rosbag2_py.ConverterOptions("cdr", "cdr"))
        first_stamp_ns = None
        last_stamp_ns = None
        while probe.has_next():
            topic, data, _ = probe.read_next()
            if topic != "/imu/data":
                continue
            message = deserialize_message(data, message_types[topic])
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if first_stamp_ns is None:
                first_stamp_ns = stamp_ns
            last_stamp_ns = stamp_ns
        if first_stamp_ns is None or last_stamp_ns is None or last_stamp_ns <= first_stamp_ns:
            raise ValueError("temperature ramp requires at least two monotonic IMU samples")
        imu_duration_ns = last_stamp_ns - first_stamp_ns
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(args.output), storage_id="mcap",
            custom_data={
                "source": str(args.source),
                "injection": "ICM-42688-P white IMU noise and/or temperature ramp",
                "accel_density_mps2_per_sqrt_hz": str(args.accel_density),
                "gyro_density_radps_per_sqrt_hz": str(args.gyro_density),
                "sample_rate_hz": str(args.sample_rate_hz),
                "seed": str(args.seed),
                "temperature_ramp_c": str(args.temperature_ramp_c),
                "vibration_accel_amplitude_mps2": str(args.vibration_accel_amplitude_mps2),
                "vibration_frequency_hz": str(args.vibration_frequency_hz),
                "accel_scale_fraction": str(args.accel_scale_fraction),
                "accel_cross_axis_fraction": str(args.accel_cross_axis_fraction),
                "gyro_scale_fraction": str(args.gyro_scale_fraction),
                "gyro_cross_axis_fraction": str(args.gyro_cross_axis_fraction),
            }),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic_id, item in enumerate(metadata, start=1):
        writer.create_topic(rosbag2_py.TopicMetadata(topic_id, item.name, item.type, "cdr"))

    rng = random.Random(args.seed)
    imu_count = 0
    first_imu_stamp_ns = None
    # Datasheet DS-000347, worst-direction coefficients.  The signs are selected
    # coherently positive to make the test a reproducible bounded sensitivity case,
    # not a claim about an unmeasured board's calibration direction.
    accel_zero_g_per_c = 0.15e-3 * 9.80665
    accel_scale_per_c = 0.005e-2
    gyro_offset_per_c = 5e-3 * math.pi / 180.0
    gyro_scale_per_c = 0.005e-2
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic == "/imu/data":
            message = deserialize_message(data, message_types[topic])
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if first_imu_stamp_ns is None:
                first_imu_stamp_ns = stamp_ns
            # The probe above gives the full authoritative IMU duration, so this
            # is exactly a linear 0->ramp profile over its recorded interval.
            elapsed_ns = max(0, stamp_ns - first_imu_stamp_ns)
            temperature_delta_c = 0.0
            if args.temperature_ramp_c != 0.0:
                temperature_delta_c = args.temperature_ramp_c * elapsed_ns / imu_duration_ns
            for axis in ("x", "y", "z"):
                if temperature_delta_c:
                    value = getattr(message.linear_acceleration, axis)
                    value = value * (1.0 + accel_scale_per_c * temperature_delta_c)
                    setattr(message.linear_acceleration, axis,
                            value + accel_zero_g_per_c * temperature_delta_c)
                    gyro_value = getattr(message.angular_velocity, axis)
                    gyro_value *= 1.0 + gyro_scale_per_c * temperature_delta_c
                    setattr(message.angular_velocity, axis,
                            gyro_value + gyro_offset_per_c * temperature_delta_c)
                setattr(message.linear_acceleration, axis,
                        getattr(message.linear_acceleration, axis) + rng.gauss(0.0, accel_sigma))
                setattr(message.angular_velocity, axis,
                        getattr(message.angular_velocity, axis) + rng.gauss(0.0, gyro_sigma))
            if args.vibration_accel_amplitude_mps2 != 0.0:
                elapsed_sec = elapsed_ns / 1_000_000_000.0
                vibration = args.vibration_accel_amplitude_mps2 * math.sin(
                    2.0 * math.pi * args.vibration_frequency_hz * elapsed_sec)
                message.linear_acceleration.x += vibration
            if args.accel_scale_fraction != 1.0:
                for axis in ("x", "y", "z"):
                    setattr(message.linear_acceleration, axis,
                            getattr(message.linear_acceleration, axis) * args.accel_scale_fraction)
            if args.accel_cross_axis_fraction != 0.0:
                message.linear_acceleration.x += (
                    args.accel_cross_axis_fraction * message.linear_acceleration.y)
            if args.gyro_scale_fraction != 1.0:
                for axis in ("x", "y", "z"):
                    setattr(message.angular_velocity, axis,
                            getattr(message.angular_velocity, axis) * args.gyro_scale_fraction)
            if args.gyro_cross_axis_fraction != 0.0:
                message.angular_velocity.x += (
                    args.gyro_cross_axis_fraction * message.angular_velocity.y)
            if accel_sigma > 0.0:
                message.linear_acceleration_covariance = [
                    accel_sigma * accel_sigma, 0.0, 0.0,
                    0.0, accel_sigma * accel_sigma, 0.0,
                    0.0, 0.0, accel_sigma * accel_sigma,
                ]
            if gyro_sigma > 0.0:
                message.angular_velocity_covariance = [
                    gyro_sigma * gyro_sigma, 0.0, 0.0,
                    0.0, gyro_sigma * gyro_sigma, 0.0,
                    0.0, 0.0, gyro_sigma * gyro_sigma,
                ]
            data = serialize_message(message)
            imu_count += 1
        writer.write(topic, data, timestamp_ns)
    writer.close()
    print(f"wrote {args.output}: IMU samples={imu_count}, "
        f"accel_sigma={accel_sigma:.9f}m/s^2, gyro_sigma={gyro_sigma:.9f}rad/s")


if __name__ == "__main__":
    main()
