"""Replay a recorded rig bag into ROS 2, optionally with a different IMU stream (repeat VIO runs).

Messages go out in recorded order, which is the lockstep order the rig published in (IMU up to
the frame stamp, then /clock, then pose and images), so consumers see the same sequence as live.
With --imu-csv, every /imu/data message is replaced by the row of that CSV with the same stamp
(another noise seed, or imu_clean.csv) - the flight and images stay identical.
Pacing follows /clock: sim time advances at --rate x wall time (OpenVINS with 4 cameras needs < 1).

Runs inside the OpenVINS image (rclpy + rosbag2_py):  ros2/bag/replay.sh <bag_dir> [--imu-csv F] [--rate R]
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
import rosbag2_py
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--imu-csv", help="t,wx,wy,wz,ax,ay,az rows replacing /imu/data (same stamps)")
    ap.add_argument("--rate", type=float, default=0.5, help="sim seconds per wall second")
    ap.add_argument("--start-delay", type=float, default=3.0, help="wall seconds before the first message")
    args = ap.parse_args()

    reader = rosbag2_py.SequentialCompressionReader()   # record.sh compresses per message (zstd)
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    imu = np.loadtxt(args.imu_csv, delimiter=",", comments="#") if args.imu_csv else None

    rclpy.init()
    node = rclpy.create_node("rig_bag_replay")
    pubs = {}
    for topic, typ in types.items():
        if topic == "/tf_static":
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        else:
            qos = QoSProfile(depth=2000 if topic == "/imu/data" else 50, reliability=ReliabilityPolicy.RELIABLE)
        pubs[topic] = node.create_publisher(get_message(typ), topic, qos)
    time.sleep(args.start_delay)          # let subscribers discover the publishers

    imu_i, n, sim0, wall0 = 0, 0, None, None
    clock_type = get_message(types["/clock"]) if "/clock" in types else None
    imu_type = get_message(types["/imu/data"]) if "/imu/data" in types else None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/clock":
            t = deserialize_message(data, clock_type).clock
            sim = t.sec + t.nanosec * 1e-9
            if sim0 is None:
                sim0, wall0 = sim, time.monotonic()
            lag = (sim - sim0) / args.rate - (time.monotonic() - wall0)
            if lag > 0:
                time.sleep(lag)
        if topic == "/imu/data" and imu is not None:
            msg = deserialize_message(data, imu_type)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            while imu_i < len(imu) - 1 and imu[imu_i, 0] < stamp - 1e-6:
                imu_i += 1
            row = imu[imu_i]
            if abs(row[0] - stamp) > 1e-5:
                raise SystemExit(f"IMU CSV has no row at t={stamp:.6f} (nearest {row[0]:.6f})")
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in row[1:4])
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in row[4:7])
            data = serialize_message(msg)
        pubs[topic].publish(data)
        n += 1
    wall = time.monotonic() - wall0 if wall0 else 0.0
    print(f"REPLAY messages={n} sim_s={sim - sim0 if sim0 is not None else 0:.2f} wall_s={wall:.1f}", flush=True)
    time.sleep(2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
