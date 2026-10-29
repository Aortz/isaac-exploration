"""Subscribe to the fisheye rig topics and report rates, stamp alignment and gaps.

Run (second terminal, while flight/rig_fisheye.py --ros2 is publishing):
  source ros2/ros_env.sh && $ISAAC_ROOT/python.sh ros2/measure_rates.py --seconds 30

Wall rate = messages per second of wall time (what a live consumer sees).
Sim rate  = messages per second of header-stamp time (what a use_sim_time consumer sees; in
            lockstep this is exactly the configured rate however slow the renderer is).
Sync      = camera frames whose stamp arrived on all cameras, and the stamp spread across cameras.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--seconds", type=float, default=30.0, help="stop after this much wall time (or on idle)")
ap.add_argument("--idle", type=float, default=5.0, help="stop when nothing arrives for this long")
ap.add_argument("--cameras", nargs="*", default=["cam0_front", "cam1_left", "cam2_back", "cam3_right"])
ap.add_argument("--out", help="write the report as JSON here")
args = ap.parse_args()

rclpy.init()
node = rclpy.create_node("rig_rate_probe")
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
arrivals = defaultdict(list)   # topic -> [(wall, stamp)]


def stamp_of(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def sub(topic, typ, stamp=stamp_of):
    node.create_subscription(typ, topic, lambda m: arrivals[topic].append((time.perf_counter(), stamp(m))), qos)


for c in args.cameras:
    sub(f"/{c}/image_raw", Image)
sub("/imu/data", Imu)
sub("/clock", Clock, lambda m: m.clock.sec + m.clock.nanosec * 1e-9)

t0 = time.perf_counter()
last = t0
seen = 0
while time.perf_counter() - t0 < args.seconds:
    rclpy.spin_once(node, timeout_sec=0.1)
    n = sum(len(v) for v in arrivals.values())
    if n != seen:
        seen, last = n, time.perf_counter()
    elif seen and time.perf_counter() - last > args.idle:
        break


def rate(v):
    if len(v) < 2:
        return {"count": len(v)}
    w = [a for a, _ in v]
    s = [b for _, b in v]
    ds = [b - a for a, b in zip(s, s[1:])]
    return {"count": len(v), "wall_hz": round((len(v) - 1) / (w[-1] - w[0]), 2),
            "sim_hz": round((len(v) - 1) / (s[-1] - s[0]), 2) if s[-1] > s[0] else None,
            "stamp_step_min_ms": round(1000 * min(ds), 3), "stamp_step_max_ms": round(1000 * max(ds), 3),
            "non_monotonic": sum(d <= 0 for d in ds)}


report = {"topics": {k: rate(v) for k, v in sorted(arrivals.items())}}
cams = [f"/{c}/image_raw" for c in args.cameras]
stamps = [{round(s, 6) for _, s in arrivals[c]} for c in cams]
if all(stamps):
    common = set.intersection(*stamps)
    union = set.union(*stamps)
    # arrival spread: wall time between the first and last camera image of the same stamp
    first = {c: {round(s, 6): w for w, s in arrivals[c]} for c in cams}
    spread = [max(first[c][s] for c in cams) - min(first[c][s] for c in cams) for s in common]
    report["sync"] = {"frames_on_all_cameras": len(common), "frames_on_some_cameras": len(union),
                      "stamp_skew_s": 0.0 if common else None,
                      "arrival_spread_ms_p50": round(1000 * sorted(spread)[len(spread) // 2], 3) if spread else None,
                      "arrival_spread_ms_max": round(1000 * max(spread), 3) if spread else None}
print(json.dumps(report, indent=2))
if args.out:
    open(args.out, "w").write(json.dumps(report, indent=2) + "\n")
node.destroy_node()
rclpy.shutdown()
