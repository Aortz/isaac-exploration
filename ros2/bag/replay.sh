#!/usr/bin/env bash
# Replay a rig bag (see replay.py). Usage: ros2/bag/replay.sh <bag_dir> [--imu-csv <csv>] [--rate 0.5]
# Paths are mounted read-only into the container under the same absolute path.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$(dirname "$HERE")")"
BAG="$(realpath "${1:?bag dir}")"
shift
docker run --rm --init --name rosbag_replay --net=host --ipc=host --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_log -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "$REPO":"$REPO":ro \
    isaac-exploration/openvins:jazzy \
    bash -c ". /opt/ros/jazzy/setup.bash && exec python3 $HERE/replay.py $BAG $*"
