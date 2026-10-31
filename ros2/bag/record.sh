#!/usr/bin/env bash
# Record the rig's ROS 2 topics to an MCAP bag (zstd per message), in the OpenVINS image (has rosbag2).
# Usage: ros2/bag/record.sh <bag_dir> [extra topics...]   (bag_dir must not exist yet)
# Start before flight/rig_fisheye.py --ros2; stop with: docker kill -s INT rosbag
set -euo pipefail
BAG="$(realpath -m "${1:?bag dir}")"
shift
mkdir -p "$(dirname "$BAG")"
TOPICS=(/clock /imu/data /tf /tf_static /ground_truth/odom
        /cam0_front/image_raw /cam1_left/image_raw /cam2_back/image_raw /cam3_right/image_raw
        /cam0_front/camera_info /cam1_left/camera_info /cam2_back/camera_info /cam3_right/camera_info "$@")
docker run --rm --init --name rosbag --net=host --ipc=host --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_log -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "$(dirname "$BAG")":/bags \
    isaac-exploration/openvins:jazzy \
    bash -c ". /opt/ros/jazzy/setup.bash && exec ros2 bag record -s mcap --compression-mode message \
             --compression-format zstd -o /bags/$(basename "$BAG") ${TOPICS[*]}"
