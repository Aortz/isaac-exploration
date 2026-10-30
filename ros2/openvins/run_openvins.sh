#!/usr/bin/env bash
# Run OpenVINS (docker, ROS 2 Jazzy) live against the Isaac rig topics.
# Usage: ros2/openvins/run_openvins.sh <config_name> <out_dir>      e.g. rig4 build/openvins/rig4/run_loop
# Start this first, then flight/rig_fisheye.py --ros2 --mode lockstep. OpenVINS uses sim time (/clock).
# Stop with: docker kill -s INT openvins   (SIGINT lets it flush ov_estimate.txt)
set -euo pipefail
source "$(dirname "$0")/../../env.sh"
CFG="$PROJECT_ROOT/build/openvins/${1:?config name, see make_config.py}"
OUT="$(realpath -m "${2:?output dir}")"
mkdir -p "$OUT"
# Same uid + host IPC/network so Fast DDS shared memory and discovery work with the host process.
docker run --rm --init --name openvins --net=host --ipc=host --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_log -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "$CFG":/cfg:ro -v "$OUT":/out \
    isaac-exploration/openvins:jazzy \
    bash -c ". /opt/ros/jazzy/setup.bash && . /ws/install/setup.bash && \
             exec /ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf --ros-args -p config_path:=/cfg/estimator_config.yaml \
             -p use_sim_time:=true -p save_total_state:=true -p filepath_est:=/out/ov_estimate.txt \
             -p filepath_std:=/out/ov_estimate_std.txt -p filepath_gt:=/out/ov_groundtruth.txt" \
    2>&1 | tee "$OUT/openvins.log"
