# Source this to use the ROS 2 Jazzy libraries bundled with Isaac Sim (no system ROS install needed).
# Works for Isaac's python.sh (rclpy) in this repo's ros2/ scripts.
source "$(dirname "${BASH_SOURCE[0]}")/../env.sh"
_ROS_INTERNAL="$ISAAC_ROOT/exts/isaacsim.ros2.core/jazzy"
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$_ROS_INTERNAL/lib"
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$_ROS_INTERNAL/rclpy"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
