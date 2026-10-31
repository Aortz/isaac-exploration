#!/usr/bin/env bash
# Inside the container: start PX4 SITL instance <i> (default 0). It connects to the simulator on
# TCP 4560+i and serves the offboard MAVLink link on UDP 14540+i. Parameters are set over MAVLink by
# flight/px4_offboard.py, so nothing here is vehicle-specific beyond the airframe.
set -e
I="${1:-0}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gazebo-classic_iris}"
WORK="/tmp/px4_${I}"
mkdir -p "$WORK"
cd "$WORK"
exec /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4 /opt/PX4-Autopilot/ROMFS/px4fmu_common \
     -s /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i "$I" -w "$WORK" -d
