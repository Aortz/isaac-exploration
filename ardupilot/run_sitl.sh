#!/usr/bin/env bash
# Inside the container: ArduCopter SITL instance <i> on the JSON physics interface (UDP 9002+10*i),
# MAVLink on TCP 5760+10*i. Usage: run_sitl.sh <i> <home lat,lon,alt,yaw> [param files (container paths)...]
set -e
I="${1:-0}"; HOME_LOC="${2:--35.363261,149.165230,584,0}"; shift 2 || true
DEFAULTS=$(IFS=,; echo "$*")
mkdir -p "/tmp/sitl_$I" && cd "/tmp/sitl_$I"
exec /opt/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I"$I" --home "$HOME_LOC" \
     ${DEFAULTS:+--defaults "$DEFAULTS"}
