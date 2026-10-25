#!/usr/bin/env bash
# Launch the Isaac Sim GUI with Cesium for Omniverse loaded at startup.
# Cesium must be enabled at launch (not later from the Extensions window) so its USD schemas register.
set -euo pipefail
source "$(dirname "$0")/../env.sh"
CESIUM_EXTS="$PROJECT_ROOT/third_party/cesium/v0.29.0"
[ -d "$CESIUM_EXTS/cesium.omniverse" ] || { echo "Cesium not found in $CESIUM_EXTS; run cesium/install_cesium.sh" >&2; exit 1; }
exec "$ISAAC_ROOT/isaac-sim.sh" --ext-folder "$CESIUM_EXTS" --enable cesium.omniverse "$@"
