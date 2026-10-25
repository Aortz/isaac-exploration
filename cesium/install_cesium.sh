#!/usr/bin/env bash
# Download Cesium for Omniverse v0.29.0 (the release built for Kit 110 / Isaac Sim 6.0.1).
set -euo pipefail
source "$(dirname "$0")/../env.sh"
DEST="$PROJECT_ROOT/third_party/cesium/v0.29.0"
if [ -d "$DEST/cesium.omniverse" ]; then echo "already installed: $DEST"; exit 0; fi
mkdir -p "$DEST"
ZIP="$(mktemp --suffix=.zip)"
curl -fsSL -o "$ZIP" https://github.com/CesiumGS/cesium-omniverse/releases/download/v0.29.0/CesiumGS-cesium-omniverse-linux-x86_64-v0.29.0.zip
unzip -q "$ZIP" -d "$DEST"
rm -f "$ZIP"
echo "installed: $DEST"
