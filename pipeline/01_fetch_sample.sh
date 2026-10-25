#!/usr/bin/env bash
# Stage 01: fetch the ODM Aukerman sample images (77 nadir photos with GPS EXIF).
set -euo pipefail
source "$(dirname "$0")/../env.sh"

DEST="$PROJECT_ROOT/datasets/aukerman"
if [ -d "$DEST/images" ] && [ "$(ls "$DEST/images" | wc -l)" -gt 0 ]; then
  echo "already present: $DEST/images ($(ls "$DEST/images" | wc -l) images)"
  exit 0
fi

TMP="$PROJECT_ROOT/datasets/_aukerman_src"
rm -rf "$TMP"
git clone --depth 1 https://github.com/OpenDroneMap/odm_data_aukerman "$TMP"
mkdir -p "$DEST"
mv "$TMP/images" "$DEST/images"
cp "$TMP/README.md" "$TMP/license.txt" "$DEST/"
rm -rf "$TMP"
echo "fetched $(ls "$DEST/images" | wc -l) images into $DEST/images"
