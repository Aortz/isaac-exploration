#!/usr/bin/env bash
# Stage 02: run OpenDroneMap on the dataset images, then prune heavy intermediates.
set -euo pipefail
source "$(dirname "$0")/../env.sh"

cfg() { uv run --project "$PROJECT_ROOT" python -c "import yaml,sys; c=yaml.safe_load(open('$SITE_CONFIG')); print($1)"; }

IMAGE="$(cfg "c['odm']['image']+'@'+c['odm']['digest']")"
IMAGES="$PROJECT_ROOT/$(cfg "c['dataset']['images']")"
PROJECT="$PROJECT_ROOT/$(cfg "c['odm']['project_dir']")"
mapfile -t ARGS < <(cfg "'\n'.join(c['odm']['args'])")

OUTPUTS=(odm_texturing/odm_textured_model_geo.obj odm_georeferencing/coords.txt odm_dem/dtm.tif odm_dem/dsm.tif)
have_outputs() { for f in "${OUTPUTS[@]}"; do test -f "$PROJECT/code/$f" || return 1; done; }
if [ -z "${FORCE:-}${RERUN_FROM:-}" ] && have_outputs; then
  echo "reconstruction already present (FORCE=1 to rebuild): $PROJECT/code"
  exit 0
fi

# RERUN_FROM=<odm stage> resumes a failed run, e.g. RERUN_FROM=odm_meshing.
EXTRA=()
[ -n "${RERUN_FROM:-}" ] && EXTRA=(--rerun-from "$RERUN_FROM")

mkdir -p "$PROJECT/code"
# ODM expects <project-path>/<name>/images. Bind-mount the images read-only instead of copying.
# Running as the host user keeps outputs user-owned; the working directory must then be writable
# because PoissonRecon writes its temp files into the current directory (the image's /code is root-owned).
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -w /datasets/code \
  -v "$PROJECT:/datasets" \
  -v "$IMAGES:/datasets/code/images:ro" \
  "$IMAGE" \
  --project-path /datasets code \
  "${ARGS[@]}" "${EXTRA[@]}"

# Keep only what later stages read. Undistorted images and depthmaps are several GB.
rm -rf "$PROJECT/code/opensfm/undistorted" "$PROJECT/code/opensfm/features" "$PROJECT/code/opensfm/matches" \
       "$PROJECT/code/odm_meshing/tmp" "$PROJECT/code/odm_filterpoints" 2>/dev/null || true

for f in "${OUTPUTS[@]}"; do
  test -f "$PROJECT/code/$f" || { echo "missing output: $f" >&2; exit 1; }
done
echo "reconstruction OK: $PROJECT/code"
