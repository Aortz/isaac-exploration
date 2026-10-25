#!/usr/bin/env bash
# Drone flight through Cesium -> Cosmos 3 variations -> QA -> EuRoC export.
# Usage: cosmos/run_cosmos.sh <flight> [first_stage]      stages: f0 f1 f2 f3 f4 f5 f6
#   f0 Cesium stage, f1 trajectory, f2 render, f3 package, f4 Cosmos, f5 QA, f6 export
# Env: CESIUM_ION_TOKEN (google/osm tiles), NVIDIA_API_KEY (hosted Cosmos), COSMOS_BACKEND (override)
set -euo pipefail
source "$(dirname "$0")/../env.sh"
cd "$PROJECT_ROOT"

FLIGHT="${1:?usage: cosmos/run_cosmos.sh <flight> [first_stage]}"
FROM="${2:-f0}"
CONFIG="$PROJECT_ROOT/config/flight.yaml"
LOGS="build/cosmos/$FLIGHT/logs"
mkdir -p "$LOGS"
uv run python -m pipeline.common.config "$CONFIG" >/dev/null   # JSON copy for Isaac's Python

flight_field() { uv run python -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['flights']['$FLIGHT']['$1'])"; }

stage() {  # stage <id> <command...>
  local id="$1"; shift
  [[ "$id" < "$FROM" ]] && return 0
  echo "== $id"
  "$@" 2>&1 | tee "$LOGS/$id.log"
}

SITE="$(flight_field site)"
SOURCE="$(flight_field source)"
stage f0 "$ISAAC_ROOT/python.sh" cesium/build_cesium_stage.py --site "$SITE" --source "$SOURCE"
stage f1 uv run python flight/f1_trajectory.py --flight "$FLIGHT"
stage f2 "$ISAAC_ROOT/python.sh" flight/f2_render.py --flight "$FLIGHT"
stage f3 uv run python cosmos/f3_package.py --flight "$FLIGHT"
stage f4 uv run python cosmos/f4_generate.py --flight "$FLIGHT" ${COSMOS_BACKEND:+--backend "$COSMOS_BACKEND"}
stage f5 uv run python cosmos/f5_qa.py --flight "$FLIGHT"
stage f6 uv run python cosmos/f6_export.py --flight "$FLIGHT" ${COSMOS_BACKEND:+$([ "$COSMOS_BACKEND" = dry_run ] && echo --include-synthetic)}
echo "done: build/cosmos/$FLIGHT/qa/report.md, build/vio_datasets/$FLIGHT/"
