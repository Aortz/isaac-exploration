#!/usr/bin/env bash
# Run the whole pipeline. Usage: ./run_all.sh [first_stage]   e.g. ./run_all.sh 03 to skip reconstruction.
set -euo pipefail
source "$(dirname "$0")/env.sh"
cd "$PROJECT_ROOT"

FROM="${1:-00}"
RUN="${RUN:-$(date +%Y%m%d-%H%M%S)}"
RUN_DIR="reports/$RUN"
mkdir -p "$RUN_DIR/logs"
ln -sfn "$RUN" reports/latest
# Blender and Isaac Pythons have no PyYAML; they read this JSON copy of the config.
uv run python -m pipeline.common.config >/dev/null

stage() {  # stage <id> <command...>
  local id="$1"; shift
  [[ "$id" < "$FROM" ]] && return 0
  echo "== stage $id"
  "$@" 2>&1 | tee "$RUN_DIR/logs/$id.log"
}

stage 00 bash pipeline/00_check_env.sh
stage 01 bash pipeline/01_fetch_sample.sh
stage 02 bash pipeline/02_reconstruct.sh
stage 03 uv run python pipeline/03_localize.py
stage 04 "$BLENDER" --background --factory-startup --python-exit-code 1 --python pipeline/04_blender_prep.py
stage 05 "$ISAAC_ROOT/python.sh" pipeline/05_compose_isaac.py
stage 06 "$ISAAC_ROOT/python.sh" pipeline/06_capture.py --run-dir "$RUN_DIR"
stage 07 uv run python pipeline/07_validate.py --run-dir "$RUN_DIR"
echo "done: $RUN_DIR/report.md"
