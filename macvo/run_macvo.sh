#!/usr/bin/env bash
# Run one headless MAC-VO instance on an isaac-exploration stereo sequence.
#
#   macvo/run_macvo.sh <seq_dir> <out_dir> [Fast|Performant] [extra run_macvo.py args...]
#
# e.g. MACVO_THREADS=2 macvo/run_macvo.sh build/stereo/seq01 build/macvo/seq01 Fast --no-mapping
#
# Env:
#   MACVO_RUNNER   docker (default) | local   (local = python from MACVO_PYTHON, e.g. a venv)
#   MACVO_IMAGE    default isaac-exploration/macvo:latest
#   MACVO_NAME     container name (default: macvo-<basename out_dir>-<pid>)
#   MACVO_GPU      GPU selector for docker --gpus (default: all)
#   MACVO_THREADS  cap on torch CPU threads per process (default: MAC-VO's own hardcoded
#                  4 in the main process + 8 in the spawned optimizer process). Set e.g. 2
#                  when running many instances.
#   MACVO_CACHE    host dir for torch.compile/triton caches shared by all instances
#                  (default: macvo/.cache) -- avoids recompiling in every new container
#   MACVO_PYTHON   interpreter for MACVO_RUNNER=local (default: python3)
set -euo pipefail

if [[ $# -lt 2 ]]; then
  sed -n '2,20p' "$0"; exit 2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
MACVO_ROOT="${MACVO_ROOT:-$REPO/third_party/macvo}"

SEQ="$(realpath "$1")"
mkdir -p "$2"
OUT="$(realpath "$2")"
MODE="${3:-Fast}"
shift $(( $# >= 3 ? 3 : 2 ))

[[ -f "$SEQ/stereo.json" ]] || { echo "no stereo.json in $SEQ" >&2; exit 2; }
[[ -f "$MACVO_ROOT/Model/MACVO_FrontendCov.pth" ]] || { echo "missing weights in $MACVO_ROOT/Model" >&2; exit 2; }

# expandable_segments cut peak per-instance VRAM from ~3.8 to ~2.65 GiB in the smoke test.
ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"

RUNNER="${MACVO_RUNNER:-docker}"
if [[ "$RUNNER" == "local" ]]; then
  exec env MACVO_ROOT="$MACVO_ROOT" PYTHONDONTWRITEBYTECODE=1 PYTORCH_CUDA_ALLOC_CONF="$ALLOC_CONF" \
    "${MACVO_PYTHON:-python3}" "$HERE/run_macvo.py" --seq "$SEQ" --out "$OUT" --mode "$MODE" "$@"
fi

IMAGE="${MACVO_IMAGE:-isaac-exploration/macvo:latest}"
CACHE="${MACVO_CACHE:-$HERE/.cache}"
mkdir -p "$CACHE"
NAME="${MACVO_NAME:-macvo-$(basename "$OUT")-$$}"

# Source trees are mounted read-only, so N concurrent containers share them safely.
# --shm-size: MAC-VO's optimizer runs in a spawned child process and exchanges tensors via /dev/shm.
exec docker run --rm --init --name "$NAME" \
  --gpus "${MACVO_GPU:-all}" \
  --shm-size=2g \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e USER=macvo -e LOGNAME=macvo -e MPLCONFIGDIR=/tmp/mpl -e MACVO_ROOT=/macvo \
  -e TORCHINDUCTOR_CACHE_DIR=/cache/inductor -e TRITON_CACHE_DIR=/cache/triton \
  -e MACVO_SEQ_NAME="$(basename "$SEQ")" \
  -e MACVO_THREADS="${MACVO_THREADS:-0}" \
  ${MACVO_THREADS:+-e OMP_NUM_THREADS="$MACVO_THREADS"} \
  -e PYTORCH_CUDA_ALLOC_CONF="$ALLOC_CONF" \
  -v "$MACVO_ROOT":/macvo:ro \
  -v "$HERE":/isaac_macvo:ro \
  -v "$SEQ":/data/seq:ro \
  -v "$OUT":/data/out \
  -v "$CACHE":/cache \
  "$IMAGE" \
  python /isaac_macvo/run_macvo.py --seq /data/seq --out /data/out --mode "$MODE" "$@"
