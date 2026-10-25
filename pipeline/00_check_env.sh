#!/usr/bin/env bash
# Stage 00: check that every tool the pipeline needs is present before a long run.
set -uo pipefail
source "$(dirname "$0")/../env.sh"

fail=0
ok()   { echo "  ok    $*"; }
bad()  { echo "  FAIL  $*"; fail=1; }
warn() { echo "  warn  $*"; }

echo "Tools"
[ -x "$ISAAC_ROOT/python.sh" ] && ok "Isaac Sim $(cat "$ISAAC_ROOT/VERSION")" || bad "Isaac Sim not found at $ISAAC_ROOT"
[ -x "$BLENDER" ] && ok "$("$BLENDER" --version 2>/dev/null | head -1)" || bad "Blender not found at $BLENDER"
command -v uv >/dev/null && ok "uv $(uv --version | cut -d' ' -f2)" || bad "uv not installed"
command -v docker >/dev/null && ok "docker $(docker version --format '{{.Server.Version}}' 2>/dev/null)" || bad "docker not available"

echo "GPU"
if nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader >/dev/null 2>&1; then
  ok "$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader)"
else
  bad "nvidia-smi failed"
fi
docker info 2>/dev/null | grep -q 'nvidia' && ok "docker nvidia runtime registered" || warn "docker nvidia runtime missing (ODM GPU image unavailable; CPU image still works)"

echo "OS"
. /etc/os-release
case "$VERSION_ID" in
  22.04|24.04) ok "$PRETTY_NAME" ;;
  *) warn "$PRETTY_NAME is not a documented Isaac Sim platform; rely on the smoke test below" ;;
esac

echo "Disk"
free_gb=$(df -BG --output=avail "$PROJECT_ROOT" | tail -1 | tr -dc '0-9')
if [ "$free_gb" -lt 20 ]; then bad "${free_gb} GB free (need >= 20 GB)"
elif [ "$free_gb" -lt 60 ]; then warn "${free_gb} GB free (60 GB recommended)"
else ok "${free_gb} GB free"; fi

echo "Isaac headless smoke test"
if [ -x "$ISAAC_ROOT/python.sh" ]; then
  out=$("$ISAAC_ROOT/python.sh" -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
import omni.usd
from pxr import UsdPhysics
print('ISAAC_SMOKE_OK', omni.usd.get_context().get_stage() is not None)
app.close()
" 2>&1)
  echo "$out" | grep -q 'ISAAC_SMOKE_OK True' && ok "SimulationApp starts headless" || { bad "SimulationApp failed"; echo "$out" | grep -iE 'error|fail' | tail -5; }
fi

exit $fail
