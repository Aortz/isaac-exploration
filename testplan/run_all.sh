#!/usr/bin/env bash
# Run every test case in config/tests.yaml in order (or the ones given), waiting for each case's Pegasus
# flight to exist first, then rebuild the report. Logs: build/tests/logs/<case>.log
# Usage: testplan/run_all.sh [CASE_ID ...]
cd "$(dirname "$0")/.."
mkdir -p build/tests/logs
if [ $# -gt 0 ]; then CASES=("$@"); else
  mapfile -t CASES < <(uv run python -c "import yaml; [print(c['id']) for c in yaml.safe_load(open('config/tests.yaml'))['cases']]")
fi
for c in "${CASES[@]}"; do
  f=$(uv run python -c "import yaml,sys; print(next(x['flight'] for x in yaml.safe_load(open('config/tests.yaml'))['cases'] if x['id']=='$c'))")
  echo "[$(date +%T)] $c (flight $f): waiting for the Pegasus flight"
  until grep -q '"dynamics": "pegasus"' "build/flight/$f/trajectory.json" 2>/dev/null; do sleep 30; done
  echo "[$(date +%T)] $c: running"
  uv run python testplan/run_case.py --case "$c" > "build/tests/logs/$c.log" 2>&1
  echo "[$(date +%T)] $c: exit $? | $(grep -E 'VIO seed|MAC-VO|G3|G1|stereo' build/tests/logs/$c.log | tr '\n' ' ' | cut -c1-600)"
  uv run python testplan/report.py > /dev/null 2>&1
done
echo "[$(date +%T)] all done"
