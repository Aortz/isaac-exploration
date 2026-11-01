"""Render-speed benchmark: the real 4-fisheye rig on each environment's loop flight, against the budget.

Runs flight/rig_fisheye.py in lockstep for the first --frames frames (no ROS, no saved images), samples
the GPU's memory once a second, and appends one row per environment to build/envs/bench.csv:
environment, flight, instances, warm-up s, median / 95th-percentile frame ms, peak VRAM, pass/fail.
Run it with nothing else on the GPU: VRAM is read for the whole card.

Run: uv run python envbuild/bench.py [--envs forest_valley_v2 sg_mangrove ...] [--frames 150] [--label before]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ISAAC = Path(os.environ.get("ISAAC_ROOT", Path.home() / "Downloads/isaac-sim-standalone-6.0.1-linux-x86_64"))
BUDGET_MS, BUDGET_GB = 300.0, 9.0

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--envs", nargs="*", help="environment names (default: every env with a P3 flight)")
ap.add_argument("--frames", type=int, default=150)
ap.add_argument("--label", default="", help="free-text tag stored with the rows (e.g. before / after)")
ap.add_argument("--deactivate", nargs="*", default=[], help="experiment: prim paths to switch off, e.g. /World/Scatter/meadow")
ap.add_argument("--no-fog", action="store_true", help="experiment: drop the environment's --/rtx/fog kit args")
ap.add_argument("--kit-arg", action="append", default=[], help="experiment: extra renderer setting, e.g. --kit-arg=--/rtx/raytracing/subsurface/enabled=false")
args = ap.parse_args()
TMP = PROJECT_ROOT / "build" / "envs" / "_bench"
TMP.mkdir(parents=True, exist_ok=True)

cfg = yaml.safe_load((PROJECT_ROOT / "config" / "flight.yaml").read_text())
flights = {f["env"]: name for name, f in cfg["flights"].items()
           if "env" in f and (name.endswith("_p3_loop") or name.endswith("_p3_street"))}
envs = args.envs or sorted(flights)


def vram_mb() -> int:
    out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout
    return int(out.split()[0]) if out.strip() else 0


rows = []
for env in envs:
    if env not in flights:
        print(f"skip {env}: no P3 flight")
        continue
    flight = flights[env]
    run_cfg = PROJECT_ROOT / "config" / "flight.yaml"
    if args.deactivate or args.no_fog or args.kit_arg:
        # Experiment variant: a layer over the environment that switches prims off, in a copy of the config.
        e = cfg["environments"][env]
        src = (PROJECT_ROOT / e["usd"]).resolve()
        lines = ["#usda 1.0", f"(\n    subLayers = [@{src}@]\n    defaultPrim = \"World\"\n    upAxis = \"Z\"\n    metersPerUnit = 1\n)"]
        tree: dict = {}
        for path in args.deactivate:                        # merge the paths: one spec per prim
            node = tree
            for n in path.strip("/").split("/"):
                node = node.setdefault(n, {})
            node[None] = True

        def emit(node: dict, depth: int) -> str:
            out = ""
            for n, sub in node.items():
                if n is None:
                    continue
                pad = "    " * depth
                out += f'{pad}over "{n}"' + (" (active = false)" if sub.get(None) else "") + " {\n"
                out += emit(sub, depth + 1) + f"{pad}}}\n"
            return out

        lines.append(emit(tree, 0))
        variant = TMP / f"{env}_variant.usda"
        variant.write_text("\n".join(lines) + "\n")
        c2 = json.loads(json.dumps(cfg))
        c2["environments"][env]["usd"] = str(variant)
        if args.no_fog:
            c2["environments"][env]["kit_args"] = [a for a in e.get("kit_args", []) if "/rtx/fog" not in a]
        c2["environments"][env]["kit_args"] = c2["environments"][env].get("kit_args", []) + args.kit_arg
        run_cfg = TMP / "flight.yaml"
        run_cfg.write_text(yaml.safe_dump(c2, sort_keys=False))
    peak, stop = [vram_mb()], threading.Event()

    def sample():
        while not stop.is_set():
            peak[0] = max(peak[0], vram_mb())
            time.sleep(1.0)

    th = threading.Thread(target=sample, daemon=True)
    th.start()
    t0 = time.time()
    log = PROJECT_ROOT / "build" / "envs" / env / "bench.log"
    with open(log, "w") as fh:
        proc = subprocess.run([str(ISAAC / "python.sh"), "flight/rig_fisheye.py", "--flight", flight, "--config", str(run_cfg), "--mode", "lockstep",
                               "--max-frames", str(args.frames), "--save-every", "0", "--tag", "bench"],
                              cwd=PROJECT_ROOT, stdout=fh, stderr=subprocess.STDOUT, timeout=3 * 3600)
    stop.set()
    th.join()
    rates = next((json.loads(ln[len("RIG_RATES "):]) for ln in log.read_text(errors="ignore").splitlines()
                  if ln.startswith("RIG_RATES ")), None)
    man = PROJECT_ROOT / "build" / "envs" / env / "env.json"
    inst = sum(json.loads(man.read_text()).get("instances", {}).values()) if man.exists() else ""
    if rates is None:
        row = {"env": env, "flight": flight, "instances": inst, "ok": False, "note": f"rig exit {proc.returncode}"}
    else:
        fr = rates["timing"]["frame"]
        row = {"env": env, "flight": flight, "instances": inst, "warmup_s": rates["warmup_s"],
               "frame_p50_ms": fr["p50_ms"], "frame_p95_ms": fr["p95_ms"], "peak_vram_gb": round(peak[0] / 1024, 2),
               "ok": fr["p50_ms"] <= BUDGET_MS and peak[0] / 1024 <= BUDGET_GB, "note": ""}
    row.update({"label": args.label + (f" off={args.deactivate}" if args.deactivate else "") + (" nofog" if args.no_fog else "") + "".join(" " + k.split("/rtx/")[-1] for k in args.kit_arg), "frames": args.frames, "wall_s": round(time.time() - t0, 1),
                "date": time.strftime("%Y-%m-%d %H:%M")})
    rows.append(row)
    print("BENCH " + json.dumps(row), flush=True)

out = PROJECT_ROOT / "build" / "envs" / "bench.csv"
fields = ["date", "label", "env", "flight", "instances", "frames", "warmup_s", "frame_p50_ms", "frame_p95_ms",
          "peak_vram_gb", "ok", "wall_s", "note"]
new = not out.exists()
with open(out, "a", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
    if new:
        w.writeheader()
    w.writerows(rows)
print(f"BENCH -> {out}  (budget: p50 <= {BUDGET_MS:g} ms, VRAM <= {BUDGET_GB:g} GB)")
