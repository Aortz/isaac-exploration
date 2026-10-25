"""F4: run the Cosmos variation matrix (clip × condition × seed) through the configured backend.

Outputs are cached by content hash in build/cosmos/cache/, so re-running (or resuming after a
failure) never re-generates or re-bills a finished request. Each variant gets a symlink under
build/cosmos/<flight>/variants/<clip>/<condition>_s<seed>.mp4 and an entry in manifest.json.

Run:
  NVIDIA_API_KEY=... uv run python cosmos/f4_generate.py --flight marina_bay_pass
  uv run python cosmos/f4_generate.py --flight sample_orbit --backend dry_run      # pipeline test
  ... --conditions clear_noon night_city --seeds 1 --frames 49                       # small M2 probe
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cosmos.backends import TransferJob, make_backend  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402

CACHE = BUILD / "cosmos" / "cache"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    ap.add_argument("--backend", help="override cosmos.backend")
    ap.add_argument("--conditions", nargs="*", help="subset of cosmos.conditions")
    ap.add_argument("--seeds", nargs="*", type=int, help="override cosmos.seeds")
    ap.add_argument("--clips", nargs="*", help="subset of clip names, e.g. clip_00")
    ap.add_argument("--frames", type=int, help="generate only the first N frames of each clip (API probe)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    ccfg = dict(cfg["cosmos"])
    if args.backend:
        ccfg["backend"] = args.backend
    backend = make_backend(ccfg)
    conditions = args.conditions or list(ccfg["conditions"])
    seeds = args.seeds or ccfg["seeds"]

    root = BUILD / "cosmos" / args.flight
    clips = sorted(p for p in (root / "clips").iterdir() if p.is_dir())
    if args.clips:
        clips = [c for c in clips if c.name in args.clips]
    CACHE.mkdir(parents=True, exist_ok=True)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"variants": {}}

    todo = [(c, cond, s) for c in clips for cond in conditions for s in seeds]
    print(f"{len(todo)} generations via {backend.name}")
    failures = 0
    for clip_dir, cond, seed in todo:
        clip = json.loads((clip_dir / "clip.json").read_text())
        n = min(args.frames or clip["num_frames"], clip["num_frames"])
        job = TransferJob(clip_dir=clip_dir, condition=cond, seed=seed, num_frames=n, fps=clip["fps"],
                          controls=list(ccfg["controls"]),
                          prompt=json.loads((clip_dir / "prompts" / f"{cond}.json").read_text()))
        key = backend.cache_key(job)
        cached = CACHE / f"{key}.mp4"
        name = f"{clip_dir.name}/{cond}_s{seed}" + (f"_f{n}" if args.frames else "")
        link = root / "variants" / f"{name}.mp4"
        link.parent.mkdir(parents=True, exist_ok=True)
        if not cached.exists():
            print(f"  generating {name} ...", flush=True)
            tmp = cached.with_suffix(".part.mp4")
            try:
                meta = backend.transfer(job, tmp)
            except Exception as exc:
                failures += 1
                manifest["variants"][name] = {"status": "failed", "error": str(exc)[:2000], "backend": backend.name,
                                              "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
                traceback.print_exc()
                manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
                continue
            tmp.rename(cached)
            (CACHE / f"{key}.json").write_text(json.dumps({**meta, "backend": backend.name}, indent=2) + "\n")
        meta = json.loads((CACHE / f"{key}.json").read_text())
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(os.path.relpath(cached, link.parent))
        manifest["variants"][name] = {"status": "ok", "clip": clip_dir.name, "condition": cond, "seed": seed,
                                      "frames": n, "cache_key": key, "path": str(link.relative_to(root)), **meta}
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"  {name}: ok ({meta.get('seconds')} s{', synthetic' if meta.get('synthetic') else ''})")
    print(f"done: {len(todo) - failures} ok, {failures} failed -> {manifest_path}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
