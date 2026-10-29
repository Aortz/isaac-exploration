"""Measure how many frames a free-running rig lags behind the pose it is stamped with.

Compares each frame of a freerun --save-all run with frames k-3..k+1 of a lockstep --save-all run
(the reference: every lockstep frame is rendered at its own pose). The best-matching offset per
frame is the lag; 0 means the image belongs to the stamped pose.

Run: uv run python flight/rig_lag_check.py --flight marina_bay_pass --freerun rig_freerun_lag --lockstep rig_lockstep_lag
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD  # noqa: E402


def load(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert("L").resize((320, 240)), np.float32)


def ncc(a: np.ndarray, b: np.ndarray) -> float:
    a, b = a - a.mean(), b - b.mean()
    return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum() + 1e-9))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--freerun", default="rig_freerun_lag")
    ap.add_argument("--lockstep", default="rig_lockstep_lag")
    ap.add_argument("--camera", default="cam0_front")
    ap.add_argument("--skip", type=int, default=30, help="ignore the first N frames (near-hover, ambiguous)")
    args = ap.parse_args()
    fdir = BUILD / "flight" / args.flight
    free = sorted((fdir / args.freerun / "frames").glob(f"*_{args.camera}.png"))
    ref = {int(p.name[:6]): p for p in (fdir / args.lockstep / "frames").glob(f"*_{args.camera}.png")}
    cache: dict[int, np.ndarray] = {}
    lags = []
    for p in free:
        k = int(p.name[:6])
        if k < args.skip:
            continue
        img = load(p)
        scores = {}
        for off in range(-3, 2):          # positive lag = the image shows an earlier pose
            j = k - off
            if j in ref:
                cache.setdefault(j, load(ref[j]))
                scores[off] = ncc(img, cache[j])
        if scores:
            lags.append(max(scores, key=scores.get))
    hist = Counter(lags)
    report = {"frames": len(lags), "lag_histogram": {str(k): hist[k] for k in sorted(hist)},
              "lag_0_fraction": round(hist.get(0, 0) / max(len(lags), 1), 3)}
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
