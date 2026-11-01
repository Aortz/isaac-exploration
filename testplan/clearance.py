"""Clearance between a flight and the environment's geometry (test-plan gate G3, and for planning).

Uses the per-object world bounds from flight/env_probe.py (build/envs/<env>/meshes.npy). Each object is an
axis-aligned box, which over-estimates rotated objects, so the reported clearance is conservative.
Ground planes and other objects larger than 200 m are skipped; flat ground decals too.

Run: uv run python testplan/clearance.py --flight rm_p6_close [--source reference|flown] [--min-m 5]
Prints the minimum clearance, where it happens, and exits 1 if it is below --min-m.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402


def load_boxes(env: str) -> tuple[np.ndarray, np.ndarray]:
    """Object boxes: env_probe's meshes.npy plus envbuild's obstacles.npy (scattered instances), if present."""
    parts = [np.load(f) for f in (BUILD / "envs" / env / "meshes.npy", BUILD / "envs" / env / "obstacles.npy")
             if f.exists()]
    m = np.vstack(parts)
    c, size = m[:, :3], m[:, 3:]
    keep = (size[:, 0] < 200) & (size[:, 1] < 200) & ~((size[:, 2] < 0.5) & ((size[:, 0] > 20) | (size[:, 1] > 20)))
    return c[keep] - size[keep] / 2, c[keep] + size[keep] / 2


def clearance(points: np.ndarray, lo: np.ndarray, hi: np.ndarray, chunk: int = 256) -> tuple[np.ndarray, np.ndarray]:
    """Distance from each point to the nearest box (0 inside), and that box's index."""
    dist, idx = np.empty(len(points)), np.empty(len(points), int)
    for i in range(0, len(points), chunk):
        p = points[i:i + chunk, None, :]
        d = np.linalg.norm(np.maximum(np.maximum(lo[None] - p, p - hi[None]), 0.0), axis=2)
        idx[i:i + chunk] = d.argmin(1)
        dist[i:i + chunk] = d[np.arange(len(d)), idx[i:i + chunk]]
    return dist, idx


def occupancy_clearance(points: np.ndarray, occ_yaml: Path) -> np.ndarray:
    """2-D distance (m) from each point to the nearest occupied cell of a ROS occupancy map (indoor scenes)."""
    import yaml
    from PIL import Image
    from scipy import ndimage

    meta = yaml.safe_load(occ_yaml.read_text())
    img = np.asarray(Image.open(occ_yaml.parent / meta["image"]).convert("L"), float) / 255.0
    occ = (1.0 - img) if not meta.get("negate", 0) else img          # occupancy probability
    free = occ < meta["free_thresh"]
    dist = ndimage.distance_transform_edt(free) * meta["resolution"]
    h = img.shape[0]
    col = ((points[:, 0] - meta["origin"][0]) / meta["resolution"]).astype(int)
    row = (h - 1 - (points[:, 1] - meta["origin"][1]) / meta["resolution"]).astype(int)   # row 0 = max y
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < img.shape[1])
    return np.where(inside, dist[np.clip(row, 0, h - 1), np.clip(col, 0, img.shape[1] - 1)], 0.0)


def flight_clearance(flight: str, source: str = "reference", hz: float = 10.0, config: str | None = None) -> dict:
    cfg = load_config(config or PROJECT_ROOT / "config" / "flight.yaml")
    env = cfg["flights"][flight]["env"]
    env_cfg = cfg["environments"][env]
    fdir = BUILD / "flight" / flight
    if source == "reference":
        traj = np.loadtxt(fdir / "reference_1000hz.csv", delimiter=",", comments="#")[:, :4]
    else:
        traj = np.loadtxt(fdir / "groundtruth_200hz.csv", delimiter=",", comments="#")[:, :4]
    traj = traj[np.searchsorted(traj[:, 0], np.arange(traj[0, 0], traj[-1, 0], 1.0 / hz))]
    if env_cfg.get("occupancy"):
        d = occupancy_clearance(traj[:, 1:4], PROJECT_ROOT / env_cfg["occupancy"])
        k = int(d.argmin())
        return {"flight": flight, "source": source, "method": "occupancy_2d", "min_clearance_m": round(float(d[k]), 2),
                "at_t_s": round(float(traj[k, 0]), 2), "at_xyz": [round(float(v), 2) for v in traj[k, 1:4]],
                "p5_clearance_m": round(float(np.percentile(d, 5)), 2)}
    lo, hi = load_boxes(env)
    d, i = clearance(traj[:, 1:4], lo, hi)
    agl = None
    hm = BUILD / "envs" / env / "heightmap.npz"
    if hm.exists():                       # envbuild terrain: height above the ground counts as clearance too
        h = np.load(hm)
        from envbuild.procedural_math import height_at
        agl = traj[:, 3] - height_at(h["heights"], float(h["x0"]), float(h["y0"]), float(h["res"]), traj[:, 1], traj[:, 2])
        d = np.minimum(d, agl)
    k = int(d.argmin())
    return {"flight": flight, "source": source, "method": "aabb" + ("+agl" if agl is not None else ""),
            "min_agl_m": round(float(agl.min()), 2) if agl is not None else None,
            "min_clearance_m": round(float(d[k]), 2),
            "at_t_s": round(float(traj[k, 0]), 2), "at_xyz": [round(float(v), 1) for v in traj[k, 1:4]],
            "nearest_box_lo": [round(float(v), 1) for v in lo[i[k]]], "nearest_box_hi": [round(float(v), 1) for v in hi[i[k]]],
            "p5_clearance_m": round(float(np.percentile(d, 5)), 2)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--source", choices=["reference", "flown"], default="reference")
    ap.add_argument("--min-m", type=float, default=5.0)
    args = ap.parse_args()
    r = flight_clearance(args.flight, args.source)
    print(json.dumps(r))
    sys.exit(0 if r["min_clearance_m"] >= args.min_m else 1)


if __name__ == "__main__":
    main()
