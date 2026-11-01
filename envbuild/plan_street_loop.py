"""Plan a street-level loop for an OSM environment: fly the road centre lines around a city block.

Builds a graph from the OSM roads wide enough to fly down (envbuild/osm.py widths), takes its cycle
basis (the faces of the street grid = blocks), keeps loops whose length is in range and that pass near
the origin, resamples the best one into waypoints, and checks clearance against the environment's
obstacle boxes (buildings + trees) at the flight altitude. Prints a waypoints line for config/flight.yaml.

Run: uv run --with networkx python envbuild/plan_street_loop.py --env sg_tampines_osm --z 18
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import networkx as nx
import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from envbuild.osm import ROAD_WIDTH, project  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--env", required=True)
ap.add_argument("--z", type=float, default=18.0, help="flight altitude (m above ground)")
ap.add_argument("--min-width", type=float, default=8.0, help="narrowest road to fly down (m)")
ap.add_argument("--length", type=float, nargs=2, default=[450.0, 1100.0], help="loop length range (m)")
ap.add_argument("--spacing", type=float, default=20.0, help="waypoint spacing (m)")
ap.add_argument("--margin", type=float, default=5.0, help="required clearance (m)")
ap.add_argument("--compactness", type=float, default=0.3, help="min 4 pi area / length^2 of the loop (circle = 1)")
args = ap.parse_args()

out = PROJECT_ROOT / "build" / "envs" / args.env
recipe = yaml.safe_load((PROJECT_ROOT / json.loads((out / "env.json").read_text())["recipe"]).read_text())
lat, lon, rad = recipe["lat"], recipe["lon"], recipe.get("radius_m", 400.0)
data = json.loads((out / "osm.json").read_text())
G = nx.Graph()
for el in data["elements"]:
    hw = el.get("tags", {}).get("highway")
    if el["type"] != "way" or ROAD_WIDTH.get(hw, 0) < args.min_width or not el.get("geometry"):
        continue
    xy = project(lat, lon, [q["lat"] for q in el["geometry"]], [q["lon"] for q in el["geometry"]])
    keys = [(round(float(x), 1), round(float(y), 1)) for x, y in xy]
    for a, b in zip(keys[:-1], keys[1:]):
        if a != b and max(map(abs, a + b)) < rad - 30:
            G.add_edge(a, b, w=float(np.hypot(a[0] - b[0], a[1] - b[1])))
# Ways that cross share OSM nodes, so their coordinates coincide; snap near-duplicates too.
cycles = nx.cycle_basis(G)
boxes = np.load(out / "obstacles.npy")
top = boxes[:, 2] + boxes[:, 5] / 2


def resample(poly, step):
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    t = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(t, s, poly[:, 0]), np.interp(t, s, poly[:, 1])]), s[-1]


def chaikin(poly, n):
    """Corner cutting on a closed polyline: rounds the street corners into flyable curves."""
    for _ in range(n):
        q, r = 0.75 * poly + 0.25 * np.roll(poly, -1, 0), 0.25 * poly + 0.75 * np.roll(poly, -1, 0)
        poly = np.stack([q, r], 1).reshape(-1, 2)
    return poly


def clearance(path, z):
    worst = np.inf
    for x, y in path:
        dx = np.maximum(np.abs(boxes[:, 0] - x) - boxes[:, 3] / 2, 0)
        dy = np.maximum(np.abs(boxes[:, 1] - y) - boxes[:, 4] / 2, 0)
        dz = np.maximum(boxes[:, 2] - boxes[:, 5] / 2 - z, 0) + np.maximum(z - top, 0)
        worst = min(worst, float(np.sqrt(dx ** 2 + dy ** 2 + dz ** 2).min()))
    return worst


best = None
for cyc in cycles:
    ring = np.array(cyc, float)
    x, y = ring[:, 0], ring[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))
    ring = chaikin(ring, 5)
    poly = np.vstack([ring, ring[:1]])
    path, L = resample(poly, 2.0)
    # Skip slivers (both carriageways of one road form a thin "loop" with U-turns at the ends).
    if not (args.length[0] <= L <= args.length[1]) or 4 * np.pi * area / L ** 2 < args.compactness:
        continue
    near = float(np.hypot(path[:, 0], path[:, 1]).min())
    c = clearance(path[::3], args.z)
    score = (c >= args.margin, -near)
    if best is None or score > best[0]:
        best = (score, path, L, c, near)
if best is None:
    raise SystemExit(f"no road loop of {args.length} m found; {len(cycles)} cycles in the street graph")
_, path, L, c, near = best
wps, _ = resample(np.vstack([path, path[:1]]), args.spacing)
wps = np.vstack([wps, wps[:1]])
print(f"loop {L:.0f} m, {len(wps)} waypoints, min clearance {c:.1f} m at z={args.z:g}, passes {near:.0f} m from the origin")
print("waypoints: [" + ", ".join(f"[{x:.1f}, {y:.1f}, {args.z:.1f}]" for x, y in wps) + "]")
