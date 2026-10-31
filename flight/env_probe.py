"""Size up an Isaac environment before planning flights over it: bounds, mesh density, heights, colliders.

Opens the stage through Kit (so asset-root URLs resolve), collects the world bounds of every Gprim
(instance proxies included), and writes a JSON summary plus a top-down mesh-density map.

Run: $ISAAC_ROOT/python.sh flight/env_probe.py --usd /Isaac/Environments/Outdoor/Rivermark/rivermark.usd --name rivermark
Writes build/envs/<name>/{probe.json, density.png, meshes.npy[, topdown.png]}
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--usd", help="path under the Isaac assets root (starts with /Isaac) or a full URL/path")
ap.add_argument("--name", help="output folder name (default: --env)")
ap.add_argument("--env", help="take usd + kit_args from config/flight.yaml environments.<env> instead")
ap.add_argument("--topdown", type=float, nargs=4, metavar=("X0", "Y0", "X1", "Y1"),
                help="also render an orthographic top view of this x/y box (m), with a metric grid")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

from pipeline.common.config import load_config as _load  # noqa: E402

_env = _load(PROJECT_ROOT / "config" / "flight.yaml")["environments"][args.env] if args.env else {}
args.name = args.name or args.env
app = SimulationApp({"headless": True, "extra_args": _env.get("kit_args", []) + [
    "--/crashreporter/url=", "--/crashreporter/skipOldDumpUpload=true"]})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

from pipeline.common.config import BUILD  # noqa: E402


def main() -> None:
    from pipeline.common import environments as envs

    out = BUILD / "envs" / args.name
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    url = envs.asset_url(_env["usd"] if args.env else args.usd)
    stage = envs.open_env(app, {"usd": url})
    root = stage.GetDefaultPrim() or stage.GetPrimAtPath("/World")
    # BBoxCache on the root fails on some environments ("null prim"), so bound each Gprim instead.
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    rows, counts = [], {"prims": 0, "gprims": 0, "colliders": 0, "lights": 0}
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        counts["prims"] += 1
        counts["colliders"] += p.HasAPI(UsdPhysics.CollisionAPI)
        counts["lights"] += "Light" in p.GetTypeName()
        if not p.IsA(UsdGeom.Gprim):
            continue
        counts["gprims"] += 1
        r = bb.ComputeWorldBound(p).ComputeAlignedRange()
        if not r.IsEmpty():
            lo, hi = np.array(r.GetMin()), np.array(r.GetMax())
            rows.append(np.r_[(lo + hi) / 2, hi - lo])
    m = np.array(rows)
    if len(m) == 0:                                   # e.g. Gaussian splats only: no Gprims to bound
        print("ENV_PROBE no gprims; only the top view")
        if args.topdown:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            topdown(stage, args.topdown, out / "topdown.png", plt)
        return
    np.save(out / "meshes.npy", m)
    lo, hi = m[:, :3] - m[:, 3:] / 2, m[:, :3] + m[:, 3:] / 2
    small = (m[:, 3] < 200) & (m[:, 4] < 200)          # drop ground planes / backdrops for the "core"
    tops = hi[small, 2]
    report = {
        "usd": url, "load_s": round(time.time() - t0, 1),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage), "up_axis": UsdGeom.GetStageUpAxis(stage),
        "bounds_all": [round(float(v), 1) for v in np.r_[lo.min(0), hi.max(0)]],
        "core_xy_p2_p98": [round(float(v), 1) for v in np.r_[np.percentile(m[small, :2], 2, axis=0),
                                                              np.percentile(m[small, :2], 98, axis=0)]],
        "object_top_z_percentiles": {str(q): round(float(np.percentile(tops, q)), 1) for q in (50, 90, 99, 99.9)},
        "ground_z_p5": round(float(np.percentile(lo[small, 2], 5)), 2),
        **counts,
    }
    report["lights_by_type"] = {}
    for p in stage.Traverse():
        if "Light" in p.GetTypeName() and p.GetTypeName() in ("DistantLight", "DomeLight"):
            report["lights_by_type"].setdefault(p.GetTypeName(), []).append(p.GetPath().pathString)
    (out / "probe.json").write_text(json.dumps(report, indent=2) + "\n")
    print("ENV_PROBE " + json.dumps(report))

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    h = ax.hist2d(m[small, 0], m[small, 1], bins=150, cmap="magma", cmin=1)
    fig.colorbar(h[3], ax=ax, label="objects per cell")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{args.name}: object density (centres of objects < 200 m)")
    fig.savefig(out / "density.png", dpi=90)
    if args.topdown:
        topdown(stage, args.topdown, out / "topdown.png", plt)


def topdown(stage, box, path: Path, plt, px_per_m: float | None = None) -> None:
    """Orthographic camera straight down over box, rendered with RTX, then a metric grid on top."""
    import omni.timeline
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from pxr import Gf

    x0, y0, x1, y1 = box
    w_m, h_m = x1 - x0, y1 - y0
    px_per_m = px_per_m or 1600.0 / max(w_m, h_m)        # ~1600 px on the long side
    W, H = int(w_m * px_per_m) // 2 * 2, int(h_m * px_per_m) // 2 * 2
    cam = RtxCamera("/World/ProbeTopDown", attributes={"projection": "orthographic",
                                                        "horizontalAperture": w_m * 10.0,   # ortho aperture is in tenths of scene units
                                                        "verticalAperture": h_m * 10.0,
                                                        "clippingRange": Gf.Vec2f(1.0, 5000.0)})
    # Looking down -Z with image up = +Y: identity rotation (USD cameras look along -Z, up +Y).
    cam.set_world_poses(positions=np.array([[(x0 + x1) / 2, (y0 + y1) / 2, 1500.0]]),
                        orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
    sensor = CameraSensor(cam, resolution=(H, W), annotators=["rgb"])
    omni.timeline.get_timeline_interface().play()
    for _ in range(120):
        app.update()
    rgb = sensor.get_data("rgb")[0].numpy()[..., :3].copy()
    fig, ax = plt.subplots(figsize=(W / 100, H / 100), dpi=100)
    ax.imshow(rgb, extent=[x0, x1, y0, y1], origin="upper")
    step = 50
    ax.set_xticks(np.arange(np.ceil(x0 / step) * step, x1 + 1, step))
    ax.set_yticks(np.arange(np.ceil(y0 / step) * step, y1 + 1, step))
    ax.grid(color="yellow", alpha=0.35, lw=0.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.savefig(path, dpi=100, bbox_inches="tight")


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
