"""F2: render the F1 flight through Cesium tiles: RGB + depth + pose per frame.

Before each capture the renderer is stepped until Cesium reports no tiles loading (bounded by
render.tile_timeout_s), after a warm-up pass that visits every Nth pose to fill the tile cache.
Otherwise level-of-detail pop-in shows up as flicker that has nothing to do with the scene.

Run: CESIUM_ION_TOKEN=... $ISAAC_ROOT/python.sh flight/f2_render.py --flight marina_bay_pass
Writes build/flight/<flight>/render/{frames/NNNNNN_rgb.png, frames/NNNNNN_depth.npy, stream_stats.csv, render.json}
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from pipeline.common.cesium_support import simulation_app_config  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--flight", required=True)
ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
ap.add_argument("--stage", help="Cesium stage (default build/cesium/<site>_<source>.usda)")
ap.add_argument("--max-frames", type=int, default=0, help="render only the first N frames (0 = all)")
ap.add_argument("--token", default=os.environ.get("CESIUM_ION_TOKEN", ""))
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp(simulation_app_config(headless=True))

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from pipeline.common import cesium_support as cs  # noqa: E402
from pipeline.common.config import BUILD, load_config  # noqa: E402

CAMERA = "/World/FlightCamera"


def main() -> None:
    cs.ensure_loaded()
    cfg = load_config(args.config)
    fl, cam, rcfg = cfg["flights"][args.flight], cfg["camera"], cfg["render"]
    fdir = BUILD / "flight" / args.flight
    poses = np.loadtxt(fdir / "camera_poses_usd.csv", delimiter=",", comments="#")
    if args.max_frames:
        poses = poses[: args.max_frames]
    stage_path = Path(args.stage) if args.stage else BUILD / "cesium" / f"{fl['site']}_{fl['source']}.usda"
    if not stage_path.exists():
        raise SystemExit(f"{stage_path} missing; run cesium/build_cesium_stage.py --site {fl['site']} --source {fl['source']}")
    if fl["source"] in ("google", "osm") and not args.token:
        raise SystemExit("CESIUM_ION_TOKEN is not set; ion tilesets will not stream")

    stage = cs.open_stage(app, stage_path, args.token)
    rtx = RtxCamera(CAMERA)
    ucam = UsdGeom.Camera(stage.GetPrimAtPath(CAMERA))
    ucam.GetFocalLengthAttr().Set(cam["focal_length_mm"])
    ucam.GetHorizontalApertureAttr().Set(cam["horizontal_aperture_mm"])
    ucam.GetClippingRangeAttr().Set(Gf.Vec2f(*cam["clip"]))
    W, H = cam["width"], cam["height"]
    sensor = CameraSensor(rtx, resolution=(H, W), annotators=["rgb", "distance_to_image_plane"])
    # Intrinsics as the renderer will use them (the sensor enforces square pixels on the prim).
    attrs = {n: ucam.GetPrim().GetAttribute(n).Get() for n in
             ("focalLength", "horizontalAperture", "verticalAperture", "horizontalApertureOffset",
              "verticalApertureOffset", "projection")}
    attrs.update({n: ucam.GetPrim().GetAttribute(n).Get() for n in ucam.GetPrim().GetPropertyNames()
                  if "lens" in n.lower() or "distortion" in n.lower() or "projectionType" in n})
    fx_usd = attrs["focalLength"] / attrs["horizontalAperture"] * W
    fy_usd = attrs["focalLength"] / attrs["verticalAperture"] * H
    viewport_bound = cs.set_viewport_camera(CAMERA)
    omni.timeline.get_timeline_interface().play()  # Replicator only delivers frames while playing

    def place(row):
        _, _, x, y, z, qw, qx, qy, qz = row
        rtx.set_world_poses(positions=np.array([[x, y, z]]), orientations=np.array([[qw, qx, qy, qz]]))

    t0 = time.time()
    for row in poses[:: rcfg["warmup_stride"]]:
        place(row)
        cs.wait_for_tiles(app, rcfg["tile_timeout_s"])
    warmup_s = time.time() - t0

    out = fdir / "render"
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    stats_f = (out / "stream_stats.csv").open("w", newline="")
    writer = None
    unsettled, close_frames = 0, []
    t1 = time.time()
    for row in poses:
        i = int(row[0])
        place(row)
        stats, settled, n_updates = cs.wait_for_tiles(app, rcfg["tile_timeout_s"])
        for _ in range(rcfg["settle_updates"]):
            app.update()
        rgb, _ = sensor.get_data("rgb")
        depth, _ = sensor.get_data("distance_to_image_plane")
        # get_data returns views of GPU buffers that later frames reuse: copy to host right away.
        rgb = None if rgb is None else rgb.numpy()[..., :3].copy()
        depth = None if depth is None else depth.numpy().astype(np.float32).squeeze()
        if rgb is None or depth is None:
            raise RuntimeError(f"frame {i}: sensor returned no data")
        Image.fromarray(rgb).save(frames / f"{i:06d}_rgb.png")
        np.save(frames / f"{i:06d}_depth.npy", depth)
        finite = depth[np.isfinite(depth) & (depth > 0)]
        dmin = float(finite.min()) if finite.size else float("inf")
        if dmin < rcfg["min_clearance_m"]:
            close_frames.append(i)
        unsettled += not settled
        rec = {"frame": i, "settled": int(settled), "updates": n_updates, "depth_min_m": dmin,
               "sky_fraction": float(1 - finite.size / depth.size), **stats}
        if writer is None:
            writer = csv.DictWriter(stats_f, fieldnames=list(rec))
            writer.writeheader()
        writer.writerow(rec)
        if i % 30 == 0:
            print(f"  frame {i}/{len(poses)}  settled={settled} updates={n_updates} depth_min={dmin:.1f} m "
                  f"tiles_rendered={stats['tiles_rendered']}", flush=True)
    stats_f.close()

    summary = {
        "flight": args.flight, "stage": str(stage_path), "frames": int(len(poses)),
        "viewport_bound": viewport_bound,
        "camera_prim_attributes": {k: (v if isinstance(v, (int, float, str, bool)) or v is None else str(v))
                                   for k, v in attrs.items()},
        "intrinsics": {"width": W, "height": H, "fx": fx_usd, "fy": fy_usd, "cx": W / 2, "cy": H / 2}, "warmup_s": round(warmup_s, 1),
        "render_s": round(time.time() - t1, 1), "unsettled_frames": unsettled,
        "close_frames": close_frames[:50], "close_frame_count": len(close_frames),
    }
    (out / "render.json").write_text(json.dumps(summary, indent=2) + "\n")
    print("F2_SUMMARY " + json.dumps(summary))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        print("F2_FAIL see traceback above")
        sys.exit(1)
    finally:
        app.close()
