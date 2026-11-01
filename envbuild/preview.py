"""Oblique preview shots of an environment, to judge its look before flying it.

Renders a few perspective views from drone-like heights around the origin (each looks at a point on the
ground), tiled into one image: build/envs/<env>/preview.jpg.

Run: $ISAAC_ROOT/python.sh envbuild/preview.py --env sg_mangrove [--heights 15 40 80] [--lighting default]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--env", required=True, help="config/flight.yaml environments.<env>")
ap.add_argument("--heights", type=float, nargs="+", default=[15.0, 40.0, 80.0], help="camera heights above z 0 (m)")
ap.add_argument("--azimuths", type=float, nargs="+", default=[30.0, 210.0], help="viewing directions (deg from +x)")
ap.add_argument("--radius", type=float, default=60.0, help="camera distance from the origin (m)")
ap.add_argument("--lighting", default="default")
ap.add_argument("--width", type=int, default=960)
ap.add_argument("--height", type=int, default=540)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

from pipeline.common.config import load_config  # noqa: E402

ENV = load_config(PROJECT_ROOT / "config" / "flight.yaml")["environments"][args.env]
app = SimulationApp({"headless": True, "extra_args": ENV.get("kit_args", []) + [
    "--/crashreporter/url=", "--/crashreporter/skipOldDumpUpload=true"]})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.prims import XformPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from pxr import Gf  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from pipeline.common import environments as envs  # noqa: E402
from pipeline.common.config import BUILD  # noqa: E402


def look_at_wxyz(eye: np.ndarray, target: np.ndarray) -> np.ndarray:
    f = target - eye
    f /= np.linalg.norm(f)
    right = np.cross(f, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    q = R.from_matrix(np.column_stack([right, np.cross(right, f), -f])).as_quat()
    return np.array([q[3], q[0], q[1], q[2]])


def main() -> None:
    stage = envs.open_env(app, ENV)
    envs.apply_lighting(stage, ENV, args.lighting)
    cam = RtxCamera("/World/PreviewCam", attributes={"focalLength": 14.0, "horizontalAperture": 20.955,
                                                     "clippingRange": Gf.Vec2f(0.1, 20000.0)})
    xf = XformPrim("/World/PreviewCam")
    sensor = CameraSensor(cam, resolution=(args.height, args.width), annotators=["rgb"])
    omni.timeline.get_timeline_interface().play()
    shots = []
    for h in args.heights:
        row = []
        for az in np.radians(args.azimuths):
            eye = np.array([-args.radius * np.cos(az), -args.radius * np.sin(az), h + ENV.get("ground_z", 0.0)])
            # Look ahead and down: at the ground ~2.5 camera heights in front (about 22 deg down).
            target = np.array([np.cos(az), np.sin(az), 0.0]) * 2.5 * max(h, 6.0) + [eye[0], eye[1], ENV.get("ground_z", 0.0)]
            xf.set_world_poses(positions=eye[None], orientations=look_at_wxyz(eye, target)[None])
            # First shot: let MDL compilation and texture streaming finish (ground renders white until then).
            for _ in range(400 if not shots and not row else max(ENV.get("warmup_updates", 30), 30)):
                app.update()
            rgb, _ = sensor.get_data("rgb")
            row.append(np.array(rgb.numpy()[..., :3]))          # copy: get_data buffers are reused
            print(f"  shot h={h:g} az={np.degrees(az):.0f}", flush=True)
        shots.append(np.hstack(row))
    out = BUILD / "envs" / args.env / "preview.jpg"
    from PIL import Image

    Image.fromarray(np.vstack(shots)).save(out, quality=90)
    print(f"PREVIEW -> {out}")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()
