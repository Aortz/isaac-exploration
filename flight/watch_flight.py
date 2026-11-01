"""Watch a flown trajectory from a chase camera: the drone model replayed through its scene.

Replays the flight's ground truth (groundtruth_200hz.csv, i.e. what Pegasus / PX4 / ArduPilot actually
flew) as a kinematic drone model inside the flight's environment (Rivermark, NuRec, Cesium), with a
camera that follows from behind and above. Nothing is simulated here, so it runs at about real time
in the GUI (the physics pass runs headless in an empty stage at ~0.1x).

  GUI:     $ISAAC_ROOT/python.sh flight/watch_flight.py --flight rm_p3_px4 [--speed 2]
  Record:  $ISAAC_ROOT/python.sh flight/watch_flight.py --flight rm_p3_px4 --record    (headless, lockstep)
           -> build/flight/<flight>/watch/chase.mp4
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--flight", required=True)
ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
ap.add_argument("--speed", type=float, default=1.0, help="GUI playback speed (sim seconds per wall second)")
ap.add_argument("--record", action="store_true", help="render the chase view to an mp4 instead of opening the GUI")
ap.add_argument("--fps", type=float, default=30.0, help="recording frame rate")
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
ap.add_argument("--dist", type=float, default=7.0, help="camera distance behind the drone (m)")
ap.add_argument("--rise", type=float, default=2.5, help="camera height above the drone (m)")
ap.add_argument("--lighting", default="default")
ap.add_argument("--loop", action="store_true", help="GUI: restart at the end")
ap.add_argument("--token", default=os.environ.get("CESIUM_ION_TOKEN", ""))
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

from pipeline.common.cesium_support import simulation_app_config  # noqa: E402
from pipeline.common.config import load_config  # noqa: E402

_cfg = load_config(args.config)
_fl = _cfg["flights"][args.flight]
_app_cfg = simulation_app_config(headless=args.record)
if "env" in _fl:
    _app_cfg["extra_args"] += _cfg["environments"][_fl["env"]].get("kit_args", [])
app = SimulationApp(_app_cfg)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.prims import XformPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402
from scipy.spatial.transform import Slerp  # noqa: E402

from pipeline.common import cesium_support as cs  # noqa: E402
from pipeline.common import environments as envs  # noqa: E402
from pipeline.common.config import BUILD  # noqa: E402

DRONE, CAM = "/World/WatchDrone", "/World/ChaseCam"
IRIS_USD = PROJECT_ROOT / "third_party/pegasus/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"


def drone_usd(fl: dict) -> Path:
    if fl.get("vehicle"):
        p = BUILD / "vehicles" / f"{fl['vehicle']}.usda"          # written by pegasus_fly.py
        if not p.exists():
            raise SystemExit(f"{p} missing: fly the flight once with pegasus_fly.py to generate the vehicle asset")
        return p
    return IRIS_USD


def make_kinematic(stage, root: str) -> None:
    """The replayed model is moved by hand: switch off every physics schema it brought along."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateArticulationEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)


def look_at_wxyz(eye: np.ndarray, target: np.ndarray) -> np.ndarray:
    """USD camera orientation (looks along -Z, up +Y) at eye, looking at target, world up = +Z."""
    f = target - eye
    f /= np.linalg.norm(f)
    right = np.cross(f, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    q = R.from_matrix(np.column_stack([right, up, -f])).as_quat()
    return np.array([q[3], q[0], q[1], q[2]])


def main() -> None:
    cfg, fl = _cfg, _fl
    env = cfg["environments"][fl["env"]] if "env" in fl else None
    gt = np.loadtxt(BUILD / "flight" / args.flight / "groundtruth_200hz.csv", delimiter=",", comments="#")
    t, p, v = gt[:, 0], gt[:, 1:4], gt[:, 8:11]
    rots = Slerp(t, R.from_quat(gt[:, [5, 6, 7, 4]]))

    if env:
        stage = envs.open_env(app, env)
        envs.apply_lighting(stage, env, args.lighting)
        settle, warm = env["settle_updates"], env["warmup_updates"]
        wait = lambda: envs.settle(app, settle)   # noqa: E731
    else:
        cs.ensure_loaded()
        stage_path = BUILD / "cesium" / f"{fl['site']}_{fl['source']}.usda"
        if fl["source"] in ("google", "osm") and not args.token:
            raise SystemExit("CESIUM_ION_TOKEN is not set; ion tilesets will not stream")
        stage = cs.open_stage(app, stage_path, args.token)
        cs.apply_look(stage, dict(cfg["render"]))
        warm = 0
        wait = lambda: cs.wait_for_tiles(app, cfg["render"]["tile_timeout_s"])   # noqa: E731

    UsdGeom.Xform.Define(stage, DRONE)
    stage.GetPrimAtPath(DRONE).GetReferences().AddReference(str(drone_usd(fl)))
    for _ in range(5):
        app.update()
    make_kinematic(stage, DRONE)
    drone = XformPrim(DRONE, reset_xform_op_properties=True)
    cam = RtxCamera(CAM, attributes={"focalLength": 18.0, "horizontalAperture": 20.955,
                                     "clippingRange": Gf.Vec2f(0.1, 20000.0)})
    cam_xf = XformPrim(CAM)
    cs.set_viewport_camera(CAM)

    heading = [None]

    def place(tq: float) -> None:
        tq = float(np.clip(tq, t[0], t[-1]))
        pos = np.array([np.interp(tq, t, p[:, k]) for k in range(3)])
        vel = np.array([np.interp(tq, t, v[:, k]) for k in range(3)])
        q = rots(tq).as_quat()
        drone.set_world_poses(positions=pos[None], orientations=np.array([[q[3], q[0], q[1], q[2]]]))
        # Follow the direction of travel when moving, the body heading when hovering; smooth the swing.
        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.7 else R.from_quat(q).as_euler("ZYX")[0]
        if heading[0] is None:
            heading[0] = yaw
        d = np.angle(np.exp(1j * (yaw - heading[0])))
        heading[0] += 0.08 * d
        eye = pos - args.dist * np.array([np.cos(heading[0]), np.sin(heading[0]), 0.0]) + [0.0, 0.0, args.rise]
        cam_xf.set_world_poses(positions=eye[None], orientations=look_at_wxyz(eye, pos + [0.0, 0.0, 0.3])[None])

    omni.timeline.get_timeline_interface().play()
    place(t[0])
    for _ in range(max(warm, 10)):
        app.update()

    if args.record:
        out = BUILD / "flight" / args.flight / "watch"
        out.mkdir(parents=True, exist_ok=True)
        sensor = CameraSensor(cam, resolution=(args.height, args.width), annotators=["rgb"])
        # Warm the scene along the path (textures / tiles), then render each frame in lockstep.
        for tq in np.arange(t[0], t[-1], 2.0):
            heading[0] = None
            place(tq)
            wait()
        heading[0] = None
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                               "-s", f"{args.width}x{args.height}", "-r", f"{args.fps:g}", "-i", "-",
                               "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-pix_fmt", "yuv420p",
                               "-movflags", "+faststart", str(out / "chase.mp4")], stdin=subprocess.PIPE)
        frames = np.arange(t[0], t[-1], 1.0 / args.fps)
        for i, tq in enumerate(frames):
            place(tq)
            wait()
            rgb, _ = sensor.get_data("rgb")
            ff.stdin.write(np.ascontiguousarray(rgb.numpy()[..., :3]).tobytes())
            if i % 150 == 0:
                print(f"  frame {i}/{len(frames)}", flush=True)
        ff.stdin.close()
        ff.wait()
        print(f"WATCH chase video -> {out / 'chase.mp4'}")
        return

    print(f"WATCH {args.flight}: {t[-1]:.0f} s at {args.speed:g}x; close the window to stop", flush=True)
    t0 = time.time()
    while app.is_running():
        tq = t[0] + (time.time() - t0) * args.speed
        if tq > t[-1]:
            if not args.loop:
                tq = t[-1]
            else:
                t0, heading[0] = time.time(), None
                continue
        place(tq)
        app.update()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()
