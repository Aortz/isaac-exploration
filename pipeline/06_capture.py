"""Stage 06: drive a scripted sensor rig through fixed poses and run physics probes.

Captures RGB + depth (RTX camera) and an RTX LiDAR point cloud at every pose, then:
  - drops rigid spheres onto the terrain and records where they come to rest
  - casts PhysX rays through and beside the fixture gap
  - measures headless frame rate with the sensors active

Everything is written to reports/<run>/ for 07_validate.py.

Run: $ISAAC_ROOT/python.sh pipeline/06_capture.py --run-dir reports/<run>
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage  # noqa: E402

enable_extension("isaacsim.sensors.rtx.nodes")
app.update()

import omni.usd  # noqa: E402
from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    CameraSensor,
    Lidar,
    LidarSensor,
    RtxCamera,
    parse_generic_model_output_data,
)
from omni.physx import get_physx_interface, get_physx_scene_query_interface  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402

REF = BUILD / "reference"


# ---------------------------------------------------------------- geometry helpers

def look_rotation(forward: np.ndarray, world_up=np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """Camera-to-world rotation for a USD camera (looks along -Z, image up +Y)."""
    f = forward / np.linalg.norm(forward)
    if abs(f @ world_up) > 0.999:
        world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(f, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    return np.stack([right, up, -f], axis=1)


def yaw_rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_wxyz(r: np.ndarray) -> np.ndarray:
    """Rotation matrix (column-vector convention) to unit quaternion (w, x, y, z)."""
    t = np.trace(r)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0] * 4
        q[0] = (r[k, j] - r[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (r[j, i] + r[i, j]) / s
        q[1 + k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def direction(pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Unit vector: yaw clockwise from north, pitch negative = looking down."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    return np.array([math.sin(y) * math.cos(p), math.cos(y) * math.cos(p), math.sin(p)])


def ray_down(x: float, y: float) -> float | None:
    hit = get_physx_scene_query_interface().raycast_closest((x, y, 2000.0), (0.0, 0.0, -1.0), 5000.0)
    return float(hit["position"][2]) if hit["hit"] else None


# ---------------------------------------------------------------- pose generation

def make_poses(cfg: dict) -> list[dict]:
    c = cfg["capture"]
    bb = json.loads((REF / "bbox_visual.json").read_text())
    fixture = json.loads((REF / "fixture.json").read_text())
    lo, hi = np.array(bb["min"]), np.array(bb["max"])
    poses = []

    def add(kind, xy, agl, fwd):
        g = ray_down(*xy)
        if g is None:
            return
        poses.append({"id": f"{len(poses):03d}_{kind}", "kind": kind,
                      "position": [float(xy[0]), float(xy[1]), g + agl], "forward": fwd.tolist(), "ground_z": g})

    hg = c["high_grid"]
    margin = 0.15 * (hi[:2] - lo[:2])
    xs = np.arange(lo[0] + margin[0], hi[0] - margin[0] + 1e-6, hg["spacing"])
    ys = np.arange(lo[1] + margin[1], hi[1] - margin[1] + 1e-6, hg["spacing"])
    for x in xs:
        for y in ys:
            add("nadir", (x, y), hg["agl"], np.array([0.0, 0.0, -1.0]))
    for i, yaw in enumerate((0.0, 90.0, 180.0, 270.0)):
        add("oblique", ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2), hg["agl"], direction(-hg["oblique_pitch_deg"], yaw))

    lp = c["low_pass"]
    y_line = (lo[1] + hi[1]) / 2 + 0.2 * (hi[1] - lo[1])
    for agl in lp["agl"]:
        for x in np.linspace(lo[0] + 0.25 * (hi[0] - lo[0]), hi[0] - 0.25 * (hi[0] - lo[0]), lp["count"]):
            add(f"low{agl:g}m", (x, y_line), agl, direction(-10.0, 0.0))

    n = np.array(fixture["wall_normal"])
    gc = np.array(fixture["gap_centre"])
    start = gc - 4.0 * n
    poses.append({"id": f"{len(poses):03d}_gap", "kind": "gap", "position": start.tolist(),
                  "forward": n.tolist(), "ground_z": fixture["ground_z"]})
    return poses


# ---------------------------------------------------------------- capture

def lidar_points_world(gmo, lidar_pos: np.ndarray, lidar_rot: np.ndarray) -> np.ndarray:
    n = gmo.numElements
    if n == 0:
        return np.zeros((0, 3))
    a, b, c = (np.asarray(v[:n], dtype=np.float64) for v in (gmo.x, gmo.y, gmo.z))
    if int(gmo.elementsCoordsType) == 1:  # SPHERICAL: azimuth deg, elevation deg, range m
        keep = c > 0  # zero range = no return
        a, b, c = a[keep], b[keep], c[keep]
        az, el = np.radians(a), np.radians(b)
        pts = np.stack([c * np.cos(el) * np.cos(az), c * np.cos(el) * np.sin(az), c * np.sin(el)], axis=1)
    else:
        pts = np.stack([a, b, c], axis=1)
    if int(gmo.frameOfReference) == 1:  # WORLD
        return pts
    return pts @ lidar_rot.T + lidar_pos


def save_rgb(path: Path, rgb: np.ndarray) -> None:
    try:
        from PIL import Image
        Image.fromarray(rgb[..., :3]).save(path)
    except ImportError:
        np.save(path.with_suffix(".npy"), rgb[..., :3])


def capture(cfg: dict, run_dir: Path, poses: list[dict]) -> dict:
    c = cfg["capture"]
    cc = c["camera"]
    frames = run_dir / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    stage = omni.usd.get_context().get_stage()

    cam = RtxCamera("/World/Robots/rig_camera")
    ucam = UsdGeom.Camera(stage.GetPrimAtPath("/World/Robots/rig_camera"))
    ucam.GetFocalLengthAttr().Set(cc["focal_length_mm"])
    ucam.GetHorizontalApertureAttr().Set(cc["horizontal_aperture_mm"])
    ucam.GetClippingRangeAttr().Set(Gf.Vec2f(*cc["clip"]))
    W, H = cc["width"], cc["height"]
    cam_sensor = CameraSensor(cam, resolution=(H, W), annotators=["rgb", "distance_to_image_plane"])
    fx = cc["focal_length_mm"] / cc["horizontal_aperture_mm"] * W
    intrinsics = {"width": W, "height": H, "fx": fx, "fy": fx, "cx": W / 2, "cy": H / 2}

    lidar = Lidar.create("/World/Robots/rig_lidar", config=c["lidar_config"], variant=c["lidar_variant"],
                         aux_output_level="NONE")
    lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])

    records = []
    lidar_frames_info = set()
    for pose in poses:
        p = np.array(pose["position"])
        r_cam = look_rotation(np.array(pose["forward"]))
        f = np.array(pose["forward"])
        yaw = math.atan2(f[1], f[0]) if np.linalg.norm(f[:2]) > 1e-6 else 0.0
        r_lidar = yaw_rotation(yaw)
        cam.set_world_poses(positions=p[None], orientations=quat_wxyz(r_cam)[None])
        lidar.set_world_poses(positions=p[None], orientations=quat_wxyz(r_lidar)[None])

        for _ in range(c["settle_frames"]):
            app.update()
        # get_data returns views of GPU buffers that later frames reuse, so copy to host right away.
        rgb, _ = cam_sensor.get_data("rgb")
        depth, _ = cam_sensor.get_data("distance_to_image_plane")
        rgb = None if rgb is None else rgb.numpy().copy()
        depth = None if depth is None else depth.numpy().astype(np.float32).squeeze()
        clouds, seen_frames = [], set()
        for _ in range(12):  # two full rotations at 10 Hz with 60 Hz rendering
            app.update()
            data, _ = lidar_sensor.get_data("generic-model-output")
            if data is None:
                continue
            gmo = parse_generic_model_output_data(data)
            if gmo.frameId in seen_frames:  # the annotator repeats the last scan until a new one lands
                continue
            seen_frames.add(gmo.frameId)
            lidar_frames_info.add((int(gmo.frameOfReference), int(gmo.elementsCoordsType)))
            clouds.append(lidar_points_world(gmo, p, r_lidar))
        cloud = np.concatenate(clouds) if clouds else np.zeros((0, 3))

        if rgb is not None:
            save_rgb(frames / f"{pose['id']}_rgb.png", rgb)
        if depth is not None:
            np.save(frames / f"{pose['id']}_depth.npy", depth)
        np.save(frames / f"{pose['id']}_lidar.npy", cloud.astype(np.float32))
        records.append({**pose, "cam_R": r_cam.tolist(), "lidar_R": r_lidar.tolist(),
                        "has_rgb": rgb is not None, "has_depth": depth is not None,
                        "lidar_points": int(len(cloud))})
        print(f"  {pose['id']}: depth={'ok' if depth is not None else 'none'} lidar={len(cloud)} pts")

    t0 = time.perf_counter()
    for _ in range(c["throughput_steps"]):
        app.update()
    fps = c["throughput_steps"] / (time.perf_counter() - t0)
    try:
        gpu_mem = int(subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"]).decode().split()[0])
    except Exception:
        gpu_mem = None

    return {"intrinsics": intrinsics, "poses": records, "throughput_fps": fps, "gpu_mem_mib": gpu_mem,
            "lidar_frame_info": sorted(lidar_frames_info)}


# ---------------------------------------------------------------- physics probes

def gap_rays() -> dict:
    fx = json.loads((REF / "fixture.json").read_text())
    n = np.array(fx["wall_normal"])
    sq = get_physx_scene_query_interface()
    out = {}
    for name, centre in (("through_gap", fx["gap_centre"]), ("blocked", fx["blocked_point"])):
        origin = np.array(centre) - 3.0 * n
        hit = sq.raycast_closest(tuple(origin), tuple(n), 6.0)
        out[name] = {"origin": origin.tolist(), "hit": bool(hit["hit"]),
                     "distance": float(hit["distance"]) if hit["hit"] else None,
                     "collider": str(hit.get("collision", "")) if hit["hit"] else None}
    return out


def sphere_probes(cfg: dict, timeline) -> list[dict]:
    pc = cfg["capture"]["probes"]
    bb = json.loads((REF / "bbox_visual.json").read_text())
    lo, hi = np.array(bb["min"]), np.array(bb["max"])
    margin = 0.1 * (hi[:2] - lo[:2])
    pts = []
    for x in np.arange(lo[0] + margin[0], hi[0] - margin[0], pc["grid_spacing"]):
        for y in np.arange(lo[1] + margin[1], hi[1] - margin[1], pc["grid_spacing"]):
            g = ray_down(x, y)
            if g is not None:
                pts.append((float(x), float(y), g))

    timeline.stop()
    app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Scope.Define(stage, "/World/Probes")
    r = pc["sphere_radius"]
    paths = []
    for i, (x, y, g) in enumerate(pts):
        path = f"/World/Probes/sphere_{i:03d}"
        s = UsdGeom.Sphere.Define(stage, path)
        s.CreateRadiusAttr(r)
        s.AddTranslateOp().Set(Gf.Vec3d(x, y, g + pc["drop_height"]))
        UsdPhysics.RigidBodyAPI.Apply(s.GetPrim())
        UsdPhysics.CollisionAPI.Apply(s.GetPrim())
        UsdPhysics.MassAPI.Apply(s.GetPrim()).CreateMassAttr(1.0)
        # Heavy angular damping so probes settle where they land instead of rolling downhill.
        PhysxSchema.PhysxRigidBodyAPI.Apply(s.GetPrim()).CreateAngularDampingAttr(pc["angular_damping"])
        paths.append(path)
    timeline.play()
    for _ in range(int(pc["settle_seconds"] * 60)):
        app.update()
    physx = get_physx_interface()

    def positions():
        ts = [physx.get_rigidbody_transformation(p) for p in paths]
        return [list(t["position"]) if t.get("ret_val") else None for t in ts]

    before = positions()
    for _ in range(30):  # 0.5 s more, to tell resting probes from moving ones
        app.update()
    after = positions()
    out = []
    for path, (x, y, g), p0, p1 in zip(paths, pts, before, after):
        speed = float(np.linalg.norm(np.subtract(p1, p0)) / 0.5) if p0 and p1 else None
        out.append({"path": path, "xy": [x, y], "ray_ground_z": g, "rest_position": p1, "radius": r,
                    "final_speed": speed})
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    args = ap.parse_args()
    run_dir = (PROJECT_ROOT / args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_config()

    open_stage(usd_path=str(BUILD / "isaac" / "site_env.usd"))
    while is_stage_loading():
        app.update()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        app.update()

    poses = make_poses(cfg)
    print(f"capturing {len(poses)} poses")
    result = capture(cfg, run_dir, poses)
    result["gap_rays"] = gap_rays()
    result["spheres"] = sphere_probes(cfg, timeline)
    timeline.stop()
    (run_dir / "capture.json").write_text(json.dumps(result, indent=1) + "\n")
    print(f"wrote {run_dir / 'capture.json'}  fps={result['throughput_fps']:.1f}  "
          f"gap_rays={ {k: v['hit'] for k, v in result['gap_rays'].items()} }")


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
