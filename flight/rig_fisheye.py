"""Fly the F1 trajectory with a 4-fisheye rig through Cesium tiles, measure rates, optionally publish ROS 2.

Modes:
  lockstep  sim time advances exactly 1/camera_hz per frame and only after every camera has a
            finished frame (tiles settled + extra renderer updates). All cameras of a frame carry
            the same stamp; IMU samples up to that stamp go out first, then /clock, then images.
            Consumers should run with use_sim_time:=true; wall speed does not matter.
  freerun   one renderer update per frame, no tile wait: the raw throughput of the rig. Frames may
            lag the pose they are stamped with; compare a --save-all run against a lockstep one
            (flight/rig_lag_check.py) to measure that.

ROS 2 (--ros2; source ros2/ros_env.sh first, uses Isaac's bundled Jazzy):
  /clock, /imu/data (200 Hz, from F1 imu.csv), /<cam>/image_raw + /<cam>/camera_info (equidistant),
  /tf (world -> base_link ground truth), /tf_static (base_link -> <cam>_optical), /ground_truth/odom

--stereo swaps the fisheyes for the pinhole stereo pair in `stereo:` (left/right, plumb_bob with zero
distortion) and also writes stereo.json, times.csv and groundtruth_left.csv for offline stereo VO.

Run: source ros2/ros_env.sh && $ISAAC_ROOT/python.sh flight/rig_fisheye.py --flight marina_bay_pass [--mode freerun] [--ros2]
Writes build/flight/<flight>/rig_<mode>[_<tag>]/{rates.json, camchain.yaml, frames/}  (_stereo for --stereo)
"""

from __future__ import annotations

import argparse
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
ap.add_argument("--mode", choices=["lockstep", "freerun"], default="lockstep")
ap.add_argument("--ros2", action="store_true", help="publish ROS 2 topics (source ros2/ros_env.sh first)")
ap.add_argument("--max-fps", type=float, default=0.0,
                help="cap frames per wall second (0 = uncapped); keeps the GPU from running flat out")
ap.add_argument("--updates", type=int, default=1, help="freerun: renderer updates per frame")
ap.add_argument("--max-frames", type=int, default=0)
ap.add_argument("--save-every", type=int, default=30, help="save the 4 images every N frames (0 = never)")
ap.add_argument("--save-all", action="store_true", help="save every frame (for --lag-check runs)")
ap.add_argument("--lag-test", action="store_true", help="measure image-vs-pose lag per renderer-update count, then exit")
ap.add_argument("--imu-file", default="imu.csv", help="IMU stream to publish (imu_clean.csv = no noise/vibration)")
ap.add_argument("--tag", default="", help="output folder suffix: rig_<mode>_<tag>")
ap.add_argument("--lighting", default="default", help="environment lighting variant (environments.<env>.lighting)")
ap.add_argument("--stereo", action="store_true", help="render the pinhole stereo pair (config stereo:) instead")
ap.add_argument("--gui", action="store_true", help="open the Isaac Sim window (viewport shows the first camera)")
ap.add_argument("--token", default=os.environ.get("CESIUM_ION_TOKEN", ""))
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

from pipeline.common.config import load_config as _load  # noqa: E402

_cfg0 = _load(args.config)
_fl0 = _cfg0["flights"][args.flight]
_app_cfg = simulation_app_config(headless=not args.gui)
if "env" in _fl0:   # e.g. Gaussian-splat environments need extra renderer settings at startup
    _app_cfg["extra_args"] += _cfg0["environments"][_fl0["env"]].get("kit_args", [])
app = SimulationApp(_app_cfg)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.prims import XformPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402
from scipy.spatial.transform import Slerp  # noqa: E402

from flight.f1_trajectory import CV_TO_USD, body_to_camera_cv  # noqa: E402
from pipeline.common import cesium_support as cs  # noqa: E402
from pipeline.common import environments as envs  # noqa: E402
from pipeline.common.config import BUILD, load_config  # noqa: E402

RIG = "/World/Rig"


def body_to_camera(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    """OpenCV camera axes in the body (FLU) frame: the forward camera pitched down, then yawed."""
    return R.from_euler("z", yaw_deg, degrees=True).as_matrix() @ body_to_camera_cv(pitch_deg)


class RosOut:
    """Thin rclpy publisher set; every message is stamped with sim time."""

    def __init__(self, cams: dict, intr: dict, extrinsics: dict, offsets: dict):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, Imu
        from tf2_msgs.msg import TFMessage

        self.m = {"Clock": Clock, "Imu": Imu, "Image": Image, "CameraInfo": CameraInfo, "Odometry": Odometry,
                  "TFMessage": TFMessage, "TransformStamped": TransformStamped}
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("isaac_fisheye_rig")
        self.clock = self.node.create_publisher(Clock, "/clock", 10)
        self.imu = self.node.create_publisher(Imu, "/imu/data", 400)
        self.odom = self.node.create_publisher(Odometry, "/ground_truth/odom", 50)
        self.tf = self.node.create_publisher(TFMessage, "/tf", 50)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static = self.node.create_publisher(TFMessage, "/tf_static", static_qos)
        self.img = {c: self.node.create_publisher(Image, f"/{c}/image_raw", 10) for c in cams}
        self.info = {c: self.node.create_publisher(CameraInfo, f"/{c}/camera_info", 10) for c in cams}
        self.info_msg = {}
        for c in cams:
            ci = CameraInfo()
            ci.header.frame_id = f"{c}_optical"
            ci.width, ci.height = intr["width"], intr["height"]
            ci.distortion_model = intr["model"]
            ci.d = [float(x) for x in intr["d"]]
            fx, fy, cx, cy = intr["fx"], intr["fy"], intr["width"] / 2, intr["height"] / 2
            ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_msg[c] = ci
        tfs = []
        for c, r_bc in extrinsics.items():
            tfs.append(self._tf(0.0, "base_link", f"{c}_optical", offsets[c], R.from_matrix(r_bc).as_quat()))
        self.tf_static.publish(TFMessage(transforms=tfs))

    def _stamp(self, msg, t: float) -> None:
        sec = int(t)
        msg.sec, msg.nanosec = sec, int(round((t - sec) * 1e9))

    def _tf(self, t, parent, child, p, q_xyzw):
        m = self.m["TransformStamped"]()
        self._stamp(m.header.stamp, t)
        m.header.frame_id, m.child_frame_id = parent, child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in p)
        (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
         m.transform.rotation.w) = (float(v) for v in q_xyzw)
        return m

    def publish_clock(self, t: float) -> None:
        c = self.m["Clock"]()
        self._stamp(c.clock, t)
        self.clock.publish(c)

    def publish_imu(self, row) -> None:
        msg = self.m["Imu"]()
        self._stamp(msg.header.stamp, row[0])
        msg.header.frame_id = "base_link"
        msg.orientation_covariance[0] = -1.0  # no orientation estimate
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in row[1:4])
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in row[4:7])
        self.imu.publish(msg)

    def publish_pose(self, t: float, p, q_xyzw, v) -> None:
        self.tf.publish(self.m["TFMessage"](transforms=[self._tf(t, "world", "base_link", p, q_xyzw)]))
        o = self.m["Odometry"]()
        self._stamp(o.header.stamp, t)
        o.header.frame_id, o.child_frame_id = "world", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(x) for x in p)
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z,
         o.pose.pose.orientation.w) = (float(x) for x in q_xyzw)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (float(x) for x in v)  # world frame
        self.odom.publish(o)

    def publish_image(self, cam: str, t: float, rgb: np.ndarray) -> None:
        msg = self.m["Image"]()
        self._stamp(msg.header.stamp, t)
        msg.header.frame_id = f"{cam}_optical"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding, msg.step = "rgb8", rgb.shape[1] * 3
        msg.data = rgb.tobytes()
        self.img[cam].publish(msg)
        info = self.info_msg[cam]
        info.header.stamp = msg.header.stamp
        self.info[cam].publish(info)

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()


def write_camchain(path: Path, rig: dict, extrinsics: dict, imu_cfg: dict) -> None:
    """Kalibr-style camchain (OpenVINS reads this). T_cam_imu maps IMU (body FLU) points into the camera."""
    lines = []
    for i, (c, r_bc) in enumerate(extrinsics.items()):
        t_cb = np.eye(4)
        t_cb[:3, :3] = r_bc.T
        rows = ",\n    ".join("[" + ", ".join(f"{v:.9f}" for v in r) + "]" for r in t_cb)
        lines += [f"cam{i}:", f"  rostopic: /{c}/image_raw", "  camera_model: pinhole",
                  "  distortion_model: equidistant",
                  f"  distortion_coeffs: [{', '.join(str(float(k)) for k in rig['k'])}]",
                  f"  intrinsics: [{rig['fx']}, {rig['fy']}, {rig['width'] / 2}, {rig['height'] / 2}]",
                  f"  resolution: [{rig['width']}, {rig['height']}]",
                  f"  T_cam_imu:\n  - {rows}", "  timeshift_cam_imu: 0.0"]
    lines += ["# IMU (body FLU): rate_hz %s, noise densities in imu0 of the EuRoC export; motor vibration "
              "is extra, inflate them for VIO" % imu_cfg["rate_hz"]]
    path.write_text("\n".join(lines) + "\n")


def write_stereo(out: Path, intr: dict, extrinsics: dict, offsets: dict, t_frames, pos, rots) -> None:
    """stereo.json + times.csv + groundtruth_left.csv (left camera, OpenCV frame, camera -> world ENU)."""
    def t_body(c):
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = extrinsics[c], offsets[c]
        return m

    t_bl, t_br = t_body("left"), t_body("right")
    meta = {**{k: intr[k] for k in ("width", "height", "fx", "fy")}, "cx": intr["width"] / 2, "cy": intr["height"] / 2,
            "baseline_m": float(np.linalg.norm(offsets["left"] - offsets["right"])), "camera_hz": round(1.0 / (t_frames[1] - t_frames[0]), 6),
            "T_body_left": t_bl.tolist(), "T_left_right": (np.linalg.inv(t_bl) @ t_br).tolist()}
    (out / "stereo.json").write_text(json.dumps(meta, indent=2) + "\n")
    np.savetxt(out / "times.csv", np.column_stack([np.arange(len(t_frames)), t_frames]), fmt=["%d", "%.9f"],
               delimiter=",", header="frame,t_s", comments="")
    r_wl = rots.as_matrix() @ extrinsics["left"]
    p_wl = pos + rots.apply(offsets["left"])
    np.savetxt(out / "groundtruth_left.csv", np.column_stack([t_frames, p_wl, R.from_matrix(r_wl).as_quat()]),
               fmt="%.9f", delimiter=",", header="t_s,px,py,pz,qx,qy,qz,qw", comments="")


def lag_test(sensors: dict, rig_xf, p0, r0, rcfg: dict, fdir: Path, wait, n: int = 36, step_deg: float = 10.0) -> None:
    """How many renderer updates until a camera's image shows the pose just set?

    The rig spins in place by step_deg per frame (big, unambiguous image change; no tile changes).
    Reference images come from a fully settled pass; then each candidate update count is replayed
    and every frame is matched against the references of the neighbouring poses.
    """
    def pose(k):
        q = (r0 * R.from_euler("z", k * step_deg, degrees=True)).as_quat()
        rig_xf.set_world_poses(positions=np.asarray(p0)[None], orientations=np.array([[q[3], q[0], q[1], q[2]]]))

    def grab():
        return {c: np.asarray(s.get_data("rgb")[0].numpy()[..., :3], np.float32).mean(-1) for c, s in sensors.items()}

    def ncc(a, b):
        a, b = a - a.mean(), b - b.mean()
        return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum() + 1e-9))

    ref = []
    for k in range(n):
        pose(k)
        wait()
        for _ in range(rcfg["settle_updates"] + 4):
            app.update()
        ref.append(grab())
    result = {}
    for updates in (1, 2, 3, 4):
        pose(0)
        for _ in range(8):
            app.update()
        lags = []
        for k in range(1, n):
            pose(k)
            for _ in range(updates):
                app.update()
            img = grab()
            for c in sensors:
                scores = {off: ncc(img[c], ref[k - off][c]) for off in range(-1, 4) if 0 <= k - off < n}
                lags.append(max(scores, key=scores.get))
        hist = {str(o): lags.count(o) for o in sorted(set(lags))}
        result[str(updates)] = {"lag_frames_histogram": hist, "lag_0_fraction": round(lags.count(0) / len(lags), 3)}
        print(f"  updates={updates}: lag histogram {hist}", flush=True)
    report = {"test": "spin in place, all cameras", "step_deg": step_deg, "frames": n - 1, "by_updates_per_frame": result}
    (fdir / "rig_lag_test.json").write_text(json.dumps(report, indent=2) + "\n")
    print("RIG_LAG " + json.dumps(report))


def main() -> None:
    cfg = load_config(args.config)
    fl, rig, rcfg, imu_cfg = cfg["flights"][args.flight], cfg["rig"], dict(cfg["render"]), cfg["imu"]
    env = cfg["environments"][fl["env"]] if "env" in fl else None
    rcfg["frustum_culling"] = False
    fdir = BUILD / "flight" / args.flight
    gt = np.loadtxt(fdir / "groundtruth_200hz.csv", delimiter=",", comments="#")   # t p(3) q_wxyz(4) v(3)
    imu = np.loadtxt(fdir / args.imu_file, delimiter=",", comments="#")
    t_frames = np.arange(0.0, gt[-1, 0], 1.0 / rig["camera_hz"])
    if args.max_frames:
        t_frames = t_frames[: args.max_frames]
    rot_gt = R.from_quat(gt[:, [5, 6, 7, 4]])
    slerp = Slerp(gt[:, 0], rot_gt)
    pos = np.stack([np.interp(t_frames, gt[:, 0], gt[:, 1 + k]) for k in range(3)], axis=1)
    vel = np.stack([np.interp(t_frames, gt[:, 0], gt[:, 8 + k]) for k in range(3)], axis=1)
    rots = slerp(t_frames)

    clip = cfg["camera"]["clip"]
    stereo_cfg = dict(cfg["stereo"])
    if env:
        # Isaac USD environment: static geometry, so a lockstep frame is a fixed number of updates.
        stage = envs.open_env(app, env)
        look = envs.apply_lighting(stage, env, args.lighting)
        clip = env.get("clip", clip)
        stereo_cfg.update(env.get("stereo", {}))
        rcfg["settle_updates"] = 0

        def wait():
            return envs.settle(app, env["settle_updates"])

        def warm():
            return envs.settle(app, env["warmup_updates"])
    else:
        cs.ensure_loaded()
        stage_path = BUILD / "cesium" / f"{fl['site']}_{fl['source']}.usda"
        if fl["source"] in ("google", "osm") and not args.token:
            raise SystemExit("CESIUM_ION_TOKEN is not set; ion tilesets will not stream")
        stage = cs.open_stage(app, stage_path, args.token)
        look = cs.apply_look(stage, rcfg)

        def wait():
            return cs.wait_for_tiles(app, rcfg["tile_timeout_s"])
        warm = wait

    if args.stereo:
        st = stereo_cfg
        intr = {"width": st["width"], "height": st["height"], "fx": st["fx"], "fy": st["fy"],
                "model": "plumb_bob", "d": [0.0] * 5}
        b = st["baseline_m"] / 2
        mounts = {"left": (st["yaw_deg"], st["pitch_deg"], [0.0, b, 0.0]),
                  "right": (st["yaw_deg"], st["pitch_deg"], [0.0, -b, 0.0])}
    else:
        intr = {"width": rig["width"], "height": rig["height"], "fx": rig["fx"], "fy": rig["fy"],
                "model": "equidistant", "d": rig["k"]}
        mounts = {name: (yaw, pitch, [0.0, 0.0, 0.0]) for name, (yaw, pitch) in rig["cameras"].items()}
    W, H = intr["width"], intr["height"]
    UsdGeom.Xform.Define(stage, RIG)
    rig_xf = XformPrim(RIG, reset_xform_op_properties=True)
    extrinsics, offsets, sensors = {}, {}, {}
    for name, (yaw, pitch, t_bc) in mounts.items():
        path = f"{RIG}/{name}"
        if args.stereo:
            # Pinhole: fx = focal / horizontal aperture * width (square pixels).
            ha = 20.955
            cam = RtxCamera(path, attributes={"focalLength": intr["fx"] * ha / W, "horizontalAperture": ha,
                                              "verticalAperture": ha * H / W})
        else:
            cam = RtxCamera(path, schemas=["OmniLensDistortionOpenCvFisheyeAPI"], attributes={
                "omni:lensdistortion:model": "opencvFisheye",
                "omni:lensdistortion:opencvFisheye:fx": float(rig["fx"]),
                "omni:lensdistortion:opencvFisheye:fy": float(rig["fy"]),
                "omni:lensdistortion:opencvFisheye:cx": W / 2,
                "omni:lensdistortion:opencvFisheye:cy": H / 2,
                "omni:lensdistortion:opencvFisheye:imageSize": Gf.Vec2i(W, H),
                **{f"omni:lensdistortion:opencvFisheye:k{i + 1}": float(k) for i, k in enumerate(rig["k"])}})
        UsdGeom.Camera(stage.GetPrimAtPath(path)).GetClippingRangeAttr().Set(Gf.Vec2f(*clip))
        r_bc = body_to_camera(yaw, pitch)
        extrinsics[name], offsets[name] = r_bc, np.asarray(t_bc, float)
        q = R.from_matrix(r_bc @ CV_TO_USD).as_quat()  # x y z w
        cam.set_local_poses(translations=offsets[name][None], orientations=np.array([[q[3], q[0], q[1], q[2]]]))
        sensors[name] = CameraSensor(cam, resolution=(H, W),
                                     annotators=["rgb"] + (["distance_to_camera"] if rig["depth"] else []))
    first = next(iter(mounts))
    cs.set_viewport_camera(f"{RIG}/{first}")
    omni.timeline.get_timeline_interface().play()

    def place_rig(k: int) -> None:
        q = rots[k].as_quat()
        rig_xf.set_world_poses(positions=pos[k][None], orientations=np.array([[q[3], q[0], q[1], q[2]]]))

    if args.lag_test:
        lag_test(sensors, rig_xf, pos[0], rots[0], rcfg, fdir, wait)
        return

    run_name = (f"rig_{args.mode}" + ("_stereo" if args.stereo else "")
                + (f"_{args.lighting}" if args.lighting != "default" else "") + (f"_{args.tag}" if args.tag else ""))
    out = fdir / run_name
    (out / "frames").mkdir(parents=True, exist_ok=True)
    if args.stereo:
        write_stereo(out, intr, extrinsics, offsets, t_frames, pos, rots)
    else:
        write_camchain(out / "camchain.yaml", rig, extrinsics, imu_cfg)
    ros = RosOut(mounts, intr, extrinsics, offsets) if args.ros2 else None

    # Warm the tile cache along the path (as F2 does), then fly. Environments with heavy MDL materials
    # also get one long settle at the start (shader compile + texture streaming), not one per warm-up pose.
    t0 = time.time()
    if env and env.get("first_warmup_updates"):
        place_rig(0)
        envs.settle(app, env["first_warmup_updates"])
    for k in range(0, len(t_frames), rcfg["warmup_stride"]):
        place_rig(k)
        warm()
    warmup_s = time.time() - t0

    timing = {k: [] for k in ("pose", "render", "readback", "publish", "frame")}
    updates, unsettled, imu_i, saved = [], 0, 0, 0
    settle = rig["settle_updates"]
    # Images trail the pose by a fixed number of frames when the renderer is not given enough
    # updates (measured with --lag-test); free-running frames are stamped with the pose they show.
    lags = rig["freerun_lag_frames"]
    lag = 0 if args.mode == "lockstep" else (lags[args.updates - 1] if args.updates <= len(lags) else 0)
    run0 = time.time()
    for k in range(len(t_frames)):
        f0 = time.perf_counter()
        place_rig(k)
        f1 = time.perf_counter()
        if args.mode == "lockstep":
            _, settled, n = wait()
            for _ in range(settle):
                app.update()
            n += settle
            unsettled += not settled
        else:
            for _ in range(args.updates):
                app.update()
            n = args.updates
        updates.append(n)
        f2 = time.perf_counter()
        images = {}
        for name, s in sensors.items():
            rgb, _ = s.get_data("rgb")
            images[name] = None if rgb is None else rgb.numpy()[..., :3].copy()
        f3 = time.perf_counter()
        j = k - lag                      # the pose these images show
        t = t_frames[max(j, 0)]
        if ros and j >= 0:
            while imu_i < len(imu) and imu[imu_i, 0] <= t + 1e-9:
                ros.publish_imu(imu[imu_i])
                ros.publish_clock(imu[imu_i, 0])
                imu_i += 1
            if imu_i == 0 or imu[imu_i - 1, 0] < t - 1e-9:   # frame between IMU samples: move the clock to it
                ros.publish_clock(t)
            ros.publish_pose(t, pos[j], rots[j].as_quat(), vel[j])
            for name, rgb in images.items():
                if rgb is not None:
                    ros.publish_image(name, t, rgb)
        f4 = time.perf_counter()
        for key, v in zip(("pose", "render", "readback", "publish", "frame"), (f1 - f0, f2 - f1, f3 - f2, f4 - f3, f4 - f0)):
            timing[key].append(v)
        if j >= 0 and (args.save_all or (args.save_every and j % args.save_every == 0)):
            for name, rgb in images.items():
                if rgb is not None:
                    Image.fromarray(rgb).save(out / "frames" / f"{j:06d}_{name}.png")
            saved += 1
        if k % 60 == 0:
            print(f"  frame {k}/{len(t_frames)}  updates={n}  frame_ms={1000 * (f4 - f0):.1f}", flush=True)
        if args.max_fps > 0:
            spare = 1.0 / args.max_fps - (time.perf_counter() - f0)
            if spare > 0:
                time.sleep(spare)
    wall = time.time() - run0
    if ros:
        ros.close()

    def stats(v):
        a = 1000 * np.asarray(v)
        return {"mean_ms": round(float(a.mean()), 2), "p50_ms": round(float(np.median(a)), 2),
                "p95_ms": round(float(np.percentile(a, 95)), 2), "max_ms": round(float(a.max()), 2)}

    frame_s = float(np.sum(timing["frame"]))
    report = {
        "flight": args.flight, "mode": args.mode, "ros2": args.ros2, "freerun_updates": args.updates, "frames": int(len(t_frames)),
        "cameras": list(mounts), "resolution": [W, H], "depth": rig["depth"],
        "sim_camera_hz": rig["camera_hz"], "sim_duration_s": round(float(t_frames[-1]), 3),
        "warmup_s": round(warmup_s, 1), "wall_s": round(wall, 2),
        "max_fps_cap": args.max_fps,
        "wall_frames_per_s": round(len(t_frames) / wall, 2),
        "achieved_frames_per_s": round(len(t_frames) / frame_s, 2),
        "achieved_images_per_s": round(len(t_frames) * len(sensors) / frame_s, 2),
        "realtime_factor": round(float(t_frames[-1]) / frame_s, 3),
        "image_lag_frames_compensated": lag,
        "renderer_updates_per_frame": {"mean": round(float(np.mean(updates)), 2), "max": int(np.max(updates))},
        "unsettled_frames": unsettled, "imu_messages": imu_i if ros else 0,
        "timing": {k: stats(v) for k, v in timing.items()},
        "look": look,
        "note": "achieved_* = frames / summed frame time (pose + render + readback + publish), i.e. the uncapped rate; "
                "wall_frames_per_s includes the cap's sleeps and image saving",
    }
    (out / "rates.json").write_text(json.dumps(report, indent=2) + "\n")
    print("RIG_RATES " + json.dumps(report))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()
