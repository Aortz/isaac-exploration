"""Gate G2: does the RTX fisheye lens work on this environment? (Matters for NuRec ParticleField splats.)

At a few poses, renders the rig fisheye (OpenCV equidistant) and a 90° pinhole from the same spot and
direction, warps the pinhole into the fisheye with the lens model, and reports the NCC over the
overlap. If the renderer ignored the lens for this geometry, the fisheye would show a plain pinhole
view and the NCC drops well below the threshold.

Run: $ISAAC_ROOT/python.sh testplan/g2_lens.py --env nurec_wormhole --poses third_party/nurec/nova_carter-wormhole/training_trajectory_poses.tum
Writes build/envs/<env>/g2.json and g2_<k>.png (fisheye | warped pinhole)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--env", required=True)
ap.add_argument("--poses", required=True, help="TUM file (t x y z qx qy qz qw) of capture poses; samples a few")
ap.add_argument("--height", type=float, default=0.8, help="camera height above the pose z (m)")
ap.add_argument("--samples", type=int, default=4)
ap.add_argument("--debug-light", action="store_true", help="add a dome light (is anything rendering at all?)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

sys.path.insert(0, str(PROJECT_ROOT))
from pipeline.common.config import load_config as _load  # noqa: E402

_kit = _load(PROJECT_ROOT / "config" / "flight.yaml")["environments"][args.env].get("kit_args", [])
app = SimulationApp({"headless": True, "extra_args": _kit + ["--/crashreporter/url=", "--/crashreporter/skipOldDumpUpload=true"]})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from flight.f1_trajectory import CV_TO_USD, body_to_camera_cv  # noqa: E402
from pipeline.common import environments as envs  # noqa: E402
from pipeline.common.config import BUILD, load_config  # noqa: E402


def main() -> None:
    cfg = load_config(PROJECT_ROOT / "config" / "flight.yaml")
    env, rig = cfg["environments"][args.env], cfg["rig"]
    th = load_config(PROJECT_ROOT / "config" / "tests.yaml")["thresholds"]["gates"]["g2_ncc"]
    out = BUILD / "envs" / args.env
    out.mkdir(parents=True, exist_ok=True)
    stage = envs.open_env(app, env)
    if args.debug_light:
        from pxr import Usd as _U
        for _p in _U.PrimRange(stage.GetPrimAtPath("/World")):
            if _p.GetPath().pathElementCount <= 4:
                print("PRIM", _p.GetPath(), _p.GetTypeName(), _p.IsActive(), flush=True)
        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage, "/World/G2Dome").CreateIntensityAttr(1000.0)
    W, H, f = rig["width"], rig["height"], rig["fx"]
    fish = RtxCamera("/World/G2Fish", schemas=["OmniLensDistortionOpenCvFisheyeAPI"], attributes={
        "omni:lensdistortion:model": "opencvFisheye", "omni:lensdistortion:opencvFisheye:fx": f,
        "omni:lensdistortion:opencvFisheye:fy": f, "omni:lensdistortion:opencvFisheye:cx": W / 2,
        "omni:lensdistortion:opencvFisheye:cy": H / 2, "omni:lensdistortion:opencvFisheye:imageSize": Gf.Vec2i(W, H),
        "clippingRange": Gf.Vec2f(*env["clip"])})
    fp = 320.0
    pin = RtxCamera("/World/G2Pin", attributes={"focalLength": fp * 20.955 / W, "horizontalAperture": 20.955,
                                                "verticalAperture": 20.955 * H / W, "clippingRange": Gf.Vec2f(*env["clip"])})
    s_f = CameraSensor(fish, resolution=(H, W), annotators=["rgb"])
    s_p = CameraSensor(pin, resolution=(H, W), annotators=["rgb"])
    omni.timeline.get_timeline_interface().play()

    poses = np.loadtxt(args.poses)
    if args.debug_light:
        from pxr import Usd as _U, UsdGeom as _G
        cp = stage.GetPrimAtPath("/World/Env/rig_trajectories/sensor_rig_0/front_stereo_camera_left__0")
        rp = stage.GetPrimAtPath("/World/Env/rig_trajectories/sensor_rig_0")
        import carb
        _st = carb.settings.get_settings()
        print("SETTINGS", {k: _st.get(k) for k in ("/UJITSO/geometry", "/UJITSO/enabled", "/rtx/rendermode", "/renderer/active")}, flush=True)
        print("RIGXF", rp.GetAttribute("xformOp:transform").GetTimeSamples()[:3] if rp.GetAttribute("xformOp:transform") else
              [a.GetName() for a in rp.GetAttributes()][:10], flush=True)
        for tc in (_U.TimeCode.Default(), _U.TimeCode(0)):
            m = _G.Xformable(cp).ComputeLocalToWorldTransform(tc)
            print("CAMXF", tc, m.ExtractTranslation(), flush=True)
        m = _G.Xformable(cp).ComputeLocalToWorldTransform(_U.TimeCode.Default())
        t = np.array(m.ExtractTranslation()); rot = np.array(m.ExtractRotationMatrix()).T   # USD row-vector -> column
        # Render one view from the scene's own camera pose (USD camera axes), then exit.
        q = R.from_matrix(rot).as_quat()
        for cam in (fish, pin):
            cam.set_world_poses(positions=t[None], orientations=np.array([[q[3], q[0], q[1], q[2]]]))
        for _ in range(80):
            app.update()
        b = s_p.get_data("rgb")[0].numpy()[..., :3]
        Image.fromarray(b).save(out / "g2_scenecam.png")
        print("SCENECAM mean", float(b.mean()), flush=True)
        return
    idx = np.linspace(len(poses) * 0.1, len(poses) * 0.9, args.samples).astype(int)
    r_bc = body_to_camera_cv(0.0)                    # forward, level, in the pose's body frame (x forward)
    results = []
    for n, i in enumerate(idx):
        p = poses[i, 1:4] + np.array([0.0, 0.0, args.height])
        r_wc = R.from_quat(poses[i, 4:8]).as_matrix() @ r_bc
        q = R.from_matrix(r_wc @ CV_TO_USD).as_quat()
        for cam in (fish, pin):
            cam.set_world_poses(positions=p[None], orientations=np.array([[q[3], q[0], q[1], q[2]]]))
        for _ in range(env["warmup_updates"] + env["settle_updates"] + (60 if n == 0 else 0)):
            app.update()
        a = s_f.get_data("rgb")[0].numpy()[..., :3].astype(np.float32)
        b = s_p.get_data("rgb")[0].numpy()[..., :3].astype(np.float32)
        # Fisheye pixel -> ray (equidistant) -> pinhole pixel.
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        dx, dy = (u - W / 2) / f, (v - H / 2) / f
        theta = np.hypot(dx, dy)
        ok = (theta > 1e-6) & (theta < np.radians(80))
        s = np.where(theta > 1e-6, np.tan(theta) / np.maximum(theta, 1e-6), 1.0)
        pu, pv = fp * dx * s + W / 2, fp * dy * s + H / 2
        ok &= (pu >= 0) & (pu < W - 1) & (pv >= 0) & (pv < H - 1)
        warped = np.zeros_like(a)
        warped[ok] = b[pv[ok].astype(int), pu[ok].astype(int)]
        ga, gb = a[ok].mean(1), warped[ok].mean(1)
        ga, gb = ga - ga.mean(), gb - gb.mean()
        ncc = float((ga * gb).sum() / np.sqrt((ga ** 2).sum() * (gb ** 2).sum() + 1e-9))
        # Also NCC if the "fisheye" were really a pinhole with focal f (lens ignored), for contrast.
        results.append({"pose_index": int(i), "ncc": round(ncc, 4), "overlap_px": int(ok.sum()),
                        "fisheye_mean": round(float(a.mean()), 1), "pinhole_mean": round(float(b.mean()), 1)})
        Image.fromarray(np.hstack([a, warped]).astype(np.uint8)).save(out / f"g2_{n}.png")
        print(f"  pose {i}: NCC {ncc:.3f}", flush=True)
    med = float(np.median([r["ncc"] for r in results]))
    report = {"gate": "g2", "env": args.env, "median_ncc": med, "pass": med >= th, "threshold": th, "poses": results}
    (out / "g2.json").write_text(json.dumps(report, indent=2) + "\n")
    print("G2 " + json.dumps(report))


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
