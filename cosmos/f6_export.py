"""F6: export the sim render and every usable Cosmos variant as EuRoC-style VIO datasets.

All variants of a clip share the same IMU, ground truth and calibration; only cam0 images differ.
Layout per sequence: build/vio_datasets/<flight>/<clip>/<variant>/mav0/
  cam0/{data/<t_ns>.png, data.csv, sensor.yaml}
  imu0/{data.csv, sensor.yaml}
  state_groundtruth_estimate0/data.csv    (position, attitude, velocity, true IMU biases)

Run: uv run python cosmos/f6_export.py --flight marina_bay_pass [--include-synthetic]
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cosmos.video_io import read_video  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402

T0_NS = 1_700_000_000_000_000_000  # arbitrary epoch so timestamps look like EuRoC's


def ns(t: np.ndarray) -> np.ndarray:
    return np.round(np.asarray(t, dtype=np.float64) * 1e9).astype(np.int64) + T0_NS  # add in int64


def yaml_matrix(m) -> str:
    rows = [", ".join(f"{v:.9f}" for v in r) for r in np.asarray(m)]
    return "[" + ",\n         ".join(rows) + "]"


def write_sequence(dst: Path, images, t_frames, imu, gt, bias, traj: dict, imu_cfg: dict, fps: int) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    cam = dst / "mav0" / "cam0"
    (cam / "data").mkdir(parents=True)
    stamps = ns(t_frames)
    with (cam / "data.csv").open("w") as f:
        f.write("#timestamp [ns],filename\n")
        for st, img in zip(stamps, images):
            Image.fromarray(img).save(cam / "data" / f"{st}.png")
            f.write(f"{st},{st}.png\n")
    I = traj["intrinsics"]
    (cam / "sensor.yaml").write_text(
        "sensor_type: camera\ncomment: Isaac Sim + Cesium render (or Cosmos 3 variant of it), pinhole, no distortion\n"
        f"T_BS:\n  cols: 4\n  rows: 4\n  data: {yaml_matrix(traj['T_body_camera_cv'])}\n"
        f"rate_hz: {fps}\nresolution: [{I['width']}, {I['height']}]\ncamera_model: pinhole\n"
        f"intrinsics: [{I['fx']:.6f}, {I['fy']:.6f}, {I['cx']:.6f}, {I['cy']:.6f}]\n"
        "distortion_model: radial-tangential\ndistortion_coefficients: [0.0, 0.0, 0.0, 0.0]\n")

    imu_dir = dst / "mav0" / "imu0"
    imu_dir.mkdir(parents=True)
    np.savetxt(imu_dir / "data.csv", np.column_stack([ns(imu[:, 0]), imu[:, 1:]]),
               fmt=["%d"] + ["%.9f"] * 6, delimiter=",",
               header="timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                      "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]", comments="#")
    (imu_dir / "sensor.yaml").write_text(
        "sensor_type: imu\ncomment: synthetic IMU from the flight trajectory (body FLU frame)\n"
        "T_BS:\n  cols: 4\n  rows: 4\n  data: " + yaml_matrix(np.eye(4)) + "\n"
        f"rate_hz: {imu_cfg['rate_hz']}\n"
        f"gyroscope_noise_density: {imu_cfg['gyro_noise_density']}\n"
        f"gyroscope_random_walk: {imu_cfg['gyro_random_walk']}\n"
        f"accelerometer_noise_density: {imu_cfg['accel_noise_density']}\n"
        f"accelerometer_random_walk: {imu_cfg['accel_random_walk']}\n")

    gt_dir = dst / "mav0" / "state_groundtruth_estimate0"
    gt_dir.mkdir(parents=True)
    np.savetxt(gt_dir / "data.csv", np.column_stack([ns(gt[:, 0]), gt[:, 1:11], bias[:, 1:4], bias[:, 4:7]]),
               fmt=["%d"] + ["%.9f"] * 16, delimiter=",",
               header="timestamp,p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [],"
                      "v_RS_R_x [m s^-1],v_RS_R_y [m s^-1],v_RS_R_z [m s^-1],b_w_RS_S_x [rad s^-1],"
                      "b_w_RS_S_y [rad s^-1],b_w_RS_S_z [rad s^-1],b_a_RS_S_x [m s^-2],b_a_RS_S_y [m s^-2],"
                      "b_a_RS_S_z [m s^-2]", comments="#")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    ap.add_argument("--include-synthetic", action="store_true", help="also export dry-run outputs (tests only)")
    args = ap.parse_args()
    cfg = load_config(args.config)
    fdir = BUILD / "flight" / args.flight
    traj = json.loads((fdir / "trajectory.json").read_text())
    if traj["camera_mount"] != "fixed":
        raise SystemExit("EuRoC export needs a fixed camera mount (constant T_BS); gimbal flights are not exported")
    imu = np.loadtxt(fdir / "imu.csv", delimiter=",")
    gt = np.loadtxt(fdir / "groundtruth_200hz.csv", delimiter=",")
    bias = np.loadtxt(fdir / "imu_bias.csv", delimiter=",")
    root = BUILD / "cosmos" / args.flight
    qa = json.loads((root / "qa" / "qa.json").read_text())["variants"]
    out_root = BUILD / "vio_datasets" / args.flight

    exported = []
    for clip_dir in sorted(p for p in (root / "clips").iterdir() if p.is_dir()):
        clip = json.loads((clip_dir / "clip.json").read_text())
        s, e = clip["frames"]
        t_frames = np.asarray(clip["poses_tum"])[:, 0]
        # IMU / GT cover the clip's frame span plus one IMU sample either side, so every frame is
        # bracketed by IMU data (VIO pre-integration needs that).
        dt = 1.0 / cfg["imu"]["rate_hz"]
        sel = (imu[:, 0] >= t_frames[0] - dt - 1e-9) & (imu[:, 0] <= t_frames[-1] + dt + 1e-9)
        sims = [np.asarray(Image.open(fdir / "render" / "frames" / f"{i:06d}_rgb.png").convert("RGB"))
                for i in range(s, e)]
        seqs = [("sim", sims)]
        for name, r in sorted(qa.items()):
            if not name.startswith(clip_dir.name + "/") or not r["passed"]:
                continue
            if r["synthetic"] and not args.include_synthetic:
                continue
            frames = read_video(root / "variants" / f"{name}.mp4", size=(clip["width"], clip["height"]))
            seqs.append((name.split("/")[1], frames))
        for vname, images in seqs:
            n = min(len(images), len(t_frames))
            dst = out_root / clip_dir.name / vname
            write_sequence(dst, images[:n], t_frames[:n], imu[sel], gt[sel], bias[sel], traj, cfg["imu"], clip["fps"])
            exported.append(str(dst.relative_to(BUILD)))
            print(f"  {clip_dir.name}/{vname}: {n} frames, {int(sel.sum())} IMU samples -> {dst}")
    (out_root / "index.json").write_text(json.dumps({"flight": args.flight, "sequences": exported}, indent=2) + "\n")
    print(f"exported {len(exported)} sequences -> {out_root}")


if __name__ == "__main__":
    main()
