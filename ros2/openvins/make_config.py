"""Write an OpenVINS config for the Isaac fisheye rig from config/flight.yaml.

Output: build/openvins/<name>/{estimator_config.yaml, kalibr_imucam_chain.yaml, kalibr_imu_chain.yaml, mask*.png}
Cameras are all-mono (max_cameras = N, use_stereo false), OpenCV fisheye = Kalibr "equidistant".
Masks keep features inside theta_max of the optical axis: past ~80 deg the undistorted
coordinates blow up, and the corners outside the 180 deg circle are black.

Run: uv run python ros2/openvins/make_config.py [--cameras cam0_front cam1_left ...] [--name rig4]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from flight.f1_trajectory import body_to_camera_cv  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

ESTIMATOR = """%YAML:1.0
verbosity: "INFO"
use_fej: true
integration: "rk4"
use_stereo: false
max_cameras: {n}
calib_cam_extrinsics: false          # simulator calibration is exact
calib_cam_intrinsics: false
calib_cam_timeoffset: false
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false
max_clones: 15                       # longer window = more baseline to far features
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1
gravity_mag: 9.81
feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"
# Feature triangulation limits: OpenVINS defaults (60 m, 40x baseline) are for indoor rigs; from
# 100+ m altitude every feature is farther than that and would be thrown away.
fi_max_dist: {fi_max_dist}
fi_max_baseline: 400.0
fi_max_cond_number: 1000000.0
try_zupt: true                       # with ZUPT on, static init fires during the start hover (no jerk needed)
zupt_chi2_multipler: 0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5
zupt_only_at_beginning: true
init_window_time: 2.0
init_imu_thresh: {init_thresh}       # accel std that counts as "started moving": 2x the vibration
init_max_disparity: 2.0              # px; at 100 m altitude a slow take-off moves features very little
init_max_features: 150
init_dyn_use: true                   # fall back to dynamic init if the take-off jerk is too gentle
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 6
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0
init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12
init_dyn_bias_g: [0.0, 0.0, 0.0]
init_dyn_bias_a: [0.0, 0.0, 0.0]
record_timing_information: false
record_timing_filepath: "/out/ov_timing.txt"
save_total_state: true
filepath_est: "/out/ov_estimate.txt"
filepath_std: "/out/ov_estimate_std.txt"
filepath_gt: "/out/ov_groundtruth.txt"
use_klt: true
num_pts: 200
fast_threshold: 10                   # bay water is low-texture
grid_x: 10                           # small cells: OpenVINS takes the strongest corners per cell
grid_y: 8                            # before masking, and on the fisheye rim those are all masked
min_px_dist: 10
knn_ratio: 0.70
track_frequency: 31.0
downsample_cameras: false
num_opencv_threads: 4
histogram_method: "HISTOGRAM"
use_aruco: false
num_aruco: 1024
downsize_aruco: true
up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1
use_mask: true
{masks}
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
"""

IMU = """%YAML:1.0
imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: {and_}   # sensor {and0} + motor vibration
  accelerometer_random_walk: {arw}
  gyroscope_noise_density: {gnd}        # sensor {gnd0} + motor vibration
  gyroscope_random_walk: {grw}
  rostopic: /imu/data
  time_offset: 0.0
  update_rate: {rate}
  model: "kalibr"
  Tw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  R_IMUtoGYRO: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  Ta: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  R_IMUtoACC: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  Tg: [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
"""


def mat_rows(m: np.ndarray) -> str:
    return "\n".join("    - [" + ", ".join(f"{v:.9f}" for v in r) + "]" for r in m)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    ap.add_argument("--cameras", nargs="*", help="subset of rig.cameras (default: all)")
    ap.add_argument("--name", default=None, help="output folder name (default rig<N>)")
    ap.add_argument("--theta-max-deg", type=float, default=80.0)
    ap.add_argument("--flight", default="marina_bay_loop", help="flight whose IMU sets the noise levels")
    ap.add_argument("--fi-max-dist", type=float, default=1500.0, help="max feature distance (m); 1500 suits Cesium at 100+ m")
    ap.add_argument("--noise-margin", type=float, default=2.0, help="factor on the IMU noise densities (ablation A2: 1)")
    args = ap.parse_args()
    cfg = load_config(args.config)
    rig, imu = cfg["rig"], cfg["imu"]
    cams = args.cameras or list(rig["cameras"])
    out = BUILD / "openvins" / (args.name or f"rig{len(cams)}")
    out.mkdir(parents=True, exist_ok=True)
    W, H = rig["width"], rig["height"]

    # Mask: 255 = do not use. Equidistant radius r = f * theta.
    r_max = rig["fx"] * np.radians(args.theta_max_deg)
    mask = Image.new("L", (W, H), 255)
    ImageDraw.Draw(mask).ellipse([W / 2 - r_max, H / 2 - r_max, W / 2 + r_max, H / 2 + r_max], fill=0)
    chain = ["%YAML:1.0", ""]
    for i, name in enumerate(cams):
        yaw, pitch = rig["cameras"][name]
        r_bc = R.from_euler("z", yaw, degrees=True).as_matrix() @ body_to_camera_cv(pitch)
        t_ic = np.eye(4)
        t_ic[:3, :3] = r_bc                     # camera -> IMU (body FLU); cameras sit at the IMU origin
        mask.save(out / f"mask{i}.png")
        chain += [f"cam{i}:", "  T_imu_cam:", mat_rows(t_ic), "  cam_overlaps: []", "  camera_model: pinhole",
                  f"  distortion_coeffs: [{', '.join(str(float(k)) for k in rig['k'])}]",
                  "  distortion_model: equidistant",
                  f"  intrinsics: [{rig['fx']}, {rig['fy']}, {W / 2}, {H / 2}]",
                  f"  resolution: [{W}, {H}]", f"  rostopic: /{name}/image_raw", ""]
    (out / "kalibr_imucam_chain.yaml").write_text("\n".join(chain))

    # Vibration is extra white-ish noise at the IMU rate: fold its measured per-axis RMS into the
    # noise density (sigma_discrete = density * sqrt(rate)), then double the result for margin.
    fdir = BUILD / "flight" / args.flight
    noisy = np.loadtxt(fdir / "imu.csv", delimiter=",")
    clean = np.loadtxt(fdir / "imu_clean.csv", delimiter=",")
    bias = np.loadtxt(fdir / "imu_bias.csv", delimiter=",")
    resid = noisy[:, 1:] - clean[:, 1:] - bias[:, 1:]
    gyr_vib, acc_vib = float(resid[:, :3].std(0).max()), float(resid[:, 3:].std(0).max())
    acc_vec_std = float(np.sqrt((resid[:, 3:].std(0) ** 2).sum()))
    rate = imu["rate_hz"]
    k = args.noise_margin
    and_ = k * np.hypot(imu["accel_noise_density"], acc_vib / np.sqrt(rate))
    gnd = k * np.hypot(imu["gyro_noise_density"], gyr_vib / np.sqrt(rate))
    (out / "kalibr_imu_chain.yaml").write_text(IMU.format(
        and_=f"{and_:.4e}", and0=imu["accel_noise_density"], arw=f"{k * imu['accel_random_walk']:.4e}",
        gnd=f"{gnd:.4e}", gnd0=imu["gyro_noise_density"], grw=f"{k * imu['gyro_random_walk']:.4e}", rate=float(rate)))
    masks = "\n".join(f'mask{i}: "mask{i}.png"' for i in range(len(cams)))
    (out / "estimator_config.yaml").write_text(ESTIMATOR.format(n=len(cams), masks=masks, fi_max_dist=args.fi_max_dist,
                                                                init_thresh=round(max(1.0, 2 * acc_vec_std), 2)))
    print(f"OpenVINS config for {cams} -> {out}")


if __name__ == "__main__":
    main()
