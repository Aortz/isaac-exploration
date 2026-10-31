"""Turn a Pegasus flight log into F1-format ground truth, camera poses and IMU.

Input: build/flight/<flight>/pegasus_state_1000hz.csv (flight/pegasus_fly.py) and reference_1000hz.csv.
Output (overwrites the kinematic F1 files, so rig_fisheye.py / OpenVINS use the flown trajectory):
  groundtruth_200hz.csv, imu.csv, imu_clean.csv, imu_bias.csv, camera_poses_tum.txt,
  camera_poses_usd.csv, trajectory.json (+ tracking error against the reference)

IMU: specific force from the physics velocity, f = R_wb^T (dv/dt + g z), and the body rate from
successive physics attitudes. (PhysX's own angular velocity for the Iris articulation is biased by a
few mrad/s against its attitude, which integrates to kilometres of drift; the attitude difference is
exactly consistent with the ground truth, which is what VIO evaluation needs.) Both are averaged over each IMU period, centred on the sample time (an integrating sensor without
the half-period delay), then get the same motor vibration and noise model as F1.

Run: uv run python flight/pegasus_post.py --flight marina_bay_loop_pegasus
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flight.f1_trajectory import CV_TO_USD, G, add_noise, body_to_camera_cv, integrate_imu, motor_vibration  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402


def centred_mean(x: np.ndarray, n: int) -> np.ndarray:
    """Mean over n samples centred on each sample (n odd), edges padded with the end values."""
    pad = np.pad(x, ((n // 2, n // 2), (0, 0)), mode="edge")
    c = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), pad]), axis=0)
    return (c[n:] - c[:-n]) / n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    ap.add_argument("--extra-seeds", type=int, nargs="*", default=[], help="also write imu_seed<N>.csv per seed")
    args = ap.parse_args()
    cfg = load_config(args.config)
    cam, imu, hz = cfg["camera"], cfg["imu"], int(cfg["pegasus"]["physics_hz"])
    fdir = BUILD / "flight" / args.flight
    log = np.loadtxt(fdir / "pegasus_state_1000hz.csv", delimiter=",", comments="#")
    ref = np.loadtxt(fdir / "reference_1000hz.csv", delimiter=",", comments="#")
    t, p, v = log[:, 0], log[:, 1:4], log[:, 8:11]
    rot = R.from_quat(log[:, 4:8])
    # Mean body rate over each physics step (same convention as the backward-difference acceleration).
    w = np.vstack([np.zeros((1, 3)), (rot[:-1].inv() * rot[1:]).as_rotvec() * hz])
    w[0] = w[1]
    r_wb = rot.as_matrix()
    step = hz // imu["rate_hz"]
    if step % 2 == 0:
        raise SystemExit("physics_hz / imu.rate_hz must be odd for a centred average")

    # Physics velocity is the end-of-step value, so the backward difference is the step's mean acceleration.
    a = np.vstack([np.zeros((1, 3)), np.diff(v, axis=0) * hz])
    a[0] = a[1]
    f_dense = np.einsum("nji,nj->ni", r_wb, a + np.array([0.0, 0.0, G]))
    gyro_d, acc_d = centred_mean(w, step), centred_mean(f_dense, step)
    idx = np.arange(0, len(t), step)
    ti, gyro, acc = t[idx], gyro_d[idx], acc_d[idx]

    def noisy(seed: int):
        rng = np.random.default_rng(seed)
        g, a_ = gyro, acc
        bg, ba = np.zeros_like(gyro), np.zeros_like(acc)
        if imu.get("enabled", True):
            if imu.get("vibration"):
                gv, av = motor_vibration(t, f_dense, imu["vibration"], rng)
                g, a_ = gyro + centred_mean(gv, step)[idx], acc + centred_mean(av, step)[idx]
            g, bg = add_noise(g, imu["rate_hz"], imu["gyro_noise_density"], imu["gyro_random_walk"], rng)
            a_, ba = add_noise(a_, imu["rate_hz"], imu["accel_noise_density"], imu["accel_random_walk"], rng)
        return g, a_, bg, ba

    gyro_n, acc_n, b_g, b_a = noisy(imu.get("seed", 0))

    q_wb = rot[idx].as_quat()
    np.savetxt(fdir / "groundtruth_200hz.csv", np.column_stack([ti, p[idx], q_wb[:, [3, 0, 1, 2]], v[idx]]),
               fmt="%.9f", delimiter=",", header="t,px,py,pz,qw,qx,qy,qz,vx,vy,vz  (body FLU in world ENU, Pegasus)")
    hdr = "t,wx,wy,wz,ax,ay,az  (body FLU, rad/s, m/s^2)"
    np.savetxt(fdir / "imu.csv", np.column_stack([ti, gyro_n, acc_n]), fmt="%.9f", delimiter=",", header=hdr)
    np.savetxt(fdir / "imu_clean.csv", np.column_stack([ti, gyro, acc]), fmt="%.9f", delimiter=",", header=hdr)
    np.savetxt(fdir / "imu_bias.csv", np.column_stack([ti, b_g, b_a]), fmt="%.9f", delimiter=",",
               header="t,bgx,bgy,bgz,bax,bay,baz  (true IMU biases)")
    # Extra noise realisations of the same flight, for repeat VIO runs (testplan: imu_seed<N>.csv).
    for seed in args.extra_seeds:
        g, a_, bg, ba = noisy(seed)
        np.savetxt(fdir / f"imu_seed{seed}.csv", np.column_stack([ti, g, a_]), fmt="%.9f", delimiter=",", header=hdr)
        np.savetxt(fdir / f"imu_bias_seed{seed}.csv", np.column_stack([ti, bg, ba]), fmt="%.9f", delimiter=",",
                   header="t,bgx,bgy,bgz,bax,bay,baz  (true IMU biases)")

    # Pinhole F1 camera poses (hard mount only: the gimbal option has no physical model here).
    r_bc = body_to_camera_cv(cam["mount_pitch_deg"])
    tf = np.arange(0.0, t[-1], 1.0 / cam["fps"])
    pf = np.stack([np.interp(tf, t, p[:, k]) for k in range(3)], axis=1)
    r_wc = Slerp(t, rot)(tf).as_matrix() @ r_bc
    q_cv, q_usd = R.from_matrix(r_wc).as_quat(), R.from_matrix(r_wc @ CV_TO_USD).as_quat()
    np.savetxt(fdir / "camera_poses_tum.txt", np.column_stack([tf, pf, q_cv]),
               fmt="%.9f", header="t x y z qx qy qz qw  (OpenCV optical frame, world = local ENU)")
    np.savetxt(fdir / "camera_poses_usd.csv", np.column_stack([np.arange(len(tf)), tf, pf, q_usd[:, [3, 0, 1, 2]]]),
               fmt=["%d"] + ["%.9f"] * 8, delimiter=",", header="frame,t,x,y,z,qw,qx,qy,qz  (USD camera frame)")

    # Tracking against the reference, and the clean IMU integrating back to the flown path.
    p_ref = np.stack([np.interp(t, ref[:, 0], ref[:, 1 + k]) for k in range(3)], axis=1)
    e = np.linalg.norm(p - p_ref, axis=1)
    p_int = integrate_imu(ti, gyro, acc, p[0], v[0], r_wb[0])
    drift = float(np.linalg.norm(p_int - p[idx], axis=1).max())
    # Whole-flight drift grows with t^2 from tiny attitude errors (0.02 deg of coning loss under PX4's
    # fast attitude corrections already gives ~20 m over 100 s), so the data check restarts the
    # integration from ground truth every 10 s and reports the worst 10 s window.
    win, drift10 = int(10 * imu["rate_hz"]), 0.0
    for a in range(0, len(ti) - 1, win):
        b = min(a + win + 1, len(ti))
        pw = integrate_imu(ti[a:b], gyro[a:b], acc[a:b], p[idx][a], v[idx][a], r_wb[idx][a])
        drift10 = max(drift10, float(np.linalg.norm(pw - p[idx][a:b], axis=1).max()))
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    tilt = np.degrees(np.arccos(np.clip(r_wb[:, 2, 2], -1, 1)))
    meta = json.loads((fdir / "trajectory.json").read_text())
    meta.update({
        "dynamics": "pegasus", "pegasus": json.loads((fdir / "pegasus.json").read_text()),
        "duration_s": float(t[-1]), "frames": int(len(tf)), "path_length_m": float(seg.sum()),
        "max_speed_mps": float(np.linalg.norm(v, axis=1).max()), "max_tilt_deg": float(tilt.max()),
        "tracking": {"pos_err_rms_m": float(np.sqrt((e ** 2).mean())), "pos_err_max_m": float(e.max()),
                     "pos_err_final_m": float(e[-1]),
                     "rotor_ref_rad_s": [float(log[:, 14:18].min()), float(log[:, 14:18].max())]},
        "checks": {"imu_integration_drift_m": drift, "imu_drift_10s_m": drift10},
    })
    (fdir / "trajectory.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps({k: meta[k] for k in ("duration_s", "frames", "path_length_m", "max_speed_mps",
                                           "max_tilt_deg", "tracking", "checks")}))


if __name__ == "__main__":
    main()
