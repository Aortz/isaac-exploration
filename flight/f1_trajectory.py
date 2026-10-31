"""F1: turn waypoints into a smooth quadrotor flight with ground-truth camera poses and IMU.

Position is a quintic spline through the waypoints (zero velocity and acceleration at both ends).
Attitude follows quadrotor differential flatness: body z is the thrust direction (a + g z),
heading follows the velocity (or looks at a target). The camera is hard-mounted (tilts with the
body) or on a yaw-only gimbal.

Frames: world = local ENU at the Cesium origin. Body = FLU (x forward, y left, z up).
Camera poses are written in the OpenCV optical frame (x right, y down, z forward) for VIO tools,
and in the USD camera frame (x right, y up, looks along -z) for Isaac.

Run: uv run python flight/f1_trajectory.py --flight marina_bay_pass
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, export_json, load_config  # noqa: E402

G = 9.81
DENSE_HZ = 1000
CV_TO_USD = np.diag([1.0, -1.0, -1.0])  # optical (y down, z fwd) -> USD camera (y up, -z fwd)


def body_to_camera_cv(pitch_deg: float) -> np.ndarray:
    """Rotation of the OpenCV camera frame expressed in the body (FLU) frame."""
    th = np.radians(pitch_deg)
    z = np.array([np.cos(th), 0.0, -np.sin(th)])   # optical axis: forward, pitched down
    x = np.array([0.0, -1.0, 0.0])                 # image right = body right = -y
    y = np.cross(z, x)                             # image down
    return np.column_stack([x, y, z])


def headings(t: np.ndarray, p: np.ndarray, v: np.ndarray, mode: str, target, first_dir: np.ndarray,
             last_dir: np.ndarray, duration: float) -> np.ndarray:
    """Heading (yaw from +x/east, CCW) per sample, continuous and smooth.

    velocity mode: direction of v_xy plus a small bias that blends from the first to the last
    segment direction, so the heading is defined at the start/end (where v = 0) without jumps.
    """
    if mode == "look_at":
        d = np.asarray(target[:2]) - p[:, :2]
        return np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    s = (t / duration)[:, None]
    u0 = first_dir[:2] / np.linalg.norm(first_dir[:2])
    u1 = last_dir[:2] / np.linalg.norm(last_dir[:2])
    h = v[:, :2] + 0.5 * ((1 - s) * u0 + s * u1)
    return np.unwrap(np.arctan2(h[:, 1], h[:, 0]))


def flat_attitude(a: np.ndarray, psi: np.ndarray) -> np.ndarray:
    """Body-to-world rotations from acceleration and heading (quadrotor flatness)."""
    zb = a + np.array([0.0, 0.0, G])
    zb /= np.linalg.norm(zb, axis=1, keepdims=True)
    xc = np.column_stack([np.cos(psi), np.sin(psi), np.zeros_like(psi)])
    yb = np.cross(zb, xc)
    yb /= np.linalg.norm(yb, axis=1, keepdims=True)
    xb = np.cross(yb, zb)
    return np.stack([xb, yb, zb], axis=2)


def body_rates(rot: np.ndarray, dt: float) -> np.ndarray:
    """Angular velocity in the body frame by central differences of the rotation sequence."""
    r = R.from_matrix(rot)
    w = np.zeros((len(rot), 3))
    rel = (r[:-2].inv() * r[2:]).as_rotvec() / (2 * dt)
    w[1:-1] = rel
    w[0], w[-1] = w[1], w[-2]
    return w


def add_noise(clean: np.ndarray, rate: float, dens: float, rw: float, rng) -> tuple[np.ndarray, np.ndarray]:
    """White noise + random-walk bias. Returns (noisy signal, true bias)."""
    dt = 1.0 / rate
    white = rng.normal(0.0, dens * np.sqrt(rate), clean.shape)
    bias = np.cumsum(rng.normal(0.0, rw * np.sqrt(dt), clean.shape), axis=0)
    return clean + white + bias, bias


def motor_vibration(td: np.ndarray, f_dense: np.ndarray, vcfg: dict, rng) -> tuple[np.ndarray, np.ndarray]:
    """Frame vibration from four rotors, at the dense rate. Returns (gyro_vib, accel_vib) in body axes.

    Rotor speed follows thrust (omega^2 ~ |specific force|), so the vibration frequency and amplitude
    change as the vehicle accelerates and brakes. Each rotor runs slightly off the others (beating),
    with tones at the shaft rate and blade-pass rate, plus broadband frame noise.
    """
    from scipy.signal import butter, sosfilt

    dt = td[1] - td[0]
    load = np.linalg.norm(f_dense, axis=1) / G                       # 1.0 at hover
    acc_amp = np.asarray(vcfg["accel_amp_mps2"], float) * load[:, None]
    gyro_amp = np.asarray(vcfg["gyro_amp_rads"], float) * load[:, None]
    acc, gyro = np.zeros_like(f_dense), np.zeros_like(f_dense)
    for _ in range(4):
        freq = vcfg["rotor_hz_hover"] * np.sqrt(load) * (1 + rng.uniform(-0.03, 0.03))
        phase = 2 * np.pi * np.cumsum(freq) * dt
        for harmonic, weight in ((1, 1.0), (vcfg["blades"], 0.5)):
            ph = rng.uniform(0, 2 * np.pi, 3)
            tone = np.sin(harmonic * phase[:, None] + ph) * weight / 2   # four rotors, each half amplitude
            acc += acc_amp * tone
            gyro += gyro_amp * tone
    sos = butter(2, [30.0, 400.0], btype="bandpass", fs=1.0 / dt, output="sos")
    for out, rms in ((acc, vcfg["broadband_accel_mps2"]), (gyro, vcfg["broadband_gyro_rads"])):
        band = sosfilt(sos, rng.normal(0.0, 1.0, out.shape), axis=0)
        out += band / band.std(axis=0) * rms * load[:, None]
    return gyro, acc


def integrate_imu(t, gyro, acc, p0, v0, r0) -> np.ndarray:
    """Strapdown integration of a clean IMU (used only to check consistency)."""
    p, v, rot = p0.copy(), v0.copy(), R.from_matrix(r0)
    out = [p.copy()]
    gvec = np.array([0.0, 0.0, -G])
    for i in range(len(t) - 1):
        dt = t[i + 1] - t[i]
        r_next = rot * R.from_rotvec(0.5 * (gyro[i] + gyro[i + 1]) * dt)
        a0 = rot.apply(acc[i]) + gvec
        a1 = r_next.apply(acc[i + 1]) + gvec
        p = p + v * dt + (2 * a0 + a1) / 6.0 * dt * dt
        v = v + 0.5 * (a0 + a1) * dt
        rot = r_next
        out.append(p.copy())
    return np.array(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    args = ap.parse_args()
    cfg = load_config(args.config)
    export_json(args.config)  # Isaac's Python reads the JSON copy
    fl = cfg["flights"][args.flight]
    cam, imu = cfg["camera"], cfg["imu"]

    wp = np.asarray(fl["waypoints"], dtype=float)
    seg = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    # Start and end at rest, so allow ~25 % more time than cruising the whole path at speed.
    knots = np.concatenate([[0.0], np.cumsum(seg)]) / fl["speed_mps"] * 1.25
    spline = make_interp_spline(knots, wp, k=5, bc_type=([(1, np.zeros(3)), (2, np.zeros(3))],
                                                         [(1, np.zeros(3)), (2, np.zeros(3))]))
    moving = knots[-1]
    # Optional hover before take-off and after arrival (VIO static initialisation needs a still start).
    hover0, hover1 = float(fl.get("hover_start_s", 0.0)), float(fl.get("hover_end_s", 0.0))
    duration = hover0 + moving + hover1

    mode, target = fl.get("yaw_mode", "velocity"), fl.get("look_at")
    r_bc = body_to_camera_cv(cam["mount_pitch_deg"])

    def state(t):
        ts = np.clip(t - hover0, 0.0, moving)   # spline time; ends have zero velocity and acceleration
        p, v, a = spline(ts), spline(ts, 1), spline(ts, 2)
        psi = headings(ts, p, v, mode, target, wp[1] - wp[0], wp[-1] - wp[-2], moving)
        r_wb = flat_attitude(a, psi)
        if cam["mount"] == "gimbal":
            r_wc = np.einsum("nij,jk->nik", R.from_euler("z", psi).as_matrix(), r_bc)
        else:
            r_wc = np.einsum("nij,jk->nik", r_wb, r_bc)
        return p, v, a, r_wb, r_wc

    td = np.arange(0.0, duration, 1.0 / DENSE_HZ)
    p, v, a, r_wb, _ = state(td)

    # IMU at rate_hz, sampled from the dense signals.
    step = DENSE_HZ // imu["rate_hz"]
    w_dense = body_rates(r_wb, 1.0 / DENSE_HZ)
    f_dense = np.einsum("nji,nj->ni", r_wb, a + np.array([0.0, 0.0, G]))  # R_wb^T (a + g z)
    ti = td[::step]
    gyro, acc = w_dense[::step], f_dense[::step]
    rng = np.random.default_rng(imu.get("seed", 0))
    if imu.get("enabled", True):
        gyro_n, acc_n = gyro, acc
        if imu.get("vibration"):
            # The sensor averages its internal samples over each output period (a crude anti-alias
            # filter), so what survives decimation is attenuated and aliased, as on a real MEMS IMU.
            gv, av = motor_vibration(td, f_dense, imu["vibration"], rng)
            box = np.ones(step) / step
            avg = lambda x: np.stack([np.convolve(x[:, k], box)[: len(x)] for k in range(3)], axis=1)[::step]  # noqa: E731
            gyro_n, acc_n = gyro + avg(gv), acc + avg(av)
        gyro_n, b_g = add_noise(gyro_n, imu["rate_hz"], imu["gyro_noise_density"], imu["gyro_random_walk"], rng)
        acc_n, b_a = add_noise(acc_n, imu["rate_hz"], imu["accel_noise_density"], imu["accel_random_walk"], rng)
    else:
        gyro_n, acc_n = gyro, acc
        b_g, b_a = np.zeros_like(gyro), np.zeros_like(acc)

    # Camera frames at exact multiples of 1/fps.
    tf = np.arange(0.0, duration, 1.0 / cam["fps"])
    pf, _, _, _, r_wc = state(tf)
    q_cv = R.from_matrix(r_wc).as_quat()                                       # x y z w
    q_usd = R.from_matrix(r_wc @ CV_TO_USD).as_quat()

    out = BUILD / "flight" / args.flight
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "camera_poses_tum.txt", np.column_stack([tf, pf, q_cv]),
               fmt="%.9f", header="t x y z qx qy qz qw  (OpenCV optical frame, world = local ENU)")
    np.savetxt(out / "camera_poses_usd.csv", np.column_stack([np.arange(len(tf)), tf, pf,
                                                              q_usd[:, [3, 0, 1, 2]]]),
               fmt=["%d"] + ["%.9f"] * 8, delimiter=",", header="frame,t,x,y,z,qw,qx,qy,qz  (USD camera frame)")
    q_wb = R.from_matrix(r_wb[::step]).as_quat()
    np.savetxt(out / "groundtruth_200hz.csv", np.column_stack([ti, p[::step], q_wb[:, [3, 0, 1, 2]], v[::step]]),
               fmt="%.9f", delimiter=",", header="t,px,py,pz,qw,qx,qy,qz,vx,vy,vz  (body FLU in world ENU)")
    hdr = "t,wx,wy,wz,ax,ay,az  (body FLU, rad/s, m/s^2)"
    np.savetxt(out / "imu.csv", np.column_stack([ti, gyro_n, acc_n]), fmt="%.9f", delimiter=",", header=hdr)
    np.savetxt(out / "imu_clean.csv", np.column_stack([ti, gyro, acc]), fmt="%.9f", delimiter=",", header=hdr)
    np.savetxt(out / "imu_bias.csv", np.column_stack([ti, b_g, b_a]), fmt="%.9f", delimiter=",",
               header="t,bgx,bgy,bgz,bax,bay,baz  (true IMU biases)")

    if fl.get("dynamics") == "pegasus":
        # Controller reference for flight/pegasus_fly.py (Pegasus NonlinearController column order).
        # pegasus_post.py then replaces the kinematic outputs above with what the vehicle actually flew.
        ts = np.clip(td - hover0, 0.0, moving)
        jerk = spline(ts, 3) * ((td > hover0) & (td < hover0 + moving))[:, None]
        psi = np.unwrap(np.arctan2(r_wb[:, 1, 0], r_wb[:, 0, 0]))
        np.savetxt(out / "reference_1000hz.csv", np.column_stack([td, p, v, a, jerk, psi, np.gradient(psi, td)]),
                   fmt="%.9f", delimiter=",", header="t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw,yaw_rate  (world ENU)")

    # Consistency checks: spline hits the waypoints; clean IMU integrates back to the path.
    wp_err = float(np.abs(spline(knots) - wp).max())
    p_int = integrate_imu(ti, gyro, acc, p[0], v[0], r_wb[0])
    drift = float(np.linalg.norm(p_int - p[::step], axis=1).max())
    fw = cam["width"]
    fx = cam["focal_length_mm"] / cam["horizontal_aperture_mm"] * fw
    meta = {
        "flight": args.flight, "site": fl["site"], "source": fl["source"],
        "duration_s": float(duration), "frames": int(len(tf)), "fps": cam["fps"], "imu_rate_hz": imu["rate_hz"],
        "path_length_m": float(seg.sum()), "max_speed_mps": float(np.linalg.norm(v, axis=1).max()),
        "max_tilt_deg": float(np.degrees(np.arccos(np.clip(r_wb[:, 2, 2], -1, 1))).max()),
        "intrinsics": {"width": fw, "height": cam["height"], "fx": fx, "fy": fx,
                       "cx": fw / 2, "cy": cam["height"] / 2},
        "T_body_camera_cv": np.block([[r_bc, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]).tolist(),
        "camera_mount": cam["mount"],
        "checks": {"waypoint_max_err_m": wp_err, "imu_integration_drift_m": drift},
    }
    (out / "trajectory.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps({k: meta[k] for k in ("duration_s", "frames", "path_length_m", "max_speed_mps",
                                           "max_tilt_deg", "checks")}))
    if wp_err > 1e-6 or drift > 0.01 * max(1.0, duration / 10.0):   # 1 cm per 10 s of flight
        raise SystemExit(f"trajectory self-check failed: waypoint err {wp_err:.2e} m, IMU drift {drift:.4f} m")


if __name__ == "__main__":
    main()
