"""Run one test case of docs/test_plans.md end to end and record the results.

Steps (each can be skipped with --steps):
  flight   gate G3 on the Pegasus flight (pegasus_fly.py + pegasus_post.py must have run; see README)
  vio      OpenVINS live on the lockstep fisheye render (seed = imu.seed) while the topics are recorded
           to a bag; then the bag is replayed once per extra seed with that seed's IMU stream; gate G1
  stereo   stereo render, gates G1 + G4, MAC-VO Fast on the frames
  clean    videos, then delete the saved frames (stereo frames only after MAC-VO)

Writes build/tests/<case>/{case.json, vio_s<seed>/, macvo/, bag/} and appends rows to build/tests/results.csv.

Run: uv run python testplan/run_case.py --case TC-E1-P3
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402
from testplan import gates  # noqa: E402
from testplan.clearance import flight_clearance  # noqa: E402
from testplan.metrics import load_gt, load_macvo, load_openvins, trajectory_metrics  # noqa: E402

ISAAC = Path(os.environ.get("ISAAC_ROOT", Path.home() / "Downloads/isaac-sim-standalone-6.0.1-linux-x86_64"))
RESULTS = BUILD / "tests" / "results.csv"
FIELDS = ["case", "profile", "env", "flight", "lighting", "sut", "run", "status", "path_m", "ate_rmse_m", "ate_pct",
          "final_err_m", "final_pct", "rot_rmse_deg", "scale_sim3", "rpe10_pct", "t_first_s", "verdict", "notes"]


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def sh(cmd: str, logfile: Path, timeout: float = 7200) -> int:
    with open(logfile, "a") as f:
        return subprocess.run(["bash", "-c", cmd], cwd=PROJECT_ROOT, stdout=f, stderr=subprocess.STDOUT,
                              timeout=timeout).returncode


def bg(cmd: str, logfile: Path) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd], cwd=PROJECT_ROOT, stdout=open(logfile, "a"), stderr=subprocess.STDOUT)


def docker_stop(name: str, proc: subprocess.Popen | None, timeout: float = 60) -> None:
    subprocess.run(["docker", "kill", "-s", "INT", name], capture_output=True)
    if proc:
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            subprocess.run(["docker", "kill", name], capture_output=True)
            proc.wait(timeout=30)


def wait_for(path: Path, text: str, timeout: float = 120) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if path.exists() and text in path.read_text(errors="ignore"):
            return True
        time.sleep(1)
    return False


def rig_cmd(flight: str, extra: str, cesium: bool) -> str:
    # Rivermark needs no token; only Cesium flights pull .env in (via ros_env.sh -> env.sh).
    pre = "source ros2/ros_env.sh && " if cesium else (
        f"export ISAAC_ROOT={ISAAC} ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 "
        f"LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{ISAAC}/exts/isaacsim.ros2.core/jazzy/lib "
        f"PYTHONPATH=$PYTHONPATH:{ISAAC}/exts/isaacsim.ros2.core/jazzy/rclpy && ")
    return pre + f"{ISAAC}/python.sh flight/rig_fisheye.py --flight {flight} --mode lockstep {extra}"


def verdict_vio(m: dict, th: dict, graded: bool) -> tuple[str, str]:
    if "ate_pct" not in m:
        return "report", "no path length (hover)"
    if m["ate_pct"] > th["divergence_pct"]:
        return "FAIL", "diverged"
    fails = [k for k, ok in (("ate", m["ate_pct"] <= th["ate_pct"]), ("drift", m["final_pct"] <= th["drift_pct"]),
                             ("rot", m["rot_rmse_deg"] <= th["rot_rmse_deg"]), ("init", m["t_first_s"] <= th["init_s"]))
             if not ok]
    if not graded:
        return "report", ",".join(fails)
    return ("PASS" if not fails else "FAIL"), ",".join(fails)


def verdict_vo(m: dict, th: dict, graded: bool) -> tuple[str, str]:
    if "ate_pct" not in m:
        return "report", "no path length (hover)"
    fails = [k for k, ok in (("ate", m["ate_pct"] <= th["ate_pct"]), ("drift", m["final_pct"] <= th["drift_pct"]),
                             ("rot", m["rot_rmse_deg"] <= th["rot_rmse_deg"]),
                             ("scale", abs(m["scale_sim3"] - 1) <= th["scale_err"]),
                             ("first", m["t_first_s"] <= th["first_pose_s"])) if not ok]
    if not graded:
        return "report", ",".join(fails)
    return ("PASS" if not fails else "FAIL"), ",".join(fails)


def append_row(row: dict) -> None:
    new = not RESULTS.exists()
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items() if k in FIELDS})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", required=True)
    ap.add_argument("--steps", default="flight,vio,stereo,clean")
    ap.add_argument("--seeds", type=int, nargs="*", help="override vio_seeds")
    args = ap.parse_args()
    steps = set(args.steps.split(","))
    tcfg = load_config(PROJECT_ROOT / "config" / "tests.yaml")
    fcfg = load_config(PROJECT_ROOT / "config" / "flight.yaml")
    case = next(c for c in tcfg["cases"] if c["id"] == args.case)
    th = tcfg["thresholds"]
    flight, profile, lighting = case["flight"], case["profile"], case.get("lighting", "default")
    fl = fcfg["flights"][flight]
    env = fcfg["environments"].get(fl.get("env")) if fl.get("env") else None
    cesium = env is None
    graded = profile in th["graded_profiles"]
    out = BUILD / "tests" / args.case
    out.mkdir(parents=True, exist_ok=True)
    caselog = out / "case.log"
    fdir = BUILD / "flight" / flight
    suffix = f"_{lighting}" if lighting != "default" else ""
    rig_run, stereo_run = f"rig_lockstep{suffix}", f"rig_lockstep_stereo{suffix}"
    state_path = out / "case.json"
    state = json.loads(state_path.read_text()) if state_path.exists() else {"case": case}
    base = {"case": args.case, "profile": profile, "env": fl.get("env") or fl.get("site"), "flight": flight,
            "lighting": lighting}

    def save():
        state_path.write_text(json.dumps(state, indent=2, default=float) + "\n")

    if "flight" in steps:
        traj = json.loads((fdir / "trajectory.json").read_text())
        g3 = {"track_rms_m": traj.get("tracking", {}).get("pos_err_rms_m"),
              "imu_drift_m": traj["checks"].get("imu_drift_10s_m", traj["checks"]["imu_integration_drift_m"]),
              "imu_drift_whole_flight_m": traj["checks"]["imu_integration_drift_m"], "dynamics": traj.get("dynamics")}
        if env:
            g3["clearance"] = flight_clearance(flight, "flown")
        track_lim = th["gates"]["g3_track_rms_px4_m" if fl.get("controller") == "px4" else "g3_track_rms_m"]
        ok = (g3["track_rms_m"] is not None and g3["track_rms_m"] <= track_lim
              and g3["imu_drift_m"] <= th["gates"]["g3_imu_drift_m"]
              and (not env or g3["clearance"]["min_clearance_m"] >= th["gates"][
                  "g3_clearance_indoor_m" if env.get("occupancy") else "g3_clearance_m"]))
        g3["pass"] = bool(ok)
        state["g3"] = g3
        save()
        log(f"G3 {'PASS' if ok else 'FAIL'} {json.dumps(g3)}")
        if not ok:
            raise SystemExit("gate G3 failed; not rendering")

    gt = load_gt(fdir / "groundtruth_200hz.csv")
    seeds = args.seeds if args.seeds is not None else tcfg["vio_seeds"]
    if "vio" in steps:
        cfg_name = f"tc_{flight}"
        fi = (env or {}).get("openvins", {}).get("fi_max_dist", 1500.0)
        sh(f"uv run python ros2/openvins/make_config.py --flight {flight} --name {cfg_name} --fi-max-dist {fi}", caselog)
        bag = out / f"bag{suffix}"
        if bag.exists():
            shutil.rmtree(bag)
        for i, seed in enumerate(seeds):
            run = out / f"vio{suffix}_s{seed}"
            if run.exists():
                shutil.rmtree(run)
            run.mkdir(parents=True)
            ov = bg(f"ros2/openvins/run_openvins.sh {cfg_name} {run}", run / "launch.log")
            if not wait_for(run / "openvins.log", "subscribing to cam", 120):
                docker_stop("openvins", ov)
                raise SystemExit("OpenVINS did not start")
            if i == 0:
                rec = bg(f"ros2/bag/record.sh {bag}", out / "bag_record.log")
                time.sleep(5)
                log(f"VIO seed {seed}: live render + bag")
                light = f"--lighting {lighting}" if lighting != "default" else ""
                rc = sh(rig_cmd(flight, f"--ros2 --save-all {light}", cesium), out / f"render_fisheye{suffix}.log")
                time.sleep(3)
                docker_stop("rosbag", rec)
            else:
                imu_csv = fdir / f"imu_seed{seed}.csv"
                log(f"VIO seed {seed}: bag replay with {imu_csv.name}")
                rc = sh(f"ros2/bag/replay.sh {bag} --imu-csv {imu_csv} --rate {tcfg['replay_rate']}",
                        run / "replay.log")
            time.sleep(3)
            docker_stop("openvins", ov)
            est = run / "ov_estimate.txt"
            row = {**base, "sut": "openvins", "run": f"s{seed}" + ("_live" if i == 0 else "_replay")}
            n_est = sum(1 for line in open(est) if line.strip() and not line.startswith("#")) if est.exists() else 0
            if rc != 0 or n_est < 10:
                append_row({**row, "status": f"error rc={rc}", "verdict": "FAIL", "notes": "no estimate"})
                state.setdefault("vio", {})[str(seed)] = {"error": rc}
                save()
                continue
            t, p, r = load_openvins(est)
            k = (t >= gt[0][0]) & (t <= gt[0][-1])
            m = trajectory_metrics(t[k], p[k], r[k], gt, "4dof")
            v, why = verdict_vio(m, th["vio"], graded)
            (run / "metrics.json").write_text(json.dumps(m, indent=2) + "\n")
            state.setdefault("vio", {})[str(seed)] = m
            save()
            append_row({**row, "status": "ok", **m, "rpe10_pct": m["rpe10"].get("rmse_pct"), "verdict": v, "notes": why})
            log(f"VIO seed {seed}: ATE {m['ate_rmse_m']:.2f} m ({m.get('ate_pct', float('nan')):.2f} %) -> {v} {why}")
        ates = [state["vio"][str(s)]["ate_rmse_m"] for s in seeds if "ate_rmse_m" in state.get("vio", {}).get(str(s), {})]
        if len(ates) > 1:
            spread = float(np.std(ates) / np.mean(ates))
            state["vio_seed_spread"] = {"ates": ates, "std_over_mean": spread,
                                        "pass": spread <= th["vio"]["seed_spread"]}
            save()
        state["g1_fisheye"] = gates.g1(flight, rig_run, stereo=False)
        state["g1_fisheye"]["pass"] = bool((state["g1_fisheye"]["median_px"] or 9) <= th["gates"]["g1_median_px"])
        save()
        log(f"G1 fisheye {state['g1_fisheye']}")

    if "stereo" in steps:
        log("stereo render")
        light = f"--lighting {lighting}" if lighting != "default" else ""
        sh(rig_cmd(flight, f"--stereo --save-all {light}", cesium), out / f"render_stereo{suffix}.log")
        state["g1_stereo"] = gates.g1(flight, stereo_run, stereo=True)
        state["g1_stereo"]["pass"] = bool((state["g1_stereo"]["median_px"] or 9) <= th["gates"]["g1_median_px"])
        state["g4"] = gates.g4(flight, stereo_run)
        state["g4"]["pass"] = bool((state["g4"]["median_px"] or 0) >= th["gates"]["g4_disparity_px"])
        save()
        log(f"G1 stereo {state['g1_stereo']['median_px']}  G4 disparity {state['g4']['median_px']}")
        mv = out / f"macvo{suffix}"
        if mv.exists():
            shutil.rmtree(mv)
        rc = sh(f"MACVO_THREADS=4 macvo/run_macvo.sh {fdir / stereo_run} {mv} Fast --no-eval", out / "macvo.log")
        row = {**base, "sut": "macvo", "run": "fast"}
        traj = mv / "traj_est_body_world.csv"
        if rc != 0 or not traj.exists():
            append_row({**row, "status": f"error rc={rc}", "verdict": "FAIL", "notes": "no trajectory"})
        else:
            t, p, r = load_macvo(traj)
            m = trajectory_metrics(t, p, r, gt, "se3")
            v, why = verdict_vo(m, th["vo"], graded)
            if not state["g4"]["pass"]:
                why = (why + ",G4").strip(",")
            (mv / "metrics.json").write_text(json.dumps(m, indent=2) + "\n")
            state["macvo"] = m
            save()
            append_row({**row, "status": "ok", **m, "rpe10_pct": m["rpe10"].get("rmse_pct"), "verdict": v, "notes": why})
            log(f"MAC-VO: ATE {m['ate_rmse_m']:.2f} m ({m.get('ate_pct', float('nan')):.2f} %), scale {m['scale_sim3']:.3f} -> {v} {why}")

    if "clean" in steps:
        for run_name, kind in ((rig_run, "fisheye"), (stereo_run, "stereo")):
            d = fdir / run_name
            if not (d / "frames").exists():
                continue
            if kind == "fisheye":
                sh(f"uv run python flight/rig_video.py --flight {flight} --run {run_name}", caselog)
                done = (d / "grid.mp4").exists()
            else:
                sh(f"ffmpeg -loglevel error -y -framerate 30 -i {d}/frames/%06d_left.png -framerate 30 -i {d}/frames/%06d_right.png "
                   f"-filter_complex hstack=inputs=2 -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p {d}/stereo.mp4", caselog)
                done = (d / "stereo.mp4").exists() and (out / f"macvo{suffix}" / "traj_est_body_world.csv").exists()
            if done:
                shutil.rmtree(d / "frames")
                log(f"{kind} video written, frames deleted")
    log("case done")


if __name__ == "__main__":
    main()
