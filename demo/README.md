# Drone simulation exploration: results

Evidence from exploring Isaac Sim 6.0.1 + Pegasus Simulator as a test bed for drone visual-inertial navigation. It covers simulated drones flown through high-fidelity scenes with a 4-fisheye camera rig and a stereo pair, two estimators scored against ground truth with fixed pass thresholds, and the drone and autopilot models built on the way.

Status: 2026-10-31. The test batch is still running. `results/` is regenerated with `uv run python testplan/package_demo.py` as cases finish; see [Still running](#still-running). What changed and why: [CHANGELOG.md](../CHANGELOG.md). The plan: [docs/test_plans.md](../docs/test_plans.md).

## What was built

| Layer | What | Where |
|---|---|---|
| Scenes | Cesium Google photoreal tiles (Marina Bay, Singapore); Rivermark (NVIDIA modelled town, 730 x 890 m core); NuRec Gaussian splats (real indoor hall scan) | `cesium/`, `pipeline/common/environments.py`, `flight/env_probe.py` |
| Drone dynamics | Pegasus Iris (PhysX, 1 kHz); motor lag; wind with turbulence and gusts; coaxial helicopter sized like the Ascent AeroSystems Spirit | `flight/pegasus_fly.py`, `flight/pegasus_models.py`, `flight/coax_vehicle.py` |
| Flight control | Pegasus geometric controller (perfect state); PX4 v1.16 SITL (EKF2, offboard); ArduCopter 4.6.3 SITL (GUIDED) | `px4/`, `ardupilot/`, `flight/px4_offboard.py` |
| Sensors | 4 fisheyes (front, left, back, right, 640 x 480, 180 deg); pinhole stereo pair; 200 Hz IMU with vibration and noise; rendered in lockstep on sim time | `flight/rig_fisheye.py`, `flight/pegasus_post.py` |
| Estimators | OpenVINS (4 fisheyes + IMU, ROS 2, live and bag replay); MAC-VO (learned stereo VO, offline) | `ros2/openvins/`, `ros2/bag/`, `macvo/` |
| Test plan | Gates before any estimator runs, fixed thresholds, 3 IMU seeds per case, automated report | `testplan/`, `config/tests.yaml` |

## Flights (previews)

Each preview is a downscaled copy of the full render. Full-resolution videos, frames and ROS 2 bags stay in `build/`, which is not in git.

| Flight | Fisheye grid (front / left / back / right) | Stereo pair |
|---|---|---|
| Marina Bay loop, Pegasus Iris, Cesium tiles | [![](media/marina_bay_loop_pegasus__rig_lockstep_grid.jpg)](media/marina_bay_loop_pegasus__rig_lockstep_grid.mp4) | [![](media/marina_bay_loop_pegasus__rig_lockstep_stereo_stereo.jpg)](media/marina_bay_loop_pegasus__rig_lockstep_stereo_stereo.mp4) |
| Rivermark P1: hover + yaw sweep | [![](media/rm_p1_hover__rig_lockstep_grid.jpg)](media/rm_p1_hover__rig_lockstep_grid.mp4) | [![](media/rm_p1_hover__rig_lockstep_stereo_stereo.jpg)](media/rm_p1_hover__rig_lockstep_stereo_stereo.mp4) |
| Rivermark P2: slow loop, 2 m/s | [![](media/rm_p2_slow__rig_lockstep_grid.jpg)](media/rm_p2_slow__rig_lockstep_grid.mp4) | [![](media/rm_p2_slow__rig_lockstep_stereo_stereo.jpg)](media/rm_p2_slow__rig_lockstep_stereo_stereo.mp4) |
| Rivermark P3: nominal loop, 6 m/s | [![](media/rm_p3_nominal__rig_lockstep_grid.jpg)](media/rm_p3_nominal__rig_lockstep_grid.mp4) | [![](media/rm_p3_nominal__rig_lockstep_stereo_stereo.jpg)](media/rm_p3_nominal__rig_lockstep_stereo_stereo.mp4) |
| Rivermark P4: aggressive figure-8, 10 m/s | [![](media/rm_p4_aggressive__rig_lockstep_grid.jpg)](media/rm_p4_aggressive__rig_lockstep_grid.mp4) | [![](media/rm_p4_aggressive__rig_lockstep_stereo_stereo.jpg)](media/rm_p4_aggressive__rig_lockstep_stereo_stereo.mp4) |
| Rivermark P3 under PX4 in wind (first run, with the 2 Hz wobble described below) | [![](media/rm_p3_px4__rig_lockstep_grid.jpg)](media/rm_p3_px4__rig_lockstep_grid.mp4) | [![](media/rm_p3_px4__rig_lockstep_stereo_stereo.jpg)](media/rm_p3_px4__rig_lockstep_stereo_stereo.mp4) |

Other cases add their previews to `media/` as they finish.

## Scenes

| Rivermark top view (planning map) | NuRec hall, fisheye vs pinhole lens check (NCC 0.98) |
|---|---|
| ![](environments/rivermark/topdown.png) | ![](environments/nurec_xgrid_wormhole/g2_0.png) |

## Test results so far

Thresholds were fixed before the first run (`config/tests.yaml`). Pass/fail applies to profiles P2 and P3; the other profiles are report-only, to find breaking points. Full table: [results/report.md](results/report.md). Raw rows: [results/results.csv](results/results.csv).

**Data gates:** every rendered case passed the image check G1 at 0.05 to 0.09 px, except the PX4 wobble run. Every flight passed G3 (tracking, IMU consistency, clearance from buildings). The stereo disparity check G4 fails at 25 to 40 m altitude with the 0.30 m baseline (1.3 to 1.6 px); it passes on the low-texture leg (4.1 px).

| Case | OpenVINS, 4 fisheyes + IMU (ATE % of path, 3 seeds) | MAC-VO, stereo (ATE %) |
|---|---|---|
| P1 hover | 0.3, 3.4, 4.6 m drift (report-only) | 0.38 m |
| P2 slow loop | 13, 33, 21: diverged | 0.99 (fails rotation 4.4 deg) |
| P3 nominal loop | 13, 22, 3.9: fail | 0.75 (fails rotation 3.3 deg) |
| P4 aggressive figure-8 | **0.18, 0.20, 0.19** | **0.18** |
| P5 low-texture leg | diverged on all three | 1.77 |
| P6 close pass between houses | diverged | still running |

What the results say:
- **OpenVINS on this rig works when the motion is strong and fails when it is gentle.** In the aggressive figure-8 it is under 0.2 % and repeatable. On gentle loops and over bare ground it loses scale (0.3 to 0.8 x) and diverges. The images are consistent with ground truth (G1), so the next step is the plan's ablations: IMU noise margin, front camera only, clean IMU.
- **MAC-VO stays near or below 2 % in every case,** with metric scale from stereo even at only 1.3 px of disparity. It fails the rotation threshold on the loops.
- **Concurrency:** the same RTX 5080 fits 6 MAC-VO instances at once, at about 10 frames/s in total ([macvo_stress/](macvo_stress)).

Earlier results on Cesium Marina Bay (OpenVINS, 4 fisheyes, 105 s loop): ATE 26 m with kinematic motion, then 7.5 m (0.89 %) once the Pegasus Iris flew the same loop ([openvins_cesium/](openvins_cesium)).

## Findings about the tools

- **Fast to prototype.** A new drone is one vehicle class and a parameter file, a new place is an environment entry, and a new autopilot is a backend.
- **Pegasus on Isaac 6.0.1 needed fixes:**
  - It needs a community port.
  - It read a stale attitude from USD.
  - Its PhysX angular velocity is biased against the attitude.
  - Its PX4 link hung with 1 kHz physics.
  - All four are fixed in this repo.
- **Physics parameters matter.** Motor lag of 35 / 60 ms put PX4's Iris tuning into a 2 Hz wobble; PX4's own 12.5 / 25 ms removed it (roll std 0.08 deg).
  - The coaxial model's first ground tests found three model bugs: no landing gear, a rotor spring active with the rotors stopped, and unstable cylinder colliders.
- **NuRec:** the dataset's current (v0.2) scenes need Isaac 6.1; on 6.0.1 only the older XGRIDS export renders.
- **Speed:** 1 kHz physics runs at about 0.14 x real time (0.08 to 0.1 x under PX4), and rendering at about 5 frames/s. A test case takes about 40 min end to end.

## Still running

- Rivermark P6 (MAC-VO), P7 lighting variants (low sun, overcast), the indoor NuRec cases E2-P1 to P3, and the Cesium regression case.
- PX4 Iris loop re-run with the fixed motor lag. The re-flight is done and tracks to 0.45 m RMS in 4 m/s wind; its test case is queued.
- Coaxial Spirit-class model:
  - **Under ArduCopter 4.6.3:** first full test flight done (20 s of the P3 loop in 4 m/s wind with turbulence). It took off from the ground in GUIDED mode, held hover to 0.09 deg roll/pitch, and tracked the loop to 0.22 m RMS (0.40 m max). Rotor speed is 250 rad/s at hover, as the model predicts ([flights/rm_p3_coax_ap](flights/rm_p3_coax_ap)).
  - **Under PX4:** re-test running.
  - **Parameters:** mass, inertia and rotor coefficients are estimates until field data arrives ([docs/field_data_checklist.html](../docs/field_data_checklist.html)).

## Folder guide

```
demo/
  results/           report.md, results.csv, report.png, cases/<case>/ (gates, per-run metrics)
  flights/<flight>/  trajectory.json (path, tracking, IMU checks), pegasus.json, rig rates, stereo.json
  media/             preview videos (.mp4) and stills (.jpg) per rendered flight
  environments/      Rivermark top view and density, NuRec lens-check images
  openvins_cesium/   earlier OpenVINS runs on Cesium Marina Bay (eval.json / eval.png)
  macvo_stress/      MAC-VO concurrency stress test
```
