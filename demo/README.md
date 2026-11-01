# Drone simulation exploration: results

Evidence from exploring Isaac Sim 6.0.1 + Pegasus Simulator as a test bed for drone visual-inertial navigation. It covers simulated drones flown through high-fidelity scenes with a 4-fisheye camera rig and a stereo pair, two estimators scored against ground truth with fixed pass thresholds, and the drone and autopilot models built on the way.

Status: 2026-11-01. The first test batch (Rivermark, NuRec, PX4, forest, desert, Cesium regression) is complete. Since then 13 more environments have been built (Singapore and Malaysia, street-level cities, battleground, military) but not yet tested; see [Environments](#environments) and [Not run yet](#not-run-yet). `results/` is regenerated with `uv run python testplan/package_demo.py`. What changed and why: [CHANGELOG.md](../CHANGELOG.md). The plan: [docs/test_plans.md](../docs/test_plans.md).

## What was built

| Layer | What | Where |
|---|---|---|
| Scenes | Cesium Google photoreal tiles (Marina Bay, Sungei Buloh, Central Catchment, Tampines, KL, JB); Rivermark (NVIDIA modelled town); NuRec Gaussian splats (real indoor hall scan); 13 built environments (see below) | `cesium/`, `pipeline/common/environments.py`, `flight/env_probe.py` |
| Environment builder | Recipe to sim-ready scene: procedural terrain (creeks, coastline, water, terraces, craters, trenches), scattered vegetation, generated mangroves and military props, street-level cities from OpenStreetMap, flight-loop planning on the road network | `envbuild/` |
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

| Forest valley (v1) loop at 68 m | [![](media/fv_p3_loop__rig_lockstep_grid.jpg)](media/fv_p3_loop__rig_lockstep_grid.mp4) | [![](media/fv_p3_loop__rig_lockstep_stereo_stereo.jpg)](media/fv_p3_loop__rig_lockstep_stereo_stereo.mp4) |
| Desert dunes loop at 25 m (low texture) | [![](media/dd_p3_loop__rig_lockstep_grid.jpg)](media/dd_p3_loop__rig_lockstep_grid.mp4) | [![](media/dd_p3_loop__rig_lockstep_stereo_stereo.jpg)](media/dd_p3_loop__rig_lockstep_stereo_stereo.mp4) |
| NuRec hall, slow loop | [![](media/nx_p2_slow__rig_lockstep_grid.jpg)](media/nx_p2_slow__rig_lockstep_grid.mp4) | [![](media/nx_p2_slow__rig_lockstep_stereo_stereo.jpg)](media/nx_p2_slow__rig_lockstep_stereo_stereo.mp4) |

**Chase-camera views** (the flown trajectory replayed with the drone model, `flight/watch_flight.py`):

| Rivermark P4 aggressive | PX4 Iris in wind | Coaxial Spirit-class, ArduCopter | Forest valley | Desert dunes |
|---|---|---|---|---|
| [![](media/rm_p4_aggressive__chase.jpg)](media/rm_p4_aggressive__chase.mp4) | [![](media/rm_p3_px4__chase.jpg)](media/rm_p3_px4__chase.mp4) | [![](media/rm_p3_coax_ap__chase.jpg)](media/rm_p3_coax_ap__chase.mp4) | [![](media/fv_p3_loop__chase.jpg)](media/fv_p3_loop__chase.mp4) | [![](media/dd_p3_loop__chase.jpg)](media/dd_p3_loop__chase.mp4) |

## Scenes

| Rivermark top view (planning map) | NuRec hall, fisheye vs pinhole lens check (NCC 0.98) |
|---|---|
| ![](environments/rivermark/topdown.png) | ![](environments/nurec_xgrid_wormhole/g2_0.png) |

## Environments

Built with `envbuild/build_env.py` from the recipes in `envbuild/recipes/`. Every built environment now renders the 4-fisheye test rig within a budget of 300 ms per frame and 9 GB of GPU memory (50–285 ms; the rainforest was 4.6 s before one costly tree model was replaced): [docs/experiments/env_performance.md](../docs/experiments/env_performance.md). Each one is registered in `config/flight.yaml` and has a P3 loop flight and a test case in `config/tests.yaml`. They have not been tested yet. Previews are oblique shots from drone heights (`envbuild/preview.py`); all props on the battleground and military scenes are generated, because no such assets ship with Isaac.

| Singapore mangrove swamp (procedural) | Forest valley v2 (procedural) |
|---|---|
| ![](environments/sg_mangrove/preview.jpg) | ![](environments/forest_valley_v2/preview.jpg) |
| **Swamp flight, chase camera (replay of the flown loop)** | |
| [![](media/sg_p3_loop__chase.jpg)](media/sg_p3_loop__chase.mp4) | |
| **Malaysian oil palm estate** | **Malaysian lowland rainforest** |
| ![](environments/my_oil_palm/preview.jpg) | ![](environments/my_rainforest/preview.jpg) |
| **Tampines, Singapore, street level (OpenStreetMap)** | **Bukit Bintang, Kuala Lumpur, street level (OpenStreetMap)** |
| ![](environments/sg_tampines_osm/preview.jpg) | ![](environments/my_kl_bukit_bintang_osm/preview.jpg) |
| **Battleground: bombed town** | **Battleground: trench field** |
| ![](environments/bg_ruined_town/preview.jpg) | ![](environments/bg_trench_field/preview.jpg) |
| **Military base, desert** | **Military base, jungle** |
| ![](environments/mil_fob_desert/preview.jpg) | ![](environments/mil_fob_tropical/preview.jpg) |

Real places on Google photoreal 3D tiles (verify renders; flights over them stay 60 m or more above the ground, where the tiles look right):

| Sungei Buloh wetland | Central Catchment rainforest | KLCC, Kuala Lumpur | Johor Bahru | Tampines HDB |
|---|---|---|---|---|
| ![](environments/sg_sungei_buloh/tiles_view.png) | ![](environments/sg_central_catchment/tiles_view.png) | ![](environments/my_kl_klcc/tiles_view.png) | ![](environments/my_jb_city/tiles_view.png) | ![](environments/sg_tampines_hdb/tiles_view.png) |

Also built: Johor Bahru street level (`environments/my_jb_osm/`).

## Test results so far

Thresholds were fixed before the first run (`config/tests.yaml`). Pass/fail applies to profiles P2 and P3; the other profiles are report-only, to find breaking points. Full table: [results/report.md](results/report.md). Raw rows: [results/results.csv](results/results.csv).

**Data gates:**
- **G1 (image consistency):** passes at 0.025–0.09 px on Rivermark, the desert and Cesium. Fails on the NuRec splats (0.15–0.20 px, which is expected for splats) and is marginal in the forest (0.101 px against 0.1).
- **G3 (flight):** every flight passes tracking, IMU consistency and clearance.
- **G4 (stereo disparity):** fails above about 25 m altitude with the 0.30 m baseline (1.0–1.6 px). It passes on the low-texture leg, the close pass and indoors (4–7 px).

| Case | OpenVINS, 4 fisheyes + IMU (ATE % of path, 3 runs) | MAC-VO, stereo (ATE %) |
|---|---|---|
| Rivermark P1 hover | 0.3, 3.4, 4.6 m drift (report-only) | 0.38 m |
| Rivermark P2 slow loop | 13, 33, 21: diverged | 0.99 (fails rotation 4.4°) |
| Rivermark P3 nominal loop | 13, 22, 3.9: fail | 0.75 (fails rotation 3.3°) |
| Rivermark P4 aggressive figure-8 | **0.18, 0.20, 0.19** | **0.18** |
| Rivermark P5 low-texture leg | diverged on all three | 1.77 |
| Rivermark P6 close pass between houses | diverged on all three | 0.91 |
| Rivermark P7 low sun / overcast | 6.9–12 / 7.1–10 | 0.79 / 0.58 |
| NuRec hall P2 / P3 | 9.7–11 / 8.9–9.1 (scale 0.36–0.44) | 0.68 / 0.56 |
| PX4 Iris in wind, P3 loop | 15, 15, 15: diverged | 0.97 |
| Forest valley v1, 705 m loop | 4.0, 13, 13 | **0.50** (fails rotation 3.1°) |
| Desert dunes, 753 m loop | 21, 684, 1373: diverged | 3.9 (fails) |
| Cesium Marina Bay regression | 20 (two replays gave no estimate) | 1.06 |

What the results say:
- **OpenVINS on this rig works when the motion is strong and fails when it is gentle.** In the aggressive figure-8 it is under 0.2 % and repeatable. On gentle loops, over bare ground and over the low-texture desert it loses scale and diverges. The images are consistent with ground truth (G1).
  - **Four separate mono cameras make OpenVINS non-repeatable run to run:** the same recorded bag gives different results. One camera is repeatable. Details: [docs/experiments/openvins_multicam.md](../docs/experiments/openvins_multicam.md).
- **MAC-VO stays at or below 2 % everywhere except the desert,** with metric scale from stereo even at 1.3 px of disparity. It usually fails the 3° rotation threshold on loops.
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

## Not run yet

- **Test cases for the 13 new environments** (`TC-EF2-P3`, `TC-SG-P3`, `TC-SB-P3`, `TC-CC-P3`, `TC-TS-P3`, `TC-KL-P3`, `TC-JB-P3`, `TC-OP-P3`, `TC-RF-P3`, `TC-BT-P3`, `TC-TF-P3`, `TC-FD-P3`, `TC-FT-P3`, tile overflights `TC-KLT/JBT/TPT-P3`). Flights exist for the swamp, forest v2, Sungei Buloh, Central Catchment, Tampines and KL streets, and the oil palm estate.
- **Why they were stopped:** the heavier scenes rendered the 4-fisheye rig at 0.7–4.6 s per frame and nearly filled the 16 GB GPU. That is fixed (all within 300 ms per frame now), so the cases can run; the oil palm, rainforest, battleground, military and JB flights still need flying, and the forest v2 loop needs re-flying at its new 72 m altitude.
- **Coaxial Spirit-class model:** test-flown under ArduCopter (0.22 m RMS) and PX4 (0.19 m RMS) for 20 s; no full test case yet. Mass, inertia and rotor coefficients are estimates until field data arrives ([docs/field_data_checklist.html](../docs/field_data_checklist.html)).

## Folder guide

```
demo/
  results/           report.md, results.csv, report.png, cases/<case>/ (gates, per-run metrics)
  flights/<flight>/  trajectory.json (path, tracking, IMU checks), pegasus.json, rig rates, stereo.json
  media/             preview videos (.mp4) and stills (.jpg) per rendered flight
  environments/      per environment: env.json / environment.yaml (recipe output), preview.jpg, top view,
                     tiles_view.png for Google-tile scenes, NuRec lens-check images
  openvins_cesium/   earlier OpenVINS runs on Cesium Marina Bay (eval.json / eval.png)
  macvo_stress/      MAC-VO concurrency stress test
```
