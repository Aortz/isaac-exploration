# Test plans: 4-fisheye VIO and stereo VO flights in high-fidelity environments

Status: in execution since 2026-10-31. Results are in `demo/results/report.md`; deviations from the draft are listed at the end.
Thresholds in section 6 are fixed before the first run and not tuned afterwards (same rule as `qa:` in `config/flight.yaml`).

## 1. Goal

Measure how well two estimators track a physically flown quadrotor, outside the Cesium setup:

| System under test | Sensors | Estimator |
|---|---|---|
| SUT-A | 4 fisheyes (front/left/back/right, 20° down, 640×480, equidistant f = 160) + IMU 200 Hz | OpenVINS (ROS 2, live, sim time) |
| SUT-B | Pinhole stereo pair (640×480, fx = 320, forward, pitched down) | MAC-VO Fast (offline on saved frames, no IMU) |

Both systems see the same flight in each test case: same Pegasus trajectory, same ground truth, same IMU noise seed.

Questions the plan answers:
1. Does accuracy hold up when the scene is close (5–40 m) and fully 3D? Cesium at 110 m was the weak-depth case.
2. Where does each system break: speed, rotation rate, low texture, lighting?
3. Run-to-run spread: OpenVINS was nondeterministic on Cesium.

## 2. Environments

| ID | Environment | What it is | Fit | Limits |
|---|---|---|---|---|
| E0 | Cesium Google 3D Tiles, Marina Bay (done) | Photogrammetry of the real city, streamed | Baseline only | Baked lighting, blurry up close, no colliders, needs an ion token |
| E1 | **Rivermark** (`/Isaac/Environments/Outdoor/Rivermark`, ships with Isaac 6.0) | Hand-modelled outdoor town block with PBR materials, lit by RTX | **Main outdoor environment.** Sharp at low altitude, controllable sun, exact depth | Modelled rather than captured, so cleaner than reality; area limited (see 3.2) |
| E2 | **NuRec Gaussian-splat captures** ([PhysicalAI-Robotics-NuRec](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-NuRec), CC-BY-4.0, gated) | Real-world scans rendered as OpenUSD ParticleField in RTX | **Photoreal real-world check**, indoors (warehouse/lab, office lounge) | Gated: needs a Hugging Face login and accepting the terms. Artifacts when the camera leaves the capture path. Fisheye lens on ParticleField not yet verified (gate G2) |
| E3 | Digital_Twin_Warehouse (Isaac 6.0) | Modelled indoor warehouse | Fallback if E2 fails gate G2 | Repetitive shelving, which is a known VIO trap (also a useful test) |

Not chosen: AirSim/Colosseum and CARLA run on Unreal, not Isaac, so the rig, ROS 2 and Pegasus code would not carry over. A capture of your own site through 3DGRuT into NuRec is the best long-term option, but it needs your own footage.

## 3. Flights

All flights use Pegasus dynamics (`dynamics: pegasus`): an Iris at 1 kHz physics, drag on, 3 s pre-roll and a 4 s hover at the start (OpenVINS static init). Rendering is lockstep at 30 Hz.

### 3.1 Flight profiles (same shapes in every environment, scaled to fit)

| ID | Profile | Speed | Max tilt / yaw rate | Duration | Purpose |
|---|---|---|---|---|---|
| P1 | Hover + slow yaw sweep | 0 m/s, yaw ±90° | 2° / 30°/s | 30 s | Sanity: initialisation, pure rotation (bad for VO scale), IMU bias |
| P2 | Slow loop | 2 m/s outdoor, 1 m/s indoor | 5° | 60–90 s | Nominal accuracy |
| P3 | Nominal loop | 6 m/s outdoor, 2 m/s indoor | 15–25° | 60–90 s | Main comparison number |
| P4 | Aggressive figure-8 | 10 m/s outdoor, 3 m/s indoor | 35° / 90°/s | 60 s | Motion blur is off, so this tests feature tracking and filter dynamics only |
| P5 | Low-texture leg | P3 speed | P3 | 60 s | Pass over an empty road, plaza or roof (E1) or a blank wall or floor (E2); stereo and VIO degeneracy |
| P6 | Close-range pass | P2 speed | P2 | 45 s | Within 3–5 m of facades or shelves; big parallax, near-plane and occlusion stress |
| P7 | Lighting variants of P3 (E1 only) | P3 | P3 | P3 | Sun noon / low sun 10° / overcast dome; same flight, same seed |

Each (environment, profile) pair is flown once by Pegasus. All repeats reuse that trajectory, so differences come from the estimator, not the flight.

### 3.2 Per-environment geometry

- **E1 Rivermark** (probed with `flight/env_probe.py`, output in `build/envs/rivermark/`):
  - Size: Z-up, metres. The full stage spans 2.3 × 2.8 km, but the detailed town core is about 730 × 890 m (x −388…342, y −398…488).
  - Contents: 99,724 meshes, 4,818 colliders and 669 lights.
  - Buildings are low-rise:
    - Object tops: median 10 m, 99th percentile 17.5 m, 99.9th percentile 28 m.
    - Ground is around z ≈ 0–5 m.
  - Layout from the density map:
    - A dense block around (0, 0) to (300, 450).
    - Long parallel rows (parking or road) running SW–NE across y −450…−100.
    - A diagonal road in the NW.
  - Flight altitudes:
    - P2–P5 at 20–35 m above ground, so above the rooftops and 5 m clear of the 28 m tallest objects.
    - P5 low-texture leg over the parking rows, south-west.
    - P6 close-range pass at 8–12 m altitude along a street canyon in the dense block, at least 5 m from facades.
  - Loop footprint: about 400 × 350 m around the dense block, so P3 at 6 m/s is about 80 s.
  - Collisions: Rivermark has colliders, so Pegasus can detect them; a collision fails the run.
- **E2 NuRec:**
  - Path: fly inside the capture volume, at 1–2 m height along the captured walking path, offset by at most 1 m. The scene's own training camera poses define the safe corridor.
  - Collisions: the splats have no colliders. Use the occupancy map shipped with the scene for clearance checks.
- **E3 Warehouse:** aisles at 2–4 m height.

### 3.3 Sensor settings per environment

| Setting | E1 outdoor | E2/E3 indoor | Why |
|---|---|---|---|
| Stereo baseline | 0.30 m | 0.12 m | Disparity of 2–20 px at the typical depth (E1 10–60 m, indoor 2–10 m) |
| Stereo pitch | 20° down | 0° | Keep the horizon/structure in view |
| Fisheye rig | unchanged | unchanged | One rig everywhere, so results compare |
| OpenVINS `fi_max_dist` | 300 m | 50 m | Close to the scene again; 1500 m was only for Cesium |
| Camera clip near | 0.1 m | 0.05 m | Close-range pass |

## 4. Test cases

Each test case is one flight, run through both systems.

| TC | Env | Profile | SUT-A runs | SUT-B runs | Notes |
|---|---|---|---|---|---|
| TC-E1-P1 … P7 | E1 | P1–P7 | 3 each (IMU seeds 1–3) | 1 each (deterministic) | P7 = 3 lighting variants |
| TC-E2-P1 … P6 | E2, 2 scenes | P1–P6 (P4 at indoor speeds) | 3 each | 1 each | Pick 2 scenes after gate G2 |
| TC-E3-P2, P3, P5 | E3 | P2, P3, P5 | 3 each | 1 each | Only if E2 fails G2 |
| TC-REG | E0 | Pegasus loop | 3 | 1 | Regression: E0 numbers must stay within 10 % of today's |

Ablations on SUT-A, on TC-E1-P3 only:
- A1: front camera only, vs 4 cameras.
- A2: IMU noise as configured, without the ×2 inflation.
- A3: clean IMU (vibration off).

Ablation on SUT-B, on TC-E1-P3 only:
- B1: Performant mode, vs Fast.

## 5. Procedure (per test case)

Gates (each must pass before any estimator runs):

| Gate | Check | Pass |
|---|---|---|
| G1 data | Render `unsettled_frames` = 0; KLT reprojection of fisheye and stereo tracks against GT poses (as done on Cesium) | Median ≤ 0.1 px |
| G2 lens | Fisheye on the environment renders a full 180° circle that matches the OpenCV model (NCC against a reprojected pinhole render, as done on Cesium) | NCC ≥ 0.9; E2 only matters if ParticleField honours the lens |
| G3 flight | Pegasus tracking error and IMU integration drift (`pegasus_post.py`) | Tracking ≤ 0.5 m RMS; drift ≤ 0.5 m over the flight; no collision; clearance ≥ the profile minimum |
| G4 stereo | Disparity distribution on the stereo pair | Median ≥ 2 px (otherwise the test measures the baseline choice, not MAC-VO) |

Run steps:
1. Generate the trajectory: `f1_trajectory.py`, then `pegasus_fly.py`, then `pegasus_post.py`.
2. Check gate G3.
3. Fisheye render: lockstep with `--ros2` while OpenVINS records (×3 seeds; the render is reused only if OpenVINS runs offline from a bag). Then `eval_vio.py`.
4. Stereo render: lockstep with `--stereo --save-all`. Check gate G4. Run MAC-VO, then score it with an SE(3)/Sim(3) eval.
5. Videos (`rig_video.py`, stereo mp4). Delete frames afterwards, except the stereo frames until MAC-VO is done.
6. Append the results row to `build/tests/results.csv`.

Make OpenVINS repeats cheap: record the ROS 2 topics to a bag once per flight, then replay the bag for each seed. This needs a second IMU stream per seed in the bag, or a replay node that swaps the IMU. Without a bag, every repeat costs a full render.

## 6. Metrics and pass criteria

| Metric | SUT-A (VIO) | SUT-B (VO) |
|---|---|---|
| Alignment | 4-DoF (yaw + translation) | SE(3); also report the Sim(3) scale |
| ATE RMSE | ≤ 1.0 % of path (P2, P3) | ≤ 2.0 % of path (P2, P3) |
| Final drift | ≤ 1.0 % of path | ≤ 3.0 % of path |
| RPE over 10 m segments | report | report |
| Rotation RMSE | ≤ 2° | ≤ 3° |
| Scale error \|s − 1\| | n/a (observable through the IMU, reported via ATE) | ≤ 5 % |
| Init time (from first frame) | ≤ 6 s (the hover is 4 s) | first pose ≤ 1 s |
| Failures | Divergence = ATE > 10 % of path, or a filter reset | Tracking lost |
| Run-to-run spread | Std of ATE over 3 seeds ≤ 25 % of the mean | n/a |
| Throughput | Render fps and VIO real-time factor (logged) | MAC-VO fps (logged) |

- **P4–P6:** report only, no pass/fail. They find the breaking point.
- **P7:** pass if the ATE of the low-sun and overcast runs is within 1.5× the noon ATE.

For reference, E0 today (Pegasus loop, one run each):
- SUT-A: 7.5 m ATE = 0.89 % of 846 m, drift 0.43 %. Passes the VIO targets.
- SUT-B: 9.4 m = 1.1 %, drift 2.0 %, scale 0.97. Passes, but on weak depth, so it is not a fair stereo test.

## 7. Cost estimate (RTX 5080)

| Step | Per test case |
|---|---|
| Pegasus flight | ≈ 7 × flight duration (0.14× real time), e.g. 60 s, about 7 min |
| Fisheye render + OpenVINS | ≈ 6 frames/s wall: 60 s flight, about 5 min (plus tile/asset warm-up) |
| Stereo render | ≈ 6 frames/s wall: about 5 min |
| MAC-VO Fast | ≈ 9 frames/s: 60 s flight, about 3.5 min |

Totals:
- E1 (7 profiles + 2 extra lighting renders + ablations): about 3 h of GPU time without bags, about 2 h with bags.
- E2 (2 scenes × 6 profiles): about 3 h.

## 8. Deliverables

- `build/tests/results.csv`: one row per run (env, profile, SUT, seed, metrics, pass/fail).
- `build/tests/report.md`:
  - Pass/fail matrix.
  - ATE-vs-profile plots per environment.
  - Breaking points for P4–P6.
  - Run-to-run spread.
- Videos for each test case: fisheye grid and stereo.

## 9. Open items and risks

- **E2 access:** you need to log in to Hugging Face and accept the dataset terms. I can't do that for you. Without it, E2 becomes E3.
- **Fisheye on ParticleField (G2):** if RTX ignores the OpenCV fisheye lens for Gaussian splats, SUT-A can't run on E2 in fisheye form.
  - Fallback: render the 4 cameras as wide pinholes (120°) and run OpenVINS with the radtan model.
- **Collisions:** Rivermark colliders plus Pegasus mean a bad trajectory crashes the drone. The clearance check in G3 catches that before rendering.
- **Lighting changes (P7):** only E1 supports them. NuRec appearance is baked in.
- **Repeats:** 3 seeds without bags triples the render time. Building the bag replay (5.6) first is worth it.

## 10. Changes made during execution

- **E1 altitudes:** P2 and P3 fly at 32 m and 40–44 m. The probe found roofs up to 22 m and 31 m in their footprints, above the drafted 20–35 m band.
- **Pegasus collisions:** Pegasus flies in an empty stage, not inside Rivermark. Rendering the whole town every physics update made the physics about 100× slower. The G3 clearance check on object bounding boxes (≥ 5 m) replaces collision detection.
- **E2 scenes:** E2 is one scene. The NuRec dataset's v0.2 scenes render blank on Isaac 6.0.1 (they target 6.1), so only the XGRIDS wormhole hall (older export) is used. It is flown with P1–P3 only, since its free space is small.
- **Repeat runs:** the ROS 2 bag replay (section 5) is built and used. Seed 7 runs live while the bag records; seeds 101 and 102 replay it.
- **G3 IMU check:** this is now the worst 10 s window. Whole-flight drift grows with t² from sub-0.1° attitude errors under PX4's fast control.
- **Added cases:**
  - TC-E1-P3-px4: the same loop flown by PX4 with motor lag and 4 m/s wind plus gusts.
  - Coaxial Spirit-class flights under ArduPilot and PX4 (not yet scored).
