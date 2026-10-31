# Changelog

What changed in this exploration, newest first. Commits are on `main`; the evidence for the test results is in [`demo/`](demo/README.md).

## 2026-10-31 (later): test plan, high-fidelity environments, PX4 / ArduPilot, coaxial vehicle

### Test plan and tooling
- **Test plan:** [`docs/test_plans.md`](docs/test_plans.md) is the plan; [`config/tests.yaml`](config/tests.yaml) holds the thresholds (fixed before the first run) and the test-case list.
- **Test runner** (`testplan/run_case.py`, `testplan/run_all.sh`). For each case it:
  1. checks the flight gate,
  2. runs OpenVINS live on the fisheye render while recording a ROS 2 bag,
  3. replays the bag once per extra IMU noise seed,
  4. runs the image gates, the stereo render and MAC-VO,
  5. scores every run and writes videos.
- **Gates** (`testplan/gates.py`, `testplan/g2_lens.py`, `testplan/clearance.py`):
  - G1: KLT tracks reprojected with ground-truth poses (0.05–0.09 px on clean renders).
  - G2: fisheye lens on a new geometry type, checked against a pinhole render.
  - G3: flight tracking, IMU consistency (worst 10 s window) and clearance from geometry.
  - G4: stereo disparity.
- **Metrics** (`testplan/metrics.py`):
  - 4-DoF / SE(3) / Sim(3) alignment, ATE, final drift, rotation error, RPE over 10 m.
  - Hover flights align heading from attitude, because a hover's path is a point.
- **Report** (`testplan/report.py`): the pass/fail report. `testplan/package_demo.py` collects the evidence into `demo/`.
- **ROS 2 bag record and replay** (`ros2/bag/`): MCAP with zstd. Replay can swap the IMU stream for another noise seed, so OpenVINS repeat runs reuse one render.
- **Field data checklist** ([`docs/field_data_checklist.html`](docs/field_data_checklist.html)): what to collect in the field for higher-fidelity simulation.

### Environments
- **Rivermark** (Isaac 6.0 outdoor town) as the main outdoor environment.
  - `flight/env_probe.py` measures an environment (bounds, object heights, colliders) and renders an orthographic top view used to plan flights.
  - Six Rivermark flight profiles (`rm_p1`–`rm_p6`), each at least 5 m from geometry by an AABB clearance check, with lighting variants.
- **NuRec Gaussian-splat scenes** (`pipeline/common/environments.py`):
  - A `.usdz` is referenced into a new stage, because a `.usdz` opened as the root stage can't take new prims.
  - The scenes need `--/UJITSO/geometry=true`.
  - Only the older XGRIDS export renders on Isaac 6.0.1; the dataset's v0.2 assets need 6.1.
  - Three indoor flights (`nx_p1`–`nx_p3`), with clearance taken from the scene's occupancy map.
- **Rig:** `rig_fisheye.py` renders in any Isaac environment and takes `--lighting` variants and per-environment stereo baseline and clip ranges. The `--stereo` pair and `--gui` arrived in the previous commit.

### Vehicle dynamics
- **Motor lag** (`flight/pegasus_models.py`): first-order rotor lag. It is matched to PX4's Iris model at 12.5 / 25 ms; 35 / 60 ms put PX4 into a 2 Hz limit cycle.
- **Wind:** mean wind, Dryden-like turbulence and 1-cos gusts. Body drag acts on the air-relative velocity, and the wind is logged with the state.
- **PX4 v1.16 SITL** (`px4/Dockerfile`, `flight/px4_offboard.py`):
  - EKF2 and the controllers run on Pegasus' simulated sensors.
  - Offboard flow: take off, hold, then stream the reference.
  - Lockstep fix: Pegasus advanced PX4's clock by one physics step per IMU sample, so with 1 kHz physics and a 250 Hz IMU PX4 ran at a quarter speed and hung.
- **ArduCopter 4.6.3 SITL** (`ardupilot/Dockerfile`) over Pegasus' JSON lockstep link, flown in GUIDED mode.
- **Coaxial helicopter** (`flight/coax_vehicle.py`), sized like the Ascent AeroSystems Spirit:
  - Two counter-rotating rotors, swashplate cyclic, yaw from the rotor-speed difference.
  - The lower rotor's wake loss, servo and flapping lag, and landing gear are modelled.
  - Runs under PX4 (CA_AIRFRAME 12, `px4/airframes/22100_pegasus_coax`) or ArduPilot (FRAME_CLASS 9, `ardupilot/params/coax_spirit.parm`).
  - First ArduPilot flight: 20 s of the P3 loop in wind, hover attitude std 0.09 deg, tracking 0.22 m RMS.
  - Three model bugs found in ground tests and fixed:
    - no landing gear, so the cylinder tipped over before take-off;
    - the rotor spring was active with the rotors stopped;
    - PhysX cylinder colliders were unstable, so they became a capsule and a box.
- **IMU check:** `pegasus_post.py` reports drift over the worst 10 s window as well as the whole flight, and writes extra IMU noise seeds.

### Fixes
- **OpenVINS init crash** (`ros2/openvins/Dockerfile`): OpenVINS could `exit()` at init with four mono cameras that share timestamps (a duplicate clone time). Patched to keep only increasing times.
- **Partial logs:** a failed flight now saves its partial log (`pegasus_state_failed.csv`).

## 2026-10-31: Pegasus quadrotor, stereo pair, MAC-VO (`c0c609b`)
- **Pegasus Iris:** flies F1 references with PhysX dynamics (`flight/pegasus_fly.py`, `flight/pegasus_post.py`). Two fixes:
  - attitude is read from the PhysX body instead of the stale USD xform;
  - the gyro comes from attitude differences, because the PhysX angular velocity is biased.
- **Stereo pair** (`rig_fisheye.py --stereo`) and the GUI flag.
- **MAC-VO** in Docker (torch 2.7 / CUDA 12.8 for sm_120) with a loader for our stereo format.
  - A concurrency stress test with a host-RAM guard: 6 instances fit on the RTX 5080, about 10 fps in total.

## 2026-10-30: OpenVINS on the 4-fisheye rig (`fbbcbd1`)
- **OpenVINS** in Docker (ROS 2 Jazzy), with a config generator and evaluation.
  - Fixes: Jazzy headers, the IMU QoS dropping samples, feature-distance limits, the fisheye mask and ZUPT init.
- **Marina Bay loop flight** and rig videos.

## 2026-10-29: 4-fisheye rig and ROS 2 (`1aeba6e`)
- **Rig and topics:** a 4-fisheye rig with lockstep and freerun modes, ROS 2 topics on sim time, rate measurement and an image-lag test.

## 2026-10-29: Marina Bay render (`9b2035d`)
- **Rendering:** opens Cesium with the token first, a cloudy sky dome, exposure settings and an IMU vibration model.

## 2026-10-25: pipelines for Isaac Sim 6.0.1 (`d90d77d`)
- **Pipelines:** photogrammetry (ODM → Blender → Isaac), Cesium for Omniverse, and the drone flight → Cosmos 3 variation pipeline with QA and EuRoC export.
