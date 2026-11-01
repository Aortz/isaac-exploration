# Changelog

What changed in this exploration, newest first. Commits are on `main`; the evidence for the test results is in [`demo/`](demo/README.md).

## 2026-11-01 (later): environments 4-65x faster to render, with more detail

- **Benchmark** (`envbuild/bench.py`): renders the real 4-fisheye rig for each environment's loop flight and checks it against a budget of 300 ms per frame and 9 GB of GPU memory. It can switch parts of a scene off, drop the haze or add renderer settings, to find what costs the time. Results: [`docs/experiments/env_performance.md`](docs/experiments/env_performance.md).
- **The cost was one tree model.** NVIDIA's `Black_Oak.usd` cost ~20x any other tree; removing it took the rainforest from 4.6 s to 0.37 s per frame. Haze, terrain, water and every renderer setting tried changed frame time by under 2 %.
- **After the changes:** rainforest 4572 → 70 ms, forest v2 1046 → 285 ms, swamp 685 → 70 ms, OSM cities ~300 → ~50 ms, oil palm 240 → 74 ms. Every built environment is now within budget.
- **Performance changes:**
  - Black_Oak replaced by American_Beech (new per-asset `asset_scale`); cheaper conifer mix in forest v2.
  - Level of detail around the flight path (`detail:` + `lod: near_only | thin`).
  - Shared instanceable mangrove crowns.
  - Static environments warm up once instead of at every 10th pose (85–520 s → 8–48 s).
- **Fidelity added within the budget:**
  - Nature scenes: trees in stands and gaps, lean, shrubby forest edges, irregular ground-material edges, fallen logs, frond piles, driftwood, mangrove breathing roots. Steep forest slopes read as soil, not pale rock.
  - OSM cities: varied windows (curtains, blinds, lit rooms, AC units), facade tints, shopfront ground floors, rooftop AC units / tanks / lift cores, lane markings, sidewalks, street lights, darker roofs.
- **Flights:** forest v2 loop raised from 68 to 72 m (the new tree layout left 4.1 m clearance at 68 m; now 12.2 m). Other loops re-checked: 6.3–20 m clearance.
- **Swamp chase video:** re-recording at 1080p on the improved scene (added to the demo when it finishes).

## 2026-11-01: environment builder v2, Singapore / Malaysia / urban / battleground / military scenes

### Environment builder (`envbuild/`)
- **Procedural terrain** (`envbuild/procedural.py`):
  - ground material layers per terrain quad, by mask, height and slope (`terrain.layers`);
  - meandering creeks and valleys at any angle (`channels`), flat-bottomed rivers (`floor_m`), steep trenches (`flat_frac`), a coastline falling into open water (`shore`), contour terraces, shell craters;
  - a flat water plane (`water`); the heightmap sidecar treats it as the ground for clearance.
- **Scatter:** planted grids (`layout: grid`, rows can face along the row), walls along a rectangle with a gate (`layout: perimeter`), masks from roads / craters / rectangles, combined masks (`[town, "!street"]`).
- **Generated assets** (no such assets ship with Isaac):
  - mangroves: arching prop roots plus a crown borrowed from an existing tree, with the crown's fruit hidden;
  - battleground and military props (`envbuild/props.py`): ruined buildings, rubble, armoured-vehicle stand-ins, shipping containers, HESCO walls, jersey barriers, sandbags, tents, guard towers, dead trees.
- **Street-level cities from OpenStreetMap** (`envbuild/osm.py`, `source: osm`): building outlines and storey counts extruded with generated window-grid facades, ear-clipped roofs, road ribbons, park and street trees, colliders, and building / tree boxes for the clearance gate.
- **Street loop planner** (`envbuild/plan_street_loop.py`): finds a loop around a city block on the road graph, rounds the corners into flyable curves, rejects slivers, checks clearance and prints waypoints.
- **Previews** (`envbuild/preview.py`): oblique shots from drone heights, to judge a scene before flying it.
- **Haze:** environments can carry renderer settings (`kit_args`); distance fog hides the terrain edge.

### New environments (each has a P3 loop flight and a test case; none tested yet)
- **Singapore:** procedural mangrove swamp (Sungei Buloh type); Google tiles over Sungei Buloh, the Central Catchment rainforest and Tampines HDB estates; Tampines street level from OpenStreetMap.
- **Malaysia:** oil palm estate on terraced hills; lowland rainforest with a river; Google tiles over KLCC and Johor Bahru; Bukit Bintang (KL) and JB city centre street level from OpenStreetMap.
- **Battleground:** bombed town; trench field with shell craters.
- **Military:** forward operating base, desert and jungle versions.
- **Forest valley v2:** summer species only, closed canopy, understory, meadow grass, layered ground, haze.

### Fixes
- **Warm-up:** environments with slow-loading ground materials settle once before the first frame (`first_warmup_updates`), instead of 400 updates at every warm-up pose. The first swamp case never reached frame 0 because of this.
- **Timeouts:** `run_case.py` allows a render 6 h (heavy scenes timed out at 2 h and left the render running); flights get 2 h.
- **Forest v2** was thinned (110k to 43k instances) after its 4-camera render ran out of GPU memory.
- **Demo:** `package_demo.py` adds environment previews and Google-tile views, and skips videos that are still being written.

### OpenVINS multi-camera finding
- Four separate mono cameras make OpenVINS non-repeatable on the same recorded bag; one camera is repeatable. The init crash at equal camera timestamps is patched (`ros2/openvins/Dockerfile`). Details: [`docs/experiments/openvins_multicam.md`](docs/experiments/openvins_multicam.md).

### Known issues
- The test queue for the new environments was stopped. Heavy scenes render the 4-fisheye rig at about 1.4 s per frame and nearly fill the 16 GB GPU. Next: a render benchmark, then a fidelity pass within about 0.3 s per frame.
- Isaac quirks found: RTX fog's height axis defaults to +Y (a wall of fog across y = 0); deactivated prims inside instancer prototypes still render; Concrete_Rough and Rough_Gravel render near-white.

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
