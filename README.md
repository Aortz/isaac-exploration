# Photogrammetry to Isaac Sim pipeline

This pipeline turns overlapping drone photos into an Isaac Sim 6.0 test environment. Each run then checks the environment and writes a validation report. v1 runs on the public OpenDroneMap Aukerman sample (77 nadir photos with GPS EXIF).

```
photos -> ODM (docker) -> localize (pyproj) -> Blender master (clean, collision, fixture) -> USD
       -> Isaac scene (colliders, physics, lights) -> scripted sensor rig + probes -> report
```

## Requirements

| Tool | Default location (override in `env.sh`) |
|---|---|
| Isaac Sim 6.0.1 standalone | `~/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64` |
| Blender 5.2 | `~/Downloads/blender-5.2.0-linux-x64/blender` |
| Docker | image `opendronemap/odm:3.6.2`, pinned by digest in `config/site.yaml` |
| uv | creates the Python env from `pyproject.toml` (`uv sync`) |

## Run

```bash
./run_all.sh          # every stage; writes reports/<timestamp>/ and links reports/latest
./run_all.sh 03       # start from stage 03
FORCE=1 bash pipeline/02_reconstruct.sh              # rebuild ODM (skipped when outputs exist)
RERUN_FROM=odm_meshing bash pipeline/02_reconstruct.sh   # resume a failed ODM run
```

| Stage | Script | Interpreter | Output |
|---|---|---|---|
| 00 | `pipeline/00_check_env.sh` | bash | tool, GPU, disk and Isaac headless checks |
| 01 | `pipeline/01_fetch_sample.sh` | bash | `datasets/aukerman/images` |
| 02 | `pipeline/02_reconstruct.sh` | ODM docker | `datasets/odm_project/code/` |
| 03 | `pipeline/03_localize.py` | uv | `build/site_frame.json`, `build/localized/site.obj` |
| 04 | `pipeline/04_blender_prep.py` | Blender | `build/site_master.blend`, `build/usd/site.usd`, `build/reference/*.obj` |
| 05 | `pipeline/05_compose_isaac.py` | Isaac `python.sh` | `build/isaac/site_env.usd` |
| 06 | `pipeline/06_capture.py` | Isaac `python.sh` | `reports/<run>/frames/`, `capture.json` |
| 07 | `pipeline/07_validate.py` | uv | `reports/<run>/report.md`, `results.json` |

Open the environment in the GUI with `$ISAAC_ROOT/isaac-sim.sh`, then File > Open `build/isaac/site_env.usd`.

## Coordinate frame

The local frame is ENU (X east, Y north, Z up), in metres, with the origin near the site centre. ODM's `*_geo.obj` is already shifted by the offset in `odm_georeferencing/coords.txt`. Stage 03 applies one more translation and records both in `build/site_frame.json`. Use `pipeline/common/frame.py` to convert to and from UTM and WGS84. Heights keep the camera's EXIF datum and are not converted.

## Scene layout (`site_env.usd`)

| Path | Contents |
|---|---|
| `/World/Site/Visual` | Cleaned textured mesh (render only) |
| `/World/Site/Collision` | Decimated copy (ratio 0.5), plus a DTM ground underlay 5 cm below the DTM that closes mesh holes. Static triangle-mesh colliders, invisible |
| `/World/Site/Fixtures` | Synthetic wall with a 1 m gap (rendered and collides) |
| `/World/PhysicsScene`, `/World/PhysicsMaterials` | Gravity and contact material |
| `/World/Lighting` | Sun (DistantLight) and DomeLight |
| `/World/Robots` | Empty; scenarios add vehicles here |

RTX cameras and RTX LiDAR see the **visual** mesh. PhysX contacts and scene queries see the **collision** mesh. The report measures how far apart the two drift.

## Validation

All thresholds are in `config/site.yaml` under `validate:` and are fixed before a run. The checks:

- frame round-trip
- reconstruction georeference and scale against camera GPS
- Isaac bbox against the Blender bbox
- texture path resolution
- depth and LiDAR returns against the visual mesh
- sphere contact against the collision mesh
- visual against collision height drift
- PhysX rays through and beside the fixture gap
- throughput (recorded only)

## Platform notes (Ubuntu 26.04)

- `omni.kit.asset_converter` does not load because it needs `libxml2.so.2`. Its errors in Isaac logs are harmless, since the pipeline converts to USD in Blender.
- RTX `get_data()` returns views of GPU buffers that later frames reuse. Copy them with `.numpy()` immediately.

## Known limits of the sample

- Nadir-only imagery, so facades and overhangs are weak. Only the synthetic fixture tests openings.
- The ODM mesh has holes under tree canopy (about 25k boundary edges). Collision is closed by the DTM underlay, but RTX sensors still see through the holes.
- Textures have baked-in capture-time lighting.
- There are no ground control points, so absolute accuracy is GPS-level. These checks prove the pipeline mechanics, not survey accuracy.

## Cesium globe (city-scale base layer)

Cesium for Omniverse v0.29.0 (built for Isaac Sim 6.0.1) streams 3D Tiles into Isaac, georeferenced so a chosen lat/lon becomes USD (0, 0, 0) with X east, Y north and Z up, in metres.

```bash
bash cesium/install_cesium.sh                         # into third_party/ (gitignored)
$ISAAC_ROOT/python.sh cesium/build_cesium_stage.py --site marina_bay --source google            # write build/cesium/marina_bay_google.usda
CESIUM_ION_TOKEN=... $ISAAC_ROOT/python.sh cesium/build_cesium_stage.py --site marina_bay --source google --verify   # stream headless + screenshot
bash cesium/launch_isaac_cesium.sh                    # GUI with Cesium loaded
```

- Sources (`--source`):
  - `google`: Google Photorealistic 3D Tiles, ion asset 2275207
  - `osm`: Cesium World Terrain + OSM Buildings
  - `url`: any `tileset.json`
  - `sample`: a public test tileset that needs no token
- Sites are `marina_bay`, `changi` and `jurong_west`, or pass `--lat/--lon/--height`. Height is WGS84 ellipsoidal; Singapore ground sits around +7 to +8 m.
- The ion token is never written to the USD file. In the GUI, sign in through Window > Cesium.
- Cesium has to be enabled at launch. Use the launcher; enabling it later from the Extensions window leaves its USD schemas unregistered.
- Cesium tiles are render-only. They have no PhysX colliders.

## Drone flight through Cesium, then Cosmos 3 variations (VIO data)

A scripted drone flies through Cesium tiles. Each rendered clip is then re-rendered by NVIDIA Cosmos 3 (Cosmos3-Nano transfer, conditioned on depth + edge) under different weather and lighting. Every variant shares one ground truth: camera poses, IMU and calibration. A variant is exported only if it still moves with the true geometry.

```bash
cosmos/run_cosmos.sh marina_bay_pass                   # f0..f6 (needs CESIUM_ION_TOKEN + NVIDIA_API_KEY)
COSMOS_BACKEND=dry_run cosmos/run_cosmos.sh sample_orbit f1   # no-token pipeline test (colour grades, marked synthetic)
uv run python cosmos/f4_generate.py --flight marina_bay_pass --conditions clear_noon --seeds 1 --frames 49   # small API probe
```

| Stage | Script | Output |
|---|---|---|
| f0 | `cesium/build_cesium_stage.py` | `build/cesium/<site>_<source>.usda` |
| f1 | `flight/f1_trajectory.py` | `build/flight/<flight>/` camera poses (TUM, OpenCV frame + USD frame), 200 Hz IMU (noisy with motor vibration + clean + true biases), body ground truth |
| f2 | `flight/f2_render.py` | `render/frames/` RGB + depth per frame under a cloudy HDR sky (`render:` in the config). Each frame waits until Cesium has no tiles loading |
| f3 | `cosmos/f3_package.py` | `build/cosmos/<flight>/clips/clip_NN/` rgb/depth/edge mp4 (832×480, 30 fps, ≤ 297 frames) + structured prompts |
| f4 | `cosmos/f4_generate.py` | `variants/clip_NN/<condition>_s<seed>.mp4`, content-hash cache in `build/cosmos/cache/` (no repeat billing) |
| f5 | `cosmos/f5_qa.py` | `qa/report.md`: edge F-score, warp-error ratio with ground-truth depth + poses, KLT reprojection vs ground-truth poses, contact sheets |
| f6 | `cosmos/f6_export.py` | `build/vio_datasets/<flight>/clip_NN/<variant>/mav0/` in EuRoC layout |

- **Keys:** put `CESIUM_ION_TOKEN` and `NVIDIA_API_KEY` in a `.env` file in the project root (gitignored); `env.sh` loads it.
- **Motion model:** the drone is a kinematic flat-output trajectory (no motors, drag, wind or controller). The IMU adds thrust-dependent rotor vibration (`imu.vibration`). The noise densities written to `imu0/sensor.yaml` do not include vibration, so inflate them in the VIO config.
- **Hosted Cosmos status (2026-10-29):** `cosmos3-nano` on build.nvidia.com returns 404 / "Function not found for account" for this account, and others report the same on NVIDIA's forum. Until access is granted, use self-hosting or the web Playground.
- **Compute:** Cosmos3-Nano (16B) does not fit on the 16 GB RTX 5080. The default backend is the hosted build.nvidia.com endpoint. The self-hosted path (`vllm_omni`, Cosmos3-Nano-Sim-Transfer on an RTX PRO 6000 or H100) is stubbed in `cosmos/backends.py`.
- **Unconfirmed request fields:** the hosted request uses the documented fields: `model_mode`, `prompt`, `resolution`, `num_frames`, `input_reference` and `transfer.<control>.video`. `seed` and `fps` are sent too but are not confirmed yet. Confirm against the live API reference on the first call.
- **Depth video convention:** the encoding (`package.depth_convention`) is not documented by NVIDIA. Settle it with a near_bright vs near_dark A/B on the first real calls.
- **Licensing:** Google 3D tiles terms may restrict caching and ML use of derived data; check before training on outputs. The hosted Cosmos API runs under NVIDIA's trial terms.

## 4-fisheye rig, rates and ROS 2

`flight/rig_fisheye.py` flies the F1 trajectory with four hard-mounted OpenCV-fisheye cameras (front, left, back and right, 20° down; `rig:` in `config/flight.yaml`). The RTX renderer stops at 90° off-axis, so each image is a 180° circle. Cesium frustum culling is turned off so all four directions get tiles.

```bash
source ros2/ros_env.sh                                   # Isaac's bundled ROS 2 Jazzy, no system ROS needed
$ISAAC_ROOT/python.sh ros2/measure_rates.py &            # subscriber: wall rate, sim-stamp rate, camera sync
$ISAAC_ROOT/python.sh flight/rig_fisheye.py --flight marina_bay_pass --mode lockstep --ros2
$ISAAC_ROOT/python.sh flight/rig_fisheye.py --flight marina_bay_pass --mode freerun --ros2 --max-fps 15
$ISAAC_ROOT/python.sh flight/rig_fisheye.py --flight marina_bay_pass --lag-test    # image-vs-pose lag per update count
```

- **Topics:** `/clock`, `/imu/data` (200 Hz), `/<cam>/image_raw` and `/<cam>/camera_info` (`equidistant`), `/tf`, `/tf_static`, `/ground_truth/odom`. Kalibr-style `camchain.yaml` (readable by OpenVINS) is written next to `rates.json`.
- **lockstep:** sim time advances 1/camera_hz per frame, and only after all cameras have a finished frame. All four images share one stamp, IMU and `/clock` lead the images, and consumers use `use_sim_time`. On the RTX 5080 this ran at 7.7 frames/s wall (31 images/s), which is 30 Hz on the sim clock.
- **freerun:** one renderer update per frame. An image shows the pose set 3 updates earlier (measured with `--lag-test`), so images are stamped with the pose they show (`rig.freerun_lag_frames`).
- **Hard freeze:** an uncapped freerun froze the whole machine once (2026-10-29), with nothing in the kernel log. Use `--max-fps`.

## OpenVINS on the fisheye rig

OpenVINS runs in Docker (ROS 2 Jazzy, pinned commit) and subscribes live to the rig topics. Jazzy dropped some `.h` headers, so the Dockerfile patches them to `.hpp`.

```bash
ros2/openvins/build.sh                                            # once, ~10 min
uv run python flight/f1_trajectory.py --flight marina_bay_loop    # 105 s loop, 4 s hover at the start for static init
uv run python ros2/openvins/make_config.py                        # build/openvins/rig4/ (all 4 cams); --cameras cam0_front for mono
ros2/openvins/run_openvins.sh rig4 build/openvins/rig4/run1 &     # starts OpenVINS, uses /clock
source ros2/ros_env.sh && $ISAAC_ROOT/python.sh flight/rig_fisheye.py --flight marina_bay_loop --mode lockstep --ros2 --save-all
docker kill -s INT openvins                                       # flushes ov_estimate.txt
uv run python ros2/openvins/eval_vio.py --flight marina_bay_loop --run build/openvins/rig4/run1   # ATE (4-DoF aligned), eval.png
uv run python flight/rig_video.py --flight marina_bay_loop --run rig_lockstep                    # grid.mp4 + per-camera mp4s
```

Config choices in `make_config.py`, each forced by this setup:
- **Triangulation limits:** `fi_max_dist` is 1500 m and `fi_max_baseline` is 400. The defaults (60 m, 40) suit indoor rigs, and every feature seen from 100 m up is farther than 60 m, so all of them would be discarded.
- **Feature detection:** the grid is 10×8 and the FAST threshold is 10. OpenVINS keeps the strongest corners in each cell before applying the mask, and on a fisheye those are the bright rim of the image circle, which the mask then rejects. The bay water also has little texture.
- **Mask:** each mask keeps features within 80° of the optical axis.
- **Initialisation:** ZUPT is on at the start with `init_max_disparity` of 2 px, so OpenVINS initialises during the start hover. Static init would otherwise wait for an acceleration jerk, and a slow take-off at altitude never produces one.
- **IMU noise:** the noise densities fold in the measured motor vibration, then are doubled for margin.

## Pegasus quadrotor dynamics (flight `marina_bay_loop_pegasus`)

By default F1 is kinematic: a spline with a flatness attitude, and the IMU is computed from it. With `dynamics: pegasus`, the spline becomes the reference for a simulated Iris quadrotor, and the ground truth and IMU come from what the vehicle actually flew. The vehicle is from [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator): PhysX rigid body, quadratic rotor thrust curve, rotor drag torque, linear body drag and Pegasus' geometric tracking controller. Settings are in `pegasus:` in `config/flight.yaml`.

```bash
# once: Pegasus' Isaac Sim 6.0 port is PR #144 (upstream dev_6.0.1 still imports the removed omni.isaac.dynamic_control)
git clone https://github.com/PegasusSimulator/PegasusSimulator third_party/pegasus
git -C third_party/pegasus fetch origin pull/144/head:pr144 && git -C third_party/pegasus checkout fcb99c0

uv run python flight/f1_trajectory.py --flight marina_bay_loop_pegasus       # reference_1000hz.csv
$ISAAC_ROOT/python.sh flight/pegasus_fly.py --flight marina_bay_loop_pegasus  # 1 kHz physics, ~13 min; no Cesium, no token needed
uv run python flight/pegasus_post.py --flight marina_bay_loop_pegasus        # ground truth + IMU in the F1 layout
```

After that, every rig, OpenVINS and stereo command takes `--flight marina_bay_loop_pegasus` unchanged. The cameras are rendered afterwards by replaying the flown poses in lockstep. The images come out the same as rendering during the flight, because the controller does not use them.

- **What changes vs kinematic:**
  - **Tilt:** the drone tilts up to 27.6° instead of 15°, because it has to lean into body drag at 12 m/s.
  - **Tracking:** the drone follows the reference to within 0.24 m RMS (0.48 m max).
- **IMU:**
  - **Accelerometer:** specific force from the PhysX velocity.
  - **Gyro:** comes from successive PhysX attitudes, not PhysX's angular velocity. On the Iris articulation, PhysX's angular velocity is biased a few mrad/s against its own attitude, which integrates to about 1.5 km of drift over the flight. The attitude-based gyro drifts 0.14 m.
  - **Sampling and noise:** both are averaged over each 5 ms IMU period, then get the same vibration and noise model as F1.
- **Pegasus fix:** Pegasus reads the attitude from the USD xform, which PhysX only writes back once per rendered frame. `pegasus_fly.py` reads it from the PhysX body every step instead.
- **OpenVINS on this flight** (rig4, same settings): ATE RMSE 7.5 m over 846 m, final error 3.6 m (0.43%), rotation RMSE 2.8°. The kinematic loop gave 26 m.

## Stereo pair and MAC-VO

`rig_fisheye.py --stereo` renders a pinhole stereo pair instead of the fisheyes (`stereo:` in `config/flight.yaml`). The pair uses TartanAir geometry, which is what MAC-VO was trained on: 640×480 at fx 320, a 0.5 m baseline, facing forward and pitched 30° down. Next to the frames it writes `stereo.json`, `times.csv` and `groundtruth_left.csv`.

[MAC-VO](https://github.com/MAC-VO/MAC-VO) (learned stereo VO, no IMU) runs in Docker. The image is torch 2.7.1 with CUDA 12.8, which the RTX 5080 (sm_120) needs. `macvo/isaac_sequence.py` adds a loader for the format above.

```bash
# once
git clone --recursive https://github.com/MAC-VO/MAC-VO third_party/macvo     # needs the FlowFormer submodule
mkdir -p third_party/macvo/Model && for f in MACVO_FrontendCov.pth MACVO_posenet.pkl; do
  wget -O third_party/macvo/Model/$f https://github.com/MAC-VO/MAC-VO/releases/download/model/$f; done
docker build -t isaac-exploration/macvo:latest macvo/

$ISAAC_ROOT/python.sh flight/rig_fisheye.py --flight marina_bay_loop_pegasus --mode lockstep --stereo --save-all
macvo/run_macvo.sh build/flight/marina_bay_loop_pegasus/rig_lockstep_stereo build/macvo/pegasus_loop_full Fast
uv run python macvo/stress_test.py --seq build/flight/marina_bay_loop_pegasus/rig_lockstep_stereo --counts 1 4 6 7 8 --frames 300
```

- **Accuracy:** on the full 105 s Pegasus loop in Fast mode, MAC-VO ran at 9.1 fps. ATE is 9.4 m after SE(3) alignment, scale is 0.97 and the final error is 16.6 m. MAC-VO's own evaluation reports 20.1 m because it aligns at the first frame only. From 110 m up the median stereo disparity is only 0.4 px, so the stereo depth is weak.
- **Concurrency on the RTX 5080 (16 GB):**
  - **Limit:** 6 instances fit at about 2.5 GiB each, peaking at 15.4 GiB of GPU memory. With 7, three instances failed because the GPU ran out of memory (CUDA OOM, or cuSOLVER could not create its handle).
  - **Throughput:** the total stays at about 10 frames/s whatever the instance count (9.1 with 1, 9.9 with 4, 9.9 with 6), because one instance already keeps the GPU at 70–95%. Extra instances split the GPU rather than add capacity. That is about a third of one 30 Hz stream.
  - **Next to Isaac:** a running Isaac renderer takes about 3 GB of GPU memory, which leaves room for about 4 MAC-VO instances.
- **Host RAM:** `stress_test.py` kills a round if host MemAvailable falls below `--min-free-ram-gib`, because a crash once took down the desktop. An earlier 8-instance round with `--preload` (all frames in RAM per instance) ran the 59 GB machine out of memory, and the kernel OOM killer took the terminal with it.
