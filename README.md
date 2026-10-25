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
| f1 | `flight/f1_trajectory.py` | `build/flight/<flight>/` camera poses (TUM, OpenCV frame + USD frame), 200 Hz IMU (noisy + clean + true biases), body ground truth |
| f2 | `flight/f2_render.py` | `render/frames/` RGB + depth per frame. Each frame waits until Cesium has no tiles loading |
| f3 | `cosmos/f3_package.py` | `build/cosmos/<flight>/clips/clip_NN/` rgb/depth/edge mp4 (832×480, 30 fps, ≤ 297 frames) + structured prompts |
| f4 | `cosmos/f4_generate.py` | `variants/clip_NN/<condition>_s<seed>.mp4`, content-hash cache in `build/cosmos/cache/` (no repeat billing) |
| f5 | `cosmos/f5_qa.py` | `qa/report.md`: edge F-score, warp-error ratio with ground-truth depth + poses, KLT reprojection vs ground-truth poses, contact sheets |
| f6 | `cosmos/f6_export.py` | `build/vio_datasets/<flight>/clip_NN/<variant>/mav0/` in EuRoC layout |

- **Compute:** Cosmos3-Nano (16B) does not fit on the 16 GB RTX 5080. The default backend is the hosted build.nvidia.com endpoint. The self-hosted path (`vllm_omni`, Cosmos3-Nano-Sim-Transfer on an RTX PRO 6000 or H100) is stubbed in `cosmos/backends.py`.
- **Unconfirmed request fields:** the hosted request uses the documented fields: `model_mode`, `prompt`, `resolution`, `num_frames`, `input_reference` and `transfer.<control>.video`. `seed` and `fps` are sent too but are not confirmed yet. Confirm against the live API reference on the first call.
- **Depth video convention:** the encoding (`package.depth_convention`) is not documented by NVIDIA. Settle it with a near_bright vs near_dark A/B on the first real calls.
- **Licensing:** Google 3D tiles terms may restrict caching and ML use of derived data; check before training on outputs. The hosted Cosmos API runs under NVIDIA's trial terms.
