# Environment recipes

Each file is one environment: `$ISAAC_ROOT/python.sh envbuild/build_env.py --recipe envbuild/recipes/<file> --register`.

| source | needs | example |
|---|---|---|
| procedural | terrain + scatter groups + sky; optional ground `layers`, `channels` (creeks), `shore`, `water` | `forest_valley_v2.yaml`, `desert_dunes.yaml`, `sg_mangrove.yaml` |
| import | `usd:` path (USD/USDZ; convert glTF/OBJ with Isaac's asset converter first), `up_axis`, `meters_per_unit`, `add_colliders`, `ground_plane` | `import_example.yaml` |
| nurec | `usd:` (3DGRuT / NuRec export), `occupancy:` map | `nurec_xgrid_wormhole.yaml` |
| cesium | `lat`, `lon`, `height`, `tiles: google\|osm` (needs CESIUM_ION_TOKEN) | `sg_sungei_buloh.yaml`, `sg_central_catchment.yaml`, `cesium_mojave.yaml` |
| osm | `lat`, `lon`, `radius_m`, `facades` (window-grid styles), `default_heights`, `style_by_type`, `trees` | `sg_tampines_osm.yaml`, `my_kl_bukit_bintang_osm.yaml`, `my_jb_osm.yaml` |
| photogrammetry | `site_config:` for the repo's ODM pipeline (run_all.sh) | see `config/site.yaml` |

Scatter assets: NVIDIA vegetation (`/NVIDIA/Assets/Vegetation/{Trees,Shrub,Rocks,Plant_Tropical}`), SimReady sets
(`/Isaac/SimReady/{Residential,Industrial,Hospital}`), or any USD path. `obstacle: false` keeps small items out of
the flight clearance check.

Procedural extras:
- `terrain.layers`: ground material per terrain quad, chosen by `mask` (`!name` = outside it), `height_range_m`
  and `slope_range_deg`; the last matching layer wins, `terrain.material` is the base.
- `terrain.channels`: meandering creeks / valleys (`angle_deg`, `offset_m`, `depth_m`, `width_m`, `meander_m`,
  `meander_wavelength_m`, `phase_deg`); `terrain.shore` drops the land into open water along a direction.
- `water`: a flat water plane at `level_m` (no collider; the heightmap sidecar treats it as the ground).
- `generate: mangrove` on a scatter group writes mangrove trees (arching prop roots + a crown from `crown_asset`)
  into `build/envs/<name>/assets/` instead of listing `assets`.
- `terrain.terraces` (contour benches), `terrain.craters` (bowl + rim, seeded), channels with `floor_m` (flat-bottomed
  river that a flat `water` plane fits) or `flat_frac` (steep-sided trenches).
- masks: fBm cover (`scale_m`, `cover`), `channels: true` (near roads / creeks / trenches), `craters: true`,
  `rect_m` + `centre_m` (a compound); a group's `mask` can be a list (all must hold), `!name` negates.
- scatter `layout: grid` (planted rows: `spacing_m`, `row_angle_deg`, `jitter_m`, `align: true` to face along rows)
  and `layout: perimeter` (walls along a rectangle `rect_m` with a `gate_m` gap; yaw follows the side).
- `generate: <type>` writes props instead of listing assets (envbuild/props.py): `ruin`, `rubble`, `vehicle_proxy`,
  `container`, `hesco`, `jersey_barrier`, `sandbags`, `tent`, `tower`, `dead_tree`.
- `environment.kit_args`: renderer settings for every run in the environment, e.g. `--/rtx/fog/*` distance haze.

Preview before flying: `$ISAAC_ROOT/python.sh envbuild/preview.py --env <name>` -> `build/envs/<name>/preview.jpg`.

Street-level flights in an `osm` environment: `uv run --with networkx python envbuild/plan_street_loop.py --env <name> --z 30`
finds a loop around a block on the road graph, rounds the corners, checks clearance against the building and tree
boxes, and prints the waypoints for config/flight.yaml.
