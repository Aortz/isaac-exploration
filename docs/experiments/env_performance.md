# Environment render cost on the 4-fisheye rig (2026-11-01)

The built environments (swamp, forest v2, rainforest) rendered the test rig at up to 4.6 s per frame and nearly filled the 16 GB GPU, so their test cases were stopped. This records what the cost was, what fixed it, and the fidelity added within the budget.

- **Budget:** median frame ≤ 300 ms and peak GPU memory ≤ 9 GB. The first forest (`forest_valley`, 0.28 s) set the bar.
- **Measurement:** `envbuild/bench.py` runs the real rig (`flight/rig_fisheye.py`, lockstep, 4 × 640 × 480 fisheyes, no ROS) for the first 45–90 frames of each environment's loop flight and samples GPU memory once a second. Rows go to `build/envs/bench.csv`.
- **Experiments:** `--deactivate <prim paths>` switches parts of a scene off through a layer over it, `--no-fog` drops the haze, `--kit-arg` adds a renderer setting.

## Before and after

| Environment | Before: frame / GPU memory | After: frame / GPU memory | Instances before → after |
|---|---|---|---|
| Rainforest | 4572 ms / 14.9 GB | **70 ms / 5.2 GB** | 57,821 → 20,081 |
| Forest v2 | 1046 ms / 12.2 GB | **285 ms / 6.6 GB** | 43,387 → 22,260 |
| Swamp | 685 ms / 10.1 GB | **70 ms / 5.3 GB** | 12,584 → 7,577 |
| Tampines streets (OSM) | 341 ms / 6.4 GB | **50 ms / 4.1 GB** | buildings + 1.9k trees |
| KL streets (OSM) | 292 ms / 6.1 GB | **53 ms / 4.2 GB** | |
| JB streets (OSM) | 274 ms / 6.1 GB | **51 ms / 4.1 GB** | |
| Oil palm | 240 ms / 6.0 GB | **74 ms / 4.7 GB** | 37,069 → 28,107 |
| Jungle military base | – | **60 ms / 4.6 GB** | 13,115 |
| Forest v1 (unchanged reference) | 276 ms / 5.8 GB | 273 ms / 5.7 GB | 9,147 |
| Desert dunes (unchanged reference) | – | 43 ms / 3.9 GB | 605 |

All "after" rows include the fidelity additions below; the "after" scenes have more kinds of objects than before.

## What cost the time

| Experiment (frame median) | Result |
|---|---|
| Forest v2, understory + meadow + rocks off | 1046 → 917 ms |
| Forest v2, all conifers off | 1046 → 595 ms |
| Forest v2, all broadleaf trees off | 1046 → 591 ms |
| Forest v2, **only Black_Oak off** (~1.7k of 43k instances) | **1046 → 614 ms** |
| Forest v2, each other broadleaf species off | 0–40 ms each |
| Forest v2, each conifer species off | 46–162 ms each (Norway spruce and Douglas fir the most) |
| Rainforest, **Black_Oak canopy off** | **4572 → 372 ms** |
| Swamp, mangroves off (a third had Black_Oak crowns) | 685 → 208 ms |
| Haze off, terrain off, water off | < 2 % change |
| Renderer: subsurface, translucency, indirect diffuse, ambient occlusion, reflections, shadows, direct lighting off, per-view TLAS | < 2 % change each |

- **One asset was almost the whole cost.** NVIDIA's `Black_Oak.usd` costs about 0.25 ms per instance per frame on this rig, roughly 20× the other broadleaf trees. Its cost tracks geometry visibility, not shading: no renderer setting moved the frame time. It was the canopy tree of the rainforest, a mangrove crown, the back-mangrove forest, the oil palm estate's remnant forest, the military base's jungle and every OSM street tree.
- **Renderer settings are not a lever here.** Turning off any lighting feature changed the frame time by under 2 %, so the visual quality of lighting was kept as it was.

## Changes

**Performance**
- Black_Oak replaced by American_Beech everywhere, scaled to the same size with the new per-asset `asset_scale` (Beech is modelled at 6 m, Black_Oak at 18 m). The swamp's open mangrove crowns use Beech too.
- Forest v2 conifer mix weighted towards hemlock and white pine (cheapest per tree); Douglas fir and Norway spruce stay, at lower weight.
- Level of detail around the flight path (`detail:` in a recipe): groups marked `lod: near_only` (undergrowth, grass, debris) exist only within 150 m of the flight path; `lod: thin` groups (trees) thin to 30–40 % beyond 500 m. The haze hides the change.
- Mangrove crowns are one shared instanceable asset per crown type, instead of one copy per variant.
- Static environments warm up once at the start pose instead of at every 10th pose of the path (`flight/rig_fisheye.py`); that pass exists for Cesium tile streaming. Warm-up fell from 85–520 s to 8–48 s.

**Fidelity (build-time only)**
- Trees in natural stands and gaps (`clumping`), slight lean (`tilt_deg`), shrubby forest edges (`edge_of` masks), irregular boundaries between ground materials (`terrain.layer_noise`).
- Forest floor debris: fallen logs (`generate: log`), frond piles in the oil palm rows, driftwood and mangrove breathing roots (`generate: pneumatophores`) in the swamp.
- OSM cities: facades with 4 × 3 window tiles and per-window variety (curtains, blinds, lit rooms, AC units), 3 colour tints per facade style, shopfront ground floors with signage, rooftop AC units / water tanks / lift cores, roads with lane markings, raised sidewalks, street lights.

## Not covered

- Battleground and military scenes were already cheap (the jungle base: 60 ms) and were not reworked beyond the Black_Oak swap.
- The benchmark uses the first 45–90 frames of each loop; a full flight can cross denser parts of a scene.
