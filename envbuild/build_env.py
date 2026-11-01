"""Environment builder: turn a recipe into a sim-ready environment the flight pipeline and test plan can use.

A recipe (envbuild/recipes/*.yaml) names one source, and the builder makes it sim-ready the same way:
Z-up metres, colliders, a ground height, semantic classes where it can, plus sidecars for flight planning.

  procedural     heightfield terrain (material layers, creeks, coastline, water) + PointInstancer scatter of
                 NVIDIA vegetation / rock / SimReady assets or generated ones (mangroves), sky dome + sun
                 (forest, desert, swamp ...)                         -> build/envs/<name>/env.usda
  import         any USD / USDZ (or glTF/OBJ converted to USD first): fix up-axis and units, add
                 triangle-mesh colliders where missing, optional ground slab -> build/envs/<name>/env.usda
  nurec          Gaussian-splat scene (3DGRuT / NuRec export) + occupancy map    -> registered as is
  cesium         Google photoreal / OSM tiles at a lat/lon (cesium/build_cesium_stage.py)
  osm            street-level city from OpenStreetMap: extruded buildings with window-grid facades, roads,
                 street / park trees, colliders              -> build/envs/<name>/env.usda
  photogrammetry the repo's ODM -> Blender -> Isaac pipeline (run_all.sh with the recipe's site config)

After building: build/envs/<name>/env.json (manifest), environment.yaml (the config/flight.yaml block),
heightmap.npz / obstacles.npy when known (testplan/clearance.py uses them), then flight/env_probe.py
renders the top view. --register adds the block to config/flight.yaml `environments:`.

Run: $ISAAC_ROOT/python.sh envbuild/build_env.py --recipe envbuild/recipes/forest_valley.yaml [--register] [--no-probe]
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--recipe", required=True)
ap.add_argument("--register", action="store_true", help="add/replace the environment in config/flight.yaml")
ap.add_argument("--no-probe", action="store_true", help="skip the env_probe top view")
args = ap.parse_args()

import yaml  # noqa: E402  (Isaac's Python ships PyYAML)

RECIPE = yaml.safe_load(Path(args.recipe).read_text())
NAME = RECIPE["name"]
ISAAC = Path(os.environ.get("ISAAC_ROOT", Path.home() / "Downloads/isaac-sim-standalone-6.0.1-linux-x86_64"))
OUT = PROJECT_ROOT / "build" / "envs" / NAME

ENV_DEFAULTS = {"colliders": False, "settle_updates": 4, "warmup_updates": 30, "clip": [0.1, 5000.0],
                "stereo": {"baseline_m": 0.30, "pitch_deg": 20.0}, "openvins": {"fi_max_dist": 300.0},
                "lighting": {"default": {}}}


def build_procedural() -> dict:
    from pxr import Usd, UsdGeom

    import numpy as np
    from envbuild import procedural as pg

    rng = np.random.default_rng(RECIPE.get("seed", 0))
    stage = Usd.Stage.CreateNew(str(OUT / "env.usda"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    tcfg = RECIPE["terrain"]
    h, x0, y0, res = pg.make_heights(tcfg, rng)
    masks = {}
    for mname, m in RECIPE.get("masks", {}).items():
        n = h.shape[0]
        if m.get("channels"):
            # Near the terrain channels (roads, creeks): weight above `threshold`.
            masks[mname] = (pg.channels_field(tcfg, x0, y0, res, n), m.get("threshold", 0.3), m.get("invert", False))
            continue
        if m.get("edge_of"):
            # A band either side of another mask's boundary (forest edges): |field - threshold| small.
            f, thr, _ = masks[m["edge_of"]]
            band = m.get("band", 0.15) * np.std(f)
            masks[mname] = (band - np.abs(f - thr), 0.0, m.get("invert", False))
            continue
        if m.get("craters"):
            masks[mname] = (pg.crater_field(tcfg, x0, y0, res, n), m.get("threshold", 0.05), m.get("invert", False))
            continue
        if m.get("rect_m"):
            # Inside an axis-aligned rectangle (a compound, a town block): centre_m, rect_m.
            xs = x0 + np.arange(n) * res
            X, Y = np.meshgrid(xs, xs)
            (cx, cy), (w, hh) = m.get("centre_m", [0.0, 0.0]), m["rect_m"]
            masks[mname] = ((np.abs(X - cx) <= w / 2) & (np.abs(Y - cy) <= hh / 2)).astype(float), 0.5, m.get("invert", False)
            continue
        # fBm field thresholded to a cover fraction: patches of forest with clearings, oases, etc.
        field = pg.fbm(n, n, m["scale_m"] / res, 4, rng)
        thr = np.quantile(field, 1.0 - m["cover"])
        masks[mname] = (field, thr, m.get("invert", False))
    layers = tcfg.get("layers")
    if layers:
        layers = [{"material": tcfg["material"]}] + layers
    pg.write_terrain(stage, h, x0, y0, res, tcfg.get("material"), tcfg.get("uv_tile_m", 4.0),
                     layers, pg.face_layers(h, res, layers, masks, tcfg.get("layer_noise"), rng) if layers else None)
    # Level of detail around the flight path: groups marked `lod:` are thinned away from it.
    detail = dict(RECIPE["detail"]) if RECIPE.get("detail") else None
    if detail:
        fl = yaml.safe_load((PROJECT_ROOT / "config" / "flight.yaml").read_text())["flights"][detail["corridor"]]
        detail["corridor"] = np.asarray(fl["waypoints"], float)[:, :2]
    water = RECIPE.get("water")
    if water:
        pg.write_water(stage, water)

    def mask_fn(name):
        if not name:
            return None
        if isinstance(name, list):                          # all of them, e.g. [town, "!street"]
            fs = [mask_fn(m) for m in name]
            return lambda x, y: np.logical_and.reduce([f(x, y) for f in fs])
        field, thr, inv = masks[name.lstrip("!")]
        inv = inv != name.startswith("!")                  # "!forest" = outside the forest mask

        def f(x, y):
            v = pg.height_at(field, x0, y0, res, x, y) >= thr
            return ~v if inv else v
        return f

    obstacles, counts = [], {}
    for gname, g in RECIPE.get("scatter", {}).items():
        if g.get("generate") == "mangrove":
            g = {**g, "assets": pg.write_mangrove_assets(OUT / "assets", g, rng)}
        elif g.get("generate"):
            from envbuild import props

            g = {**g, "assets": props.write_assets(g["generate"], OUT / "assets", g, rng, prefix=gname)}
        pts = pg.scatter_points(g, h, x0, y0, res, rng, mask_fn(g.get("mask")), detail)
        if len(pts):
            boxes = pg.write_scatter(stage, gname, g, pts, rng)
            if g.get("obstacle", True):
                obstacles.append(boxes)
        counts[gname] = int(len(pts))
        print(f"  scatter {gname}: {len(pts)} instances", flush=True)
    pg.write_sky(stage, RECIPE.get("sky", {}))
    stage.GetRootLayer().Save()
    # Clearance is measured to whatever is below: the water surface where the terrain is submerged.
    ground = np.maximum(h, water["level_m"]) if water else h
    np.savez(OUT / "heightmap.npz", heights=ground.astype(np.float32), x0=x0, y0=y0, res=res)
    if obstacles:
        np.save(OUT / "obstacles.npy", np.vstack(obstacles))
    half = tcfg["size_m"] / 2
    return {"usd": str((OUT / "env.usda").relative_to(PROJECT_ROOT)), "ground_z": 0.0,
            "bounds_xy": [-half, -half, half, half], "instances": counts,
            "terrain_z_range": [float(h.min()), float(h.max())],
            **({"water_level_m": float(water["level_m"])} if water else {}), "sidecars": ["heightmap.npz", "obstacles.npy"]}


def build_osm() -> dict:
    """Street-level city from OpenStreetMap around (lat, lon): see envbuild/osm.py."""
    from pxr import Usd, UsdGeom

    import numpy as np
    from envbuild import osm
    from envbuild import procedural as pg

    rng = np.random.default_rng(RECIPE.get("seed", 0))
    lat, lon, rad = RECIPE["lat"], RECIPE["lon"], float(RECIPE.get("radius_m", 400.0))
    data = osm.fetch(lat, lon, rad, OUT / "osm.json")
    stage = Usd.Stage.CreateNew(str(OUT / "env.usda"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    floor_m, bay_m = RECIPE.get("floor_m", 3.2), RECIPE.get("bay_m", 3.6)
    styles = RECIPE["facades"]
    blds = []
    for poly, tags in osm.footprints(data, lat, lon):
        if np.abs(poly).max() > rad:
            continue
        h = osm.parse_height(tags, rng, RECIPE.get("default_heights", {}), floor_m)
        # Style: by building type when the recipe maps it, else seeded.
        s = RECIPE.get("style_by_type", {}).get(tags.get("building"), int(rng.integers(len(styles))))
        # Shopfront ground floor: shops tagged, commercial types, and a share of untagged / mixed blocks.
        shop = ("shop" in tags or tags.get("building") in RECIPE.get("shopfront_types", ["retail", "commercial", "hotel"])
                or rng.random() < RECIPE.get("shopfront_frac", 0.15))
        blds.append((poly, h, s, shop))
    boxes = osm.write_buildings(stage, blds, styles, OUT / "textures", floor_m, bay_m, RECIPE.get("roof_material"), rng)
    ways = []
    for el in data["elements"]:
        hw = el.get("tags", {}).get("highway")
        if el["type"] == "way" and hw in osm.ROAD_WIDTH and el.get("geometry"):
            g = el["geometry"]
            ways.append((osm.project(lat, lon, [q["lat"] for q in g], [q["lon"] for q in g]), osm.ROAD_WIDTH[hw]))
    n_roads = osm.write_roads(stage, ways, RECIPE.get("road_material", "/NVIDIA/Materials/Base/Natural/Asphalt.mdl"),
                              tex_dir=OUT / "textures" if RECIPE.get("street_detail", True) else None, rng=rng)
    if RECIPE.get("street_lights_m"):
        # Lamps along the wider roads, alternating sides, arms facing the carriageway.
        from envbuild import props

        sp, lamps = RECIPE["street_lights_m"], []
        for line, w in ways:
            if w < 8:
                continue
            for a, b in zip(line[:-1], line[1:]):
                d = b - a
                L = np.linalg.norm(d)
                if L < 1:
                    continue
                nrm = np.array([-d[1], d[0]]) / L
                for k, t_ in enumerate(np.arange(sp / 2, L, sp)):
                    side = 1 if k % 2 else -1
                    p_ = a + d / L * t_ + side * nrm * (w / 2 + 0.6)
                    lamps.append([p_[0], p_[1], 0.0, np.arctan2(-side * nrm[1], -side * nrm[0])])
        lamps = np.array(lamps).reshape(-1, 4)
        lamps = lamps[np.abs(lamps[:, :2]).max(1) < rad] if len(lamps) else lamps
        if len(lamps):
            lg = {"assets": props.write_assets("street_light", OUT / "assets", {"variants": 1}, rng), "class": "pole",
                  "scale_range": [1.0, 1.0]}
            pg.write_scatter(stage, "street_lights", lg, lamps, rng)
    osm.write_ground(stage, rad + 200.0, RECIPE.get("ground_material", "/NVIDIA/Materials/Base/Natural/Grass_Cut.mdl"))
    # Trees: OSM tree nodes + a scatter over parks / grass, kept clear of buildings and roads.
    trees = [osm.project(lat, lon, [el["lat"]], [el["lon"]])[0] for el in data["elements"]
             if el["type"] == "node" and el.get("tags", {}).get("natural") == "tree"]
    tcfg = RECIPE.get("trees")
    if tcfg:
        parks = [osm.project(lat, lon, [q["lat"] for q in el["geometry"]], [q["lon"] for q in el["geometry"]])
                 for el in data["elements"] if el["type"] == "way" and el.get("geometry")
                 and (el.get("tags", {}).get("leisure") == "park" or el.get("tags", {}).get("landuse") == "grass")]
        sp = 100.0 / np.sqrt(tcfg.get("park_density_per_ha", 60))
        for poly in parks:
            lo, hi = poly.min(0), poly.max(0)
            g = np.stack(np.meshgrid(np.arange(lo[0], hi[0], sp), np.arange(lo[1], hi[1], sp)), -1).reshape(-1, 2)
            g = g + rng.uniform(-sp / 3, sp / 3, g.shape)
            trees += list(g[osm.point_in_poly(g, poly)]) if len(g) else []
        if tcfg.get("street_spacing_m"):
            # Along roads, offset to the verge.
            for line, w in ways:
                for a, b in zip(line[:-1], line[1:]):
                    d = b - a
                    L = np.linalg.norm(d)
                    if L < 1 or w < 6:
                        continue
                    nrm = np.array([-d[1], d[0]]) / L
                    for t_ in np.arange(0, L, tcfg["street_spacing_m"]):
                        for side in (-1, 1):
                            trees.append(a + d / L * t_ + side * nrm * (w / 2 + 2.0))
        trees = np.array(trees).reshape(-1, 2)
        trees = trees[np.abs(trees).max(1) < rad]
        if len(boxes) and len(trees):
            hit = np.zeros(len(trees), bool)
            for bx in boxes:                                          # drop trees inside building boxes
                hit |= (np.abs(trees[:, 0] - bx[0]) < bx[3] / 2 + 1) & (np.abs(trees[:, 1] - bx[1]) < bx[4] / 2 + 1)
            trees = trees[~hit]
        if len(trees):
            tb = pg.write_scatter(stage, "trees", {**tcfg, "class": "tree"},
                                  np.column_stack([trees, np.zeros(len(trees))]), rng)
            boxes = np.vstack([boxes, tb]) if len(boxes) else tb
    pg.write_sky(stage, RECIPE.get("sky", {}))
    stage.GetRootLayer().Save()
    n = int(2 * (rad + 200) / 5) + 1
    np.savez(OUT / "heightmap.npz", heights=np.zeros((n, n), np.float32), x0=-(rad + 200), y0=-(rad + 200), res=5.0)
    np.save(OUT / "obstacles.npy", boxes)
    return {"usd": str((OUT / "env.usda").relative_to(PROJECT_ROOT)), "ground_z": 0.0,
            "bounds_xy": [-rad, -rad, rad, rad], "buildings": len(blds), "road_segments": n_roads,
            "trees": int(len(trees)) if tcfg else 0, "tallest_m": float(max(b[1] for b in blds)) if blds else 0.0,
            "sidecars": ["heightmap.npz", "obstacles.npy", "osm.json"]}


def build_import() -> dict:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    from pipeline.common.environments import asset_url

    src = RECIPE["usd"]
    src_url = asset_url(src)                    # /Isaac/... and /NVIDIA/... resolve to the asset root
    stage = Usd.Stage.CreateNew(str(OUT / "env.usda"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    site = UsdGeom.Xform.Define(stage, "/World/Imported")
    site.GetPrim().GetReferences().AddReference(src_url)
    src_stage = Usd.Stage.Open(src_url)
    mpu = RECIPE.get("meters_per_unit", UsdGeom.GetStageMetersPerUnit(src_stage) or 1.0)
    up = RECIPE.get("up_axis", UsdGeom.GetStageUpAxis(src_stage))
    xf = UsdGeom.Xformable(site)
    if up == "Y":
        xf.AddRotateXOp().Set(90.0)
    if mpu != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(mpu, mpu, mpu))
    added = 0
    if RECIPE.get("add_colliders", True):
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.none)
                added += 1
    if RECIPE.get("ground_plane"):
        g = RECIPE["ground_plane"]
        cube = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
        cube.CreateSizeAttr(1.0)
        cx = UsdGeom.Xformable(cube)
        cx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, g.get("z", 0.0) - 0.5))
        cx.AddScaleOp().Set(Gf.Vec3f(g.get("size_m", 2000.0), g.get("size_m", 2000.0), 1.0))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    stage.GetRootLayer().Save()
    return {"usd": str((OUT / "env.usda").relative_to(PROJECT_ROOT)), "ground_z": RECIPE.get("ground_z", 0.0),
            "source_usd": src, "up_axis_in": up, "meters_per_unit_in": mpu, "colliders_added": added}


def build_nurec() -> dict:
    env = {"usd": RECIPE["usd"], "ground_z": RECIPE.get("ground_z", 0.0),
           "kit_args": ["--/UJITSO/geometry=true"]}
    if RECIPE.get("occupancy"):
        env["occupancy"] = RECIPE["occupancy"]
    return env


def build_cesium() -> dict:
    out = PROJECT_ROOT / "build" / "cesium" / f"{NAME}_{RECIPE.get('tiles', 'google')}.usda"
    cmd = [str(ISAAC / "python.sh"), "cesium/build_cesium_stage.py", "--source", RECIPE.get("tiles", "google"),
           "--lat", str(RECIPE["lat"]), "--lon", str(RECIPE["lon"]), "--height", str(RECIPE.get("height", 0.0)),
           "--out", str(out)]
    subprocess.run(cmd, cwd=PROJECT_ROOT, check=True)
    return {"cesium_stage": str(out.relative_to(PROJECT_ROOT)),
            "flight_fields": {"site": NAME, "source": RECIPE.get("tiles", "google")}}


def build_photogrammetry() -> dict:
    env = dict(os.environ, SITE_CONFIG=str(PROJECT_ROOT / RECIPE["site_config"]))
    subprocess.run(["bash", "run_all.sh"], cwd=PROJECT_ROOT, env=env, check=True)
    RECIPE.update({"usd": "build/isaac/site_env.usd", "add_colliders": False})   # the pipeline adds its own
    return build_import()


def register(env_block: dict) -> None:
    """Insert (or replace) `NAME:` under `environments:` in config/flight.yaml, keeping comments."""
    cfg = PROJECT_ROOT / "config" / "flight.yaml"
    lines = cfg.read_text().splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith("environments:"))
    end = next((i for i in range(start + 1, len(lines)) if lines[i] and not lines[i].startswith(" ")), len(lines))
    # Drop an existing block for this name.
    blk = next((i for i in range(start + 1, end) if lines[i].startswith(f"  {NAME}:")), None)
    if blk is not None:
        stop = next((i for i in range(blk + 1, end) if lines[i].startswith("  ") and not lines[i].startswith("    ")
                     and lines[i].strip() and not lines[i].strip().startswith("#")), end)
        del lines[blk:stop]
        end -= stop - blk
    text = yaml.safe_dump({NAME: env_block}, sort_keys=False, default_flow_style=None).rstrip().splitlines()
    block = [f"  {t}" for t in text]
    block[0] += f"   # envbuild: {Path(args.recipe).as_posix()}"
    if end < len(lines) and lines[end].strip():
        block.append("")                                     # keep a blank line before the next section
    lines[end:end] = block
    cfg.write_text("\n".join(lines) + "\n")
    print(f"registered environments.{NAME} in config/flight.yaml")


def finish(src: str, info: dict) -> None:
    manifest = {"name": NAME, "source": src, "recipe": Path(args.recipe).as_posix(), **info}
    (OUT / "env.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if src == "cesium":
        print(f"ENVBUILD cesium stage -> {info['cesium_stage']}; flights use {info['flight_fields']}", flush=True)
        return
    env_block = {**ENV_DEFAULTS, **RECIPE.get("environment", {}),
                 **{k: info[k] for k in ("usd", "ground_z", "kit_args", "occupancy") if k in info}}
    (OUT / "environment.yaml").write_text(yaml.safe_dump({NAME: env_block}, sort_keys=False))
    if args.register:
        register(env_block)
    if not args.no_probe:
        box = info.get("bounds_xy")
        # Registered: probe through the config block so its kit_args (fog, UJITSO ...) apply.
        src_args = ["--env", NAME] if args.register else ["--usd", str(PROJECT_ROOT / env_block["usd"]), "--name", NAME]
        cmd = [str(ISAAC / "python.sh"), "flight/env_probe.py", *src_args] + (["--topdown", *map(str, box)] if box else [])
        subprocess.run(cmd, cwd=PROJECT_ROOT, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("ENVBUILD " + json.dumps(manifest), flush=True)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    src = RECIPE["source"]
    builders = {"procedural": build_procedural, "import": build_import, "nurec": build_nurec,
                "cesium": build_cesium, "photogrammetry": build_photogrammetry, "osm": build_osm}
    if src not in ("procedural", "import", "photogrammetry", "osm"):
        finish(src, builders[src]())
        return
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "extra_args": ["--/crashreporter/url=",
                                                          "--/crashreporter/skipOldDumpUpload=true"]})
    try:
        # Everything happens before app.close(): closing the app ends the process.
        finish(src, builders[src]())
    except BaseException:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
