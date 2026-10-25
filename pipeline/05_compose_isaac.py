"""Stage 05: compose the Isaac Sim environment USD around the exported site.

Layout:
  /World/Site/{Visual,Collision,Fixtures}   referenced from build/usd/site.usd
  /World/PhysicsScene                       gravity -Z
  /World/PhysicsMaterials/ground            friction/restitution, bound to colliders
  /World/Lighting/{Sun,Dome}
  /World/Robots                             empty; filled by scenarios

Collision meshes get static triangle-mesh colliders (no rigid body) and are hidden from rendering.
Fixtures keep both rendering and collision.

Run: $ISAAC_ROOT/python.sh pipeline/05_compose_isaac.py
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, load_config  # noqa: E402

OUT = BUILD / "isaac" / "site_env.usd"


def sun_direction(elevation_deg: float, azimuth_deg: float) -> Gf.Vec3d:
    """Unit vector the sunlight travels along, in ENU (azimuth clockwise from north)."""
    el, az = math.radians(elevation_deg), math.radians(azimuth_deg)
    to_sun = Gf.Vec3d(math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))
    return -to_sun


def apply_static_collider(prim: Usd.Prim, material: UsdShade.Material) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.none)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")


def meshes_under(stage: Usd.Stage, path: str) -> list[Usd.Prim]:
    root = stage.GetPrimAtPath(path)
    if not root.IsValid():
        raise SystemExit(f"missing prim {path}; check the Blender export hierarchy")
    return [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Mesh)]


def compose(cfg: dict) -> dict:
    icfg = cfg["isaac"]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(OUT))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    site = UsdGeom.Xform.Define(stage, "/World/Site").GetPrim()
    site.GetReferences().AddReference("../usd/site.usd", Sdf.Path("/Site"))

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)

    pm = icfg["physics_material"]
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/ground")
    mat_api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mat_api.CreateStaticFrictionAttr(pm["static_friction"])
    mat_api.CreateDynamicFrictionAttr(pm["dynamic_friction"])
    mat_api.CreateRestitutionAttr(pm["restitution"])

    collision = meshes_under(stage, "/World/Site/Collision")
    fixtures = meshes_under(stage, "/World/Site/Fixtures")
    for prim in collision:
        apply_static_collider(prim, mat)
        UsdGeom.Imageable(prim).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    for prim in fixtures:
        apply_static_collider(prim, mat)

    UsdGeom.Xform.Define(stage, "/World/Lighting")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Sun")
    sun.CreateIntensityAttr(icfg["sun"]["intensity"])
    sun.CreateAngleAttr(0.53)
    rot = Gf.Rotation(Gf.Vec3d(0, 0, -1), sun_direction(icfg["sun"]["elevation_deg"], icfg["sun"]["azimuth_deg"]))
    UsdGeom.Xformable(sun).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(rot.GetQuat()))
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome")
    dome.CreateIntensityAttr(icfg["dome_intensity"])

    UsdGeom.Xform.Define(stage, "/World/Robots")
    stage.GetRootLayer().Save()
    return {"collision_meshes": [str(p.GetPath()) for p in collision],
            "fixture_meshes": [str(p.GetPath()) for p in fixtures]}


def verify() -> dict:
    """Re-open the saved stage and record what Isaac actually sees."""
    stage = Usd.Stage.Open(str(OUT))
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/Site/Visual")).ComputeAlignedRange()
    colliders = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    unresolved = []
    for p in stage.Traverse():
        if p.IsA(UsdShade.Shader):
            for inp in UsdShade.Shader(p).GetInputs():
                v = inp.Get()
                if isinstance(v, Sdf.AssetPath) and v.path and not v.resolvedPath:
                    unresolved.append(f"{p.GetPath()}.{inp.GetBaseName()} -> {v.path}")
    return {
        "visual_bbox": {"min": list(rng.GetMin()), "max": list(rng.GetMax())},
        "colliders": colliders,
        "unresolved_textures": unresolved,
        "mesh_count": sum(1 for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)),
    }


def main() -> None:
    cfg = load_config()
    info = compose(cfg)
    info.update(verify())
    (BUILD / "reference" / "isaac_scene.json").write_text(json.dumps(info, indent=2) + "\n")
    print(f"saved {OUT}")
    print(f"  colliders: {len(info['colliders'])}  meshes: {info['mesh_count']}  "
          f"unresolved textures: {len(info['unresolved_textures'])}")
    print(f"  visual bbox: {info['visual_bbox']}")
    if info["unresolved_textures"]:
        print("  first unresolved:", info["unresolved_textures"][:3])


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
