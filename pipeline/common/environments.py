"""Isaac USD environments (config/flight.yaml `environments:`), the non-Cesium counterpart of cesium_support.

Isaac Python only (needs a running SimulationApp). Edits go to the session layer, never the asset.
"""

from __future__ import annotations

from pathlib import Path


def asset_url(path: str) -> str:
    """Paths starting with /Isaac or /NVIDIA live under the Isaac assets root; anything else is used as is."""
    if path.startswith(("/Isaac/", "/NVIDIA/")):
        from isaacsim.storage.native import get_assets_root_path

        return get_assets_root_path(skip_check=True) + path
    if "://" not in path and not Path(path).is_absolute():
        return str(Path(__file__).resolve().parents[2] / path)     # relative to the repo root
    return path


def open_env(app, env_cfg: dict):
    import omni.usd

    ctx = omni.usd.get_context()
    url = asset_url(env_cfg["usd"])
    if url.endswith(".usdz"):
        # A .usdz opened as the root stage cannot take new prims (Isaac 6 NuRec known issue: they come out
        # empty), so start a fresh stage and reference the package under /World/Env.
        from pxr import UsdGeom

        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        stage.DefinePrim("/World/Env").GetReferences().AddReference(url)
        while ctx.get_stage_loading_status()[2] > 0:
            app.update()
        return stage
    if not ctx.open_stage(url):
        raise RuntimeError(f"could not open {url}")
    while ctx.get_stage_loading_status()[2] > 0:
        app.update()
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    return stage


def apply_lighting(stage, env_cfg: dict, variant: str) -> dict:
    """Swap the sky dome HDR / intensity for a lighting variant. Returns what the dome ends up with."""
    from pxr import Sdf, UsdLux

    spec = env_cfg.get("lighting", {}).get(variant)
    if spec is None:
        raise SystemExit(f"unknown lighting variant {variant!r}; have {list(env_cfg.get('lighting', {}))}")
    dome = UsdLux.DomeLight(stage.GetPrimAtPath(env_cfg.get("sky_dome", "")))
    if not dome:
        return {"variant": variant, "dome": None}
    if spec.get("sky_hdr"):
        dome.CreateTextureFileAttr().Set(Sdf.AssetPath(asset_url(spec["sky_hdr"])))
        dome.CreateTextureFormatAttr().Set(UsdLux.Tokens.latlong)
    if "sky_intensity" in spec:
        dome.GetIntensityAttr().Set(float(spec["sky_intensity"]))
    tex = dome.GetTextureFileAttr().Get()
    return {"variant": variant, "dome": str(dome.GetPath()), "texture": tex.path if tex else None,
            "intensity": dome.GetIntensityAttr().Get()}


def settle(app, updates: int) -> tuple[dict, bool, int]:
    """Lockstep wait for a static environment: a fixed number of renderer updates (no streaming to poll)."""
    for _ in range(updates):
        app.update()
    return {}, True, updates


def ensure_physics_scene(stage, hz: int) -> str:
    from pxr import Gf, PhysxSchema, UsdPhysics

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene = UsdPhysics.Scene(prim)
            break
    else:
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(hz)
    return str(scene.GetPath())
