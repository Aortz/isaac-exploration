"""Cesium for Omniverse helpers shared by cesium/build_cesium_stage.py and flight/f2_render.py.

Import this before creating the SimulationApp only for the constants and `simulation_app_config`;
the runtime helpers import omni/Cesium modules lazily, so call them after the app exists.

(Kept under pipeline.common rather than a top-level `cesium` package so it cannot shadow the
extension's own `cesium.omniverse` / `cesium.usd.plugins` packages.)
"""

from __future__ import annotations

import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CESIUM_EXTS = PROJECT_ROOT / "third_party" / "cesium" / "v0.29.0"

# Named origins. Heights are WGS84 ellipsoidal metres; Singapore's geoid sits ~7-8 m above the
# ellipsoid, so ground near sea level appears at roughly z = +8 m.
SITES = {
    "marina_bay": (1.2834, 103.8607, 0.0),
    "changi": (1.3644, 103.9915, 0.0),
    "jurong_west": (1.3404, 103.7090, 0.0),
    # Public 3D Tiles sample (no token needed); used to prove the plugin streams on this machine.
    # Note its root transform scales the model 100x: it is a ~1.4 km dragon, view it from ~3 km.
    "cesium_sample": (40.0425306, -75.6120943, 500.0),
}

# Cesium ion asset ids (Asset Depot).
ION_ASSETS = {
    "google": [("Google3DTiles", 2275207)],
    "osm": [("CesiumWorldTerrain", 1), ("OSMBuildings", 96188)],
}
SAMPLE_URL = "https://raw.githubusercontent.com/CesiumGS/3d-tiles-samples/main/1.0/TilesetWithDiscreteLOD/tileset.json"
ION_SERVER = "/CesiumServers/IonOfficial"


def simulation_app_config(headless: bool = True) -> dict:
    """SimulationApp config that loads Cesium at startup.

    The extension must load at startup: its USD schemas are only picked up when the schema
    registry is first built, so enabling it later leaves the Cesium prim types without attributes.
    """
    return {"headless": headless,
            "extra_args": ["--ext-folder", str(CESIUM_EXTS), "--enable", "cesium.omniverse",
                           # Crash dumps hold process memory (the ion token): keep them local.
                           "--/crashreporter/url=", "--/crashreporter/skipOldDumpUpload=true",
                           "--/crashreporter/includeEnvironmentAsMetadata=false"]}


def ensure_loaded() -> None:
    import omni.kit.app

    if not omni.kit.app.get_app().get_extension_manager().is_extension_enabled("cesium.omniverse"):
        raise RuntimeError("cesium.omniverse did not load; is third_party/cesium installed?")


def open_stage(app, path: str | Path, token: str = ""):
    """Open a Cesium stage. The ion token goes to the session layer only, never to the file.

    The token must be in place before Kit sees the stage: tilesets start loading on open, and a
    tokenless ion request (401) sends cesium.omniverse 0.29 into its troubleshooting path, which
    segfaults headless. So compose the stage in memory with a pre-filled session layer and attach it.
    """
    import omni.usd
    from cesium.usd.plugins.CesiumUsdSchemas import IonServer
    from pxr import Sdf, Usd, UsdUtils

    ctx = omni.usd.get_context()
    session = Sdf.Layer.CreateAnonymous("cesium_session")
    stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(path)), session)
    stage.SetEditTarget(session)
    if token:
        server = stage.GetPrimAtPath(ION_SERVER)
        if server.IsValid():
            IonServer(server).GetProjectDefaultIonAccessTokenAttr().Set(token)
    stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    ctx.attach_stage_with_callback(stage_id=stage_id, on_finish_fn=lambda success, message: None)
    while ctx.get_stage() is None or ctx.get_stage_loading_status()[2] > 0:
        app.update()
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    return stage


def apply_look(stage, rcfg: dict) -> dict:
    """Session-layer render look: Cesium tile detail, HDR sky dome, sun. Returns what was applied.

    Google tiles have daylight baked in, so the sky dome is the only light (soft, cloudy) and the
    sun stays off unless rcfg asks for it.
    """
    from cesium.usd.plugins.CesiumUsdSchemas import Tileset
    from pxr import Sdf, UsdLux

    applied = {}
    if "max_screen_space_error" in rcfg:
        for prim in stage.Traverse():
            if prim.IsA(Tileset):
                Tileset(prim).GetMaximumScreenSpaceErrorAttr().Set(float(rcfg["max_screen_space_error"]))
        applied["max_screen_space_error"] = rcfg["max_screen_space_error"]
    dome = UsdLux.DomeLight(stage.GetPrimAtPath("/World/Lighting/Dome"))
    if dome and rcfg.get("sky_hdr"):
        from isaacsim.storage.native import get_assets_root_path

        url = get_assets_root_path(skip_check=True) + rcfg["sky_hdr"]
        dome.CreateTextureFileAttr().Set(Sdf.AssetPath(url))
        dome.CreateTextureFormatAttr().Set(UsdLux.Tokens.latlong)
        applied["sky_hdr"] = url
    if dome and "sky_intensity" in rcfg:
        dome.GetIntensityAttr().Set(float(rcfg["sky_intensity"]))
        applied["sky_intensity"] = rcfg["sky_intensity"]
    sun = UsdLux.DistantLight(stage.GetPrimAtPath("/World/Lighting/Sun"))
    if sun and "sun_intensity" in rcfg:
        sun.GetIntensityAttr().Set(float(rcfg["sun_intensity"]))
        applied["sun_intensity"] = rcfg["sun_intensity"]
    return applied


def set_viewport_camera(camera_path: str) -> bool:
    """Cesium picks tiles from the viewport cameras, so point the active viewport at our camera."""
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is None:
            return False
        vp.camera_path = camera_path
        return True
    except Exception:
        return False


def tile_stats() -> dict:
    from cesium.omniverse.utils.cesium_interface import CesiumInterfaceManager

    with CesiumInterfaceManager() as cesium:
        s = cesium.get_render_statistics()
        return {k: getattr(s, k) for k in ("tiles_visited", "tiles_loaded", "tiles_rendered", "tiles_culled",
                                            "tiles_loading_worker", "tiles_loading_main", "max_depth_visited",
                                            "tileset_cached_bytes")}


def wait_for_tiles(app, timeout_s: float, min_updates: int = 3,
                   require_rendered: bool = True) -> tuple[dict, bool, int]:
    """Step the app until Cesium has no tiles loading (or timeout). Returns (stats, settled, updates).

    Idle has to hold for min_updates consecutive updates: the statistics trail the camera by an
    update or so, so after a big camera jump the first "idle" still describes the old view (and
    right after the stage opens nothing is loading *yet*). require_rendered also waits for at
    least one tile on screen.
    """
    t0 = time.time()
    n = streak = 0
    while True:
        app.update()
        n += 1
        s = tile_stats()
        idle = s["tiles_loading_worker"] == 0 and s["tiles_loading_main"] == 0
        if require_rendered and s["tiles_rendered"] == 0:
            idle = False
        streak = streak + 1 if idle else 0
        if streak >= min_updates:
            return s, True, n
        if time.time() - t0 > timeout_s:
            return s, False, n
