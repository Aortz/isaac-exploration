"""Build a Cesium for Omniverse stage georeferenced to a chosen location, and optionally verify it.

The stage contains:
  /Cesium                      CesiumData (selects the ion server)
  /CesiumServers/IonOfficial   ion.cesium.com server
  /CesiumGeoreference          origin (lat, lon, ellipsoidal height) -> USD (0, 0, 0), Z up, metres
  /World/Tiles/<name>          3D Tiles sources (Google photoreal, Cesium terrain + OSM buildings, or a URL)
  /World/Lighting              sun + dome

The ion token is NOT written to the saved USD. Pass it with CESIUM_ION_TOKEN (or --token) for
headless verification; in the GUI sign in through Window > Cesium instead.

Run:
  $ISAAC_ROOT/python.sh cesium/build_cesium_stage.py --site marina_bay --source google --verify
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from pipeline.common.cesium_support import (  # noqa: E402
    ION_ASSETS,
    ION_SERVER,
    SAMPLE_URL,
    SITES,
    open_stage,
    simulation_app_config,
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="marina_bay", choices=sorted(SITES))
    ap.add_argument("--lat", type=float, help="override origin latitude")
    ap.add_argument("--lon", type=float, help="override origin longitude")
    ap.add_argument("--height", type=float, help="override origin ellipsoidal height (m)")
    ap.add_argument("--source", default="google", choices=["google", "osm", "url", "sample"])
    ap.add_argument("--url", help="tileset.json URL for --source url (e.g. Google Map Tiles API root.json?key=...)")
    ap.add_argument("--token", default=os.environ.get("CESIUM_ION_TOKEN", ""), help="ion access token (not saved)")
    ap.add_argument("--out", default=None, help="output USD (default build/cesium/<site>_<source>.usda)")
    ap.add_argument("--verify", action="store_true", help="stream tiles headless and report statistics")
    ap.add_argument("--marker", action="store_true", help="add a 5 m red cube at the origin (camera aim check)")
    ap.add_argument("--verify-seconds", type=float, default=90.0)
    ap.add_argument("--camera-distance", type=float, default=800.0, help="verify camera distance to the origin (m)")
    ap.add_argument("--camera-pitch", type=float, default=45.0, help="verify camera look-down angle (deg)")
    return ap.parse_args()


args = parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp(simulation_app_config(headless=True))

import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402

if not omni.kit.app.get_app().get_extension_manager().is_extension_enabled("cesium.omniverse"):
    print("CESIUM_FAIL cesium.omniverse did not load")
    app.close()
    sys.exit(1)

from cesium.usd.plugins.CesiumUsdSchemas import (  # noqa: E402
    Data as CesiumData,
    Georeference as CesiumGeoreference,
    IonServer as CesiumIonServer,
    Tileset as CesiumTileset,
    Tokens as CesiumTokens,
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402


def build_stage(stage: Usd.Stage, lat: float, lon: float, height: float) -> list[str]:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    server = CesiumIonServer.Define(stage, ION_SERVER)
    server.GetDisplayNameAttr().Set("ion.cesium.com")
    server.GetIonServerUrlAttr().Set("https://ion.cesium.com/")
    server.GetIonServerApiUrlAttr().Set("https://api.cesium.com/")
    server.GetIonServerApplicationIdAttr().Set(413)
    data = CesiumData.Define(stage, "/Cesium")
    data.GetSelectedIonServerRel().SetTargets([ION_SERVER])

    geo = CesiumGeoreference.Define(stage, "/CesiumGeoreference")
    geo.GetGeoreferenceOriginLatitudeAttr().Set(lat)
    geo.GetGeoreferenceOriginLongitudeAttr().Set(lon)
    geo.GetGeoreferenceOriginHeightAttr().Set(height)

    UsdGeom.Scope.Define(stage, "/World/Tiles")
    paths = []
    if args.source in ION_ASSETS:
        for name, asset_id in ION_ASSETS[args.source]:
            t = CesiumTileset.Define(stage, f"/World/Tiles/{name}")
            t.GetSourceTypeAttr().Set(CesiumTokens.ion)
            t.GetIonAssetIdAttr().Set(asset_id)
            t.GetIonServerBindingRel().SetTargets([ION_SERVER])
            t.GetGeoreferenceBindingRel().SetTargets(["/CesiumGeoreference"])
            paths.append(str(t.GetPath()))
    else:
        url = SAMPLE_URL if args.source == "sample" else args.url
        if not url:
            raise SystemExit("--source url needs --url")
        t = CesiumTileset.Define(stage, "/World/Tiles/UrlTileset")
        t.GetSourceTypeAttr().Set(CesiumTokens.url)
        t.GetUrlAttr().Set(url)
        t.GetGeoreferenceBindingRel().SetTargets(["/CesiumGeoreference"])
        paths.append(str(t.GetPath()))

    UsdGeom.Xform.Define(stage, "/World/Lighting")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Sun")
    sun.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 30.0))
    UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome").CreateIntensityAttr(500.0)
    return paths


def ecef(lat: float, lon: float, h: float) -> Gf.Vec3d:
    """WGS84 geodetic -> ECEF metres."""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    la, lo = math.radians(lat), math.radians(lon)
    n = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    return Gf.Vec3d((n + h) * math.cos(la) * math.cos(lo), (n + h) * math.cos(la) * math.sin(lo),
                    (n * (1 - e2) + h) * math.sin(la))


def check_georeference(stage: Usd.Stage, lat: float, lon: float, height: float) -> dict:
    """Map points near the origin through the plugin's ECEF->USD transform; they should land at
    (0,0,0), (0,0,100) and roughly (0,111,0) (1 km north is ~+Y)."""
    m = CesiumGeoreference(stage.GetPrimAtPath("/CesiumGeoreference")).GetEcefToUsdTransformAttr().Get()
    if m is None:
        return {"ok": False, "reason": "ecefToUsdTransform not computed by plugin"}
    to_usd = lambda p: m.Transform(p)  # noqa: E731  (Gf row-vector convention)
    origin = to_usd(ecef(lat, lon, height))
    up = to_usd(ecef(lat, lon, height + 100.0))
    north = to_usd(ecef(lat + 0.001, lon, height))
    east = to_usd(ecef(lat, lon + 0.001, height))
    res = {"origin": list(origin), "up_100m": list(up), "north_0.001deg": list(north), "east_0.001deg": list(east)}
    # 0.001 deg of latitude ~ 110.6-111.7 m; 0.001 deg of longitude ~ 111.32 * cos(lat) m.
    east_expected = 111.32 * math.cos(math.radians(lat))
    res["ok"] = (origin.GetLength() < 0.01 and abs(up[2] - 100.0) < 0.01
                 and 110.0 < north[1] < 112.0 and abs(north[0]) < 0.5
                 and abs(east[0] - east_expected) < 0.01 * east_expected and abs(east[1]) < 0.5)
    return res


def stream(stage: Usd.Stage, seconds: float, out_png: Path) -> dict:
    import numpy as np
    import omni.replicator.core as rep
    from cesium.omniverse.utils.cesium_interface import CesiumInterfaceManager

    if args.marker:
        cube = UsdGeom.Cube.Define(stage, "/World/OriginMarker")
        cube.CreateSizeAttr(5.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.0, 0.0)])
    cam = UsdGeom.Camera.Define(stage, "/World/VerifyCamera")
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(1.0, 1.0e6))
    # South of the origin, looking north and down `camera_pitch` straight at the origin.
    # A USD camera looks along -Z; rotating it (90 - pitch) about X tips that toward +Y.
    p = math.radians(args.camera_pitch)
    xf = UsdGeom.Xformable(cam)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -args.camera_distance * math.cos(p), args.camera_distance * math.sin(p)))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(90.0 - args.camera_pitch, 0.0, 0.0))
    rp = rep.create.render_product(str(cam.GetPath()), (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(rp)
    # Cesium selects tiles from the active viewport camera.
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = str(cam.GetPath())
    except Exception as exc:  # headless builds may have no viewport
        print(f"no viewport to bind ({exc}); relying on render product camera")

    # Replicator annotators only receive frames while the timeline is playing.
    import omni.timeline

    omni.timeline.get_timeline_interface().play()

    history = []
    t0 = time.time()
    with CesiumInterfaceManager() as cesium:
        while time.time() - t0 < seconds:
            app.update()
            s = cesium.get_render_statistics()
            history.append((round(time.time() - t0, 1), s.tiles_loaded, s.tiles_rendered, s.tiles_loading_worker))
            if len(history) > 200 and s.tiles_loading_worker == 0 and s.tiles_loading_main == 0 and s.tiles_rendered > 0:
                break
        s = cesium.get_render_statistics()
        stats = {k: getattr(s, k) for k in ("tiles_visited", "tiles_loaded", "tiles_rendered", "tiles_culled",
                                            "geometries_loaded", "geometries_rendered", "max_depth_visited",
                                            "tileset_cached_bytes")}
        credits = [c[0] for c in cesium.get_credits()][:5] if cesium.credits_available() else []
    def grab():
        d = rgb.get_data()
        d = d.get("data") if isinstance(d, dict) else d
        return d if d is not None and getattr(d, "size", 0) else None

    img = None
    for _ in range(300):  # the render product needs some frames before its first image
        app.update()
        if grab() is not None:
            break
    for _ in range(60):  # then let the path tracer / streaming settle before the real capture
        app.update()
    img = grab()
    if img is not None and getattr(img, "size", 0):
        from PIL import Image
        Image.fromarray(np.asarray(img)[..., :3]).save(out_png)
    stats["elapsed_s"] = round(time.time() - t0, 1)
    stats["credits"] = credits
    stats["screenshot"] = str(out_png) if out_png.exists() else None
    stats["progress"] = history[:: max(1, len(history) // 10)]
    return stats


def main() -> None:
    lat, lon, height = SITES[args.site]
    lat = args.lat if args.lat is not None else lat
    lon = args.lon if args.lon is not None else lon
    height = args.height if args.height is not None else height
    out = Path(args.out) if args.out else PROJECT_ROOT / "build" / "cesium" / f"{args.site}_{args.source}.usda"
    out.parent.mkdir(parents=True, exist_ok=True)

    # Author the file with plain USD, then open it the same way the GUI does. The file never holds
    # the ion token.
    file_stage = Usd.Stage.CreateNew(str(out))
    tilesets = build_stage(file_stage, lat, lon, height)
    file_stage.GetRootLayer().Save()
    del file_stage
    print(f"CESIUM_STAGE {out}  origin=({lat}, {lon}, {height})  tilesets={tilesets}")

    # Verification-only edits (token, camera, marker) go to the session layer so the file stays clean.
    # The token has to be there before Kit sees the stage (see open_stage).
    token = args.token if args.verify and args.source in ION_ASSETS else ""
    stage = open_stage(app, out, token)
    for _ in range(30):
        app.update()
    report = {"stage": str(out), "origin": [lat, lon, height], "source": args.source, "tilesets": tilesets,
              "georeference": check_georeference(stage, lat, lon, height)}

    if args.verify:
        if args.source in ION_ASSETS:
            if not args.token:
                report["stream"] = {"skipped": "no ion token (set CESIUM_ION_TOKEN)"}
        if "stream" not in report:
            report["stream"] = stream(stage, args.verify_seconds, out.with_suffix(".png"))

    print("CESIUM_REPORT " + json.dumps(report))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        print("CESIUM_FAIL see traceback above")
    finally:
        app.close()
