"""Street-level city blocks from OpenStreetMap: extruded, textured buildings, roads, street trees.

Isaac Python (pxr), except fetch() / project() / earclip(), which are plain Python + numpy. Writes into
an open Usd.Stage:
  /World/Ground          flat ground slab (collider) with a base material
  /World/Roads           road ribbons (asphalt) from OSM highway centre lines and their usual widths
  /World/Buildings/<s>   one mesh per facade style: walls with a generated window-grid texture (one
                         floor per `floor_m`, one window bay per `bay_m`) + ear-clipped roofs; colliders
  /World/Scatter/trees   street / park trees (PointInstancer)
and returns building boxes (N x 6, centre + size) for the clearance sidecar.

Heights come from OSM `height` / `building:levels`; untagged buildings get a seeded height from the
recipe's per-type defaults. The local frame is ENU metres around (lat, lon), the same origin as a Cesium
recipe at that point, so tile and OSM versions of a place line up.
"""

from __future__ import annotations

import json
import re
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

OVERPASS = "https://overpass-api.de/api/interpreter"
ROAD_WIDTH = {"motorway": 16.0, "trunk": 14.0, "primary": 12.0, "secondary": 10.0, "tertiary": 8.0,
              "unclassified": 6.0, "residential": 6.0, "living_street": 5.0, "service": 4.0,
              "motorway_link": 7.0, "trunk_link": 7.0, "primary_link": 7.0, "secondary_link": 6.0}


# --- data ---------------------------------------------------------------------------------------
def fetch(lat: float, lon: float, radius_m: float, cache: Path) -> dict:
    """Buildings, roads, parks and trees within radius_m (bbox) of (lat, lon); cached as JSON."""
    if cache.exists():
        return json.loads(cache.read_text())
    dlat = radius_m / 110540.0
    dlon = radius_m / (111320.0 * np.cos(np.radians(lat)))
    bb = f"{lat - dlat},{lon - dlon},{lat + dlat},{lon + dlon}"
    q = (f'[out:json][timeout:120];(way["building"]({bb});relation["building"]({bb});'
         f'way["highway"]({bb});way["leisure"="park"]({bb});way["landuse"="grass"]({bb});'
         f'node["natural"="tree"]({bb}););out geom;')
    req = urllib.request.Request(OVERPASS, data=urllib.parse.urlencode({"data": q}).encode(),
                                 headers={"User-Agent": "isaac-exploration-envbuild/1.0"})
    with urllib.request.urlopen(req, timeout=180) as r:
        data = json.loads(r.read())
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps(data))
    return data


def project(lat0: float, lon0: float, lats, lons) -> np.ndarray:
    """Local ENU metres (equirectangular; < 0.1 % error within a few km)."""
    x = (np.asarray(lons) - lon0) * 111320.0 * np.cos(np.radians(lat0))
    y = (np.asarray(lats) - lat0) * 110540.0
    return np.column_stack([x, y])


def parse_height(tags: dict, rng, defaults: dict, floor_m: float) -> float:
    for key in ("height", "building:height"):
        if key in tags:
            m = re.match(r"\s*([0-9.]+)", tags[key])
            if m:
                return float(m.group(1))
    if "building:levels" in tags:
        m = re.match(r"\s*([0-9.]+)", tags["building:levels"])
        if m:
            return float(m.group(1)) * floor_m + 1.0
    lo, hi = defaults.get(tags.get("building", "yes"), defaults.get("yes", [6.0, 14.0]))
    return float(rng.uniform(lo, hi))


def footprints(data: dict, lat0: float, lon0: float) -> list[tuple[np.ndarray, dict]]:
    """(polygon N x 2, CCW, no repeated end point; tags) for every building way / multipolygon outer."""
    out = []
    for el in data["elements"]:
        tags = el.get("tags", {})
        if "building" not in tags:
            continue
        rings = []
        if el["type"] == "way" and el.get("geometry"):
            rings = [el["geometry"]]
        elif el["type"] == "relation":
            rings = [m["geometry"] for m in el.get("members", []) if m.get("role") == "outer" and m.get("geometry")]
        for g in rings:
            p = project(lat0, lon0, [q["lat"] for q in g], [q["lon"] for q in g])
            if len(p) > 1 and np.allclose(p[0], p[-1]):
                p = p[:-1]
            if len(p) < 3:
                continue
            if signed_area(p) < 0:
                p = p[::-1]
            if abs(signed_area(p)) > 4.0:                            # drop slivers / mapping noise
                out.append((p, tags))
    return out


def signed_area(p: np.ndarray) -> float:
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def earclip(p: np.ndarray) -> list[tuple[int, int, int]]:
    """Triangulate a simple CCW polygon (ear clipping). Hydra fan-triangulates n-gons, which breaks concave roofs."""
    idx = list(range(len(p)))
    tris = []

    def inside(a, b, c, q):
        d1 = (q[0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (q[1] - b[1])
        d2 = (q[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (q[1] - c[1])
        d3 = (q[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (q[1] - a[1])
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))

    guard = 0
    while len(idx) > 3 and guard < 10 * len(p):
        guard += 1
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[(k - 1) % n], idx[k], idx[(k + 1) % n]
            a, b, c = p[i0], p[i1], p[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-9:
                continue                                              # reflex or degenerate corner
            if any(inside(a, b, c, p[j]) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break                                                     # self-intersecting: fall back to a fan
    if len(idx) == 3:
        tris.append(tuple(idx))
    elif len(idx) > 3:
        tris += [(idx[0], idx[k], idx[k + 1]) for k in range(1, len(idx) - 1)]
    return tris


def point_in_poly(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    x, y = pts[:, 0][:, None], pts[:, 1][:, None]
    xa, ya = poly[:, 0][None, :], poly[:, 1][None, :]
    xb, yb = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
    cross = ((ya > y) != (yb > y)) & (x < (xb - xa) * (y - ya) / (yb - ya + 1e-12) + xa)
    return cross.sum(axis=1) % 2 == 1


# --- facade textures ------------------------------------------------------------------------------
def facade_texture(path: Path, style: dict, rng, px: int = 128) -> None:
    """One window bay x one floor tile: wall colour, a window (dark glass with a lighter frame), noise."""
    from PIL import Image

    wall = np.array(style["wall_rgb"], float)
    glass = np.array(style.get("glass_rgb", [0.12, 0.15, 0.18]), float)
    img = np.ones((px, px, 3)) * wall
    wx, wy = style.get("window_frac", [0.55, 0.5])
    x0, x1 = int(px * (1 - wx) / 2), int(px * (1 + wx) / 2)
    y0, y1 = int(px * 0.25), int(px * (0.25 + wy))
    img[y0 - 3:y1 + 3, x0 - 3:x1 + 3] = wall * 0.75 + 0.25                # frame / sill
    img[y0:y1, x0:x1] = glass
    img[y0:y1, (x0 + x1) // 2 - 1:(x0 + x1) // 2 + 1] = wall * 0.6        # mullion
    img[int(px * 0.92):, :] *= 0.85                                       # floor slab line
    img *= 1.0 + rng.normal(0, style.get("noise", 0.04), (px, px, 1))     # grime
    Image.fromarray((np.clip(img, 0, 1)[::-1] * 255).astype(np.uint8)).save(path)


def preview_material(stage, look: str, tex: Path | None, rgb=(0.5, 0.5, 0.5), roughness=0.85):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, look)
    sh = UsdShade.Shader.Define(stage, look + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if tex is None:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    else:
        rd = UsdShade.Shader.Define(stage, look + "/st")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, look + "/tex")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(tex))
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    sh.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


# --- geometry -------------------------------------------------------------------------------------
def _mesh(stage, path: str, pts, counts, idx, st=None, collider: bool = True):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt

    pts = np.asarray(pts, np.float32)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.asarray(counts, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(idx, np.int32)))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
    if st is not None:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray.FromNumpy(np.asarray(st, np.float32)))
    if collider:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.none)
    return m


def write_buildings(stage, blds: list[tuple[np.ndarray, float, int]], styles: list[dict], tex_dir: Path,
                    floor_m: float, bay_m: float, roof_mdl: str | None, rng) -> np.ndarray:
    """blds: (footprint, height, style index). One wall mesh + one roof mesh per style. Returns AABBs."""
    from pxr import Sdf, UsdShade

    from envbuild.procedural import bind_mdl

    tex_dir.mkdir(parents=True, exist_ok=True)
    boxes = []
    for s, style in enumerate(styles):
        mine = [b for b in blds if b[2] == s]
        if not mine:
            continue
        wp, wi, wst, rp, ri = [], [], [], [], []
        for poly, hgt, _ in mine:
            n = len(poly)
            base = len(wp)
            per = np.r_[0.0, np.cumsum(np.linalg.norm(np.roll(poly, -1, 0) - poly, axis=1))]
            for k in range(n):
                a, b = poly[k], poly[(k + 1) % n]
                wp += [[a[0], a[1], 0.0], [b[0], b[1], 0.0], [b[0], b[1], hgt], [a[0], a[1], hgt]]
                u0, u1 = per[k] / bay_m, per[k + 1] / bay_m
                v1 = hgt / floor_m
                wst += [[u0, 0.0], [u1, 0.0], [u1, v1], [u0, v1]]
                wi += [base + 4 * k, base + 4 * k + 1, base + 4 * k + 2, base + 4 * k + 3]
            rb = len(rp)
            rp += [[x, y, hgt] for x, y in poly]
            ri += [rb + t for tri in earclip(poly) for t in tri]
            lo, hi = poly.min(0), poly.max(0)
            boxes.append([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hgt / 2, hi[0] - lo[0], hi[1] - lo[1], hgt])
        tex = tex_dir / f"facade_{s}.png"
        facade_texture(tex, style, rng)
        walls = _mesh(stage, f"/World/Buildings/style{s}/Walls", wp, [4] * (len(wi) // 4), wi, wst)
        UsdShade.MaterialBindingAPI.Apply(walls.GetPrim()).Bind(preview_material(stage, f"/World/Looks/Facade{s}", tex))
        walls.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("building")
        roof = _mesh(stage, f"/World/Buildings/style{s}/Roofs", rp, [3] * (len(ri) // 3), ri,
                     np.asarray(rp)[ri][:, :2] / 4.0)
        if roof_mdl:
            bind_mdl(stage, roof.GetPrim(), roof_mdl, f"/World/Looks/Roof{s}")
        roof.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("roof")
    return np.array(boxes)


def write_roads(stage, ways: list[tuple[np.ndarray, float]], material: str, z: float = 0.03) -> int:
    """Road ribbons: each centre-line segment as a quad `width` wide, plus a square cap at every joint."""
    from pxr import Sdf

    from envbuild.procedural import bind_mdl

    pts, idx = [], []
    for line, w in ways:
        for a, b in zip(line[:-1], line[1:]):
            d = b - a
            L = np.linalg.norm(d)
            if L < 0.1:
                continue
            nrm = np.array([-d[1], d[0]]) / L * w / 2
            ext = d / L * w / 2                                       # overlap the joints
            q = [a - ext + nrm, b + ext + nrm, b + ext - nrm, a - ext - nrm]
            base = len(pts)
            pts += [[p[0], p[1], z] for p in q]
            idx += [base, base + 3, base + 2, base + 1]               # face up
    if not pts:
        return 0
    m = _mesh(stage, "/World/Roads", pts, [4] * (len(idx) // 4), idx,
              np.asarray(pts)[idx][:, :2] / 6.0, collider=False)
    bind_mdl(stage, m.GetPrim(), material, "/World/Looks/Road")
    m.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("road")
    return len(idx) // 4


def write_ground(stage, half: float, material: str) -> None:
    pts = [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]]
    m = _mesh(stage, "/World/Ground", pts, [4], [0, 1, 2, 3], np.asarray(pts)[:, :2] / 4.0)
    from envbuild.procedural import bind_mdl
    from pxr import Sdf

    bind_mdl(stage, m.GetPrim(), material, "/World/Looks/Ground")
    m.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("terrain")
