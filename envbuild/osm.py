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
BAYS, FLOORS = 4, 3                    # one facade tile covers 4 window bays x 3 floors


def facade_texture(path: Path, style: dict, rng, px: int = 128) -> None:
    """A 4-bay x 3-floor facade tile: wall colour, windows with per-window variety (curtains, blinds,
    the odd lit room, AC units under some sills), floor slab lines and grime. Varied windows give the
    trackers distinct features instead of one window repeated across the building."""
    from PIL import Image

    wall = np.array(style["wall_rgb"], float)
    glass = np.array(style.get("glass_rgb", [0.12, 0.15, 0.18]), float)
    H, W = FLOORS * px, BAYS * px
    img = np.ones((H, W, 3)) * wall
    wx, wy = style.get("window_frac", [0.55, 0.5])
    for f in range(FLOORS):
        for b in range(BAYS):
            ox, oy = b * px, f * px
            x0, x1 = ox + int(px * (1 - wx) / 2), ox + int(px * (1 + wx) / 2)
            y0, y1 = oy + int(px * 0.25), oy + int(px * (0.25 + wy))
            img[y0 - 3:y1 + 3, x0 - 3:x1 + 3] = wall * 0.75 + 0.25        # frame / sill
            r = rng.random()
            if r < 0.07:
                pane = np.array([0.85, 0.75, 0.50])                        # lit room
            elif r < 0.35:
                pane = rng.choice([[0.75, 0.72, 0.65], [0.55, 0.45, 0.35], [0.60, 0.62, 0.66]])   # curtains
            else:
                pane = glass * rng.uniform(0.8, 1.25)
            img[y0:y1, x0:x1] = pane
            if rng.random() < 0.3:                                         # half-drawn blinds
                img[y0:y0 + int((y1 - y0) * rng.uniform(0.2, 0.6)), x0:x1] = [0.80, 0.80, 0.78]
            img[y0:y1, (x0 + x1) // 2 - 1:(x0 + x1) // 2 + 1] = wall * 0.6     # mullion
            if rng.random() < style.get("ac_frac", 0.25):                  # AC unit under the sill
                img[y0 - 22:y0 - 6, x0 + 4:x0 + 28] = [0.78, 0.78, 0.76]
        img[oy + int(px * 0.92):oy + px, :] *= 0.85                       # floor slab line
    img *= 1.0 + rng.normal(0, style.get("noise", 0.04), (H, W, 1))       # grime
    img *= np.linspace(0.92, 1.0, H)[:, None, None]                       # weathering towards the bottom
    Image.fromarray((np.clip(img, 0, 1)[::-1] * 255).astype(np.uint8)).save(path)


def shopfront_texture(path: Path, rng, px: int = 128) -> None:
    """Ground-floor shopfronts, 4 units wide: tall glazing, a coloured signage band, a kerb-level plinth."""
    from PIL import Image

    img = np.ones((px, 4 * px, 3)) * 0.55
    for b in range(4):
        ox = b * px
        sign = rng.choice([[0.75, 0.15, 0.12], [0.10, 0.30, 0.60], [0.90, 0.75, 0.15], [0.15, 0.50, 0.25],
                           [0.85, 0.85, 0.85], [0.20, 0.20, 0.22]])
        img[int(px * 0.72):int(px * 0.88), ox + 4:ox + px - 4] = sign
        img[int(px * 0.10):int(px * 0.68), ox + 8:ox + px - 8] = np.array([0.10, 0.12, 0.13]) * rng.uniform(0.8, 1.6)
        if rng.random() < 0.4:                                             # roller shutter down
            img[int(px * 0.10):int(px * 0.68), ox + 8:ox + px - 8] = 0.62
            img[int(px * 0.10):int(px * 0.68):6, ox + 8:ox + px - 8] *= 0.8
        img[:int(px * 0.08), ox:ox + px] = 0.45                           # plinth
    img *= 1.0 + rng.normal(0, 0.04, (px, 4 * px, 1))
    Image.fromarray((np.clip(img, 0, 1)[::-1] * 255).astype(np.uint8)).save(path)


def ground_texture(path: Path, rng, kind: str, px: int = 256) -> None:
    """Road (lane markings across u, dashes along v over 6 m) or sidewalk (paving joints) tile."""
    from PIL import Image

    if kind == "road":
        img = np.ones((2 * px, px, 3)) * 0.18 * (1 + rng.normal(0, 0.08, (2 * px, px, 1)))
        for u in (0.05, 0.95):                                             # edge lines
            img[:, int(u * px) - 2:int(u * px) + 2] = 0.85
        img[:px, px // 2 - 2:px // 2 + 2] = 0.85                           # dashed centre line (3 m of 6 m)
    else:
        img = np.ones((px, px, 3)) * 0.62 * (1 + rng.normal(0, 0.05, (px, px, 1)))
        img[::px // 4, :] *= 0.75                                          # paving joints every 0.6 m
        img[:, ::px // 4] *= 0.75
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


def write_buildings(stage, blds: list[tuple[np.ndarray, float, int, bool]], styles: list[dict], tex_dir: Path,
                    floor_m: float, bay_m: float, roof_mdl: str | None, rng, tints: int = 3) -> np.ndarray:
    """blds: (footprint, height, style index, has shopfront). Walls per style and tint, one shopfront mesh,
    roofs, rooftop clutter (AC units, water tanks, lift cores). Returns AABBs (with the clutter)."""
    from pxr import Sdf, UsdShade

    from envbuild.procedural import bind_mdl

    tex_dir.mkdir(parents=True, exist_ok=True)
    boxes = []
    shop_p, shop_i, shop_st = [], [], []
    clutter = []                                                       # (centre, size, yaw)
    ground_h = 4.5
    variants = {}
    for poly, hgt, s_, shop in blds:
        variants.setdefault((s_, int(rng.integers(tints))), []).append((poly, hgt, shop))
    for (s_, t_), mine in sorted(variants.items()):
        style = dict(styles[s_])
        style["wall_rgb"] = list(np.clip(np.asarray(style["wall_rgb"]) * rng.uniform(0.88, 1.08, 3), 0, 1))
        wp, wi, wst, rp, ri = [], [], [], [], []
        for poly, hgt, shop in mine:
            n = len(poly)
            per = np.r_[0.0, np.cumsum(np.linalg.norm(np.roll(poly, -1, 0) - poly, axis=1))]
            z0 = ground_h if shop and hgt > ground_h + 2 else 0.0
            for k in range(n):
                a, b = poly[k], poly[(k + 1) % n]
                base = len(wp)
                wp += [[a[0], a[1], z0], [b[0], b[1], z0], [b[0], b[1], hgt], [a[0], a[1], hgt]]
                u0, u1 = per[k] / (bay_m * BAYS), per[k + 1] / (bay_m * BAYS)
                wst += [[u0, z0 / (floor_m * FLOORS)], [u1, z0 / (floor_m * FLOORS)], [u1, hgt / (floor_m * FLOORS)],
                        [u0, hgt / (floor_m * FLOORS)]]
                wi += [base, base + 1, base + 2, base + 3]
                if z0 > 0:
                    sb = len(shop_p)
                    shop_p += [[a[0], a[1], 0.0], [b[0], b[1], 0.0], [b[0], b[1], z0], [a[0], a[1], z0]]
                    su0, su1 = per[k] / 24.0, per[k + 1] / 24.0                # 4 shop units per 24 m
                    shop_st += [[su0, 0.0], [su1, 0.0], [su1, 1.0], [su0, 1.0]]
                    shop_i += [sb, sb + 1, sb + 2, sb + 3]
            rb = len(rp)
            rp += [[x, y, hgt] for x, y in poly]
            ri += [rb + t for tri in earclip(poly) for t in tri]
            lo, hi = poly.min(0), poly.max(0)
            boxes.append([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hgt / 2, hi[0] - lo[0], hi[1] - lo[1], hgt])
            # Rooftop clutter: a few boxes inside the footprint, more on bigger roofs.
            area = abs(signed_area(poly))
            want = int(np.clip(area / 120, 0, 8))
            if want:
                cand = rng.uniform(lo, hi, (want * 4, 2))
                cand = cand[point_in_poly(cand, poly)][:want]
                for c in cand:
                    kind = rng.choice(["ac", "ac", "tank", "core"] if hgt > 15 else ["ac", "ac", "tank"])
                    size = {"ac": [1.4, 0.9, 1.0], "tank": [2.5, 2.5, 2.2], "core": [4.5, 4.0, 3.2]}[kind]
                    clutter.append(([c[0], c[1], hgt + size[2] / 2], size, rng.uniform(0, np.pi)))
        name = f"style{s_}_{t_}"
        tex = tex_dir / f"facade_{name}.png"
        facade_texture(tex, style, rng)
        walls = _mesh(stage, f"/World/Buildings/{name}/Walls", wp, [4] * (len(wi) // 4), wi, wst)
        UsdShade.MaterialBindingAPI.Apply(walls.GetPrim()).Bind(preview_material(stage, f"/World/Looks/Facade_{name}", tex))
        walls.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("building")
        roof = _mesh(stage, f"/World/Buildings/{name}/Roofs", rp, [3] * (len(ri) // 3), ri,
                     np.asarray(rp)[ri][:, :2] / 4.0)
        if roof_mdl:
            bind_mdl(stage, roof.GetPrim(), roof_mdl, f"/World/Looks/Roof_{name}")
        roof.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("roof")
    if shop_p:
        tex = tex_dir / "shopfront.png"
        shopfront_texture(tex, rng)
        m = _mesh(stage, "/World/Buildings/Shopfronts", shop_p, [4] * (len(shop_i) // 4), shop_i, shop_st)
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(preview_material(stage, "/World/Looks/Shopfront", tex, roughness=0.5))
        m.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("building")
    if clutter:
        from envbuild.props import MeshBuf

        buf = MeshBuf()
        for c, sz, yaw in clutter:
            buf.box(c, sz, yaw=yaw)
        buf.write(stage, "/World/Buildings/Rooftop", rgb=(0.58, 0.58, 0.56))
        cl = np.array([[c[0], c[1], c[2], max(sz[0], sz[1]), max(sz[0], sz[1]), sz[2]] for c, sz, _ in clutter])
        boxes = boxes + cl.tolist()
    return np.array(boxes)


def write_roads(stage, ways: list[tuple[np.ndarray, float]], material: str, z: float = 0.03,
                tex_dir: Path | None = None, rng=None, sidewalk_m: float = 2.5) -> int:
    """Road ribbons along the OSM centre lines. With `tex_dir`: a generated lane-marking texture (edge lines,
    dashed centre line) and raised sidewalks either side; otherwise the plain `material`."""
    from pxr import Sdf, UsdShade

    from envbuild.procedural import bind_mdl

    pts, idx, st, sw_p, sw_i = [], [], [], [], []
    for line, w in ways:
        along = 0.0
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
            v0, v1 = (along - w / 2) / 6.0, (along + L + w / 2) / 6.0
            st += [[0.0, v0], [0.0, v1], [1.0, v1], [1.0, v0]]
            along += L
            if tex_dir is not None and w >= 6:
                for side in (-1, 1):
                    off = side * (w / 2 + sidewalk_m / 2) / (w / 2)
                    c0, c1 = a + nrm * off, b + nrm * off
                    hw = np.array([-d[1], d[0]]) / L * sidewalk_m / 2
                    sb = len(sw_p)
                    sw_p += [[p[0], p[1], 0.12] for p in (c0 + hw, c1 + hw, c1 - hw, c0 - hw)]
                    sw_i += [sb, sb + 3, sb + 2, sb + 1]
    if not pts:
        return 0
    if tex_dir is None:
        m = _mesh(stage, "/World/Roads", pts, [4] * (len(idx) // 4), idx,
                  np.asarray(pts)[idx][:, :2] / 6.0, collider=False)
        bind_mdl(stage, m.GetPrim(), material, "/World/Looks/Road")
    else:
        # st was authored per vertex in quad order; idx winds each quad as (0, 3, 2, 1).
        fv = np.asarray(st).reshape(-1, 4, 2)[:, [0, 3, 2, 1]].reshape(-1, 2)
        m = _mesh(stage, "/World/Roads", pts, [4] * (len(idx) // 4), idx, fv, collider=False)
        tex = tex_dir / "road.png"
        ground_texture(tex, rng, "road")
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(preview_material(stage, "/World/Looks/Road", tex, roughness=0.9))
        if sw_p:
            stex = tex_dir / "sidewalk.png"
            ground_texture(stex, rng, "sidewalk")
            sw = _mesh(stage, "/World/Sidewalks", sw_p, [4] * (len(sw_i) // 4), sw_i,
                       np.asarray(sw_p)[sw_i][:, :2] / 2.4, collider=False)
            UsdShade.MaterialBindingAPI.Apply(sw.GetPrim()).Bind(preview_material(stage, "/World/Looks/Sidewalk", stex))
            sw.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("sidewalk")
    m.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("road")
    return len(idx) // 4


def write_ground(stage, half: float, material: str) -> None:
    pts = [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]]
    m = _mesh(stage, "/World/Ground", pts, [4], [0, 1, 2, 3], np.asarray(pts)[:, :2] / 4.0)
    from envbuild.procedural import bind_mdl
    from pxr import Sdf

    bind_mdl(stage, m.GetPrim(), material, "/World/Looks/Ground")
    m.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("terrain")
