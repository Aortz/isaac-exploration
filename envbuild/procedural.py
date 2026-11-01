"""Procedural terrain + scattered assets (forest, desert, ...) as a sim-ready USD stage.

Isaac Python only (pxr + Isaac asset root). Writes into an open Usd.Stage:
  /World/Terrain      heightfield mesh, triangle-mesh collider, MDL ground material
  /World/Scatter/<g>  one PointInstancer per scatter group (trees, shrubs, rocks ...)
  /World/Sky          dome light (HDR) + sun
and returns the sidecars the rest of the pipeline uses instead of walking instances:
  heightmap (x0, y0, res, heights)  ground clearance (AGL) for flight planning
  obstacles (N x 6: centre, size)   instance bounding boxes, same format as env_probe's meshes.npy
"""

from __future__ import annotations

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

from envbuild.procedural_math import height_at
from pipeline.common.environments import asset_url


# --- terrain -------------------------------------------------------------------------------------
def value_noise(nx: int, ny: int, cell: float, rng) -> np.ndarray:
    """Smooth value noise on an nx x ny grid with lattice spacing `cell` samples (cosine interpolation)."""
    gx, gy = int(np.ceil(nx / cell)) + 2, int(np.ceil(ny / cell)) + 2
    lattice = rng.uniform(-1.0, 1.0, (gy, gx))
    x, y = np.arange(nx) / cell, np.arange(ny) / cell
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = (1 - np.cos(np.pi * (x - x0))) / 2, (1 - np.cos(np.pi * (y - y0))) / 2
    a = lattice[np.ix_(y0, x0)]
    b = lattice[np.ix_(y0, x0 + 1)]
    c = lattice[np.ix_(y0 + 1, x0)]
    d = lattice[np.ix_(y0 + 1, x0 + 1)]
    top = a + (b - a) * fx[None, :]
    bot = c + (d - c) * fx[None, :]
    return top + (bot - top) * fy[:, None]


def fbm(nx: int, ny: int, base_cell: float, octaves: int, rng, persistence: float = 0.5) -> np.ndarray:
    out, amp, cell, norm = np.zeros((ny, nx)), 1.0, base_cell, 0.0
    for _ in range(octaves):
        out += amp * value_noise(nx, ny, max(cell, 1.0), rng)
        norm += amp
        amp *= persistence
        cell /= 2.0
    return out / norm


def make_heights(tcfg: dict, rng) -> tuple[np.ndarray, float, float, float]:
    """Height grid (rows = +y) for the terrain recipe. Returns (heights, x0, y0, res)."""
    size, res = float(tcfg["size_m"]), float(tcfg["resolution_m"])
    n = int(size / res) + 1
    x0 = y0 = -size / 2
    h = float(tcfg.get("hills_m", 0.0)) * fbm(n, n, tcfg.get("hill_scale_m", 200.0) / res, int(tcfg.get("octaves", 5)), rng)
    xs = x0 + np.arange(n) * res
    X, Y = np.meshgrid(xs, xs)
    if tcfg.get("dunes"):
        d = tcfg["dunes"]
        ang = np.radians(d.get("direction_deg", 30.0))
        u = X * np.cos(ang) + Y * np.sin(ang)
        warp = d.get("warp_m", 20.0) * fbm(n, n, 150.0 / res, 3, rng)
        phase = 2 * np.pi * (u + warp) / d["wavelength_m"]
        # Asymmetric dune profile: gentle windward slope, steeper slip face.
        h += d["height_m"] * (0.5 + 0.5 * np.sin(phase + 0.35 * np.sin(phase)))
    if tcfg.get("terraces"):
        # Contour terraces (plantations on hills): flat benches of `step_m`, joined by a steep riser over the
        # last `blend` fraction of each step.
        t = tcfg["terraces"]
        f = h / t["step_m"]
        frac = f - np.floor(f)
        b = t.get("blend", 0.3)
        h = t["step_m"] * (np.floor(f) + np.clip((frac - (1 - b)) / b, 0, 1))
    # Meandering valleys / tidal creeks: lower the terrain near a sinusoidal centre line. `valley` is one
    # running along x through the origin; `channels` take an angle (deg from +x) and an across-offset.
    if tcfg.get("craters"):
        for cx, cy, r in crater_list(tcfg):
            # Bowl inside the radius, raised rim just outside it.
            d = np.hypot(X - cx, Y - cy) / r
            c = tcfg["craters"]
            h += np.where(d < 1, -c.get("depth_ratio", 0.25) * r * (1 - d ** 2), 0.0)
            h += c.get("rim_ratio", 0.08) * r * np.exp(-((d - 1.0) / 0.3) ** 2)
    chans = ([tcfg["valley"]] if tcfg.get("valley") else []) + list(tcfg.get("channels", []))
    for v in chans:
        if "floor_m" not in v:
            h -= v["depth_m"] * channel_weight(v, X, Y)
    h -= h[n // 2, n // 2]                                          # ground at the origin = z 0
    for v in chans:
        if "floor_m" in v:
            # Flat-bottomed river: pull the ground down to a fixed level (after the origin is levelled) so a
            # flat water plane fits the whole length.
            w = channel_weight(v, X, Y)
            h = h * (1 - w) + np.minimum(h, v["floor_m"]) * w
    if tcfg.get("shore"):
        # Land falling away into open water beyond `at_m` along a direction (a coastline), after the
        # origin is levelled so the take-off point keeps z 0.
        s = tcfg["shore"]
        a = np.radians(s.get("direction_deg", 90.0))
        along = X * np.cos(a) + Y * np.sin(a)
        h -= s["drop_m"] / (1.0 + np.exp(-(along - s["at_m"]) / s.get("width_m", 80.0)))
    return h, x0, y0, res


def crater_list(tcfg: dict) -> np.ndarray:
    """(cx, cy, radius) per crater, from the craters block's own seed (so masks can recompute them)."""
    c = tcfg["craters"]
    r = np.random.default_rng(c.get("seed", 0))
    half = c.get("area_m", tcfg["size_m"] / 2 * 0.9)
    n = int(c["count"])
    xy = r.uniform(-half, half, (n, 2))
    keep = np.hypot(xy[:, 0], xy[:, 1]) > c.get("clear_radius_m", 25.0)   # keep the take-off point intact
    rad = r.uniform(*c.get("radius_m", [2.0, 6.0]), n)
    return np.column_stack([xy, rad])[keep]


def crater_field(tcfg: dict, x0: float, y0: float, res: float, n: int) -> np.ndarray:
    xs = x0 + np.arange(n) * res
    X, Y = np.meshgrid(xs, xs)
    f = np.zeros((n, n))
    for cx, cy, r in crater_list(tcfg):
        f = np.maximum(f, np.clip(1.3 - np.hypot(X - cx, Y - cy) / r, 0, 1))   # bowl + scorched apron
    return f


def channel_weight(v: dict, X, Y) -> np.ndarray:
    """0..1 profile of one channel (`valley` / `channels` entry) over the grid: 1 on its centre line."""
    a = np.radians(v.get("angle_deg", 0.0))
    along, across = X * np.cos(a) + Y * np.sin(a), -X * np.sin(a) + Y * np.cos(a)
    centre = v.get("offset_m", 0.0) + v.get("meander_m", 60.0) * np.sin(
        2 * np.pi * along / v.get("meander_wavelength_m", 400.0) + v.get("phase_deg", 0.0) * np.pi / 180)
    prof = np.exp(-((across - centre) / v["width_m"]) ** 2)
    if "flat_frac" in v:                    # wider flat floor: saturate the centre
        prof = np.clip(prof / (1 - v["flat_frac"]), 0, 1)
    return prof


def channels_field(tcfg: dict, x0: float, y0: float, res: float, n: int) -> np.ndarray:
    """Max channel weight over the grid (roads, creeks): a mask field for scatter groups and ground layers."""
    xs = x0 + np.arange(n) * res
    X, Y = np.meshgrid(xs, xs)
    chans = ([tcfg["valley"]] if tcfg.get("valley") else []) + list(tcfg.get("channels", []))
    return np.max([channel_weight(v, X, Y) for v in chans], axis=0) if chans else np.zeros((n, n))


def face_layers(heights: np.ndarray, res: float, layers: list[dict], masks: dict) -> np.ndarray:
    """Material layer index per terrain quad: 0 = base, then each layer whose conditions all hold (last wins).

    Conditions: mask (a recipe mask name, `!name` = outside it), height_range_m, slope_range_deg."""
    hc = 0.25 * (heights[:-1, :-1] + heights[:-1, 1:] + heights[1:, :-1] + heights[1:, 1:])
    sl = slope_deg(heights, res)[:-1, :-1]
    out = np.zeros(hc.shape, int)
    for k, lay in enumerate(layers[1:], start=1):
        ok = np.ones(hc.shape, bool)
        if lay.get("mask"):
            name = lay["mask"].lstrip("!")
            field, thr, inv = masks[name]
            m = field[:-1, :-1] >= thr
            ok &= ~m if inv != lay["mask"].startswith("!") else m
        if "height_range_m" in lay:
            ok &= (hc >= lay["height_range_m"][0]) & (hc <= lay["height_range_m"][1])
        if "slope_range_deg" in lay:
            ok &= (sl >= lay["slope_range_deg"][0]) & (sl <= lay["slope_range_deg"][1])
        out[ok] = k
    return out.ravel()


def write_terrain(stage, heights: np.ndarray, x0: float, y0: float, res: float, material: str | None,
                  uv_tile_m: float = 4.0, layers: list[dict] | None = None, face_layer: np.ndarray | None = None) -> None:
    ny, nx = heights.shape
    xs, ys = x0 + np.arange(nx) * res, y0 + np.arange(ny) * res
    X, Y = np.meshgrid(xs, ys)
    pts = np.column_stack([X.ravel(), Y.ravel(), heights.ravel()]).astype(np.float32)
    i = np.arange(ny - 1)[:, None] * nx + np.arange(nx - 1)[None, :]
    quads = np.stack([i, i + 1, i + nx + 1, i + nx], axis=-1).reshape(-1, 4)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Terrain")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(quads), 4, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(quads.ravel().astype(np.int32)))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray.FromNumpy((pts[:, :2] / uv_tile_m).astype(np.float32)))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.none)
    mesh.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("terrain")
    if material:
        bind_mdl(stage, mesh.GetPrim(), material, "/World/Looks/Ground")
    if layers and face_layer is not None:
        # Ground material per quad (forest floor under canopy, mud by the water ...) as materialBind subsets.
        UsdGeom.Subset.SetFamilyType(mesh, "materialBind", UsdGeom.Tokens.partition)
        for k, lay in enumerate(layers):
            faces = np.flatnonzero(face_layer == k)
            if not len(faces):
                continue
            sub = UsdGeom.Subset.CreateGeomSubset(mesh, f"layer{k}", UsdGeom.Tokens.face,
                                                  Vt.IntArray.FromNumpy(faces.astype(np.int32)), "materialBind")
            bind_mdl(stage, sub.GetPrim(), lay["material"], f"/World/Looks/Ground{k}")


def bind_mdl(stage, prim, mdl_path: str, look_path: str) -> None:
    """Bind an MDL material: /NVIDIA/Materials/Base/Natural/Sand.mdl (export = file stem), or pick another
    export of the module with a suffix, e.g. /NVIDIA/Materials/vMaterials_2/Ground/Mulch.mdl#Mulch_Wet."""
    mdl_path, _, export = mdl_path.partition("#")
    mat = UsdShade.Material.Define(stage, look_path)
    sh = UsdShade.Shader.Define(stage, look_path + "/Shader")
    sh.SetSourceAsset(Sdf.AssetPath(asset_url(mdl_path)), "mdl")
    sh.SetSourceAssetSubIdentifier(export or mdl_path.rsplit("/", 1)[-1].rsplit(".", 1)[0], "mdl")
    sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def slope_deg(heights, res) -> np.ndarray:
    gy, gx = np.gradient(heights, res)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


# --- scatter -------------------------------------------------------------------------------------
def asset_bounds(url: str) -> np.ndarray:
    """(min xyz, max xyz) of an asset in its own frame, metres (stage units honoured)."""
    st = Usd.Stage.Open(url)
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    root = st.GetDefaultPrim() or st.GetPseudoRoot()
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"]).ComputeWorldBound(root).ComputeAlignedRange()
    if r.IsEmpty():
        # Some assets carry no extents the cache can use: bound the mesh points directly.
        pts = [np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get() or [], float).reshape(-1, 3)
               @ np.asarray(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))[:3, :3]
               + np.asarray(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))[3, :3]
               for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
        pts = np.vstack([q for q in pts if len(q)]) if any(len(q) for q in pts) else None
        if pts is None:
            raise ValueError(f"cannot bound asset {url}")
        lo, hi = pts.min(0) * mpu, pts.max(0) * mpu
    else:
        lo, hi = np.array(r.GetMin()) * mpu, np.array(r.GetMax()) * mpu
    up = UsdGeom.GetStageUpAxis(st)
    if up == "Y":                                                   # Y-up asset: (x, y, z) -> (x, -z, y)
        lo, hi = np.array([lo[0], -hi[2], lo[1]]), np.array([hi[0], -lo[2], hi[1]])
    return np.array([lo, hi]), mpu, up


def scatter_points(group: dict, heights, x0, y0, res, rng, mask_fn=None) -> np.ndarray:
    """Jittered-grid points at `density_per_ha` with min spacing, filtered by slope, height and mask."""
    ny, nx = heights.shape
    size_x, size_y = (nx - 1) * res, (ny - 1) * res
    if group.get("layout") == "perimeter":
        # Along a rectangle's sides (walls, fences), with a gate gap mid-south; yaw follows the side.
        (w, hgt), (cx, cy) = group["rect_m"], group.get("centre_m", [0.0, 0.0])
        sp, gap = float(group["spacing_m"]), float(group.get("gate_m", 0.0))
        out = []
        for (ax, ay), (bx, by) in (((-w / 2, -hgt / 2), (w / 2, -hgt / 2)), ((w / 2, -hgt / 2), (w / 2, hgt / 2)),
                                   ((w / 2, hgt / 2), (-w / 2, hgt / 2)), ((-w / 2, hgt / 2), (-w / 2, -hgt / 2))):
            L = np.hypot(bx - ax, by - ay)
            t = np.arange(sp / 2, L, sp)
            px, py = ax + (bx - ax) * t / L, ay + (by - ay) * t / L
            ok = ~((ay == -hgt / 2) & (by == -hgt / 2) & (np.abs(px) < gap / 2))
            out.append(np.column_stack([cx + px[ok], cy + py[ok], np.full(ok.sum(), np.arctan2(by - ay, bx - ax))]))
        o = np.vstack(out)
        z = height_at(heights, x0, y0, res, o[:, 0], o[:, 1])
        return np.column_stack([o[:, 0], o[:, 1], z, o[:, 2]])
    if group.get("layout") == "grid":
        # Planted rows: triangular lattice at `spacing_m`, rows along `row_angle_deg`, small jitter.
        sp = float(group["spacing_m"])
        half = np.hypot(size_x, size_y) / 2
        cx, cy = x0 + size_x / 2, y0 + size_y / 2
        rows = np.arange(-half, half, sp * np.sqrt(3) / 2)
        cols = np.arange(-half, half, sp)
        U, V = np.meshgrid(cols, rows)
        U = U + (np.arange(len(rows))[:, None] % 2) * sp / 2
        a = np.radians(group.get("row_angle_deg", 0.0))
        jit = group.get("jitter_m", 0.3)
        x = (cx + U * np.cos(a) - V * np.sin(a)).ravel() + rng.uniform(-jit, jit, U.size)
        y = (cy + U * np.sin(a) + V * np.cos(a)).ravel() + rng.uniform(-jit, jit, U.size)
        inside = (x >= x0) & (x <= x0 + size_x) & (y >= y0) & (y <= y0 + size_y)
        x, y = x[inside], y[inside]
    else:
        spacing = 100.0 / np.sqrt(group["density_per_ha"])           # mean spacing (m) for that density
        gx, gy = np.arange(x0 + spacing / 2, x0 + size_x, spacing), np.arange(y0 + spacing / 2, y0 + size_y, spacing)
        X, Y = np.meshgrid(gx, gy)
        jit = group.get("jitter", 0.8) * spacing / 2
        x = (X + rng.uniform(-jit, jit, X.shape)).ravel()
        y = (Y + rng.uniform(-jit, jit, Y.shape)).ravel()
    keep = np.ones(len(x), bool)
    if "slope_max_deg" in group:
        sl = slope_deg(heights, res)
        keep &= height_at(sl, x0, y0, res, x, y) <= group["slope_max_deg"]
    z = height_at(heights, x0, y0, res, x, y)
    if "height_range_m" in group:
        keep &= (z >= group["height_range_m"][0]) & (z <= group["height_range_m"][1])
    if mask_fn is not None:
        keep &= mask_fn(x, y)
    clear = group.get("clear_radius_m", 0.0)                         # keep a clearing around the origin
    keep &= np.hypot(x, y) >= clear
    return np.column_stack([x[keep], y[keep], z[keep]])


def write_scatter(stage, name: str, group: dict, pts: np.ndarray, rng) -> np.ndarray:
    """One PointInstancer for the group; returns per-instance AABBs (centre, size) for clearance checks."""
    path = f"/World/Scatter/{name}"
    inst = UsdGeom.PointInstancer.Define(stage, path)
    protos, bounds = [], []
    for k, a in enumerate(group["assets"]):
        url = asset_url(a)
        b, mpu, up = asset_bounds(url)
        bounds.append(b)
        # Unit / up-axis fix on a wrapper: the asset's own root may already author xform ops.
        proto = UsdGeom.Xform.Define(stage, f"{path}/Prototypes/p{k}")
        UsdGeom.Xform.Define(stage, f"{path}/Prototypes/p{k}/asset").GetPrim().GetReferences().AddReference(url)
        xf = UsdGeom.Xformable(proto)
        if mpu != 1.0:
            xf.AddScaleOp().Set(Gf.Vec3f(mpu, mpu, mpu))
        if up == "Y":
            xf.AddRotateXOp().Set(90.0)
        proto.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set(group.get("class", name))
        protos.append(proto.GetPath())
    inst.CreatePrototypesRel().SetTargets(protos)
    n = len(pts)
    w = np.asarray(group.get("weights", [1.0] * len(protos)), float)
    idx = rng.choice(len(protos), n, p=w / w.sum())
    s_lo, s_hi = group.get("scale_range", [0.8, 1.2])
    scale = rng.uniform(s_lo, s_hi, n)
    if pts.shape[1] == 4:                                            # layout-given yaw (perimeter walls)
        yaw = pts[:, 3]
    elif group.get("align"):                                         # rows (tents, vehicles): face along the rows
        yaw = np.radians(group.get("row_angle_deg", 0.0)) + rng.normal(0, np.radians(group.get("yaw_jitter_deg", 3.0)), n)
    else:
        yaw = rng.uniform(0, 2 * np.pi, n)
    sink = group.get("sink_m", 0.0)                                  # push rocks/trunks into the ground a bit
    pts = pts[:, :3]
    pos = pts - np.array([0.0, 0.0, sink])
    inst.CreateProtoIndicesAttr(Vt.IntArray.FromNumpy(idx.astype(np.int32)))
    inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(pos.astype(np.float32)))
    q = np.column_stack([np.cos(yaw / 2), np.zeros(n), np.zeros(n), np.sin(yaw / 2)])   # w, x, y, z about +z
    inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(*map(float, r)) for r in q]))
    inst.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(np.repeat(scale[:, None], 3, 1).astype(np.float32)))
    # Instance AABBs: prototype box scaled, yaw handled by taking the larger horizontal half-extent.
    out = np.zeros((n, 6))
    for k, b in enumerate(bounds):
        sel = idx == k
        half_xy = np.max(np.abs(b[:, :2])) * scale[sel]
        zlo, zhi = b[0, 2] * scale[sel], b[1, 2] * scale[sel]
        c = pos[sel] + np.column_stack([np.zeros(sel.sum()), np.zeros(sel.sum()), (zlo + zhi) / 2])
        out[sel] = np.column_stack([c, 2 * half_xy, 2 * half_xy, zhi - zlo])
    return out


def write_water(stage, w: dict) -> None:
    """Flat water surface at `level_m` (no collider: the heightmap sidecar treats it as the ground)."""
    half = float(w.get("size_m", 4000.0)) / 2
    z = float(w["level_m"])
    mesh = UsdGeom.Mesh.Define(stage, "/World/Water")
    pts = np.array([[-half, -half, z], [half, -half, z], [half, half, z], [-half, half, z]], np.float32)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)] * 4)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, half, z)])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray.FromNumpy((pts[:, :2] / float(w.get("uv_tile_m", 10.0))).astype(np.float32)))
    mesh.GetPrim().CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set("water")
    bind_mdl(stage, mesh.GetPrim(), w["material"], "/World/Looks/Water")


# --- generated assets ----------------------------------------------------------------------------
def _tube(path_pts: np.ndarray, r0: float, r1: float, sides: int = 6) -> tuple[np.ndarray, np.ndarray]:
    """Tapered tube along a polyline: (points, quads)."""
    m = len(path_pts)
    tang = np.gradient(path_pts, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    ref = np.where(np.abs(tang[:, 2:3]) > 0.9, [[1.0, 0.0, 0.0]], [[0.0, 0.0, 1.0]])
    nrm = np.cross(tang, ref)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    bin_ = np.cross(tang, nrm)
    ang = 2 * np.pi * np.arange(sides) / sides
    rad = np.linspace(r0, r1, m)[:, None, None]
    ring = np.cos(ang)[None, :, None] * nrm[:, None, :] + np.sin(ang)[None, :, None] * bin_[:, None, :]
    pts = (path_pts[:, None, :] + rad * ring).reshape(-1, 3)
    i = np.arange(m - 1)[:, None] * sides + np.arange(sides)[None, :]
    j = np.arange(m - 1)[:, None] * sides + (np.arange(sides)[None, :] + 1) % sides
    quads = np.stack([i, j, j + sides, i + sides], axis=-1).reshape(-1, 4)
    return pts, quads


def write_mangrove_assets(out_dir, gen: dict, rng) -> list[str]:
    """Mangrove trees (Rhizophora-like): arching prop roots from a raised hub, a short trunk, and a crown
    referenced from an existing broadleaf asset. Writes `variants` USD files; returns their paths.

    `crowns`: list of {asset, scale: [lo, hi], hide: [child prims]}, used in turn across the variants."""
    from pathlib import Path

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    crowns = []
    for c in gen["crowns"]:
        url = asset_url(c["asset"])
        crowns.append((c, url, *asset_bounds(url)))
    paths = []
    for k in range(int(gen.get("variants", 4))):
        c, crown_url, cb, cmpu, cup = crowns[k % len(crowns)]
        f = out_dir / f"mangrove_{k}.usda"
        st = Usd.Stage.CreateNew(str(f))
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        root = UsdGeom.Xform.Define(st, "/Mangrove")
        st.SetDefaultPrim(root.GetPrim())
        hub = rng.uniform(*gen.get("hub_height_m", [1.0, 1.8]))
        spread = rng.uniform(*gen.get("root_spread_m", [1.6, 3.0]))
        pts_all, quads_all, off = [], [], 0
        n_roots = int(rng.integers(*gen.get("roots", [14, 22])))
        for r in range(n_roots):
            th = 2 * np.pi * (r + rng.uniform(-0.3, 0.3)) / n_roots
            d = np.array([np.cos(th), np.sin(th), 0.0])
            z0 = hub * rng.uniform(0.6, 1.05)
            reach = spread * rng.uniform(0.55, 1.1)
            p0 = d * 0.12 + [0, 0, z0]
            p1 = d * reach * 0.45 + [0, 0, z0 + rng.uniform(0.2, 0.6)]
            p2 = d * reach + [0, 0, -0.3]
            t = np.linspace(0, 1, 10)[:, None]
            curve = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2
            p, q = _tube(curve, 0.07, 0.04)
            pts_all.append(p)
            quads_all.append(q + off)
            off += len(p)
        p, q = _tube(np.array([[0, 0, hub * 0.5], [0, 0, hub + 0.4], [0, 0, hub + 1.2]]), 0.16, 0.13, 8)
        pts_all.append(p)
        quads_all.append(q + off)
        pts, quads = np.vstack(pts_all).astype(np.float32), np.vstack(quads_all)
        mesh = UsdGeom.Mesh.Define(st, "/Mangrove/Roots")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(quads), 4, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(quads.ravel().astype(np.int32)))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
        mat = UsdShade.Material.Define(st, "/Mangrove/Looks/Bark")
        sh = UsdShade.Shader.Define(st, "/Mangrove/Looks/Bark/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*gen.get("bark_rgb", [0.26, 0.22, 0.18])))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        # Crown: the broadleaf asset scaled down, its trunk base on the root hub.
        s = rng.uniform(*c.get("scale", [1.0, 1.0]))
        crown = UsdGeom.Xform.Define(st, "/Mangrove/Crown")
        cx = UsdGeom.Xformable(crown)
        cx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, hub + 0.3 - cb[0, 2] * s))
        cx.AddRotateZOp().Set(float(rng.uniform(0, 360)))
        cx.AddScaleOp().Set(Gf.Vec3f(s * cmpu, s * cmpu, s * cmpu))
        if cup == "Y":
            cx.AddRotateXOp().Set(90.0)
        ref = UsdGeom.Xform.Define(st, "/Mangrove/Crown/asset").GetPrim()
        ref.GetReferences().AddReference(crown_url)
        if c.get("hide"):
            # e.g. the fruit on an orange tree; an instanceable asset ignores overrides below it
            ref.SetInstanceable(False)
            for child in c["hide"]:
                # Invisible rather than inactive: RTX still drew deactivated prims inside instancer prototypes.
                UsdGeom.Imageable(st.OverridePrim(f"/Mangrove/Crown/asset/{child}")).CreateVisibilityAttr(
                    UsdGeom.Tokens.invisible)
        st.GetRootLayer().Save()
        paths.append(str(f))
    return paths


def write_sky(stage, sky: dict) -> None:
    dome = UsdLux.DomeLight.Define(stage, "/World/Sky/Dome")
    if sky.get("hdr"):
        dome.CreateTextureFileAttr().Set(Sdf.AssetPath(asset_url(sky["hdr"])))
        dome.CreateTextureFormatAttr().Set(UsdLux.Tokens.latlong)
    dome.CreateIntensityAttr(float(sky.get("intensity", 1000.0)))
    if sky.get("sun_intensity", 0.0) > 0:
        sun = UsdLux.DistantLight.Define(stage, "/World/Sky/Sun")
        sun.CreateIntensityAttr(float(sky["sun_intensity"]))
        sun.CreateAngleAttr(0.53)
        el, az = np.radians(sky.get("sun_elevation_deg", 45.0)), np.radians(sky.get("sun_azimuth_deg", 135.0))
        # DistantLight shines along its -Z: aim it from the sun direction towards the ground.
        d = -np.array([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)])
        z = -d
        x = np.cross([0.0, 0.0, 1.0], z) if abs(z[2]) < 0.999 else np.array([1.0, 0.0, 0.0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        m = Gf.Matrix4d(*np.r_[x, 0, y, 0, z, 0, 0, 0, 0, 1].tolist())
        UsdGeom.Xformable(sun).AddTransformOp().Set(m)
