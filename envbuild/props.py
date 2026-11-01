"""Generated props for battleground / military scenes (no such assets ship with Isaac).

Each generator writes `variants` small USD assets into build/envs/<name>/assets/ and returns their paths;
a scatter group with `generate: <type>` then instances them like any other asset (envbuild/build_env.py):

  ruin            bombed building shell: block walls with holes, a broken top edge, part-collapsed slabs
  rubble          pile of concrete chunks
  vehicle_proxy   armoured-vehicle stand-in: hull, glacis, turret, barrel, road wheels (intact or burned)
  container       20 ft shipping container
  hesco           HESCO-style barrier cell (1.1 m earth-filled mesh basket)
  jersey_barrier  concrete road barrier
  sandbags        sandbag wall segment
  tent            ridge tent
  tower           guard tower on legs
  dead_tree       bare, charred tree (recursive branches)

Geometry is plain boxes / tubes with UsdPreviewSurface colours (or an MDL via `material`); the point is
believable silhouettes, scale and texture density for VIO, not hero assets.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

from envbuild.procedural import _tube, bind_mdl


class MeshBuf:
    """Accumulates boxes / tubes into one mesh."""

    def __init__(self):
        self.pts, self.counts, self.idx = [], [], []

    def box(self, centre, size, yaw=0.0, tilt=(0.0, 0.0)):
        c, s = np.asarray(centre, float), np.asarray(size, float) / 2
        corners = np.array([[sx, sy, sz] for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]) * s
        rx, ry = np.radians(tilt)
        Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        corners = corners @ (Rz @ Ry @ Rx).T + c
        b = len(self.pts)
        self.pts += corners.tolist()
        for f in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
            self.counts.append(4)
            self.idx += [b + i for i in f]

    def prism(self, centre, length, width, height, yaw=0.0):
        """Ridge (tent) shape along local x."""
        c = np.asarray(centre, float)
        L, W = length / 2, width / 2
        p = np.array([[-L, -W, 0], [L, -W, 0], [L, W, 0], [-L, W, 0], [-L, 0, height], [L, 0, height]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        p = p @ Rz.T + c
        b = len(self.pts)
        self.pts += p.tolist()
        for f in ((0, 1, 5, 4), (2, 3, 4, 5), (0, 4, 3), (1, 2, 5), (0, 3, 2, 1)):
            self.counts.append(len(f))
            self.idx += [b + i for i in f]

    def tube(self, path_pts, r0, r1, sides=6):
        p, q = _tube(np.asarray(path_pts, float), r0, r1, sides)
        b = len(self.pts)
        self.pts += p.tolist()
        self.counts += [4] * len(q)
        self.idx += (q + b).ravel().tolist()

    def write(self, stage, path, look=None, rgb=(0.5, 0.5, 0.5), mdl=None, roughness=0.85):
        if not self.pts:
            return None
        pts = np.asarray(self.pts, np.float32)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(self.counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(self.idx))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateDoubleSidedAttr(True)
        m.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
        # World-ish UVs (top-down + side) so tiling MDL textures have a scale.
        fv = pts[self.idx]
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set(Vt.Vec2fArray.FromNumpy(np.column_stack([fv[:, 0] + fv[:, 1], fv[:, 2] + 0.5 * fv[:, 1]]).astype(np.float32) / 2.0))
        look = look or path + "_Look"
        if mdl:
            bind_mdl(stage, m.GetPrim(), mdl, look)
        else:
            mat = UsdShade.Material.Define(stage, look)
            sh = UsdShade.Shader.Define(stage, look + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, rgb)))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        return m


def _stage(path: Path):
    st = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.SetDefaultPrim(UsdGeom.Xform.Define(st, "/Prop").GetPrim())
    return st


def _jit(rgb, rng, amt=0.06):
    return np.clip(np.asarray(rgb, float) * (1 + rng.uniform(-amt, amt)), 0, 1)


CONCRETE = "/NVIDIA/Materials/Base/Masonry/Concrete_Block.mdl"
# Concrete_Rough / Rough_Gravel render near-white under a strong sun; these keep contrast and texture.
MASONRY = ["/NVIDIA/Materials/Base/Masonry/Adobe_Brick.mdl", "/NVIDIA/Materials/Base/Masonry/Brick_Wall_Brown.mdl",
           "/NVIDIA/Materials/Base/Masonry/Concrete_Block.mdl", "/NVIDIA/Materials/Base/Masonry/Stucco.mdl"]


# --- generators -----------------------------------------------------------------------------------
def ruin(st, g, rng):
    w, d = rng.uniform(*g.get("width_m", [8, 16])), rng.uniform(*g.get("depth_m", [8, 14]))
    floors = int(rng.integers(*g.get("floors", [1, 4])))
    fh, cell, t = 3.0, 1.5, 0.3
    H = floors * fh
    walls, slabs, scorch = MeshBuf(), MeshBuf(), MeshBuf()
    damage = rng.uniform(*g.get("damage", [0.25, 0.6]))
    # Broken top edge: each wall column keeps a random height, lower where the damage is.
    for side in range(4):
        L = w if side % 2 == 0 else d
        nc = max(int(L / cell), 2)
        top = H * np.clip(1 - damage * np.abs(np.cumsum(rng.normal(0, 0.35, nc))), 0.15, 1.0)
        for i in range(nc):
            u = -L / 2 + (i + 0.5) * L / nc
            for z0 in np.arange(0, top[i], cell):
                hz = min(cell, top[i] - z0)
                # holes: windows on a regular grid + random blast holes
                win = (i % 3 == 1) and (z0 % fh > 0.6) and (z0 % fh < 2.2)
                if win or rng.random() < damage * 0.25:
                    continue
                if side == 0:
                    c, sz = (u, -d / 2, z0 + hz / 2), (L / nc, t, hz)
                elif side == 2:
                    c, sz = (u, d / 2, z0 + hz / 2), (L / nc, t, hz)
                elif side == 1:
                    c, sz = (w / 2, u, z0 + hz / 2), (t, L / nc, hz)
                else:
                    c, sz = (-w / 2, u, z0 + hz / 2), (t, L / nc, hz)
                (scorch if rng.random() < 0.15 else walls).box(c, sz)
    for f in range(1, floors):
        # Part-collapsed slabs: some sag at an angle, some are gone.
        if rng.random() < damage * 0.6:
            continue
        tilt = (rng.uniform(-12, 12), rng.uniform(-12, 12)) if rng.random() < damage else (0.0, 0.0)
        slabs.box((0, 0, f * fh), (w - 2 * t, d - 2 * t, 0.25), tilt=tilt)
    for _ in range(int(20 + 40 * damage)):                        # debris around the base
        s = rng.uniform(0.3, 1.2, 3)
        walls.box((rng.uniform(-w / 2 - 2, w / 2 + 2), rng.uniform(-d / 2 - 2, d / 2 + 2), s[2] / 3), s,
                  yaw=rng.uniform(0, np.pi), tilt=rng.uniform(-30, 30, 2))
    mats = g.get("materials", MASONRY)
    walls.write(st, "/Prop/Walls", mdl=mats[int(rng.integers(len(mats)))])
    slabs.write(st, "/Prop/Slabs", mdl=CONCRETE)
    scorch.write(st, "/Prop/Scorched", rgb=(0.08, 0.075, 0.07))


def rubble(st, g, rng):
    b = MeshBuf()
    r = rng.uniform(*g.get("radius_m", [1.5, 4.0]))
    for _ in range(int(rng.integers(25, 70))):
        q = rng.uniform(0, 2 * np.pi)
        rr = r * np.sqrt(rng.uniform())
        s = rng.uniform(0.2, 1.1, 3)
        z = max(0.0, (1 - rr / r)) * r * 0.5 * rng.uniform(0.3, 1)
        b.box((rr * np.cos(q), rr * np.sin(q), z), s, yaw=rng.uniform(0, np.pi), tilt=rng.uniform(-40, 40, 2))
    b.write(st, "/Prop/Chunks", mdl=g.get("material", CONCRETE))


def vehicle_proxy(st, g, rng):
    burned = rng.random() < g.get("burned_frac", 0.0)
    body = (0.07, 0.065, 0.06) if burned else _jit(g.get("rgb", [0.27, 0.30, 0.20]), rng)
    hull, dark = MeshBuf(), MeshBuf()
    L, W = rng.uniform(6.0, 7.5), rng.uniform(3.0, 3.5)
    hull.box((0, 0, 1.05), (L, W, 1.1))
    hull.box((L / 2 - 0.5, 0, 1.45), (1.2, W * 0.95, 0.5), tilt=(0, 20))          # glacis
    if rng.random() < g.get("turret_frac", 0.7):
        hull.box((-0.4, 0, 1.95), (2.6, 2.2, 0.75), yaw=rng.uniform(-0.6, 0.6))
        a = rng.uniform(-0.5, 0.5)
        hull.tube([[-0.4, 0, 2.0], [-0.4 + 4.2 * np.cos(a), 4.2 * np.sin(a), 2.1]], 0.08, 0.07)
    n = 6 if rng.random() < 0.5 else 4
    for side in (-1, 1):
        for x in np.linspace(-L / 2 + 0.8, L / 2 - 0.8, n):
            dark.tube([[x, side * W / 2, 0.55], [x, side * (W / 2 + 0.3), 0.55]], 0.55, 0.55, 10)
    hull.write(st, "/Prop/Hull", rgb=body, roughness=0.7)
    dark.write(st, "/Prop/Wheels", rgb=(0.05, 0.05, 0.05))


def container(st, g, rng):
    b = MeshBuf()
    b.box((0, 0, 1.295), (6.06, 2.44, 2.59))
    palette = g.get("palette", [[0.55, 0.18, 0.12], [0.15, 0.28, 0.45], [0.30, 0.34, 0.24], [0.75, 0.62, 0.25]])
    b.write(st, "/Prop/Box", rgb=_jit(palette[int(rng.integers(len(palette)))], rng), roughness=0.6)


def hesco(st, g, rng):
    b = MeshBuf()
    n = int(g.get("cells", 4))
    for i in range(n):
        b.box(((i - (n - 1) / 2) * 1.1, 0, 0.55 * g.get("height_scale", 1.0)), (1.06, 1.06, 1.1 * g.get("height_scale", 1.0)))
    b.write(st, "/Prop/Cells", rgb=_jit(g.get("rgb", [0.55, 0.48, 0.34]), rng), roughness=0.95)


def jersey_barrier(st, g, rng):
    b = MeshBuf()
    b.box((0, 0, 0.2), (3.0, 0.6, 0.4))
    b.box((0, 0, 0.6), (3.0, 0.3, 0.6))
    b.write(st, "/Prop/Barrier", mdl=g.get("material", CONCRETE))


def sandbags(st, g, rng):
    b = MeshBuf()
    L, rows = g.get("length_m", 4.0), int(g.get("rows", 4))
    for r in range(rows):
        off = 0.3 if r % 2 else 0.0
        for x in np.arange(-L / 2 + off, L / 2, 0.6):
            b.box((x, rng.uniform(-0.03, 0.03), 0.1 + r * 0.16), (0.58, 0.35, 0.17), yaw=rng.uniform(-0.08, 0.08))
    b.write(st, "/Prop/Bags", rgb=_jit(g.get("rgb", [0.50, 0.46, 0.32]), rng), roughness=0.95)


def tent(st, g, rng):
    b = MeshBuf()
    b.prism((0, 0, 0), rng.uniform(6, 10), rng.uniform(4.5, 6), rng.uniform(2.6, 3.4))
    b.write(st, "/Prop/Canvas", rgb=_jit(g.get("rgb", [0.33, 0.35, 0.24]), rng), roughness=0.9)


def tower(st, g, rng):
    b, roof = MeshBuf(), MeshBuf()
    h = rng.uniform(5, 8)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.tube([[sx * 1.3, sy * 1.3, 0], [sx * 1.0, sy * 1.0, h]], 0.12, 0.1)
    b.box((0, 0, h), (2.8, 2.8, 0.2))
    for k in range(4):
        a = k * np.pi / 2
        b.box((1.35 * np.cos(a), 1.35 * np.sin(a), h + 0.6), (0.1 if k % 2 == 0 else 2.8, 2.8 if k % 2 == 0 else 0.1, 1.0))
    roof.box((0, 0, h + 2.3), (3.2, 3.2, 0.15))
    b.write(st, "/Prop/Frame", rgb=(0.36, 0.30, 0.22))
    roof.write(st, "/Prop/Roof", rgb=(0.30, 0.32, 0.24))


def dead_tree(st, g, rng):
    b = MeshBuf()
    h = rng.uniform(*g.get("height_m", [6, 12]))

    def branch(p0, d, length, r, depth):
        p1 = p0 + d * length
        mid = (p0 + p1) / 2 + rng.normal(0, length * 0.05, 3)
        b.tube([p0, mid, p1], r, r * 0.7, 5)
        if depth == 0 or r < 0.02:
            return
        for _ in range(int(rng.integers(2, 4))):
            nd = d + rng.normal(0, 0.6, 3)
            nd[2] = abs(nd[2]) + 0.2
            nd /= np.linalg.norm(nd)
            branch(p1, nd, length * rng.uniform(0.5, 0.7), r * 0.6, depth - 1)

    branch(np.zeros(3), np.array([0.0, 0.0, 1.0]), h * 0.45, h * 0.025, 3)
    b.write(st, "/Prop/Wood", rgb=(0.10, 0.09, 0.08) if rng.random() < 0.6 else (0.30, 0.27, 0.23))


GENERATORS = {f.__name__: f for f in (ruin, rubble, vehicle_proxy, container, hesco, jersey_barrier,
                                      sandbags, tent, tower, dead_tree)}


def write_assets(kind: str, out_dir: Path, g: dict, rng, prefix: str | None = None) -> list[str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for k in range(int(g.get("variants", 4))):
        f = out_dir / f"{prefix or kind}_{k}.usda"
        st = _stage(f)
        GENERATORS[kind](st, g, rng)
        st.GetRootLayer().Save()
        paths.append(str(f))
    return paths
