"""Stage 04: build the Blender master scene and export the USD site for Isaac Sim.

Imports the localized OBJ, cleans it, builds Visual / Collision / Fixtures groups, saves the
.blend master and exports build/usd/site.usd plus plain OBJ references for validation.

Run: $BLENDER --background --factory-startup --python pipeline/04_blender_prep.py
"""

from __future__ import annotations

import json
import math
import sys
from collections import deque
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, load_config  # noqa: E402

REF = BUILD / "reference"


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0


def bbox_of(objs) -> tuple[list[float], list[float]]:
    """World bbox of vertices used by faces (stray unreferenced vertices are ignored)."""
    pts = []
    for o in objs:
        verts = o.data.vertices
        used = {i for poly in o.data.polygons for i in poly.vertices}
        pts.extend(o.matrix_world @ verts[i].co for i in used)
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    return lo, hi


def import_site(obj_path: Path) -> bpy.types.Object:
    bpy.ops.wm.obj_import(filepath=str(obj_path), forward_axis="Y", up_axis="Z", validate_meshes=True)
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    if not meshes:
        raise SystemExit(f"no meshes imported from {obj_path}")
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    site = bpy.context.view_layer.objects.active
    site.name = site.data.name = "site_visual"
    return site


def check_import_bbox(site: bpy.types.Object, tol: float) -> None:
    expected = json.loads((REF / "bbox_local.json").read_text())
    lo, hi = bbox_of([site])
    err = max(abs(a - b) for a, b in zip(lo + hi, expected["min"] + expected["max"]))
    print(f"import bbox error vs stage 03: {err * 1000:.2f} mm")
    if err > tol:
        raise SystemExit(f"OBJ import changed the geometry frame (bbox error {err:.4f} m > {tol} m)")


def clean_mesh(obj: bpy.types.Object, min_part_faces: int, merge_distance: float) -> dict:
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    faces_before = len(bm.faces)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=merge_distance)

    # Drop small loose parts (floating reconstruction fragments).
    bm.faces.ensure_lookup_table()
    seen: set[int] = set()
    doomed = []
    for f in bm.faces:
        if f.index in seen:
            continue
        part, queue = [], deque([f])
        seen.add(f.index)
        while queue:
            cur = queue.popleft()
            part.append(cur)
            for e in cur.edges:
                for nb in e.link_faces:
                    if nb.index not in seen:
                        seen.add(nb.index)
                        queue.append(nb)
        if len(part) < min_part_faces:
            doomed.extend(part)
    bmesh.ops.delete(bm, geom=doomed, context="FACES")
    loose = [v for v in bm.verts if not v.link_faces]
    bmesh.ops.delete(bm, geom=loose, context="VERTS")

    # Consistent winding, then make the surface face up overall (terrain is an open surface).
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    up = sum(f.normal.z * f.calc_area() for f in bm.faces)
    flipped = up < 0
    if flipped:
        bmesh.ops.reverse_faces(bm, faces=bm.faces)

    bm.to_mesh(obj.data)
    faces_after = len(bm.faces)
    bm.free()
    obj.data.update()
    return {"faces_before": faces_before, "faces_after": faces_after,
            "dropped_fragment_faces": len(doomed), "flipped_all": flipped}


def make_collision(visual: bpy.types.Object, ratio: float) -> bpy.types.Object:
    col = visual.copy()
    col.data = visual.data.copy()
    col.name = col.data.name = "site_collision"
    bpy.context.scene.collection.objects.link(col)
    col.data.materials.clear()
    mod = col.modifiers.new("decimate", "DECIMATE")
    mod.decimate_type = "COLLAPSE"
    mod.ratio = ratio
    mod.use_collapse_triangulate = True
    bpy.context.view_layer.objects.active = col
    bpy.ops.object.select_all(action="DESELECT")
    col.select_set(True)
    bpy.ops.object.modifier_apply(modifier=mod.name)
    return col


def import_underlay(path: Path, offset: float) -> bpy.types.Object:
    """DTM grid as a collision-only ground, sunk `offset` below the DTM so it never shadows the mesh."""
    before = set(bpy.context.scene.objects)
    bpy.ops.wm.obj_import(filepath=str(path), forward_axis="Y", up_axis="Z")
    (obj,) = [o for o in bpy.context.scene.objects if o not in before]
    obj.name = obj.data.name = "ground_underlay"
    obj.location.z -= offset
    return obj


def box(name: str, size: Vector, centre: Vector, mat: bpy.types.Material) -> bpy.types.Object:
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=size, verts=bm.verts)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    obj.location = centre
    me.materials.append(mat)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def make_fixture(fcfg: dict, visual: bpy.types.Object) -> tuple[list[bpy.types.Object], dict]:
    """Wall with a vertical gap. Two boxes, base sunk below the terrain so no gap forms underneath."""
    bvh = BVHTree.FromObject(visual, bpy.context.evaluated_depsgraph_get())
    x, y = fcfg["xy"]
    hit, *_ = bvh.ray_cast(Vector((x, y, 1000.0)), Vector((0, 0, -1)))
    if hit is None:
        raise SystemExit(f"fixture position {fcfg['xy']} is outside the reconstructed surface")
    ground = hit.z

    length, height, thick, gap = (fcfg[k] for k in ("wall_length", "wall_height", "wall_thickness", "gap_width"))
    sink = 0.5
    seg = (length - gap) / 2
    zc = ground + (height - sink) / 2
    yaw = math.radians(fcfg["yaw_deg"])
    rot = Matrix.Rotation(yaw, 4, "Z")

    mat = bpy.data.materials.new("fixture_grey")
    mat.diffuse_color = (0.6, 0.6, 0.6, 1.0)
    parts = []
    for side, sign in (("left", -1), ("right", 1)):
        local_c = Vector((sign * (gap / 2 + seg / 2), 0.0, 0.0))
        c = Vector((x, y, zc)) + (rot @ local_c.to_4d()).to_3d()
        b = box(f"fixture_wall_{side}", Vector((seg, thick, height + sink)), c, mat)
        b.rotation_euler = (0, 0, yaw)
        parts.append(b)

    along = (rot @ Vector((1, 0, 0, 0))).to_3d()
    normal = (rot @ Vector((0, 1, 0, 0))).to_3d()
    probe_z = ground + height / 2
    info = {
        "ground_z": ground,
        "gap_centre": [x, y, probe_z],
        "wall_normal": list(normal),
        "wall_along": list(along),
        "blocked_point": list(Vector((x, y, probe_z)) + along * (gap / 2 + seg / 2)),
        "gap_width": gap,
        "wall_thickness": thick,
        "wall_height": height,
    }
    return parts, info


def group(name: str, objs) -> bpy.types.Object:
    """Parent objects under an empty so the USD export has /Site/<name>/<object>."""
    empty = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(empty)
    coll = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(coll)
    for o in [empty, *objs]:
        for c in o.users_collection:
            c.objects.unlink(o)
        coll.objects.link(o)
    for o in objs:
        o.parent = empty
    return empty


def export_obj(objs, path: Path) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.ops.wm.obj_export(filepath=str(path), export_selected_objects=True, export_materials=False,
                          export_uv=False, forward_axis="Y", up_axis="Z", apply_transform=True)


def main() -> None:
    cfg = load_config()
    bcfg = cfg["blender"]
    reset_scene()

    site = import_site(BUILD / "localized" / "site.obj")
    check_import_bbox(site, cfg["validate"]["import_bbox_m"])
    stats = clean_mesh(site, bcfg["min_part_faces"], bcfg["merge_distance"])
    print(f"cleanup: {stats}")

    collision = make_collision(site, bcfg["collision_decimate_ratio"])
    stats["collision_faces"] = len(collision.data.polygons)
    fixtures, fixture_info = make_fixture(bcfg["fixture"], site)
    underlay = import_underlay(BUILD / "localized" / "dtm.obj", bcfg["ground_underlay_offset"])
    stats["underlay_faces"] = len(underlay.data.polygons)

    group("Visual", [site])
    group("Collision", [collision, underlay])
    group("Fixtures", fixtures)

    REF.mkdir(parents=True, exist_ok=True)
    lo, hi = bbox_of([site])
    (REF / "bbox_visual.json").write_text(json.dumps({"min": lo, "max": hi}, indent=2) + "\n")
    (REF / "fixture.json").write_text(json.dumps(fixture_info, indent=2) + "\n")
    (REF / "blender_stats.json").write_text(json.dumps(stats, indent=2) + "\n")
    export_obj([site, *fixtures], REF / "visual.obj")
    export_obj([collision, underlay, *fixtures], REF / "collision.obj")

    bpy.ops.wm.save_as_mainfile(filepath=str(BUILD / "site_master.blend"))

    usd_dir = BUILD / "usd"
    usd_dir.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.usd_export(
        filepath=str(usd_dir / "site.usd"),
        root_prim_path="/Site",
        export_materials=True,
        generate_preview_surface=True,
        export_textures_mode="NEW",
        overwrite_textures=True,
        relative_paths=True,
        export_uvmaps=True,
        export_normals=True,
        export_lights=False,
        export_cameras=False,
        convert_orientation=False,
        convert_scene_units="METERS",
        triangulate_meshes=True,
    )
    print(f"exported {usd_dir / 'site.usd'}")


if __name__ == "__main__":
    main()
