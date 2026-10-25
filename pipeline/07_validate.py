"""Stage 07: check the Isaac environment against the reference geometry and write a report.

Reference geometry is what Blender exported (build/reference/visual.obj, collision.obj), so
these checks cover the USD export, Isaac import, rendering and physics setup, plus the
coordinate frame and reconstruction scale against camera GPS.

Run: uv run python pipeline/07_validate.py --run-dir reports/<run>
Exits 1 if any gated check fails.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
from pyproj import Geod

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402
from pipeline.common.frame import SiteFrame  # noqa: E402

REF = BUILD / "reference"
RNG = np.random.default_rng(0)


class Report:
    def __init__(self) -> None:
        self.rows: list[dict] = []

    def add(self, name: str, passed: bool | None, value: str, threshold: str, detail: dict | None = None) -> None:
        self.rows.append({"check": name, "passed": passed, "value": value, "threshold": threshold,
                          "detail": detail or {}})
        mark = {True: "PASS", False: "FAIL", None: "info"}[passed]
        print(f"  {mark:4}  {name}: {value} (threshold {threshold})")

    @property
    def ok(self) -> bool:
        return all(r["passed"] is not False for r in self.rows)


def load_mesh(path: Path) -> trimesh.Trimesh:
    return trimesh.load(path, force="mesh", process=False)


def surface_distance(mesh: trimesh.Trimesh, pts: np.ndarray) -> np.ndarray:
    _, dist, _ = trimesh.proximity.closest_point(mesh, pts)
    return dist


def ray_down(mesh: trimesh.Trimesh, xy: np.ndarray) -> np.ndarray:
    """Height of the highest surface under each XY, NaN where nothing is hit."""
    origins = np.column_stack([xy, np.full(len(xy), mesh.bounds[1][2] + 10.0)])
    dirs = np.tile([0.0, 0.0, -1.0], (len(xy), 1))
    locs, idx, _ = mesh.ray.intersects_location(origins, dirs, multiple_hits=False)
    z = np.full(len(xy), np.nan)
    z[idx] = locs[:, 2]
    return z


def ray_down_lowest(mesh: trimesh.Trimesh, xy: np.ndarray) -> np.ndarray:
    """Height of the lowest surface under each XY (ground under canopy), NaN where nothing is hit."""
    origins = np.column_stack([xy, np.full(len(xy), mesh.bounds[1][2] + 10.0)])
    dirs = np.tile([0.0, 0.0, -1.0], (len(xy), 1))
    locs, idx, _ = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)
    z = np.full(len(xy), np.inf)
    np.minimum.at(z, idx, locs[:, 2])
    z[np.isinf(z)] = np.nan
    return z


def stats(d: np.ndarray) -> dict:
    return {"n": int(len(d)), "median": float(np.median(d)), "p95": float(np.percentile(d, 95)),
            "max": float(d.max())}


# ---------------------------------------------------------------- checks

def check_frame(rep: Report, v: dict) -> None:
    frame = SiteFrame.load(BUILD / "site_frame.json")
    cams = json.loads((REF / "cameras_local.json").read_text())
    ll = np.array([c["exif_lonlat"] for c in cams.values() if "exif_lonlat" in c])
    local = frame.lonlat_to_local(ll[:, 0], ll[:, 1], ll[:, 2])
    back = frame.lonlat_to_local(*frame.local_to_lonlat(local).T)
    err = float(np.abs(back - local).max())
    rep.add("Frame round-trip", err < v["frame_roundtrip_m"], f"{err * 1000:.4f} mm",
            f"< {v['frame_roundtrip_m'] * 1000:g} mm")

    pairs = [(c["reconstructed"], c["exif"], c["exif_lonlat"]) for c in cams.values()
             if "reconstructed" in c and "exif" in c]
    if not pairs:
        rep.add("Georeference sanity", False, "no reconstructed cameras", "-")
        return
    rec = np.array([p[0] for p in pairs])
    exif = np.array([p[1] for p in pairs])
    ll = np.array([p[2] for p in pairs])
    horiz = np.linalg.norm(rec[:, :2] - exif[:, :2], axis=1)
    med = float(np.median(horiz))
    rep.add("Georeference sanity", med < v["georef_median_horizontal_m"], f"median {med:.2f} m horizontal",
            f"< {v['georef_median_horizontal_m']} m", {"cameras": len(pairs), **stats(horiz)})

    geod = Geod(ellps="WGS84")
    ratios = []
    for i, j in itertools.combinations(range(len(pairs)), 2):
        _, _, dh = geod.inv(ll[i, 0], ll[i, 1], ll[j, 0], ll[j, 1])
        gps = float(np.hypot(dh, ll[i, 2] - ll[j, 2]))
        if gps > 50.0:
            ratios.append(np.linalg.norm(rec[i] - rec[j]) / gps)
    scale_err = abs(float(np.median(ratios)) - 1.0)
    rep.add("Scale vs GPS", scale_err < v["scale_rel_error"], f"{scale_err * 100:.3f} %",
            f"< {v['scale_rel_error'] * 100:g} %", {"pairs": len(ratios)})


def check_import(rep: Report, v: dict) -> None:
    blender = json.loads((REF / "bbox_visual.json").read_text())
    isaac = json.loads((REF / "isaac_scene.json").read_text())
    err = max(abs(a - b) for a, b in zip(blender["min"] + blender["max"],
                                          isaac["visual_bbox"]["min"] + isaac["visual_bbox"]["max"]))
    rep.add("Import axes/scale (Isaac bbox vs Blender)", err < v["import_bbox_m"], f"{err * 1000:.2f} mm",
            f"< {v['import_bbox_m'] * 1000:g} mm")
    n = len(isaac["unresolved_textures"])
    rep.add("Texture paths resolve", n == 0, f"{n} unresolved", "0")


def unproject(depth: np.ndarray, intr: dict, R: np.ndarray, t: np.ndarray, n: int, far: float) -> np.ndarray:
    v, u = np.nonzero(np.isfinite(depth) & (depth > 0) & (depth < 0.99 * far))
    if len(u) == 0:
        return np.zeros((0, 3))
    pick = RNG.choice(len(u), size=min(n, len(u)), replace=False)
    u, v = u[pick], v[pick]
    d = depth[v, u].astype(np.float64)
    x = (u + 0.5 - intr["cx"]) / intr["fx"] * d
    y = -(v + 0.5 - intr["cy"]) / intr["fy"] * d
    cam = np.column_stack([x, y, -d])
    return cam @ R.T + t


def check_sensors(rep: Report, v: dict, cap: dict, run_dir: Path, visual: trimesh.Trimesh, far: float) -> None:
    depth_d, lidar_d, per_pose = [], [], []
    for p in cap["poses"]:
        row = {"id": p["id"]}
        f = run_dir / "frames" / f"{p['id']}_depth.npy"
        if f.exists():
            pts = unproject(np.load(f), cap["intrinsics"], np.array(p["cam_R"]), np.array(p["position"]),
                            v["depth_sample_pixels"], far)
            if len(pts):
                d = surface_distance(visual, pts)
                depth_d.append(d)
                row["depth_median"] = float(np.median(d))
        f = run_dir / "frames" / f"{p['id']}_lidar.npy"
        if f.exists():
            pts = np.load(f).astype(np.float64)
            if len(pts):
                pts = pts[RNG.choice(len(pts), size=min(3000, len(pts)), replace=False)]
                d = surface_distance(visual, pts)
                lidar_d.append(d)
                row["lidar_median"] = float(np.median(d))
        per_pose.append(row)

    if depth_d:
        d = np.concatenate(depth_d)
        s = stats(d)
        rep.add("Depth vs visual mesh", s["median"] < v["depth_median_m"],
                f"median {s['median'] * 100:.2f} cm, p95 {s['p95'] * 100:.2f} cm ({len(depth_d)} poses)",
                f"median < {v['depth_median_m'] * 100:g} cm", {**s, "per_pose": per_pose})
    else:
        rep.add("Depth vs visual mesh", False, "no depth frames", "-")

    if lidar_d:
        d = np.concatenate(lidar_d)
        s = stats(d)
        rep.add("LiDAR vs visual mesh", s["median"] < v["lidar_median_m"] and s["p95"] < v["lidar_p95_m"],
                f"median {s['median'] * 100:.2f} cm, p95 {s['p95'] * 100:.2f} cm ({len(lidar_d)} poses)",
                f"median < {v['lidar_median_m'] * 100:g} cm, p95 < {v['lidar_p95_m'] * 100:g} cm", s)
    else:
        rep.add("LiDAR vs visual mesh", False, "no LiDAR returns", "-",
                {"lidar_frame_info": cap.get("lidar_frame_info")})


def check_physics(rep: Report, v: dict, cap: dict, visual: trimesh.Trimesh, collision: trimesh.Trimesh) -> None:
    spheres = [s for s in cap["spheres"] if s["rest_position"] is not None]
    if spheres:
        rest = np.array([s["rest_position"] for s in spheres])
        r = np.array([s["radius"] for s in spheres])
        speed = np.array([s.get("final_speed") if s.get("final_speed") is not None else np.inf for s in spheres])
        lowest = ray_down_lowest(collision, rest[:, :2])
        on_map = np.isfinite(lowest)
        # Below the lowest surface under it = tunnelled through the collider. Fast = still falling
        # or bouncing (for example off a tree crown); slow sliding on a slope still counts as contact.
        tunnelled = on_map & (rest[:, 2] < lowest - r)
        airborne = on_map & ~tunnelled & (speed > v["contact_max_speed"])
        contact = on_map & ~tunnelled & ~airborne
        # On a slope the contact point is not straight below the centre, so compare against the
        # closest collision-surface point.
        _, dist, _ = trimesh.proximity.closest_point(collision, rest[contact])
        err = np.abs(dist - r[contact])
        s = stats(err) if len(err) else {"n": 0, "max": float("inf")}
        detail = {**s, "tunnelled": int(tunnelled.sum()), "airborne": int(airborne.sum()),
                  "off_map": int((~on_map).sum()), "total": len(spheres)}
        rep.add("Contact (sphere rest vs collision surface)",
                s["n"] > 0 and s["max"] < v["contact_m"] and detail["tunnelled"] == 0,
                f"max {s['max'] * 100:.2f} cm over {s['n']} spheres in contact; {detail['tunnelled']} tunnelled, "
                f"{detail['airborne']} airborne, {detail['off_map']} off map",
                f"< {v['contact_m'] * 100:g} cm, 0 tunnelled", detail)
    else:
        rep.add("Contact (sphere rest vs collision surface)", False, "no sphere results", "-")

    lo, hi = visual.bounds[0], visual.bounds[1]
    gx, gy = np.meshgrid(np.linspace(lo[0], hi[0], 60)[5:-5], np.linspace(lo[1], hi[1], 60)[5:-5])
    xy = np.column_stack([gx.ravel(), gy.ravel()])
    dz = np.abs(ray_down(visual, xy) - ray_down(collision, xy))
    dz = dz[np.isfinite(dz)]
    s = stats(dz)
    rep.add("Visual vs collision height drift", s["p95"] < v["collision_drift_p95_m"],
            f"p95 {s['p95'] * 100:.2f} cm, max {s['max'] * 100:.1f} cm",
            f"p95 < {v['collision_drift_p95_m'] * 100:g} cm", s)

    fx = json.loads((REF / "fixture.json").read_text())
    g = cap["gap_rays"]
    wall_at = 3.0 - fx["wall_thickness"] / 2
    through_ok = (not g["through_gap"]["hit"]) or g["through_gap"]["distance"] > wall_at + fx["wall_thickness"] + 0.05
    blocked_ok = g["blocked"]["hit"] and abs(g["blocked"]["distance"] - wall_at) < 0.05
    rep.add("Opening preserved (PhysX ray through gap)", through_ok and blocked_ok,
            f"gap hit={g['through_gap']['hit']}, wall hit={g['blocked']['hit']} "
            f"at {g['blocked']['distance'] if g['blocked']['distance'] is not None else '-'} m",
            f"gap clear, wall hit at {wall_at:.2f} m", g)


def write_report(rep: Report, cap: dict, run_dir: Path) -> None:
    rep.add("Throughput (camera + LiDAR, headless)", None,
            f"{cap['throughput_fps']:.1f} FPS, GPU mem {cap.get('gpu_mem_mib')} MiB", "recorded only")
    (run_dir / "results.json").write_text(json.dumps({"passed": rep.ok, "checks": rep.rows}, indent=2) + "\n")
    blender_stats = json.loads((REF / "blender_stats.json").read_text())
    lines = [
        f"# Validation report: {run_dir.name}",
        "",
        f"**Overall: {'PASS' if rep.ok else 'FAIL'}**",
        "",
        "| Check | Result | Value | Threshold |",
        "|---|---|---|---|",
    ]
    for r in rep.rows:
        mark = {True: "PASS", False: "**FAIL**", None: "info"}[r["passed"]]
        lines.append(f"| {r['check']} | {mark} | {r['value']} | {r['threshold']} |")
    lines += [
        "",
        "## Build",
        "",
        f"- Poses captured: {len(cap['poses'])}",
        f"- Mesh faces: {blender_stats['faces_before']} raw, {blender_stats['faces_after']} cleaned, "
        f"{blender_stats['collision_faces']} collision + {blender_stats.get('underlay_faces', 0)} DTM underlay",
        f"- LiDAR frame/coords types seen: {cap.get('lidar_frame_info')}",
        "",
        "Frames are in `frames/`; per-check detail is in `results.json`.",
    ]
    (run_dir / "report.md").write_text("\n".join(lines) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    args = ap.parse_args()
    run_dir = (PROJECT_ROOT / args.run_dir).resolve()
    cfg = load_config()
    v = cfg["validate"]
    cap = json.loads((run_dir / "capture.json").read_text())
    visual = load_mesh(REF / "visual.obj")
    collision = load_mesh(REF / "collision.obj")

    rep = Report()
    check_frame(rep, v)
    check_import(rep, v)
    check_sensors(rep, v, cap, run_dir, visual, cfg["capture"]["camera"]["clip"][1])
    check_physics(rep, v, cap, visual, collision)
    write_report(rep, cap, run_dir)
    print(f"report: {run_dir / 'report.md'}  overall {'PASS' if rep.ok else 'FAIL'}")
    sys.exit(0 if rep.ok else 1)


if __name__ == "__main__":
    main()
