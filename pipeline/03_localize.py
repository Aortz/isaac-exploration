"""Stage 03: move the ODM reconstruction into a local metric site frame.

Reads the ODM outputs, chooses a local origin near the site, rewrites the textured OBJ into that
frame and records the transform in build/site_frame.json. Camera positions from the
reconstruction and from image EXIF are written in the same frame for validation.

Run: uv run python pipeline/03_localize.py
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, export_json, load_config, project_path  # noqa: E402
from pipeline.common.frame import SiteFrame, parse_odm_coords  # noqa: E402


def read_obj_vertices(path: Path) -> np.ndarray:
    """Vertex positions referenced by at least one face (ODM leaves stray unreferenced vertices)."""
    verts, used = [], set()
    with path.open() as f:
        for line in f:
            if line.startswith("v "):
                verts.append(line.split()[1:4])
            elif line.startswith("f "):
                used.update(int(tok.split("/")[0]) for tok in line.split()[1:])
    verts = np.asarray(verts, dtype=np.float64)
    idx = np.array(sorted(used))
    idx = np.where(idx > 0, idx - 1, len(verts) + idx)
    return verts[idx]


def write_shifted_obj(src: Path, dst: Path, shift: np.ndarray) -> int:
    """Copy an OBJ, subtracting `shift` from every vertex position. Other lines are untouched."""
    n = 0
    with src.open() as fin, dst.open("w") as fout:
        for line in fin:
            if line.startswith("v "):
                x, y, z = (float(v) for v in line.split()[1:4])
                fout.write(f"v {x - shift[0]:.4f} {y - shift[1]:.4f} {z - shift[2]:.4f}\n")
                n += 1
            else:
                fout.write(line)
    return n


def read_geotiff_grid(path: Path) -> tuple[np.ndarray, float, float, float, float, float]:
    """Return (array, west, north, pixel_x, pixel_y, nodata) for a north-up GeoTIFF (PixelIsArea)."""
    import tifffile

    with tifffile.TiffFile(path) as tif:
        page = tif.pages[0]
        sx, sy, _ = page.tags["ModelPixelScaleTag"].value
        _, _, _, west, north, _ = page.tags["ModelTiepointTag"].value
        nodata = float(page.tags["GDAL_NODATA"].value) if "GDAL_NODATA" in page.tags else np.nan
        return page.asarray().astype(np.float64), west, north, sx, sy, nodata


def write_dtm_obj(dtm_path: Path, dst: Path, frame: SiteFrame, cell_m: float) -> dict:
    """Resample the DTM to `cell_m` cells (mean of valid pixels) and write it as a grid mesh in the local frame.

    Used as a collision underlay that closes holes in the photogrammetry mesh.
    """
    z, west, north, sx, sy, nodata = read_geotiff_grid(dtm_path)
    valid = np.isfinite(z) & (z != nodata)
    k = max(1, int(round(cell_m / sx)))
    h, w = (z.shape[0] // k) * k, (z.shape[1] // k) * k
    zb = np.where(valid, z, 0.0)[:h, :w].reshape(h // k, k, w // k, k)
    vb = valid[:h, :w].reshape(h // k, k, w // k, k)
    count = vb.sum(axis=(1, 3))
    grid = np.where(count >= 0.5 * k * k, zb.sum(axis=(1, 3)) / np.maximum(count, 1), np.nan)

    rows, cols = grid.shape
    cell_x, cell_y = sx * k, sy * k
    jj, ii = np.meshgrid(np.arange(cols), np.arange(rows))
    east = west + (jj + 0.5) * cell_x
    northing = north - (ii + 0.5) * cell_y
    local = frame.utm_to_local(np.stack([east, northing, grid], axis=-1))
    index = -np.ones(grid.shape, dtype=np.int64)
    ok = np.isfinite(grid)
    index[ok] = np.arange(ok.sum())
    a, b, c, d = index[:-1, :-1], index[:-1, 1:], index[1:, :-1], index[1:, 1:]
    quad = (a >= 0) & (b >= 0) & (c >= 0) & (d >= 0)
    # Rows run north to south, so (a, c, b) and (b, c, d) wind counter-clockwise seen from above.
    tris = np.concatenate([np.stack([a[quad], c[quad], b[quad]], 1), np.stack([b[quad], c[quad], d[quad]], 1)])
    with dst.open("w") as f:
        f.write(f"# DTM {dtm_path.name} resampled to {cell_x:.2f} m, local frame\no dtm_ground\n")
        np.savetxt(f, local[ok], fmt="v %.4f %.4f %.4f")
        np.savetxt(f, tris + 1, fmt="f %d %d %d")
    return {"cell_m": cell_x, "vertices": int(ok.sum()), "faces": int(len(tris))}


def _dms(v) -> float:
    d, m, s = (float(x) for x in v)
    return d + m / 60.0 + s / 3600.0


def read_exif_gps(images: Path) -> list[dict]:
    out = []
    for p in sorted(images.iterdir()):
        if p.suffix.lower() not in (".jpg", ".jpeg", ".tif", ".tiff"):
            continue
        gps = Image.open(p).getexif().get_ifd(0x8825)
        if not gps or 2 not in gps or 4 not in gps:
            continue
        lat = _dms(gps[2]) * (-1 if gps.get(1) == "S" else 1)
        lon = _dms(gps[4]) * (-1 if gps.get(3) == "W" else 1)
        alt = float(gps.get(6, 0.0)) * (-1 if gps.get(5) == 1 else 1)
        out.append({"filename": p.name, "lon": lon, "lat": lat, "alt": alt})
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config)
    export_json(args.config)
    odm = project_path(cfg["odm"]["project_dir"]) / "code"
    obj_src = odm / "odm_texturing" / "odm_textured_model_geo.obj"

    epsg, offset = parse_odm_coords(odm / "odm_georeferencing" / "coords.txt")
    verts = read_obj_vertices(obj_src)
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    if np.abs(verts[:, :2]).max() > 1e5:
        raise SystemExit("OBJ X/Y look absolute, not ODM-offset; refusing to apply the offset logic")

    # Origin in UTM, rounded to whole metres so local coordinates stay readable.
    centre_odm = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2])
    ground_z = float(np.percentile(verts[:, 2], cfg["localize"]["ground_percentile"]))
    origin_utm = np.round([centre_odm[0] + offset[0], centre_odm[1] + offset[1], ground_z])

    frame = SiteFrame(
        epsg=epsg,
        odm_offset=offset.tolist(),
        origin_utm=origin_utm.tolist(),
        vertical_datum="as recorded by camera EXIF GPS altitude (not converted)",
        notes=[
            f"source: {obj_src.relative_to(project_path('.'))}",
            "local = odm - (origin_utm - [odm_offset_e, odm_offset_n, 0])",
        ],
    )
    frame.save(BUILD / "site_frame.json")
    shift = frame.odm_to_local_shift

    out_dir = BUILD / "localized"
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    n = write_shifted_obj(obj_src, out_dir / "site.obj", shift)
    for f in obj_src.parent.iterdir():
        if f.suffix.lower() in (".mtl", ".png", ".jpg", ".jpeg"):
            shutil.copy2(f, out_dir / f.name)

    dtm = write_dtm_obj(odm / "odm_dem" / "dtm.tif", out_dir / "dtm.obj", frame, cfg["localize"]["dtm_cell_m"])

    local = verts - shift
    ref = BUILD / "reference"
    ref.mkdir(parents=True, exist_ok=True)
    (ref / "bbox_local.json").write_text(json.dumps({
        "min": local.min(axis=0).tolist(),
        "max": local.max(axis=0).tolist(),
        "vertex_count": int(len(local)),
    }, indent=2) + "\n")

    # Camera centres: reconstruction (ODM frame) and EXIF GPS (geographic), both into local.
    cameras = {}
    shots = odm / "odm_report" / "shots.geojson"
    if shots.exists():
        for feat in json.loads(shots.read_text())["features"]:
            props = feat["properties"]
            t = np.asarray(props["translation"], dtype=np.float64)
            # ODM 3.6 writes shot centres in absolute UTM, unlike the offset mesh.
            centre = frame.utm_to_local(t) if np.abs(t[:2]).max() > 1e5 else frame.odm_to_local(t)
            cameras.setdefault(props["filename"], {})["reconstructed"] = centre.tolist()
    for e in read_exif_gps(project_path(cfg["dataset"]["images"])):
        cameras.setdefault(e["filename"], {})["exif"] = frame.lonlat_to_local(e["lon"], e["lat"], e["alt"]).tolist()
        cameras[e["filename"]]["exif_lonlat"] = [e["lon"], e["lat"], e["alt"]]
    (ref / "cameras_local.json").write_text(json.dumps(cameras, indent=1) + "\n")

    print(f"EPSG:{epsg}  ODM offset {offset.tolist()}  origin_utm {origin_utm.tolist()}")
    print(f"{n} vertices -> {out_dir / 'site.obj'}")
    print(f"local bbox min {np.round(local.min(axis=0), 2).tolist()} max {np.round(local.max(axis=0), 2).tolist()}")
    print(f"DTM underlay: {dtm}")
    print(f"{len(cameras)} cameras ({sum('reconstructed' in c for c in cameras.values())} reconstructed)")


if __name__ == "__main__":
    main()
