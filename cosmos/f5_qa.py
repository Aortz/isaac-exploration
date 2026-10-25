"""F5: check every generated variant against the simulator ground truth and accept or reject it.

Checks per variant (thresholds in config qa:, fixed before a batch):
  format      frame count / resolution match the clip
  edges       Canny F-score of variant vs the sim render (geometry stayed put)
  warp        photometric error when warping frame t -> t+1 with GT depth + poses, as a ratio to the
              same error on the sim render over the same pixels (texture must move with the scene)
  tracks      KLT tracks triangulated with GT poses: median reprojection error above the sim
              render's, and mean track length relative to the sim render (VIO front-end usability)
Synthetic dry-run outputs are checked like any other but are never marked usable.

Run: uv run python cosmos/f5_qa.py --flight marina_bay_pass
Writes build/cosmos/<flight>/qa/{qa.json, report.md, contact_<clip>.jpg}
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cosmos import qa_metrics as qm  # noqa: E402
from cosmos.video_io import probe, read_video  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402


def clip_ground_truth(flight: str, clip: dict, n: int):
    frames_dir = BUILD / "flight" / flight / "render" / "frames"
    s = clip["frames"][0]
    depths = [np.load(frames_dir / f"{i:06d}_depth.npy") for i in range(s, s + n)]
    poses = np.asarray(clip["poses_tum"])[:n]
    I = clip["intrinsics"]
    K = np.array([[I["fx"], 0, I["cx"]], [0, I["fy"], I["cy"]], [0, 0, 1.0]])
    return depths, poses, K


def score(frames, ref_frames, depths, poses, K, ref_cache: dict, canny, tol) -> dict:
    n = len(frames)
    if "warp" not in ref_cache:
        e_ref, masks = qm.warp_error_sequence(ref_frames, depths, poses, K)
        er, ln = qm.track_reprojection(qm.klt_tracks(ref_frames), poses, K)
        ref_cache.update(warp=e_ref, masks=masks, track_err=er, track_len=ln)
    e_var, _ = qm.warp_error_sequence(frames, depths, poses, K, common_masks=ref_cache["masks"])
    er, ln = qm.track_reprojection(qm.klt_tracks(frames), poses, K)
    edge = [qm.edge_f_score(frames[i], ref_frames[i], canny, tol) for i in range(0, n, max(1, n // 12))]
    ref_len = float(ref_cache["track_len"].mean()) if len(ref_cache["track_len"]) else float("nan")
    return {
        "edge_f_score": float(np.nanmean(edge)),
        "warp_error": float(np.nanmean(e_var)),
        "warp_error_ref": float(np.nanmean(ref_cache["warp"])),
        "warp_error_ratio": float(np.nanmean(e_var) / np.nanmean(ref_cache["warp"])),
        "track_count": int(len(er)),
        "track_reproj_px": float(np.median(er)) if len(er) else float("nan"),
        "track_reproj_px_ref": float(np.median(ref_cache["track_err"])) if len(ref_cache["track_err"]) else float("nan"),
        "track_length_ratio": float(ln.mean() / ref_len) if len(ln) else 0.0,
    }


def contact_sheet(rows: list[tuple[str, list[np.ndarray]]], path: Path) -> None:
    th_w, th_h = 320, 185
    cols = len(rows[0][1])
    sheet = Image.new("RGB", (150 + cols * th_w, len(rows) * th_h), "white")
    draw = ImageDraw.Draw(sheet)
    for r, (label, frames) in enumerate(rows):
        draw.text((6, r * th_h + th_h // 2), label, fill="black")
        for c, f in enumerate(frames):
            sheet.paste(Image.fromarray(f).resize((th_w, th_h - 5)), (150 + c * th_w, r * th_h))
    sheet.save(path, quality=88)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    args = ap.parse_args()
    cfg = load_config(args.config)
    q, pk = cfg["qa"], cfg["package"]
    root = BUILD / "cosmos" / args.flight
    manifest = json.loads((root / "manifest.json").read_text())
    out = root / "qa"
    out.mkdir(exist_ok=True)

    by_clip: dict[str, list[tuple[str, dict]]] = {}
    for name, v in manifest["variants"].items():
        if v.get("status") == "ok":
            by_clip.setdefault(v["clip"], []).append((name, v))

    results = {}
    for clip_name, variants in sorted(by_clip.items()):
        clip = json.loads((root / "clips" / clip_name / "clip.json").read_text())
        size = (clip["width"], clip["height"])
        ref_all = read_video(root / "clips" / clip_name / "rgb.mp4")
        ref_cache: dict[int, dict] = {}
        sheet_rows = []
        for name, v in sorted(variants):
            path = root / v["path"]
            info = probe(path)
            frames = read_video(path, size=size)
            n = min(len(frames), v["frames"])
            fmt_ok = info["frames"] == v["frames"] and (info["width"], info["height"]) == size
            depths, poses, K = clip_ground_truth(args.flight, clip, n)
            m = score(frames[:n], ref_all[:n], depths, poses, K, ref_cache.setdefault(n, {}), pk["canny"],
                      q["edge_tolerance_px"])
            checks = {
                "format": fmt_ok,
                "edges": m["edge_f_score"] >= q["edge_f_score_min"],
                "warp": m["warp_error_ratio"] <= q["warp_error_ratio_max"],
                "track_reproj": m["track_reproj_px"] - m["track_reproj_px_ref"] <= q["track_reproj_excess_px_max"],
                "track_length": m["track_length_ratio"] >= q["track_length_ratio_min"],
            }
            passed = all(checks.values())
            results[name] = {**m, "probe": info, "checks": checks, "passed": passed,
                             "usable": passed and not v.get("synthetic", False),
                             "synthetic": bool(v.get("synthetic", False))}
            print(f"  {name}: {'PASS' if passed else 'FAIL'} edgeF={m['edge_f_score']:.2f} "
                  f"warp×{m['warp_error_ratio']:.2f} reproj={m['track_reproj_px']:.2f}px "
                  f"len×{m['track_length_ratio']:.2f}" + ("  [synthetic]" if v.get("synthetic") else ""))
            if len(sheet_rows) == 0:
                idx = [0, n // 2, n - 1]
                sheet_rows.append(("sim render", [ref_all[i] for i in idx]))
            sheet_rows.append((name.split("/")[-1], [frames[i] for i in idx]))
        contact_sheet(sheet_rows, out / f"contact_{clip_name}.jpg")

    (out / "qa.json").write_text(json.dumps({"thresholds": q, "variants": results}, indent=2) + "\n")
    lines = [f"# Cosmos QA: {args.flight}", "",
             "| Variant | Result | Edge F | Warp ratio | Track reproj (px) | Track length ratio |",
             "|---|---|---|---|---|---|"]
    for name, r in sorted(results.items()):
        res = ("PASS" if r["passed"] else "**FAIL**") + (" (synthetic)" if r["synthetic"] else "")
        lines.append(f"| {name} | {res} | {r['edge_f_score']:.2f} | {r['warp_error_ratio']:.2f} | "
                     f"{r['track_reproj_px']:.2f} (sim {r['track_reproj_px_ref']:.2f}) | {r['track_length_ratio']:.2f} |")
    lines += ["", f"Thresholds: edge F ≥ {q['edge_f_score_min']}, warp ratio ≤ {q['warp_error_ratio_max']}, "
              f"track reproj ≤ sim + {q['track_reproj_excess_px_max']} px, "
              f"track length ratio ≥ {q['track_length_ratio_min']}.",
              "", "Contact sheets: `contact_<clip>.jpg` (first / middle / last frame)."]
    (out / "report.md").write_text("\n".join(lines) + "\n")
    usable = sum(r["usable"] for r in results.values())
    print(f"{len(results)} variants checked, {sum(r['passed'] for r in results.values())} passed, "
          f"{usable} usable -> {out / 'report.md'}")


if __name__ == "__main__":
    main()
