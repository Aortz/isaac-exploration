"""F3: package the F2 render into Cosmos-ready clips (control videos + structured prompts).

Per clip (≤ package.max_frames frames at camera.fps):
  rgb.mp4     the simulator render (also the input_reference for derived controls)
  depth.mp4   inverse depth in [depth_near_m, depth_far_m] -> 8 bit; sky = far
              (near_bright: near = white; near_dark: inverted)
  edge.mp4    Canny edges of the render
  clip.json   frame range, fps, intrinsics, the TUM poses of the clip
  prompts/<condition>.json   structured "upsampled" prompt (Cosmos 3 transfer format)

Run: uv run python cosmos/f3_package.py --flight marina_bay_pass
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cosmos.video_io import write_video as encode  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402


def depth_to_rgb(depth: np.ndarray, near: float, far: float, convention: str) -> np.ndarray:
    d = np.where(np.isfinite(depth) & (depth > 0), depth, far)
    inv = (1.0 / np.clip(d, near, far) - 1.0 / far) / (1.0 / near - 1.0 / far)   # 1 = near, 0 = far/sky
    if convention == "near_dark":
        inv = 1.0 - inv
    g = np.round(inv * 255).astype(np.uint8)
    return np.repeat(g[..., None], 3, axis=2)


def edges_to_rgb(rgb: np.ndarray, canny) -> np.ndarray:
    e = cv2.Canny(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), *canny)
    return np.repeat(e[..., None], 3, axis=2)


def structured_prompt(cfg: dict, scene: str, condition: str, w: int, h: int, mount_pitch: float) -> dict:
    """Cosmos 3 transfer takes an 'upsampled' JSON caption; mirror the keys of NVIDIA's samples."""
    c = cfg["cosmos"]
    desc = c["conditions"][condition]
    return {
        "subjects": [],
        "background_setting": f"{scene} {desc}.",
        "lighting": {"conditions": desc, "direction": "Natural outdoor light consistent with the conditions",
                     "shadows": "Physically consistent with the light source and weather",
                     "illumination_effect": "Realistic atmospheric perspective over distance"},
        "aesthetics": {"composition": "Forward-looking aerial view, horizon in the upper part of the frame",
                       "color_scheme": "Natural, true-to-life colours for the stated conditions",
                       "mood_atmosphere": desc},
        "cinematography": {"framing": "Wide aerial shot", "camera_angle": f"Looking forward and about "
                           f"{mount_pitch:.0f} degrees down from a drone at 80-120 m altitude",
                           "depth_of_field": "Deep, everything in focus", "focus": "Sharp across the frame",
                           "lens_focal_length": "Wide-angle action camera", "camera_motion":
                           "Smooth forward drone flight, no cuts"},
        "style_medium": "Live-action video",
        "artistic_style": "Photorealistic drone footage, no stylisation",
        "text_and_signage_elements": [],
        "resolution": {"W": w, "H": h},
        "aspect_ratio": "16,9",
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    args = ap.parse_args()
    cfg = load_config(args.config)
    cam = cfg["camera"]
    pk = {**cfg["package"], **cfg["flights"][args.flight].get("package", {})}
    fdir = BUILD / "flight" / args.flight
    render = json.loads((fdir / "render" / "render.json").read_text())
    n = render["frames"]
    poses = np.loadtxt(fdir / "camera_poses_tum.txt")[:n]
    frames_dir = fdir / "render" / "frames"

    out = BUILD / "cosmos" / args.flight / "clips"
    out.mkdir(parents=True, exist_ok=True)
    starts = list(range(0, n, pk["max_frames"]))
    for ci, s in enumerate(starts):
        e = min(s + pk["max_frames"], n)
        if e - s < 25:  # hosted API minimum
            print(f"skipping tail clip {s}-{e} (< 25 frames)")
            continue
        cdir = out / f"clip_{ci:02d}"
        (cdir / "prompts").mkdir(parents=True, exist_ok=True)
        rgb = [np.asarray(Image.open(frames_dir / f"{i:06d}_rgb.png").convert("RGB")) for i in range(s, e)]
        depth = [np.load(frames_dir / f"{i:06d}_depth.npy") for i in range(s, e)]
        h, w = rgb[0].shape[:2]
        if (w, h) != (cam["width"], cam["height"]):
            raise SystemExit(f"render is {w}x{h}, config expects {cam['width']}x{cam['height']}")
        encode(rgb, cdir / "rgb.mp4", cam["fps"])
        encode([depth_to_rgb(d, pk["depth_near_m"], pk["depth_far_m"], pk["depth_convention"]) for d in depth],
               cdir / "depth.mp4", cam["fps"])
        encode([edges_to_rgb(f, pk["canny"]) for f in rgb], cdir / "edge.mp4", cam["fps"])
        for cond in cfg["cosmos"]["conditions"]:
            p = structured_prompt(cfg, cfg["flights"][args.flight]["scene_description"], cond, w, h,
                                  cam["mount_pitch_deg"])
            (cdir / "prompts" / f"{cond}.json").write_text(json.dumps(p, indent=2) + "\n")
        clip = {"flight": args.flight, "clip": ci, "frames": [s, e], "num_frames": e - s, "fps": cam["fps"],
                "width": w, "height": h, "intrinsics": render["intrinsics"],
                "depth_encoding": {k: pk[k] for k in ("depth_near_m", "depth_far_m", "depth_convention")},
                "poses_tum": poses[s:e].tolist()}
        (cdir / "clip.json").write_text(json.dumps(clip) + "\n")
        print(f"clip_{ci:02d}: frames {s}-{e} ({e - s}) -> {cdir}")


if __name__ == "__main__":
    main()
