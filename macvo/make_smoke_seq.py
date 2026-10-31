#!/usr/bin/env python3
"""
Build a small IsaacStereo-format sequence from an existing mono RGB+depth render
(build/flight/<flight>/render/frames/NNNNNN_{rgb.png,depth.npy} + camera_poses_tum.txt),
synthesising the right view by depth-based warping (u_R = u_L - fx*B/z). Used to smoke-test
MAC-VO without the stereo renderer. Runs fine inside the macvo Docker image (CPU only):

  docker run --rm -u $(id -u):$(id -g) -v $PWD:/repo -v <out_parent>:/out isaac-exploration/macvo \
      python /repo/macvo/make_smoke_seq.py --flight /repo/build/flight/marina_bay_pass --out /out/macvo_smoke
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--start", type=int, default=300)
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--baseline", type=float, default=2.0, help="metres")
    ap.add_argument("--width", type=int, default=640)
    args = ap.parse_args()

    render = json.loads(Path(args.flight, "render", "render.json").read_text())
    K = render["intrinsics"]
    W0, H = K["width"], K["height"]
    x0 = (W0 - args.width) // 2
    fx, fy, cx, cy = K["fx"], K["fy"], K["cx"] - x0, K["cy"]
    B = args.baseline

    frames_out = Path(args.out, "frames")
    frames_out.mkdir(parents=True, exist_ok=True)

    # Drone-style body (FLU) with a forward-looking camera 10 cm ahead of the body origin.
    T_body_left = np.array([[0., 0., 1., 0.10],
                            [-1., 0., 0., 0.],
                            [0., -1., 0., 0.],
                            [0., 0., 0., 1.]])
    T_left_right = np.eye(4); T_left_right[0, 3] = B
    meta = {"width": args.width, "height": H, "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "baseline_m": B, "camera_hz": 30,
            "T_body_left": T_body_left.tolist(), "T_left_right": T_left_right.tolist()}
    Path(args.out, "stereo.json").write_text(json.dumps(meta, indent=2))

    poses = np.loadtxt(Path(args.flight, "camera_poses_tum.txt"), comments="#")
    src = Path(args.flight, "render", "frames")
    xs = np.arange(W0, dtype=np.float32)[None].repeat(H, 0)
    ys = np.arange(H, dtype=np.float32)[:, None].repeat(W0, 1)
    disp_stats = []
    with open(Path(args.out, "times.csv"), "w") as ft:
        ft.write("frame,t_s\n")
        for k in range(args.count):
            i = args.start + k
            rgb = cv2.imread(str(Path(src, f"{i:06d}_rgb.png")), cv2.IMREAD_COLOR)
            z = np.load(Path(src, f"{i:06d}_depth.npy")).astype(np.float32)
            d = np.where(np.isfinite(z) & (z > 0), fx * B / np.maximum(z, 1e-3), 0.).astype(np.float32)
            disp_stats.append(np.median(d[d > 0]) if (d > 0).any() else 0.)
            # forward-splat disparity into the right view (z-buffer: keep largest disparity)
            xr = np.round(xs - d).astype(np.int64)
            ok = (xr >= 0) & (xr < W0)
            dR = np.full((H, W0), -1., np.float32)
            np.maximum.at(dR, (ys.astype(np.int64)[ok], xr[ok]), d[ok])
            # fill disocclusion holes with the nearest valid value from the right (background side)
            for row in dR:
                valid = row >= 0
                if not valid.any(): row[:] = 0; continue
                idx = np.where(valid, np.arange(W0), W0 - 1)
                idx = np.minimum.accumulate(idx[::-1])[::-1]
                row[:] = row[idx]
                row[row < 0] = 0
            right = cv2.remap(rgb, xs + dR, ys, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            cv2.imwrite(str(Path(frames_out, f"{k:06d}_left.png")), rgb[:, x0:x0 + args.width])
            cv2.imwrite(str(Path(frames_out, f"{k:06d}_right.png")), right[:, x0:x0 + args.width])
            ft.write(f"{k},{poses[i, 0]:.9f}\n")

    with open(Path(args.out, "groundtruth_left.csv"), "w") as fg:
        fg.write("t_s,px,py,pz,qx,qy,qz,qw\n")
        for row in poses:
            fg.write(",".join(f"{v:.9f}" for v in row) + "\n")
    print(f"wrote {args.count} stereo frames to {args.out}; median disparity {np.median(disp_stats):.2f} px")


if __name__ == "__main__":
    main()
