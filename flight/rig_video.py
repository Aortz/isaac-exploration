"""Turn a rig run saved with --save-all into videos: one mp4 per camera and a 2x2 grid.

Run: uv run python flight/rig_video.py --flight marina_bay_loop --run rig_lockstep
Writes build/flight/<flight>/<run>/{<cam>.mp4, grid.mp4}
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402

ENCODE = ["-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--run", default="rig_lockstep")
    ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
    args = ap.parse_args()
    rig = load_config(args.config)["rig"]
    run = BUILD / "flight" / args.flight / args.run
    cams = list(rig["cameras"])
    fps = str(rig["camera_hz"])
    inputs = []
    for c in cams:
        pattern = str(run / "frames" / f"%06d_{c}.png")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", fps, "-i", pattern, *ENCODE,
                        str(run / f"{c}.mp4")], check=True)
        inputs += ["-framerate", fps, "-i", pattern]
    # 2x2: front | left over back | right, with camera names burned in.
    labels = "".join(f"[{i}]drawtext=text='{c}':x=10:y=10:fontsize=22:fontcolor=white[v{i}];" for i, c in enumerate(cams))
    grid = labels + "[v0][v1][v2][v3]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[out]"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", *inputs, "-filter_complex", grid, "-map", "[out]", *ENCODE,
                    str(run / "grid.mp4")], check=True)
    print(f"videos -> {run}/grid.mp4 and {', '.join(c + '.mp4' for c in cams)}")


if __name__ == "__main__":
    main()
