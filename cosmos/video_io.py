"""Read/write videos through ffmpeg (handles the VP9-in-MP4 that the hosted Cosmos API returns)."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np


def probe(path: Path) -> dict:
    out = subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0", "-count_frames",
                                   "-show_entries", "stream=width,height,r_frame_rate,nb_read_frames,codec_name",
                                   "-of", "json", str(path)])
    s = json.loads(out)["streams"][0]
    num, den = (int(x) for x in s["r_frame_rate"].split("/"))
    return {"width": int(s["width"]), "height": int(s["height"]), "fps": num / den,
            "frames": int(s["nb_read_frames"]), "codec": s["codec_name"]}


def read_video(path: Path, size: tuple[int, int] | None = None) -> list[np.ndarray]:
    """Decode to RGB uint8 frames, optionally resized to size=(width, height)."""
    info = probe(path)
    w, h = size or (info["width"], info["height"])
    cmd = ["ffmpeg", "-loglevel", "error", "-i", str(path)]
    if size:
        cmd += ["-vf", f"scale={w}:{h}:flags=bicubic"]
    cmd += ["-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    raw = subprocess.check_output(cmd)
    return list(np.frombuffer(raw, np.uint8).reshape(-1, h, w, 3))


def write_video(frames, path: Path, fps: int) -> None:
    """Write RGB uint8 frames to an H.264 mp4 (yuv420p, near-lossless)."""
    h, w = frames[0].shape[:2]
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
         "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "12", "-pix_fmt", "yuv420p",
         "-movflags", "+faststart", str(path)],
        stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(np.ascontiguousarray(f, dtype=np.uint8).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed writing {path}")
