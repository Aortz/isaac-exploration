"""Collect the evidence of the exploration into demo/ (committed), so results can be shown without the build tree.

Copies the test report, results table and per-case metrics, the per-flight summaries (trajectory, Pegasus,
render rates), environment probe images and evaluation plots, and makes small preview videos plus a
still per rendered flight. Full-resolution videos, frames and ROS 2 bags stay in build/ (too big for git).
Safe to re-run as more test cases finish: it rebuilds demo/ from build/.

Run: uv run python testplan/package_demo.py
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT  # noqa: E402

DEMO = PROJECT_ROOT / "demo"
SMALL = ("case.json", "metrics.json", "eval.json", "trajectory.json", "pegasus.json", "rates.json",
         "stereo.json", "camchain.yaml", "probe.json", "g2.json", "stress.json")


def copy(src: Path, dst: Path) -> None:
    if src.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def preview(src: Path, dst: Path, width: int) -> None:
    """Small H.264 preview (about 2-6 MB for a 100 s flight) and a still from the middle."""
    if not src.exists() or (dst.exists() and dst.stat().st_mtime > src.stat().st_mtime):
        return
    if time.time() - src.stat().st_mtime < 120:                      # still being written (a render in progress)
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(src), "-vf", f"scale={width}:-2",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "30", "-pix_fmt", "yuv420p", "-an",
                    "-movflags", "+faststart", str(dst)], check=True)
    dur = float(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0",
                                str(dst)], capture_output=True, text=True).stdout or 0)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-ss", f"{dur / 2:.1f}", "-i", str(dst), "-frames:v", "1",
                    "-q:v", "4", str(dst.with_suffix(".jpg"))], check=True)


def main() -> None:
    tests = BUILD / "tests"
    # Test-plan results.
    for name in ("report.md", "report.png", "results.csv"):
        copy(tests / name, DEMO / "results" / name)
    for case in sorted(p for p in tests.glob("TC-*") if p.is_dir()):
        for f in case.rglob("*"):
            if f.name in SMALL or f.name == "eval.png":
                copy(f, DEMO / "results" / "cases" / case.name / f.relative_to(case))
    # Flights: summaries + previews.
    for fdir in sorted(p for p in (BUILD / "flight").iterdir() if p.is_dir()):
        out = DEMO / "flights" / fdir.name
        for f in fdir.glob("*.json"):
            copy(f, out / f.name)
        for run in fdir.glob("rig_*"):
            for f in run.glob("*"):
                if f.name in SMALL:
                    copy(f, out / run.name / f.name)
            preview(run / "grid.mp4", DEMO / "media" / f"{fdir.name}__{run.name}_grid.mp4", 640)
            preview(run / "stereo.mp4", DEMO / "media" / f"{fdir.name}__{run.name}_stereo.mp4", 960)
        preview(fdir / "render" / f"{fdir.name}.mp4", DEMO / "media" / f"{fdir.name}__render.mp4", 832)
        preview(fdir / "watch" / "chase.mp4", DEMO / "media" / f"{fdir.name}__chase.mp4", 960)
    # Environments (envbuild recipes keep their manifest and config block too).
    for env in sorted(p for p in (BUILD / "envs").glob("*") if p.is_dir()):
        for f in env.glob("*"):
            if f.suffix == ".png" or f.name in SMALL or f.name in ("env.json", "environment.yaml", "preview.jpg"):
                copy(f, DEMO / "environments" / env.name / f.name)
    # Google-tile environments: the verify render of each georeferenced stage.
    for png in (BUILD / "cesium").glob("*_google.png"):
        copy(png, DEMO / "environments" / png.stem.removesuffix("_google") / "tiles_view.png")
    # Earlier evaluations: OpenVINS on Cesium, MAC-VO stress test.
    for ev in (BUILD / "openvins").rglob("eval.*"):
        copy(ev, DEMO / "openvins_cesium" / ev.parent.relative_to(BUILD / "openvins") / ev.name)
    for st in (p for p in (BUILD / "macvo" / "stress").rglob("stress.*") if "smoke" not in str(p)):
        copy(st, DEMO / "macvo_stress" / st.parent.name / st.name)
    copy(BUILD / "macvo" / "pegasus_loop_full" / "run_stats.json", DEMO / "macvo_stress" / "full_loop_run_stats.json")
    sizes = sum(f.stat().st_size for f in DEMO.rglob("*") if f.is_file())
    print(json.dumps({"demo_dir": str(DEMO), "files": sum(1 for f in DEMO.rglob("*") if f.is_file()),
                      "size_mb": round(sizes / 1e6, 1)}))


if __name__ == "__main__":
    main()
