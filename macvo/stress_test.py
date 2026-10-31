"""How many MAC-VO instances fit on this GPU at once? Run N concurrent instances, for growing N.

Each round starts N containers (macvo/run_macvo.sh) on the same stereo sequence, samples GPU memory,
GPU utilisation and host MemAvailable every 0.5 s, and reads each instance's run_stats.json. Rounds
stop when an instance fails (e.g. CUDA OOM), GPU memory passes --vram-limit-gib, or aggregate
throughput drops. If host MemAvailable falls below --min-free-ram-gib mid-round, every container of
the round is killed: an 8-instance round with --preload once ran the 59 GB host out of RAM, and the
kernel OOM killer took the terminal (and the Claude Code session in it) with it.

Run: uv run python macvo/stress_test.py --seq build/flight/marina_bay_loop_pegasus/rig_lockstep_stereo
Writes build/macvo/stress/<seq name>/{stress.json, stress.png, n<N>/i<k>/...}
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD  # noqa: E402

HERE = Path(__file__).resolve().parent


def gpu_sample() -> tuple[float, float]:
    q = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,utilization.gpu", "--format=csv,noheader,nounits"],
                       capture_output=True, text=True, check=True).stdout.split(",")
    return float(q[0]), float(q[1])


def mem_available_gib() -> float:
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 2 ** 20
    raise RuntimeError("no MemAvailable in /proc/meminfo")


def run_round(n: int, args, out_root: Path) -> dict:
    base_mib, _ = gpu_sample()
    samples, ram, stop, guard = [], [], threading.Event(), threading.Event()

    def sampler():
        while not stop.is_set():
            samples.append((time.time(),) + gpu_sample())
            ram.append(mem_available_gib())
            if ram[-1] < args.min_free_ram_gib and not guard.is_set():
                guard.set()
                names = [f"macvo-stress-n{n}-i{i}" for i in range(n)]
                subprocess.run(["docker", "kill", *names], capture_output=True)
            time.sleep(0.5)

    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    env = {**os.environ, "MACVO_THREADS": str(args.threads)}
    procs, t0 = [], time.time()
    for i in range(n):
        out = out_root / f"n{n}" / f"i{i}"
        out.mkdir(parents=True, exist_ok=True)
        cmd = [str(HERE / "run_macvo.sh"), str(args.seq), str(out), args.mode, "--seq-to", str(args.frames),
               "--no-eval", *(["--preload"] if args.preload else []), *(["--no-mapping"] if args.no_mapping else [])]
        log = open(out / "run.log", "w")
        procs.append((subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                       env={**env, "MACVO_NAME": f"macvo-stress-n{n}-i{i}"}), out, log))
    codes = []
    for p, _, log in procs:
        codes.append(p.wait())
        log.close()
    wall = time.time() - t0
    stop.set()
    th.join()

    inst = []
    for (p, out, _), code in zip(procs, codes):
        st = out / "run_stats.json"
        s = json.loads(st.read_text()) if st.exists() and code == 0 else {}
        inst.append({"exit": code, "fps_steady": s.get("fps_steady"), "fps_overall": s.get("fps_overall"),
                     "cuda_max_reserved_MiB": s.get("cuda_max_reserved_MiB")})
    ok = [i for i in inst if i["exit"] == 0 and i["fps_steady"]]
    peak = max(m for _, m, _ in samples) if samples else base_mib
    utils = [u for _, _, u in samples]
    agg = sum(i["fps_steady"] for i in ok)
    return {"instances": n, "ok": len(ok), "failed": n - len(ok), "wall_s": round(wall, 1),
            "fps_per_instance_mean": round(agg / len(ok), 2) if ok else 0.0,
            "fps_per_instance_min": round(min(i["fps_steady"] for i in ok), 2) if ok else 0.0,
            "fps_aggregate": round(agg, 2),
            "fps_aggregate_wall": round(len(ok) * args.frames / wall, 2),   # includes container start + model load
            "aggregate_vs_camera_hz": round(agg / args.camera_hz, 3),
            "gpu_mem_baseline_MiB": base_mib, "gpu_mem_peak_MiB": peak,
            "gpu_mem_per_instance_MiB": round((peak - base_mib) / n, 0),
            "gpu_util_mean_pct": round(sum(utils) / len(utils), 1) if utils else None,
            "host_mem_available_min_GiB": round(min(ram), 1) if ram else None, "ram_guard_tripped": guard.is_set(),
            "per_instance": inst}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seq", required=True, type=Path)
    ap.add_argument("--counts", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6, 8, 10])
    ap.add_argument("--frames", type=int, default=600, help="frames per instance (from the start)")
    ap.add_argument("--mode", default="Fast", choices=["Fast", "Performant"])
    ap.add_argument("--threads", type=int, default=2, help="torch CPU threads per process")
    ap.add_argument("--no-mapping", action="store_true")
    ap.add_argument("--vram-limit-gib", type=float, default=15.5)
    ap.add_argument("--min-free-ram-gib", type=float, default=6.0, help="kill the round below this host MemAvailable")
    ap.add_argument("--preload", action="store_true", help="each instance loads its frames into RAM first (~1 GB each)")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    args.seq = args.seq.resolve()
    args.camera_hz = json.loads((args.seq / "stereo.json").read_text())["camera_hz"]
    out_root = BUILD / "macvo" / "stress" / (args.seq.parent.name + "_" + args.seq.name + (f"_{args.tag}" if args.tag else ""))
    out_root.mkdir(parents=True, exist_ok=True)

    rounds = []
    for n in args.counts:
        r = run_round(n, args, out_root)
        rounds.append(r)
        write_report(rounds, args, out_root)     # after every round, so a crash keeps what finished
        print(json.dumps({k: v for k, v in r.items() if k != "per_instance"}), flush=True)
        if r["ram_guard_tripped"]:
            print(f"stopping: host MemAvailable fell below {args.min_free_ram_gib} GiB, round killed")
            break
        if r["failed"]:
            print(f"stopping: {r['failed']} of {n} instances failed")
            break
        if r["gpu_mem_peak_MiB"] > args.vram_limit_gib * 1024:
            print("stopping: GPU memory limit reached")
            break
        if len(rounds) > 1 and r["fps_aggregate"] < 0.95 * max(x["fps_aggregate"] for x in rounds[:-1]):
            print("stopping: aggregate throughput dropped")
            break
    print(f"report -> {out_root}/stress.json")


def write_report(rounds: list, args, out_root: Path) -> None:
    ok = [x for x in rounds if not x["failed"]]
    best = max(ok, key=lambda x: x["fps_aggregate"]) if ok else None
    report = {"seq": str(args.seq), "mode": args.mode, "frames_per_instance": args.frames, "threads": args.threads,
              "no_mapping": args.no_mapping, "preload": args.preload, "camera_hz": args.camera_hz, "rounds": rounds,
              "max_concurrent_ok": max((x["instances"] for x in ok), default=0),
              "best_throughput_instances": best["instances"] if best else None,
              "best_fps_aggregate": best["fps_aggregate"] if best else None}
    (out_root / "stress.json").write_text(json.dumps(report, indent=2) + "\n")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = [x["instances"] for x in rounds]
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    ax[0].plot(n, [x["fps_aggregate"] for x in rounds], "o-", label="aggregate")
    ax[0].plot(n, [x["fps_per_instance_mean"] for x in rounds], "s-", label="per instance (mean)")
    ax[0].axhline(args.camera_hz, color="grey", ls="--", lw=1, label=f"camera {args.camera_hz:g} Hz")
    ax[0].set_ylabel("frames / s")
    ax[0].legend()
    ax[1].plot(n, [x["gpu_mem_peak_MiB"] / 1024 for x in rounds], "o-")
    ax[1].axhline(16.0, color="r", ls="--", lw=1)
    ax[1].set_ylabel("peak GPU memory [GiB]")
    ax[2].plot(n, [x["host_mem_available_min_GiB"] or 0 for x in rounds], "o-")
    ax[2].axhline(args.min_free_ram_gib, color="r", ls="--", lw=1)
    ax[2].set_ylabel("min host MemAvailable [GiB]")
    for a in ax:
        a.set_xlabel("concurrent MAC-VO instances")
        a.grid(alpha=0.3)
    fig.suptitle(f"MAC-VO {args.mode} on RTX 5080, {args.frames} frames 640x480 per instance")
    fig.tight_layout()
    fig.savefig(out_root / "stress.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
