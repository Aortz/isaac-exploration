#!/usr/bin/env python3
"""
Headless single-instance MAC-VO runner for isaac-exploration stereo sequences.

    python macvo/run_macvo.py --seq <seq_dir> --out <out_dir> [--mode Fast|Performant]

Normally launched through macvo/run_macvo.sh (Docker). Needs MAC-VO's repo root on disk
(env MACVO_ROOT, default ../third_party/macvo relative to this file).

Outputs in <out_dir>:
  poses.npy                 MAC-VO native: N x 8 [time_ns, tx,ty,tz,qx,qy,qz,qw], BODY pose
                            relative to the first body pose (first row = identity).
  traj_est_left_world.csv   t_s,px,py,pz,qx,qy,qz,qw  LEFT camera (OpenCV, cam->world), same
                            format as groundtruth_left.csv. Origin-aligned to GT at frame 0 if GT
                            exists, else world == left camera frame at frame 0.
  traj_est_body_world.csv   same, for the body frame.
  ref_poses.npy             GT body poses (MAC-VO native), only if groundtruth_left.csv exists.
  eval.json                 MAC-VO EvalSeq metrics (ATE/RTE/ROE/RPE), only with GT.
  run_stats.json            frames, fps, startup time, peak VRAM.
  config.yaml, tensor_map.npz, elapsed_time.json   MAC-VO sandbox files.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

T_START = time.time()
HERE = Path(__file__).resolve().parent
MACVO_ROOT = Path(os.environ.get("MACVO_ROOT", HERE.parent / "third_party" / "macvo")).resolve()
if str(MACVO_ROOT) not in sys.path:
    sys.path.insert(0, str(MACVO_ROOT))
if str(HERE) not in sys.path:
    sys.path.insert(1, str(HERE))


def limit_torch_threads(n: int) -> None:
    """MAC-VO hardcodes torch.set_num_threads(4) in the main process and (8) in its spawned
    optimizer process. Cap both at n. This module is re-imported as __mp_main__ by the spawned
    child, so running this at import time (driven by env MACVO_THREADS) covers the child too."""
    if n <= 0:
        return
    os.environ["MACVO_THREADS"] = str(n)
    import torch
    if not hasattr(torch, "_macvo_orig_set_num_threads"):
        torch._macvo_orig_set_num_threads = torch.set_num_threads
        torch.set_num_threads = lambda k: torch._macvo_orig_set_num_threads(max(1, min(int(k), n)))
    torch.set_num_threads(n)


limit_torch_threads(int(os.environ.get("MACVO_THREADS", "0") or 0))


def get_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seq", required=True, type=Path, help="sequence dir (stereo.json, frames/, times.csv, ...)")
    p.add_argument("--out", required=True, type=Path, help="output dir (created; must be unique per instance)")
    p.add_argument("--mode", default="Fast", choices=["Fast", "Performant"])
    p.add_argument("--odom", type=Path, default=None, help="override odometry config yaml (relative to MAC-VO root)")
    p.add_argument("--seq-from", type=int, default=0)
    p.add_argument("--seq-to", type=int, default=None)
    p.add_argument("--no-gt", action="store_true", help="ignore groundtruth_left.csv")
    p.add_argument("--no-eval", action="store_true", help="skip MAC-VO EvalSeq after the run")
    p.add_argument("--no-mapping", action="store_true", help="disable dense mapping (less RAM/VRAM, VO only)")
    p.add_argument("--preload", action="store_true", help="load all frames into RAM before running")
    p.add_argument("--threads", type=int, default=int(os.environ.get("MACVO_THREADS", "0") or 0),
                   help="cap torch CPU threads per process (0 = MAC-VO defaults: 4 main + 8 optimizer)")
    p.add_argument("--timing", action="store_true", help="enable MAC-VO Utility.Timer report")
    return p.parse_args()


def write_traj_csv(path: Path, t_s, T):
    from scipy.spatial.transform import Rotation
    import numpy as np
    q = Rotation.from_matrix(T[:, :3, :3]).as_quat()
    data = np.concatenate([np.asarray(t_s)[:, None], T[:, :3, 3], q], axis=1)
    np.savetxt(path, data, delimiter=",", header="t_s,px,py,pz,qx,qy,qz,qw", comments="", fmt="%.9f")


def main():
    args = get_args()
    seq_dir, out_dir = args.seq.resolve(), args.out.resolve()
    os.chdir(MACVO_ROOT)  # MAC-VO configs use ./Model/... and relative !include paths

    limit_torch_threads(args.threads)
    import numpy as np
    import torch

    from DataLoader import SequenceBase, StereoFrame, smart_transform
    from Odometry.MACVO import MACVO
    from Utility.Config import load_config, asNamespace
    from Utility.Sandbox import Sandbox
    from Utility.Timer import Timer
    from Utility.Visualize import fig_plt, rr_plt
    import isaac_sequence  # noqa: F401  registers "IsaacStereo"
    from isaac_sequence import _matrix_from_se3

    rr_plt.default_mode = "none"   # never touch rerun (no viewer spawn / TCP connect)
    fig_plt.default_mode = "none"
    Timer.setup(active=args.timing)

    odom_path = args.odom or Path(f"Config/Experiment/MACVO/MACVO_{args.mode}.yaml")
    cfg, cfg_dict = load_config(odom_path)
    odomcfg_dict = cfg_dict["Odometry"]
    if args.no_mapping:
        odomcfg_dict["args"]["mapping"] = False
    datacfg_dict = {"type": "IsaacStereo", "name": os.environ.get("MACVO_SEQ_NAME", seq_dir.name),
                    "args": {"root": str(seq_dir), "gt_pose": not args.no_gt}}
    project_name = f"{odomcfg_dict['name']}@{datacfg_dict['name']}"

    exp_space = Sandbox(out_dir)
    exp_space.config = {
        "Project": project_name,
        "Odometry": odomcfg_dict,
        "Data": {"args": datacfg_dict, "end_idx": args.seq_to, "start_idx": args.seq_from},
    }

    seq_raw = SequenceBase[StereoFrame].instantiate(datacfg_dict["type"], datacfg_dict["args"])
    sequence = smart_transform(seq_raw.clip(args.seq_from, args.seq_to), cfg.Preprocess)
    if args.preload:
        sequence = sequence.preload()

    system = MACVO[StereoFrame].from_config(asNamespace(exp_space.config))
    t_ready = time.time()

    frame_t: list[float] = []
    def on_frame(frame, system, pb):
        frame_t.append(time.time())

    system.receive_frames(sequence, exp_space, on_frame_finished=on_frame)
    t_done = time.time()
    Timer.report()
    Timer.save_elapsed(exp_space.path("elapsed_time.json"))

    if not exp_space.path("poses.npy").exists():
        print("MAC-VO run failed (no poses.npy); see log above", file=sys.stderr)
        sys.exit(1)

    # ---- export trajectories in the renderer's convention -------------------------------
    est = np.load(exp_space.path("poses.npy"))           # [time_ns, 7] body poses rel. body0
    t_s = est[:, 0] / 1e9
    T_b0_bk = _matrix_from_se3(est[:, 1:8])
    T_body_left = seq_raw.T_body_left
    if seq_raw.gt_pose is not None:
        first = int(seq_raw.get_index(0))
        T_w_b0 = _matrix_from_se3(seq_raw.gt_pose[first].numpy())[0]
    else:
        T_w_b0 = np.linalg.inv(T_body_left)                # world := left camera at frame 0
    T_w_body = T_w_b0[None] @ T_b0_bk
    write_traj_csv(exp_space.path("traj_est_body_world.csv"), t_s, T_w_body)
    write_traj_csv(exp_space.path("traj_est_left_world.csv"), t_s, T_w_body @ T_body_left[None])

    # ---- stats ---------------------------------------------------------------------------
    n = len(frame_t)
    steady = (n - 2) / (frame_t[-1] - frame_t[1]) if n > 3 else float("nan")  # skip init + CUDA-graph build
    stats = {
        "frames": n,
        "mode": args.mode,
        "startup_s": round(t_ready - T_START, 2),
        "run_s": round(t_done - t_ready, 2),
        "fps_overall": round(n / (t_done - t_ready), 2),
        "fps_steady": round(steady, 2),
        "first_frame_s": round(frame_t[0] - t_ready, 2) if n else None,
        "cuda_max_allocated_MiB": round(torch.cuda.max_memory_allocated() / 2**20, 1),
        "cuda_max_reserved_MiB": round(torch.cuda.max_memory_reserved() / 2**20, 1),
        "torch_threads": torch.get_num_threads(),
    }
    with open(exp_space.path("run_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)
    print("RUN_STATS " + json.dumps(stats))

    if not args.no_eval and exp_space.path("ref_poses.npy").exists():
        from Evaluation.EvalSeq import EvaluateSequences
        from Utility.PrettyPrint import print_as_table
        header, result = EvaluateSequences([str(exp_space.folder)], correct_scale=False)
        print_as_table(header, result)
        def _num(c):
            try: return float(c)
            except (TypeError, ValueError): return str(c)
        rows = [dict(zip([str(h) for h in header], map(_num, r))) for r in result]
        with open(exp_space.path("eval.json"), "w") as f:  # ATE/RTE in m, ROE in deg
            json.dump(rows[0] if rows else {}, f, indent=2)


if __name__ == "__main__":
    main()
