"""Score an OpenVINS run against the F1 ground truth: ATE after 4-DoF alignment, drift, plot.

OpenVINS (save_total_state) writes: t, q_GtoI (JPL x y z w = Hamilton IMU-to-global), p_IinG, v_IinG, ...
The IMU frame is the body (FLU) frame of groundtruth_200hz.csv, so poses compare directly.
VIO is observable only up to yaw and translation, so the estimate is aligned with a 4-DoF fit
(yaw + translation) over the whole run, as in the standard VIO evaluation.

Run: uv run python ros2/openvins/eval_vio.py --flight marina_bay_loop --run build/openvins/rig4/run_loop
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pipeline.common.config import BUILD  # noqa: E402


def align_4dof(est: np.ndarray, gt: np.ndarray) -> tuple[float, np.ndarray]:
    """Yaw + translation minimising ||gt - (Rz est + t)||."""
    mu_e, mu_g = est.mean(0), gt.mean(0)
    e, g = est - mu_e, gt - mu_g
    yaw = np.arctan2((e[:, 0] * g[:, 1] - e[:, 1] * g[:, 0]).sum(), (e[:, 0] * g[:, 0] + e[:, 1] * g[:, 1]).sum())
    rz = R.from_euler("z", yaw).as_matrix()
    return yaw, mu_g - rz @ mu_e


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", required=True)
    ap.add_argument("--run", required=True, help="OpenVINS output dir (holds ov_estimate.txt)")
    args = ap.parse_args()
    run = Path(args.run)
    est = np.loadtxt(run / "ov_estimate.txt", comments="#")
    gt = np.loadtxt(BUILD / "flight" / args.flight / "groundtruth_200hz.csv", delimiter=",", comments="#")
    t = est[:, 0]
    keep = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    est, t = est[keep], t[keep]
    p_gt = np.stack([np.interp(t, gt[:, 0], gt[:, 1 + k]) for k in range(3)], axis=1)
    r_gt = Slerp(gt[:, 0], R.from_quat(gt[:, [5, 6, 7, 4]]))(t)
    p_est, r_est = est[:, 5:8], R.from_quat(est[:, 1:5])

    yaw, trans = align_4dof(p_est, p_gt)
    rz = R.from_euler("z", yaw)
    p_al = rz.apply(p_est) + trans
    err = np.linalg.norm(p_al - p_gt, axis=1)
    rot_err = np.degrees(((rz * r_est).inv() * r_gt).magnitude())
    path = float(np.linalg.norm(np.diff(p_gt, axis=0), axis=1).sum())
    report = {
        "flight": args.flight, "run": str(run), "poses": int(len(t)),
        "time_span_s": [round(float(t[0]), 2), round(float(t[-1]), 2)],
        "first_estimate_after_s": round(float(t[0] - gt[0, 0]), 2),
        "path_length_m": round(path, 1),
        "ate_rmse_m": round(float(np.sqrt((err ** 2).mean())), 3),
        "ate_max_m": round(float(err.max()), 3),
        "final_error_m": round(float(err[-1]), 3),
        "final_error_pct_of_path": round(100 * float(err[-1]) / max(path, 1e-6), 3),
        "rotation_error_deg_rmse": round(float(np.sqrt((rot_err ** 2).mean())), 3),
        "alignment_yaw_deg": round(float(np.degrees(yaw)), 3),
    }
    (run / "eval.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].plot(gt[:, 1], gt[:, 2], "k-", lw=1, label="ground truth")
    ax[0].plot(p_al[:, 0], p_al[:, 1], "r-", lw=1, label="OpenVINS (4-DoF aligned)")
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("east [m]")
    ax[0].set_ylabel("north [m]")
    ax[0].legend()
    ax[0].set_title(f"ATE RMSE {report['ate_rmse_m']} m over {report['path_length_m']} m")
    ax[1].plot(t, err, "r-", lw=1)
    ax[1].set_xlabel("sim time [s]")
    ax[1].set_ylabel("position error [m]")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run / "eval.png", dpi=110)


if __name__ == "__main__":
    main()
