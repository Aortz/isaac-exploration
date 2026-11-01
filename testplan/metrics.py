"""Trajectory metrics for the test plan: alignment, ATE, final drift, rotation error, RPE over distance."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def load_gt(path) -> tuple[np.ndarray, np.ndarray, R]:
    """groundtruth_200hz.csv -> (t, p, rot) of the body in world ENU."""
    gt = np.loadtxt(path, delimiter=",", comments="#")
    return gt[:, 0], gt[:, 1:4], R.from_quat(gt[:, [5, 6, 7, 4]])


def gt_at(t: np.ndarray, gt) -> tuple[np.ndarray, R]:
    tg, pg, rg = gt
    return np.stack([np.interp(t, tg, pg[:, k]) for k in range(3)], axis=1), Slerp(tg, rg)(t)


def align(est: np.ndarray, ref: np.ndarray, mode: str, r_est: R | None = None, r_ref: R | None = None
          ) -> tuple[float, np.ndarray, np.ndarray]:
    """Least-squares (scale, R, t) with ref ~ s R est + t. mode: '4dof' (yaw + translation), 'se3', 'sim3'.

    Rotation (4-DoF: yaw) comes from the positions, unless they barely move (hover) or lie on a line (SE(3)/Sim(3)):
    then from the attitudes.
    """
    mu_e, mu_r = est.mean(0), ref.mean(0)
    e, r = est - mu_e, ref - mu_r
    if mode == "4dof":
        if np.sqrt((r[:, :2] ** 2).sum(1).mean()) < 1.0 and r_est is not None:
            d = (r_ref * r_est.inv()).as_rotvec()
            yaw = float(np.angle(np.exp(1j * R.from_rotvec(d).as_euler("ZYX")[:, 0]).mean()))
        else:
            yaw = np.arctan2((e[:, 0] * r[:, 1] - e[:, 1] * r[:, 0]).sum(), (e[:, 0] * r[:, 0] + e[:, 1] * r[:, 1]).sum())
        rot = R.from_euler("z", yaw).as_matrix()
        return 1.0, rot, mu_r - rot @ mu_e
    sv_ref = np.linalg.svd(r, compute_uv=False)
    if r_est is not None and (np.sqrt((r ** 2).sum(1).mean()) < 1.0 or sv_ref[1] / np.sqrt(len(r)) < 1.0):
        # Hover, or a straight line (rotation about the line is unobservable from positions): take the
        # rotation from the attitudes.
        rot = (r_ref * r_est.inv()).mean().as_matrix()
        return 1.0, rot, mu_r - rot @ mu_e
    u, sv, vt = np.linalg.svd(r.T @ e / len(e))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    rot = u @ d @ vt
    s = float((sv * np.diag(d)).sum() / ((e ** 2).sum() / len(e))) if mode == "sim3" else 1.0
    return s, rot, mu_r - s * rot @ mu_e


def rpe_distance(p_est: np.ndarray, r_est: R, p_gt: np.ndarray, r_gt: R, seg_m: float = 10.0) -> dict:
    """Relative translation error over gt path segments of seg_m metres (KITTI-style, no alignment needed)."""
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1))])
    if s[-1] < seg_m:
        return {"segments": 0}
    j = np.searchsorted(s, s + seg_m)
    i = np.nonzero(j < len(s))[0][:: max(1, len(s) // 2000)]
    j = j[i]
    d_gt = r_gt[i].inv().apply(p_gt[j] - p_gt[i])
    d_est = r_est[i].inv().apply(p_est[j] - p_est[i])
    err = np.linalg.norm(d_est - d_gt, axis=1)
    return {"segments": int(len(i)), "seg_m": seg_m, "rmse_m": float(np.sqrt((err ** 2).mean())),
            "rmse_pct": float(100 * np.sqrt((err ** 2).mean()) / seg_m)}


def trajectory_metrics(t: np.ndarray, p_est: np.ndarray, r_est: R, gt, mode: str) -> dict:
    p_gt, r_gt = gt_at(t, gt)
    s, rot, tr = align(p_est, p_gt, mode, r_est, r_gt)
    p_al = s * (rot @ p_est.T).T + tr
    err = np.linalg.norm(p_al - p_gt, axis=1)
    r_al = R.from_matrix(rot) * r_est
    rot_err = np.degrees((r_al.inv() * r_gt).magnitude())
    path = float(np.linalg.norm(np.diff(p_gt, axis=0), axis=1).sum())
    out = {"poses": int(len(t)), "t_first_s": float(t[0]), "t_last_s": float(t[-1]), "path_m": path,
           "align": mode, "scale": s, "ate_rmse_m": float(np.sqrt((err ** 2).mean())), "ate_max_m": float(err.max()),
           "final_err_m": float(err[-1]), "rot_rmse_deg": float(np.sqrt((rot_err ** 2).mean())),
           "rpe10": rpe_distance(p_est, r_est, p_gt, r_gt)}
    if path > 1.0:
        out["ate_pct"] = 100 * out["ate_rmse_m"] / path
        out["final_pct"] = 100 * out["final_err_m"] / path
    if mode != "sim3":
        out["scale_sim3"] = align(p_est, p_gt, "sim3")[0]
    return out


def load_openvins(path) -> tuple[np.ndarray, np.ndarray, R]:
    """ov_estimate.txt (save_total_state): t, q (x y z w), p, ... ; q maps IMU to global (see eval_vio.py)."""
    est = np.loadtxt(path, comments="#", ndmin=2)
    return est[:, 0], est[:, 5:8], R.from_quat(est[:, 1:5])


def load_macvo(path) -> tuple[np.ndarray, np.ndarray, R]:
    """traj_est_body_world.csv: t_s,px,py,pz,qx,qy,qz,qw (body in world)."""
    est = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return est[:, 0], est[:, 1:4], R.from_quat(est[:, 4:8])
