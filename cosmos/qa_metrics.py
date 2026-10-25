"""Metrics that check whether a generated video still matches the simulated geometry and motion.

All of them use the ground truth from the render: per-frame depth (z in the camera frame) and
camera poses in the OpenCV optical frame. That ground truth is identical for every variant, so a
variant is only usable for VIO if its pixels move the way the true geometry says they should.
"""

from __future__ import annotations

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_matrix(tum_row: np.ndarray) -> np.ndarray:
    """TUM row (t x y z qx qy qz qw) -> 4x4 camera-to-world."""
    T = np.eye(4)
    T[:3, :3] = R.from_quat(tum_row[4:8]).as_matrix()
    T[:3, 3] = tum_row[1:4]
    return T


def gray(rgb: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)


def gray_standardized(rgb: np.ndarray) -> np.ndarray:
    """Zero-mean, unit-std grey image, so residuals compare across day/night/haze variants."""
    g = gray(rgb)
    return (g - g.mean()) / max(float(g.std()), 1e-3)


def equalized_gray(rgb: np.ndarray) -> np.ndarray:
    return cv2.equalizeHist(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))


def warp_residual(rgb_a, rgb_b, depth_a, depth_b, T_wa, T_wb, K, occlusion_rel=0.02):
    """Photometric residual of frame a warped into frame b with ground-truth depth and poses.

    Returns (per-pixel |I_b(warp(x)) - I_a(x)| in frame a's grid, valid mask), on standardized grey
    images so the value does not scale with brightness or contrast. Pixels without
    depth, falling outside frame b, or occluded in b (depth disagreement) are invalid.
    """
    h, w = depth_a.shape
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    z = depth_a.astype(np.float64)
    ok = np.isfinite(z) & (z > 0)
    x = (u + 0.5 - K[0, 2]) / K[0, 0] * z
    y = (v + 0.5 - K[1, 2]) / K[1, 1] * z
    pa = np.stack([x, y, z, np.ones_like(z)], axis=-1)
    T_ba = np.linalg.inv(T_wb) @ T_wa
    pb = pa @ T_ba.T
    zb = pb[..., 2]
    ok &= zb > 0
    ub = K[0, 0] * pb[..., 0] / np.where(zb > 0, zb, 1) + K[0, 2] - 0.5
    vb = K[1, 1] * pb[..., 1] / np.where(zb > 0, zb, 1) + K[1, 2] - 0.5
    ok &= (ub >= 0) & (ub <= w - 1) & (vb >= 0) & (vb <= h - 1)
    mx, my = ub.astype(np.float32), vb.astype(np.float32)
    db = cv2.remap(np.nan_to_num(depth_b.astype(np.float32), posinf=0), mx, my, cv2.INTER_NEAREST)
    ok &= np.abs(db - zb) < occlusion_rel * zb
    ib = cv2.remap(gray_standardized(rgb_b), mx, my, cv2.INTER_LINEAR)
    res = np.abs(ib - gray_standardized(rgb_a))
    return res, ok


def warp_error_sequence(frames, depths, poses, K, step=1, common_masks=None):
    """Mean warp residual per consecutive pair. Returns (errors, masks) so another video can be
    scored on exactly the same pixels via common_masks."""
    errs, masks = [], []
    for k, i in enumerate(range(0, len(frames) - step, step)):
        res, ok = warp_residual(frames[i], frames[i + step], depths[i], depths[i + step],
                                pose_matrix(poses[i]), pose_matrix(poses[i + step]), K)
        if common_masks is not None:
            ok = ok & common_masks[k]
        errs.append(float(res[ok].mean()) if ok.any() else float("nan"))
        masks.append(ok)
    return np.array(errs), masks


def edge_f_score(rgb_out, rgb_ref, canny=(60, 160), tol_px=2) -> float:
    """F-score of output edges against reference edges, allowing tol_px of misplacement.
    Both images are histogram-equalized first so dark or hazy variants are not penalised for contrast."""
    e_out = cv2.Canny(equalized_gray(rgb_out), *canny) > 0
    e_ref = cv2.Canny(equalized_gray(rgb_ref), *canny) > 0
    if not e_out.any() or not e_ref.any():
        return float("nan")
    dt_ref = cv2.distanceTransform((~e_ref).astype(np.uint8), cv2.DIST_L2, 3)
    dt_out = cv2.distanceTransform((~e_out).astype(np.uint8), cv2.DIST_L2, 3)
    precision = float((dt_ref[e_out] <= tol_px).mean())
    recall = float((dt_out[e_ref] <= tol_px).mean())
    return 0.0 if precision + recall == 0 else 2 * precision * recall / (precision + recall)


def klt_tracks(frames, max_corners=400, min_dist=8, redetect_every=5):
    """Track corners through the sequence. Returns list of tracks, each a list of (frame, u, v)."""
    lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    grays = [cv2.cvtColor(f, cv2.COLOR_RGB2GRAY) for f in frames]
    tracks, active = [], []
    for i, g in enumerate(grays):
        if i > 0 and active:
            p0 = np.float32([t[-1][1:] for t in active]).reshape(-1, 1, 2)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(grays[i - 1], g, p0, None, **lk)
            back, st2, _ = cv2.calcOpticalFlowPyrLK(g, grays[i - 1], p1, None, **lk)
            good = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(back - p0, axis=2).ravel() < 1.0)
            keep = []
            for t, ok, p in zip(active, good, p1.reshape(-1, 2)):
                if ok:
                    t.append((i, float(p[0]), float(p[1])))
                    keep.append(t)
            active = keep
        if i % redetect_every == 0:
            mask = np.full(g.shape, 255, np.uint8)
            for t in active:
                cv2.circle(mask, (int(t[-1][1]), int(t[-1][2])), min_dist, 0, -1)
            pts = cv2.goodFeaturesToTrack(g, max_corners - len(active), 0.01, min_dist, mask=mask)
            if pts is not None:
                for p in pts.reshape(-1, 2):
                    t = [(i, float(p[0]), float(p[1]))]
                    tracks.append(t)
                    active.append(t)
    return tracks


def track_reprojection(tracks, poses, K, min_len=3):
    """Triangulate each track with ground-truth poses (linear DLT) and return per-track median
    reprojection error in pixels plus track lengths."""
    P = {}
    errs, lengths = [], []
    for t in tracks:
        if len(t) < min_len:
            continue
        rows = []
        for f, u, v in t:
            if f not in P:
                P[f] = K @ np.linalg.inv(pose_matrix(poses[f]))[:3]
            M = P[f]
            # +0.5 shifts to the pixel-centre convention used by the depth back-projection.
            rows.append((u + 0.5) * M[2] - M[0])
            rows.append((v + 0.5) * M[2] - M[1])
        _, _, vt = np.linalg.svd(np.array(rows))
        X = vt[-1]
        if abs(X[3]) < 1e-12:
            continue
        X = X / X[3]
        e = []
        for f, u, v in t:
            x = P[f] @ X
            if x[2] <= 0:
                e = None
                break
            e.append(np.hypot(x[0] / x[2] - 0.5 - u, x[1] / x[2] - 0.5 - v))
        if e is None:
            continue
        errs.append(float(np.median(e)))
        lengths.append(len(t))
    return np.array(errs), np.array(lengths)
