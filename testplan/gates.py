"""Image gates of the test plan, run on a rig render before any estimator sees it.

g1  KLT tracks against the ground-truth poses: features tracked k -> k+d -> k+2d are triangulated from
    k and k+2d with the GT camera poses and reprojected into k+d; the median pixel error says whether
    images, intrinsics, extrinsics and poses agree. With too little translation (hover), the check is
    rotation-only: the tracked bearing must follow the GT rotation (cameras sit near the IMU origin).
g4  median horizontal disparity of the stereo pair (is the stereo baseline meaningful for this scene?).

Run: uv run python testplan/gates.py g1 --flight rm_p3_nominal --run rig_lockstep [--stereo]
     uv run python testplan/gates.py g4 --flight rm_p3_nominal --run rig_lockstep_stereo
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flight.f1_trajectory import body_to_camera_cv  # noqa: E402
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402
from testplan.metrics import gt_at, load_gt  # noqa: E402


class Cam:
    def __init__(self, model: str, f: float, cx: float, cy: float, r_bc: np.ndarray, t_bc: np.ndarray):
        self.model, self.f, self.c, self.r_bc, self.t_bc = model, f, np.array([cx, cy]), r_bc, t_bc

    def unproject(self, px: np.ndarray) -> np.ndarray:
        d = (px - self.c) / self.f
        if self.model == "pinhole":
            v = np.column_stack([d, np.ones(len(d))])
        else:                                   # equidistant: r = f * theta
            th = np.linalg.norm(d, axis=1)
            s = np.where(th > 1e-9, np.sin(th) / np.maximum(th, 1e-9), 1.0)
            v = np.column_stack([d * s[:, None], np.cos(th)])
        return v / np.linalg.norm(v, axis=1, keepdims=True)

    def project(self, v: np.ndarray) -> np.ndarray:
        if self.model == "pinhole":
            return self.f * v[:, :2] / v[:, 2:3] + self.c
        r = np.linalg.norm(v[:, :2], axis=1)
        th = np.arctan2(r, v[:, 2])
        return self.f * v[:, :2] * (th / np.maximum(r, 1e-12))[:, None] + self.c


def rig_cameras(cfg: dict, run: Path, stereo: bool) -> dict:
    if stereo:
        st = json.loads((run / "stereo.json").read_text())
        t = np.array(st["T_body_left"])
        return {"left": Cam("pinhole", st["fx"], st["cx"], st["cy"], t[:3, :3], t[:3, 3])}
    rig = cfg["rig"]
    cams = {}
    for name, (yaw, pitch) in rig["cameras"].items():
        r_bc = R.from_euler("z", yaw, degrees=True).as_matrix() @ body_to_camera_cv(pitch)
        cams[name] = Cam("equidistant", rig["fx"], rig["width"] / 2, rig["height"] / 2, r_bc, np.zeros(3))
    return cams


def gray(path: Path) -> np.ndarray:
    return cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2GRAY)


def track(a, b, pts):
    fwd, st1, _ = cv2.calcOpticalFlowPyrLK(a, b, pts, None, winSize=(21, 21), maxLevel=3)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, fwd, None, winSize=(21, 21), maxLevel=3)
    ok = (st1[:, 0] == 1) & (st2[:, 0] == 1) & (np.linalg.norm(back - pts, axis=2)[:, 0] < 0.1)
    return fwd, ok


def g1(flight: str, run_name: str, stereo: bool, every: int = 45, min_base_m: float = 1.0) -> dict:
    cfg = load_config(PROJECT_ROOT / "config" / "flight.yaml")
    hz = cfg["rig"]["camera_hz"]
    run = BUILD / "flight" / flight / run_name
    gt = load_gt(BUILD / "flight" / flight / "groundtruth_200hz.csv")
    cams = rig_cameras(cfg, run, stereo)
    n_frames = len(list((run / "frames").glob(f"*_{next(iter(cams))}.png")))
    t_all = np.arange(n_frames) / hz
    p_all, r_all = gt_at(t_all, gt)
    errs, modes = {c: [] for c in cams}, {"triangulated": 0, "rotation_only": 0}
    for k in range(0, n_frames - 8, every):
        # Frame gap d giving >= min_base_m over 2d frames, else rotation-only with d = 3.
        d = 3
        while k + 2 * d < n_frames and np.linalg.norm(p_all[k + 2 * d] - p_all[k]) < min_base_m and d < 30:
            d += 1
        tri = k + 2 * d < n_frames and np.linalg.norm(p_all[k + 2 * d] - p_all[k]) >= min_base_m
        if not tri:
            d = 3
        modes["triangulated" if tri else "rotation_only"] += 1
        for name, cam in cams.items():
            f = [run / "frames" / f"{k + i * d:06d}_{name}.png" for i in range(3)]
            if not all(x.exists() for x in f):
                continue
            ims = [gray(x) for x in f]
            mask = np.zeros_like(ims[0])
            if cam.model == "equidistant":
                cv2.circle(mask, tuple(int(v) for v in cam.c), int(cam.f * np.radians(75)), 255, -1)
            else:
                mask[:] = 255
            p0 = cv2.goodFeaturesToTrack(ims[0], 300, 0.01, 10, mask=mask)
            if p0 is None:
                continue
            p1, ok1 = track(ims[0], ims[1], p0)
            p2, ok2 = track(ims[1], ims[2], p1)
            ok = ok1 & ok2
            if ok.sum() < 10:
                continue
            u0, u1, u2 = (x[ok, 0] for x in (p0, p1, p2))
            # Camera poses (world from camera) at the three frames.
            rc = [r_all[k + i * d].as_matrix() @ cam.r_bc for i in range(3)]
            pc = [p_all[k + i * d] + r_all[k + i * d].apply(cam.t_bc) for i in range(3)]
            b0, b2 = cam.unproject(u0) @ rc[0].T, cam.unproject(u2) @ rc[2].T      # world-frame rays
            if tri:
                # Midpoint triangulation of rays from pc[0] and pc[2].
                w = pc[0] - pc[2]
                a, b, c = (b0 * b0).sum(1), (b0 * b2).sum(1), (b2 * b2).sum(1)
                dd, e = (b0 * w).sum(1), (b2 * w).sum(1)
                den = a * c - b * b
                good = den > 1e-8
                s0 = (b * e - c * dd) / np.where(good, den, 1)
                s2 = (a * e - b * dd) / np.where(good, den, 1)
                X = 0.5 * (pc[0] + s0[:, None] * b0 + pc[2] + s2[:, None] * b2)
                ang = np.degrees(np.arccos(np.clip((b0 * b2).sum(1), -1, 1)))
                good &= (s0 > 0) & (s2 > 0) & (ang > 0.3)
                v1 = (X - pc[1]) @ rc[1]
            else:
                good = np.ones(len(u0), bool)
                v1 = b0 @ rc[1]
            good &= v1[:, 2] > 0.05 if cam.model == "pinhole" else np.ones(len(v1), bool)
            if good.sum() < 5:
                continue
            e1 = np.linalg.norm(cam.project(v1[good]) - u1[good], axis=1)
            errs[name] += e1.tolist()
    allv = np.concatenate([np.asarray(v) for v in errs.values()]) if any(errs.values()) else np.array([])
    return {"gate": "g1", "flight": flight, "run": run_name, "frames": n_frames, **modes,
            "tracks": int(len(allv)), "median_px": float(np.median(allv)) if len(allv) else None,
            "p90_px": float(np.percentile(allv, 90)) if len(allv) else None,
            "per_camera_median_px": {c: float(np.median(v)) if v else None for c, v in errs.items()}}


def g4(flight: str, run_name: str, every: int = 30) -> dict:
    run = BUILD / "flight" / flight / run_name
    lefts = sorted((run / "frames").glob("*_left.png"))
    disp = []
    for lp in lefts[::every]:
        rp = lp.with_name(lp.name.replace("_left", "_right"))
        a, b = gray(lp), gray(rp)
        p = cv2.goodFeaturesToTrack(a, 400, 0.01, 8)
        if p is None:
            continue
        q, ok = track(a, b, p)
        d = (p - q)[ok, 0]
        d = d[np.abs(d[:, 1]) < 1.0]
        disp += d[:, 0].tolist()
    d = np.asarray(disp)
    return {"gate": "g4", "flight": flight, "run": run_name, "samples": int(len(d)),
            "median_px": float(np.median(d)) if len(d) else None,
            "p10_px": float(np.percentile(d, 10)) if len(d) else None,
            "p90_px": float(np.percentile(d, 90)) if len(d) else None}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("gate", choices=["g1", "g4"])
    ap.add_argument("--flight", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--stereo", action="store_true")
    a = ap.parse_args()
    r = g1(a.flight, a.run, a.stereo) if a.gate == "g1" else g4(a.flight, a.run)
    print(json.dumps(r))


if __name__ == "__main__":
    main()
