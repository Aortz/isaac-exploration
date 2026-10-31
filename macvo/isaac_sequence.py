"""
MAC-VO sequence type for stereo sequences rendered from Isaac Sim (isaac-exploration format).

Importing this module (after MAC-VO's root is on sys.path) registers the sequence type
`IsaacStereo` in MAC-VO's SequenceBase registry, so a data config like

    type: IsaacStereo
    name: my_seq
    args:
      root: /path/to/seq_dir
      gt_pose: true          # use groundtruth_left.csv if present

works with `SequenceBase[StereoFrame].instantiate(cfg.type, cfg.args)`.

On-disk format (<seq_dir>):
  stereo.json            {"width","height","fx","fy","cx","cy","baseline_m","camera_hz",
                          "T_body_left": 4x4 row-major (left cam -> body, OpenCV cam),
                          "T_left_right": 4x4 (right cam -> left cam, OpenCV cam)}
  frames/NNNNNN_left.png, NNNNNN_right.png   8-bit RGB, rectified pinhole, right at +x of left
  times.csv              frame,t_s
  groundtruth_left.csv   t_s,px,py,pz,qx,qy,qz,qw   (LEFT camera, OpenCV, camera->world, ENU m)

Frame conventions inside MAC-VO:
  * MAC-VO tracks the camera in an NED-style camera frame (x fwd, y right, z down) and
    reports *body* poses as  T_BS @ P_sensor @ T_BS^-1  with T_BS = sensor(NED cam) -> body.
    We therefore set  T_BS = T_body_left @ T_cvcam_nedcam  (same construction as EuRoC.py).
  * gt_pose given to MAC-VO is the BODY pose in world: T_world_body = T_world_left @ T_body_left^-1,
    so MAC-VO's own evaluation (Evaluation.EvalSeq, which origin-aligns est->gt) compares
    body trajectories in world ENU.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
import pypose as pp
import torch
from scipy.spatial.transform import Rotation, Slerp

from DataLoader import StereoData, StereoFrame, SequenceBase
from Utility.PrettyPrint import Logger

# Maps NED-camera coordinates -> OpenCV(EDN)-camera coordinates (as a 4x4 transform).
T_CV_NED = np.array([
    [0., 1., 0., 0.],
    [0., 0., 1., 0.],
    [1., 0., 0., 0.],
    [0., 0., 0., 1.],
])


def _se3_from_matrix(T: np.ndarray) -> np.ndarray:
    """4x4 (or Nx4x4) -> pypose layout [tx,ty,tz,qx,qy,qz,qw] (N x 7)."""
    T = np.asarray(T, dtype=np.float64).reshape(-1, 4, 4)
    q = Rotation.from_matrix(T[:, :3, :3]).as_quat()  # xyzw
    return np.concatenate([T[:, :3, 3], q], axis=1)


def _matrix_from_se3(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64).reshape(-1, 7)
    T = np.tile(np.eye(4), (v.shape[0], 1, 1))
    T[:, :3, :3] = Rotation.from_quat(v[:, 3:7]).as_matrix()
    T[:, :3, 3] = v[:, :3]
    return T


def load_stereo_meta(seq_dir: Path) -> dict[str, Any]:
    with open(Path(seq_dir, "stereo.json")) as f:
        meta = json.load(f)
    meta["T_body_left"] = np.array(meta.get("T_body_left", np.eye(4)), dtype=np.float64).reshape(4, 4)
    meta["T_left_right"] = np.array(meta.get("T_left_right", np.eye(4)), dtype=np.float64).reshape(4, 4)
    return meta


def load_times(seq_dir: Path, camera_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """Returns (frame_ids[int], t_s[float64]). Falls back to globbing frames/ at camera_hz."""
    times_csv = Path(seq_dir, "times.csv")
    if times_csv.exists():
        frames, ts = [], []
        with open(times_csv) as f:
            for row in csv.DictReader(f):
                frames.append(int(row["frame"]))
                ts.append(float(row["t_s"]))
        return np.array(frames, dtype=np.int64), np.array(ts, dtype=np.float64)
    ids = sorted(int(p.name.split("_")[0]) for p in Path(seq_dir, "frames").glob("*_left.png"))
    Logger.write("warn", f"{times_csv} missing; assuming {camera_hz} Hz from frame ids")
    return np.array(ids, dtype=np.int64), np.array(ids, dtype=np.float64) / float(camera_hz)


def load_groundtruth_left(seq_dir: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """Returns (t_s [N], T_world_left [N,4,4]) or None."""
    gt_csv = Path(seq_dir, "groundtruth_left.csv")
    if not gt_csv.exists():
        return None
    data = np.loadtxt(gt_csv, delimiter=",", skiprows=1, ndmin=2)
    order = np.argsort(data[:, 0])
    data = data[order]
    return data[:, 0], _matrix_from_se3(data[:, 1:8])


def interpolate_poses(t_src: np.ndarray, T_src: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """Linear translation + SLERP rotation; clamps outside [t_src[0], t_src[-1]]."""
    if len(t_src) == 1:
        return np.repeat(T_src, len(t_query), axis=0)
    tq = np.clip(t_query, t_src[0], t_src[-1])
    out = np.tile(np.eye(4), (len(tq), 1, 1))
    for i in range(3):
        out[:, i, 3] = np.interp(tq, t_src, T_src[:, i, 3])
    out[:, :3, :3] = Slerp(t_src, Rotation.from_matrix(T_src[:, :3, :3]))(tq).as_matrix()
    return out


class IsaacStereoSequence(SequenceBase[StereoFrame]):
    @classmethod
    def name(cls) -> str: return "IsaacStereo"

    def __init__(self, config: SimpleNamespace | dict[str, Any]) -> None:
        cfg = self.config_dict2ns(config)
        self.seq_dir = Path(cfg.root)
        meta = load_stereo_meta(self.seq_dir)
        self.meta = meta
        self.width, self.height = int(meta["width"]), int(meta["height"])

        # Baseline: stereo.json value, sanity-checked against T_left_right.
        self.baseline = float(meta["baseline_m"])
        t_lr = meta["T_left_right"][:3, 3]
        if np.linalg.norm(t_lr) > 0 and abs(np.linalg.norm(t_lr) - self.baseline) > 1e-3:
            Logger.write("warn", f"baseline_m={self.baseline} disagrees with |T_left_right.t|={np.linalg.norm(t_lr):.4f}")
        if np.linalg.norm(t_lr) > 0 and t_lr[0] <= 0:
            Logger.write("warn", f"T_left_right translation {t_lr} is not +x; MAC-VO expects right camera at +x of left")

        self.K = torch.tensor([[
            [float(meta["fx"]), 0., float(meta["cx"])],
            [0., float(meta["fy"]), float(meta["cy"])],
            [0., 0., 1.],
        ]], dtype=torch.float32)

        self.T_body_left = meta["T_body_left"]
        T_BS = self.T_body_left @ T_CV_NED   # NED sensor frame -> body
        self.T_BS = pp.SE3(torch.tensor(_se3_from_matrix(T_BS), dtype=torch.float32))

        self.frame_ids, self.t_s = load_times(self.seq_dir, float(meta.get("camera_hz", 30)))
        self.frame_dir = Path(self.seq_dir, "frames")
        missing = [i for i in self.frame_ids if not Path(self.frame_dir, f"{i:06d}_left.png").exists()]
        assert not missing, f"{len(missing)} frames listed in times.csv are missing, e.g. {missing[:3]}"

        self.gt_pose: pp.LieTensor | None = None
        if getattr(cfg, "gt_pose", True):
            gt = load_groundtruth_left(self.seq_dir)
            if gt is None:
                Logger.write("warn", f"No groundtruth_left.csv in {self.seq_dir}; running without GT")
            else:
                t_gt, T_w_left = gt
                if self.t_s[0] < t_gt[0] - 1e-6 or self.t_s[-1] > t_gt[-1] + 1e-6:
                    Logger.write("warn", "Frame times extend beyond GT time range; GT is clamped at the ends")
                T_w_left_f = interpolate_poses(t_gt, T_w_left, self.t_s)
                T_w_body = T_w_left_f @ np.linalg.inv(self.T_body_left)
                self.gt_pose = pp.SE3(torch.tensor(_se3_from_matrix(T_w_body), dtype=torch.float32))

        super().__init__(len(self.frame_ids))

    def _load(self, path: Path) -> torch.Tensor:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"Failed to read image {path}")
        if image.shape[0] != self.height or image.shape[1] != self.width:
            raise ValueError(f"{path} is {image.shape[1]}x{image.shape[0]}, stereo.json says {self.width}x{self.height}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0).float().div_(255.)

    def __getitem__(self, local_index: int) -> StereoFrame:
        index = self.get_index(local_index)
        fid = int(self.frame_ids[index])
        t_ns = int(round(self.t_s[index] * 1e9))
        return StereoFrame(
            idx=[local_index],
            time_ns=[t_ns],
            stereo=StereoData(
                T_BS=self.T_BS,
                K=self.K,
                baseline=torch.tensor([self.baseline]),
                width=self.width,
                height=self.height,
                time_ns=[t_ns],
                imageL=self._load(Path(self.frame_dir, f"{fid:06d}_left.png")),
                imageR=self._load(Path(self.frame_dir, f"{fid:06d}_right.png")),
            ),
            gt_pose=None if self.gt_pose is None else pp.SE3(self.gt_pose[index:index + 1]),
        )

    @classmethod
    def is_valid_config(cls, config: SimpleNamespace | None) -> None:
        cls._enforce_config_spec(config, {
            "root": lambda s: isinstance(s, str),
            "gt_pose": lambda b: isinstance(b, bool),
        })
