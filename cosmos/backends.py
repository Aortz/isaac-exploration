"""Cosmos 3 transfer backends behind one interface.

  nvidia_hosted  build.nvidia.com hosted Cosmos3-Nano (`model_mode: video2video` + `transfer`).
                 Request shape follows the build.nvidia.com page (2026-10): control videos are
                 base64 data URIs in transfer.<control>.video, the result is `b64_video` (MP4/VP9).
                 Anything beyond the documented fields goes through cosmos.extra_fields.
  vllm_omni      self-hosted Cosmos3-Nano-Sim-Transfer via vLLM-Omni /v1/videos/sync (not wired yet).
  dry_run        no model: colour-grades rgb.mp4 locally. Lets F5/F6 be exercised end to end; its
                 outputs are tagged synthetic and must never be used as data.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class TransferJob:
    clip_dir: Path
    condition: str
    prompt: dict
    seed: int
    num_frames: int
    fps: int
    controls: list[str]


def _data_uri(path: Path) -> str:
    return "data:video/mp4;base64," + base64.b64encode(path.read_bytes()).decode()


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


class CosmosBackend:
    name = "base"

    def __init__(self, cfg: dict):
        self.cfg = cfg

    def cache_key(self, job: TransferJob) -> str:
        """Identifies a request by content, so re-running never pays for the same generation twice."""
        ident = {"backend": self.name, "endpoint": self.cfg.get("endpoint"), "res": self.cfg.get("resolution"),
                 "extra": self.cfg.get("extra_fields", {}), "prompt": job.prompt, "seed": job.seed,
                 "frames": job.num_frames, "fps": job.fps,
                 "controls": {c: _sha(job.clip_dir / f"{c}.mp4") for c in job.controls},
                 "rgb": _sha(job.clip_dir / "rgb.mp4")}
        if self.name == "dry_run":
            ident["noise"] = os.environ.get("COSMOS_DRY_RUN_NOISE", "0")
        return hashlib.sha256(json.dumps(ident, sort_keys=True).encode()).hexdigest()[:24]

    def transfer(self, job: TransferJob, out_path: Path) -> dict:
        raise NotImplementedError


class NvidiaHostedBackend(CosmosBackend):
    name = "nvidia_hosted"
    NVCF_STATUS = "https://api.nvcf.nvidia.com/v2/nvcf/pexec/status/{}"

    def __init__(self, cfg: dict):
        super().__init__(cfg)
        self.key = os.environ.get("NVIDIA_API_KEY", "")
        if not self.key:
            raise RuntimeError("NVIDIA_API_KEY is not set (create one at build.nvidia.com)")

    def request_body(self, job: TransferJob) -> dict:
        transfer = {}
        for c in job.controls:
            transfer[c] = {"video": _data_uri(job.clip_dir / f"{c}.mp4")}
        body = {
            "model_mode": "video2video",
            "prompt": json.dumps(job.prompt),
            "resolution": self.cfg["resolution"],
            "num_frames": job.num_frames,
            "fps": job.fps,
            "seed": job.seed,
            "input_reference": _data_uri(job.clip_dir / "rgb.mp4"),
            "transfer": transfer,
        }
        body.update(self.cfg.get("extra_fields", {}))
        return body

    def transfer(self, job: TransferJob, out_path: Path) -> dict:
        import requests

        headers = {"Authorization": f"Bearer {self.key}", "Accept": "application/json",
                   "Content-Type": "application/json"}
        timeout = self.cfg.get("timeout_s", 900)
        t0 = time.time()
        resp = None
        for attempt in range(5):
            resp = requests.post(self.cfg["endpoint"], headers=headers, json=self.request_body(job), timeout=timeout)
            if resp.status_code in (429, 500, 502, 503, 504):
                time.sleep(min(60, 5 * 2 ** attempt))
                continue
            break
        req_id = resp.headers.get("NVCF-REQID")
        # NVCF-style async: 202 + request id, poll until the result is ready.
        while resp.status_code == 202 and req_id and time.time() - t0 < timeout:
            time.sleep(5)
            resp = requests.get(self.NVCF_STATUS.format(req_id), headers=headers, timeout=60)
        if resp.status_code != 200:
            raise RuntimeError(f"Cosmos API {resp.status_code} (request id {req_id}): {resp.text[:2000]}")
        data = resp.json()
        video_b64 = data.get("b64_video") or data.get("video")
        if not video_b64:
            raise RuntimeError(f"no b64_video in response; keys: {sorted(data)} (request id {req_id})")
        out_path.write_bytes(base64.b64decode(video_b64, validate=True))
        return {"request_id": req_id, "seconds": round(time.time() - t0, 1),
                "response_fields": sorted(k for k in data if k != "b64_video"), "synthetic": False}


class VllmOmniBackend(CosmosBackend):
    name = "vllm_omni"

    def transfer(self, job: TransferJob, out_path: Path) -> dict:
        raise NotImplementedError(
            "Self-hosted path: serve nvidia/Cosmos3-Nano-Sim-Transfer with vLLM-Omni "
            "(docker vllm/vllm-omni:cosmos3, --model-class-name Cosmos3NanoSimTransferPipeline) and POST to "
            "/v1/videos/sync. Wire this up when an RTX PRO 6000 / H100 host is available.")


class DryRunBackend(CosmosBackend):
    """Per-condition colour grade (no noise, no geometry change). Exercises the pipeline only; not a
    model. Set COSMOS_DRY_RUN_NOISE=<sigma> to add per-frame flicker and watch F5 reject it."""

    name = "dry_run"
    GRADES = {"clear_noon": (1.05, 1.0, 0.95, 1.1), "overcast": (0.85, 0.9, 0.95, 0.7),
              "haze": (1.0, 0.95, 0.8, 0.6), "tropical_rain": (0.6, 0.65, 0.7, 0.8),
              "dusk_low_sun": (1.2, 0.85, 0.55, 1.0), "night_city": (0.25, 0.25, 0.35, 1.3),
              "morning_mist": (0.9, 0.95, 1.05, 0.65)}

    def transfer(self, job: TransferJob, out_path: Path) -> dict:
        from cosmos.video_io import read_video, write_video

        frames = read_video(job.clip_dir / "rgb.mp4")
        r, g, b, contrast = self.GRADES.get(job.condition, (1, 1, 1, 1))
        rng = np.random.default_rng(job.seed)
        noise = float(os.environ.get("COSMOS_DRY_RUN_NOISE", "0"))
        out = []
        for f in frames:
            x = f.astype(np.float32)
            x = (x - 128) * contrast + 128
            x *= np.array([r, g, b], np.float32)
            if noise:
                x += rng.normal(0, noise, x.shape)
            out.append(np.clip(x, 0, 255).astype(np.uint8))
        write_video(out, out_path, job.fps)
        return {"request_id": None, "seconds": 0.0, "synthetic": True, "dry_run_noise": noise}


def make_backend(cfg: dict) -> CosmosBackend:
    kind = cfg["backend"]
    return {"nvidia_hosted": NvidiaHostedBackend, "vllm_omni": VllmOmniBackend, "dry_run": DryRunBackend}[kind](cfg)
