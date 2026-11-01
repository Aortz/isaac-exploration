"""Re-score every finished case from its saved estimates (after a metrics fix), appending fresh rows.

testplan/report.py keeps the latest row per (case, SUT, run), so re-scored rows replace the old ones.
Run: uv run python testplan/rescore.py [CASE_ID ...]
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402
from testplan.metrics import load_gt, load_macvo, load_openvins, trajectory_metrics  # noqa: E402
from testplan.run_case import append_row, verdict_vio, verdict_vo  # noqa: E402


def main() -> None:
    tcfg = load_config(PROJECT_ROOT / "config" / "tests.yaml")
    fcfg = load_config(PROJECT_ROOT / "config" / "flight.yaml")
    th = tcfg["thresholds"]
    want = set(sys.argv[1:])
    for case in tcfg["cases"]:
        out = BUILD / "tests" / case["id"]
        if (want and case["id"] not in want) or not (out / "case.json").exists():
            continue
        flight, lighting = case["flight"], case.get("lighting", "default")
        fl = fcfg["flights"][flight]
        suffix = f"_{lighting}" if lighting != "default" else ""
        graded = case["profile"] in th["graded_profiles"]
        gt = load_gt(BUILD / "flight" / flight / "groundtruth_200hz.csv")
        base = {"case": case["id"], "profile": case["profile"], "env": fl.get("env") or fl.get("site"),
                "flight": flight, "lighting": lighting}
        state = json.loads((out / "case.json").read_text())
        for run in sorted(out.glob(f"vio{suffix}_s*")):
            est = run / "ov_estimate.txt"
            if not est.exists() or sum(1 for ln in open(est) if ln.strip() and not ln.startswith("#")) < 10:
                continue
            t, p, r = load_openvins(est)
            k = (t >= gt[0][0]) & (t <= gt[0][-1])
            m = trajectory_metrics(t[k], p[k], r[k], gt, "4dof")
            v, why = verdict_vio(m, th["vio"], graded)
            seed = run.name.rsplit("_s", 1)[1]
            label = f"s{seed}" + ("_live" if seed == str(tcfg["vio_seeds"][0]) else "_replay")
            append_row({**base, "sut": "openvins", "run": label, "status": "ok", **m,
                        "rpe10_pct": m["rpe10"].get("rmse_pct"), "verdict": v, "notes": (why + " (rescored)").strip()})
            state.setdefault("vio", {})[seed] = m
        traj = out / f"macvo{suffix}" / "traj_est_body_world.csv"
        if traj.exists():
            t, p, r = load_macvo(traj)
            m = trajectory_metrics(t, p, r, gt, "se3")
            v, why = verdict_vo(m, th["vo"], graded)
            if not state.get("g4", {}).get("pass", True):
                why = (why + ",G4").strip(",")
            append_row({**base, "sut": "macvo", "run": "fast", "status": "ok", **m,
                        "rpe10_pct": m["rpe10"].get("rmse_pct"), "verdict": v, "notes": (why + " (rescored)").strip()})
            state["macvo"] = m
        (out / "case.json").write_text(json.dumps(state, indent=2, default=float) + "\n")
        print(f"rescored {case['id']}")


if __name__ == "__main__":
    main()
