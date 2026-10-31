"""Build build/tests/report.md (+ report.png) from results.csv and each case's case.json.

Run: uv run python testplan/report.py
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline.common.config import BUILD, PROJECT_ROOT, load_config  # noqa: E402

T = BUILD / "tests"


def f(v, nd=2):
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return "–"


def main() -> None:
    tcfg = load_config(PROJECT_ROOT / "config" / "tests.yaml")
    th = tcfg["thresholds"]
    rows = list(csv.DictReader(open(T / "results.csv"))) if (T / "results.csv").exists() else []
    # Latest row per (case, sut, run) wins (re-runs append).
    latest = {}
    for r in rows:
        latest[(r["case"], r["sut"], r["run"])] = r
    rows = list(latest.values())
    cases = [c for c in tcfg["cases"] if (T / c["id"] / "case.json").exists()]
    lines = ["# Test results", "",
             f"Thresholds (config/tests.yaml): VIO ATE ≤ {th['vio']['ate_pct']} %, drift ≤ {th['vio']['drift_pct']} %, "
             f"rot ≤ {th['vio']['rot_rmse_deg']}°, init ≤ {th['vio']['init_s']} s, seed spread ≤ {th['vio']['seed_spread']}; "
             f"VO ATE ≤ {th['vo']['ate_pct']} %, drift ≤ {th['vo']['drift_pct']} %, rot ≤ {th['vo']['rot_rmse_deg']}°, "
             f"|scale−1| ≤ {th['vo']['scale_err']}. Graded profiles: {', '.join(th['graded_profiles'])}; others report only.", ""]

    lines += ["## Gates", "", "| Case | G3 flight (track RMS / IMU drift / clearance) | G1 fisheye px | G1 stereo px | G4 disparity px |",
              "|---|---|---|---|---|"]
    for c in cases:
        s = json.loads((T / c["id"] / "case.json").read_text())
        g3 = s.get("g3", {})
        clr = g3.get("clearance", {}).get("min_clearance_m")
        mark = lambda g: "" if g is None else (" ✅" if g.get("pass") else " ❌")  # noqa: E731
        lines.append(f"| {c['id']} | {f(g3.get('track_rms_m'))} m / {f(g3.get('imu_drift_m'))} m / {f(clr, 1)} m{mark(g3 or None)} "
                     f"| {f(s.get('g1_fisheye', {}).get('median_px'), 3)}{mark(s.get('g1_fisheye'))} "
                     f"| {f(s.get('g1_stereo', {}).get('median_px'), 3)}{mark(s.get('g1_stereo'))} "
                     f"| {f(s.get('g4', {}).get('median_px'))}{mark(s.get('g4'))} |")

    lines += ["", "## Estimator runs", "",
              "| Case | Profile | SUT | Run | Path m | ATE m | ATE % | Final % | Rot ° | Scale | RPE10 % | First pose s | Verdict | Notes |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    order = {c["id"]: i for i, c in enumerate(tcfg["cases"])}
    for r in sorted(rows, key=lambda r: (order.get(r["case"], 99), r["sut"], r["run"])):
        lines.append(f"| {r['case']} | {r['profile']} | {r['sut']} | {r['run']} | {f(r['path_m'], 0)} | {f(r['ate_rmse_m'])} | "
                     f"{f(r['ate_pct'])} | {f(r['final_pct'])} | {f(r['rot_rmse_deg'])} | {f(r['scale_sim3'], 3)} | "
                     f"{f(r['rpe10_pct'], 1)} | {f(r['t_first_s'], 1)} | {r['verdict']} | {r['notes']} |")

    lines += ["", "## Run-to-run spread (OpenVINS seeds)", "", "| Case | ATEs m | std/mean | Pass |", "|---|---|---|---|"]
    for c in cases:
        s = json.loads((T / c["id"] / "case.json").read_text()).get("vio_seed_spread")
        if s:
            lines.append(f"| {c['id']} | {', '.join(f(a) for a in s['ates'])} | {f(s['std_over_mean'], 3)} | "
                         f"{'✅' if s['pass'] else '❌'} |")

    # P7: lighting variants vs the default-lighting P3 case.
    ref = {r["sut"]: float(r["ate_rmse_m"]) for r in rows if r["case"] == "TC-E1-P3" and r["ate_rmse_m"] and r["run"] in ("s7_live", "fast")}
    p7 = [r for r in rows if r["profile"] == "P7" and r["ate_rmse_m"] and r["run"] in ("s7_live", "fast")]
    if ref and p7:
        lines += ["", "## Lighting (P7) vs default lighting (TC-E1-P3)", "", "| Case | SUT | ATE m | ratio | Pass (≤ "
                  f"{th['p7_ratio']}×) |", "|---|---|---|---|---|"]
        for r in p7:
            if r["sut"] in ref:
                ratio = float(r["ate_rmse_m"]) / ref[r["sut"]]
                lines.append(f"| {r['case']} | {r['sut']} | {f(r['ate_rmse_m'])} | {f(ratio)} | {'✅' if ratio <= th['p7_ratio'] else '❌'} |")

    (T / "report.md").write_text("\n".join(lines) + "\n")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 4))
    names = [c["id"] for c in cases]
    for k, (sut, run_filter, col) in enumerate((("openvins", "s", "C0"), ("macvo", "fast", "C1"))):
        vals = []
        for n in names:
            v = [float(r["ate_pct"]) for r in rows if r["case"] == n and r["sut"] == sut and r["ate_pct"]]
            vals.append(np.mean(v) if v else np.nan)
        ax.bar(np.arange(len(names)) + (k - 0.5) * 0.38, vals, 0.38, label=f"{sut} (mean over runs)", color=col)
    ax.axhline(th["vio"]["ate_pct"], color="C0", ls="--", lw=1, label="VIO limit")
    ax.axhline(th["vo"]["ate_pct"], color="C1", ls="--", lw=1, label="VO limit")
    ax.set_xticks(np.arange(len(names)), names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("ATE RMSE [% of path]")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(T / "report.png", dpi=110)
    print(f"report -> {T / 'report.md'}")


if __name__ == "__main__":
    main()
