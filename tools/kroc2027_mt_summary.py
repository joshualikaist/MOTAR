#!/usr/bin/env python3
"""Per-cell summary of an MT run folder (MT-P2-C01 canary or KROC27-A-M02 main), read-only.

    python -B tools/kroc2027_mt_summary.py --root results/mt_p1_precision_v1/MT-P2-C01 \
        --report mt_canary_report.json --out results/mt_p1_precision_v1/MT-P2-C01/run_summary.json

For each cell in the run report: verdict, wall seconds, recorded episodes, G1/G2 violations and the
largest excess over each limit (<= 0 means within it), swept-clearance and wall violations, the
smallest swept clearance, the largest target step speed, packets written and the runner's matmul
precision. ``--out`` never overwrites an existing file.
"""
import argparse
import json
from pathlib import Path
import sys


def density(key):
    return int(key.split("__")[1].replace("bars", ""))


def summarise(root, report_name):
    report = json.loads((root / report_name).read_text())
    cells = []
    for key in sorted(report["cells"], key=lambda k: (k.split("/")[0], k.split("/")[1], density(k), k)):
        entry = report["cells"][key]
        directory = root / key
        summary = json.loads((directory / "kroc2027_summary.json").read_text())
        episodes = [json.loads(line) for line in (directory / "kroc2027_episodes.jsonl").read_text().splitlines()
                    if line.strip()]
        gate, swept = summary["motion_gate"], summary["swept_clearance"]
        cells.append({
            "cell": key, "verdict": "PASS" if not entry["issues"] else "FAIL", "issues": entry["issues"],
            "seconds": entry["seconds"], "episodes_recorded": len(episodes),
            "judged_steps": swept.get("judged_steps"), "motion_gate": gate["verdict"],
            "g1_g2_violations": sum(gate["violations"].values()),
            "max_excess": {k: v for k, v in gate["max_excess"].items() if k.startswith(("g1", "g2"))},
            "swept_violations": swept["violations"], "wall_violations": swept["wall_violations"],
            "min_swept_clearance_m": swept["min_clearance_m"],
            "target_step_speed_max_mps": max((e.get("target_step_speed_max_mps") or 0.0) for e in episodes),
            "packets_written": summary["violation_packets"].get("written"),
            "matmul_precision": summary.get("matmul_precision"),
        })
    worst = {}
    for cell in cells:
        for k, v in cell["max_excess"].items():
            worst[k] = max(worst.get(k, -float("inf")), v)
    return {"run_id": report.get("run_id"), "git_commit": report.get("git_commit"), "verdict": report.get("verdict"),
            "stop_reason": report.get("stop_reason"), "cells_run": len(cells),
            "seconds_total": round(sum(c["seconds"] for c in cells), 1),
            "violations_total": {k: sum(c[k] for c in cells)
                                 for k in ("g1_g2_violations", "swept_violations", "wall_violations")},
            "min_swept_clearance_m": min(c["min_swept_clearance_m"] for c in cells),
            "target_step_speed_max_mps": max(c["target_step_speed_max_mps"] for c in cells),
            "worst_max_excess": worst, "cells": cells}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--root", required=True)
    parser.add_argument("--report", required=True, help="the run report inside --root")
    parser.add_argument("--out", help="new JSON file (default: stdout)")
    args = parser.parse_args(argv)
    out = summarise(Path(args.root), args.report)
    text = json.dumps(out, indent=1, sort_keys=True) + "\n"
    if args.out:
        with open(args.out, "x") as handle:
            handle.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
