#!/usr/bin/env python3
"""Compact, source-bound record of the seven-seed target-motion replication.

    python -B tools/build_tm_replication_record.py            # write the compact record
    python -B tools/build_tm_replication_record.py --check    # re-derive from raw and compare

The raw cell directories (397,619 ledger rows, 229 MB) stay out of normal Git. They are frozen by
`results/target_motion_replication_7seed/RAW_MANIFEST.json` (commit 2f723ed) and archived as a
deterministic tar.zst outside the repository. This tool derives from them only compact counts:
- `cells.csv`: one row per cell, with primary-quota and legacy-window outcome counts;
- `canary_provenance.json`: the canary gates and digests without the large observation dumps;
- `min_distance_semantic_check.json`: whether every primary capture episode has
  `min_relative_distance_m` within the capture radius.

No statistical analysis happens here. The preregistered analysis is
`tools/target_motion_replication.py analyze` at the measurement commit `b1c0bc6`.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / "results/target_motion_replication_7seed"
sys.path.insert(0, str(ROOT / "tools"))
import target_motion_replication as rep  # noqa: E402

CAPTURE_RADIUS_M = 0.5            # NavRLTaskConfig.success_radius
TOLERANCE_M = 1e-6
COLUMNS = ("cell_id", "arm", "bars", "seed", "primary_episodes", "primary_capture", "primary_crash",
           "primary_timeout", "legacy_episodes", "legacy_capture", "legacy_crash", "legacy_timeout",
           "raw_ledger_rows", "nonprimary_tail_rows", "rows_sha256", "duration_s")


def cell_rows():
    rows = []
    for arm, bars, seed in rep.cells():
        cid = rep.cell_id(arm, bars, seed)
        summary = json.loads((R / cid / "episode_ledger_summary.json").read_text())
        receipt = json.loads((R / cid / "receipt.json").read_text())
        ledger = [json.loads(x) for x in (R / cid / summary["rows_file"]).read_text().splitlines()]
        quota = [r for r in ledger if r["in_quota"]]
        legacy = [r for r in ledger if r["in_legacy_window"]]
        rows.append({"cell_id": cid, "arm": arm, "bars": bars, "seed": seed, "primary_episodes": len(quota),
                     **{"primary_" + m: sum(r[m] for r in quota) for m in rep.METRICS},
                     "legacy_episodes": len(legacy), **{"legacy_" + m: sum(r[m] for r in legacy) for m in rep.METRICS},
                     "raw_ledger_rows": len(ledger), "nonprimary_tail_rows": len(ledger) - len(quota),
                     "rows_sha256": summary["rows_sha256"], "duration_s": receipt["duration_s"]})
    return rows


def cells_csv(rows):
    out = io.StringIO()
    writer = csv.DictWriter(out, fieldnames=COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return out.getvalue()


def min_distance_check():
    checked, violations, worst, by_arm = 0, 0, 0.0, {}
    for arm, bars, seed in rep.cells():
        cid = rep.cell_id(arm, bars, seed)
        summary = json.loads((R / cid / "episode_ledger_summary.json").read_text())
        for line in (R / cid / summary["rows_file"]).read_text().splitlines():
            r = json.loads(line)
            if not (r["in_quota"] and r["capture"]):
                continue
            checked += 1
            worst = max(worst, r["min_relative_distance_m"])
            if r["min_relative_distance_m"] > CAPTURE_RADIUS_M + TOLERANCE_M:
                violations += 1
                by_arm[arm] = by_arm.get(arm, 0) + 1
    return {
        "check": "every PRIMARY capture episode has min_relative_distance_m <= capture radius + tolerance",
        "capture_radius_m": CAPTURE_RADIUS_M, "tolerance_m": TOLERANCE_M,
        "capture_episodes_checked": checked, "violations": violations, "violations_by_arm": by_arm,
        "max_captured_min_distance_m": worst,
        "verdict": "PASS" if violations == 0 else "SEMANTIC_MISMATCH_METRIC_NOT_PUBLISHED",
        "diagnosis": ("Capture uses a swept-segment test: the closest approach along the 0.1 s motion segment "
                      "must fall inside the radius (navrl_task.py, 'Swept-SEGMENT test'). min_relative_distance_m "
                      "is ep_min_goal_dist, the point distance sampled only at the end of each step, as the "
                      "preregistration defines it. A capture in mid-step can leave every end-of-step sample "
                      "slightly outside the radius. The metric is therefore an end-of-step upper bound on the "
                      "closest approach, not the closest approach, and it is not published as a validated "
                      "distance result."),
    }


def canary_provenance():
    report = json.loads((R / "canary" / "canary_report.json").read_text())
    receipt = json.loads((R / "canary" / "ledger_on" / report["cell"] / "receipt.json").read_text())
    import numpy as np
    dumps = [np.load(R / "canary" / run / "obs_dump.npz") for run in ("ledger_off", "ledger_on")]
    key_on = {(int(c), int(e)): i for i, (c, e) in enumerate(zip(dumps[1]["call_index"], dumps[1]["env_id"]))}
    pairs = [(i, key_on[(int(c), int(e))]) for i, (c, e) in
             enumerate(zip(dumps[0]["call_index"], dumps[0]["env_id"])) if (int(c), int(e)) in key_on]
    digests = [hashlib.sha256(np.ascontiguousarray(d["obs"][[p[k] for p in pairs]]).tobytes()).hexdigest()
               for k, d in enumerate(dumps)]
    d = report["detail"]
    return {
        "purpose": report["purpose"], "verdict": report["verdict"], "gates": report["checks"],
        "cell": report["cell"], "source_commit": report["git_commit"], "checkpoint_sha256": receipt["checkpoint_sha256"],
        "timings_s": report["timings"],
        "trajectory_digest": {"sha256": d["digest_sha256"][0], "steps": d["digest_steps"][0],
                              "ledger_on_equal": d["digest_sha256"][0] == d["digest_sha256"][1]},
        "observation_equality": {"rows": len(pairs), "sha256_ledger_off": digests[0], "sha256_ledger_on": digests[1]},
        "ledger": {"rows": d["ledger_rows"], "quota_episodes": sum(d["ledger_quota_counts"].values()),
                   "crash_rows_with_distance": d["crash_rows_with_distance"]},
        "legacy_reproduction": {"original_outcome": d["original_outcome"], "ledger_off_outcome": d["ledger_off_outcome"]},
        "large_artifacts": "observation dumps kept locally; see archives in replication_record.json",
    }


def build():
    rows = cell_rows()
    return {"cells.csv": cells_csv(rows),
            "min_distance_semantic_check.json": json.dumps(min_distance_check(), indent=1, sort_keys=True) + "\n",
            "canary_provenance.json": json.dumps(canary_provenance(), indent=1, sort_keys=True) + "\n"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    if not (R / rep.cell_id(*rep.cells()[0])).is_dir():
        print(json.dumps({"status": "SKIPPED_RAW_ABSENT",
                          "reason": "raw cells are archived outside Git; see replication_record.json"}))
        return 0
    files = build()
    if args.check:
        stale = [name for name, text in files.items() if (R / name).read_text() != text]
        print(json.dumps({"status": "FAIL" if stale else "PASS", "stale": stale}))
        return 1 if stale else 0
    for name, text in files.items():
        (R / name).write_text(text)
    print("wrote", ", ".join(files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
