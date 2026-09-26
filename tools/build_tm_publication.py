#!/usr/bin/env python3
"""Generate the portable canonical status from frozen replication records; no measurement."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "results/target_motion_replication_7seed"


def build():
    def read(name):
        return json.loads((RESULT / name).read_text())
    analysis = read("analysis.json")
    integrity = read("integrity_report.json")
    record = read("replication_record.json")
    assert integrity["VALID"]
    return {
        "status": analysis["verdict"],
        "scope": analysis["design"].replace(" ", "_").replace("-", "_"),
        "evidence_role": "CURRENT",
        "valid_cells": integrity["cells_found"],
        "primary_episodes": integrity["primary_rows"],
        "fresh_seeds": analysis["n_seeds"],
        "seeds": analysis["seeds"],
        "ppo_training_started": read("preflight_main.json")["ppo_training_started"],
        "minimum_distance_public_status": "WITHHELD_SEMANTIC_MISMATCH",
        "commit": record["measurement_commit"],
        "freeze_commit": record["freeze_commit"],
        "checkpoint_sha256": record["checkpoint_sha256"],
        "analysis_sha256": hashlib.sha256((RESULT / "analysis.json").read_bytes()).hexdigest(),
        "analysis_path": "results/target_motion_replication_7seed/analysis.json",
        "historical_result": "results/target_motion_e0_e2_2026-09-18/canonical_summary.json",
        "pooled_with_historical": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    path = RESULT / "summary.json"
    content = json.dumps(build(), indent=2, sort_keys=True) + "\n"
    if args.check:
        assert path.read_text() == content, "canonical replication summary is stale"
        print("PASS: canonical replication status matches frozen analysis")
    else:
        path.write_text(content)


if __name__ == "__main__":
    main()
