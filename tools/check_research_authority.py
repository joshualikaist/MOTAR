#!/usr/bin/env python3
"""Verify the frozen MOTAR execution authority against immutable result summaries.

This tool does not launch training or evaluation.  It makes the current NO-GO/allowed-next
boundary machine-readable so a stale document or a convenient rerun command cannot silently
supersede the preregistered Track A/B decisions.
"""

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECEIPT = ROOT / "docs" / "research_authority_2026-08-26.json"


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_authority(receipt_path=DEFAULT_RECEIPT):
    receipt_path = Path(receipt_path).resolve()
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    if receipt.get("schema") != "motar_research_execution_authority_v1":
        raise RuntimeError("research authority schema drift")

    evidence = {}
    for record in receipt.get("evidence", []):
        relative = record["path"]
        path = ROOT / relative
        if not path.is_file():
            raise RuntimeError("missing frozen evidence: %s" % relative)
        actual_sha = _sha256(path)
        if actual_sha != record["sha256"]:
            raise RuntimeError(
                "frozen evidence SHA drift: %s expected=%s actual=%s"
                % (relative, record["sha256"], actual_sha)
            )
        evidence[relative] = json.loads(path.read_text(encoding="utf-8"))

    stage1 = evidence["results/navrl_ref5in_detection_range_stage1_s457/summary.json"]
    if stage1.get("verdict") != receipt["track_a"]["result"]:
        raise RuntimeError("Track A verdict drift")
    if stage1.get("stage2_authorised") is not False:
        raise RuntimeError("Track A Stage 2 unexpectedly authorized")

    recovery = evidence[
        "results/navrl_physical_target_recovery_v2_gate_lower1p25_seed827/summary.json"
    ]
    verdict = recovery.get("verdict", {})
    if verdict.get("execution_integrity") != receipt["track_b"]["execution_integrity"]:
        raise RuntimeError("Track B execution-integrity drift")
    if verdict.get("route_mechanism") != receipt["track_b"]["route_mechanism"]:
        raise RuntimeError("Track B route-mechanism drift")
    if verdict.get("long_training_authorized") is not False:
        raise RuntimeError("Track B long training unexpectedly authorized")
    if recovery.get("claim_boundary", {}).get("ppo_policy_loaded") is not False:
        raise RuntimeError("Track B diagnostic unexpectedly claims a PPO policy")
    if recovery.get("claim_boundary", {}).get("hardware_validation") is not False:
        raise RuntimeError("Track B diagnostic unexpectedly claims hardware validation")

    followup = evidence[
        "results/navrl_physical_target_recovery_v2_no_connector_forensics_seed827/summary.json"
    ]
    decision = followup.get("decision_rule", {})
    if decision.get("label") != receipt["track_b"]["no_anchor_followup"]:
        raise RuntimeError("Track B no-anchor verdict drift")
    if decision.get("authorizes_retune_or_ppo") is not False:
        raise RuntimeError("Track B no-anchor follow-up unexpectedly authorizes work")
    if followup.get("passes_32_cell_mechanism") is not False:
        raise RuntimeError("Track B no-anchor follow-up unexpectedly supersedes 32-cell FAIL")

    corrected = receipt["corrected_environment_v2_2026_08_27"]
    corrected_gate = corrected["route_physical_gate_r2"]
    corrected_result = evidence[
        "results/navrl_corrected_nonoverlap_route_gate_r2_seed829/summary.json"
    ]
    corrected_verdict = corrected_result.get("verdicts", {})
    for field in ("execution_integrity", "route_mechanism", "full_1p5_contract"):
        if corrected_verdict.get(field) != corrected_gate[field]:
            raise RuntimeError("corrected route gate %s drift" % field)
    corrected_inputs = corrected_verdict.get("route_mechanism_inputs", {})
    for field in (
        "plan_success_fraction_70",
        "fallback_interval_fraction_70",
        "goal_completions_per_env_70_speed_0p6",
    ):
        if corrected_inputs.get(field) != corrected_gate[field]:
            raise RuntimeError("corrected route gate %s drift" % field)
    if corrected_verdict.get("highest_passing_speed_mps_by_density") != (
        corrected_gate["highest_passing_speed_mps_by_density"]
    ):
        raise RuntimeError("corrected route gate density envelope drift")
    if corrected_verdict.get("long_training_authority") is not False:
        raise RuntimeError("corrected route gate unexpectedly authorizes long training")
    if corrected_gate.get("authorizes_ppo") is not False:
        raise RuntimeError("corrected route gate receipt unexpectedly authorizes PPO")
    if corrected_gate.get("fresh_ppo_epochs_run") != 0:
        raise RuntimeError("corrected route gate receipt unexpectedly claims fresh PPO epochs")

    if receipt.get("hardware_state") != {
        "assembled_airframe": False,
        "real_sensor_logs": 0,
        "real_flights": 0,
        "sim_to_real_claim_authorized": False,
    }:
        raise RuntimeError("hardware claim boundary drift")
    return receipt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--receipt", default=str(DEFAULT_RECEIPT))
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    receipt = verify_authority(args.receipt)
    summary = {
        "verified": True,
        "track_a": receipt["track_a"]["result"],
        "track_a_stage2_authorised": receipt["track_a"]["stage2_authorised"],
        "track_b": receipt["track_b"]["route_mechanism"],
        "track_b_long_training_authorized": receipt["track_b"]["long_training_authorized"],
        "corrected_route_gate": receipt["corrected_environment_v2_2026_08_27"][
            "route_physical_gate_r2"
        ]["route_mechanism"],
        "corrected_fresh_ppo_epochs_run": receipt["corrected_environment_v2_2026_08_27"][
            "route_physical_gate_r2"
        ]["fresh_ppo_epochs_run"],
        "next": receipt["track_a"]["allowed_next"],
    }
    if args.json:
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    else:
        print("PASS_RESEARCH_AUTHORITY_FREEZE")
        print("Track A: %s; Stage 2=false" % summary["track_a"])
        print("Track B: %s; long training=false" % summary["track_b"])
        print("Corrected route gate: %s; fresh PPO epochs=0" % summary["corrected_route_gate"])
        print("Next: hardware BOM/calibration/210 trials/real-log offline replay")


if __name__ == "__main__":
    main()
