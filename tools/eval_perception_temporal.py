"""Evaluate one frozen P6/P7 selector on an audited candidate manifest without retraining."""

import argparse
import gzip
import json
from datetime import datetime, timezone
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from perception_candidates import canonical_line, sha256_file
from perception_temporal import (
    NO_LOCK_CLASS, TemporalCandidateDataset, build_temporal_model, candidate_box,
    load_aligned_records, parameter_count,
)
from train_perception_temporal import association_metrics, infer_dataset


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-receipt", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--candidate-receipt", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    return parser.parse_args()


def main():
    args = parse_args()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit("[temporal-eval] refusing existing output")
    checkpoint_receipt = json.loads(args.checkpoint_receipt.read_text())
    checkpoint_sha = sha256_file(args.checkpoint)
    if checkpoint_sha != checkpoint_receipt["checkpoint_sha256"]:
        raise SystemExit("[temporal-eval] checkpoint hash mismatch")
    aligned, manifest_meta, candidate_meta = load_aligned_records(
        args.manifest, args.manifest_receipt, args.candidates, args.candidate_receipt)
    if manifest_meta["split"] not in ("val", "test"):
        raise SystemExit("[temporal-eval] evaluation requires validation or test split")
    if candidate_meta["weights_sha256"] != checkpoint_receipt["detector_weights_sha256"]:
        raise SystemExit("[temporal-eval] checkpoint/candidate detector hashes differ")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise SystemExit("[temporal-eval] CUDA requested but unavailable")
    payload = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    if payload.get("schema_version") != 1:
        raise SystemExit("[temporal-eval] unsupported checkpoint schema")
    config = payload["config"]
    if payload["architecture"] != checkpoint_receipt["architecture"]:
        raise SystemExit("[temporal-eval] checkpoint architecture receipt mismatch")
    if payload["detector_weights_sha256"] != candidate_meta["weights_sha256"]:
        raise SystemExit("[temporal-eval] checkpoint payload detector hash mismatch")
    dataset = TemporalCandidateDataset(
        aligned, config["history_length"], config["evaluation_iou"])
    loader = DataLoader(
        dataset, batch_size=int(config["batch_size"]), shuffle=False, num_workers=0)
    model = build_temporal_model(config)
    model.load_state_dict(payload["model_state_dict"], strict=True)
    model.to(device)
    inference = infer_dataset(model, loader, device, collect_predictions=True)
    predictions = inference.pop("predictions")
    probabilities = inference.pop("probabilities")
    arm_metrics, detailed = association_metrics(
        dataset, predictions, float(config["evaluation_iou"]))

    output.mkdir(parents=True)
    raw_path = output / "predictions.jsonl.gz"
    with gzip.open(str(raw_path), "wt", encoding="utf-8", compresslevel=6) as stream:
        for (index, selected_candidate, box, result), distribution in zip(detailed, probabilities):
            source = dataset.records[index]["source"]
            predicted_rank = int(selected_candidate["rank"]) if selected_candidate is not None else None
            selected_probability = (
                distribution[predicted_rank] if predicted_rank is not None
                else distribution[NO_LOCK_CLASS])
            stream.write(canonical_line({
                "frame_id": source["frame_id"],
                "source_sequence_id": source["source_sequence_id"],
                "frame_index": source["frame_index"],
                "capture_timestamp_ns": source["capture_timestamp_ns"],
                "predicted_rank": predicted_rank,
                "selected_probability": selected_probability,
                "no_lock_probability": distribution[NO_LOCK_CLASS],
                "box_xyxy": box,
                **result,
            }))
    report = {
        "schema_version": 1,
        "experiment": "motar.p6-p7.frozen-temporal-evaluation.v1",
        "completed_utc": datetime.now(timezone.utc).isoformat(),
        "split": manifest_meta["split"],
        "architecture": payload["architecture"],
        "config": config,
        "checkpoint_sha256": checkpoint_sha,
        "checkpoint_source_git_commit": checkpoint_receipt["source_git_commit"],
        "detector_weights_sha256": candidate_meta["weights_sha256"],
        "manifest_sha256": manifest_meta["manifest_sha256"],
        "candidates_sha256": candidate_meta["output_sha256"],
        "frames": len(dataset),
        "sequences": len(manifest_meta["sequences"]),
        "label_counts": dataset.label_counts(),
        "validation_or_test": inference,
        "association_metrics": arm_metrics,
        "parameter_count": parameter_count(model),
        "latency_ms": {
            "candidate_producer": candidate_meta["latency_ms"],
            "temporal_model": inference["model_latency_ms_per_frame"],
            "combined_mean": (
                candidate_meta["latency_ms"]["mean"]
                + inference["model_latency_ms_per_frame"]["mean"]),
        },
        "limitations": {
            "identity_metrics": "NOT_IDENTIFIABLE_SOURCE_TRACK_IDS_DROPPED",
            "ftlr": "NOT_IDENTIFIABLE_WITHOUT_DESIGNATED_TARGET_ID",
        },
    }
    report_path = output / "report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    receipt = {
        "schema_version": 1,
        "report_sha256": sha256_file(report_path),
        "predictions_sha256": sha256_file(raw_path),
        "checkpoint_sha256": checkpoint_sha,
        "manifest_sha256": manifest_meta["manifest_sha256"],
        "candidates_sha256": candidate_meta["output_sha256"],
        "detector_weights_sha256": candidate_meta["weights_sha256"],
    }
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print("[temporal-eval] PASS: %s %s utility %.6f -> %s" % (
        manifest_meta["split"], payload["architecture"],
        arm_metrics["selection_utility"], output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
