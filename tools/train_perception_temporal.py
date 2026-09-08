"""Train one preregistered MOTAR temporal candidate selector and audit validation output."""

import argparse
import copy
import gzip
import json
import math
import os
import random
import statistics
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")

import numpy as np
import torch
from torch.nn import functional as F
from torch.utils.data import DataLoader

from eval_perception_kf import ArmMetrics, selected_result
from perception_candidates import canonical_line, sha256_file
from perception_temporal import (
    NO_LOCK_CLASS, TemporalCandidateDataset, build_temporal_model, candidate_box,
    load_aligned_records, parameter_count, selection_utility,
)


REPOSITORY = Path(__file__).resolve().parents[1]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-manifest", type=Path, required=True)
    parser.add_argument("--train-manifest-receipt", type=Path, required=True)
    parser.add_argument("--train-candidates", type=Path, required=True)
    parser.add_argument("--train-candidate-receipt", type=Path, required=True)
    parser.add_argument("--val-manifest", type=Path, required=True)
    parser.add_argument("--val-manifest-receipt", type=Path, required=True)
    parser.add_argument("--val-candidates", type=Path, required=True)
    parser.add_argument("--val-candidate-receipt", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    return parser.parse_args()


def git_value(*arguments):
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY), *arguments], text=True).strip()


def seed_everything(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False


def to_device(batch, device):
    return {
        key: value.to(device, non_blocking=True) if torch.is_tensor(value) else value
        for key, value in batch.items()
    }


def model_logits(model, batch):
    return model(
        batch["features"], batch["candidate_mask"], batch["frame_mask"],
        batch["delta_seconds"], batch["length"],
    )


def train_epoch(model, loader, optimizer, device, gradient_clip_norm):
    model.train()
    total_loss, total_rows = 0.0, 0
    for cpu_batch in loader:
        batch = to_device(cpu_batch, device)
        optimizer.zero_grad(set_to_none=True)
        logits = model_logits(model, batch)
        loss = F.cross_entropy(logits, batch["target"])
        if not torch.isfinite(loss):
            raise RuntimeError("non-finite temporal training loss")
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), gradient_clip_norm)
        optimizer.step()
        rows = int(batch["target"].shape[0])
        total_loss += float(loss.detach().cpu()) * rows
        total_rows += rows
    return total_loss / total_rows


def infer_dataset(model, loader, device, collect_predictions):
    model.eval()
    total_loss, total_rows, correct = 0.0, 0, 0
    predictions, probabilities = [], []
    latency_per_frame_ms = []
    with torch.no_grad():
        for cpu_batch in loader:
            batch = to_device(cpu_batch, device)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            began = time.perf_counter_ns()
            logits = model_logits(model, batch)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            elapsed_ms = (time.perf_counter_ns() - began) / 1e6
            rows = int(batch["target"].shape[0])
            latency_per_frame_ms.append(elapsed_ms / rows)
            loss = F.cross_entropy(logits, batch["target"])
            predicted = logits.argmax(dim=1)
            total_loss += float(loss.detach().cpu()) * rows
            total_rows += rows
            correct += int((predicted == batch["target"]).sum().detach().cpu())
            if collect_predictions:
                distribution = F.softmax(logits, dim=1).detach().cpu().numpy()
                positions = batch["record_index"].detach().cpu().numpy()
                for position, prediction, distribution_row in zip(
                        positions, predicted.detach().cpu().numpy(), distribution):
                    predictions.append((int(position), int(prediction)))
                    probabilities.append((int(position), distribution_row.tolist()))
    result = {
        "cross_entropy": total_loss / total_rows,
        "classification_accuracy": correct / total_rows,
        "rows": total_rows,
        "model_latency_ms_per_frame": {
            "scope": "batched tensor input through temporal logits; excludes P4 detector/descriptor",
            "mean": statistics.fmean(latency_per_frame_ms),
            "p50": statistics.median(latency_per_frame_ms),
            "p95": sorted(latency_per_frame_ms)[int(0.95 * (len(latency_per_frame_ms) - 1))],
            "batch_size": loader.batch_size,
        },
    }
    if collect_predictions:
        predictions.sort()
        probabilities.sort()
        result["predictions"] = [value for _, value in predictions]
        result["probabilities"] = [value for _, value in probabilities]
    return result


def association_metrics(dataset, predictions, evaluation_iou):
    metrics = ArmMetrics()
    detailed = []
    for index, (aligned, prediction) in enumerate(zip(dataset.records, predictions)):
        source = aligned["source"]
        record = aligned["candidate_record"]
        selected_candidate = None
        if prediction < len(record["candidates"]):
            selected_candidate = record["candidates"][prediction]
        box = candidate_box(selected_candidate) if selected_candidate is not None else None
        result = selected_result(box, source["ground_truth_xyxy"], evaluation_iou)
        metrics.add(source["source_sequence_id"], source["capture_timestamp_ns"], result)
        detailed.append((index, selected_candidate, box, result))
    report = metrics.report()
    report["selection_utility"] = selection_utility(report)
    return report, detailed


def main():
    args = parse_args()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit("[temporal] refusing existing output")
    config = json.loads(args.config.read_text())
    if config["architecture"] not in ("gru", "transformer"):
        raise SystemExit("[temporal] config architecture must be gru or transformer")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise SystemExit("[temporal] CUDA requested but unavailable")

    train_aligned, train_manifest_meta, train_candidate_meta = load_aligned_records(
        args.train_manifest, args.train_manifest_receipt,
        args.train_candidates, args.train_candidate_receipt)
    val_aligned, val_manifest_meta, val_candidate_meta = load_aligned_records(
        args.val_manifest, args.val_manifest_receipt,
        args.val_candidates, args.val_candidate_receipt)
    if train_manifest_meta["split"] != "train" or val_manifest_meta["split"] != "val":
        raise SystemExit("[temporal] requires train and validation manifests; test is forbidden")
    if train_candidate_meta["weights_sha256"] != val_candidate_meta["weights_sha256"]:
        raise SystemExit("[temporal] train/validation detector hashes differ")
    if (train_candidate_meta["appearance_encoder_config_sha256"]
            != val_candidate_meta["appearance_encoder_config_sha256"]):
        raise SystemExit("[temporal] train/validation appearance encoders differ")

    train_data = TemporalCandidateDataset(
        train_aligned, config["history_length"], config["evaluation_iou"])
    val_data = TemporalCandidateDataset(
        val_aligned, config["history_length"], config["evaluation_iou"])
    seed_everything(int(config["seed"]))
    generator = torch.Generator()
    generator.manual_seed(int(config["seed"]))
    train_loader = DataLoader(
        train_data, batch_size=int(config["batch_size"]), shuffle=True,
        num_workers=0, generator=generator)
    val_loader = DataLoader(
        val_data, batch_size=int(config["batch_size"]), shuffle=False, num_workers=0)
    model = build_temporal_model(config).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=float(config["learning_rate"]),
        weight_decay=float(config["weight_decay"]))

    output.mkdir(parents=True)
    history_path = output / "training_history.jsonl"
    best_loss, best_epoch, best_state = math.inf, None, None
    stale_epochs = 0
    began_training = time.monotonic()
    with history_path.open("w") as history_stream:
        for epoch in range(int(config["epochs_max"])):
            train_loss = train_epoch(
                model, train_loader, optimizer, device, float(config["gradient_clip_norm"]))
            validation = infer_dataset(model, val_loader, device, collect_predictions=False)
            row = {
                "epoch_zero_based": epoch,
                "train_cross_entropy": train_loss,
                "validation_cross_entropy": validation["cross_entropy"],
                "validation_classification_accuracy": validation["classification_accuracy"],
            }
            history_stream.write(canonical_line(row))
            history_stream.flush()
            if validation["cross_entropy"] < best_loss - 1e-12:
                best_loss = validation["cross_entropy"]
                best_epoch = epoch
                best_state = copy.deepcopy({
                    key: value.detach().cpu() for key, value in model.state_dict().items()})
                stale_epochs = 0
            else:
                stale_epochs += 1
            print("[temporal] %s epoch %d train %.6f val %.6f best %d" % (
                config["architecture"], epoch, train_loss, validation["cross_entropy"],
                best_epoch), flush=True)
            if stale_epochs >= int(config["patience"]):
                break
    if best_state is None:
        raise RuntimeError("temporal training did not produce a checkpoint")
    model.load_state_dict(best_state, strict=True)
    final_validation = infer_dataset(model, val_loader, device, collect_predictions=True)
    arm_metrics, detailed = association_metrics(
        val_data, final_validation.pop("predictions"), float(config["evaluation_iou"]))
    distributions = final_validation.pop("probabilities")

    checkpoint_path = output / "best.pt"
    torch.save({
        "schema_version": 1,
        "experiment": "motar.p6-p7.temporal-candidate-selector.v1",
        "architecture": config["architecture"],
        "config": config,
        "feature_contract": {
            "dimension": 69,
            "fields": "normalized [u,v,w,h], confidence, appearance_64d",
            "no_lock_class": NO_LOCK_CLASS,
            "causal_clip_bounded": True,
        },
        "model_state_dict": best_state,
        "best_epoch_zero_based": best_epoch,
        "detector_weights_sha256": train_candidate_meta["weights_sha256"],
    }, checkpoint_path)

    predictions_path = output / "validation_predictions.jsonl.gz"
    with gzip.open(str(predictions_path), "wt", encoding="utf-8", compresslevel=6) as stream:
        for (index, selected_candidate, box, result), distribution in zip(detailed, distributions):
            source = val_data.records[index]["source"]
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
        "experiment": "motar.p6-p7.temporal-candidate-selector.v1",
        "completed_utc": datetime.now(timezone.utc).isoformat(),
        "architecture": config["architecture"],
        "config": config,
        "best_epoch_zero_based": best_epoch,
        "best_validation_cross_entropy": best_loss,
        "validation": final_validation,
        "association_metrics": arm_metrics,
        "train": {
            "frames": len(train_data),
            "sequences": len(train_manifest_meta["sequences"]),
            "label_counts": train_data.label_counts(),
            "manifest_sha256": train_manifest_meta["manifest_sha256"],
            "candidates_sha256": train_candidate_meta["output_sha256"],
        },
        "validation_provenance": {
            "frames": len(val_data),
            "sequences": len(val_manifest_meta["sequences"]),
            "label_counts": val_data.label_counts(),
            "manifest_sha256": val_manifest_meta["manifest_sha256"],
            "candidates_sha256": val_candidate_meta["output_sha256"],
        },
        "detector_weights_sha256": train_candidate_meta["weights_sha256"],
        "appearance_encoder_config_sha256": train_candidate_meta[
            "appearance_encoder_config_sha256"],
        "parameter_count": parameter_count(model),
        "training_elapsed_seconds": time.monotonic() - began_training,
        "limitations": {
            "identity_metrics": "NOT_IDENTIFIABLE_SOURCE_TRACK_IDS_DROPPED",
            "ftlr": "NOT_IDENTIFIABLE_WITHOUT_DESIGNATED_TARGET_ID",
            "reported_proxy": "selected candidate has IoU below evaluation_iou against every UAV annotation",
        },
    }
    report_path = output / "report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    receipt = {
        "schema_version": 1,
        "architecture": config["architecture"],
        "source_git_commit": git_value("rev-parse", "HEAD"),
        "source_git_dirty": bool(git_value("status", "--porcelain", "--untracked-files=no")),
        "config_sha256": sha256_file(args.config),
        "checkpoint_sha256": sha256_file(checkpoint_path),
        "history_sha256": sha256_file(history_path),
        "validation_predictions_sha256": sha256_file(predictions_path),
        "report_sha256": sha256_file(report_path),
        "train_manifest_sha256": train_manifest_meta["manifest_sha256"],
        "train_candidates_sha256": train_candidate_meta["output_sha256"],
        "validation_manifest_sha256": val_manifest_meta["manifest_sha256"],
        "validation_candidates_sha256": val_candidate_meta["output_sha256"],
        "detector_weights_sha256": train_candidate_meta["weights_sha256"],
    }
    receipt_path = output / "receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print("[temporal] PASS: %s epoch %d utility %.6f -> %s" % (
        config["architecture"], best_epoch, arm_metrics["selection_utility"], output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
