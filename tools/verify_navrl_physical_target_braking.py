#!/usr/bin/env python3
"""Standalone, raw-data-first verifier for the physical-target braking receipt."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from probe_navrl_physical_target_braking import (
    COMPLETE_MARKER,
    FROZEN_CONTRACT,
    REGISTERED_ENVS,
    REGISTERED_SPEEDS,
    RECEIPT_SCHEMA,
    RECOVERY_SOURCE_PATHS,
    SATURATION_MAX,
    SCHEMA,
    TILT_MAX_DEG,
    canonical_json_bytes,
    git_head,
    quantile_stats,
    require_finite,
    sha256_bytes,
    sha256_file,
    source_manifest,
    TOOL_SOURCE_PATHS,
)


def _same_contract(observed: Any, expected: Any, path: str = "contract") -> None:
    if isinstance(expected, float):
        if not isinstance(observed, (int, float)) or abs(float(observed) - expected) > 1e-9:
            raise ValueError("%s mismatch: %r != %r" % (path, observed, expected))
        return
    if isinstance(expected, list):
        if not isinstance(observed, list) or len(observed) != len(expected):
            raise ValueError("%s length mismatch" % path)
        for index, (got, want) in enumerate(zip(observed, expected)):
            _same_contract(got, want, "%s[%d]" % (path, index))
        return
    if observed != expected:
        raise ValueError("%s mismatch: %r != %r" % (path, observed, expected))


def _read_json(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise ValueError("missing JSON: %s" % path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    require_finite(payload, str(path))
    if canonical_json_bytes(payload) != path.read_bytes():
        raise ValueError("non-canonical JSON: %s" % path)
    if not isinstance(payload, dict):
        raise ValueError("JSON root is not an object: %s" % path)
    return payload


def _validate_provenance(payload: Mapping[str, Any]) -> None:
    provenance = payload.get("provenance")
    if not isinstance(provenance, dict):
        raise ValueError("cell runtime provenance is missing")
    required = (
        "python_executable", "python_version", "torch_version", "torch_cuda_version",
        "cuda_device", "nvidia_smi_path", "nvidia_smi_sha256", "nvidia_smi_identity",
        "gpu_driver_version", "ninja_path", "ninja_sha256", "ninja_version",
    )
    if any(not isinstance(provenance.get(key), str) or not provenance.get(key) for key in required):
        raise ValueError("cell runtime provenance is incomplete")
    imported = provenance.get("imported_modules")
    if not isinstance(imported, dict) or not imported:
        raise ValueError("cell import-origin provenance is missing")
    for module, record in imported.items():
        if not isinstance(module, str) or not isinstance(record, dict) or not record.get("path") or not record.get("sha256"):
            raise ValueError("cell import-origin provenance is malformed")
    tools = provenance.get("tool_hashes")
    if not isinstance(tools, dict) or set(tools) != set(TOOL_SOURCE_PATHS):
        raise ValueError("cell tool hash provenance is incomplete")
    if any(not isinstance(value, str) or len(value) != 64 for value in tools.values()):
        raise ValueError("cell tool hash provenance is malformed")


def validate_cell(payload: Mapping[str, Any]) -> Dict[str, Any]:
    if payload.get("schema") != SCHEMA:
        raise ValueError("cell schema mismatch")
    _same_contract(payload.get("contract"), FROZEN_CONTRACT)
    _validate_provenance(payload)
    cell = payload.get("cell")
    if not isinstance(cell, dict):
        raise ValueError("cell metadata missing")
    speed = float(cell.get("speed_mps"))
    if speed not in REGISTERED_SPEEDS or int(cell.get("envs", -1)) != REGISTERED_ENVS:
        raise ValueError("cell is outside the frozen speed/env grid")
    rows = payload.get("raw_samples")
    if not isinstance(rows, list) or len(rows) != REGISTERED_ENVS:
        raise ValueError("raw_samples must contain exactly 32 env rows")
    env_ids = []
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("raw sample is not an object")
        env_id = int(row.get("env_id", -1))
        if env_id in env_ids or env_id < 0 or env_id >= REGISTERED_ENVS:
            raise ValueError("raw env ids are not a unique 0..31 population")
        env_ids.append(env_id)
        if abs(float(row.get("requested_speed_mps")) - speed) > 1e-12:
            raise ValueError("raw requested speed differs from cell speed")
        for field in ("measured_initial_speed_mps", "stop_time_s", "stop_distance_m", "effective_deceleration_mps2", "motor_saturation_fraction", "max_tilt_deg"):
            value = float(row.get(field))
            if value < 0.0:
                raise ValueError("negative raw %s" % field)
        if bool(row.get("contact")):
            raise ValueError("contact gate failed")
        if bool(row.get("invalid_obb")):
            raise ValueError("invalid OBB gate failed")
        if float(row["motor_saturation_fraction"]) > SATURATION_MAX:
            raise ValueError("motor saturation gate failed")
        if float(row["max_tilt_deg"]) > TILT_MAX_DEG:
            raise ValueError("tilt gate failed")
    rows = sorted(rows, key=lambda item: int(item["env_id"]))
    summary = {
        "speed_mps": speed,
        "envs": len(rows),
        "stop_time_s": quantile_stats([float(row["stop_time_s"]) for row in rows]),
        "stop_distance_m": quantile_stats([float(row["stop_distance_m"]) for row in rows]),
        "effective_deceleration_mps2": quantile_stats([float(row["effective_deceleration_mps2"]) for row in rows]),
        "max_motor_saturation_fraction": max(float(row["motor_saturation_fraction"]) for row in rows),
        "max_tilt_deg": max(float(row["max_tilt_deg"]) for row in rows),
        "contact_count": sum(bool(row["contact"]) for row in rows),
        "invalid_obb_count": sum(bool(row["invalid_obb"]) for row in rows),
        "gates": {
            "contact": True,
            "invalid_obb": True,
            "motor_saturation": True,
            "tilt": True,
        },
    }
    return summary


def summarize_cells(cells: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    if len(cells) != len(REGISTERED_SPEEDS):
        raise ValueError("the receipt must contain one cell for every registered speed")
    summaries = []
    seen = set()
    for payload in cells:
        summary = validate_cell(payload)
        speed = float(summary["speed_mps"])
        if speed in seen:
            raise ValueError("duplicate speed cell")
        seen.add(speed)
        summaries.append(summary)
    if seen != set(REGISTERED_SPEEDS):
        raise ValueError("speed grid is incomplete")
    summaries.sort(key=lambda item: item["speed_mps"])
    lookup = {}
    for row in summaries:
        key = format(float(row["speed_mps"]), ".1f")
        lookup[key] = {
            "speed_mps": row["speed_mps"],
            "p95_stop_time_s": row["stop_time_s"]["p95"],
            "p95_stop_distance_m": row["stop_distance_m"]["p95"],
            "p05_effective_deceleration_mps2": row["effective_deceleration_mps2"]["p05"],
        }
    return {
        "schema": SCHEMA,
        "speeds_mps": list(REGISTERED_SPEEDS),
        "envs_per_speed": REGISTERED_ENVS,
        "cells": summaries,
        "measured_speed_to_p95_lookup": lookup,
        "raw_first": True,
        "quantile_method": "linear_sorted_n_minus_1_probability",
        "gates": {
            "contact_count": 0,
            "invalid_obb_count": 0,
            "motor_saturation_fraction_max": SATURATION_MAX,
            "max_tilt_deg_max": TILT_MAX_DEG,
        },
    }


def _verify_git_object(repo_root: Path, recorded_head: str) -> None:
    if len(recorded_head) != 40:
        raise ValueError("receipt git_head is not a full commit id")
    check = subprocess.run(["git", "-C", str(repo_root), "cat-file", "-e", recorded_head + "^{commit}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if check.returncode != 0:
        raise ValueError("receipt git_head is not an available commit object")


def verify_receipt(output: Path, repo_root: Optional[Path] = None) -> Dict[str, Any]:
    """Verify an immutable finalized receipt and recompute all statistics from raw cells.

    Recorded git HEAD is required to be a valid commit object, but is deliberately not compared
    to the current HEAD: committing the receipt must not make its provenance unverifiable.
    """
    output = output.resolve()
    root = (repo_root or Path(__file__).resolve().parents[1]).resolve()
    if not output.is_dir() or (output / ".partial").exists():
        raise ValueError("output is not a finalized receipt directory")
    marker = output / "complete.marker"
    if not marker.is_file() or marker.read_text(encoding="utf-8") != COMPLETE_MARKER + "\n":
        raise ValueError("completion marker missing or invalid")
    receipt = _read_json(output / "receipt.json")
    if receipt.get("schema") != RECEIPT_SCHEMA or receipt.get("probe_schema") != SCHEMA:
        raise ValueError("receipt schema mismatch")
    _same_contract(receipt.get("contract"), FROZEN_CONTRACT)
    _verify_git_object(root, str(receipt.get("git_head", "")))
    manifest_rel = str(receipt.get("source_manifest", ""))
    if Path(manifest_rel).is_absolute() or manifest_rel != "source_manifest.json":
        raise ValueError("source manifest must be receipt-relative")
    manifest_path = output / manifest_rel
    manifest = _read_json(manifest_path)
    if sha256_file(manifest_path) != receipt.get("source_manifest_sha256"):
        raise ValueError("source manifest hash mismatch")
    entries = manifest.get("entries")
    if not isinstance(entries, list):
        raise ValueError("source manifest entries are missing")
    recorded_paths = [entry.get("path") for entry in entries if isinstance(entry, dict)]
    if any(not isinstance(path, str) or Path(path).is_absolute() or ".." in Path(path).parts for path in recorded_paths):
        raise ValueError("source manifest contains an unsafe path")
    if set(recorded_paths) != set(RECOVERY_SOURCE_PATHS):
        raise ValueError("source manifest path set is not the recovery contract")
    current_manifest = source_manifest(root, tuple(recorded_paths))
    if current_manifest != manifest:
        raise ValueError("runtime source bytes differ from receipt manifest")
    cell_payloads = []
    for cell in receipt.get("cells", []):
        path = Path(str(cell.get("path", "")))
        if path.is_absolute() or ".." in path.parts:
            raise ValueError("cell path must be receipt-relative")
        cell_path = output / path
        if sha256_file(cell_path) != cell.get("sha256"):
            raise ValueError("cell hash mismatch: %s" % path)
        payload = _read_json(cell_path)
        summary = validate_cell(payload)
        if abs(float(cell.get("speed_mps")) - float(summary["speed_mps"])) > 1e-12:
            raise ValueError("cell metadata speed mismatch")
        expected_provenance_hash = sha256_bytes(canonical_json_bytes(payload["provenance"]))
        if cell.get("provenance_sha256") != expected_provenance_hash:
            raise ValueError("cell runtime provenance hash mismatch")
        cell_payloads.append(payload)
    summary_path = output / str(receipt.get("summary_path", "summary.json"))
    summary = _read_json(summary_path)
    if sha256_file(summary_path) != receipt.get("summary_sha256"):
        raise ValueError("summary hash mismatch")
    recomputed = summarize_cells(cell_payloads)
    if summary != recomputed or receipt.get("summary") != recomputed:
        raise ValueError("summary is not a raw-data recomputation")
    lookup_values = list(recomputed["measured_speed_to_p95_lookup"].values())
    expected_decel = min(float(row["p05_effective_deceleration_mps2"]) for row in lookup_values)
    expected_time = max(float(row["p95_stop_time_s"]) for row in lookup_values)
    if abs(float(receipt.get("decel_p05_mps2", -1.0)) - expected_decel) > 1e-12:
        raise ValueError("receipt decel_p05_mps2 is not derived from raw cells")
    if abs(float(receipt.get("stop_time_p95_s", -1.0)) - expected_time) > 1e-12:
        raise ValueError("receipt stop_time_p95_s is not derived from raw cells")
    if receipt.get("measured_speed_to_p95_lookup") != recomputed["measured_speed_to_p95_lookup"]:
        raise ValueError("receipt speed lookup mismatch")
    return {"verified": True, "schema": receipt["schema"], "summary": recomputed, "git_head": receipt["git_head"]}


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", required=True, metavar="RECEIPT_DIR")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    result = verify_receipt(Path(args.verify))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
