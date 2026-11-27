"""Read-only structural readiness audit; this tool never authorizes a formal gate.

The fixed seed4501 cohort needs target states 0..600 for 600 scored actions.
Missing states are reported, never padded, extrapolated or replaced by another
episode. S3a's original tolerances stay fixed. Even complete inputs and numerically
equivalent reports require source-bound controls and independent review before
formal execution. Historical S3a reports and target files are never rewritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[1]
FRAME = struct.Struct("<HHI20fBbi")
FRAME_CONTRACT = "frame k = state after k completed actions; recorded when its observation is consumed"
EXPECTED = {(bars, env) for bars in (70, 205) for env in range(10)}
OBS_TOL, ACTION_TOL = 1e-3, 1e-4
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
LIMIT_BYTES = 512 * 1024**2
FRAME_DTYPE = [["env", "<u2"], ["frame", "<u2"], ["episode", "<u4"],
               ["p_pos", "<f4", [3]], ["p_vel", "<f4", [3]], ["p_quat", "<f4", [4]],
               ["p_vehicle_quat", "<f4", [4]], ["t_pos", "<f4", [3]], ["t_vel", "<f4", [3]],
               ["flags", "|u1"], ["probe_center_blocked", "|i1"], ["probe_visible_pixels", "<i4"]]
TARGET_IDENTITY = {"behavior_level": "e2_obstacle_aware", "distance_backend": "direct_l2_v1",
                   "target_dynamics": "bounded",
                   "target_motion_model": "bounded_planar_drone_v1_rollout_direct_l2_v1"}


def validate_trace_contract(summary, receipt, bindings, bars):
    """Cross-check wire schema, contextual identities and internal/external hashes."""
    cid = "E2_obstacle_aware__%dbars__seed4501" % bars
    metadata = summary["metadata"]
    context = metadata["context"]
    expected_context = {"arm": "A3", "bars": bars, "seed": 4501, "cell_id": cid,
                        "checkpoint_sha256": F_SHA, "target_behavior_level": "e2_obstacle_aware"}
    if (summary["schema"] != "motar.occlusion-trace.v1" or summary["frame_itemsize"] != FRAME.size
            or summary["frame_dtype"] != FRAME_DTYPE
            or metadata["frame_contract"] != FRAME_CONTRACT or metadata["dt_s"] != 0.1
            or metadata["identity"] != TARGET_IDENTITY
            or metadata["geometry"]["bar_pool"] != "bars_h3"
            or metadata["geometry"]["obstacle_slots"] != [0, bars]
            or any(context.get(k) != v for k, v in expected_context.items())
            or receipt["status"] != "OK" or receipt["seed"] != 4501
            or receipt["density_bars"] != bars or receipt["cell_id"] != cid
            or receipt["arm"] != "E2_obstacle_aware" or receipt["checkpoint_sha256"] != F_SHA
            or metadata["source_commit"] != receipt["git_commit"]
            or metadata["source_dirty"] is not False or receipt["git_dirty_files"] != 0):
        raise ValueError("trace/receipt schema or identity contract differs")
    for name in ("occlusion_frames.bin", "occlusion_layouts.npz"):
        if (summary["files"][name] != bindings[name]["sha256"]
                or summary["bytes"][name] != bindings[name]["bytes"]):
            raise ValueError("summary/input binding differs: " + name)
    if type(summary["frames"]) is not int or summary["frames"] * FRAME.size != bindings["occlusion_frames.bin"]["bytes"]:
        raise ValueError("summary frame count/byte count differs")


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            h.update(block)
    return h.hexdigest()


def checked_file(path, binding):
    path = Path(path)
    size = binding.get("bytes")
    if type(size) is not int or not 0 < size <= LIMIT_BYTES:
        raise ValueError("invalid recorded byte count: " + str(path))
    if path.stat().st_size != size or digest(path) != binding.get("sha256"):
        raise ValueError("input binding differs: " + str(path))


def target_windows(stream, count):
    """Decode selected episode0 states, refusing duplicate or malformed records."""
    if type(count) is not int or not 0 < count <= LIMIT_BYTES // FRAME.size:
        raise ValueError("invalid trace frame count")
    rows = {env: set() for env in range(10)}
    for _ in range(count):
        raw = stream.read(FRAME.size)
        if len(raw) != FRAME.size:
            raise ValueError("truncated frame")
        row = FRAME.unpack(raw)
        env, k, episode = row[:3]
        if env not in rows or episode != 0:
            continue
        if k in rows[env] or k > 600 or not all(math.isfinite(x) for x in row[3:23]):
            raise ValueError("duplicate, nonfinite or out-of-horizon selected state")
        rows[env].add(k)
    if stream.read(1):
        raise ValueError("extra bytes after declared frames")
    return {env: {"recorded_states": len(ks), "last_frame_index": max(ks) if ks else None,
                  "missing_frame_indices": sorted(set(range(601)) - ks),
                  "contiguous_from_zero": ks == set(range(len(ks))),
                  "status": "INPUTS_COMPLETE" if ks == set(range(601)) else "BLOCKED_INCOMPLETE_INPUTS"}
            for env, ks in rows.items()}


def audit_inputs(inventory):
    data = json.loads(Path(inventory).read_text())
    if set(data["cells"]) != {"70", "205"}:
        raise ValueError("fixed density cohort differs")
    windows = []
    for bars in (70, 205):
        cell = data["cells"][str(bars)]["zero"]
        directory = Path(cell["directory"])
        for name in ("receipt.json", "occlusion_trace_summary.json", "occlusion_frames.bin", "occlusion_layouts.npz"):
            checked_file(directory / name, cell["files"][name])
        summary = json.loads((directory / "occlusion_trace_summary.json").read_text())
        receipt = json.loads((directory / "receipt.json").read_text())
        validate_trace_contract(summary, receipt, cell["files"], bars)
        with (directory / "occlusion_frames.bin").open("rb") as stream:
            selected = target_windows(stream, summary["frames"])
        windows.extend(dict(row, bars=bars, env=env, episode=0) for env, row in selected.items())
    if {(r["bars"], r["env"]) for r in windows} != EXPECTED:
        raise ValueError("fixed scene cohort differs")
    return {"inventory": str(Path(inventory).resolve()), "inventory_sha256": digest(inventory),
            "status": "INPUTS_COMPLETE" if all(r["status"] == "INPUTS_COMPLETE" for r in windows)
            else "BLOCKED_INCOMPLETE_INPUTS", "windows": windows,
            "endpoint": "state600 at60.0s after600 completed actions; no command at endpoint"}


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def audit_s3a(path, bars):
    report = json.loads(Path(path).read_text())
    problems = []
    if report.get("cell") != "E2_obstacle_aware__%dbars__seed4501" % bars:
        problems.append("wrong cell identity")
    if report.get("tolerances") != {"obs": OBS_TOL, "action": ACTION_TOL}:
        problems.append("fixed tolerances differ")
    if report.get("alignment_mismatches") != 0:
        problems.append("missing or mismatched alignment")
    envs = report.get("envs", {})
    if not isinstance(envs, dict) or set(envs) != {str(i) for i in range(8)}:
        problems.append("original first8-env S3a coverage differs")
        envs = {}
    for env, row in envs.items():
        if type(row.get("steps")) is not int or row["steps"] <= 0:
            problems.append("empty S3a env" + env)
        slices = row.get("obs_err", {})
        if set(slices) != {"scan", "obstacles", "robot", "target"}:
            problems.append("incomplete observation slices in env" + env)
        for label, value in slices.items():
            if not finite_number(value) or value > OBS_TOL:
                problems.append("observation mismatch env%s/%s" % (env, label))
        for label in ("act_err_bridge", "act_err_dumpobs", "gov_err"):
            value = row.get(label)
            if not finite_number(value) or value > ACTION_TOL:
                problems.append("action/control mismatch env%s/%s" % (env, label))
    if report.get("reading") != "EQUIVALENT":
        problems.append("S3a does not report equivalence")
    return {"path": str(Path(path).resolve()), "sha256": digest(path), "bars": bars,
            "status": "S3A_REPORTED_EQUIVALENT" if not problems else "BLOCKED_S3A",
            "problems": problems, "independent_source_bound_review": "REQUIRED_SEPARATELY",
            "positive_and_one_degree_yaw_negative_controls": "REQUIRED_SEPARATELY"}


def audit(inventory, s3a_paths):
    if set(s3a_paths) != {70, 205}:
        raise ValueError("both S3a cells required")
    inputs = audit_inputs(inventory)
    s3a = [audit_s3a(s3a_paths[bars], bars) for bars in (70, 205)]
    complete = inputs["status"] == "INPUTS_COMPLETE" and all(
        row["status"] == "S3A_REPORTED_EQUIVALENT" for row in s3a)
    return {"schema": "motar.sitl-formal-readiness.structural.v1",
            "status": "STRUCTURAL_CHECKS_PASS_REVIEW_REQUIRED" if complete else "BLOCKED",
            "formal_gate_authorized": False, "formal_gate_verdict": "NOT_TESTED",
            "inputs": inputs, "s3a": s3a,
            "limits": ["Structural audit only; no simulator, policy inference or gate executed.",
                       "S3a first8-env scope and target fixed20-scene scope are distinct.",
                       "A reading label alone does not establish source binding, control sensitivity or review.",
                       "New source-bound S3a controls and independent review are required before formal execution.",
                       "Same-observation CPU/GPU action discrepancy is retained as a readiness control.",
                       "Frozen records and missing target states are never repaired retrospectively."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, default=ROOT / "docs/evidence/sitl_2026-11-05/dev08/input_inventory.json")
    parser.add_argument("--s3a-70", type=Path, default=ROOT / "docs/evidence/sitl_2026-11-05/s3a/s3a_70bars_9882889.json")
    parser.add_argument("--s3a-205", type=Path, default=ROOT / "docs/evidence/sitl_2026-11-05/s3a/s3a_205bars_9882889.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = audit(args.inventory, {70: args.s3a_70, 205: args.s3a_205})
        with args.output.open("x") as stream:
            json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError, TypeError, KeyError, struct.error) as exc:
        parser.exit(2, "formal readiness audit refused: %s\n" % exc)
    print(json.dumps({"status": result["status"], "formal_gate_authorized": False}))
    return 0 if result["status"] == "STRUCTURAL_CHECKS_PASS_REVIEW_REQUIRED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
