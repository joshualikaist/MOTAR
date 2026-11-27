#!/usr/bin/env python3
"""Validate recorded R-P1a sidecars on CPU; no launch, render, rollout or policy commands.

Only scalar JSONL and optional episode ledgers are read. Reports go to stdout or an
exclusively created file outside any results tree. Unknown geometry stays unknown.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
POLICY_F_SHA256 = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
SIDECAR_SCHEMA = "motar.recovery-observability.v1"
AUDIT_SCHEMA = "motar.recovery-observability-audit.v1"
NOT_RECORDED = "NOT_RECORDED"
LINEAGES = {
    "H_historical", "E0_static", "E1_cv", "E2_obstacle_aware",
    "E1_direct_l2_v1", "E2_direct_l2_v1", "historical", "e0_static", "e1_cv",
    "e2_obstacle_aware", "bounded_planar_drone_v1_rollout",
    "bounded_planar_drone_v1_rollout_direct_l2_v1",
    "symmetric_local_steer_v2_heading_continuity90",
}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_event_core():
    """Load the standard-library-only event core without importing the simulator package."""
    path = ROOT / "motar_eval/recovery_events.py"
    spec = importlib.util.spec_from_file_location("_recovery_observability_cpu_core", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def tracker_expiry_source_contract():
    """Read the actual age > memory predicate and 5 s configuration using AST, not imports."""
    perception = ROOT / "aerial_gym/task/navrl_task/navrl_perception.py"
    config = ROOT / "aerial_gym/config/task_config/navrl_task_config.py"
    tree = ast.parse(perception.read_text())
    tracker = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == "BatchedConstantVelocityTracker")
    step = next(n for n in tracker.body if isinstance(n, ast.FunctionDef) and n.name == "step")
    strict = any(isinstance(n, ast.Compare) and len(n.ops) == 1 and isinstance(n.ops[0], ast.Gt)
                 and isinstance(n.left, ast.Attribute) and n.left.attr == "age"
                 and isinstance(n.comparators[0], ast.Attribute)
                 and n.comparators[0].attr == "memory_s" for n in ast.walk(step))
    values = [n.value.value for n in ast.walk(ast.parse(config.read_text()))
              if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant)
              and any(isinstance(t, ast.Name) and t.id == "tracker_memory_s" for t in n.targets)]
    if not strict or values != [5.0]:
        raise ValueError("tracker expiry source differs from the frozen strict age > 5.0 s contract")
    return {"memory_s": 5.0, "comparator": "age > memory_s",
            "age_boundary": "actual recorded float32 age; no evaluator timer expiry",
            "perception_sha256": sha256_file(perception), "config_sha256": sha256_file(config)}


def _finite(value, label, *, optional=False):
    if optional and value is None:
        return
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(label + " must be a finite number")


def _hex(value, length, label):
    if not isinstance(value, str) or re.fullmatch("[0-9a-f]{%d}" % length, value) is None:
        raise ValueError(label + " must be a lowercase hexadecimal digest")


def _validate_metadata(header, core, expected_source_commit, expected_checkpoint_sha256,
                       expected_lineage):
    if header.get("schema") != SIDECAR_SCHEMA:
        raise ValueError("unsupported recovery sidecar schema")
    meta = header.get("metadata", header.get("run_identity", {}))
    if not isinstance(meta, dict):
        raise ValueError("header metadata must be an object")
    if header.get("evaluation_only", meta.get("evaluation_only")) is not True:
        raise ValueError("sidecar must declare evaluation_only=true")
    _hex(meta.get("source_commit"), 40, "source_commit")
    _hex(meta.get("checkpoint_sha256"), 64, "checkpoint_sha256")
    if expected_source_commit and meta["source_commit"] != expected_source_commit:
        raise ValueError("source commit differs from the requested source binding")
    if meta["checkpoint_sha256"] != expected_checkpoint_sha256:
        raise ValueError("checkpoint differs from the requested Policy F binding")
    lineage = meta.get("motion_lineage", meta.get("target_motion_model"))
    if lineage not in LINEAGES or (expected_lineage and lineage != expected_lineage):
        raise ValueError("missing, unsupported or mismatched motion lineage")
    if "direct_l2_v1" in lineage and meta.get("distance_backend") != "direct_l2_v1":
        raise ValueError("direct lineage requires an explicit direct_l2_v1 backend")
    backend = meta.get("distance_backend")
    if backend not in ("legacy_cdist", "direct_l2_v1"):
        raise ValueError("unsupported distance backend")
    if backend == "direct_l2_v1" and "direct_l2_v1" not in lineage:
        raise ValueError("direct backend must retain its distinct lineage identity")
    seed = meta.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a recorded nonnegative integer")
    if meta.get("event_contract", core.CONTRACT) != core.CONTRACT:
        raise ValueError("event contract mismatch")
    dt = meta.get("dt_s", header.get("dt_s"))
    _finite(dt, "dt_s")
    if dt <= 0:
        raise ValueError("dt_s must be positive")
    if meta.get("tracker_memory_s", 5.0) != 5.0:
        raise ValueError("tracker memory differs from the actual frozen source")
    if meta.get("tracker_expiry_comparator", "age > memory_s") != "age > memory_s":
        raise ValueError("recorded expiry comparator differs from actual strict source")
    identity = header.get("run_identity", {})
    if not isinstance(identity, dict):
        raise ValueError("run_identity must be an object")
    for name in ("source_commit", "checkpoint_sha256", "motion_lineage", "distance_backend", "seed"):
        if name in identity and identity[name] != meta.get(name):
            raise ValueError("header metadata differs from hashed run identity: " + name)
    return meta, float(dt)


def _source_file_binding(header, metadata):
    dirty = header.get("source_dirty", metadata.get("source_dirty"))
    if dirty is not None and not isinstance(dirty, bool):
        raise ValueError("source_dirty must be an actual boolean when recorded")
    files = header.get("source_files", metadata.get("source_files",
                       header.get("source_files_sha256", metadata.get("source_files_sha256"))))
    if files is None:
        if dirty:
            raise ValueError("dirty source needs source_files hashes; HEAD alone does not bind it")
        return {"status": NOT_RECORDED, "note": "declared commit alone does not verify runtime file bytes"}
    if not isinstance(files, dict) or not files:
        raise ValueError("source_files must be a nonempty relative-path SHA mapping")
    for relative, digest in files.items():
        _hex(digest, 64, "source file SHA-256")
        path = ROOT / relative
        if Path(relative).is_absolute() or ".." in Path(relative).parts or ROOT not in path.resolve().parents:
            raise ValueError("source file must remain inside the replay checkout")
        if sha256_file(path) != digest:
            raise ValueError("source file differs from recorded bytes: " + relative)
    return {"status": "CURRENT_CHECKOUT_BYTES_MATCH", "files": files, "source_dirty": dirty}


def _episode_key(record, episode_id):
    if "env_index" in record and "env_episode_index" in record:
        values = (record["env_index"], record["env_episode_index"])
        if any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in values):
            raise ValueError("environment and episode indices must be nonnegative integers")
        return values
    match = re.fullmatch(r"s\d+/b\d+/e(\d+)/k(\d+)", episode_id)
    return (int(match[1]), int(match[2])) if match else None


def _load_ledger(path):
    ledger = {}
    with Path(path).open() as stream:
        for number, line in enumerate(stream, 1):
            row = json.loads(line)
            key = (int(row["env_index"]), int(row["env_episode_index"]))
            if key in ledger:
                raise ValueError("duplicate ledger episode at line %d" % number)
            if row["outcome"] not in ("capture", "crash", "timeout"):
                raise ValueError("unsupported ledger outcome")
            if any(name in row for name in ("capture", "crash", "timeout")):
                flags = [row.get(name) for name in ("capture", "crash", "timeout")]
                if (not all(isinstance(v, bool) for v in flags) or sum(flags) != 1
                        or not row[row["outcome"]]):
                    raise ValueError("ledger outcome flags do not form the recorded partition")
            ledger[key] = row
    return ledger


def _input_records(record):
    """The lossless sidecar stores core constructor inputs separately from emitted output."""
    inputs = record.get("inputs", record)
    if not all(isinstance(inputs.get(k), dict) for k in ("stamp", "sensor", "geometry")):
        raise ValueError("observation needs scalar stamp/sensor/geometry input objects")
    return inputs["stamp"], inputs["sensor"], inputs["geometry"]


def validate_trace(trace_path, ledger_path=None, *, expected_source_commit=None,
                   expected_checkpoint_sha256=POLICY_F_SHA256, expected_lineage=None,
                   include_episodes=False):
    """Return an audit dict; malformed traces are INVALID, never silently repaired or closed."""
    path = Path(trace_path)
    report = {"schema": AUDIT_SCHEMA, "status": "INVALID", "device": "cpu",
              "trace": str(path), "problems": [], "safe_abort": NOT_RECORDED,
              "full_visual_occlusion": NOT_RECORDED, "runtime_parity": "NOT_TESTED",
              "identity_limit": "sensor/track resumption is not persistent physical target identity",
              "scene_and_actor_prefix_limit": "event-prefix digests do not prove matched scenes or actor/action prefixes"}
    monitors, closed, identity, join_keys = {}, set(), {}, {}
    intervals = defaultdict(list)
    outcomes = Counter()
    closure = Counter()
    observations = 0
    eligible_observations = 0
    summaries = []
    matched_ledger = set()
    prefix_checked = 0
    try:
        core = load_event_core()
        report["event_core_sha256"] = sha256_file(ROOT / "motar_eval/recovery_events.py")
        report["tracker_expiry_source_contract"] = tracker_expiry_source_contract()
        ledger = _load_ledger(ledger_path) if ledger_path else None
        header = None
        with path.open() as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    raise ValueError("blank sidecar line at %d" % line_number)
                record = json.loads(line)
                if not isinstance(record, dict):
                    raise ValueError("sidecar records must be JSON objects")
                json.dumps(record, allow_nan=False)
                kind = record.get("record_type")
                if header is None:
                    if kind != "header":
                        raise ValueError("the first record must be a schema/source header")
                    header = record
                    metadata, dt = _validate_metadata(header, core, expected_source_commit,
                                                       expected_checkpoint_sha256, expected_lineage)
                    report["metadata"] = metadata
                    report["frame_contract"] = header.get("frame_contract", metadata.get("frame_contract", NOT_RECORDED))
                    report["source_file_binding"] = _source_file_binding(header, metadata)
                    report["source_binding"] = ("EXPECTED_COMMIT_MATCH" if expected_source_commit
                                                 else "DECLARED_COMMIT_ONLY")
                    continue
                if kind == "header":
                    raise ValueError("duplicate or changed run header")
                if kind == "observation":
                    stamp_dict, sensor_dict, geometry_dict = _input_records(record)
                    stamp = core.FrameStamp(**stamp_dict)
                    episode = stamp.episode_id
                    if episode in closed:
                        raise ValueError("observation after episode closure")
                    if episode not in monitors:
                        context = record.get("episode_context", {})
                        identity[episode] = context
                        monitors[episode] = core.RecoveryEpisodeEvents(
                            episode, dt_s=dt,
                            run_identity=header.get("run_identity", metadata),
                            scene_manifest_hash=context.get("scene_manifest_hash", NOT_RECORDED),
                            pre_loss_actor_prefix_hash=context.get("pre_loss_actor_prefix_hash", NOT_RECORDED))
                        join_keys[episode] = _episode_key(record, episode)
                    elif ("episode_context" in record and record["episode_context"] != identity[episode]):
                        raise ValueError("episode context changed inside a consumed trace")
                    if _episode_key(record, episode) != join_keys[episode]:
                        raise ValueError("episode environment/index identity changed")
                    if sensor_dict.get("track_memory_s", 5.0) != 5.0:
                        raise ValueError("observed track memory differs from frozen 5 s")
                    output = monitors[episode].observe(stamp, core.SensorFrame(**sensor_dict),
                                                       core.EvaluationGeometry(**geometry_dict))
                    stored = record.get("observation", record.get("output"))
                    if not isinstance(stored, dict):
                        raise ValueError("missing emitted observation for replay comparison")
                    if stored != output:
                        raise ValueError("emitted observation/prefix differs from exact CPU event replay")
                    prefix_checked += 1
                    observations += 1
                    eligible_observations += bool(output["eligible_before_frame"])
                elif kind in ("episode_end", "run_shutdown", "reset"):
                    stored = record.get("summary")
                    if not isinstance(stored, dict):
                        raise ValueError("closure needs the emitted episode summary")
                    episode = stored.get("episode_id")
                    if episode not in monitors or episode in closed:
                        raise ValueError("closure does not match exactly one open episode")
                    terminal_kind = stored.get("terminal_kind")
                    if terminal_kind != kind:
                        raise ValueError("closure kind disagrees with its terminal contract")
                    terminal_time = record.get("terminal_time_s")
                    _finite(terminal_time, "terminal_time_s")
                    if (terminal_kind == "episode_end"
                            and abs(terminal_time - monitors[episode].frames * dt) > 1e-6):
                        raise ValueError("episode terminal boundary must follow its final consumed action")
                    actual = monitors[episode].finish(terminal_time, stored.get("outcome"),
                                                      terminal_kind=terminal_kind)
                    if actual != stored:
                        raise ValueError("stored episode intervals/prefix differ from CPU replay")
                    if stored.get("safe_abort") != NOT_RECORDED:
                        raise ValueError("safe abort provenance is not implemented")
                    closed.add(episode)
                    closure[terminal_kind] += 1
                    outcome = stored.get("outcome")
                    if outcome is not None:
                        outcomes[outcome] += 1
                    if ledger is not None and terminal_kind == "episode_end":
                        key = join_keys[episode]
                        if (key in matched_ledger or key not in ledger
                                or ledger[key]["outcome"] != outcome):
                            raise ValueError("resolved episode does not join the ledger outcome")
                        matched_ledger.add(key)
                    for event in stored.get("intervals", []):
                        name = event["event"]
                        intervals[name].append(event)
                    if include_episodes:
                        summaries.append(stored)
                else:
                    raise ValueError("unsupported record type: %r" % kind)
        if header is None:
            raise ValueError("empty sidecar")
        incomplete = sorted(set(monitors) - closed)
        if incomplete:
            raise ValueError("open episodes lack explicit terminal/shutdown/reset closure: %s" % incomplete)
        if ledger is not None and set(ledger) != matched_ledger:
            raise ValueError("ledger episodes are absent from the resolved sidecar closures")
        report["trace_sha256"] = sha256_file(path)
        report["status"] = "PASS"
    except (OSError, ValueError, TypeError, KeyError, StopIteration, json.JSONDecodeError) as exc:
        report["problems"].append("%s: %s" % (type(exc).__name__, exc))
    report.update(observations=observations, eligible_observations=eligible_observations,
                  episodes_closed=len(closed), closure_counts=dict(closure), outcomes=dict(outcomes),
                  observation_prefixes_replayed=prefix_checked,
                  intervals={name: {"count": len(events),
                                    "censored": sum(bool(e.get("censored")) for e in events),
                                    "boundaries": dict(Counter(e.get("boundary", "instantaneous")
                                                               for e in events)),
                                    "closed_duration_s": sum(e.get("duration_s", 0.0) for e in events
                                                             if not e.get("censored", False)),
                                    "censored_elapsed_s": sum(e.get("duration_s", 0.0) for e in events
                                                               if e.get("censored", False)),
                                    "runs_at_least_s": {str(t): sum(e.get("duration_s", 0.0) >= t
                                                                    for e in events)
                                                        for t in (2, 5, 10, 15)}}
                             for name, events in intervals.items()})
    if include_episodes:
        report["episodes"] = summaries
    if ledger_path:
        report["ledger"] = str(ledger_path)
        report["ledger_episodes_joined"] = len(matched_ledger)
    return report


audit_sidecar = validate_trace


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--episodes", type=Path, help="existing same-cell episode ledger for outcome joins")
    parser.add_argument("--device", choices=("cpu",), default="cpu")
    parser.add_argument("--expected-source-commit")
    parser.add_argument("--expected-checkpoint-sha256", default=POLICY_F_SHA256)
    parser.add_argument("--expected-lineage")
    parser.add_argument("--include-episodes", action="store_true")
    parser.add_argument("--out", type=Path, help="new report path outside any results tree")
    args = parser.parse_args(argv)
    if args.out and (args.out.exists() or "results" in args.out.resolve().parts):
        parser.error("refusing to overwrite a path or write under results")
    result = validate_trace(args.trace, args.episodes,
                            expected_source_commit=args.expected_source_commit,
                            expected_checkpoint_sha256=args.expected_checkpoint_sha256,
                            expected_lineage=args.expected_lineage, include_episodes=args.include_episodes)
    payload = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.out:
        with args.out.open("x") as stream:
            stream.write(payload)
    print(payload, end="")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
