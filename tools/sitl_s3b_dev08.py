"""Prepare dev08's gain configuration without importing or starting the SITL runner.

Native is the default. --arm candidate explicitly selects the three-gain treatment.
This tool prints JSON only: it does not launch PX4, import the bridge/checkpoint,
write a world, or measure an episode. Runtime integration is a separate step.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = "tools/sitl_s3b_closed_loop.py"
BRIDGE = "tools/sitl_bridge.py"
REPORTS = {str(b): "docs/evidence/sitl_2026-11-05/s3b_dev/dev07_%d_s3b_report.json" % b
           for b in (70, 205)}
PINS = {
    RUNNER: "d170b0271f73822280d8c92af68ab2bb56e350bbdf598702c353d8b79f0284b4",
    BRIDGE: "f0031f114f4428ddd5f3b126067c80b1808071e1864402fb119b342259083db7",
    REPORTS["70"]: "e35d5d0f0909ef986c836bdbb3022f02b072fd0334f8744d4254f31144466bc8",
    REPORTS["205"]: "e7210f50730e3c4e266842d178f296f6b0187227e3cb5d86c0b6d3c13d7eaaee",
}
CANDIDATE = {"MPC_XY_VEL_P_ACC": 2.0, "MC_YAWRATE_P": 0.2, "MC_YAWRATE_I": 0.1}
CONSTANTS = {"DT": 0.1, "ODE_ITERS": 100, "ALT": 1.0, "MAX_STEPS": 600,
             "CAPTURE_M": 0.5, "FOOT_HALF": 0.14, "OOB_MARGIN": 1.0}


def literal_contract(source):
    """Read assignments as data; never execute historical source or its imports."""
    wanted = set(CONSTANTS) | {"PARAMS"}
    values = {}
    for node in ast.parse(source).body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in wanted:
                if target.id in values:
                    raise ValueError("duplicate contract assignment: " + target.id)
                values[target.id] = ast.literal_eval(node.value)
    if set(values) != wanted:
        raise ValueError("missing literal runner contract")
    if {k: values[k] for k in CONSTANTS} != CONSTANTS:
        raise ValueError("runner timing/termination contract changed")
    if not isinstance(values["PARAMS"], dict) or not values["PARAMS"]:
        raise ValueError("empty or invalid native parameter map")
    return values


def variant_params(native, arm="native"):
    """Return a new parameter map; keep the caller's native defaults untouched."""
    if arm not in ("native", "candidate"):
        raise ValueError("unknown arm: " + str(arm))
    params = dict(native)
    if arm == "candidate":
        params.update(CANDIDATE)
    return params


def prepare(root=ROOT, arm="native"):
    root = Path(root)
    sources = {}
    for rel, expected in PINS.items():
        data = (root / rel).read_bytes()
        actual = hashlib.sha256(data).hexdigest()
        if actual != expected:
            raise ValueError("historical input changed: " + rel)
        sources[rel] = data
    contract = literal_contract(sources[RUNNER].decode("utf-8"))
    native = contract.pop("PARAMS")
    candidate = variant_params(native, "candidate")
    delta = {k for k in set(native) | set(candidate) if native.get(k) != candidate.get(k)}
    if delta != set(CANDIDATE):
        raise ValueError("candidate must change exactly the three registered gains")
    cells, coverage_shortfalls = {}, []
    for bars, rel in REPORTS.items():
        report = json.loads(sources[rel])
        episodes = report["episodes"]
        if report["params"] != native or report["ode_iters"] != contract["ODE_ITERS"]:
            raise ValueError("dev07 report and native runner configuration disagree")
        if len(episodes) != 10 or [e["env"] for e in episodes] != list(range(10)):
            raise ValueError("dev07 episode selection changed")
        if any(e["target_source"] != "zero_action_cell" or e["target_frames"] <= 0
               or e["start_check"]["command_yaw"] != "truth" for e in episodes):
            raise ValueError("dev07 target/yaw contract changed")
        target_frames = {str(e["env"]): e["target_frames"] for e in episodes}
        coverage_shortfalls.extend({"bars": int(bars), "env": e["env"],
                                    "recorded_frames": e["target_frames"], "required_frames": 600}
                                   for e in episodes if e["target_frames"] < 600)
        cell = Path(report["cell"])
        if cell.parent.name != "full" or cell.name != "E2_obstacle_aware__%sbars__seed4501" % bars:
            raise ValueError("unexpected dev07 trace cell")
        cells[bars] = {
            "trace_cell": str(cell), "target_cell": str(cell.parent.parent / "zero" / cell.name),
            "envs": list(range(10)), "env_episode_index": 0,
            "historical_comparator": rel,
            "target_frames_in_historical_report": target_frames,
            "trace_payload_verified": False,
        }
    return {
        "unit": "S3b dev08 preparation v1", "status": "CONFIGURATION_PREPARED",
        "measurement_status": "NOT_RUN", "runtime_status": "NOT_IMPLEMENTED", "execution_supported": False,
        "input_readiness": "BLOCKED_INCOMPLETE_TARGET_TRACE" if coverage_shortfalls else "NOT_VERIFIED",
        "target_coverage_shortfalls": coverage_shortfalls,
        "scope": "static configuration and historical provenance only; no S3b acceptance",
        "selected_arm": arm, "selected_params": variant_params(native, arm),
        "native_params": native, "candidate_params": candidate,
        "gain_delta": {k: {"native": native.get(k, "NOT_EXPLICITLY_OVERRIDDEN"), "candidate": v}
                       for k, v in CANDIDATE.items()},
        "unchanged_constants": contract, "command_yaw": "truth",
        "historical_sha256": dict(PINS), "cells": cells,
        "runtime_requirements": [
            "new scoped dev08 execution approval and committed runtime source/preregistration",
            "strict trace/layout/env selection and full target coverage, no fallback or tail hold",
            "parameter echo/readback before takeoff; persist effective values including native yaw",
            "exclusive SITL ownership and cleanup restricted to owned descendants",
            "fresh output/world names, source/platform/checkpoint bindings, source freeze",
            "CUDA hidden, idle host, cores 6,7,12,13, nice10, time/memory bounds and host-fault checks",
            "10Hz simulator-time loop, telemetry freshness/stall checks and RTF reporting",
        ],
        "limits": [
            "native parameter equality is not episode/output identity",
            "historical dev07 is a descriptive comparator, not a concurrent randomized control",
            "S3a NOT_EQUIVALENT; replay target and MOTAR truth-rendered sensors remain",
            "live trace payloads, vehicle/template synchronization and real sensors are not qualified here",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=("native", "candidate"), default="native")
    args = parser.parse_args(argv)
    try:
        result = prepare(arm=args.arm)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, "dev08 preparation refused: %s\n" % exc)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
