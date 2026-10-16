#!/usr/bin/env python3
"""CPU-only precision audit; never imports the simulator or launches an experiment.

Reports to stdout by default. Historical packets are incomplete inputs, so replaying
them is a diagnostic, never a motion-validity PASS or a reconstruction of the GPU run.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]
SYNTHETIC_DENSITIES = (0, 40, 70, 80, 115, 120, 160, 205)
MEASURED_DENSITIES = (70, 115, 160, 205)
DT = 0.1


def load_gate_contract():
    path = ROOT / "motar_eval/contracts.py"
    spec = importlib.util.spec_from_file_location("_mt_p1_cpu_gate_contract", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.MotionLimits(), module.GateTolerances()


LIMITS, TOLERANCES = load_gate_contract()


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_motion():
    path = ROOT / "aerial_gym/task/navrl_task/target_motion.py"
    spec = importlib.util.spec_from_file_location("_mt_p1_cpu_motion", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def reference_distances(a, b):
    """Float64 subtraction oracle, rounded to the caller dtype; no matmul/cdist."""
    return (a.double().unsqueeze(-2) - b.double().unsqueeze(-3)).square().sum(-1).sqrt().to(a.dtype)


def point_rect_distance(point, center, half):
    return math.hypot(*(max(0.0, abs(p - c) - h) for p, c, h in zip(point, center, half)))


def swept_rect_distance(p0, p1, center, half):
    """Independent 2D segment-to-closed-rectangle distance, including corner cuts."""
    enter, leave = 0.0, 1.0
    direction = [b - a for a, b in zip(p0, p1)]
    intersects = True
    for i in range(2):
        low, high = center[i] - half[i], center[i] + half[i]
        if abs(direction[i]) < 1e-15:
            if not low <= p0[i] <= high:
                intersects = False
                break
        else:
            a, b = (low - p0[i]) / direction[i], (high - p0[i]) / direction[i]
            enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
            if enter > leave:
                intersects = False
                break
    if intersects:
        return 0.0
    values = [point_rect_distance(p0, center, half), point_rect_distance(p1, center, half)]
    length2 = sum(v * v for v in direction)
    for sx in (-1, 1):
        for sy in (-1, 1):
            corner = [center[0] + sx * half[0], center[1] + sy * half[1]]
            u = (sum((corner[i] - p0[i]) * direction[i] for i in range(2)) / length2
                 if length2 else 0.0)
            u = min(1.0, max(0.0, u))
            values.append(math.dist(corner, [p0[i] + u * direction[i] for i in range(2)]))
    return min(values)


def kinematic_excess(old_xy, new_xy, old_v, new_v, nominal, previous_displacement=None):
    speed = math.hypot(*new_v)
    accel = math.dist(old_v, new_v) / DT
    displacement = [(b - a) / DT for a, b in zip(old_xy, new_xy)]
    correction = math.dist([new_xy[i] - old_xy[i] for i in range(2)],
                           [v * DT for v in new_v])
    turn = None
    if (math.hypot(*old_v) > LIMITS.heading_valid_speed_mps + TOLERANCES.g1_heading_band_mps
            and speed > TOLERANCES.g1_speed_floor_mps):
        cross = old_v[0] * new_v[1] - old_v[1] * new_v[0]
        dot = sum(a * b for a, b in zip(old_v, new_v))
        turn = abs(math.degrees(math.atan2(cross, dot))) / DT
    rel = 1.0 + TOLERANCES.g1_relative
    out = {"speed": speed - min(LIMITS.speed_max_mps, nominal) * rel - TOLERANCES.g1_speed_floor_mps,
           "acceleration": accel - LIMITS.accel_max_mps2 * rel - TOLERANCES.g1_speed_floor_mps / DT,
           "turn": (turn - LIMITS.turn_rate_max_deg_s * rel if turn is not None else None),
           "position": correction - TOLERANCES.g2_position_m(),
           "displacement_speed": math.hypot(*displacement) - LIMITS.speed_max_mps - TOLERANCES.g2_speed_mps(DT),
           "displacement_acceleration": None, "displacement_turn": None}
    if previous_displacement is not None:
        out["displacement_acceleration"] = (math.dist(previous_displacement, displacement) / DT
                                            - LIMITS.accel_max_mps2 - TOLERANCES.g2_accel_mps2(DT))
        vmin = min(math.hypot(*previous_displacement), math.hypot(*displacement))
        if vmin >= TOLERANCES.g2_turn_min_speed_mps:
            a, b = previous_displacement, displacement
            angle = abs(math.atan2(a[0] * b[1] - a[1] * b[0], sum(x*y for x, y in zip(a, b))))
            out["displacement_turn"] = (angle / DT - LIMITS.turn_rate_max_rad_s
                                         - TOLERANCES.g2_turn_rad_s(vmin, DT))
    return out


def execute(motion, xy, velocity, desired, nominal, bars, sign, backend, oracle=False):
    n = xy.shape[0]
    full = lambda value: torch.full((n,), value, dtype=xy.dtype, device="cpu")
    original = torch.cdist
    try:
        if oracle:
            torch.cdist = reference_distances
        return motion.bounded_drone_target_step(
            xy, velocity, desired, nominal, DT, bars,
            torch.full_like(xy, 0.5), torch.full_like(xy, 39.5), 0.77,
            sign, full(4.0), full(math.radians(150.0)), 1.0,
            return_certificate=True, distance_backend=backend)
    finally:
        torch.cdist = original


def synthetic_audit(motion, steps=80):
    """Deterministic safe-start fixtures, not production scene sampling or rollouts."""
    rows = []
    for density in SYNTHETIC_DENSITIES:
        # One obstacle challenges steering; other bars leave the tested corridor clear.
        centers = ([[14.0, 20.0]] if density else [])
        centers += [[1.0 + 2.0 * (i % 19), 1.0 + 0.6 * (i // 19)]
                    for i in range(max(0, density - len(centers)))]
        bars = torch.tensor(centers, dtype=torch.float32).reshape(1, density, 2).repeat(2, 1, 1)
        xy = torch.tensor([[10.0, 20.0], [10.0, 20.0]], dtype=torch.float32)
        velocity = torch.zeros_like(xy)
        nominal = torch.full((2,), 1.5)
        signs = torch.tensor([1.0, -1.0])
        previous_displacement = [None, None]
        violations = dict.fromkeys(("speed", "acceleration", "turn", "position",
                                    "displacement_speed", "displacement_acceleration", "displacement_turn",
                                    "swept_clearance", "wall", "nonfinite",
                                    "oracle_ranking", "oracle_position"), 0)
        minimum = None
        infeasible = 0
        for _ in range(steps):
            desired = torch.tensor([[1.5, 0.0], [1.5, 0.0]])
            out = execute(motion, xy, velocity, desired, nominal, bars, signs, "direct_l2_v1")
            oracle = execute(motion, xy, velocity, desired, nominal, bars, signs,
                             "legacy_cdist", oracle=True)
            violations["oracle_ranking"] += int((out[4]["selected_index"] != oracle[4]["selected_index"]).sum())
            violations["oracle_position"] += int(((out[0] - oracle[0]).norm(dim=1) > 1e-6).sum())
            infeasible += int((~out[3]).sum())
            for i in range(2):
                old, new = xy[i].tolist(), out[0][i].tolist()
                values = kinematic_excess(old, new, velocity[i].tolist(), out[1][i].tolist(), 1.5,
                                          previous_displacement[i])
                previous_displacement[i] = [(b - a) / DT for a, b in zip(old, new)]
                for key, excess in values.items():
                    violations[key] += excess is not None and excess > 0.0
                violations["nonfinite"] += not all(math.isfinite(v) for v in new + out[1][i].tolist())
                violations["wall"] += not all(0.5 <= x <= 39.5 for x in new)
                if centers:
                    clearance = min(swept_rect_distance(old, new, c, [0.3, 0.3]) for c in centers)
                    minimum = clearance if minimum is None else min(minimum, clearance)
                    violations["swept_clearance"] += clearance < 0.15 - 1e-6
            xy, velocity = out[0], out[1]
        rows.append({"bars": density, "environment_steps": 2 * steps,
                     "violations": violations, "infeasible_first_steps": infeasible,
                     "minimum_swept_surface_clearance_m": minimum})
    return {"status": "CPU_SYNTHETIC_PASS" if all(not any(r["violations"].values()) for r in rows)
            else "CPU_SYNTHETIC_FAIL", "rows": rows,
            "scope": "two turn-sign safe-start fixtures per density; no simulator or GPU validity claim",
            "actual_experiment_densities": list(MEASURED_DENSITIES),
            "synthetic_stress_densities": [0, 40, 80, 120, 160]}


def packet_audit(motion, root):
    root = Path(root)
    reference_path = root / "tf32_replay.json"
    reference = json.loads(reference_path.read_text())
    config = reference.get("executor_config", {})
    totals = {"packets": 0, "unsafe_start_center_clearance": 0, "unsafe_executed_sweep": 0,
              "recorded_task_flag_true": 0, "kinematic_violations": 0,
              "cpu_direct_oracle_ranking_mismatches": 0}
    hypotheses = {str(sign): {"matches_executed_position": 0, "immediate_feasible": 0,
                             "selected_index_histogram": {}} for sign in (1, -1)}
    files, problems = {}, []
    expected_config = {"bar_slots": 160, "dt_s": DT, "filler_bar_xy": -1000.0,
                       "lookahead_s": 1.0, "max_accel": 4.0, "max_turn_rate_deg": 150.0}
    if config != expected_config:
        raise ValueError("historical replay executor assumptions differ from the declared CPU hypotheses")
    for relative, expected in sorted(reference["packet_files"].items()):
        path = root / relative
        files[relative] = sha256(path)
        if files[relative] != expected:
            problems.append("packet SHA mismatch: " + relative)
        for line in path.read_text().splitlines():
            packet = json.loads(line)
            if (packet.get("bounds_lo") != [0.5, 0.5] or packet.get("bounds_hi") != [39.5, 39.5]
                    or packet.get("planner_clearance_m") != 0.77
                    or not packet.get("context", {}).get("episode_key", "").startswith("s4190/b160/")
                    or "160bars__seed4190" not in packet.get("context", {}).get("cell_id", "")):
                raise ValueError("historical packet does not satisfy the declared reconstruction assumptions")
            totals["packets"] += 1
            centers, half = packet["bars_within_3m"]["centre"], packet["bars_within_3m"]["half"]
            old, new = packet["old_xy"], packet["new_xy"]
            totals["unsafe_start_center_clearance"] += min(math.dist(old, c) for c in centers) < packet["planner_clearance_m"]
            totals["unsafe_executed_sweep"] += min(swept_rect_distance(old, new, c, h)
                                                   for c, h in zip(centers, half)) < 0.15
            totals["recorded_task_flag_true"] += packet["task_step_feasible_flag"]
            positions = packet.get("recent_positions", [])
            previous = ([(b - a) / DT for a, b in zip(positions[-2], positions[-1])]
                        if len(positions) >= 2 else None)
            excess = kinematic_excess(old, new, packet["old_v"], packet["new_v"], packet["speed_mps"], previous)
            totals["kinematic_violations"] += any(v is not None and v > 0 for v in excess.values())
            bars = torch.full((1, 160, 2), -1000.0)
            bars[0, :len(centers)] = torch.tensor(centers)
            row = lambda value: torch.tensor([value], dtype=torch.float32)
            for sign in (1, -1):
                args = (motion, row(old), row(packet["old_v"]), row(packet["desired_v"]),
                        row(packet["speed_mps"]), bars, row(float(sign)))
                out = execute(*args, "direct_l2_v1")
                oracle = execute(*args, "legacy_cdist", oracle=True)
                selected = int(out[4]["selected_index"][0])
                hyp = hypotheses[str(sign)]
                hyp["matches_executed_position"] += math.dist(out[0][0].tolist(), new) < 1e-4
                hyp["immediate_feasible"] += bool(out[3][0])
                hist = hyp["selected_index_histogram"]
                hist[str(selected)] = hist.get(str(selected), 0) + 1
                totals["cpu_direct_oracle_ranking_mismatches"] += selected != int(oracle[4]["selected_index"][0])
    if totals["packets"] != 606:
        problems.append("expected the frozen 606 packets")
    historical = reference["replay"]["cuda_tf32_on_batch2"]
    return {"status": "INCOMPLETE_INPUTS" if not problems else "INVALID_INPUTS",
            "problems": problems, "totals": totals, "cpu_turn_sign_hypotheses": hypotheses,
            "input_sha256": files, "historical_replay_sha256": sha256(reference_path),
            "historical_gpu_measurement_NOT_RECOMPUTED": {
                "matches_executed": historical["matches_executed"],
                "packets": historical["packets"],
                "unresolved_count": historical["packets"] - historical["matches_executed"],
                "unresolved_status": "UNRESOLVED_INPUTS",
                "unresolved_packet_ids": "NOT_RECORDED: frozen replay stores aggregate tallies only"},
            "validated_reconstruction_assumptions": dict(expected_config, bounds_lo=[0.5, 0.5],
                                                          bounds_hi=[39.5, 39.5], clearance_m=0.77),
            "missing_inputs": ["full ordered bar tensor and original obstacle indices",
                               "turn_sign and complete waypoint/avoidance state",
                               "original batched environment identities and tensor strides/order",
                               "all original candidate distances/scores and selected index"],
            "interpretation": "unsafe starts cannot certify safe recovery; local-bar/filler and both-sign CPU hypotheses are not exact historical trajectory replay"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("cpu",), default="cpu")
    parser.add_argument("--suite", choices=("synthetic", "packets", "all"), default="synthetic")
    parser.add_argument("--packets-root", type=Path)
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--out", type=Path, help="new report only; existing paths and results/ are refused")
    args = parser.parse_args(argv)
    if args.steps < 1:
        parser.error("--steps must be positive")
    if args.suite in ("packets", "all") and args.packets_root is None:
        parser.error("--packets-root is required for packet diagnostics")
    if args.out and (args.out.exists() or "results" in args.out.resolve().parts):
        parser.error("output must be a new path outside the immutable results tree")
    # Torch package import does not start Isaac Gym. Never query or initialize CUDA here.
    before = torch.random.get_rng_state().clone()
    previous_threads = torch.get_num_threads()
    torch.set_num_threads(1)
    try:
        motion = load_motion()
        report = {"schema": "motar.mt-p1-cpu-precision.v1", "device": "cpu",
                  "simulator_started": False, "gpu_started": False,
                  "motion_source_sha256": sha256(ROOT / "aerial_gym/task/navrl_task/target_motion.py"),
                  "verification_tool_sha256": sha256(__file__),
                  "gate_contract_sha256": sha256(ROOT / "motar_eval/contracts.py"),
                  "gate_tolerances": TOLERANCES.describe(DT),
                  "command_arguments": list(argv if argv is not None else sys.argv[1:])}
        if args.suite in ("synthetic", "all"):
            report["synthetic"] = synthetic_audit(motion, args.steps)
        if args.suite in ("packets", "all"):
            report["packets"] = packet_audit(motion, args.packets_root)
        report["torch_cpu_rng_unchanged"] = torch.equal(before, torch.random.get_rng_state())
        payload = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
        if args.out:
            with args.out.open("x") as stream:
                stream.write(payload)
        print(payload, end="")
        failed = not report["torch_cpu_rng_unchanged"]
        failed |= report.get("synthetic", {}).get("status") == "CPU_SYNTHETIC_FAIL"
        failed |= report.get("packets", {}).get("status") == "INVALID_INPUTS"
        return 1 if failed else 0
    finally:
        torch.set_num_threads(previous_threads)


if __name__ == "__main__":
    sys.exit(main())
