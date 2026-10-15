#!/usr/bin/env python3
"""Replay the KROC2027 violation packets under each matmul precision the evaluation can run with.

    python -B tools/kroc2027_packet_replay.py --packets results/kroc2027_moving_target_packets_v1 \
        --out results/kroc2027_moving_target_packets_v1/tf32_replay.json

Read-only: loads ``target_motion.py`` by path, reads ``kroc2027_violations.jsonl`` and writes one JSON.
It uses the GPU for a few seconds of tensor arithmetic and never starts the simulator.

rl_games' ``Runner.__init__`` sets ``torch.backends.cuda.matmul.allow_tf32 = True`` and
``torch.set_float32_matmul_precision('high')`` for the whole process, so every evaluation step runs
with TF32 matmul allowed. ``torch.cdist`` computes ||x||^2 + ||y||^2 - 2 x.y through a matmul once
either point set has more than 25 rows. This tool checks whether that explains the packets: the
executor reported a safe first step while the executed step ended inside the 0.77 m bar clearance.

Each packet is replayed four ways on CUDA (TF32 off/on, batch 1/2) and once on the CPU. The bars are
placed in a stride-13 root-state layout with 160 slots, as ``obs_dict["obstacle_position"]`` is; bars
beyond 3 m are not in the packets and their slots hold far filler bars. A shape scan then measures the
largest ``cdist`` error against the exact difference norm for the executor's call (73 candidates
against the bar count) and the spawn rule's call (one point against the bar count).
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
TARGET_MOTION = ROOT / "aerial_gym/task/navrl_task/target_motion.py"
TASK_CONFIG = ROOT / "aerial_gym/config/task_config/navrl_task_config.py"
SLOTS, FILLER = 160, -1000.0


def _load(name, path):
    """A module by path: importing the aerial_gym package would start Isaac Gym."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _precision(tf32):
    torch.backends.cuda.matmul.allow_tf32 = tf32
    torch.set_float32_matmul_precision("high" if tf32 else "highest")


def _replay(tm, packet, device, batch):
    """The executor's call on one packet, copied ``batch`` times; returns (xy, immediate_feasible)."""
    near = torch.tensor(packet["bars_within_3m"]["centre"], dtype=torch.float32)
    root = torch.full((batch, 1 + SLOTS, 13), FILLER, device=device)
    root[:, 1 : 1 + len(near), 0:2] = near.to(device)
    bars = root[:, 1 : 1 + SLOTS, 0:2]
    row = lambda v: torch.tensor([v] * batch, dtype=torch.float32, device=device)  # noqa: E731
    full = lambda x: torch.full((batch,), float(x), device=device)  # noqa: E731
    xy, _, _, immediate = tm.bounded_drone_target_step(
        row(packet["old_xy"]), row(packet["old_v"]), row(packet["desired_v"]), full(packet["speed_mps"]),
        0.1, bars, row(packet["bounds_lo"]), row(packet["bounds_hi"]), packet["planner_clearance_m"],
        full(1.0), full(tm_config["max_accel"]), full(math.radians(tm_config["max_turn_rate_deg"])),
        tm_config["lookahead_s"], None)
    return xy[0].cpu().tolist(), bool(immediate[0].item())


def _tally(tm, packets, device, batch):
    out = {"packets": len(packets), "immediate_feasible": 0, "matches_executed": 0,
           "agrees_with_task_flag": 0, "matches_recorder_replay": 0}
    for packet in packets:
        xy, immediate = _replay(tm, packet, device, batch)
        out["immediate_feasible"] += immediate
        out["matches_executed"] += math.dist(xy, packet["new_xy"]) < 1e-4
        out["agrees_with_task_flag"] += immediate == packet["task_step_feasible_flag"]
        out["matches_recorder_replay"] += math.dist(xy, packet["replay"]["xy"]) < 1e-4
    return out


def _shape_scan(bar_counts, points, batch=128, trials=3, seed=0):
    """Largest |cdist - exact| (m) for coordinates uniform in [0, 40] m, TF32 off and on."""
    generator = torch.Generator(device="cuda").manual_seed(seed)
    out = {}
    for tf32 in (False, True):
        _precision(tf32)
        for bars in bar_counts:
            worst = 0.0
            for _ in range(trials):
                y = torch.rand(batch, bars, 2, device="cuda", generator=generator) * 40.0
                x = torch.rand(batch, points, 2, device="cuda", generator=generator) * 40.0
                exact = (x.unsqueeze(2) - y.unsqueeze(1)).norm(dim=-1)
                worst = max(worst, float((torch.cdist(x, y) - exact).abs().max().item()))
            out["tf32_%s/bars%d" % ("on" if tf32 else "off", bars)] = round(worst, 4)
    return out


def _rl_games_sets_tf32():
    try:
        import rl_games.torch_runner as runner
    except ImportError:
        return "NOT_RECORDED"
    path = Path(runner.__file__)
    text = path.read_text()
    return {"file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "sets_matmul_allow_tf32": "torch.backends.cuda.matmul.allow_tf32 = True" in text,
            "sets_float32_matmul_precision_high": "torch.set_float32_matmul_precision('high')" in text}


tm_config = {}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--packets", required=True, help="packet run folder (KROC27-MT-P01)")
    parser.add_argument("--out", required=True, help="JSON to write")
    args = parser.parse_args(argv)

    packets = []
    files = sorted(Path(args.packets).glob("*/*/kroc2027_violations.jsonl"))
    for path in files:
        packets += [json.loads(line) for line in path.read_text().splitlines()]
    if not packets:
        raise SystemExit("no packets under %s" % args.packets)
    tm = _load("_kroc2027_replay_target_motion", TARGET_MOTION)
    target_cfg = _load("_kroc2027_replay_task_config", TASK_CONFIG).task_config.target_motion
    tm_config.update(max_accel=float(target_cfg.max_accel),
                     max_turn_rate_deg=float(target_cfg.max_turn_rate_deg),
                     lookahead_s=float(target_cfg.avoidance_lookahead_s))

    report = {
        "purpose": "test whether TF32 matmul in torch.cdist explains the KROC27-MT-P01 packets",
        "packet_files": {str(p.relative_to(args.packets)): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in files},
        "target_motion_sha256": hashlib.sha256(TARGET_MOTION.read_bytes()).hexdigest(),
        "executor_config": dict(tm_config, dt_s=0.1, bar_slots=SLOTS, filler_bar_xy=FILLER),
        "torch": torch.__version__,
        "rl_games_runner": _rl_games_sets_tf32(),
        "packets": len(packets),
        "task_flag_true": sum(bool(p["task_step_feasible_flag"]) for p in packets),
    }
    _precision(False)
    report["replay"] = {"cpu_fp32": _tally(tm, packets, "cpu", 1)}
    if torch.cuda.is_available():
        report["device"] = torch.cuda.get_device_name(0)
        for tf32 in (False, True):
            for batch in (1, 2):
                _precision(tf32)
                key = "cuda_tf32_%s_batch%d" % ("on" if tf32 else "off", batch)
                report["replay"][key] = _tally(tm, packets, "cuda", batch)
        report["cdist_max_abs_error_m"] = {
            "executor_73_candidates": _shape_scan((70, 115, 160, 205, 72, 116, 156, 164), 73),
            "spawn_rule_1_point": _shape_scan((70, 115, 160, 205), 1),
        }
        _precision(False)
    else:
        report["device"] = "NOT_RECORDED"
        report["cdist_max_abs_error_m"] = "NOT_RECORDED"
    Path(args.out).write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: report[k] for k in ("packets", "task_flag_true", "replay")}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
