"""Reconstruct the declared scalar fixture and time standalone CPU C02 only.

This is a receipt, not a simulator/flight test or a new mission-effect test.
The scalar test fixture is independent of production dynamics. No random inputs.
"""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import torch

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests')]
import test_boia_c02_safety as fixture
from motar_eval.boia.safety import screen_next_cycle
from motar_eval.boia.types import FreeContext, Rollout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    yaw = math.atan2(.2, .97) - math.radians(55)
    slant = math.hypot(.97, .2)
    rows = [[12.] * 72 for _ in range(4)]
    rows[2][25] = slant
    base = [max(0., min(rows[v][j] * math.cos(math.radians(e))
                       for v, e in enumerate((20., 10., 0., -10.))) - .1)
            for j in range(72)]
    contracted = [min(base[j], base[(j-1) % 72], base[(j+1) % 72]) for j in range(72)]
    ctx = replace(fixture.context(yaw=yaw), free_radius=torch.tensor([contracted], dtype=torch.float64))
    roll = fixture.scalar_roll((2., 0.), (0., 2.), yaw=yaw)
    obs = (torch.tensor([[[.97, .2]]], dtype=torch.float64), torch.ones(1, 1, dtype=torch.bool))
    result = fixture.run(roll, ctx, obs)
    counterexample = {
        'v0_mps': [2., 0.], 'world_command_mps': [0., 2.], 'obstacle_xy_m': [.97, .2],
        'yaw_rad': yaw, 'tau_s': .4, 'decision_s': .1, 'substep_s': .01,
        'legacy_hard_ok': bool(result.legacy_hard_ok), 'legacy_stop_ok': bool(result.legacy_stop_ok),
        'prefix_ok': bool(result.prefix_ok), 'backup_ok': bool(result.backup_ok),
        'status': int(result.status), 'status_name': 'MODEL_UNSAFE',
        'first_prefix_end_m': roll.pos[0, 0, 0, -1].tolist(),
        'first_prefix_velocity_mps': roll.vel[0, 0, 0, -1].tolist(),
        'backup_distance_m': float(result.backup_distance_m),
        'backup_min_clearance_m': float(result.backup_min_clearance_m),
        'independent_zero_input_limit_without_reaction_xy_m': [.78, .2],
        'obstacle_body_xy_at_limit_m': [math.cos(yaw)*.19, -math.sin(yaw)*.19],
        'scope': 'MATHEMATICAL_COUNTEREXAMPLE_NOT_A_SIMULATOR_INCIDENT',
    }
    assert (counterexample['legacy_hard_ok'] and counterexample['legacy_stop_ok']
            and counterexample['prefix_ok'] and not counterexample['backup_ok']
            and counterexample['status'] == 1)
    timings = []
    simple = fixture.scalar_roll((.6, 0.), (.6, 0.))
    for n, k in ((1, 1), (16, 20), (128, 20)):
        expand = lambda x: x.expand((n, k) + tuple(x.shape[2:])).clone()
        batch = Rollout(*(expand(x) for x in (simple.pos, simple.vel, simple.psi,
                       simple.step_pos, simple.step_vel, simple.step_psi, simple.governed_cmd)),
                       {name: expand(x) for name, x in simple.diagnostics.items()})
        bc = fixture.context(n=n)
        points, valid = fixture.empty_points(n)
        start = time.perf_counter()
        output = screen_next_cycle(batch, points, valid, bc, fixture.params())
        elapsed = time.perf_counter() - start
        assert bool(output.model_safe.all())
        timings.append({'N': n, 'K': k, 'H': 10, 'S': 10, 'boundary_bins': 72,
                        'valid_obstacle_points': 0, 'backup_samples': 64,
                        'dtype': 'float64', 'seconds': elapsed,
                        'model_safe': int(output.model_safe.sum()),
                        'scope': 'ONE_SHOT_STANDALONE_CPU_NOT_PLANNER_NOT_WCET'})
        if elapsed > 20:
            break
    report = {'kind': 'CPU_MODEL_RECEIPT_ONLY', 'counterexample': counterexample,
              'timings': timings, 'torch_version': torch.__version__, 'threads': torch.get_num_threads(),
              'cuda_visible_devices': '', 'physical_qualification': 'PHYSICAL_QUALIFICATION_NOT_AVAILABLE',
              'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted((ROOT / 'motar_eval/boia').glob('*.py'))}}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
