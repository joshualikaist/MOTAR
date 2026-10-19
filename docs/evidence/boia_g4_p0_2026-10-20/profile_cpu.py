"""CPU-only one-decision instrumentation; never imports a simulator or checkpoint."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from unittest import mock

import torch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import planner, trace
from motar_eval.boia.types import BoiaParams
from tools.boia_replay import make_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=128)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.n <= 0:
        parser.error('n must be positive')
    task, perception, action = make_scene('clear', args.n)
    params = BoiaParams(tau_on=.1, r_bar_est=.2, R_camera_diag=(.04, .04, .04))
    rng = torch.Generator(device='cpu')
    rng.manual_seed(20261020)
    measured = {'trace': 0., 'canonical': 0., 'sha': 0.}
    counts = {'trace': 0, 'canonical': 0, 'sha': 0}
    originals = {'trace': trace.record, 'canonical': trace.canonical_input, 'sha': trace.sha256}

    def instrument(name):
        def call(*a, **kw):
            started = time.perf_counter()
            result = originals[name](*a, **kw)
            measured[name] += time.perf_counter() - started
            counts[name] += 1
            return result
        return call

    # Patches are scoped to this isolated measurement child and always restored.
    with mock.patch.object(trace, 'record', instrument('trace')), \
            mock.patch.object(trace, 'canonical_input', instrument('canonical')), \
            mock.patch.object(trace, 'sha256', instrument('sha')):
        started = time.perf_counter()
        chosen, decision = planner.BoiaPlanner(params).step(
            task, perception, action, torch.zeros(args.n, dtype=torch.bool), rng)
        total = time.perf_counter() - started
    assert torch.equal(chosen, action) and not decision.engaged.any()
    result = {'status': 'NOT_A_GPU_MEASUREMENT', 'device': 'cpu', 'dtype': str(action.dtype),
        'N': args.n, 'K': params.K, 'M': params.M, 'H': params.H, 'S': params.S,
        'dt_s': params.dt, 'threads': torch.get_num_threads(), 'repetitions': 1, 'warmup': 0,
        'seconds': {'total': total, 'core_excluding_trace': total - measured['trace'],
            'trace_total': measured['trace'], 'canonical_serialization': measured['canonical'],
            'sha256': measured['sha'],
            'trace_other': measured['trace'] - measured['canonical'] - measured['sha']},
        'instrumentation_calls': counts,
        'point_distance_operations_per_decision': args.n * params.K * params.M * params.H * 288,
        'rollout_substep_states': args.n * params.K * params.H * params.S,
        'interpretation': 'One instrumented cold synthetic decision. Core includes access/copy, geometry, dynamics, safety, belief, visibility, costs and arbitration. Trace includes canonicalization, pure-Python SHA and numpy copies; instrumentation overhead is not subtracted. No latency bound or GPU timing claim.',
        'measurement_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted((ROOT / 'motar_eval/boia').glob('*.py'))}}
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'source_sha256'}, indent=2))


if __name__ == '__main__':
    main()
