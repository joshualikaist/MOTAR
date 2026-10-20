"""Finite independent falsification, not a continuum/physical safety certificate."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import time

import torch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from motar_eval.boia.geometry import capsule_in_free_model
from motar_eval.boia.types import FreeBoundary2D, FreeContext


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rng = random.Random(20261020)
    n, bins = 96, 72
    step = 2 * math.pi / bins
    a, b, radii, body = [], [], [], []
    for _ in range(n):
        field = [3.] * bins
        for j in rng.sample(range(bins), 4):
            field[j] = rng.uniform(.25, .9)
        radii.append(field)
        a.append([rng.uniform(-1, 1), rng.uniform(-1, 1)])
        b.append([rng.uniform(-1, 1), rng.uniform(-1, 1)])
        body.append(rng.uniform(.02, .25))
    dtype = torch.float64
    ctx = FreeContext(torch.zeros(n, 3, dtype=dtype), torch.zeros(n, dtype=dtype),
        torch.tensor(radii, dtype=dtype), math.pi - torch.arange(bins, dtype=dtype) * step, 12.)
    started = time.perf_counter()
    result = capsule_in_free_model(torch.tensor(a, dtype=dtype), torch.tensor(b, dtype=dtype),
                                   FreeBoundary2D(ctx), torch.tensor(body, dtype=dtype))
    elapsed = time.perf_counter() - started
    violations, checks = [], 0
    # Independent scalar polar membership. This samples candidate counterexamples;
    # absence of one is NOT the proof used by the analytic implementation.
    for e in range(n):
        if not result.model_safe[e]:
            continue
        for ti in range(33):
            t = ti / 32
            cx = a[e][0] * (1 - t) + b[e][0] * t
            cy = a[e][1] * (1 - t) + b[e][1] * t
            for j in range(64):
                phi = 2 * math.pi * j / 64
                x, y = cx + body[e] * math.cos(phi), cy + body[e] * math.sin(phi)
                index = math.floor(((math.pi - math.atan2(y, x)) / step) % bins + .5) % bins
                checks += 1
                if math.hypot(x, y) > radii[e][index] + 1e-11:
                    violations.append([e, ti, j, x, y])
    report = {'kind': 'FINITE_FALSIFICATION_NOT_CONTINUUM_PROOF', 'seed': 20261020,
        'capsules': n, 'model_safe': int(result.model_safe.sum()), 'witness_checks': checks,
        'false_safe_witnesses': violations, 'query_seconds': elapsed,
        'physical_qualification': 'PHYSICAL_QUALIFICATION_NOT_AVAILABLE',
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in (ROOT / 'motar_eval/boia/geometry.py', ROOT / 'motar_eval/boia/types.py')}}
    with args.output.open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'source_sha256'}, indent=2))
    if violations:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
