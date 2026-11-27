#!/usr/bin/env python3
"""SL1D-C02 sensor-only subprocess (S-L1D amendment 1). Run only through the watchdogged runner.

Reuses the committed S-L1D sensor launcher unchanged; only the scene list is replaced by the
amendment's 13 scenes (no EQ-1D/S-L1A/S-L1B scenes).
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
import s_l1d_launcher as d  # noqa: E402  (imports isaacgym before torch)
import torch  # noqa: E402

AMENDMENT = json.loads((ROOT / 'docs/plans/s_l1d_v1_amendment1.json').read_text())
MODES = tuple(AMENDMENT['run']['sensor_modes'])


def process_spec(pid):
    mode = pid.split('_', 1)[1] if pid.startswith('sensors_') else None
    if mode not in MODES:
        raise ValueError('Unknown process ' + pid)
    return {'id': pid, 'batch': 'SENSORS', 'mode': mode, 'num_envs': AMENDMENT['run']['sensor_num_envs'],
            'seed': AMENDMENT['run']['sensor_seed']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--process', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args(argv)
    spec = process_spec(args.process)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    d.eq.PLAN = copy.deepcopy(d.eq.PLAN)
    d.eq.PLAN['D2']['scenes'] = []
    d.S_L1A_PLAN = {'extra_scenes': []}
    d.S_L1B_PLAN = {'extra_scenes': copy.deepcopy(AMENDMENT['scenes'])}
    files, bad = d.run_sensors(spec, out)
    d.tel.write_json(out / 'process_done.json', {'process': spec['id'], 'process_spec': spec, 'files': files,
        'nan_fields': bad, 'wall_s': time.time() - t0, 'peak_allocated_bytes': int(torch.cuda.max_memory_allocated())})
    return 3 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
