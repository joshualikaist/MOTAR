#!/usr/bin/env python3
"""Verify published BOIA status against pinned research Git objects; read-only."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
SNAPSHOT = 'fde21cdf969a7df380aa1c4deddc1824cb524506'

def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--research', required=True, type=Path)
    args = parser.parse_args()
    record = json.loads((ROOT / 'docs/status/data/boia-progress.json').read_text())
    if record['research_snapshot'] != SNAPSHOT:
        raise SystemExit('progress record does not use the pinned snapshot')
    if git(args.research, 'cat-file', '-e', SNAPSHOT + '^{commit}') is None:
        raise SystemExit('pinned research commit is unavailable')
    checked = 0
    for name, item in record['sources'].items():
        raw = git(args.research, 'show', item['snapshot_commit'] + ':' + item['path'])
        digest = hashlib.sha256(raw).hexdigest()
        if digest != item['sha256']:
            raise SystemExit('source digest mismatch: ' + name)
        checked += 1
    if record['q02b']['review_commit'] != SNAPSHOT:
        raise SystemExit('Q02-B/Q03 review cutoff mismatch')
    if record['q02b']['review_findings'] != {'p0': 0, 'p1': 2, 'p2': 9}:
        raise SystemExit('review finding count mismatch')
    if record['latest_public_cpu'] != {
        'run': 3177, 'failures': 0, 'errors': 4, 'skipped': 10,
        'aggregate': 'FAIL', 'known_errors': 'Warp DetectReduction: vec3f argument conversion'
    }:
        raise SystemExit('latest PUBLIC_CPU record mismatch')
    print(json.dumps({
        'status': 'PASS',
        'snapshot': SNAPSHOT,
        'sources_checked': checked,
        'q02b_q03_review': 'FAIL_HOLD: 0 P0, 2 P1, 9 P2',
        'latest_public_cpu': record['latest_public_cpu'],
    }, indent=2))

if __name__ == '__main__':
    main()
