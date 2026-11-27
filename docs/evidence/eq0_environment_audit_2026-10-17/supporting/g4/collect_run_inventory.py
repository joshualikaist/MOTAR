"""EQ-0 read-only inventory; no simulator import or production metric calls."""
from pathlib import Path
import collections
import hashlib
import json
import tempfile

ROOT = Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2')
OUT = Path(tempfile.mkdtemp(prefix='motar-eq0-g4-')) / 'run_inventory.json'
print('Fresh scratch output:', OUT)
conds = collections.defaultdict(collections.Counter)
envs = collections.defaultdict(collections.Counter)
meta = collections.defaultdict(collections.Counter)
crashes = collections.Counter()
presence = collections.Counter()
tf32 = collections.Counter()
ledger = collections.Counter()
outcomes = collections.Counter()
population_differences = []
bindings = {}
receipts = sorted(ROOT.glob('*/*/*/receipt.json'))
for receipt in receipts:
    cell = receipt.parent
    r = json.loads(receipt.read_text())
    result = json.loads((cell / 'result.json').read_text())
    tr = json.loads((cell / 'occlusion_trace_summary.json').read_text())
    ks = json.loads((cell / 'kroc2027_summary.json').read_text())
    rs = json.loads((cell / 'recovery_summary.json').read_text())
    for key, value in result['condition'].items():
        conds[key][json.dumps(value, sort_keys=True)] += 1
    for key, value in r['env_snapshot_navrl'].items():
        envs[key][json.dumps(value, sort_keys=True)] += 1
    for key in ['dt_s', 'detection_latency_s', 'render_alignment', 'sensor_capture_time',
                'gpu_trajectory_noninterference', 'target_radius_m', 'camera']:
        meta[key][json.dumps(tr['metadata'].get(key), sort_keys=True)] += 1
    for key, value in result['crash_causes'].items():
        if isinstance(value, int):
            crashes[key] += value
    tf32[json.dumps(ks.get('matmul_precision', 'NOT_RECORDED'), sort_keys=True)] += 1
    presence['has_runtime_fingerprint'] += ('runtime_fingerprint' in r or 'runtime_fingerprint' in result)
    presence['safe_abort_' + str(rs['safe_abort'])] += 1
    presence['noninterference_' + str(rs['gpu_trajectory_noninterference'])] += 1
    for line in (cell / 'episodes.jsonl').read_text().splitlines():
        row = json.loads(line)
        ledger['all_rows'] += 1
        ledger['quota' if row['in_quota'] else 'nonprimary_tail'] += 1
        outcomes[('quota/' if row['in_quota'] else 'tail/') + row['outcome']] += 1
    quota = collections.Counter(json.loads(line)['outcome'] for line in
        (cell / 'episodes.jsonl').read_text().splitlines() if json.loads(line)['in_quota'])
    legacy = collections.Counter(json.loads(line)['outcome'] for line in
        (cell / 'episodes.jsonl').read_text().splitlines() if json.loads(line)['in_legacy_window'])
    aggregate = {key: result['outcome'][field] for key, field in
                 [('capture', 'captured'), ('crash', 'crash'), ('timeout', 'timeout')]}
    for key, value in aggregate.items():
        outcomes['legacy_result/' + key] += value
    assert all(aggregate[key] == legacy.get(key, 0) for key in aggregate)
    if any(aggregate[key] != quota.get(key, 0) for key in aggregate):
        population_differences.append({'cell': str(cell.relative_to(ROOT)),
                                      'quota': dict(quota), 'legacy_result': aggregate})
    for name in ['receipt.json', 'result.json', 'occlusion_trace_summary.json',
                 'recovery_summary.json', 'kroc2027_summary.json']:
        file = cell / name
        bindings[str(file.relative_to(ROOT))] = hashlib.sha256(file.read_bytes()).hexdigest()
log = []
for line in (ROOT / 'moving/A3/E2_obstacle_aware__70bars__seed4301/cell.log').read_text().splitlines():
    if len(line) < 300 and any(token in line for token in [
            'PyTorch version', 'Warp 1.', 'CUDA Toolkit', 'Using GPU PhysX', 'Physics Engine:',
            'Physics Device:', 'GPU Pipeline:', 'Robot mass:', 'use_warp      :',
            'sim env       :', 'config        :', 'num envs      :']):
        log.append(line)
report = {
    'schema': 'motar.eq0.run-inventory.v1',
    'measurement_source': '5ff285ad053f181c5a1c86dddb2462bb44e6327b',
    'raw_root': str(ROOT), 'cells': len(receipts),
    'condition_values': {key: dict(value) for key, value in conds.items()},
    'environment_values': {key: dict(value) for key, value in envs.items()
                           if len(value) <= 8 and not any(t in key for t in ['NONCE', 'SEED', 'JSON', 'RUN_ID'])},
    'trace_metadata_values': {key: dict(value) for key, value in meta.items()},
    'crash_causes_count_sum': dict(crashes), 'matmul_precision_values': dict(tf32),
    'record_presence': dict(presence), 'ledger_counts': dict(ledger),
    'outcome_populations': dict(outcomes),
    'quota_vs_legacy_result_differences': population_differences,
    'historical_log_excerpts': log, 'evidence_file_sha256': bindings,
    'note': 'Recorded-state inventory only; no simulator or production metric function invoked. '
            'Missing fingerprint fields remain missing; log versions are partial. '
            'result.json is a legacy aggregate snapshot with batch overshoot; '
            'confirmatory primary outcomes use in_quota ledgers, not that snapshot. '
            'Crash-cause and governor diagnostics belong to their snapshot population.',
}
OUT.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
print(json.dumps({key: report[key] for key in ['cells', 'ledger_counts', 'crash_causes_count_sum',
                                               'record_presence', 'matmul_precision_values']}, indent=2))
