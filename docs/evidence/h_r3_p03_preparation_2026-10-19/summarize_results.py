#!/usr/bin/env python3
"""CPU-only post-run receipt; never writes to the qualification/raw directory.

Run from the repository root with --raw and --out. Reads the frozen analysis and
independently hashes its raw inventory. Additional terminal follow-up is explicitly
post-hoc descriptive and cannot alter the registered primary analysis or GO gates.
"""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path
from statistics import NormalDist

import numpy as np


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    raw, out = args.raw.resolve(), args.out.resolve()
    assert raw != out and raw not in out.parents
    out.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((raw / 'RAW_MANIFEST.json').read_text())
    members = manifest['members']
    names = [m['path'] for m in members]
    assert len(names) == len(set(names))
    checked_bytes = 0
    for member in members:
        relative = Path(member['path'])
        assert not relative.is_absolute() and '..' not in relative.parts
        path = raw / relative
        assert raw in path.resolve().parents
        assert not any(p.is_symlink() for p in (path, *path.parents) if p != raw.parent)
        assert path.is_file() and path.stat().st_size == member['bytes']
        assert digest(path) == member['sha256'], member['path']
        checked_bytes += member['bytes']
    actual = {str(p.relative_to(raw)) for p in raw.rglob('*') if p.is_file()}
    assert actual == set(names) | {'RAW_MANIFEST.json'}, actual ^ set(names)
    a = json.loads((raw / 'qualification_analysis.json').read_text())
    r = json.loads((raw / 'run_report.json').read_text())
    assert r['status'] == 'COMPLETED' and a['verdict'] == 'GO_QUALIFICATION_ONLY'
    assert not a['failures'] and all(p['gate']['PASS'] for p in r['processes'])
    assert len(r['processes']) == 11
    summary = {k: v for k, v in a.items() if k != 'events'}
    summary['raw_analysis_sha256'] = digest(raw / 'qualification_analysis.json')
    save(out / 'qualification_summary.json', summary)
    shutil.copyfile(raw / 'RAW_MANIFEST.json', out / 'RAW_MANIFEST.json')
    sizes = {p['id']: sum(f.stat().st_size for f in (raw / p['id']).rglob('*') if f.is_file())
             for p in r['processes']}
    resource = {k: r[k] for k in ('run_id', 'source_commit', 'status', 'wall_s',
                                'gpu_process_seconds', 'output_bytes', 'free_disk_gb_after')}
    resource.update(raw_path=str(raw), final_output_bytes=sum(p.stat().st_size for p in raw.rglob('*') if p.is_file()),
                    peak_device_mib=max(p['peak_gpu_mib'] for p in r['processes']),
                    memory_scope='Whole-device sampled use, including existing display/background processes',
                    processes=[{k: p[k] for k in ('id', 'returncode', 'wall_s', 'loop_wall_s', 'peak_gpu_mib', 'snapshots')}
                               for p in r['processes']],
                    per_process_output_bytes=sizes,
                    runtime_wall_reduction_vs_audit=1-next(p['wall_s'] for p in r['processes'] if p['id']=='long_B0')/
                        next(p['wall_s'] for p in r['processes'] if p['id']=='long_audit'),
                    runtime_storage_reduction_vs_audit=1-sizes['long_B0']/sizes['long_audit'],
                    performance_scope='One matched seed/process pair; descriptive, not a general benchmark')
    save(out / 'resource_report.json', resource)
    primary = [e for e in a['events'] if e['primary']]
    followup = {'status': 'POST_HOC_DESCRIPTIVE_ONLY', 'registered_primary_modified': False,
                'definition': 'First original-episode terminal from t0 through trace end; includes terminal row. '
                              'No later reset episode counted. After 20 intervention steps, Frozen F resumes.',
                'safe_abort': 'NOT_RECORDED', 'true_obstacle_clearance': 'NOT_RECORDED',
                'arms': {}, 'events': []}
    for arm in a['primary']['arms']:
        records = []
        with np.load(raw / ('long_' + arm) / 'h_r3_p03_trace.npz', allow_pickle=False) as tr:
            for event in primary:
                env, start = event['env'], event['t0']
                steps = len(tr['reset'])
                flags = {k: tr[k][start:steps, env].astype(bool)
                         for k in ('reset', 'crashed', 'captured', 'timeout')}
                hits = np.flatnonzero(np.logical_or.reduce(list(flags.values())))
                stop = start + int(hits[0]) if len(hits) else None
                end = stop + 1 if stop is not None else steps
                record = dict(arm=arm, env=env, t0=start, terminal_step=stop,
                              observed_s=(end-start)*.1, censored=stop is None,
                              **{k: bool(tr[k][stop, env]) if stop is not None else False
                                 for k in ('crashed', 'captured', 'timeout')},
                              contact_observed=bool((tr['contact_substeps'][start:end, env] > 0).any()))
                record['other_reset'] = stop is not None and not any(record[k] for k in ('crashed', 'captured', 'timeout'))
                records.append(record)
        followup['events'].extend(records)
        followup['arms'][arm] = dict(n=len(records),
            **{k: sum(v[k] for v in records) for k in ('crashed', 'captured', 'timeout', 'censored', 'contact_observed', 'other_reset')},
            total_observed_s=sum(v['observed_s'] for v in records),
            overlap_warning='Terminal flags can overlap; categories are not forced to be mutually exclusive',
            within_5s={key: sum(e['results'][arm][key] for e in primary)
                       for key in ('capture_by_w_plus_3s', 'crash_by_w_plus_3s', 'timeout_by_w_plus_3s')},
            within_2s_contact=sum(e['results'][arm]['contact_in_window'] for e in primary),
            tracker_fresh_within_2s=sum(e['results'][arm]['tracker_fresh_measurement_within_window'] for e in primary))
    save(out / 'secondary_followup.json', followup)
    # Assumption-only paired-binary normal approximation, not an approved sample size.
    z = NormalDist().inv_cdf(1-.05/(2*3)) + NormalDist().inv_cdf(.8)
    sensitivity = {'status': 'PLANNING_ASSUMPTIONS_ONLY_CONFIRMATION_HOLD',
                   'observed_qualification_effect_used_for_power': False,
                   'formula': 'ceil((z_(1-.05/6)+z_.8)^2 * (q-delta^2)/delta^2 * (1+(m-1)*ICC))',
                   'warning': 'Equal cluster sizes and exchangeable contrast ICC assumed. Normal approximation; '
                              'no joint cluster simulation performed. Bonferroni is a conservative planning bound '
                              'for three future Holm tests. No power guarantee from this table.',
                   'rows': []}
    for delta in (.10, .15, .20):
        for q in (.2, .4, .6):
            for m in (10, 20):
                for icc in (0., .05, .10, .20):
                    n = math.ceil(z*z*(q-delta*delta)/(delta*delta)*(1+(m-1)*icc))
                    sensitivity['rows'].append(dict(delta=delta, discordance_q=q, events_per_seed=m,
                        contrast_icc=icc, paired_events=n, seeds_arithmetic=math.ceil(n/m),
                        seeds_with_provisional_32_cluster_floor=max(32, math.ceil(n/m))))
    save(out / 'sample_size_sensitivity.json', sensitivity)
    receipt = dict(status='PASS', source_commit=r['source_commit'], raw_path=str(raw),
                   raw_manifest_sha256=digest(raw/'RAW_MANIFEST.json'), verified_members=len(members),
                   verified_payload_bytes=checked_bytes, complete_inventory=True,
                   analysis_sha256=digest(raw/'qualification_analysis.json'),
                   run_report_sha256=digest(raw/'run_report.json'),
                   method='Independent streaming SHA-256/size/full inventory verification after all GPU processes exited',
                   raw_mutated=False)
    save(out / 'raw_verification.json', receipt)
    print(json.dumps(dict(receipt=receipt, followup=followup['arms'], resources=resource), indent=2))


if __name__ == '__main__':
    main()
