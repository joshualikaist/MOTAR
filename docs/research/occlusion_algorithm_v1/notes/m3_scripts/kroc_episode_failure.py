"""Read-only aggregation of KROC2027 recorder episode records (kroc2027_episodes.jsonl).
Per (campaign, arm, density): outcomes, never-acquired, and the measurement/track state at a failure end.
Quota filter: env_episode_index < quota. Descriptive only (no causal claim)."""
import json, sys, glob, os, collections, statistics
R = '/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results'
CAMPAIGNS = [
    ('KROC-A1_static_visible', R + '/kroc2027_A_v1/A1/*/kroc2027_episodes.jsonl', 16),
    ('KROC-A2_static_hidden',  R + '/kroc2027_A_v1/A2/*/kroc2027_episodes.jsonl', 16),
    ('KROC-A3_E2_visible',     R + '/mt_p1_precision_v1/KROC27-A-M02/E2_direct_l2_v1/A3/*/kroc2027_episodes.jsonl', 16),
    ('KROC-A4_E2_hidden',      R + '/mt_p1_precision_v1/KROC27-A-M02/E2_direct_l2_v1/A4/*/kroc2027_episodes.jsonl', 16),
    ('OCC-P2_static_visible',  R + '/occlusion_confirmatory_v1/OCC-P2/static/A1/*/kroc2027_episodes.jsonl', 4),
    ('OCC-P2_E2_visible',      R + '/occlusion_confirmatory_v1/OCC-P2/moving/A3/*/kroc2027_episodes.jsonl', 4),
]
def bars_of(path):
    cell = os.path.basename(os.path.dirname(path))
    return int(cell.split('__')[1].replace('bars', '')), cell
out = {}
for name, pat, quota in CAMPAIGNS:
    files = sorted(glob.glob(pat))
    agg = collections.defaultdict(lambda: collections.Counter())
    lat = collections.defaultdict(list)
    cells = collections.Counter()
    for f in files:
        b, cell = bars_of(f)
        cells[b] += 1
        for line in open(f):
            d = json.loads(line)
            if d['env_episode_index'] >= quota:
                continue
            A = agg[b]; o = d['outcome']
            A['n'] += 1; A[o] += 1
            acq = bool(d['ever_fused_valid'])
            if not acq:
                A['never_acq'] += 1; A['never_acq_' + o] += 1
            else:
                lat[b].append(d['first_acquisition_latency_s'])
                if o != 'capture':
                    A['acq_fail'] += 1
                    if d.get('track_open_loss_censored_obs') is not None:
                        A['acq_fail_track_lost_at_end'] += 1
                    if d.get('detection_open_loss_censored_obs') is not None:
                        A['acq_fail_fused_lost_at_end'] += 1
                    if d.get('track_losses', 0) > 0:
                        A['acq_fail_any_track_loss'] += 1
                if o == 'capture':
                    if d.get('track_losses', 0) > 0: A['cap_any_track_loss'] += 1
                    if d.get('detection_losses', 0) > 0: A['cap_any_fused_loss'] += 1
                if d.get('track_losses', 0) > 0: A['acq_any_track_loss'] += 1
                A['acq'] += 1
    res = {}
    for b in sorted(agg):
        A = agg[b]; L = sorted(lat[b])
        fail = A['crash'] + A['timeout']
        res[b] = dict(cells=cells[b], n=A['n'], capture=A['capture'], crash=A['crash'], timeout=A['timeout'],
                      never_acq=A['never_acq'], never_acq_crash=A['never_acq_crash'], never_acq_timeout=A['never_acq_timeout'],
                      never_acq_capture=A['never_acq_capture'],
                      failures=fail, share_fail_never_acq=(A['never_acq_crash'] + A['never_acq_timeout']) / fail if fail else None,
                      acq_fail=A['acq_fail'], acq_fail_track_lost_at_end=A['acq_fail_track_lost_at_end'],
                      acq_fail_fused_lost_at_end=A['acq_fail_fused_lost_at_end'], acq_fail_any_track_loss=A['acq_fail_any_track_loss'],
                      acq=A['acq'], acq_any_track_loss=A['acq_any_track_loss'], cap_any_track_loss=A['cap_any_track_loss'],
                      first_acq_latency_s_median=statistics.median(L) if L else None,
                      first_acq_latency_s_p90=L[int(0.9 * (len(L) - 1))] if L else None)
    out[name] = res
    print('==', name, 'files', len(files))
    for b, r in res.items():
        print(' ', b, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
json.dump(out, open(sys.argv[1], 'w'), indent=1)
