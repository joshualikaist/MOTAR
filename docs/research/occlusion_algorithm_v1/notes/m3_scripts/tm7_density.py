"""7-seed target-motion replication: per arm x density. Quota outcomes from committed cells.csv; never-acquired
by outcome from raw result.json first_acquisition (LEGACY pooled window, not quota). Read-only."""
import csv, json, collections, os
REPO = '/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/results/target_motion_replication_7seed/cells.csv'
RAW = '/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/target_motion_replication_7seed'
agg = collections.defaultdict(collections.Counter)
for r in csv.DictReader(open(REPO)):
    k = (r['arm'], int(r['bars'])); A = agg[k]
    for f in ('primary_episodes', 'primary_capture', 'primary_crash', 'primary_timeout'): A[f] += int(r[f])
    A['seeds'] += 1
    res = json.load(open(os.path.join(RAW, r['cell_id'], 'result.json')))
    fa = res['target_motion']['first_acquisition']
    for o in ('capture', 'crash', 'timeout'):
        A['leg_' + o] += fa[o]['episodes']; A['leg_na_' + o] += fa[o]['never_acquired']
        A['leg_vh_' + o] += fa[o]['visible_hidden_transitions']
    cc = res.get('crash_causes', {})
    A['cc_bar'] += cc.get('bar_contact', 0); A['cc_below'] += cc.get('below', 0); A['cc_oob'] += cc.get('out_of_bounds', 0); A['cc_n'] += cc.get('count', 0)
print('arm bars seeds | quota cap% crash% timeout% | legacy: crash never-acq%, timeout never-acq%, failures never-acq% | vis->hid per capture ep | crash causes bar/below/oob')
for k in sorted(agg, key=lambda x: (x[0], x[1])):
    A = agg[k]; n = A['primary_episodes']
    fl = A['leg_crash'] + A['leg_timeout']; fna = A['leg_na_crash'] + A['leg_na_timeout']
    print(k[0], k[1], A['seeds'], '|', f"{100*A['primary_capture']/n:.2f} {100*A['primary_crash']/n:.2f} {100*A['primary_timeout']/n:.2f}", '|',
          f"{100*A['leg_na_crash']/A['leg_crash']:.1f} {100*A['leg_na_timeout']/max(A['leg_timeout'],1):.1f} {100*fna/fl:.1f} (n_fail={fl})", '|',
          f"{A['leg_vh_capture']/A['leg_capture']:.2f}", '|', f"{A['cc_bar']}/{A['cc_below']}/{A['cc_oob']} of {A['cc_n']}")
