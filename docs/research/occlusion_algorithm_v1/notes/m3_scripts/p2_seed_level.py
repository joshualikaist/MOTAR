"""OCC-P2 seed-level descriptive spread of (a) crash-ended centre-occlusion intervals per interval,
(b) share of crash episodes with an active centre-occlusion or out-of-view interval at the crash, (c) crash rate.
Planning context only; no new inference."""
import json, sys, statistics as st, collections
a = json.load(open(sys.argv[1]))
by = collections.defaultdict(list)
for k, c in a['cells'].items():
    cr = c['outcomes'].get('crash', 0); ints = c['intervals']
    cc = c['duration']['end_causes'].get('crash', 0); co = c['diagnostic_indicators']['out_of_view']['end_causes'].get('crash', 0)
    by[(c['group'], c['density'])].append(dict(seed=c['seed'], crash_rate=cr / c['episodes'], crash_end_per_interval=cc / ints if ints else None,
        crash_share_occ=cc / cr if cr else None, crash_share_occ_or_oov=(cc + co) / cr if cr else None, intervals=ints, crash=cr))
for g in sorted(by):
    rows = by[g]
    def ms(key):
        v = [r[key] for r in rows if r[key] is not None]
        return f"{st.mean(v):.4f}+-{st.stdev(v):.4f}(n={len(v)})"
    print(g, 'intervals/seed', st.mean(r['intervals'] for r in rows), 'crash/seed', st.mean(r['crash'] for r in rows),
          '| crash_rate', ms('crash_rate'), '| crash_end/interval', ms('crash_end_per_interval'), '| crash@occ share', ms('crash_share_occ'), '| crash@(occ|oov)', ms('crash_share_occ_or_oov'))
