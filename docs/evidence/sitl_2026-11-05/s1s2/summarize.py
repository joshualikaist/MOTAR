import json, sys
r = json.load(open(sys.argv[1]))
print(r['status'], r.get('error'), 'faults', r['host_faults_before'], r['host_faults_after'], 'RTF', round(r.get('real_time_factor') or 0, 3), 'overrides', r.get('param_overrides'))
print('A0', (r.get('A0') or {}).get('ok'), round((r.get('A0') or {}).get('altitude_after_climb_m', 0), 3))
for k in ('A2', 'A5', 'A6'):
    print(k, {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in (r.get(k) or {}).items()})
for ax, steps in (r.get('A3') or {}).items():
    for s in steps:
        print('A3', ax, s['from'], '->', s['to'], {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in s.items() if kk in ('rise_10_90_s', 'onset_latency_s', 'overshoot_pct', 'settling_5pct_s', 'final_mean')})
for s in (r.get('A4') or []):
    print('A4', {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in s.items() if kk in ('cmd_rad_s', 'achieved_mean_last1s', 'rise_10_90_s', 'overshoot_pct')})
