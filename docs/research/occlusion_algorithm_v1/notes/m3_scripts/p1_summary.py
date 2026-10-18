import json,sys
a=json.load(open(sys.argv[1]))
for g,v in sorted(a['groups'].items()):
    print('==',g, [k for k in v if k!='indicators'])
    for ind,x in v['indicators'].items():
        if not isinstance(x,dict) or 'intervals' not in x: print(ind, str(x)[:200]); continue
        bi=x.get('episode_cluster_bootstrap_rate_per_exposure_s',{}).get('interval')
        print(f"  {ind:26s} n_int={x['intervals']:5d} rate/min={60*x['rate_per_exposure_s']:.3f} boot={[round(60*b,3) for b in bi] if bi else None} share_ep={x['share_episodes_with_interval']:.3f} expo_s={x['exposure_s']:.0f} med={x['duration_quantiles_observed_s']} ends={x['end_causes']}")
    for k in v:
        if k!='indicators': print('  ',k, json.dumps(v[k])[:600])
