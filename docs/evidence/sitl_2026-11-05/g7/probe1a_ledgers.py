
import resource
resource.setrlimit(resource.RLIMIT_AS,(4294967296,4294967296)); resource.setrlimit(resource.RLIMIT_CPU,(300,300))
import json, hashlib, pathlib, collections
A=pathlib.Path('/home/fair/motar_tmp/g7_20261105/opus_results_review')
(A/'probe1a_ledgers.py').open('xb').write(open('/proc/self/cmdline','rb').read().split(b'\0')[3])
RUN=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/actionhold_g7_v1')
REF=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00')
PLAN=[('k1d0',205),('k2d0',205),('k4d0',205),('k1d1',205),('k2d0',70),('k4d0',70),('k1d1',70)]
def cid(d): return 'E2_obstacle_aware__' + str(d) + 'bars__seed4501'
def ledger(p):
    raw=p.read_bytes(); rows=[json.loads(l) for l in raw.splitlines()]
    q=[r for r in rows if r['in_quota']]
    per_env=collections.Counter(r['env_index'] for r in q)
    cnt=collections.Counter(r['outcome'] for r in q)
    s=json.loads((p.parent/'episode_ledger_summary.json').read_text())
    return {'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw),'summary_rows_sha_ok':s['rows_sha256']==hashlib.sha256(raw).hexdigest(),
      'rows':len(rows),'quota_rows':len(q),'envs':len(per_env),'per_env_quota':sorted(set(per_env.values())),
      'ids_unique':len(set(r['episode_id'] for r in rows))==len(rows),
      'quota_is_first16':all(r['in_quota']==(r['env_episode_index'] in range(16)) for r in rows),
      'outcome_exclusive':all(int(r['capture'])+int(r['crash'])+int(r['timeout'])==1 and r[r['outcome']] is True for r in rows),
      'counts':dict(sorted(cnt.items())),'summary_counts_ok':s['quota']['counts']==dict(cnt),'crash_pct':100*cnt['crash']/len(q),'capture_pct':100*cnt['capture']/len(q)}
out={'reference':{str(d):ledger(REF/cid(d)/'episodes.jsonl') for d in (70,205)},'cells':{a+'/'+str(d):ledger(RUN/a/cid(d)/'episodes.jsonl') for a,d in PLAN}}
a=RUN/'k1d0'/cid(205); b=REF/cid(205)
ba,bb=(a/'episodes.jsonl').read_bytes(),(b/'episodes.jsonl').read_bytes()
out['identity']={'bytes_equal':ba==bb,'parsed_equal':[json.loads(l) for l in ba.splitlines()]==[json.loads(l) for l in bb.splitlines()]}
(A/'probe1a_ledgers.json').open('x').write(json.dumps(out,indent=1,sort_keys=True)+'\n')
for k,v in list(out['reference'].items())+list(out['cells'].items()): print(k, v['sha256'][:16], v['bytes'], v['rows'], v['quota_rows'], v['envs'], v['per_env_quota'], v['ids_unique'], v['quota_is_first16'], v['outcome_exclusive'], v['summary_rows_sha_ok'], v['summary_counts_ok'], v['counts'], round(v['crash_pct'],4))
print('identity', out['identity'])
