
import resource
resource.setrlimit(resource.RLIMIT_AS,(4294967296,4294967296)); resource.setrlimit(resource.RLIMIT_CPU,(300,300))
import json, hashlib, pathlib, collections
A=pathlib.Path('/home/fair/motar_tmp/g7_20261105/opus_results_review')
(A/'probe1b_receipts.py').open('xb').write(open('/proc/self/cmdline','rb').read().split(b'\0')[3])
RUN=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/actionhold_g7_v1')
REF=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00')
TREE='/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly/'
MC='06383eba74134a594bd38ba933cb5e0512f1d342'; FSHA='f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40'
PLAN=[('k1d0',205,1,0),('k2d0',205,2,0),('k4d0',205,4,0),('k1d1',205,1,1),('k2d0',70,2,0),('k4d0',70,4,0),('k1d1',70,1,1)]
def cid(d): return 'E2_obstacle_aware__' + str(d) + 'bars__seed4501'
def J(p): return json.loads(p.read_text())
def H(p): return hashlib.sha256(p.read_bytes()).hexdigest()
modsha=H(pathlib.Path(TREE+'aerial_gym/task/navrl_task/navrl_sensor_ablation.py'))
out={}
for arm,d,k,dd in PLAN:
    c=RUN/arm/cid(d); rc=J(c/'receipt.json'); s=J(c/'episode_ledger_summary.json'); ks=J(c/'kroc2027_summary.json'); sc=J(c/'actionhold_hook.json'); ab=J(c/'sensor_ablation_receipt.json'); rr=J(REF/cid(d)/'receipt.json')
    sa,sb=rc['env_snapshot_navrl'],rr['env_snapshot_navrl']
    diff={x:[sa.get(x),sb.get(x)] for x in sorted(set(sa).union(sb)) if not sa.get(x)==sb.get(x) and x not in ('NAVRL_BULK_EVAL_JSON','NAVRL_EVAL_RUN_NONCE','NAVRL_SENSOR_ABLATION_RECEIPT')}
    hold=not k==1; dly=not dd==0
    full={o:s['quota']['counts'].get(o,0) for o in ('capture','crash','timeout')}
    mine=collections.Counter(json.loads(l)['outcome'] for l in (c/'episodes.jsonl').read_bytes().splitlines() if json.loads(l)['in_quota'])
    out[arm+'/'+str(d)]={'receipt':[rc['status'],rc['git_commit']==MC,rc['git_dirty_files'],rc['checkpoint_sha256']==FSHA,rc['requested_episodes'],rc['seed'],rc['density_bars'],rc['result_sha256']==H(c/'result.json')],
      'snapshot_diff_vs_F_excl_paths_nonce':diff,'MOTAR_in_snapshot':[x for x in sa if 'MOTAR' in x],
      'ablation':[ab['condition'],ab['module_sha256']==modsha,all(str(ab[x]).startswith(TREE) for x in ('module','perception','task','script')),ab['hbeams'],ab['vbeams']],
      'ledger_summary':[s['status'],s['num_envs'],s['quota_per_env'],s['partition_violations'],s['quota']['episodes'],s['quota']['expected_episodes'],full=={o:mine.get(o,0) for o in full},s['task_steps'],s['rows_written']],
      'recorder':[ks['reason'],ks['motion_gate']['verdict'],sum(ks['motion_gate']['violations'].values()),ks['swept_clearance']['violations'],ks['swept_clearance']['wall_violations'],(c/'kroc2027_violations.jsonl').exists(),ks['context']['checkpoint_sha256']==FSHA,ks['context']['run_id'],all(v.startswith(TREE) for v in ks['import_origins'].values()),ks['episodes_sha256']==H(c/'kroc2027_episodes.jsonl'),ks['initial_view']['failed_closed']],
      'sidecar':[sc['config']=={'hold_k':k,'delay_d':dd},sc['patch_applied'] is True,sc['written_by'],sc['calls'],sc['calls']==s['task_steps'],sc['latched_env_steps'],sc['held_env_steps'],sc['delayed_zero_env_steps'],
          (sc['latched_env_steps']+sc['held_env_steps']==sc['calls']*s['num_envs']) if hold else None,round(sc['held_env_steps']/sc['latched_env_steps'],3) if hold else None,
          (sc['delayed_zero_env_steps']-s['rows_written']) if dly else None,(bool(sc['held_env_steps']) if hold else (bool(sc['delayed_zero_env_steps']) if dly else 'identity'))],
      'files':sorted(p.name for p in c.iterdir())}
(A/'probe1b_receipts.json').open('x').write(json.dumps(out,indent=1,sort_keys=True)+'\n')
for kk,v in out.items():
    print(kk); [print('  ',f,v[f]) for f in ('receipt','snapshot_diff_vs_F_excl_paths_nonce','MOTAR_in_snapshot','ablation','ledger_summary','recorder','sidecar')]
print(out['k1d0/205']['files'])
