
import resource
resource.setrlimit(resource.RLIMIT_AS,(4294967296,4294967296)); resource.setrlimit(resource.RLIMIT_CPU,(300,300))
import json, hashlib, pathlib, subprocess
A=pathlib.Path('/home/fair/motar_tmp/g7_20261105/opus_results_review')
(A/'probe1c_source.py').open('xb').write(open('/proc/self/cmdline','rb').read().split(b'\0')[3])
RUN=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/actionhold_g7_v1')
REF=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00')
T='/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly'; MC='06383eba74134a594bd38ba933cb5e0512f1d342'; B='3f6f7e540117df25b40dd8ad1cb1f02e67c4cf4c'
def git(*x): return subprocess.run(['git','-C',T]+list(x),capture_output=True,text=True,timeout=300).stdout.strip()
def H(p): return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def rdiff(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        o=[]
        for k in sorted(set(a).union(b)):
            if k not in a or k not in b: o.append(path+'/'+k+' one_side')
            else: o+=rdiff(a[k],b[k],path+'/'+k)
        return o
    return [] if a==b else [path]
rep=json.loads((RUN/'g7_run_report.json').read_text())
c='E2_obstacle_aware__205bars__seed4501'
ra=json.loads((RUN/'k1d0'/c/'result.json').read_text()); rb=json.loads((REF/c/'result.json').read_text())
out={'head':git('rev-parse','HEAD'),'branch':git('rev-parse','--abbrev-ref','HEAD'),'porcelain':git('status','--porcelain','--untracked-files=all'),
 'meas_path_changed_base_to_meas':git('diff','--name-only',B,MC,'--','tools/actionhold_hook.py','tools/actionhold_site','tools/actionhold_analyse.py','aerial_gym','motar_eval','tools/camonly_run.py','tools/gr_run.py','tools/kroc2027_eval.py','tools/camonly_python','tools/camonly_entry.py'),
 'prereg_changed_base_to_meas':git('diff','--name-only',B,MC,'--','docs/plans/prereg_actionhold_g7_v1.md'),
 'prereg_log':git('log','--format=%h %cI %s','--','docs/plans/prereg_actionhold_g7_v1.md'),
 'analyser_log':git('log','--format=%h %cI %s','--','tools/actionhold_analyse.py'),
 'hook_log':git('log','--format=%h %cI %s','--','tools/actionhold_hook.py','tools/actionhold_site/sitecustomize.py'),
 'runner_log':git('log','--format=%h %cI %s','--','tools/actionhold_run.py'),
 'meas_commit_time':git('show','-s','--format=%cI',MC),
 'hook_sha_report_eq_tree':{f:rep['hook_sha256'][f]==H(T+'/'+f) for f in rep['hook_sha256']},
 'report_origins':rep['import_origins'],'preflight_problems':rep['preflight_problems'],'reading':rep['reading'],
 'record_verdicts':[r['verdict'] for r in rep['records']],'record_problems':[r['problems'] for r in rep['records']],
 'record_hosts_same':all(r['host_after'][k]==rep['host_start'][k] for r in rep['records'] for k in ('fault_count','boot_id','online')),
 'k1d0_result_json_diff':rdiff(ra,rb),'k1d0_result_values':{p:None for p in []},
 'k1d0_kroc_episodes_bytes_equal':(RUN/'k1d0'/c/'kroc2027_episodes.jsonl').read_bytes()==(REF/c/'kroc2027_episodes.jsonl').read_bytes(),
 'k1d0_kroc_summary_diff':rdiff(json.loads((RUN/'k1d0'/c/'kroc2027_summary.json').read_text()),json.loads((REF/c/'kroc2027_summary.json').read_text())),
 'k1d0_ledger_summary_diff':rdiff(json.loads((RUN/'k1d0'/c/'episode_ledger_summary.json').read_text()),json.loads((REF/c/'episode_ledger_summary.json').read_text())),
 'ref_receipts':{d:{k:json.loads((REF/('E2_obstacle_aware__'+d+'bars__seed4501')/'receipt.json').read_text()).get(k) for k in ('git_commit','git_dirty_files','status','finished_at','requested_episodes')} for d in ('70','205')},
 'boot_id_now':pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
for p in out['k1d0_result_json_diff']:
    keys=[x for x in p.split('/') if x]; va=ra; vb=rb
    for x in keys: va=va.get(x) if isinstance(va,dict) else None; vb=vb.get(x) if isinstance(vb,dict) else None
    out['k1d0_result_values'][p]=[str(va)[:120],str(vb)[:120]]
(A/'probe1c_source.json').open('x').write(json.dumps(out,indent=1,sort_keys=True)+'\n')
print(json.dumps(out,indent=1,sort_keys=True))
