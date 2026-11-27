#!/usr/bin/env python3
"""Strict five-process reset-isolation supervision; no retry, no P03, no training."""
from __future__ import annotations
import argparse,copy,json,sys,time,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT/'tools') not in sys.path:sys.path.insert(0,str(ROOT/'tools'))
import s_l1b_run as b
a,base=b.a,b.base
PLAN=ROOT/'docs/plans/h_r3_env_isolation_v1.json'
CPU=ROOT/'docs/evidence/h_r3_env_isolation_2026-10-18/cpu_report.json'
LAUNCHER=ROOT/'tools/h_r3_env_isolation_launcher.py'
INPUTS=[PLAN,PLAN.with_suffix('.md'),ROOT/'docs/plans/h_r3_env_isolation_amendment1.md',CPU,ROOT/'docs/plans/h_r3_env_isolation_amendment2.md']+[ROOT/p for p in (
 'tools/h_r3_env_isolation.py','tools/h_r3_env_isolation_launcher.py','tools/h_r3_env_isolation_run.py','tools/h_r3_env_isolation_analysis.py',
 'tests/test_h_r3_env_isolation.py','tests/test_h_r3_env_isolation_analysis.py','tests/test_h_r3_env_isolation_run.py')]


def specs(p):
    return [dict(x,seed=p['run']['seed'],num_envs=p['run']['num_envs'],steps=p['run']['steps'],config='moving',arm='P0',batch='HR3',lineage='off',cap_s=p['run']['process_wall_cap_s']) for x in p['processes']]


def preflight(p,approved,out):
    old=a.read_json(ROOT/'docs/plans/eq1d_qualification_v1.json');problems,checks=base.preflight(old,approved,out,False)
    checks['committed_inputs']={str(x.relative_to(ROOT)):a.committed(x) for x in INPUTS}
    checks['input_sha256']={str(x.relative_to(ROOT)):base.sha256_file(x) for x in INPUTS if x.is_file()}
    if not all(checks['committed_inputs'].values()):problems.append('qualification inputs not committed unchanged')
    if not CPU.is_file() or a.read_json(CPU).get('status')!='PASS':problems.append('independent CPU oracle not PASS')
    if checks.get('policy_f_sha256')!=p['policy_f_sha256']:problems.append('policy binding differs')
    extra=base.git('ls-files','--others','--exclude-standard','--','aerial_gym','motar_eval','tools','tests').stdout.splitlines()
    checks['untracked_runtime_or_test_files']=extra
    if extra:problems.append('untracked source/test files outside frozen source')
    return problems,checks


def validate(out,spec,checks):
    done=a.read_json(out/'process_done.json')
    if done.get('process')!=spec['id'] or done.get('nan_fields')!=[] or not done.get('files'):raise ValueError('invalid completion identity/outputs')
    seen=set()
    for item in done['files']:
        rel=Path(item['file']);path=out/rel
        if rel.is_absolute() or '..'in rel.parts or item['file']in seen or out.resolve()not in path.resolve().parents or any((out/Path(*rel.parts[:i])).is_symlink()for i in range(1,len(rel.parts)+1)):raise ValueError('unsafe or duplicate completion output')
        seen.add(item['file'])
        if path.stat().st_size!=item['bytes'] or base.sha256_file(path)!=item['sha256']:raise ValueError('missing or changed completion output')
    actual={str(x.relative_to(out))for x in out.rglob('*')if x.is_file()and x.name!='process_done.json'}
    if actual!=seen:raise ValueError('unbound completion output')
    m=a.read_json(out/'process_meta.json')
    for key in ('id','seed','num_envs','steps','mode','force_reset','evaluator'):
        if m[key]!=spec[key]:raise ValueError('process specification changed: '+key)
    if m['actor_obs_dim']!=898 or done['actor_obs_dim']!=898 or m['model_training'] or m['step_dt']!=.1:raise ValueError('actor/timing mode changed')
    if m['rms_digest_start']!=checks['rms_digest'] or m['rms_digest_end']!=checks['rms_digest']:raise ValueError('RMS changed')
    if m['policy_f_sha256']!=checks['policy_f_sha256'] or m['source_commit']!=checks['head']:raise ValueError('source/checkpoint changed')
    if any(ROOT.resolve()not in Path(m['origins'][key]).resolve().parents for key in ('aerial_gym','motar_eval')):raise ValueError('import outside source worktree')
    return done


def run(p,out,checks):
    out.mkdir(parents=True,exist_ok=False);shutil.copyfile(CPU,out/'cpu_report.json')
    start=time.monotonic();deadline=start+p['run']['total_wall_cap_s'];head=base.git('rev-parse','HEAD').stdout.strip();tree=base.git('status','--porcelain','--untracked-files=no').stdout
    report=dict(schema='motar.environment-isolation-run.v1',source_commit=head,run_id=p['run']['run_id'],preflight=checks,processes=[],status='RUNNING',stop_reason=None)
    envp=a.read_json(ROOT/'docs/plans/h_r3_p02.json');envp['run'].update(run_id=p['run']['run_id'],closed_loop_density=p['run']['density'])
    try:
        for spec in specs(p):
            reason,_=a.runtime_guard(p,out,head,tree,deadline)
            if reason:report['stop_reason']=reason;break
            gpu=base.gpu_state();cap=gpu['total_mib']-gpu['used_mib']-p['run']['oom_reserve_mib']
            if cap<=0 or '3070'not in gpu['name']:raise RuntimeError('GPU/headroom mismatch')
            target=out/spec['id'];cmd=[base.PYTHON,'-B',str(LAUNCHER),'--process',spec['id'],'--out',str(target),'--source-commit',head]
            entry=dict(id=spec['id'],spec=spec,command=cmd,gpu_at_launch=gpu);report['processes'].append(entry)
            limit=time.monotonic()+spec['cap_s'];guard=lambda proc,limit=limit,cap=cap:a.runtime_guard(p,out,head,tree,deadline,process_deadline=limit,proc=proc,cap_mib=cap)
            rc,wall,peak,why=a.supervised_launch(cmd,b.process_env(envp,spec,target),out/(spec['id']+'.log'),guard,p['run']['poll_s'])
            entry.update(returncode=rc,wall_s=wall,peak_gpu_mib=peak)
            if why or rc:report['stop_reason']=why or 'NONZERO_EXIT:'+spec['id']+':'+str(rc);break
            reason,_=a.runtime_guard(p,out,head,tree,deadline)
            if reason:report['stop_reason']=reason;break
            entry['completion']=validate(target,spec,checks)
            print('[isolation] '+spec['id']+' completed %.1fs'%wall,flush=True)
    except Exception as e:report['stop_reason']='INTEGRITY_EXCEPTION: '+type(e).__name__+': '+str(e)
    report.update(status='STOPPED_INTEGRITY' if report['stop_reason']else'COMPLETED',wall_s=time.monotonic()-start,gpu_seconds=sum(x.get('wall_s',0)for x in report['processes']),free_disk_gb_after=base.free_gb(out),output_bytes=base.dir_bytes(out))
    frozen=a.read_json(ROOT/'docs/plans/eq1d_qualification_v1.json');report['policy_f_sha256_after']=base.sha256_file(base.RL_DIR/frozen['frozen']['policy_f_rel_to_rl_games'])
    if report['policy_f_sha256_after']!=p['policy_f_sha256']:report.update(status='STOPPED_INTEGRITY',stop_reason='POLICY_CHANGED')
    with(out/'run_report.json').open('x')as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    base.write_manifest(out);return report


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('stage',choices=('preflight','run'));ap.add_argument('--approved-commit',required=True);ap.add_argument('--run-id',choices=('ISO-Q02',),default='ISO-Q02');args=ap.parse_args()
    p=a.read_json(PLAN);p['run']['run_id']=args.run_id;out=base.resolve(p['run']['output_root'])/p['run']['run_id'];problems,checks=preflight(p,args.approved_commit,out)
    if problems or args.stage=='preflight':print(json.dumps(dict(problems=problems,checks=checks),indent=2));return 2 if problems else 0
    report=run(p,out,checks);print(json.dumps({k:report[k]for k in ('status','stop_reason','gpu_seconds')},indent=2));return 0 if report['status']=='COMPLETED'else 1

if __name__=='__main__':raise SystemExit(main())
