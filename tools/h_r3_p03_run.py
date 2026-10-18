#!/usr/bin/env python3
"""Bounded P03 qualification, stop on any failed gate; never launches a large pilot."""
from __future__ import annotations
import argparse,json,sys,time,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT/'tools')not in sys.path:sys.path.insert(0,str(ROOT/'tools'))
import s_l1b_run as b
import h_r3_p03_analysis as analysis
a,base=b.a,b.base
PLAN=ROOT/'docs/plans/h_r3_p03_qualification_v1.json'
EVIDENCE=ROOT/'docs/evidence/h_r3_p03_preparation_2026-10-19'
LAUNCHER=ROOT/'tools/h_r3_p03_launcher.py'
INPUTS=[PLAN,PLAN.with_suffix('.md'),EVIDENCE/'cpu_validation.json',EVIDENCE/'review_receipt.json']+[ROOT/p for p in ('tools/h_r3_p03_isolation.py','tools/h_r3_p03_launcher.py','tools/h_r3_p03_analysis.py','tools/h_r3_p03_run.py','motar_eval/h_r3_p03_arms.py','tests/test_h_r3_p03_isolation.py','tests/test_h_r3_p03_arms.py','tests/test_h_r3_p03_analysis.py','tests/test_h_r3_p03_runner.py')]


def specs(plan):
    return[dict(x,seed=plan['run']['seed'],num_envs=plan['run']['num_envs'],config='moving',arm='P0',batch='HR3',lineage='off')for x in plan['processes']]


def preflight(plan,approved,out):
    old=a.read_json(ROOT/'docs/plans/eq1d_qualification_v1.json');problems,checks=base.preflight(old,approved,out,False)
    checks['committed_inputs']={str(x.relative_to(ROOT)):a.committed(x)for x in INPUTS}
    checks['input_sha256']={str(x.relative_to(ROOT)):base.sha256_file(x)for x in INPUTS if x.is_file()}
    if not all(checks['committed_inputs'].values()):problems.append('qualification source/tests/CPU/review/prereg not committed unchanged')
    for name,wanted in (('cpu_validation.json','PASS'),('review_receipt.json','COMPLETED_READ_ONLY_SOURCE_REVIEW')):
        try:
            j=a.read_json(EVIDENCE/name)
            if j['status']!=wanted:problems.append(name+' not qualified')
            if name=='review_receipt.json'and j.get('unresolved_blocker_high',1):problems.append('unresolved independent review blocker')
        except Exception:problems.append('missing CPU/review evidence '+name)
    extra=base.git('ls-files','--others','--exclude-standard','--','aerial_gym','motar_eval','tools','tests').stdout.splitlines();checks['untracked_runtime_files']=extra
    if extra:problems.append('untracked runtime/test source')
    if checks.get('policy_f_sha256')!=plan['policy_f_sha256']:problems.append('FrozenF changed')
    return problems,checks


def validate(out,spec,checks):
    tr,meta=analysis.load_process(out)
    expected=dict(seed=spec['seed'],num_envs=128,steps=spec['steps'],arm=spec['hr3_arm'],isolation_mode=spec['isolation_mode'],force_reset=spec['force_reset'],identity_enabled=spec['evaluator'],source_commit=checks['head'],policy_f_sha256=checks['policy_f_sha256'],rms_digest_start=checks['rms_digest'],rms_digest_end=checks['rms_digest'])
    if any(meta.get(k)!=v for k,v in expected.items()):raise ValueError('runtime specification or binding differs: '+spec['id'])
    if any(ROOT.resolve()not in Path(meta['origins'][k]).resolve().parents for k in('aerial_gym','motar_eval')):raise ValueError('import outside frozen worktree')
    if meta['geometry_integrity']['status']!='PASS':raise ValueError('geometry integrity failed')
    return tr,meta


def next_gate(out,spec,trace):
    name=spec['id']
    if name in('short_normal','long_B0'):
        if trace['natural_reset'][2].any()or trace['reset'][2].any():raise ValueError('baseline reset overlaps registeredforce')
        return{'PASS':True,'natural_reset_at2':[]}
    if name in('short_forced','long_forced'):
        normal='short_normal'if name.startswith('short')else'long_B0';reference,_=analysis.load_process(out/normal)
        chronology=analysis.check_forced_reset(reference,trace);comparison=analysis.compare_noninterference(out,normal,name,(127,))
        if not chronology['equal']or not comparison['equal']:raise ValueError('forced reset noninterference failed: '+name)
        return dict(PASS=True,chronology=chronology,comparison=comparison)
    if name in('long_noeval','long_audit'):
        comparison=analysis.compare_noninterference(out,'long_B0',name)
        if not comparison['equal']:raise ValueError('evaluator/mode parity failed: '+name)
        return dict(PASS=True,comparison=comparison)
    reference,_=analysis.load_process(out/'long_B0');arm=spec['hr3_arm'];prefix=analysis.matched_prefix({'B0':reference,arm:trace},required_arms=('B0',arm))
    if not prefix['equal']:raise ValueError('commonprefix failed: '+arm+' '+str(prefix['failures'][:3]))
    return dict(PASS=True,matched_prefix=prefix)


def run(plan,out,checks):
    out.mkdir(parents=True,exist_ok=False);start=time.monotonic();deadline=start+plan['run']['total_wall_cap_s'];head=checks['head'];tree=base.git('status','--porcelain','--untracked-files=no').stdout
    report=dict(schema='motar.hr3-p03-qualification.run.v1',run_id=plan['run']['run_id'],source_commit=head,preflight=checks,processes=[],status='RUNNING',stop_reason=None)
    envp=a.read_json(ROOT/'docs/plans/h_r3_p02.json');envp['run'].update(run_id=plan['run']['run_id'],closed_loop_density=160)
    try:
        for spec in specs(plan):
            why,_=a.runtime_guard(plan,out,head,tree,deadline)
            if why:report['stop_reason']=why;break
            gpu=base.gpu_state();cap=gpu['total_mib']-gpu['used_mib']-plan['run']['oom_reserve_mib']
            if cap<=0 or'3070'not in gpu['name']:raise ValueError('GPU/headroom changed')
            dest=out/spec['id'];cmd=[base.PYTHON,'-B',str(LAUNCHER),'--process',spec['id'],'--out',str(dest),'--source-commit',head]
            entry=dict(id=spec['id'],spec=spec,command=cmd,gpu_at_launch=gpu);report['processes'].append(entry)
            process_deadline=time.monotonic()+spec['cap_s'];guard=lambda proc,limit=process_deadline,memory=cap:a.runtime_guard(plan,out,head,tree,deadline,process_deadline=limit,proc=proc,cap_mib=memory)
            rc,wall,peak,why=a.supervised_launch(cmd,b.process_env(envp,spec,dest),out/(spec['id']+'.log'),guard,plan['run']['poll_s'])
            entry.update(returncode=rc,wall_s=wall,peak_gpu_mib=peak)
            if why or rc:report['stop_reason']=why or'NONZERO_EXIT:'+spec['id']+':'+str(rc);break
            why,_=a.runtime_guard(plan,out,head,tree,deadline)
            if why:report['stop_reason']=why;break
            tr,meta=validate(dest,spec,checks);entry['gate']=next_gate(out,spec,tr);entry['loop_wall_s']=meta['loop_wall_s'];entry['snapshots']=meta['snapshot_count']
            print('[qualification] '+spec['id']+' PASS %.1fs'%wall,flush=True)
        if report['stop_reason']is None:
            result=analysis.analyse(out,plan=plan)
            with(out/'qualification_analysis.json').open('x')as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
            report['analysis_verdict']=result['verdict']
            if result['verdict']!='GO_QUALIFICATION_ONLY':report['stop_reason']='FINAL_QUALIFICATION_'+result['verdict']
    except Exception as error:report['stop_reason']='INTEGRITY_EXCEPTION:'+type(error).__name__+':'+str(error)
    report.update(status='STOPPED_INTEGRITY'if report['stop_reason']else'COMPLETED',wall_s=time.monotonic()-start,gpu_process_seconds=sum(x.get('wall_s',0)for x in report['processes']),output_bytes=base.dir_bytes(out),free_disk_gb_after=base.free_gb(out))
    frozen=a.read_json(ROOT/'docs/plans/eq1d_qualification_v1.json');report['policy_f_sha256_after']=base.sha256_file(base.RL_DIR/frozen['frozen']['policy_f_rel_to_rl_games'])
    if report['policy_f_sha256_after']!=plan['policy_f_sha256']:report.update(status='STOPPED_INTEGRITY',stop_reason='POLICY_CHANGED')
    with(out/'run_report.json').open('x')as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    base.write_manifest(out);return report


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('stage',choices=('preflight','smoke'));ap.add_argument('--approved-commit',required=True);args=ap.parse_args();p=a.read_json(PLAN);out=base.resolve(p['run']['output_root'])/p['run']['run_id']
    problems,checks=preflight(p,args.approved_commit,out)
    if problems or args.stage=='preflight':print(json.dumps(dict(problems=problems,checks=checks),indent=2));return 2 if problems else 0
    report=run(p,out,checks);print(json.dumps({k:report[k]for k in('status','stop_reason','gpu_process_seconds')},indent=2));return 0 if report['status']=='COMPLETED'else 1

if __name__=='__main__':raise SystemExit(main())
