#!/usr/bin/env python3
"""C-v2 fixed-matrix canary with committed inputs and inherited fail-closed supervision."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import s_l1c_run as c

a,base,b=c.a,c.base,c.b
PLAN=ROOT/'docs/plans/s_l1c_v2.json'
LAUNCHER=ROOT/'tools/s_l1c_v2_launcher.py'
PARENTS={
 'off':(a.HISTORICAL_ROOT,'v3_{config}_p2','docs/evidence/e_l0_evaluation_2026-10-18/run_records/RAW_MANIFEST.json'),
 's_l1a':(b.S_L1A_ROOT,'{config}_on_p2','docs/evidence/s_l1a_2026-10-18/run_records/RAW_MANIFEST.json'),
 's_l1b':(c.S_L1B_ROOT,'{config}_s_l1b_p2','docs/evidence/s_l1b_2026-10-18/run_records/RAW_MANIFEST.json'),
 's_l1c':(base.resolve('../aerial_gym_simulator/results/s_l1c_v1/SL1C-C01'),'{config}_s_l1c_p2','docs/evidence/s_l1c_2026-10-18/run_records/RAW_MANIFEST.json'),
 's_l1d':(base.resolve('../aerial_gym_simulator/results/s_l1d_v1/SL1D-C01'),'{config}_s_l1d_p2','docs/evidence/s_l1d_2026-10-18/run_records/RAW_MANIFEST.json')}


def plan():
    p=a.read_json(PLAN);parent=c.load_plan()
    p['policy_f_sha256']=parent['policy_f_sha256'];p['references']=parent['references'];p['switch']={'name':'NAVRL_SENSOR_LINEAGE'}
    g=p['gpu']
    p['run']={'run_id':g['run_id'],'disk_min_free_gb':g['disk_min_free_gb'],'output_max_gb':g['output_max_gb'],
      'oom_reserve_mib':g['oom_reserve_mib'],'poll_s':g['poll_s'],'total_wall_cap_s':g['total_wall_s'],
      'closed_loop_density':g['density']}
    return p


def specs(p):
    g=p['gpu'];result=[]
    for kind,groups in (('positive',range(3)),('negative',range(1))):
        for group in groups:
            for mode in ('v1','v2'):
                result.append(dict(id='%s%d_%s'%(kind,group,mode),batch='SENSORS',kind=kind,group=group,mode=mode,
                    config='static',arm='P2',seed=4401,num_envs=48 if kind=='positive' else 31,
                    lineage='s_l1c_v1' if mode=='v1' else 's_l1c_v2',cap_s=g['per_process_wall_s']))
    for config in ('static','moving'):
        for variant in g['closed_loop_variants']:
            lineage={'off':'off','s_l1a':'s_l1a_v1','s_l1b':'s_l1b_v1','s_l1c':'s_l1c_v1','s_l1d':'s_l1d_v1',
                     'v2_p1':'s_l1c_v2','v2_p2':'s_l1c_v2'}[variant]
            result.append(dict(id=config+'_'+variant,batch='V3',config=config,variant=variant,
                 arm='P1' if variant=='v2_p1' else 'P2',seed=g['closed_loop_seeds'][config],
                 num_envs=g['closed_loop_envs'],steps=g['closed_loop_steps'],lineage=lineage,cap_s=g['per_process_wall_s']))
    result.append(dict(id='firewall_v2',batch='D3',config='static',arm='P2',seed=4401,num_envs=4,
                       steps=320,lineage='s_l1c_v2',cap_s=g['per_process_wall_s']))
    return result


def preflight(p,approved,out):
    problems,checks=c.preflight(c.load_plan(),approved,out,False)
    owned=[PLAN,PLAN.with_suffix('.md'),ROOT/'docs/plans/s_l1c_v2_amendment1.md',ROOT/'docs/plans/s_l1c_v2_amendment2.md',Path(__file__).resolve(),LAUNCHER,
      ROOT/'tools/s_l1c_v2_analysis.py',ROOT/'aerial_gym/task/navrl_task/navrl_lidar_association_v2.py']
    checks['v2_committed_files']={str(x.relative_to(ROOT)):a.committed(x) for x in owned}
    if not all(checks['v2_committed_files'].values()):problems.append('C-v2 launch/analysis/plan source not committed unchanged')
    selected=a.read_json(ROOT/'docs/evidence/s_l1c_v2_2026-10-18/cpu_selection_summary.json')
    checks['cpu_selection']=selected['selected_candidate']
    if selected['decision']!='CPU_GO' or selected['selected_candidate']!='V2-A':problems.append('No eligible CPU-selected V2-A')
    checks['prior_manifests']={}
    for variant,(root,pattern,copy) in PARENTS.items():
        try:
            manifest=a.read_json(root/'RAW_MANIFEST.json');members={m['path']:m for m in manifest['members']}
            digest=base.sha256_file(root/'RAW_MANIFEST.json')
            if digest!=base.sha256_file(ROOT/copy):raise ValueError('prior manifest changed')
            for config in ('static','moving'):
                rel=pattern.format(config=config)+'/v3_steps.jsonl';record=members[rel]
                if base.sha256_file(root/rel)!=record['sha256']:raise ValueError('prior digest changed')
            checks['prior_manifests'][variant]=digest
        except (OSError,KeyError,ValueError) as error:problems.append(variant+': '+str(error))
    checks['effective_processes']=specs(p);checks['effective_run_limits']=p['run']
    return problems,checks


def comparisons(out,spec):
    if spec['batch']!='V3':return {}
    config=spec['config'];variant=spec['variant'];path=out/spec['id']/'v3_steps.jsonl'
    if variant in PARENTS:
        root,pattern,_=PARENTS[variant];other=root/pattern.format(config=config)/'v3_steps.jsonl'
    elif variant=='v2_p2':other=out/(config+'_v2_p1')/'v3_steps.jsonl'
    else:return {}
    return {spec['id']:a.compare_digest_files(path,other,spec['steps'])}


def validate(out,spec,checks):
    done=a.validate_done(out,spec,checks)
    if spec['batch']=='V3':
        level={'off':0,'s_l1a':1,'s_l1b':2,'s_l1c':3,'s_l1d':4,'v2_p1':3,'v2_p2':3}[spec['variant']]
        flags=done['flags'];expected={'s_l1a':level>=1,'s_l1b':level>=2,'s_l1c':level>=3,
                   's_l1d':spec['variant']=='s_l1d','s_l1c_v2':spec['variant'].startswith('v2_')}
        if flags!=expected:raise ValueError('Actual lineage flags differ from declared mode')
    elif spec['batch']=='D3':
        meta=a.read_json(out/'d3_meta.json')
        if meta['actor_width']!=898 or meta['rms_digest_start']!=checks['rms_digest'] or meta['rms_digest_end']!=checks['rms_digest']:
            raise ValueError('D3 actor/RMS changed')
        if done['flags']!=dict(s_l1a=True,s_l1b=True,s_l1c=True,s_l1d=False,s_l1c_v2=True):
            raise ValueError('D3 not on selected C-v2 lineage')
    else:
        meta=a.read_json(out/'sensor_meta.json')
        expected={'s_l1a':True,'s_l1b':True,'s_l1c':True,'s_l1d':False,'s_l1c_v2':spec['mode']=='v2'}
        if meta['flags']!=expected:raise ValueError('Sensor lineage flags differ')
        if any(not meta['origins'].get(k) or ROOT.resolve() not in Path(meta['origins'][k]).resolve().parents for k in ('aerial_gym','motar_eval')):
            raise ValueError('Sensor imports outside worktree')
    return done


def run(p,out,checks):
    out.mkdir(parents=True,exist_ok=False);start=time.monotonic();deadline=start+p['run']['total_wall_cap_s']
    head=base.git('rev-parse','HEAD').stdout.strip();clean=base.git('status','--porcelain','--untracked-files=no').stdout
    report=dict(schema='motar.s-l1c-v2-run.v1',source_commit=head,preflight=checks,effective_processes=specs(p),
                processes=[],noninterference={},status='RUNNING',stop_reason=None,run_dir=str(out))
    try:
        for spec in specs(p):
            reason,_=a.runtime_guard(p,out,head,clean,deadline)
            if reason:report['stop_reason']=reason;break
            gpu=base.gpu_state();cap=gpu['total_mib']-gpu['used_mib']-p['run']['oom_reserve_mib']
            if cap<=0 or '3070' not in gpu['name']:raise ValueError('GPU platform/headroom')
            entry=dict(id=spec['id'],gpu_at_launch=gpu,gpu_memory_cap_mib=cap,free_disk_gb_before=base.free_gb(out));report['processes'].append(entry)
            command=[base.PYTHON,'-B',str(LAUNCHER),'--process',spec['id'],'--out',str(out/spec['id'])]
            proc_deadline=time.monotonic()+spec['cap_s']
            def guard(proc):return a.runtime_guard(p,out,head,clean,deadline,process_deadline=proc_deadline,proc=proc,cap_mib=cap)
            rc,wall,peak,reason=a.supervised_launch(command,b.process_env(p,spec,out/spec['id']),out/(spec['id']+'.log'),guard,p['run']['poll_s'])
            entry.update(returncode=rc,wall_s=wall,peak_gpu_mib=peak,command=command)
            if reason or rc:report['stop_reason']=reason or 'NONZERO_EXIT '+spec['id'];break
            entry['process_done']=validate(out/spec['id'],spec,checks)
            paired=comparisons(out,spec);report['noninterference'].update(paired)
            if any(not r['identical'] for r in paired.values()):report['stop_reason']='NONINTERFERENCE_FAIL '+spec['id'];break
            reason,_=a.runtime_guard(p,out,head,clean,deadline)
            if reason:report['stop_reason']=reason;break
            print('[C-v2]',spec['id'],'completed',round(wall,1),'s',flush=True)
    except Exception as error:report['stop_reason']='INTEGRITY_EXCEPTION '+type(error).__name__+': '+str(error)
    report['status']='STOPPED' if report['stop_reason'] else 'COMPLETED'
    report['gpu_seconds_total']=sum(x.get('wall_s',0) for x in report['processes']);report['wall_s_total']=time.monotonic()-start
    report['free_disk_gb_after']=base.free_gb(out);report['output_bytes_total']=base.dir_bytes(out)
    frozen=a.read_json(base.resolve(p['references']['eq1d_plan']))['frozen']
    report['policy_f_sha256_after']=base.sha256_file(base.RL_DIR/frozen['policy_f_rel_to_rl_games'])
    if report['policy_f_sha256_after']!=checks['policy_f_sha256']:
        report['status']='STOPPED';report['stop_reason']='POLICY_F_CHANGED'
    with (out/'run_report.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    base.write_manifest(out)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('stage',choices=['preflight','run']);parser.add_argument('--approved-commit',required=True)
    args=parser.parse_args();p=plan();out=base.resolve(p['gpu']['raw_root']);problems,checks=preflight(p,args.approved_commit,out)
    if problems or args.stage=='preflight':print(json.dumps(dict(problems=problems,checks=checks),indent=2));return 2 if problems else 0
    report=run(p,out,checks);print(json.dumps({k:report[k] for k in ('status','stop_reason','gpu_seconds_total')},indent=2))
    return 0 if report['status']=='COMPLETED' else 1

if __name__=='__main__':sys.exit(main())
