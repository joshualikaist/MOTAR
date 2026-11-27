#!/usr/bin/env python3
"""NumPy-only independent C-v2 GPU analysis: first-hit truth, paired recall, clocks and digests."""
from __future__ import annotations
import argparse
import collections
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/audits'))
import s_l1c_v2_reference as ref


def load(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k] for k in z.files}


def covariance_ratios(before,after,direction):
    u,t1,t2=ref.v1.orthonormal_basis(direction)
    return [float(t@after@t/(t@before@t)) for t in (u,t1,t2)]


def independent_digests(left,right):
    keys=('obs','action','reward','done','captured','crashed','pursuer','target','resets','rgb','depth','lidar','semantic','tracker')
    def read(path):
        with open(path) as f:return [json.loads(line) for line in f]
    a,b=read(left),read(right)
    if len(a)!=600 or len(b)!=600:return False
    return all(x.get('step')==y.get('step')==i and all(k in x and k in y and x[k]==y[k] for k in keys) for i,(x,y) in enumerate(zip(a,b)))


def analyse(run):
    run=Path(run).resolve();report=json.loads((run/'run_report.json').read_text())
    rows=[];geometry_mismatches=0;cov_pass=True;clock_pass=True;firewall=True;provenance=True;flags=True
    model_mismatch=0;new_error_violations=0
    for kind,groups in (('positive',range(3)),('negative',range(1))):
        for group in groups:
            pair={}
            for mode in ('v1','v2'):
                root=run/('%s%d_%s'%(kind,group,mode));z=load(root/'sensor_probes.npz');meta=json.loads((root/'sensor_meta.json').read_text())
                pair[mode]=(z,meta)
                clock_pass &= all(meta[k] for k in ('identity_age_10_losses_ok','active_on_miss50','expired_on_miss51','camera_resets_identity'))
                expected_ages=np.arange(1,11)[:,None]*.1
                clock_pass &= np.allclose(z['clock_identity_trace'],expected_ages,atol=1e-5,rtol=0)
                clock_pass &= bool(z['clock_expiry_active'][:50].all() and not z['clock_expiry_active'][50].any())
                clock_pass &= bool(z['clock_camera_valid'].all() and (z['clock_camera_identity_age']==0).all() and (z['clock_camera_update_source']==1).all())
                firewall &= meta['firewall_equal']
                flags &= meta['flags']==dict(s_l1a=True,s_l1b=True,s_l1c=True,s_l1d=False,s_l1c_v2=mode=='v2')
                angles=z['angles'].astype(float);elev=z['elevations'].astype(float)
                beams=np.stack(np.broadcast_arrays(np.cos(elev[:,None])*np.cos(angles),np.cos(elev[:,None])*np.sin(angles),np.sin(elev[:,None])+0*angles),axis=-1)
                for i,s in enumerate(meta['scenes']):
                    boxes=meta['layouts'][i];target=z['target'][i].astype(float)-z['drone'][i].astype(float) if s['target'] is not None else None
                    truth=ref.v1.ref.first_hits(np.zeros(3),beams,[x['center'] for x in boxes],[x['half'] for x in boxes],sphere_center=target,sphere_radius=.2,far_m=12.)
                    scan=z['lidar'][i].reshape(4,72).astype(float)
                    geometric_ok=np.abs(truth['depth_m']-scan)<=1e-4
                    geometry_mismatches+=int((~geometric_ok).sum())
                    classes=truth['hit_class']
                    for oi,off in enumerate((-.15,0,.15)):
                        prefix='o%d_gate_'%oi
                        valid=bool(z[prefix+'valid'][i]);r=int(z[prefix+'row'][i]);h=int(z[prefix+'column'][i])
                        tp=valid and classes[r,h]==ref.v1.ref.SPHERE;fp=valid and not tp
                        opportunity=bool((classes[:,h]==ref.v1.ref.SPHERE).any())
                        pred=z[prefix+'prediction_rel'][i].astype(float)
                        candidate='V2-0' if mode=='v1' else 'V2-A'
                        expected=ref.associate(scan,pred,candidate,noise_bound_m=.0001)
                        model_mismatch+=int(valid!=expected['valid'])
                        error=abs(float(z[prefix+'centre'][i])-float(np.linalg.norm(target))) if tp else None
                        label=int(z[prefix+'el0_label'][i])
                        provenance &= (not valid) or (label==(1 if tp else 2) and bool(z[prefix+'el0_consistent'][i]))
                        rows.append(dict(kind=kind,group=group,index=i,scene=s['id'],mode=mode,range_m=s['r'],elevation_deg=s['el'],bearing_deg=s['b'],
                            radial_offset_m=off,opportunity=opportunity,true_accept=bool(tp),false_accept=bool(fp),valid=valid,
                            centre_error_m=error,first_hit_class=int(classes[r,h]) if valid else None,
                            source='frozen_EQ1D_GPU_replay' if s.get('replay') else 'new_GPU_render',
                            model_expected=bool(expected['valid'])))
                        covariance='o%d_cov_'%oi
                        if z[covariance+'valid'][i]:
                            ratios=covariance_ratios(z[covariance+'cov_before'][i].astype(float),z[covariance+'cov_after'][i].astype(float),z[covariance+'prediction_rel'][i])
                            cov_pass &= ratios[0]<1 and ratios[1]>=.99 and ratios[2]>=.99
                        clock_pass &= z[prefix+'identity_age'][i]==5.
                        clock_pass &= z[prefix+'fused_age'][i]==(0. if valid else 1.)
                        for label in ('gate','cov'):
                            for key in ('valid','measured','state','cov_after','identity_age','fused_age'):
                                firewall &= np.array_equal(z['o%d_%s_%s'%(oi,label,key)][i],z['o%d_%s_replay_%s'%(oi,label,key)][i])
            # Matched rendered inputs, not merely the intended geometry.
            flags &= np.array_equal(pair['v1'][0]['lidar'],pair['v2'][0]['lidar'])
            flags &= np.array_equal(pair['v1'][0]['semantic'],pair['v2'][0]['semantic'])
    grouped=collections.defaultdict(dict)
    for row in rows:grouped[(row['kind'],row['group'],row['index'],row['radial_offset_m'])][row['mode']]=row
    lost=newfalse=gain=0;near_n={m:0 for m in ('v1','v2')};near_tp=dict(near_n)
    for pair in grouped.values():
        old,new=pair['v1'],pair['v2']
        lost+=int(old['true_accept'] and not new['true_accept']);newfalse+=int(new['false_accept'] and not old['false_accept'])
        if new['kind']=='positive' and new['range_m']<2.295:
            for m in ('v1','v2'):near_n[m]+=int(pair[m]['opportunity']);near_tp[m]+=int(pair[m]['true_accept'])
            gain+=int(new['true_accept'])-int(old['true_accept'])
        if new['true_accept'] and not old['true_accept'] and new['centre_error_m']>.02:new_error_violations+=1
    # All 600-step/14-key historical and E-L0 comparisons are recorded by the fail-closed runner.
    nonint=report.get('noninterference',{})
    regression=len(nonint)==12 and all(v['identical'] for v in nonint.values())
    history=ROOT.parent/'aerial_gym_simulator/results'
    paths={'off':('e_l0_evaluation_v1/EL0-R1','v3_{config}_p2'),
           's_l1a':('s_l1a_v1/SL1A-C01','{config}_on_p2'),'s_l1b':('s_l1b_v1/SL1B-C01','{config}_s_l1b_p2'),
           's_l1c':('s_l1c_v1/SL1C-C01','{config}_s_l1c_p2'),'s_l1d':('s_l1d_v1/SL1D-C01','{config}_s_l1d_p2')}
    for config in ('static','moving'):
        for variant,(root,pattern) in paths.items():
            regression &= independent_digests(run/(config+'_'+variant)/'v3_steps.jsonl',history/root/pattern.format(config=config)/'v3_steps.jsonl')
        regression &= independent_digests(run/(config+'_v2_p1')/'v3_steps.jsonl',run/(config+'_v2_p2')/'v3_steps.jsonl')
    fm=json.loads((run/'firewall_v2/d3_meta.json').read_text());fz=load(run/'firewall_v2/d3_branches.npz')
    actor_firewall=bool(fm['branch_log']) and fm['actor_width']==898 and fm['rms_digest_start']==fm['rms_digest_end'] and not fm['model_training_flag'] and not fm['rms_training_flag']
    actor_firewall &= any(r['reason']=='reset_boundary' for r in fm['branch_log'])
    actor_firewall &= {50,150,300}<={r['step'] for r in fm['branch_log']}
    positive_controls=0
    for row in fm['branch_log']:
        prefix='s%03d_'%row['step']
        for kind in ('B_all','B_perm','B_subset'):
            actor_firewall &= np.array_equal(fz[prefix+'A_obs'],fz[prefix+kind+'_obs'])
            actor_firewall &= all(row['branches'][kind]['actions_equal'])
        positive=row['branches']['P_control']
        if positive['in_view_envs']>0:
            positive_controls+=1;actor_firewall &= positive['changed_envs']>0
    actor_firewall &= positive_controls>0
    strata=collections.defaultdict(lambda:dict(opportunities=0,true_accepts=0,false_accepts=0,rejected=0))
    negatives=collections.defaultdict(lambda:dict(cases=0,false_accepts=0,true_accepts=0))
    for row in rows:
        if row['kind']=='positive':
            key=(row['mode'],row['range_m'],row['elevation_deg'])
            s=strata[key];s['opportunities']+=int(row['opportunity']);s['true_accepts']+=int(row['true_accept']);s['false_accepts']+=int(row['false_accept']);s['rejected']+=int(not row['valid'])
        else:
            key=(row['mode'],row['source']);s=negatives[key];s['cases']+=1;s['false_accepts']+=int(row['false_accept']);s['true_accepts']+=int(row['true_accept'])
    eq1d_zero=all(v['false_accepts']==0 for k,v in negatives.items() if k[1]=='frozen_EQ1D_GPU_replay')
    gates={'GPU-RECALL':gain>0 and lost==0,'GPU-NEG':newfalse==0 and eq1d_zero,
           'GPU-RANGE':new_error_violations==0,'C-COV':bool(cov_pass),'C-CLOCK':bool(clock_pass),
           'C-REGRESSION':regression,'C-FIREWALL':bool(firewall and actor_firewall),'E-L0-PROVENANCE':bool(provenance),
           'INTEGRITY':report['status']=='COMPLETED' and len(report['processes'])==23 and geometry_mismatches==0 and model_mismatch==0 and bool(flags)}
    return dict(schema='motar.s-l1c-v2-gpu-analysis.v1',source_sha=report['source_commit'],gates={k:'PASS' if v else 'FAIL' for k,v in gates.items()},
       decision='GPU_QUALIFIED_PENDING_REVIEW' if all(gates.values()) else 'GPU_NO_GO',
       near_recall={m:dict(true_accepts=near_tp[m],opportunities=near_n[m],rate=near_tp[m]/near_n[m] if near_n[m] else None) for m in near_n},
       paired_true_losses=lost,new_false_accepts=newfalse,independent_model_mismatches=model_mismatch,geometry_mismatches=geometry_mismatches,
       actor_firewall=dict(pass_gate=bool(actor_firewall),branches=len(fm['branch_log']),positive_controls=positive_controls),
       strata=[dict(mode=k[0],range_m=k[1],elevation_deg=k[2],**v) for k,v in sorted(strata.items())],
       negatives=[dict(mode=k[0],source=k[1],**v) for k,v in sorted(negatives.items())],rows=rows,
       noninterference=nonint,resources={k:report[k] for k in ('gpu_seconds_total','wall_s_total','free_disk_gb_after','output_bytes_total')},
       limits=['Precision-limited simulated LiDAR sphere model; not hardware realism or Recovery performance.',
               'C-v2 does not enable D; D-v1 GO is not inherited.',
               'Seven negatives are original sensor replays; 24 negatives and 144 positives are new render fixtures.',
               'ON behavior is frozen-checkpoint robustness; no PPO or algorithm performance conclusion.'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True);p.add_argument('--out',required=True)
    args=p.parse_args();result=analyse(args.run)
    path=Path(args.out);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(decision=result['decision'],gates=result['gates'],near_recall=result['near_recall']),indent=2))

if __name__=='__main__':main()
