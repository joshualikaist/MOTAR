#!/usr/bin/env python3
"""Preregistered S-L1C v2 CPU selection. Writes fresh external raw only; no GPU/production import."""
from __future__ import annotations
import argparse
import collections
import csv
import gzip
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/audits'))
import s_l1c_v2_reference as ref
PLAN=ROOT/'docs/plans/s_l1c_v2.json'


def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()


def cell():
    return dict(cases=0,opportunities=0,true_accepts=0,false_accepts=0,abstentions=0,rejections=0,
                paired_true_losses=0,new_false_accepts=0,new_true_accepts=0,new_range_violations=0,
                new_error_sum_m=0.,new_error_max_m=0.,decisions={})


def select_candidate(gates, scores):
    eligible=[c for c in ('V2-A','V2-B') if all(gates[c].values())]
    if not eligible:return None
    return sorted(eligible,key=lambda c:(-scores[c]['new_true'],scores[c]['false_accepts'],
                                         scores[c]['new_range_mae_m'],c!='V2-A'))[0]


def evaluate(scene, prediction, bound, sign, candidate):
    scan=ref.perturb_scan(scene['scan'],bound,sign)
    result=ref.associate(scan,prediction,candidate,noise_bound_m=bound)
    h=result['column'];row=result['row']
    observed=bool((scene['labels'][:,h]==ref.v1.ref.SPHERE).any())
    actual_label=int(scene['labels'][row,h]) if result['valid'] and row>=0 else 0
    tp=bool(result['valid'] and actual_label==ref.v1.ref.SPHERE)
    fp=bool(result['valid'] and actual_label!=ref.v1.ref.SPHERE)
    error=abs(float(result['centre_range'])-float(np.linalg.norm(scene['target']))) if tp else None
    return result,observed,tp,fp,error


def run(out):
    if out.exists():raise FileExistsError('CPU raw path already exists')
    p=json.loads(PLAN.read_text())
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=ROOT,text=True).strip():
        raise ValueError('CPU selection source must be committed and clean')
    out.mkdir(parents=True)
    started=time.monotonic()
    aggregate=collections.defaultdict(cell);strata=collections.defaultdict(cell)
    counts=collections.Counter();records=0
    ledger=gzip.open(out/'matched.csv.gz','wt',newline='')
    writer=csv.writer(ledger)
    writer.writerow(['case','set','range','elevation','bearing','prediction_offset','noise_bound','noise_sign',
                     'candidate','valid','row','column','decision','genuine_hit_opportunity','true_accept','false_accept','centre_error_m'])

    def observe(scene,prediction,offset,bound,sign,case_id,scope='matrix'):
        nonlocal records
        outcomes={c:evaluate(scene,prediction,bound,sign,c) for c in ref.CANDIDATES}
        old=outcomes['V2-0']
        for c,(res,opportunity,tp,fp,error) in outcomes.items():
            aggregate_key=(scope,c,str(bound),scene['set'])
            stratum_key=(scope,c,str(bound),str(scene['r']),str(scene['el']),str(scene['b']),str(offset),scene['set'])
            for key,collection in ((aggregate_key,aggregate),(stratum_key,strata)):
                value=collection[key];value['cases']+=1
                value['opportunities']+=int(opportunity);value['true_accepts']+=int(tp);value['false_accepts']+=int(fp)
                abstain=res['decision'].startswith('ABSTAIN_');value['abstentions']+=int(abstain)
                value['rejections']+=int(not res['valid'] and not abstain)
                value['paired_true_losses']+=int(old[2] and not tp)
                value['new_false_accepts']+=int(fp and not old[3])
                new=bool(tp and not old[2]);value['new_true_accepts']+=int(new)
                if new:
                    value['new_error_sum_m']+=error;value['new_error_max_m']=max(value['new_error_max_m'],error)
                    value['new_range_violations']+=int(error>(.001 if bound==0 else .02))
                value['decisions'][res['decision']]=value['decisions'].get(res['decision'],0)+1
            writer.writerow([case_id,scene['set'],scene['r'],scene['el'],scene['b'],offset,bound,sign,c,
                int(res['valid']),res['row'],res['column'],res['decision'],int(opportunity),int(tp),int(fp),'' if error is None else error])
        records+=1

    for geometry_index,scene in enumerate(ref.geometries(ROOT)):
        counts[scene['set']]+=1
        center=scene['prediction'];norm=float(np.linalg.norm(center));unit=center/norm
        for offset in p['cpu']['prediction_radial_offsets_m']:
            prediction=center+offset*unit
            for bound in p['cpu']['noise_bound_m']:
                for sign in ((1,) if bound==0 else (1,-1)):
                    observe(scene,prediction,offset,bound,sign,geometry_index)
        if geometry_index%500==0:print('CPU geometries',geometry_index,'matched',records,flush=True)
    expected={'positive':2928,'tall':105,'narrow':480,'foreground':384,'eq1d':7}
    if dict(counts)!=expected or records!=p['cpu']['matched_cases_total']:
        raise ValueError('Preregistered matrix counts differ: '+str(counts))
    matrix_records=records
    diag=p['cpu']['angular_error_diagnostic']
    for index,(r,el,b,offset,delta) in enumerate(itertools.product(diag['ranges_m'],diag['elevations_deg'],diag['bearings_deg'],diag['radial_offsets_m'],diag['prediction_errors_bearing_elevation_deg'])):
        target=r*ref.direction(b,el);scan,labels=ref.v1.lidar_scan(sphere_center=target)
        scene=dict(set='positive',r=r,el=el,b=b,target=target,scan=scan,labels=labels)
        prediction=(r+offset)*ref.direction(b+delta[0],el+delta[1])
        observe(scene,prediction,offset,diag['noise_bound_m'],1,'diag'+str(index),'angular_diagnostic')
    ledger.close()
    if records-matrix_records!=diag['count']:raise ValueError('Angular diagnostic count mismatch')
    gates={};scores={}
    for c in ref.CANDIDATES:
        selected=[(key,value) for key,value in strata.items() if key[0]=='matrix' and key[1]==c]
        primary=[(key,value) for key,value in selected if float(key[2]) in p['cpu']['primary_noise_bounds_m']]
        near=[(key,value) for key,value in primary if key[-1]=='positive' and float(key[3]) in p['cpu']['primary_near_ranges_m']]
        gain_per_noise={str(bound):sum(v['new_true_accepts']-v['paired_true_losses'] for k,v in near if float(k[2])==bound) for bound in p['cpu']['primary_noise_bounds_m']}
        neg_ok=not any(v['new_false_accepts'] for _,v in selected)
        eq_ok=not any(v['false_accepts'] for k,v in primary if k[-1]=='eq1d')
        recall=all(value>0 for value in gain_per_noise.values()) and not any(v['paired_true_losses'] for _,v in primary)
        range_ok=not any(v['new_range_violations'] for _,v in primary)
        gates[c]={'recall':recall,'negative_no_worsening':neg_ok,'eq1d_zero':eq_ok,'new_range_error':range_ok,'clock_contract_eligible':c!='V2-C'}
        new_count=sum(v['new_true_accepts'] for _,v in primary)
        scores[c]={'new_true':new_count,'false_accepts':sum(v['false_accepts'] for _,v in selected),
                   'new_range_mae_m':sum(v['new_error_sum_m'] for _,v in primary)/new_count if new_count else 0.,
                   'gain_by_primary_noise':gain_per_noise}
    chosen=select_candidate(gates,scores)
    summary={'schema':'motar.s-l1c-v2-cpu-selection.v1','source_sha':head,'plan_sha256':sha(PLAN),
       'geometry_counts':dict(counts),'matched_matrix_cases':matrix_records,'angular_diagnostic_cases':records-matrix_records,
       'candidate_gates':gates,'scores':scores,'selected_candidate':chosen,'decision':'CPU_GO' if chosen else 'CPU_NO_GO',
       'aggregate':[dict(scope=k[0],candidate=k[1],noise_bound_m=float(k[2]),set=k[3],**v) for k,v in aggregate.items()],
       'strata':[dict(scope=k[0],candidate=k[1],noise_bound_m=float(k[2]),range_m=float(k[3]),elevation_deg=float(k[4]),
                       bearing_deg=float(k[5]),radial_offset_m=float(k[6]),set=k[7],**v) for k,v in strata.items()],
       'wall_s':time.monotonic()-started,
       'interpretation':'CPU candidate selection only; not runtime/GPU/E-L0 qualification. V2-C ineligible; noisy/angular-error findings remain explicit.'}
    (out/'selection.json').write_text(json.dumps(summary,indent=1,allow_nan=False)+'\n')
    members=[{'path':f.name,'bytes':f.stat().st_size,'sha256':sha(f)} for f in sorted(out.iterdir()) if f.is_file()]
    (out/'RAW_MANIFEST.json').write_text(json.dumps({'members':members},indent=2)+'\n')
    print(json.dumps({'decision':summary['decision'],'candidate':chosen,'gates':gates,'scores':scores,'wall_s':summary['wall_s']},indent=2))
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out')
    args=p.parse_args();plan=json.loads(PLAN.read_text())
    run(Path(args.out).resolve() if args.out else (ROOT/plan['cpu']['raw_root']).resolve())

if __name__=='__main__':main()
