#!/usr/bin/env python3
"""CPU-only confirmation design sensitivity, not a confirmatory execution or power guarantee."""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
from statistics import NormalDist


def paired_effective_n(delta,q,alpha=.05/3,power=.8):
    if not (0<delta<=q<=1) or not(0<alpha<1 and .5<power<1):raise ValueError('invalid paired-design assumptions')
    z=NormalDist().inv_cdf(1-alpha/2);b=NormalDist().inv_cdf(power)
    return math.ceil((z*math.sqrt(q)+b*math.sqrt(q-delta*delta))**2/(delta*delta))


def plan(analysis):
    per=analysis['primary']['per_seed'];observed=[dict(seed=int(s),geometric_events=d['n']) for s,d in per.items()]
    rows=[]
    for delta in (.10,.15,.20):
        for q in (.2,.4,.6):
            iid=paired_effective_n(delta,q)
            for m in (20,30,40):
                for rho in (0,.01,.05,.10):
                    de=1+(m-1)*rho;seeds=math.ceil(iid*de/m)
                    rows.append(dict(effect_pp=100*delta,discordance=q,paired_effective_n=iid,events_per_seed=m,icc_assumed=rho,
                                     design_effect=de,seed_count_approx=seeds,events_approx=seeds*m))
    sensitivity=[x for x in rows if x['effect_pp']==15 and x['events_per_seed']==20]
    return dict(schema='motar.h-r3-p02-confirmation-planning.v1',status='PLAN_ONLY_NOT_AUTHORIZED',
      observed_yield=observed,seed_clusters=len(per),population_uncertainty='NOT_ESTIMABLE from two clusters; no reliable pilot ICC or population power estimate',
      method='Normal-approximation paired discordance effective-n with conservative Bonferroni alpha .05/3, then equal-cluster DE=1+(m-1)*rho; future analysis Holm3 contrasts',
      power='80% per contrast under assumptions, NOT 80% joint-all-three power; correlated shared-B3 contrasts require registered simulation before final sample fixation',
      assumptions='Positive minimum effects10/15/20pp are design scenarios, not validated clinical/practical margins; q and rho are assumed grids, not fitted from two seeds; P01 p-values unused',
      proposal=dict(new_seed_range_if_approved=[4701,4732],seed_count=32,events_if_20_per_seed=640,
        conditions='Example for15pp,q<=.4,rho<=.1,yield>=20; grid sensitivity can require more (e.g.q=.6). User review and independently justified final assumptions required; no automatic enrollment/extra seed.',
        analysis='paired scene outcomes; seed as independent cluster, equal-seed contrasts B3-B2/B4/B5, Holm; cluster-robust or seed-randomization inference with enough clusters, retain all matched events and terminal failures; no event-only McNemar confirmation',
        registration='fix geometry population/identity rule, all-arm common prefix, fixed exposure, sham/nominal-budget controls, effect margins, cluster count, max scenes, yield/futility and missing-data rules before new confirmatory data; P01/P02 excluded from confirmation'),
      sensitivity_15pp_20_events_per_seed=sensitivity,grid=rows,
      limits=['IID effective event counts alone are not an executable cluster sample size.','Unequal scene yields/cluster sizes and treatment-dependent effort require further design validation.','Two exploratory seeds never confirm H-R3; current run cap remains two seeds irrespective of planning output.'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--analysis',required=True);p.add_argument('--out',required=True);a=p.parse_args()
    value=plan(json.loads(Path(a.analysis).read_text()))
    with Path(a.out).open('x') as f:json.dump(value,f,indent=2);f.write('\n')
    print(json.dumps({k:value[k] for k in ('status','observed_yield','proposal')},indent=2))
if __name__=='__main__':main()
