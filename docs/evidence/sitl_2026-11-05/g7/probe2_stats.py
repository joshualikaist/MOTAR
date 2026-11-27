
import resource
resource.setrlimit(resource.RLIMIT_AS,(4294967296,4294967296)); resource.setrlimit(resource.RLIMIT_CPU,(300,300))
import json, pathlib, operator, fractions, math
import numpy as np
RUN=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/actionhold_g7_v1')
REF=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00')
A=pathlib.Path('/home/fair/motar_tmp/g7_20261105/opus_results_review')
NB=10000; SEED=20261105; ARMS=('k2d0','k4d0','k1d1'); DENS=(70,205)
def cid(d): return 'E2_obstacle_aware__'+str(d)+'bars__seed4501'
def load(p):
    cr=[]; ca=[]; ev=[]
    for line in open(str(p/'episodes.jsonl')):
        e=json.loads(line)
        if e['in_quota']:
            cr.append(int(e['outcome']=='crash')); ca.append(int(e['outcome']=='capture')); ev.append(e['env_index'])
    return np.array(cr), np.array(ca), np.array(ev)
D={}
for d in DENS:
    D[('F',d)]=load(REF/cid(d))
    for a in ARMS: D[(a,d)]=load(RUN/a/cid(d))
def reading(delta,lo,hi):
    if operator.le(delta,2.0) and operator.le(hi,4.0): return 'TOLERANT'
    if operator.ge(delta,5.0) and operator.gt(lo,0.0): return 'HARMFUL'
    return 'INCONCLUSIVE'
def q(v): return [float(x) for x in np.percentile(v,[2.5,97.5])]
R={'exact':{},'boot_same_stream_choice':{},'boot_same_stream_integers':{},'boot_per_arm_rng':{},'boot_alt_seeds':{},'env_cluster_boot':{},'env_icc':{}}
for a in ARMS:
    n=sum(len(D[(a,d)][0]) for d in DENS)
    num=sum(len(D[(a,d)][0])*(fractions.Fraction(int(D[(a,d)][0].sum()),len(D[(a,d)][0]))-fractions.Fraction(int(D[('F',d)][0].sum()),len(D[('F',d)][0]))) for d in DENS)
    delta=100*num/n
    xa=sum(int(D[(a,d)][0].sum()) for d in DENS); xf=sum(int(D[('F',d)][0].sum()) for d in DENS)
    capd=sum(len(D[(a,d)][1])*(D[(a,d)][1].mean()-D[('F',d)][1].mean()) for d in DENS)*100/n
    R['exact'][a]={'delta_pp':float(delta),'delta_fraction':str(delta),'arm_crashes':xa,'F_crashes':xf,'episodes_per_side':n,
        'extra_crashes':xa-xf,'max_extra_for_delta_le_2':math.floor(0.02*n),'per_density_pp':{str(d):float(100*(D[(a,d)][0].mean()-D[('F',d)][0].mean())) for d in DENS},
        'capture_delta_pp':float(capd),'per_density_capture_pp':{str(d):float(100*(D[(a,d)][1].mean()-D[('F',d)][1].mean())) for d in DENS}}
def boot(rng,a,mode):
    tot=np.zeros(NB); W=0
    for d in DENS:
        x=D[(a,d)][0].astype(float); f=D[('F',d)][0].astype(float)
        if mode=='choice':
            bx=rng.choice(x,size=(NB,len(x)),replace=True).mean(axis=1); bf=rng.choice(f,size=(NB,len(f)),replace=True).mean(axis=1)
        else:
            bx=x[rng.integers(0,len(x),size=(NB,len(x)))].mean(axis=1); bf=f[rng.integers(0,len(f),size=(NB,len(f)))].mean(axis=1)
        tot+=len(x)*(bx-bf); W+=len(x)
    return q(100*tot/W)
for mode in ('choice','integers'):
    rng=np.random.default_rng(SEED)
    for a in ARMS:
        ci=boot(rng,a,mode); R['boot_same_stream_'+mode][a]={'ci95':ci,'reading':reading(R['exact'][a]['delta_pp'],ci[0],ci[1])}
for a in ARMS:
    ci=boot(np.random.default_rng(SEED),a,'integers'); R['boot_per_arm_rng'][a]={'ci95':ci,'reading':reading(R['exact'][a]['delta_pp'],ci[0],ci[1])}
for a in ARMS:
    cis=[boot(np.random.default_rng(s),a,'integers') for s in (1,2,3,4,5)]
    R['boot_alt_seeds'][a]={'lo_range':[min(c[0] for c in cis),max(c[0] for c in cis)],'hi_range':[min(c[1] for c in cis),max(c[1] for c in cis)],'readings':sorted(set(reading(R['exact'][a]['delta_pp'],c[0],c[1]) for c in cis))}
def envsums(t):
    cr,ca,ev=t; s=np.zeros(128); np.add.at(s,ev,cr); return s
rng=np.random.default_rng(SEED)
for a in ARMS:
    tot=np.zeros(NB); W=0
    for d in DENS:
        sx=envsums(D[(a,d)]); sf=envsums(D[('F',d)]); nx=len(D[(a,d)][0]); nf=len(D[('F',d)][0])
        bx=sx[rng.integers(0,128,size=(NB,128))].sum(axis=1)/nx; bf=sf[rng.integers(0,128,size=(NB,128))].sum(axis=1)/nf
        tot+=nx*(bx-bf); W+=nx
    ci=q(100*tot/W); R['env_cluster_boot'][a]={'ci95':ci,'would_read':reading(R['exact'][a]['delta_pp'],ci[0],ci[1])}
for key in [('F',70),('F',205)]+[(a,d) for a in ARMS for d in DENS]:
    s=envsums(D[key]); p=s.sum()/2048.0
    R['env_icc'][key[0]+'/'+str(key[1])]={'var_ratio_env_vs_binomial':float(np.var(s/16.0,ddof=1)/(p*(1-p)/16.0)) if p*(1-p) else None}
R['k2d0_one_crash_fewer']={'delta_pp':100*(R['exact']['k2d0']['extra_crashes']-1)/R['exact']['k2d0']['episodes_per_side']}
(A/'probe2_stats.json').open('x').write(json.dumps(R,indent=1,sort_keys=True)+chr(10))
print(json.dumps(R,indent=1,sort_keys=True))
