import json
from pathlib import Path
import numpy as np
BASE=Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1')
RAW=BASE/'../aerial_gym_simulator/results/h_r3_p02_v1/HR3-P02'
OUT=Path('/tmp/motar_hr3_p02_20261018')
KEYS=['digest','sensor_digest','before_pos','actual_pos','actual_vel','target_pos','rng_probe','robot_pos','belief_pos','action','reset','crashed','captured','timeout','intervening','command','raw_nearest_m','camera','id_reported_centroid_uv','id_detector_px','contact_peak_n','contact_substeps','t0','initial_digest','initial_sensor_digest']
def load(seed,arm):
 with np.load(RAW/('seed%d_%s'%(seed,arm))/'h_r3_p02_trace.npz') as f:return {k:f[k] for k in KEYS}
def equality(x,y):
 a=np.ascontiguousarray(x).view(np.uint8).reshape(x.shape[0],x.shape[1],-1)
 b=np.ascontiguousarray(y).view(np.uint8).reshape(y.shape[0],y.shape[1],-1)
 return (a==b).all(-1)
def value(x):return x.tolist() if hasattr(x,'tolist') else x
def numdiff(a,b,k,e):
 x,y=a[k,e],b[k,e]
 o={'a':value(x),'b':value(y),'bitwise_equal':x.tobytes()==y.tobytes()}
 if np.issubdtype(np.asarray(x).dtype,np.number):
  o['max_abs_diff']=float(np.max(np.abs(np.asarray(x,dtype=np.float64)-np.asarray(y,dtype=np.float64))))
 return o
report={'schema':'motar.hr3p02.main-failure-readonly.v1','source_head':'d64cf84185018ab720040f266a4a481929cfa62b','raw':str(RAW.resolve()),'pair_reports':[],'findings':[]}
for seed in (4601,4602):
 ref=load(seed,'B0')
 for arm in ['B1','B2','B3','B4','B5']:
  alt=load(seed,arm); T,N=ref['digest'].shape[:2]
  ed=equality(ref['digest'],alt['digest']); es=equality(ref['sensor_digest'],alt['sensor_digest'])
  ends=np.array([min([int(t) for t in (ref['t0'][e],alt['t0'][e]) if t>=0] or [T]) for e in range(N)])
  mask=np.arange(T)[:,None]<ends[None,:]
  locations=np.argwhere(mask & ~(ed&es))
  p={'seed':seed,'arm':arm,'initial_equal':{key:bool(np.array_equal(ref[key],alt[key])) for key in ('initial_digest','initial_sensor_digest')},'t0_mismatch_envs':np.flatnonzero(ref['t0']!=alt['t0']).tolist(),'true_prebranch_mismatch_envs':np.unique(locations[:,1]).tolist() if locations.size else [],'earliest_true_prebranch':None}
  for key in ['before_pos','actual_pos','actual_vel','action','command','target_pos','rng_probe','belief_pos','sensor_digest','digest','robot_pos','raw_nearest_m']:
   loc=np.argwhere(mask & ~equality(ref[key],alt[key]))
   p['first_'+key]=loc[0].tolist() if loc.size else None
  if locations.size:
   k,e=map(int,locations[0]); details={}
   for j in range(max(0,k-2),min(T,k+3)):
    details[str(j)]={'reset_B0':np.flatnonzero(ref['reset'][j]).tolist(),'reset_'+arm:np.flatnonzero(alt['reset'][j]).tolist(),'differing_digest_envs':np.flatnonzero(~ed[j]).tolist(),'differing_sensor_envs':np.flatnonzero(~es[j]).tolist(),'fields':{key:numdiff(ref[key],alt[key],j,e) for key in ['before_pos','actual_pos','actual_vel','action','command','target_pos','rng_probe','belief_pos','robot_pos','raw_nearest_m','camera','contact_peak_n','contact_substeps','id_reported_centroid_uv','id_detector_px']}}
   p['earliest_true_prebranch']={'step_zero_based':k,'env':e,'t0_B0':int(ref['t0'][e]),'t0_'+arm:int(alt['t0'][e]),'around':details}
  report['pair_reports'].append(p)
summary=[]
for p in report['pair_reports']:
 f=p['earliest_true_prebranch']; s=None if f is None else (f['step_zero_based'],f['env'])
 summary.append({'seed':p['seed'],'arm':p['arm'],'earliest':s,'first_before':p['first_before_pos'],'first_vel':p['first_actual_vel'],'first_action':p['first_action'],'first_target':p['first_target_pos'],'first_probe':p['first_rng_probe'],'n_true_prebranch_envs':len(p['true_prebranch_mismatch_envs'])})
report['summary']=summary
(OUT/'main_failure_audit.json').write_text(json.dumps(report,indent=2,allow_nan=True)+'\n')
print(json.dumps(summary,indent=2))
# Focused chronology and numerical localization, retained independently of the main analysis.
report['first_reset_chronology']=[]
report['first_prebranch_numeric_differences']=[]
for seed in (4601,4602):
 a=load(seed,'B0')
 for arm in ['B0','B1','B2','B3','B4','B5']:
  b=load(seed,arm); loc=np.flatnonzero(b['reset'].any(1))
  k=int(loc[0]) if loc.size else None
  report['first_reset_chronology'].append({'seed':seed,'arm':arm,'first_reset_step':k,'envs':np.flatnonzero(b['reset'][k]).tolist() if k is not None else []})
 for arm in ['B1','B3']:
  pair=next(p for p in report['pair_reports'] if p['seed']==seed and p['arm']==arm)
  b=load(seed,arm)
  for key in ['action','actual_vel','actual_pos','belief_pos']:
   loc=pair['first_'+key] if 'first_'+key in pair else None
   if loc:
    k,e=loc;report['first_prebranch_numeric_differences'].append({'seed':seed,'arm':arm,'key':key,'step':k,'env':e,'difference':numdiff(a[key],b[key],k,e)})
report['findings']=[
 {'status':'OBSERVED','claim':'All initial state and raw sensor digests match across arms.'},
 {'status':'OBSERVED','claim':'Seed4602 earliest genuine prebranch digest/sensor failure is zero-based step7/env2; its B0/B1 t0 is18. Seed4601 earliest is step11/env1 with t0=15.'},
 {'status':'OBSERVED','claim':'Earliest failures coincide exactly with first differing reset schedules:4602 B0 first reset step7 env56 versus other arms step8;4601 B0 and B3 first reset step11 envs120,123 versus B1/B2/B4/B5 later.'},
 {'status':'OBSERVED','claim':'At earliest failing rows all separately recorded numeric before/actual/root position, velocity, action, command, belief position, target position and RNG probe fields remain bitwise equal. Full state and combined raw sensor hashes differ. Therefore the first recorded divergence is in unexpanded observation/sensor/history components.'},
 {'status':'SOURCE_FACT','source':'aerial_gym/env_manager/warp_env_manager.py:40','claim':'reset_idx(env_ids) rewrites vertex_maps_per_env_updated[:] for ALL envs from current actor root poses, then refits only meshes for env_ids.'},
 {'status':'HYPOTHESIS_CONSISTENT_WITH_EVIDENCE_NOT_PROVEN','claim':'An unrelated first reset recomputes every scene mesh from post-physics actor poses; small changes such as PhysX-normalized actor quaternions can alter other-env ray casts. This explains sensor-first all-env differences without initial root-state/target/RNG changes. Mesh vertices and static actor root histories were NOT_RECORDED, so this mechanism cannot be proved retrospectively.'},
 {'status':'LIMIT','claim':'Raw RGB/depth/LiDAR were stored jointly as per-env SHA256 digests, so these records cannot identify which raw modality/pixels first changed. Robot orientation/angular velocity and derived kinematics are inside hashes without separate numeric traces; therefore they cannot be numerically excluded.'},
 {'status':'DECISION','claim':'Main remains INTEGRITY_FAIL. No post-hoc exclusions or salvaged causal sensor/control comparisons. N16 diagnostic does not establish N128 intervention/reset noninterference.'}]
def strict(obj):
 if isinstance(obj,float) and not np.isfinite(obj):return str(obj)
 if isinstance(obj,dict):return {k:strict(v) for k,v in obj.items()}
 if isinstance(obj,list):return [strict(v) for v in obj]
 return obj
report=strict(report)
(OUT/'main_failure_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
lines=['# H-R3 P02 main integrity failure: independent read-only localization','',
'Source HEAD: `d64cf84185018ab720040f266a4a481929cfa62b`. No repository edits or GPU execution.',
'Raw: `'+str(RAW.resolve())+'`. All step indices below are zero based.','',
'| Seed | Arm versus B0 | Earliest genuine prebranch state/sensor mismatch | First prebranch action/velocity difference |',
'|---|---|---|---|']
for p in report['pair_reports']:
 f=p['earliest_true_prebranch'];loc='none' if f is None else 'step%d / env%d'%(f['step_zero_based'],f['env'])
 lines.append('| %d | %s | %s | action %s; velocity %s |'%(p['seed'],p['arm'],loc,p['first_action'],p['first_actual_vel']))
lines+=['','Seed4602 first onset: B0 resets env56 at step7; B1–B5 do not. Env2 has t0=18 in both B0/B1. Seed4601 first onset: B0/B3 reset envs120/123 at step11; B1/B2/B4/B5 do not. Env1 has t0=15. Immediately before these steps reset sets are empty.','',
'At both earliest failing rows, all separately recorded numeric before/actual/root positions, velocity, action, command, target, belief position and RNG probes are bitwise equal. Only full-state and combined raw-sensor hashes differ initially. The first prebranch action/velocity differences follow one step later (4602: step8/env14;4601:step12/env22). The JSON retains exact values, per-field first locations and surrounding reset sets.','',
'`warp_env_manager.py:40–48` is the strongest verified source coupling: its reset method ignores env_ids when rewriting the entire shared vertex buffer, and uses env_ids only for mesh refit. Recomputing all static mesh vertices from current actor poses after physics can change unrelated ray casts, for example after actor quaternion normalization. This is consistent with the chronology, not a proved numerical cause: mesh vertices/static actor poses were NOT_RECORDED. Full root setter outside reset and unrecorded orientation/derived-state changes also cannot be conclusively excluded retrospectively.','',
'Combined raw-sensor hashes cannot distinguish RGB, depth or LiDAR as the first changed modality. Orientation, angular velocity, observation rows and histories were hashed but not independently retained numerically. RNG/target differences occurring much later follow divergent outcomes/reset schedules and do not explain onset.','',
'Keep the main run INTEGRITY_FAIL. No salvage exclusions. A future separately approved diagnostic must inspect global mesh writes and snapshot per-modality raw products, static actor/root orientations, mesh vertices and all derived kinematics around the first differing reset, at N128. N16 smoke PASS was insufficient.','',
'Git Traceability: read-only at source HEAD above; only `/tmp/motar_hr3_p02_20261018/main_failure_audit.py`, `.json`, `.md` written. No repository changes, commits, GPU work, push or merge; raw/checkpoints unchanged.']
(OUT/'main_failure_audit.md').write_text('\n'.join(lines)+'\n')
print('WROTE',OUT/'main_failure_audit.json',OUT/'main_failure_audit.md')
print('FIRST_NUMERIC',json.dumps(report['first_prebranch_numeric_differences'][:4]))
report['smoke_shared_reset_blindspot']={'run':'HR3-P02-SMOKE02','seed':4601,'num_envs':16,'first_reset_both':{'step':27,'envs':[4]},'forced_injection_step':40,'baseline_reset_envs':[5,6],'forced_reset_envs':[5,6,15],'interpretation':'Both runs already performed a global Warp vertex refresh at27 and both invoke it again at40. The forced reset tests membership within a shared reset transaction, not a reset-vs-no-reset geometry refresh at the first differing reset step.'}
report['future_diagnostics_planned_not_executed']=[
 {'kind':'CPU_INDEPENDENT_GEOMETRY_ISOLATION_TEST','proposal':'Use two or more fake scenes, unique actor-to-vertex maps, independent expected per-env transforms and fake mesh.refit counters. Between snapshots change an unselected scene actor pose by a small floating-point increment (including a normalized quaternion fixture), reset a different selected scene, and assert every unselected vertex remains bitwise unchanged while selected geometry follows its expected transform. Existing global reset_idx[:] logic violates that isolation contract. Verify selected-only refit and order/split invariance without Isaac/GPU. This tests the proposed isolation invariant, not whether main actually had such pose changes.'},
 {'kind':'REAL_N128_NONINTERFERENCE_DIAGNOSTIC','proposal':'Only with new explicit approval/preregistration: N128 identical-action B0 pair, inject one unrelated reset at an early step with NO natural reset in reference (preferably before its first natural reset), so one process executes a reset and the other does not. Record before/after per-env static actor poses, transformed mesh vertices, robot derived kinematics, root orientation/angular velocity and separate RGB/depth/LiDAR/semantic arrays. Require all untouched envs byte-identical through future steps. Include natural-reset-coincident and noncoincident injections; do not pool/retry the failed P02 measurement.'}]
(OUT/'main_failure_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
with (OUT/'main_failure_audit.md').open('a') as f:
 f.write('\nThe SMOKE02 blind spot is observed directly: both N16 runs first reset env4 at step27. At forced step40 baseline naturally resets [5,6], while forced resets [5,6,15]. Both therefore execute the global Warp refresh at the same times around the injection; this does not exercise reset-versus-no-reset geometry refresh before the first natural reset.\n')
 f.write('\nFuture tests, PLANNED and NOT_EXECUTED: (1) CPU independent per-env geometry oracle with actor-to-vertex maps and fake refit counters; perturb an unselected scene input pose by a tiny increment/quaternion normalization, reset another scene, and require unselected geometry bytes to remain unchanged, selected transforms correct, and order/split invariance. This tests isolation, not the unrecorded historical mechanism. (2) Separately preregistered real N128 B0 forced-reset pair at an early step with no natural reset in the reference, preferably before its first natural reset; retain before/after static actor roots, vertex arrays, derived kinematics, root orientations/angular velocities and separate raw sensor modalities, then require all untouched env future state/sensors/probes byte-identical. No implementation or GPU diagnostic was performed in this audit.\n')
