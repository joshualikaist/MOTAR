#!/usr/bin/env python3
"""Post-run reporting only. Imports the independent NumPy analysis, never runtime code."""
import argparse,collections,csv,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
import s_l1c_v2_analysis as analysis
parser=argparse.ArgumentParser(description='Derive C-v2 tables and covariance/range/abstention supplement from immutable GPU raw.')
parser.add_argument('--run',required=True);parser.add_argument('--analysis',required=True);parser.add_argument('--cpu-manifest',required=True);parser.add_argument('--out-dir',required=True)
args=parser.parse_args();E=Path(args.out_dir);E.mkdir(parents=True,exist_ok=False);R=Path(args.run)
ANALYSIS=Path(args.analysis);CPU_MANIFEST=Path(args.cpu_manifest)
a=json.loads(ANALYSIS.read_text());run=json.loads((R/'run_report.json').read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def grouped(fields,positive=True):
 out=collections.defaultdict(lambda:dict(cases=0,opportunities=0,true_accepts=0,false_accepts=0,rejected=0))
 for row in a['rows']:
  if (row['kind']=='positive')!=positive:continue
  key=tuple(row[k] for k in fields);s=out[key];s['cases']+=1;s['opportunities']+=row['opportunity'];s['true_accepts']+=row['true_accept'];s['false_accepts']+=row['false_accept'];s['rejected']+=not row['valid']
 return [dict(zip(fields,k),**v) for k,v in sorted(out.items())]
pairs=collections.defaultdict(dict)
for row in a['rows']:pairs[(row['kind'],row['group'],row['index'],row['radial_offset_m'])][row['mode']]=row
new_errors=[v['v2']['centre_error_m'] for v in pairs.values() if v['v2']['true_accept'] and not v['v1']['true_accept']]
old_errors={m:[r['centre_error_m'] for r in a['rows'] if r['mode']==m and r['true_accept']] for m in ('v1','v2')}
ratios={m:[] for m in ('v1','v2')};decisions=collections.Counter();geometry_max=0
closed_loop=[]
for config in ('static','moving'):
 for variant in ('s_l1c','v2_p2'):
  z=analysis.load(R/(config+'_'+variant)/'identity_trace.npz')
  values,counts=np.unique(z['update_source'],return_counts=True)
  closed_loop.append(dict(config=config,mode='v1' if variant=='s_l1c' else 'v2',
    frames=int(z['update_source'].size),update_source_counts={str(int(v)):int(n) for v,n in zip(values,counts)},
    max_identity_age_s=float(z['identity_age'].max()),max_fused_age_s=float(z['track_age'].max())))
for m in ratios:
 for prefix in ('positive0','positive1','positive2','negative0'):
  p=R/(prefix+'_'+m);z=analysis.load(p/'sensor_probes.npz');meta=json.loads((p/'sensor_meta.json').read_text())
  for i,scene in enumerate(meta['scenes']):
   for oi in range(3):
    key='o%d_cov_'%oi
    if z[key+'valid'][i]:ratios[m].append(analysis.covariance_ratios(z[key+'cov_before'][i].astype(float),z[key+'cov_after'][i].astype(float),z[key+'prediction_rel'][i]))
    key='o%d_gate_'%oi
    d=analysis.ref.associate(z['lidar'][i].reshape(4,72),z[key+'prediction_rel'][i], 'V2-0' if m=='v1' else 'V2-A',noise_bound_m=.0001)
    decisions[(m,'positive' if prefix.startswith('positive') else 'negative',d['decision'])]+=1
  angles=z['angles'].astype(float);el=z['elevations'].astype(float)
  beams=np.stack(np.broadcast_arrays(np.cos(el[:,None])*np.cos(angles),np.cos(el[:,None])*np.sin(angles),np.sin(el[:,None])+0*angles),axis=-1)
  for i,scene in enumerate(meta['scenes']):
   boxes=meta['layouts'][i];target=z['target'][i].astype(float)-z['drone'][i].astype(float) if scene['target'] is not None else None
   truth=analysis.ref.v1.ref.first_hits(np.zeros(3),beams,[b['center'] for b in boxes],[b['half'] for b in boxes],sphere_center=target,sphere_radius=.2,far_m=12.)
   geometry_max=max(geometry_max,float(np.max(abs(truth['depth_m']-z['lidar'][i].reshape(4,72)))))
metrics=dict(schema='motar.s-l1c-v2-gpu-supplement.v1',source_sha=a['source_sha'],analysis_sha256=sha(ANALYSIS),
 by_range=grouped(('mode','range_m')),by_elevation=grouped(('mode','elevation_deg')),
 by_negative_scene=grouped(('mode','scene'),False),
 decision_counts=[dict(mode=k[0],set=k[1],decision=k[2],cases=v) for k,v in sorted(decisions.items())],
 new_correct_range=dict(count=len(new_errors),mae_m=float(np.mean(new_errors)),max_m=max(new_errors)),
 all_correct_range={m:dict(count=len(v),mae_m=float(np.mean(v)),max_m=max(v)) for m,v in old_errors.items()},
 covariance={m:dict(updates=len(v),radial_ratio_min=float(np.min(np.array(v)[:,0])),radial_ratio_max=float(np.max(np.array(v)[:,0])),orthogonal_ratio_min=float(np.min(np.array(v)[:,1:])),orthogonal_ratio_max=float(np.max(np.array(v)[:,1:]))) for m,v in ratios.items()},
 geometry_max_abs_error_m=geometry_max,
 peak_gpu_used_mib=max(p['peak_gpu_mib'] for p in run['processes']),gpu_process_wall_hours=run['gpu_seconds_total']/3600,
 manifest_sha256=sha(R/'RAW_MANIFEST.json'),manifest_members=len(json.loads((R/'RAW_MANIFEST.json').read_text())['members']),
 raw_report_sha256=sha(R/'run_report.json'),cpu_manifest_sha256=sha(CPU_MANIFEST),
 closed_loop=closed_loop,update_source_legend={'0':'prediction/no measurement','1':'camera identity update','2':'LiDAR range-only update'},
 overall_status='GPU_QUALIFIED_PENDING_REVIEW')
(E/'gpu_metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
for filename,data in [('gpu_by_range.csv',metrics['by_range']),('gpu_by_elevation.csv',metrics['by_elevation']),('gpu_distance_elevation.csv',a['strata'])]:
 with (E/filename).open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(data[0]),lineterminator='\n');w.writeheader();w.writerows(data)
print(json.dumps(dict(new_correct_range=metrics['new_correct_range'],covariance=metrics['covariance'],manifest_sha256=metrics['manifest_sha256']),indent=2))
