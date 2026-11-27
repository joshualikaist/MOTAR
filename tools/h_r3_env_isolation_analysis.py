#!/usr/bin/env python3
"""Exact N128 reset-isolation comparisons; no simulator imports or GPU execution."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

PROCESSES = {'legacy_normal': ('legacy_audit', False, True), 'legacy_forced': ('legacy_audit', True, True),
             'fixed_normal': ('selected', False, True), 'fixed_forced': ('selected', True, True),
             'fixed_noeval': ('selected', False, False)}
DERIVED = ('robot_euler_angles','robot_vehicle_orientation','robot_vehicle_linvel','robot_body_linvel','robot_body_angvel')
NUMERIC = ('rgb','depth','lidar','actor_obs','root_state','robot_pos','robot_quat','robot_linvel','robot_angvel',
           'target_pos','target_vel','tracker_state','tracker_cov','tracker_age','tracker_active',
           'robot_history','target_history','obstacle_history','camera','fused','action') + tuple('derived_'+n for n in DERIVED)
HASHES = ('rgb_sha','depth_sha','lidar_sha','state_sha','vertices_sha')
LAYOUT = ('vertex_env_index','vertex_actor_index','vertex_ranges')
REQUIRED = NUMERIC + HASHES + LAYOUT + ('vertices','mesh_ids','reset','natural_reset')


def _same(a,b):
    return a.dtype == b.dtype and a.shape == b.shape and np.ascontiguousarray(a).tobytes() == np.ascontiguousarray(b).tobytes()


def _hash(a):
    return np.frombuffer(hashlib.sha256(np.ascontiguousarray(a).tobytes()).digest(), dtype=np.uint8)


def _metric(a,b):
    if a.shape != b.shape or a.dtype != b.dtype:
        return {'shape_or_dtype_mismatch': True, 'left_shape': list(a.shape), 'right_shape':list(b.shape)}
    changed = np.any(a.view(np.uint8).reshape(a.shape[0], -1) != b.view(np.uint8).reshape(b.shape[0], -1), axis=1) if a.ndim else np.array([not _same(a,b)])
    x,y = a.astype(np.float64), b.astype(np.float64)
    return {'changed_rows':int(changed.sum()), 'max_abs_delta':float(np.max(np.abs(x-y))) if x.size else 0.0}


def _load(path):
    with np.load(path,allow_pickle=False) as f:
        result={k:f[k] for k in f.files}
    absent=set(REQUIRED)-set(result)
    if absent: raise ValueError(str(path)+': missing '+','.join(sorted(absent)))
    return result


def _validate_frame(f,n):
    for k in NUMERIC + ('reset','natural_reset'):
        if f[k].ndim < 1 or f[k].shape[0] != n: raise ValueError(k+': invalid environment axis')
        if f[k].dtype.kind in 'fc' and not np.isfinite(f[k]).all(): raise ValueError(k+': nonfinite numeric telemetry')
    if f['actor_obs'].shape != (n,898) or f['root_state'].ndim !=3 or f['root_state'].shape[2]!=13:
        raise ValueError('actor898/root13 schema mismatch')
    for k in HASHES:
        if f[k].shape != (n,32) or f[k].dtype!=np.uint8: raise ValueError(k+': SHA256 schema mismatch')
    for k in ('rgb','depth','lidar'):
        for i in range(n):
            if not _same(f[k+'_sha'][i],_hash(f[k][i])): raise ValueError(k+': numeric/SHA disagreement env'+str(i))
    v,ve,va,r=f['vertices'],f['vertex_env_index'],f['vertex_actor_index'],f['vertex_ranges']
    if v.ndim!=2 or v.shape[1]!=3 or ve.shape!=(len(v),) or va.shape!=(len(v),) or r.shape!=(n,2):
        raise ValueError('geometry layout dimensions')
    if not np.isfinite(v).all() or f['mesh_ids'].shape!=(n,) or len(np.unique(f['mesh_ids']))!=n:
        raise ValueError('invalid vertices/mesh correspondence')
    a=f['root_state'].shape[1]
    if va.dtype.kind not in 'iu' or ve.dtype.kind not in 'iu' or r.dtype.kind not in 'iu': raise ValueError('noninteger geometry mapping')
    if np.any(va<0) or np.any(va>=n*a) or not np.array_equal(va//a,ve): raise ValueError('vertex actor/environment map mismatch')
    cursor=0
    for i,(lo,hi) in enumerate(r):
        if lo!=cursor or hi<=lo or hi>len(v) or not np.all(ve[lo:hi]==i): raise ValueError('mesh slice/environment mismatch')
        if not _same(f['vertices_sha'][i],_hash(v[lo:hi])): raise ValueError('vertex numeric/SHA disagreement')
        cursor=int(hi)
    if cursor!=len(v): raise ValueError('unmapped vertices')


def compare(run,left,right,excluded=()):
    mask=np.ones(128,bool);mask[list(excluded)]=False
    output={'left':left,'right':right,'excluded_envs':list(excluded),'compared_envs':int(mask.sum()),'equal':True,'first_mismatch':{},'frames':41}
    for step in range(-1,40):
        name='frame_init.npz' if step<0 else 'frame%04d.npz'%step
        a,b=(_load(Path(run)/p/'frames'/name) for p in (left,right))
        for f in (a,b):_validate_frame(f,128)
        for k in LAYOUT:
            if not _same(a[k],b[k]):raise ValueError('cross-process geometry layout differs '+k)
        for k in NUMERIC+HASHES+('reset','natural_reset'):
            x,y=a[k][mask],b[k][mask]
            if not _same(x,y):
                output['equal']=False
                if k not in output['first_mismatch']:
                    output['first_mismatch'][k]={'step':step,**_metric(x,y)}
        vm=np.isin(a['vertex_env_index'],np.flatnonzero(mask))
        if not _same(a['vertices'][vm],b['vertices'][vm]):
            output['equal']=False
            if 'vertices' not in output['first_mismatch']:output['first_mismatch']['vertices']={'step':step,**_metric(a['vertices'][vm],b['vertices'][vm])}
    output['modalities']={k:{'equal':k not in output['first_mismatch'],'first_mismatch':output['first_mismatch'].get(k)} for k in ('rgb','depth','lidar')}
    return output


def _events(run,p):
    d=Path(run)/p
    fm=json.loads((d/'frame_meta.json').read_text());ge=json.loads((d/'geometry_events.json').read_text());rw=json.loads((d/'root_write_events.json').read_text())
    if isinstance(fm,dict):fm=fm.get('frames',fm.get('events'))
    if isinstance(ge,dict):ge=ge.get('events')
    if isinstance(rw,dict):rw=rw.get('events')
    if not all(isinstance(x,list) for x in (fm,ge,rw)):raise ValueError(p+': malformed event lists')
    if [int(x['step']) for x in fm]!=list(range(-1,40)):raise ValueError(p+': frame chronology')
    for row in fm:
        s=int(row['step']);f=_load(d/'frames'/('frame_init.npz' if s<0 else 'frame%04d.npz'%s))
        for key,field in (('reset_ids','reset'),('natural_reset_ids','natural_reset')):
            if sorted(row[key]) != np.flatnonzero(f[field]).tolist():raise ValueError(p+': frame/reset event disagreement '+key)
        if bool(row['forced_injected']) != bool(PROCESSES[p][1] and s==2):raise ValueError(p+': forced injection chronology')
        frame_event_sets={}
        for prefix,ev in (('warp_event',ge),('root_write_event',rw)):
            lo,hi=int(row[prefix+'_start']),int(row[prefix+'_end'])
            if not 0<=lo<=hi<=len(ev) or any(int(x['step'])!=s for x in ev[lo:hi]):raise ValueError(p+': event range timestamp mismatch')
            frame_event_sets[prefix]=ev[lo:hi]
        expected=set(row['reset_ids'])
        geometry_ids={i for e in frame_event_sets['warp_event'] for i in e['env_ids']}
        flushed_ids={i for e in frame_event_sets['root_write_event'] if e['kind']=='indexed_flush' for i in e['env_ids']}
        if geometry_ids!=expected or flushed_ids!=expected:raise ValueError(p+': reset/geometry/root-write selected IDs disagree')
    geometry_frames={}
    for event in ge:
        ids=event['env_ids']
        if sorted(event['refit_env_ids'])!=sorted(ids) or len(set(ids))!=len(ids):raise ValueError(p+': refit selection mismatch')
        step=int(event['step'])
        if step not in geometry_frames:
            raw=_load(d/'frames'/('frame_init.npz' if step<0 else 'frame%04d.npz'%step))
            geometry_frames[step]={k:raw[k] for k in ('mesh_ids','vertex_ranges')}
        f=geometry_frames[step]
        if not np.array_equal(np.asarray(event['refit_mesh_ids'],np.uint64),f['mesh_ids'][ids]):raise ValueError(p+': mesh/refit identity mismatch')
        expected=f['vertex_ranges'][ids] if PROCESSES[p][0]=='selected' else f['vertex_ranges']
        if not np.array_equal(np.asarray(event['write_vertex_ranges']),expected):raise ValueError(p+': unexpected vertex write range')
        if PROCESSES[p][0]=='selected' and any(x['changed_vertices'] for x in event['vertices'] if x['env'] not in ids):raise ValueError(p+': unselected geometry mutation')
    for e in rw:
        if e['kind'] not in ('queued','indexed_flush','full'):raise ValueError(p+': root setter kind')
        if int(e['step'])>=0 and e['kind']=='full':raise ValueError(p+': unexpected runtime full root setter')
    return fm,ge,rw


def analyse(run):
    run=Path(run);out={'schema':'h-r3.environment-isolation-analysis.v1','scope':'CPU_ORACLE_AND_ACTUAL_N128_GPU_MECHANICS','verdict':'INCOMPLETE','failures':[],'comparisons':{}}
    try:
        metas={p:json.loads((run/p/'process_meta.json').read_text()) for p in PROCESSES}
        for p,(mode,forced,evaluator) in PROCESSES.items():
            m=metas[p]
            expected={'id':p,'mode':mode,'force_reset':forced,'evaluator':evaluator,'seed':4602,'num_envs':128,'steps':40,'actor_obs_dim':898}
            if any(m.get(k)!=v for k,v in expected.items()):raise ValueError(p+': preregistered process contract mismatch')
            if not m.get('rms_digest_start') or m['rms_digest_start']!=m.get('rms_digest_end'):raise ValueError(p+': frozen RMS changed')
        if len({m['source_commit'] for m in metas.values()})!=1 or len({m['rms_digest_start'] for m in metas.values()})!=1:
            raise ValueError('process source/RMS mismatch')
        out['source_commit']=metas['fixed_normal']['source_commit']
        cpu_path=run/'cpu_report.json';cpu=json.loads(cpu_path.read_text())
        out['cpu_oracle']={'status':cpu.get('status'),'report_sha256':hashlib.sha256(cpu_path.read_bytes()).hexdigest(),'tests':cpu.get('tests')}
        if cpu.get('status')!='PASS':out['failures'].append('CPU_ORACLE_NOT_PASS')
        ev={p:_events(run,p) for p in PROCESSES}
        chronology={}
        for p in PROCESSES:
            f=_load(run/p/'frames'/'frame0002.npz');row=ev[p][0][3]
            ok=not np.any(f['natural_reset']) and np.flatnonzero(f['reset']).tolist()==([127] if PROCESSES[p][1] else []) and bool(row['forced_injected'])==PROCESSES[p][1]
            chronology[p]={'PASS':bool(ok),'reset_ids':np.flatnonzero(f['reset']).tolist(),'natural_reset_ids':np.flatnonzero(f['natural_reset']).tolist(),'forced_injected':bool(row['forced_injected'])}
        out['reset_chronology']=chronology
        if not all(x['PASS'] for x in chronology.values()):out['failures'].append('INVALID_STEP2_RESET_CONTRAST')
        for key,a,b,excluded in (('legacy_negative','legacy_normal','legacy_forced',(127,)),('fixed_isolation','fixed_normal','fixed_forced',(127,)),('evaluator_noninterference','fixed_normal','fixed_noeval',())):
            out['comparisons'][key]=compare(run,a,b,excluded)
        for k in ('fixed_isolation','evaluator_noninterference'):
            if not out['comparisons'][k]['equal']:out['failures'].append(k.upper()+'_FAIL')
        a,b=(_load(run/p/'frames'/'frame0002.npz') for p in ('legacy_forced','fixed_forced'))
        own={k:_same(a[k][127],b[k][127]) for k in NUMERIC+HASHES}
        va=a['vertex_env_index']==127;own['vertices']=_same(a['vertices'][va],b['vertices'][va])
        out['selected_reset_parity']={'PASS':all(own.values()),'fields':own}
        if not all(own.values()):out['failures'].append('SELECTED_RESET_SEMANTICS_FAIL')
        # Initial scene equality across every process is required, including selected row127.
        initial=[_load(run/p/'frames'/'frame_init.npz') for p in PROCESSES]
        initial_equal=all(_same(initial[0][k],f[k]) for f in initial[1:] for k in NUMERIC+HASHES+LAYOUT+('vertices',))
        out['initial_all_process_equal']=initial_equal
        if not initial_equal:out['failures'].append('INITIAL_STATE_FAIL')
        old=out['comparisons']['legacy_negative'];first=old['first_mismatch']
        vertex_at2=first.get('vertices',{}).get('step')==2
        sensor_at2=any(first.get(k,{}).get('step')==2 for k in ('rgb','depth','lidar'))
        out['causality']={'legacy_negative_reproduced':not old['equal'],'unselected_vertices_onset_step2':vertex_at2,'sensor_onset_step2':sensor_at2,
                         'classification':'CONTROLLED_CURRENT_DIAGNOSTIC_CAUSE_SUPPORTED_NOT_RETROSPECTIVE_PROOF' if vertex_at2 and sensor_at2 and not out['failures'] else 'INCOMPLETE_CAUSE_LOCALIZATION',
                         'historical_P02_numerical_cause_proven':False}
        if out['failures']:out['verdict']='NO_GO'
        elif old['equal'] or not vertex_at2 or not sensor_at2:out['verdict']='INCOMPLETE'
        else:out['verdict']='GO_MECHANICS_QUALIFIED'
        out['n128_verified']=True
    except (OSError,ValueError,KeyError,IndexError,TypeError) as error:
        out['failures'].append(type(error).__name__+': '+str(error));out['verdict']='INCOMPLETE';out['n128_verified']=False
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True);p.add_argument('--out',required=True);a=p.parse_args()
    result=analyse(a.run)
    with Path(a.out).open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(result['verdict'])
    return 0 if result['verdict']=='GO_MECHANICS_QUALIFIED' else 2

if __name__=='__main__':raise SystemExit(main())
