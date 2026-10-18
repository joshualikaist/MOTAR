#!/usr/bin/env python3
"""P03 qualification only: exact isolation/prefix gates and descriptive arm metrics.

Frozen P02 scoring helpers are reused for their terminal-first spatial support rule;
P02 raw data, estimators and outcome numbers are never loaded or pooled.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import h_r3_p02_analysis as p02

ARMS = p02.ARMS
CLASSES = p02.CLASSES
PRIMARY = p02.PRIMARY
CONTRASTS = p02.CONTRASTS
DIGEST_KEYS = ('digest', 'sensor_digest', 'rgb_digest', 'depth_digest', 'lidar_digest',
               'roots_digest', 'tracker_digest', 'actor_history_digest')
EXACT_FIELDS = DIGEST_KEYS + ('action', 'rng_probe', 'before_pos', 'actual_pos', 'actual_vel',
                              'target_pos', 'camera', 'reset', 'command', 'pre_vel', 'actual_angvel',
                              'tracker_active', 'tracker_age', 'natural_reset', 'forced_injected')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def exact(a, b):
    return p02.exact(np.asarray(a), np.asarray(b))


def compare_traces(left, right, excluded_envs=(), fields=EXACT_FIELDS):
    """Compare all registered frames; excluded reset target is explicit, never inferred."""
    n = left['camera'].shape[1]
    keep = np.ones(n, bool)
    keep[list(excluded_envs)] = False
    result = {'equal': True, 'compared_envs': int(keep.sum()), 'excluded_envs': list(excluded_envs),
              'steps': int(left['camera'].shape[0]), 'fields': {}, 'initial_equal': True}
    for key in fields:
        a, b = left[key], right[key]
        if a.shape != b.shape or a.dtype != b.dtype:
            result['fields'][key] = {'equal': False, 'shape_or_dtype_mismatch': True}
            result['equal'] = False
            continue
        a, b = a[:, keep], b[:, keep]
        ok = exact(a, b)
        detail = {'equal': ok}
        if not ok:
            unequal = np.any(np.ascontiguousarray(a).view(np.uint8).reshape(*a.shape[:2], -1) !=
                             np.ascontiguousarray(b).view(np.uint8).reshape(*b.shape[:2], -1), axis=2)
            step, row = np.argwhere(unequal)[0]
            detail.update(first_step=int(step), first_env=int(np.flatnonzero(keep)[row]),
                          changed_envs=int(unequal.any(axis=0).sum()),
                          changed_step_env_rows=int(unequal.sum()))
            if a.dtype.kind in 'fiu':
                detail['max_abs_delta'] = float(np.max(np.abs(a.astype(np.float64)-b.astype(np.float64))))
        result['fields'][key] = detail
        result['equal'] &= ok
    for key in DIGEST_KEYS:
        initial = 'initial_' + key
        ok = exact(left[initial][keep], right[initial][keep])
        result['initial_equal'] &= ok
        result['equal'] &= ok
    result['modalities'] = {name: result['fields'][name+'_digest'] for name in ('rgb', 'depth', 'lidar')}
    return result


def matched_prefix(arms, required_arms=ARMS):
    """Any arm/environment failure invalidates the whole cohort; no salvage subset."""
    ref = arms['B0']
    failures, events = [], []
    n, steps = ref['camera'].shape[1], ref['camera'].shape[0]
    initial_ok = {}
    for arm in required_arms:
        tr = arms[arm]
        initial_ok[arm] = all(exact(tr['initial_'+k], ref['initial_'+k]) for k in DIGEST_KEYS)
        if not initial_ok[arm]:
            failures.append(arm + ': initial state/sensor/history differs')
    for env in range(n):
        t0 = int(ref['t0'][env])
        same_t0 = all(int(arms[a]['t0'][env]) == t0 for a in required_arms)
        stop = steps if t0 < 0 else t0
        mismatches = []
        for arm in required_arms:
            for key in EXACT_FIELDS:
                if not exact(arms[arm][key][:stop, env], ref[key][:stop, env]):
                    mismatches.append({'arm': arm, 'field': key})
        if not same_t0 or mismatches:
            failures.append('env%d: %s' % (env, 't0 differs' if not same_t0 else 'prebranch/no-trigger state/sensor/history differs'))
        events.append({'env': env, 't0': t0, 'prefix_matched': same_t0 and not mismatches,
                       'mismatches': mismatches})
    return {'equal': not failures, 'initial_by_arm': initial_ok, 'failures': failures, 'environments': events}


def _mean(values):
    return float(np.mean(values)) if len(values) else None


def event_outcome(trace, env, t0, window=20, dt=.1, stall_mps=.1):
    out = p02.metric_outcome(trace, env, t0, window, dt)
    end = min(t0 + window, len(trace['camera']))
    terminal = p02.terminal(trace, env, t0, end)
    rows = np.arange(t0, terminal + 1 if terminal is not None else end)
    valid_rows = np.arange(t0, terminal if terminal is not None else end)
    for label, use in (('fixed_window', rows), ('control_only', rows[trace['intervening'][rows, env]])):
        velocity = trace['actual_vel'][use, env]
        pre = trace['pre_vel'][use, env]
        acceleration = (velocity - pre) / dt
        actual = np.linalg.norm(velocity[:, :2], axis=1)
        command = np.linalg.norm(trace['command'][use, env, :2], axis=1)
        nominal = np.linalg.norm(trace['nominal_xy'][use, env], axis=1)
        post = np.linalg.norm(trace['post_cap_xy'][use, env], axis=1)
        denom = nominal > 0
        out[label].update(
            command_speed_mps_mean=_mean(command), post_safety_speed_mps_mean=_mean(post),
            actual_speed_mps_mean=_mean(actual),
            acceleration_norm_mps2_mean=_mean(np.linalg.norm(acceleration, axis=1)),
            acceleration_sq_integral_m2_s3=float(np.square(acceleration).sum() * dt),
            stall_duration_s=float((actual < stall_mps).sum() * dt),
            nominal_to_command_attenuation_mean=_mean((1-command[denom]/nominal[denom])) if denom.any() else None,
            post_safety_speed_sq_integral_m2_s=float(np.square(post).sum() * dt),
            min_lidar_clearance_m=float(np.min(trace['raw_nearest_m'][use, env])) if len(use) else None,
            mechanical_control_effort='NOT_RECORDED; acceleration and squared speed are kinematic proxies')
    active = trace.get('tracker_active')
    age = trace.get('tracker_age')
    if active is not None and age is not None:
        hit = valid_rows[active[valid_rows, env].astype(bool) & (age[valid_rows, env] == 0)]
        out['tracker_fresh_measurement_within_window'] = bool(len(hit))
        out['tracker_fresh_measurement_delay_s'] = float((hit[0]-t0+1)*dt) if len(hit) else None
        out['tracker_fresh_measurement_definition'] = 'active AND age==0; not persistent semantic identity'
    else:
        out['tracker_fresh_measurement_within_window'] = 'NOT_RECORDED'
    return out


def summarize(events, valid):
    result = {'n': len(events), 'integrity_valid': bool(valid),
              'interpretation': 'QUALIFICATION_DESCRIPTIVE_ONLY' if valid else 'INVALID_INTEGRITY_FAILURE',
              'arms': {}, 'contrasts': {}}
    for arm in ARMS:
        outcomes = [e['results'][arm] for e in events]
        result['arms'][arm] = {'n': len(outcomes), 'reobserved': sum(e['reobserved'] for e in outcomes),
                               'rate': _mean([e['reobserved'] for e in outcomes]),
                               'mean_reobservation_s_among_successes': _mean([e['time_to_reobs_s'] for e in outcomes if e['reobserved']]),
                               'collision': sum(e['collision_in_window'] for e in outcomes),
                               'capture': sum(e['capture_in_window'] for e in outcomes),
                               'timeout': sum(e['timeout_in_window'] for e in outcomes),
                               'p01_literal_reobserved': sum(e['variants']['p01_literal']['reobserved'] for e in outcomes),
                               'p01_terminal_corrected_reobserved': sum(e['variants']['p01_terminal_corrected']['reobserved'] for e in outcomes)}
        for scope in ('fixed_window', 'control_only'):
            keys = sorted({key for e in outcomes for key, value in e[scope].items()
                           if isinstance(value, (int, float)) or value is None})
            result['arms'][arm][scope] = {key: _mean([e[scope][key] for e in outcomes if e[scope].get(key) is not None]) for key in keys}
    for a, b in CONTRASTS:
        av = np.asarray([e['results'][a]['reobserved'] for e in events], dtype=np.int64)
        bv = np.asarray([e['results'][b]['reobserved'] for e in events], dtype=np.int64)
        result['contrasts'][a+'-'+b] = {'rate_diff': _mean(av-bv), 'n10': int(((av==1)&(bv==0)).sum()),
                                       'n01': int(((av==0)&(bv==1)).sum()),
                                       'uncertainty': 'NOT_ESTIMATED; one qualification seed is not a population cluster sample'}
    return result


def effort_balance(events, margin=.1):
    """Registered descriptive balance; both-zero pairs count as exact equality."""
    out = {}
    primary_metrics = ('translation_xy_m', 'mean_actual_speed_mps', 'actual_xy_sq_integral')
    supplementary = ('command_xy_sq_integral', 'acceleration_sq_integral_m2_s3')
    for a, b in CONTRASTS:
        metrics = {}
        for key in primary_metrics + supplementary:
            differences, zeros = [], 0
            for e in events:
                x, y = (e['results'][arm]['control_only'][key] for arm in (a,b))
                denom = (abs(x)+abs(y))/2
                if denom == 0:
                    zeros += 1
                    differences.append(0.)
                else:
                    differences.append(abs(x-y)/denom)
            value = _mean(differences)
            metrics[key] = {'mean_absolute_symmetric_relative_difference': value,
                            'both_zero_pairs_retained_as_equal': zeros,
                            'within_registered_margin': value <= margin if value is not None else None,
                            'role': 'PRIMARY_BALANCE_DIAGNOSTIC' if key in primary_metrics else 'SUPPLEMENTARY_PROXY'}
        exposure_equal = all(e['results'][a]['control_only']['steps'] == e['results'][b]['control_only']['steps'] for e in events)
        balance_pass = bool(events) and exposure_equal and all(metrics[k]['within_registered_margin'] is True for k in primary_metrics)
        out[a+'-'+b] = {'metrics': metrics, 'all_exposures_equal': exposure_equal,
                        'descriptive_balance': 'WITHIN_REGISTERED_MARGIN' if balance_pass else 'IMBALANCED_OR_INSUFFICIENT',
                        'pure_viewpoint_attribution': 'NOT_ESTABLISHED' if balance_pass else 'INCONCLUSIVE',
                        'warning': 'Realized-effort checks are post-treatment descriptive diagnostics; passing does not identify a controlled direct effect.'}
    return out


QUALIFICATION = {
    'short_normal': ('B0', 16, False, True, 'runtime'),
    'short_forced': ('B0', 16, True, True, 'runtime'),
    'long_B0': ('B0', 600, False, True, 'runtime'),
    'long_forced': ('B0', 600, True, True, 'runtime'),
    'long_noeval': ('B0', 600, False, False, 'runtime'),
    'long_audit': ('B0', 600, False, True, 'audit'),
    **{'long_'+a: (a, 600, False, True, 'runtime') for a in ARMS[1:]},
}



def compare_snapshots(left,right,excluded_envs=(), traces=None):
    """Full numeric audit windows supplement, rather than replace, whole-horizon SHA."""
    left,right=Path(left),Path(right)
    names=sorted(p.name for p in (left/'snapshots').glob('*.npz'))
    other=sorted(p.name for p in (right/'snapshots').glob('*.npz'))
    if not names or names!=other:
        raise ValueError('registered numeric snapshot set missing/differs')
    if traces is not None:
        steps=len(traces[0]['camera'])
        windows=([0,1,2,3,4,14,15] if steps==16 else [0,1,2,3,4,38,39,40,41,42,598,599] if steps==600 else None)
        if windows is None or names!=sorted(['initial.npz']+['step%04d.npz'%v for v in windows]):
            raise ValueError('numeric snapshot set differs from preregistered windows')
    result={'equal':True,'frames':len(names),'mismatches':[]}
    for name in names:
        with np.load(left/'snapshots'/name,allow_pickle=False) as f:
            a={k:f[k] for k in f.files}
        with np.load(right/'snapshots'/name,allow_pickle=False) as f:
            b={k:f[k] for k in f.files}
        n=a['root_state'].shape[0]
        keep=np.ones(n,bool);keep[list(excluded_envs)]=False
        for key in ('vertex_env_index','vertex_actor_index','vertex_ranges'):
            if not exact(a[key],b[key]):
                raise ValueError('snapshot geometry layout differs '+key)
        for key in ('rgb','depth','lidar','actor_obs','root_state'):
            if key not in a or key not in b or a[key].shape[0]!=n or b[key].shape[0]!=n:
                raise ValueError('numeric snapshot missing environment axis '+key)
            for f in (a,b):
                if not np.isfinite(f[key]).all():
                    raise ValueError('nonfinite numeric snapshot '+key)
            if traces is not None and key in ('rgb','depth','lidar'):
                index=-1 if name=='initial.npz' else int(name[4:-4])
                for numeric,tr in zip((a,b),traces):
                    archived=tr['initial_'+key+'_digest'] if index<0 else tr[key+'_digest'][index]
                    actual=np.stack([np.frombuffer(hashlib.sha256(np.ascontiguousarray(v).tobytes()).digest(),np.uint8) for v in numeric[key]])
                    if not exact(actual,archived):
                        raise ValueError(name+': numeric/trace sensor SHA differs '+key)
            if not exact(a[key][keep],b[key][keep]):
                result['equal']=False
                result['mismatches'].append({'frame':name,'field':key})
        vm=np.isin(a['vertex_env_index'],np.flatnonzero(keep))
        if not exact(a['vertices'][vm],b['vertices'][vm]):
            result['equal']=False
            result['mismatches'].append({'frame':name,'field':'vertices'})
    return result


def validate_chronology(path,tr,meta):
    """Independently bind every reset to selected vertex writes/refits/root flushes."""
    path=Path(path);steps,n=tr['camera'].shape
    frames=json.loads((path/'frame_meta.json').read_text())
    geometry=json.loads((path/'geometry_events.json').read_text())
    roots=json.loads((path/'root_write_events.json').read_text())
    if [r['step'] for r in frames]!=list(range(-1,steps)):
        raise ValueError(path.name+': missing/nonmonotonic frame chronology')
    guard=meta['geometry_integrity']
    if guard.get('status')!='PASS' or guard.get('bitwise_geometry') is not True or guard.get('immutable_topology') is not True or guard.get('checks',0)<steps+1:
        raise ValueError(path.name+': exact geometry guard not established for whole horizon')
    if guard.get('reset_event_count')!=len(geometry):
        raise ValueError(path.name+': geometry event count differs')
    with np.load(path/'snapshots'/'initial.npz',allow_pickle=False) as f:
        ranges=f['vertex_ranges'];mesh_ids=f['mesh_ids']
    cursors={'warp_event':0,'root_write_event':0}
    for row in frames:
        step=int(row['step']);expected=set(row['reset_ids'])
        if len(row['reset_ids'])!=len(expected) or any(i<0 or i>=n for i in expected):
            raise ValueError(path.name+': invalid reset environment IDs')
        if step>=0:
            if sorted(expected)!=np.flatnonzero(tr['reset'][step]).tolist() or sorted(row['natural_reset_ids'])!=np.flatnonzero(tr['natural_reset'][step]).tolist():
                raise ValueError(path.name+': trace/reset chronology differs')
            if bool(row['forced_injected']) != bool(tr['forced_injected'][step].any()):
                raise ValueError(path.name+': trace/injection chronology differs')
        selected={}
        for prefix,events in (('warp_event',geometry),('root_write_event',roots)):
            lo,hi=row[prefix+'_start'],row[prefix+'_end']
            if lo!=cursors[prefix] or not 0<=lo<=hi<=len(events) or any(e['step']!=step for e in events[lo:hi]):
                raise ValueError(path.name+': event chronology range differs '+prefix)
            cursors[prefix]=hi;selected[prefix]=events[lo:hi]
        ge_ids={i for e in selected['warp_event'] for i in e['env_ids']}
        rw_ids={i for e in selected['root_write_event'] if e['kind']=='indexed_flush' for i in e['env_ids']}
        if ge_ids!=expected or rw_ids!=expected:
            raise ValueError(path.name+': reset/geometry/indexed root writes differ')
    if cursors['warp_event']!=len(geometry) or cursors['root_write_event']!=len(roots):
        raise ValueError(path.name+': unbound reset events')
    for event in geometry:
        ids=event['env_ids']
        if len(set(ids))!=len(ids) or event['refit_env_ids']!=ids or event.get('mode')!='selected':
            raise ValueError(path.name+': geometry/refit selection differs')
        if not np.array_equal(np.asarray(event['write_vertex_ranges']),ranges[ids]) or not np.array_equal(np.asarray(event['refit_mesh_ids'],np.uint64),mesh_ids[ids]):
            raise ValueError(path.name+': geometry write/refit mapping differs')
        if event.get('unselected_vertices_bitwise_preserved') is not True:
            raise ValueError(path.name+': reset lacks untouched-geometry proof')
        if any(r['changed_vertices'] for r in event.get('vertices',[]) if r['env'] not in ids):
            raise ValueError(path.name+': unexpected unselected geometry change')
    for event in roots:
        if event['kind'] not in ('queued','indexed_flush','full') or (event['kind']=='full' and event['step']>=0):
            raise ValueError(path.name+': unexpected full/unclassified root write')
    return {'status':'PASS','frames':len(frames),'geometry_events':len(geometry),'root_write_events':len(roots)}


def validate_sham_assignment(tr,seed):
    steps,n=tr['camera'].shape
    for key in ('episode_id','event_id','decision_index'):
        value=tr[key]
        if not np.isfinite(value).all() or np.any(value<0) or not np.array_equal(value,value.astype(np.int64)):
            raise ValueError('sham key identities are not nonnegative integers: '+key)
    if np.any(tr['decision_index']>=20) or np.any(tr['event_id']!=0):
        raise ValueError('registered first-event/decision range differs')
    expected=np.empty((steps,n),np.float32)
    for step in range(steps):
        for env in range(n):
            payload='HR3-P03|%d|%d|%d|%d|%d'%(seed,env,tr['episode_id'][step,env],tr['event_id'][step,env],tr['decision_index'][step,env])
            expected[step,env]=1. if hashlib.sha256(payload.encode('ascii')).digest()[0]&1 else -1.
    if not np.array_equal(tr['sham_sign'],expected) or not np.array_equal(tr['sham_side'],expected):
        raise ValueError('actual sham assignment differs from independent registered SHA oracle')
    return {'status':'PASS','decisions':steps*n,'seed':seed,'event_id':0}


def validate_control_contract(tr,arm):
    active=tr['intervening'].astype(bool)
    valid=tr['direction_valid'].astype(bool)
    reference=tr['reference_norm_mps']
    for shadow in ('B3','B4','B5'):
        nominal=np.linalg.norm(tr['shadow_'+shadow+'_nominal_xy'],axis=-1)
        error=np.where(valid,np.abs(nominal-reference),nominal)
        if np.max(error,initial=0)>1e-6:
            raise ValueError(arm+': independent archived nominal budget/direction error')
    selected='B2' if arm=='B1' else arm
    if selected!='B0':
        for key in ('nominal_xy','post_point_xy','post_cap_xy'):
            if not exact(tr[key][active],tr['shadow_'+selected+'_'+key][active]):
                raise ValueError(arm+': selected command stages differ from archived same-state control')
    rows=[{'env':i,'t0':int(t0)} for i,t0 in enumerate(tr['t0'])]
    failures=_budget_and_exposure({'B0':tr,arm:tr},{'environments':rows})
    if failures:
        raise ValueError('; '.join(failures))
    return {'status':'PASS','active_step_env_rows':int(active.sum())}

def load_process(path, identity=None):
    path = Path(path)
    with np.load(path/'h_r3_p03_trace.npz', allow_pickle=False) as data:
        tr = {k: data[k] for k in data.files}
    meta = json.loads((path/'h_r3_p03_meta.json').read_text())
    done = json.loads((path/'process_done.json').read_text())
    if identity is None:
        identity = bool(meta['identity_enabled'])
    steps, n = tr['camera'].shape
    for key in p02.CORE + p02.SHADOWS + DIGEST_KEYS + ('pre_vel', 'actual_angvel', 'tracker_active', 'tracker_age',
                                       'natural_reset', 'forced_injected', 'schedule_lateral_mps',
                                       'reference_norm_mps', 'budget_frozen', 'decision_index', 'episode_id', 'event_id', 'direction_valid', 'candidate_scores', 'selected_side', 'sham_side'):
        if key not in tr or tr[key].shape[:2] != (steps,n):
            raise ValueError(path.name+': missing/malformed '+key)
    for key in DIGEST_KEYS:
        for value, shape in ((tr[key], (steps,n,32)), (tr['initial_'+key], (n,32))):
            if value.shape != shape or value.dtype != np.uint8:
                raise ValueError(path.name+': invalid SHA256 '+key)
    for key, shape in (('t0',(n,)), ('event_schedule_lateral_mps',(n,20)),
                       ('event_reference_norm_mps',(n,20)), ('event_budget_digest',(n,32))):
        if tr[key].shape != shape:
            raise ValueError(path.name+': invalid event schedule '+key)
    if tr['event_budget_digest'].dtype != np.uint8:
        raise ValueError(path.name+': invalid event budget SHA')
    if tr.get('initial_camera',np.empty(0)).shape!=(n,):
        raise ValueError(path.name+': initial actual-camera measurement missing')
    if identity:
        if tr.get('initial_identity_correct',np.empty(0)).shape!=(n,):
            raise ValueError(path.name+': initial spatial identity measurement missing')
        for key in p02.IDENTITY:
            if key not in tr or tr[key].shape[:2] != (steps,n):
                raise ValueError(path.name+': missing identity telemetry '+key)
        if not exact(tr['camera'], tr['id_camera_delivered']):
            raise ValueError(path.name+': identity/camera timestamp differs')
    if tr['controller_ms'].shape != (steps,):
        raise ValueError(path.name+': controller timing dimension differs')
    if meta['steps'] != steps or meta['num_envs'] != n or meta['step_dt'] != .1 or meta['window_steps'] != 20:
        raise ValueError(path.name+': runtime dimension/timing contract differs')
    if meta.get('nan_fields') or done.get('nan_fields') or any(not np.isfinite(v).all() for v in tr.values() if v.dtype.kind == 'f'):
        raise ValueError(path.name+': nonfinite telemetry')
    if done['process'] != path.name or done['actor_obs_dim'] != 898 or meta['model_training_flag'] is not False or not meta['vision_mode']:
        raise ValueError(path.name+': actor/vision/process contract differs')
    if meta['rms_digest_start'] != meta['rms_digest_end']:
        raise ValueError(path.name+': RMS changed')
    if meta.get('sensor_lineage') != 'off':
        raise ValueError(path.name+': sensor lineage changed')
    listed = done.get('files',[])
    names = [item['file'] for item in listed]
    if len(names) != len(set(names)) or not {'h_r3_p03_trace.npz','h_r3_p03_meta.json'} <= set(names):
        raise ValueError(path.name+': completion file set malformed')
    inventory=[]
    for member in path.rglob('*'):
        if member.is_symlink():
            raise ValueError(path.name+': symlink in completion tree')
        if member.is_file() and member!=path/'process_done.json':
            inventory.append(member.relative_to(path).as_posix())
    if set(inventory)!=set(names):
        raise ValueError(path.name+': unlisted or missing completion member')
    for item in listed:
        rel = Path(item['file'])
        if rel.is_absolute() or '..' in rel.parts:
            raise ValueError(path.name+': unsafe completion path')
        f = path/rel
        if any((path/Path(*rel.parts[:i])).is_symlink() for i in range(1,len(rel.parts)+1)):
            raise ValueError(path.name+': symlink parent in completion path')
        try:
            f.resolve().relative_to(path.resolve())
        except ValueError:
            raise ValueError(path.name+': completion member escapes output')
        if f.is_symlink() or not f.is_file() or f.stat().st_size != item['bytes'] or sha(f) != item['sha256']:
            raise ValueError(path.name+': completion file binding differs '+str(rel))
    validate_chronology(path,tr,meta)
    validate_sham_assignment(tr,int(meta['seed']))
    validate_control_contract(tr,meta['arm'])
    return tr, meta


def compare_noninterference(run, left, right, excluded_envs=(), fields=None):
    a, am = load_process(Path(run)/left)
    b, bm = load_process(Path(run)/right)
    result = compare_traces(a,b,excluded_envs,fields=EXACT_FIELDS if fields is None else fields)
    result.update(left=left,right=right)
    result['snapshots'] = compare_snapshots(Path(run)/left,Path(run)/right,excluded_envs,traces=(a,b))
    result['equal'] &= result['snapshots']['equal']
    return result


def check_forced_reset(normal, forced, env=127, step=2):
    failures=[]
    if normal['camera'].shape[1] <= env or normal['camera'].shape[0] <= step:
        return {'equal':False,'failures':['forced-reset dimensions invalid']}
    if normal['natural_reset'][step].any() or normal['reset'][step].any():
        failures.append('baseline has natural reset at registered injection step')
    if forced['natural_reset'][step].any():
        failures.append('treatment natural reset overlaps injection step')
    if np.flatnonzero(forced['reset'][step]).tolist() != [env]:
        failures.append('forced reset did not execute exactly registered target/step')
    expected = np.zeros_like(forced['forced_injected'], dtype=bool)
    expected[step,env] = True
    if not exact(forced['forced_injected'],expected) or normal['forced_injected'].any():
        failures.append('forced injection chronology differs')
    return {'equal':not failures,'step':step,'env':env,'failures':failures,
            'baseline_reset_ids':np.flatnonzero(normal['reset'][step]).tolist(),
            'treatment_reset_ids':np.flatnonzero(forced['reset'][step]).tolist()}


def _budget_and_exposure(arms, prefix):
    failures=[]
    reference=arms['B0']
    for arm,tr in arms.items():
        for key in ('event_schedule_lateral_mps','event_reference_norm_mps','event_budget_digest'):
            if not exact(tr[key],reference[key]):
                failures.append(arm+': common pre-loss '+key+' differs')
        for row in prefix['environments']:
            env,t0=row['env'],row['t0']
            expected=np.zeros(len(tr['camera']),bool)
            if t0 >= 0:
                if t0 < 1 or t0+20>len(tr['camera']):
                    failures.append(arm+'/env%d: invalid trigger boundary'%env)
                    continue
                end=t0+20
                term=p02.terminal(tr,env,t0,end)
                expected[t0:term+1 if term is not None else end]=True
            if not exact(tr['intervening'][:,env],expected):
                failures.append(arm+'/env%d: intervention exposure differs'%env)
        active=tr['intervening'].astype(bool)
        if not tr['budget_frozen'][active].all():
            failures.append(arm+': active intervention has no frozen prefix budget')
        for env in range(len(tr['t0'])):
            rows=np.flatnonzero(active[:,env])
            if not len(rows):
                continue
            expected_decisions=rows-int(tr['t0'][env])
            if not exact(tr['decision_index'][rows,env].astype(np.int64),expected_decisions.astype(np.int64)):
                failures.append(arm+'/env%d: registered decision index differs'%env)
                continue
            for current,archive in (('schedule_lateral_mps','event_schedule_lateral_mps'),('reference_norm_mps','event_reference_norm_mps')):
                if not exact(tr[current][rows,env],tr[archive][env,expected_decisions]):
                    failures.append(arm+'/env%d: command budget differs from frozen archive'%env)
        if float(tr['budget_error'].max(initial=0))>1e-6:
            failures.append(arm+': nominal budget tolerance exceeded')
        if arm in ('B3','B4','B5'):
            active=tr['intervening'].astype(bool)
            actual=np.linalg.norm(tr['nominal_xy'],axis=-1)
            direction_valid=tr['direction_valid'].astype(bool)
            valid=active & direction_valid
            invalid=active & ~direction_valid
            if np.any(actual[invalid] != 0):
                failures.append(arm+': direction-invalid nominal command did not abstain')
            if np.max(np.abs(actual[valid]-tr['reference_norm_mps'][valid]),initial=0)>1e-6:
                failures.append(arm+': archived actual nominal norm differs from frozen budget')
    return failures


def analyse(run, plan=None):
    run=Path(run)
    result={'schema':'motar.h-r3-p03-qualification-analysis.v1', 'verdict':'INCOMPLETE',
            'scope':'LIMITED_QUALIFICATION_ONLY; NO_P03_PILOT_OR_CONFIRMATION',
            'hypothesis':'PROVISIONAL / CONFIRMATION HOLD','failures':[], 'comparisons':{},
            'population_inference':'NOT_SUPPORTED; one qualification seed cluster',
            'past_results_reused':False}
    try:
        data={name:load_process(run/name) for name in QUALIFICATION}
        for name,(arm,steps,forced,identity,mode) in QUALIFICATION.items():
            tr,meta=data[name]
            expected={'arm':arm,'steps':steps,'num_envs':128,'seed':4801,'isolation_mode':mode,
                      'forced_reset':forced,'identity_enabled':identity}
            if any(meta.get(k)!=v for k,v in expected.items()):
                result['failures'].append(name+': registered process contract differs')
        sources={meta['source_commit'] for _,meta in data.values()}
        rms={meta['rms_digest_start'] for _,meta in data.values()}
        if len(sources)!=1 or len(rms)!=1:
            result['failures'].append('source/RMS differs across qualification processes')
        result['source_commit']=sorted(sources)
        for key,left,right,excluded in (
            ('short_reset','short_normal','short_forced',(127,)),
            ('long_reset','long_B0','long_forced',(127,)),
            ('evaluator_noninterference','long_B0','long_noeval',()),
            ('audit_runtime_parity','long_B0','long_audit',())):
            comparison=compare_noninterference(run,left,right,excluded)
            result['comparisons'][key]=comparison
            if not comparison['equal']:
                result['failures'].append(key+': exact state/sensor/history comparison failed')
        result['forced_reset_chronology']={
            k:check_forced_reset(data[a][0],data[b][0]) for k,a,b in (
                ('short','short_normal','short_forced'),('long','long_B0','long_forced'))}
        for key,check in result['forced_reset_chronology'].items():
            result['failures'].extend(key+': '+f for f in check['failures'])
        arms={a:data['long_'+a][0] for a in ARMS}
        prefix=matched_prefix(arms)
        result['all_arm_common_prefix']=prefix
        result['failures'].extend(prefix['failures'])
        result['failures'].extend(_budget_and_exposure(arms,prefix))
        events=[]
        for row in prefix['environments']:
            env,t0=row['env'],row['t0']
            if t0<1 or t0+20>len(arms['B0']['camera']):
                continue
            class_id=int(arms['B0']['el0_cls'][t0-1,env])
            if not 0<=class_id<len(CLASSES):
                result['failures'].append('env%d: unknown E-L0 class'%env)
                continue
            end=t0+20
            terms=[p02.terminal(arms[a],env,t0,end) for a in ARMS]
            shared_end=min([end]+[v for v in terms if v is not None])
            shared={key:all(exact(arms[a][key][t0:shared_end,env],arms['B0'][key][t0:shared_end,env]) for a in ARMS)
                    for key in ('target_pos','rng_probe')}
            if not all(shared.values()):
                result['failures'].append('env%d: exogenous target/RNG differs before earliest terminal'%env)
            results={a:event_outcome(arms[a],env,t0) for a in ARMS}
            events.append({'env':env,'t0':t0,'seed':4801,'el0_class':CLASSES[class_id],
                           'primary':CLASSES[class_id] in PRIMARY,'prefix_matched':row['prefix_matched'],
                           'exogenous_shared':shared,
                           'confirmed_preloss_spatial_identity':bool(arms['B0']['id_camera_identity_correct'][t0-2,env]) if t0>=2 else bool(arms['B0']['initial_identity_correct'][env]),
                           'nonzero_lateral_budget':bool(np.any(arms['B0']['event_schedule_lateral_mps'][env]>0)),
                           'results':results})
        valid=not result['failures']
        primary=[e for e in events if e['primary']]
        result['counts']={'all_triggers':len(events),'primary_events':len(primary),
                          'matched_events':sum(e['prefix_matched'] for e in events),
                          'by_class':{c:sum(e['el0_class']==c for e in events) for c in CLASSES}}
        result['primary']=summarize(primary,valid)
        result['confirmed_preloss_primary_diagnostic_only']=summarize([e for e in primary if e['confirmed_preloss_spatial_identity']],valid)
        result['lateral_coverage']={'primary_events':len(primary),'nonzero_prefix_budget_events':sum(e['nonzero_lateral_budget'] for e in primary),
                                    'all_zero_budget_events_retained':sum(not e['nonzero_lateral_budget'] for e in primary)}
        result['primary_strata']={c:summarize([e for e in events if e['el0_class']==c],valid) for c in PRIMARY}
        result['separate_classes']={c:summarize([e for e in events if e['el0_class']==c],valid) for c in CLASSES if c not in PRIMARY}
        result['effort_balance_diagnostic']=effort_balance(primary)
        if not primary or not any(e['nonzero_lateral_budget'] for e in primary):
            for diagnostic in result['effort_balance_diagnostic'].values():
                diagnostic['pure_viewpoint_attribution']='INCONCLUSIVE'
                diagnostic['coverage']='INSUFFICIENT_NONZERO_LATERAL_EXPOSURE'
        result['events']=events
        result['verdict']='GO_QUALIFICATION_ONLY' if valid else 'NO_GO'
        result['interpretation']='Mechanics and control qualification only; no H-R3 efficacy or pure viewpoint conclusion.'
        if not events:
            result['verdict']='INCOMPLETE' if valid else 'NO_GO'
            result['event_coverage']='NOT_TESTED; no triggered event'
        else:
            result['event_coverage']='OBSERVED; zero-count classes remain NOT_TESTED'
    except (OSError,ValueError,KeyError,IndexError,TypeError) as error:
        result['failures'].append(type(error).__name__+': '+str(error))
        result['verdict']='INCOMPLETE'
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True)
    parser.add_argument('--out',required=True)
    args=parser.parse_args(argv)
    result=analyse(args.run)
    with Path(args.out).open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({k:result[k] for k in ('verdict','failures')},indent=2))
    return 0 if result['verdict']=='GO_QUALIFICATION_ONLY' else 2


if __name__=='__main__':
    raise SystemExit(main())
