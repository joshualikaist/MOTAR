#!/usr/bin/env python3
"""P03 preparation qualification only: frozen B0, prefix-budget arms, selected geometry."""
from __future__ import annotations
import argparse,hashlib,json,os,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for path in (str(ROOT),str(ROOT/'tools')):
    if path not in sys.path:sys.path.insert(0,path)
import isaacgym  # before torch
import numpy as np
import torch
import eq1d_fixture_launcher as eq
from motar_eval import e_l0,h_r3_p03_arms as arms,eq1d_telemetry as tel
from motar_eval.h_r3_p02_identity import attach_camera_identity
from h_r3_p02_rng import FixedNRandomness,DERIVED_KINEMATICS
from h_r3_p03_isolation import attach
PLAN=json.loads((ROOT/'docs/plans/h_r3_p03_qualification_v1.json').read_text())


def host(x):return x.detach().cpu().numpy().copy()
def hashes(*values):
    arrays=[host(x)if isinstance(x,torch.Tensor)else np.ascontiguousarray(x)for x in values]
    output=[]
    for i in range(len(arrays[0])):
        h=hashlib.sha256()
        for x in arrays:h.update(np.ascontiguousarray(x[i]).tobytes())
        output.append(np.frombuffer(h.digest(),np.uint8))
    return np.stack(output)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--process',required=True);ap.add_argument('--out',required=True);ap.add_argument('--source-commit',required=True);args=ap.parse_args()
    spec=next(p for p in PLAN['processes']if p['id']==args.process);n=PLAN['run']['num_envs'];steps=spec['steps'];seed=PLAN['run']['seed'];W=20
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False);snapshots=out/'snapshots';snapshots.mkdir()
    ckpt=ROOT/'aerial_gym/rl_training/rl_games'/eq.PLAN['frozen']['policy_f_rel_to_rl_games'];assert tel.sha256_file(ckpt)==PLAN['policy_f_sha256']
    player,_=eq.build_player(n,ckpt);task=eq.find_task(player);od=task.obs_dict;dev=task.device
    assert task.task_obs['observations'].shape[1]==898
    rms=eq.find_rms(player.model);rms0=tel.rms_digest(rms);assert rms0==eq.PLAN['frozen']['rms_digest_sha256']
    rng=FixedNRandomness(task,seed);geometry=attach(task,mode=spec['isolation_mode']);identity=attach_camera_identity(task.perception,task.detector)if spec['evaluator']else None
    p=task.perception;tracker=p.tracker;robot=task.sim_env.robot_manager.robot;ige=task.sim_env.IGE_env
    reset_ids=[];natural_ids=[];root_events=[];frame_events=[];sensor={};physical={};command={};forced=False
    old_reset=task.reset_idx
    def reset(ids,*a,**kw):reset_ids.extend(torch.as_tensor(ids).detach().cpu().tolist());return old_reset(ids,*a,**kw)
    task.reset_idx=reset
    old_write=ige.write_to_sim;old_flush=rng._flush_selected_roots
    def write():
        ids=[]if rng._reset_env_id is None else[int(rng._reset_env_id)]
        root_events.append(dict(step=rng.logical_step,kind='full'if rng._reset_env_id is None else'queued',env_ids=ids));return old_write()
    def flush():
        ids=sorted(rng._pending_root_envs);value=old_flush()
        if ids:root_events.append(dict(step=rng.logical_step,kind='indexed_flush',env_ids=ids))
        return value
    ige.write_to_sim=write;rng._flush_selected_roots=flush
    old_rgbd=task.detector.render_raw_rgbd
    def render(*a,**kw):
        value=old_rgbd(*a,**kw);sensor.update(rgb=host(value[0]),depth=host(value[1]),calls=sensor.get('calls',0)+1);return value
    task.detector.render_raw_rgbd=render
    old_post=task.sim_env.post_reward_calculation_step
    def post():
        nonlocal forced
        for key,value in [('position',od['robot_position']),('velocity',od['robot_linvel']),('angular_velocity',od['robot_angvel']),('target',task.target_position),('crashed',task.crashed_now.bool()),('captured',task.captured_now.bool()),('timeout',task.truncations.bool()&~task.crashed_now.bool()&~task.captured_now.bool())]:physical[key]=value.detach().clone()
        flags=task.sim_env.collision_tensor*int(task.sim_env.cfg.env.reset_on_collision)+task.sim_env.truncation_tensor
        natural_ids.extend(torch.nonzero(flags).flatten().detach().cpu().tolist())
        if spec['force_reset']and rng.logical_step==PLAN['force']['step']:
            if natural_ids:raise RuntimeError('forced reset overlaps natural reset')
            task.sim_env.truncation_tensor[PLAN['force']['env']]=1;forced=True
        return old_post()
    task.sim_env.post_reward_calculation_step=post
    old_transform=task.transform_action_to_command
    def transform(action):
        value=old_transform(action);command['value']=value.detach().clone();command['governor']={k:v.detach().clone()for k,v in task._speed_governor_last.items()if isinstance(v,torch.Tensor)};return value
    task.transform_action_to_command=transform
    contact=e_l0.ContactChain(task.sim_env.physics_step_callback,lambda:od['robot_contact_force_tensor'],float(task.sim_env.cfg.env.collision_force_threshold),n,dev);task.sim_env.set_physics_step_callback(contact)
    def packet(obs):
        actor=obs['obs']if isinstance(obs,dict)else obs;lidar=host(task._lidar_distance_m());roots=host(ige.unfolded_vec_root_tensor).reshape(n,-1,13)
        derived=[getattr(robot,key)for key in DERIVED_KINEMATICS]
        state=hashes(actor,roots,od['robot_position'],od['robot_orientation'],od['robot_linvel'],od['robot_angvel'],task.target_position,task.target_vel_w,tracker.state,tracker.cov,tracker.age,tracker.active,p.robot_history,p.target_history,p.obstacle_history,task._camera_visible_now,task._visible_now,np.asarray(rng.episodes,dtype=np.int64),*derived)
        return dict(digest=state,sensor_digest=hashes(sensor['rgb'],sensor['depth'],lidar),rgb_digest=hashes(sensor['rgb']),depth_digest=hashes(sensor['depth']),lidar_digest=hashes(lidar),roots_digest=hashes(roots),tracker_digest=hashes(tracker.state,tracker.cov,tracker.age,tracker.active),actor_history_digest=hashes(actor,p.robot_history,p.target_history,p.obstacle_history)),lidar
    windows=set(PLAN['diagnostic_windows']['short'if steps==16 else'long']);snap_count=0;ge_start=rw_start=0
    def snapshot(k,obs,action,lidar):
        nonlocal snap_count
        if k>=0 and k not in windows:return
        g=geometry.snapshot();actor=obs['obs']if isinstance(obs,dict)else obs
        data=dict(rgb=sensor['rgb'],depth=sensor['depth'],lidar=lidar,actor_obs=host(actor),root_state=g['roots'].reshape(n,-1,13),vertices=g['vertices'],vertex_env_index=g['vertex_env_index'],vertex_actor_index=g['vertex_actor_index'],vertex_ranges=g['vertex_ranges'],mesh_ids=g['mesh_ids'],action=host(action),tracker_state=host(tracker.state),tracker_cov=host(tracker.cov),tracker_age=host(tracker.age),tracker_active=host(tracker.active),robot_history=host(p.robot_history),target_history=host(p.target_history),obstacle_history=host(p.obstacle_history))
        for key in ('rgb','depth','lidar'):data[key+'_sha']=hashes(data[key])
        np.savez_compressed(snapshots/('initial.npz'if k<0 else'step%04d.npz'%k),**data);snap_count+=1
    def chronology(k):
        nonlocal ge_start,rw_start
        for event in geometry.events[ge_start:]:event['step']=k
        row=dict(step=k,reset_ids=sorted(set(reset_ids)),natural_reset_ids=sorted(set(natural_ids)),forced_injected=forced,warp_event_start=ge_start,warp_event_end=len(geometry.events),root_write_event_start=rw_start,root_write_event_end=len(root_events))
        frame_events.append(row);ge_start,rw_start=len(geometry.events),len(root_events)
    obs=player.env_reset(player.env);player.get_batch_size(obs,1);geometry.check_integrity();initial,lidar_cpu=packet(obs);snapshot(-1,obs,torch.zeros(n,4,device=dev),lidar_cpu);chronology(-1)
    initial_camera=host(task._camera_visible_now).astype(bool);initial_identity=host(identity.latest['camera_identity_correct']).astype(bool)if identity is not None else np.zeros(n,bool)
    active=torch.zeros(n,dtype=torch.bool,device=dev);acquired=task._camera_visible_now.clone().bool();prev_cam=acquired.clone();first=torch.ones_like(active)
    t0=torch.full((n,),-1,dtype=torch.long,device=dev);event_sha=np.zeros((n,32),np.uint8)
    schedule=dict(lateral_abs_mps=torch.zeros(n,W,device=dev),reference_norm_mps=torch.full((n,W),float(arms.PARAMS['pursuit_speed_mps']),device=dev),window_steps=W,dt_s=.1)
    elev=torch.deg2rad(torch.tensor([20.,10.,0.,-10.],device=dev));angles=p._lidar_angles;maxv=float(task.task_config.max_velocity);yawmax=float(task.task_config.yaw_rate_max)
    trace={};compute=[];guard_ms=[];reasons=['']*n;wall0=time.monotonic();digest_s=0.
    def record(key,value):trace.setdefault(key,[]).append(host(value)if isinstance(value,torch.Tensor)else np.asarray(value).copy())
    for k in range(steps):
        reset_ids.clear();natural_ids.clear();forced=False;calls=sensor['calls'];idcalls=identity.call_count if identity else 0
        with rng.action():action=player.get_action(obs,is_deterministic=True)
        # Proposal is computed BEFORE the step that may lose the camera measurement.
        inputs=(od['robot_position'],od['robot_vehicle_orientation'],tracker.state[:,:3],task._lidar_distance_m(),angles,elev)
        proposal=arms.freeze_schedule(*inputs,max_velocity=maxv,yaw_rate_max=yawmax,lidar_far_m=float(p.lidar_max_range))
        proposal_sha=hashes(*inputs[:4],proposal['lateral_abs_mps'],proposal['reference_norm_mps'])
        decision=(k-t0).clamp(min=0,max=W-1);episode=torch.as_tensor(rng.episodes,device=dev,dtype=torch.long)
        sign=arms.sham_signs(seed,list(range(n)),episode,torch.zeros_like(episode),decision,device=dev)
        before=od['robot_position'].detach().clone();record('before_pos',before);record('pre_vel',od['robot_linvel']);record('raw_nearest_m',inputs[3].reshape(n,-1).amin(1))
        torch.cuda.synchronize();begin=time.perf_counter()
        actions,details=arms.evaluate_actions(*inputs,schedule=schedule,decision_index=decision,sham_sign=sign,max_velocity=maxv,yaw_rate_max=yawmax,lidar_far_m=float(p.lidar_max_range),frozen_action=action)
        norms=torch.stack([details['arms'][a]['nominal_xy'].norm(dim=1)for a in('B3','B4','B5')],1);expected=details['reference_norm_mps'];valid=details['direction_valid']
        error=torch.where(valid[:,None],(norms-expected[:,None]).abs(),norms.abs())
        if bool((error>1e-6).any()):raise RuntimeError('nominal prefixbudget or direction-abstention violation')
        for key in ('nominal_xy','post_point_xy','post_cap_xy'):
            policy=action[:,:2]*maxv;selected=details['arms'][spec['hr3_arm']][key]if spec['hr3_arm']!='B0'else policy
            record(key,torch.where(active[:,None],selected,policy))
            for a in('B2','B3','B4','B5'):record('shadow_'+a+'_'+key,details['arms'][a][key])
        if spec['hr3_arm']!='B0':action=torch.where(active[:,None],actions[spec['hr3_arm']].to(action.dtype),action)
        torch.cuda.synchronize();compute.append((time.perf_counter()-begin)*1000)
        for key,value in [('action',action),('intervening',active),('shadow_offset',details['chosen_offset_m']),('shadow_clear',details['clear_found']),('budget_error',error),('sham_sign',sign),('selected_side',details['selected_side']),('sham_side',sign),('candidate_scores',details['candidate_scores']),('direction_valid',valid),('schedule_lateral_mps',details['lateral_abs_mps']),('reference_norm_mps',expected),('budget_frozen',t0>=0),('decision_index',decision),('episode_id',episode),('event_id',torch.zeros_like(episode))]:record(key,value)
        contact.begin_action()
        with rng.step(k):obs,reward,done,info=player.env_step(player.env,action)
        if sensor['calls']<=calls or(identity is not None and identity.call_count<=idcalls):raise RuntimeError('stale sensor/identity packet')
        if identity is not None:
            for key,value in identity.latest.items():record('id_'+key,value)
        cam=task._camera_visible_now.clone().bool();fused=task._visible_now.clone().bool()
        if identity is not None and not torch.equal(identity.latest['camera_delivered'],cam):raise RuntimeError('identity timestamp mismatch')
        sil=e_l0.silhouette_from_task(task)if spec['evaluator']else dict(cls=torch.full((n,),-1,device=dev,dtype=torch.long),visible_px=torch.zeros(n,device=dev,dtype=torch.long),alone_px=torch.zeros(n,device=dev,dtype=torch.long),centre_in_frustum=torch.zeros(n,device=dev,dtype=torch.bool));resets=torch.zeros(n,dtype=torch.bool,device=dev);natural=resets.clone();injected=resets.clone()
        if reset_ids:resets[sorted(set(reset_ids))]=True
        if natural_ids:natural[sorted(set(natural_ids))]=True
        if forced:injected[PLAN['force']['env']]=True
        for key,value in [('camera',cam),('lidar_only',fused&~cam),('el0_cls',sil['cls']),('el0_visible_px',sil['visible_px']),('el0_alone_px',sil['alone_px']),('el0_in_frustum',sil['centre_in_frustum']),('reset',resets),('natural_reset',natural),('forced_injected',injected),('crashed',physical['crashed']),('captured',physical['captured']),('timeout',physical['timeout']),('actual_pos',physical['position']),('actual_vel',physical['velocity']),('actual_angvel',physical['angular_velocity']),('target_pos',physical['target']),('robot_pos',od['robot_position']),('belief_pos',tracker.state[:,:3]),('tracker_active',tracker.active),('tracker_age',tracker.age),('command',command['value']),('contact_peak_n',contact.peak),('contact_substeps',contact.count),('rng_probe',rng.probe())]:
            record(key,value)
        for key in ('clearance_m','requested_speed_mps','executed_speed_mps','scale'):
            if key not in command['governor']:raise RuntimeError('missing governor telemetry '+key)
            record('governor_'+key,command['governor'][key])
        gc=time.perf_counter();geometry.check_integrity();guard_ms.append((time.perf_counter()-gc)*1000)
        dg=time.perf_counter();current,lidar_cpu=packet(obs);digest_s+=time.perf_counter()-dg
        for key,value in current.items():record(key,value)
        snapshot(k,obs,action,lidar_cpu);chronology(k)
        ended=active&(resets|physical['crashed']|physical['captured']|((k+1-t0)>=W))
        for i in torch.nonzero(ended).flatten().tolist():reasons[i]='terminal'if bool(resets[i]or physical['crashed'][i]or physical['captured'][i])else'window'
        active&=~ended
        trigger=first&acquired&prev_cam&~cam&(t0<0)&~resets&(k+1+W<=steps)
        schedule['lateral_abs_mps'][trigger]=proposal['lateral_abs_mps'][trigger];schedule['reference_norm_mps'][trigger]=proposal['reference_norm_mps'][trigger]
        ids=torch.nonzero(trigger).flatten().detach().cpu().numpy();event_sha[ids]=proposal_sha[ids]
        t0=torch.where(trigger,torch.full_like(t0,k+1),t0);active|=trigger;acquired|=cam;first&=~resets;prev_cam=cam
        if k%100==0:print('[p03-qualification] '+args.process+' step'+str(k),flush=True)
    torch.cuda.synchronize();wall=time.monotonic()-wall0
    arrays={key:np.stack(value)for key,value in trace.items()};arrays.update(initial_camera=initial_camera,initial_identity_correct=initial_identity,t0=host(t0),event_schedule_lateral_mps=host(schedule['lateral_abs_mps']),event_reference_norm_mps=host(schedule['reference_norm_mps']),event_budget_digest=event_sha,controller_ms=np.asarray(compute),geometry_check_ms=np.asarray(guard_ms))
    for key,value in initial.items():arrays['initial_'+key]=value
    bad=eq.nan_report({key:value for key,value in arrays.items()if value.dtype.kind=='f'})
    meta=dict(schema='motar.hr3-p03-qualification.process.v1',id=args.process,arm=spec['hr3_arm'],seed=seed,num_envs=n,steps=steps,window_steps=W,source_commit=args.source_commit,isolation_mode=spec['isolation_mode'],force_reset=spec['force_reset'],forced_reset=spec['force_reset'],evaluator=spec['evaluator'],identity_enabled=spec['evaluator'],actor_obs_dim=898,rms_digest_start=rms0,rms_digest_end=tel.rms_digest(rms),model_training_flag=bool(player.model.training),origins=eq.check_origins(),vision_mode=bool(task.vision_mode),step_dt=float(task.step_dt),max_velocity=maxv,yaw_rate_max=yawmax,speed_governor=task.speed_governor_cfg.mode,end_reason=reasons,loop_wall_s=wall,sensor_digest_wall_s=digest_s,nan_fields=bad,el0_classes=list(e_l0.CLASSES),rng=rng.metadata(),sensor_lineage=os.environ.get('NAVRL_SENSOR_LINEAGE','off'),policy_f_sha256=tel.sha256_file(ckpt),snapshot_count=snap_count,geometry_integrity=geometry.check_integrity(),transform_binding=getattr(geometry._transform,'qualified_name',getattr(geometry._transform,'__name__',type(geometry._transform).__name__)),online_GT='UNCHANGED; all GT/provenance telem evaluationonly',env={k:os.environ[k]for k in sorted(os.environ)if k.startswith(('NAVRL_SEED','NAVRL_NUM_BARS','NAVRL_TARGET','NAVRL_KROC2027_INITIAL','NAVRL_SENSOR_LINEAGE','NAVRL_PERCEPTION_PERTURB'))})
    if meta['rms_digest_end']!=rms0 or meta['model_training_flag']or meta['step_dt']!=.1:raise RuntimeError('FrozenF/RMS/timing contract')
    tel.write_npz(out/'h_r3_p03_trace.npz',arrays)
    for name,value in [('h_r3_p03_meta.json',meta),('frame_meta.json',frame_events),('geometry_events.json',geometry.events),('root_write_events.json',root_events)]:tel.write_json(out/name,value)
    products=[dict(file=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=tel.sha256_file(p))for p in sorted(out.rglob('*'))if p.is_file()]
    tel.write_json(out/'process_done.json',dict(process=args.process,files=products,nan_fields=bad,actor_obs_dim=898,wall_s=wall))
    return 3 if bad else 0

if __name__=='__main__':raise SystemExit(main())
