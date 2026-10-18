#!/usr/bin/env python3
"""Bounded B0 reset-isolation diagnostic; all geometry/GT records are evaluator-only."""
from __future__ import annotations
import argparse,hashlib,json,os,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for p in (str(ROOT),str(ROOT/'tools')):
    if p not in sys.path:sys.path.insert(0,p)
import isaacgym  # must precede torch
import numpy as np
import torch
import eq1d_fixture_launcher as eq
from motar_eval import eq1d_telemetry as tel
from motar_eval.h_r3_p02_identity import attach_camera_identity
from h_r3_p02_rng import FixedNRandomness,DERIVED_KINEMATICS
from h_r3_env_isolation import attach_warp_isolation


def host(x):return x.detach().cpu().numpy().copy()
def sha_rows(x):
    return np.stack([np.frombuffer(hashlib.sha256(np.ascontiguousarray(row).tobytes()).digest(),np.uint8) for row in x])
def joint_rows(values):
    n=values[0].shape[0];out=[]
    for i in range(n):
        h=hashlib.sha256()
        for a in values:h.update(np.ascontiguousarray(a[i]).tobytes())
        out.append(np.frombuffer(h.digest(),np.uint8))
    return np.stack(out)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--process',required=True);ap.add_argument('--out',required=True);ap.add_argument('--source-commit',required=True)
    args=ap.parse_args();plan=json.loads((ROOT/'docs/plans/h_r3_env_isolation_v1.json').read_text());spec=next(x for x in plan['processes'] if x['id']==args.process);cfg=plan['run'];n=cfg['num_envs']
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False);frames=out/'frames';frames.mkdir()
    ckpt=ROOT/'aerial_gym/rl_training/rl_games'/eq.PLAN['frozen']['policy_f_rel_to_rl_games'];assert tel.sha256_file(ckpt)==plan['policy_f_sha256']
    player,_=eq.build_player(n,ckpt);task=eq.find_task(player);od=task.obs_dict;dev=task.device
    assert task.task_obs['observations'].shape[1]==898
    rms=eq.find_rms(player.model);rms0=tel.rms_digest(rms);assert rms0==eq.PLAN['frozen']['rms_digest_sha256']
    rng=FixedNRandomness(task,cfg['seed']);geometry=attach_warp_isolation(task,mode=spec['mode'])
    evaluator=attach_camera_identity(task.perception,task.detector) if spec['evaluator'] else None
    # Observe every requested write without replacing its behavior.
    root_events=[];ige=task.sim_env.IGE_env;old_write=ige.write_to_sim;old_flush=rng._flush_selected_roots
    def write():
        ids=[] if rng._reset_env_id is None else [int(rng._reset_env_id)]
        root_events.append(dict(step=rng.logical_step,kind='full' if rng._reset_env_id is None else 'queued',env_ids=ids))
        return old_write()
    def flush():
        ids=sorted(rng._pending_root_envs)
        result=old_flush()
        if ids:root_events.append(dict(step=rng.logical_step,kind='indexed_flush',env_ids=ids))
        return result
    ige.write_to_sim=write;rng._flush_selected_roots=flush
    reset_ids=[];old_reset=task.reset_idx
    def reset(ids,*a,**kw):
        reset_ids.extend(torch.as_tensor(ids).detach().cpu().tolist());return old_reset(ids,*a,**kw)
    task.reset_idx=reset
    sensor={};old_rgbd=task.detector.render_raw_rgbd
    def rgbd(*a,**kw):
        result=old_rgbd(*a,**kw);sensor['rgb']=host(result[0]);sensor['depth']=host(result[1]);sensor['calls']=sensor.get('calls',0)+1;return result
    task.detector.render_raw_rgbd=rgbd
    lidar_calls=[];old_sensors=task.sim_env.render_sensors
    def sensors(*a,**kw):
        result=old_sensors(*a,**kw);lidar_calls.append((rng.logical_step,host(task._lidar_distance_m())));return result
    task.sim_env.render_sensors=sensors
    natural_ids=[];forced=False;old_post=task.sim_env.post_reward_calculation_step
    def post():
        nonlocal forced
        flags=task.sim_env.collision_tensor*int(task.sim_env.cfg.env.reset_on_collision)+task.sim_env.truncation_tensor
        natural_ids.extend(torch.nonzero(flags).flatten().detach().cpu().tolist())
        if spec['force_reset'] and rng.logical_step==cfg['forced_step']:
            if natural_ids:raise RuntimeError('registered force overlaps a natural reset')
            task.sim_env.truncation_tensor[cfg['forced_env']]=1;forced=True
        return old_post()
    task.sim_env.post_reward_calculation_step=post
    perception=task.perception;tracker=perception.tracker;robot=task.sim_env.robot_manager.robot
    fm=[];ge_start=rw_start=lidar_start=0;wall0=time.monotonic()
    def save(step,obs,action):
        nonlocal ge_start,rw_start,lidar_start
        g=geometry.snapshot();ot=obs['obs'] if isinstance(obs,dict) else obs
        data=dict(rgb=sensor['rgb'],depth=sensor['depth'],lidar=host(task._lidar_distance_m()),actor_obs=host(ot),
                  root_state=g['roots'].reshape(n,-1,13),vertices=g['vertices'],vertex_env_index=g['vertex_env_index'],
                  vertex_actor_index=g['vertex_actor_index'],vertex_ranges=g['vertex_ranges'],mesh_ids=g['mesh_ids'],
                  robot_pos=host(od['robot_position']),robot_quat=host(od['robot_orientation']),robot_linvel=host(od['robot_linvel']),robot_angvel=host(od['robot_angvel']),
                  target_pos=host(task.target_position),target_vel=host(task.target_vel_w),tracker_state=host(tracker.state),tracker_cov=host(tracker.cov),tracker_age=host(tracker.age),tracker_active=host(tracker.active),
                  robot_history=host(perception.robot_history),target_history=host(perception.target_history),obstacle_history=host(perception.obstacle_history),
                  camera=host(task._camera_visible_now),fused=host(task._visible_now),action=host(action),episode_id=np.asarray(rng.episodes,dtype=np.int64))
        for key in DERIVED_KINEMATICS:data['derived_'+key]=host(getattr(robot,key))
        for key in ('rgb','depth','lidar'):data[key+'_sha']=sha_rows(data[key])
        state_keys=['actor_obs','root_state','robot_pos','robot_quat','robot_linvel','robot_angvel','target_pos','target_vel','tracker_state','tracker_cov','tracker_age','tracker_active','robot_history','target_history','obstacle_history','camera','fused','episode_id']+['derived_'+k for k in DERIVED_KINEMATICS]
        data['state_sha']=joint_rows([data[k]for k in state_keys]);data['vertices_sha']=np.stack([sha_rows(data['vertices'][lo:hi][None])[0]for lo,hi in g['vertex_ranges']])
        reset_array=np.zeros(n,bool);reset_array[sorted(set(reset_ids))]=True;data['reset']=reset_array
        natural=np.zeros(n,bool);natural[sorted(set(natural_ids))]=True;data['natural_reset']=natural
        for a in data.values():
            if a.dtype.kind in 'fc' and not np.isfinite(a).all():raise RuntimeError('nonfinite diagnostic numeric snapshot')
        for event in geometry.events[ge_start:]:event['step']=step
        fm.append(dict(step=step,reset_ids=sorted(set(reset_ids)),natural_reset_ids=sorted(set(natural_ids)),forced_injected=forced,
                       warp_event_start=ge_start,warp_event_end=len(geometry.events),root_write_event_start=rw_start,root_write_event_end=len(root_events),
                       lidar_render_start=lidar_start,lidar_render_end=len(lidar_calls),sensor_calls=sensor['calls']))
        name='frame_init.npz' if step<0 else 'frame%04d.npz'%step
        np.savez_compressed(frames/name,**data)
        ge_start,rw_start,lidar_start=len(geometry.events),len(root_events),len(lidar_calls)
    obs=player.env_reset(player.env);player.get_batch_size(obs,1);save(-1,obs,torch.zeros(n,4,device=dev))
    for k in range(cfg['steps']):
        reset_ids.clear();natural_ids.clear();forced=False;rgb_calls=sensor['calls'];id_calls=evaluator.call_count if evaluator else 0
        with rng.action():action=player.get_action(obs,is_deterministic=True)
        with rng.step(k):obs,rew,done,info=player.env_step(player.env,action)
        if sensor['calls']<=rgb_calls or (evaluator is not None and evaluator.call_count<=id_calls):raise RuntimeError('stale sensor/evaluator packet')
        save(k,obs,action)
        if k%10==0:print('[isolation] %s step%d'% (args.process,k),flush=True)
    np.savez_compressed(out/'intermediate_lidar.npz',step=np.asarray([x[0]for x in lidar_calls]),value=np.stack([x[1]for x in lidar_calls]))
    meta=dict(id=args.process,mode=spec['mode'],force_reset=spec['force_reset'],evaluator=spec['evaluator'],seed=cfg['seed'],num_envs=n,steps=cfg['steps'],actor_obs_dim=898,
              rms_digest_start=rms0,rms_digest_end=tel.rms_digest(rms),source_commit=args.source_commit,model_training=bool(player.model.training),step_dt=float(task.step_dt),origins=eq.check_origins(),
              policy_f_sha256=tel.sha256_file(ckpt),rng=rng.metadata(),loop_wall_s=time.monotonic()-wall0,evaluator_calls=evaluator.call_count if evaluator else 0,
              schema='motar.environment-isolation-process.v1',online_GT='UNCHANGED; geometry/root/provenance snapshots evaluator-only')
    if meta['rms_digest_end']!=rms0 or meta['model_training'] or meta['step_dt']!=.1:raise RuntimeError('frozen actor/RMS/timestep contract')
    for name,value in [('process_meta.json',meta),('frame_meta.json',fm),('geometry_events.json',geometry.events),('root_write_events.json',root_events)]:
        with (out/name).open('x')as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
    # Completion binds every data product, never a mutable or pre-existing output.
    products=[]
    for p in sorted(out.rglob('*')):
        if p.is_file():products.append(dict(file=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=tel.sha256_file(p)))
    tel.write_json(out/'process_done.json',dict(process=args.process,files=products,actor_obs_dim=898,nan_fields=[],wall_s=time.monotonic()-wall0))
    return 0

if __name__=='__main__':raise SystemExit(main())
