#!/usr/bin/env python3
"""P02-only bounded simulator process; production and P01 source remain untouched."""
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
from motar_eval import e_l0,h_r3_p02_arms as arms
from motar_eval.h_r3_p02_identity import attach_camera_identity
from motar_eval import eq1d_telemetry as tel
from h_r3_p02_rng import FixedNRandomness
PLAN=json.loads((ROOT/'docs/plans/h_r3_p02.json').read_text())


def host(x):return x.detach().cpu().numpy().copy()


def digests(*tensors):
    values=[np.ascontiguousarray(host(x)) for x in tensors]
    n=values[0].shape[0];out=np.empty((n,32),np.uint8)
    for i in range(n):
        h=hashlib.sha256()
        for x in values:h.update(np.ascontiguousarray(x[i]).tobytes())
        out[i]=np.frombuffer(h.digest(),np.uint8)
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--arm',choices=['B0','B1','B2','B3','B4','B5'],required=True)
    ap.add_argument('--num-envs',type=int,required=True);ap.add_argument('--steps',type=int,required=True)
    ap.add_argument('--out',required=True);ap.add_argument('--process-id',required=True);ap.add_argument('--seed',type=int,required=True)
    ap.add_argument('--force-reset',action='store_true');ap.add_argument('--no-identity',action='store_true')
    args=ap.parse_args();out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    n,steps,W=args.num_envs,args.steps,PLAN['intervention']['window_steps']
    ckpt=ROOT/'aerial_gym/rl_training/rl_games'/eq.PLAN['frozen']['policy_f_rel_to_rl_games']
    if tel.sha256_file(ckpt)!=PLAN['policy_f_sha256']:raise RuntimeError('Policy F SHA mismatch')
    player,_=eq.build_player(n,ckpt);task=eq.find_task(player);od=task.obs_dict;dev=task.device
    if task.task_obs['observations'].shape[1]!=898:raise RuntimeError('actor898 mismatch')
    rms=eq.find_rms(player.model);rms_start=tel.rms_digest(rms)
    if rms_start!=eq.PLAN['frozen']['rms_digest_sha256']:raise RuntimeError('RMS mismatch')
    rng=FixedNRandomness(task,args.seed)
    identity=None if args.no_identity else attach_camera_identity(task.perception,task.detector)
    sensor={};original_render=task.detector.render_raw_rgbd
    def render(*a,**kw):
        result=original_render(*a,**kw)
        sensor['digest']=digests(result[0],result[1],task._lidar_distance_m())
        sensor['calls']=sensor.get('calls',0)+1
        return result
    task.detector.render_raw_rgbd=render
    reset_ids=[];original_reset=task.reset_idx
    def reset(env_ids,*a,**kw):
        reset_ids.extend(torch.as_tensor(env_ids).detach().cpu().tolist())
        return original_reset(env_ids,*a,**kw)
    task.reset_idx=reset
    physical={};old_post=task.sim_env.post_reward_calculation_step
    def post():
        for key,value in [('position',od['robot_position']),('velocity',od['robot_linvel']),('target',task.target_position),
                          ('crashed',task.crashed_now.bool()),('captured',task.captured_now.bool()),('timeout',task.truncations.bool() & ~task.crashed_now.bool() & ~task.captured_now.bool())]:
            physical[key]=value.detach().clone()
        return old_post()
    task.sim_env.post_reward_calculation_step=post
    command={};old_transform=task.transform_action_to_command
    def transform(action):
        result=old_transform(action);command['value']=result.detach().clone()
        command['governor']={k:v.detach().clone() for k,v in task._speed_governor_last.items() if isinstance(v,torch.Tensor)}
        return result
    task.transform_action_to_command=transform
    contact=e_l0.ContactChain(task.sim_env.physics_step_callback,lambda:od['robot_contact_force_tensor'],
                             float(task.sim_env.cfg.env.collision_force_threshold),n,dev)
    task.sim_env.set_physics_step_callback(contact)
    p=task.perception;tracker=p.tracker
    def state_digest(obs):
        ot=obs['obs'] if isinstance(obs,dict) else obs
        return digests(ot,od['robot_position'],od['robot_orientation'],od['robot_linvel'],od['robot_angvel'],
                       task.target_position,task.target_vel_w,tracker.state,tracker.cov,tracker.age,tracker.active,p.robot_history,p.target_history,p.obstacle_history,task._camera_visible_now,task._visible_now)
    obs=player.env_reset(player.env);player.get_batch_size(obs,1)
    initial_state=state_digest(obs);initial_sensor=sensor['digest'].copy()
    active=torch.zeros(n,dtype=torch.bool,device=dev);acquired=active.clone();prev_cam=active.clone();first=torch.ones_like(active)
    t0=torch.full((n,),-1,dtype=torch.long,device=dev);sign=arms.sham_signs(args.seed,list(range(n)),device=dev)
    elev=torch.deg2rad(torch.tensor([20.,10.,0.,-10.],device=dev));angles=p._lidar_angles
    maxv=float(task.task_config.max_velocity);yawmax=float(task.task_config.yaw_rate_max)
    trace={};compute=[];reasons=['']*n
    def record(k,v):trace.setdefault(k,[]).append(host(v) if isinstance(v,torch.Tensor) else np.asarray(v).copy())
    wall0=time.monotonic()
    for k in range(steps):
        reset_ids.clear();sensor_calls=sensor['calls'];identity_calls=identity.call_count if identity else None
        with rng.action():action=player.get_action(obs,is_deterministic=True)
        before=od['robot_position'].detach().clone();record('before_pos',before)
        lidar=task._lidar_distance_m();record('raw_nearest_m',lidar.reshape(n,-1).amin(1))
        torch.cuda.synchronize();c0=time.perf_counter()
        actions,details=arms.evaluate_actions(od['robot_position'],od['robot_vehicle_orientation'],tracker.state[:,:3],lidar,angles,elev,
                                             sham_sign=sign,max_velocity=maxv,yaw_rate_max=yawmax,lidar_far_m=float(p.lidar_max_range))
        nominal_error=torch.stack([(details['arms'][a]['nominal_xy'].norm(dim=1)-details['arms']['B3']['nominal_xy'].norm(dim=1)).abs() for a in ('B4','B5')],1)
        if bool((nominal_error>1e-6).any()):raise RuntimeError('same-state nominal budget mismatch')
        for key in ('nominal_xy','post_point_xy','post_cap_xy'):
            policy=action[:,:2]*maxv
            selected=details['arms'][args.arm][key] if args.arm!='B0' else policy
            record(key,torch.where(active[:,None],selected,policy))
            for shadow_arm in ('B2','B3','B4','B5'):record('shadow_'+shadow_arm+'_'+key,details['arms'][shadow_arm][key])
        if args.arm!='B0':action=torch.where(active[:,None],actions[args.arm].to(action.dtype),action)
        torch.cuda.synchronize();compute.append((time.perf_counter()-c0)*1e3)
        record('action',action);record('intervening',active);record('shadow_offset',details['chosen_offset_m'])
        record('shadow_clear',details['clear_found']);record('budget_error',nominal_error);record('sham_sign',sign)
        contact.begin_action()
        with rng.step(k):obs,rew,done,info=player.env_step(player.env,action)
        if args.force_reset and k==40:rng.force_reset([15])
        if sensor['calls']<=sensor_calls:raise RuntimeError('stale RGB-D packet')
        if identity is not None:
            if identity.call_count<=identity_calls:raise RuntimeError('stale identity packet')
            for key,val in identity.latest.items():record('id_'+key,val)
        cam=task._camera_visible_now.clone().bool();fused=task._visible_now.clone().bool()
        if identity is not None and not torch.equal(identity.latest['camera_delivered'],cam):raise RuntimeError('identity timestamp/camera mismatch')
        sil=e_l0.silhouette_from_task(task)
        resets=torch.zeros(n,dtype=torch.bool,device=dev)
        if reset_ids:resets[torch.as_tensor(sorted(set(reset_ids)),device=dev)]=True
        for key,val in [('camera',cam),('lidar_only',fused&~cam),('el0_cls',sil['cls']),('el0_visible_px',sil['visible_px']),
                        ('el0_alone_px',sil['alone_px']),('el0_in_frustum',sil['centre_in_frustum']),('reset',resets),
                        ('crashed',physical['crashed']),('captured',physical['captured']),('timeout',physical['timeout']),
                        ('actual_pos',physical['position']),('actual_vel',physical['velocity']),('target_pos',physical['target']),
                        ('robot_pos',od['robot_position']),('belief_pos',tracker.state[:,:3]),('command',command['value']),
                        ('contact_peak_n',contact.peak),('contact_substeps',contact.count),('rng_probe',rng.probe())]:record(key,val)
        record('digest',state_digest(obs));record('sensor_digest',sensor['digest'])
        for key in ('clearance_m','requested_speed_mps','executed_speed_mps','scale'):
            if key not in command['governor']:raise RuntimeError('missing governor telemetry '+key)
            record('governor_'+key,command['governor'][key])
        ended=active&(resets|physical['crashed']|physical['captured']|((k+1-t0)>=W))
        for i in torch.nonzero(ended).flatten().tolist():reasons[i]='terminal' if bool(resets[i] or physical['crashed'][i] or physical['captured'][i]) else 'window'
        active&=~ended
        trigger=first&acquired&prev_cam&~cam&(t0<0)&~resets&(k+1+W<=steps)
        t0=torch.where(trigger,torch.full_like(t0,k+1),t0);active|=trigger
        acquired|=cam;first&=~resets;prev_cam=cam
    torch.cuda.synchronize();wall=time.monotonic()-wall0
    arrays={k:np.stack(v) for k,v in trace.items()};arrays.update(t0=host(t0),controller_ms=np.asarray(compute),initial_digest=initial_state,initial_sensor_digest=initial_sensor)
    bad=eq.nan_report({k:v for k,v in arrays.items() if v.dtype.kind=='f'})
    meta={'schema':'motar.h-r3-p02-process.v1','arm':args.arm,'seed':args.seed,'num_envs':n,'steps':steps,'window_steps':W,
          'rms_digest_start':rms_start,'rms_digest_end':tel.rms_digest(rms),'model_training_flag':bool(player.model.training),
          'origins':eq.check_origins(),'vision_mode':bool(task.vision_mode),'step_dt':float(task.step_dt),'max_velocity':maxv,
          'yaw_rate_max':yawmax,'speed_governor':task.speed_governor_cfg.mode,'end_reason':reasons,'loop_wall_s':wall,
          'nan_fields':bad,'el0_classes':list(e_l0.CLASSES),'rng':rng.metadata(),'identity_enabled':identity is not None,
          'forced_reset':bool(args.force_reset),'sensor_lineage':os.environ.get('NAVRL_SENSOR_LINEAGE','off'),
          'env':{k:os.environ[k] for k in sorted(os.environ) if k.startswith(('NAVRL_SEED','NAVRL_NUM_BARS','NAVRL_TARGET','NAVRL_KROC2027_INITIAL','NAVRL_SENSOR_LINEAGE','NAVRL_PERCEPTION_PERTURB'))}}
    files=[tel.write_npz(out/'h_r3_p02_trace.npz',arrays),tel.write_json(out/'h_r3_p02_meta.json',meta)]
    tel.write_json(out/'process_done.json',{'process':args.process_id,'files':files,'nan_fields':bad,'wall_s':wall,'actor_obs_dim':898,'peak_allocated_bytes':int(torch.cuda.max_memory_allocated())})
    return 3 if bad else 0

if __name__=='__main__':sys.exit(main())
