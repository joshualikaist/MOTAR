#!/usr/bin/env python3
"""C-v2 GPU sensor fixtures and unchanged E-L0 closed-loop recorder."""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools'),str(ROOT/'tools/audits')]
import isaacgym  # noqa: E402,F401
import numpy as np
import torch
import eq1d_fixture_launcher as eq
import e_l0_launcher as el
from motar_eval import e_l0,eq1d_telemetry as tel
import s_l1c_v2_run as run


def host(x):return x.detach().cpu().numpy().copy()


def direction(b,e):
    b,e=math.radians(b),math.radians(e)
    return np.array([math.cos(e)*math.cos(b),math.cos(e)*math.sin(b),math.sin(e)])


def scenes(spec):
    if spec['kind']=='positive':
        values=list(itertools.product((1.16,1.5,2,2.28,2.3,3),(-5,0,5,10,15,20),(0,20,45,90)))
        values=values[48*spec['group']:48*(spec['group']+1)]
        return [dict(id='P%d'%i,r=r,el=el,b=b,target=(r*direction(b,el)).tolist(),bars=[])
                for i,(r,el,b) in enumerate(values,start=48*spec['group'])]
    result=[dict(id=s,replay=True,r=0.,el=0.,b=b,target=None,bars=[]) for s,b in
            (('N1',0),('N2',20),('N3',0),('N5',90),('N6',180),('N7',0),('N8',0))]
    for r,width in itertools.product((1.16,1.5,2,2.28,2.3,3),(.02,.6)):
        result.append(dict(id='BAR_%s_%s'%(r,width),r=r,el=5.,b=0.,target=None,
                           bars=[dict(center=[r+width/2,0,.5],half=[width/2,width/2,1.5])]))
    for r,gap in itertools.product((1.16,1.5,2,2.28,2.3,3),(.05,.3)):
        result.append(dict(id='FG_%s_%s'%(r,gap),r=r,el=5.,b=0.,target=(r*direction(0,5)).tolist(),
                           bars=[dict(center=[r*math.cos(math.radians(5))-.2-gap+.12,0,.5],half=[.12,.12,1.5])]))
    return result


def prime(perception,prediction,prior=.01):
    n=prediction.shape[0];dev=prediction.device;perception.reset_idx(torch.arange(n,device=dev))
    tr=perception.tracker;tr.state[:,:3]=prediction;tr.state[:,3:]=0
    tr.cov[:]=torch.eye(6,device=dev)*prior;tr.active[:]=True;tr.age[:]=1.


def associate(task,lidar,prediction,prior):
    p=task.perception;prime(p,prediction,prior)
    drone=task.obs_dict['robot_position'];q=task.obs_dict['robot_vehicle_orientation']
    before=p.tracker.cov[:,:3,:3].clone()
    valid,confidence,measured,bearing=p._associate_lidar_target(lidar,drone,q,torch.zeros(len(drone),dtype=torch.bool,device=drone.device))
    assoc=p.last_s_l1c_association
    provenance=e_l0.lidar_provenance(valid,bearing,measured,lidar.view(len(drone),4,72),
        task.obs_dict['segmentation_pixels'].view(len(drone),4,72),p._lidar_angles)
    return {'valid':host(valid),'measured':host(measured),'bearing':host(bearing),'prediction_rel':host(prediction-drone),
            'row':host(assoc['row']),'column':host(assoc['column']),'centre':host(assoc['centre_range']),
            'cov_before':host(before),'cov_after':host(p.tracker.cov[:,:3,:3]),'state':host(p.tracker.state),
            'el0_label':host(provenance['label']),'el0_consistent':host(provenance['consistent']),
            'identity_age':host(p._s_l1c_identity_age),'fused_age':host(p.tracker.age)}


def run_sensors(spec,out):
    ss=scenes(spec);n=len(ss);task=eq.build_task(n,4401);dev=task.device;p=task.perception
    origins=eq.check_origins();slots=eq.bar_slots(task);eq.park_all(task,slots)
    drone=torch.tensor([[20.,20.,1.]]*n,device=dev);q=torch.tensor([[0.,0.,0.,1.]]*n,device=dev)
    eq.set_robot(task,drone,q)
    target=torch.tensor([(np.array([500.,0,0]) if s['target'] is None else np.array(s['target']))+np.array([20.,20.,1.]) for s in ss],dtype=torch.float32,device=dev)
    task.target_position[:]=target
    # Sensor-only fixtures: scale just the placed asset's Warp vertices to the declared AABB.
    # No physics integration/contact claim; production assets, URDF and source remain unchanged.
    warp=task.sim_env.warp_env;per_env=warp.unfolded_env_vec_root_tensor.shape[0]//n
    for i,s in enumerate(ss):
        for j,b in enumerate(s['bars']):
            slot=slots[j];actor=i*per_env+slot+1
            mask=warp.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR==actor
            vertices=warp.VERTEX_MAPS_PER_ENV_ORIGINAL[mask]
            lo,hi=vertices.amin(0),vertices.amax(0)
            half=torch.tensor(b['half'],device=dev)
            warp.VERTEX_MAPS_PER_ENV_ORIGINAL[mask]=(vertices-(lo+hi)*.5)*(half/((hi-lo)*.5))
            task.obs_dict['obstacle_position'][i,slot]=drone[i]+torch.tensor(b['center'],device=dev)
    task.sim_env.IGE_env.write_to_sim();warp.reset_idx(torch.arange(n,device=dev))
    task._sync_target_to_sensor();task.detector.render_raw_rgbd(drone,task.obs_dict['robot_vehicle_orientation'],target,task.target_orientation)
    task.sim_env.render(render_components='sensors')
    lidar=task._lidar_distance_m().clone();semantic=task.obs_dict['segmentation_pixels']
    layouts=[s['bars'] for s in ss];pred_rel=[]
    for i,s in enumerate(ss):
        if s['target'] is not None:pred_rel.append(s['target'])
        else:
            row=int(np.argmin(abs(np.radians([20,10,0,-10])-math.radians(s['el']))))
            col=int(torch.atan2(torch.sin(p._lidar_angles-math.radians(s['b'])),torch.cos(p._lidar_angles-math.radians(s['b']))).abs().argmin())
            pred_rel.append(((float(lidar.view(n,4,72)[i,row,col])+.15)*direction(s['b'],s['el'])).tolist())
    if spec['kind']=='negative':
        original=ROOT.parent/'aerial_gym_simulator/results/eq1d_qualification_v1/EQ1D-R1/d2_scenes'
        original_meta=json.loads((original/'d2_meta.json').read_text())
        with np.load(original/'d2_renders.npz',allow_pickle=False) as z:
            for i,s in enumerate(ss[:7]):
                index=original_meta['scenes'].index(s['id'])
                lidar[i]=torch.as_tensor(z['level_scene__lidar_m'][index],device=dev).reshape_as(lidar[i])
                semantic[i]=torch.as_tensor(z['level_scene__seg_segmentation_pixels'][index],device=dev).reshape_as(semantic[i])
                layouts[i]=[{'center':(np.array(b['centre'])-z['drone_env_m'][index]).tolist(),'half':b['half']} for b in original_meta['layout'][index]]
                col=int(torch.atan2(torch.sin(p._lidar_angles-math.radians(s['b'])),torch.cos(p._lidar_angles-math.radians(s['b']))).abs().argmin())
                pred_rel[i]=((float(lidar.view(n,4,72)[i,2,col])+.15)*direction(s['b'],0)).tolist()
    rel=torch.tensor(pred_rel,dtype=torch.float32,device=dev);base_prediction=drone+rel
    arrays={'lidar':host(lidar),'semantic':host(semantic),'drone':host(drone),'target':host(target),
            'prediction_rel':host(rel),'angles':host(p._lidar_angles),'elevations':host(p._s_l1b_lidar_elevations)}
    all_firewall=True
    for oi,off in enumerate((-.15,0,.15)):
        prediction=base_prediction+off*rel/rel.norm(dim=1,keepdim=True)
        for label,prior in (('gate',.01),('cov',1.)):
            data=associate(task,lidar,prediction,prior)
            for key,value in data.items():arrays['o%d_%s_%s'%(oi,label,key)]=value
            saved_target=task.target_position.clone();saved_semantic=semantic.clone();saved_mask=task.detector.target_mask.clone()
            task.target_position.add_(7.);semantic.add_(100);task.detector.target_mask.zero_()
            repeated=associate(task,lidar,prediction,prior)
            task.target_position.copy_(saved_target);semantic.copy_(saved_semantic);task.detector.target_mask.copy_(saved_mask)
            for key in ('valid','measured','state','cov_after','identity_age','fused_age'):
                all_firewall &= np.array_equal(data[key],repeated[key])
                arrays['o%d_%s_replay_%s'%(oi,label,key)]=repeated[key]
    # GPU observation-level clock contract, with explicit sensor-packet loss, not GT visibility.
    prime(p,base_prediction);p.tracker.age[:]=0;p._s_l1c_identity_age[:]=0
    def observe(packet,rgb=None,depth=None):
        return p.observe(torch.zeros(n,3,90,160,device=dev) if rgb is None else rgb,
            torch.full((n,90,160),10.,device=dev) if depth is None else depth,packet,
            drone,torch.zeros_like(drone),q,torch.zeros(n,device=dev),torch.zeros(n,4,device=dev),
            task.task_config.max_velocity,task.task_config.flight_altitude,training=False)
    identity_trace=[];fused_trace=[];source_trace=[]
    for _ in range(10):
        observe(lidar)
        identity_trace.append(host(p._s_l1c_identity_age));fused_trace.append(host(p.tracker.age));source_trace.append(host(p._s_l1c_update_source))
    identity_ok=bool(torch.allclose(p._s_l1c_identity_age,torch.ones_like(p._s_l1c_identity_age),atol=1e-5))
    prime(p,base_prediction);p.tracker.age[:]=0
    expiry_active=[];expiry_age=[]
    for _ in range(50):
        observe(torch.full_like(lidar,12.));expiry_active.append(host(p.tracker.active));expiry_age.append(host(p.tracker.age))
    active50=bool(p.tracker.active.all());observe(torch.full_like(lidar,12.));expired51=bool((~p.tracker.active).all())
    expiry_active.append(host(p.tracker.active));expiry_age.append(host(p.tracker.age))
    camera_packet=torch.zeros(n,3,90,160,device=dev);camera_packet[:,0,43:47,78:82]=1.
    camera_depth=torch.full((n,90,160),10.,device=dev);camera_depth[:,43:47,78:82]=2.
    _,camera_diag=observe(torch.full_like(lidar,12.),camera_packet,camera_depth)
    camera_reset=bool(camera_diag['camera_visible'].all() and (p._s_l1c_identity_age==0).all())
    arrays.update(clock_identity_trace=np.stack(identity_trace),clock_fused_trace=np.stack(fused_trace),
        clock_update_source=np.stack(source_trace),clock_expiry_active=np.stack(expiry_active),clock_expiry_age=np.stack(expiry_age),
        clock_camera_valid=host(camera_diag['camera_visible']),clock_camera_identity_age=host(p._s_l1c_identity_age),
        clock_camera_update_source=host(p._s_l1c_update_source))
    files=[tel.write_npz(out/'sensor_probes.npz',arrays)]
    meta={'spec':spec,'scenes':ss,'layouts':layouts,'origins':origins,'firewall_equal':all_firewall,
          'identity_age_10_losses_ok':identity_ok,'active_on_miss50':active50,'expired_on_miss51':expired51,'camera_resets_identity':camera_reset,
          'replay_source':'EQ1D-R1 level_scene first seven' if spec['kind']=='negative' else None,
          'sensor_geometry_only':'Warp fixture AABB dimensions; no physics/contact qualification',
          'firewall_intervention':'GT target position +[7,7,7], semantic IDs+100, private renderer mask zero; fixed sensor packet/prediction replay',
          'flags':{x:bool(getattr(p,'sensor_lineage_'+x,False)) for x in ('s_l1a','s_l1b','s_l1c','s_l1d','s_l1c_v2')}}
    files.append(tel.write_json(out/'sensor_meta.json',meta))
    return files,eq.nan_report(arrays)


def run_v3(spec,out):
    original=eq.build_player;measured={};trace={'identity_age':[],'update_source':[],'camera_visible':[],'track_age':[]}
    def build(*args,**kwargs):
        result=original(*args,**kwargs);task=eq.find_task(result[0]);p=task.perception
        measured['actor_obs_dim']=int(task.task_obs['observations'].shape[1])
        measured['flags']={x:bool(getattr(p,'sensor_lineage_'+x,False)) for x in ('s_l1a','s_l1b','s_l1c','s_l1d','s_l1c_v2')}
        old=p.observe
        def observe(*a,**k):
            obs,diag=old(*a,**k)
            for dest,source in (('identity_age','s_l1c_identity_age'),('update_source','s_l1c_update_source'),('camera_visible','camera_visible'),('track_age','track_age')):
                if source in diag:trace[dest].append(host(diag[source]))
            return obs,diag
        p.observe=observe
        return result
    eq.build_player=build
    try:files,bad=el.run_v3(spec,out)
    finally:eq.build_player=original
    files.append(tel.write_npz(out/'identity_trace.npz',{k:np.stack(v) for k,v in trace.items() if v}))
    return files,bad,measured


def run_firewall(spec,out):
    original=eq.build_player;measured={}
    def build(*args,**kwargs):
        result=original(*args,**kwargs);p=eq.find_task(result[0]).perception
        measured['flags']={x:bool(getattr(p,'sensor_lineage_'+x,False)) for x in ('s_l1a','s_l1b','s_l1c','s_l1d','s_l1c_v2')}
        return result
    eq.build_player=build
    try:files,bad=eq.run_d3(spec,out)
    finally:eq.build_player=original
    return files,bad,measured


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--process',required=True);parser.add_argument('--out',required=True)
    args=parser.parse_args();spec=next(x for x in run.specs(run.plan()) if x['id']==args.process)
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False);t0=time.time()
    if spec['batch']=='V3':files,bad,measured=run_v3(spec,out)
    elif spec['batch']=='D3':files,bad,measured=run_firewall(spec,out)
    else:files,bad=run_sensors(spec,out);measured={}
    tel.write_json(out/'process_done.json',dict(process=spec['id'],process_spec=spec,files=files,nan_fields=bad,
        wall_s=time.time()-t0,**measured))
    return 3 if bad else 0

if __name__=='__main__':sys.exit(main())
