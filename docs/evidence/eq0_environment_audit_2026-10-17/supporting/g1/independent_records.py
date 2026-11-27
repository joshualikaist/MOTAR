"""Read-only CPU record audit; independent NumPy formulas, no production geometry/control imports."""
import pathlib,json,hashlib,collections,math,xml.etree.ElementTree as ET,tempfile
import numpy as np
ROOT=pathlib.Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1')
RAW=ROOT.parent/'aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2'
OUT=pathlib.Path(tempfile.mkdtemp(prefix='motar-eq0-g1-'))
print('Fresh scratch output:', OUT)
DT=np.dtype([('env','<u2'),('frame','<u2'),('episode','<u4'),('p_pos','<f4',(3,)),('p_vel','<f4',(3,)),('p_quat','<f4',(4,)),('p_vehicle_quat','<f4',(4,)),('t_pos','<f4',(3,)),('t_vel','<f4',(3,)),('flags','u1'),('probe_center_blocked','i1'),('probe_visible_pixels','<i4')])
report={'scope':'primary consumed observations only; terminal post-action states and 100 Hz substeps absent','dtype_bytes':DT.itemsize,'files':{},'groups':{},'layout_samples':[]}
for group in ['static','moving']:
 acc=collections.defaultdict(list);count=0
 for p in sorted((RAW/group).rglob('occlusion_frames.bin')):
  f=np.fromfile(p,dtype=DT);ledger=[json.loads(x) for x in (p.parent/'episodes.jsonl').read_text().splitlines() if x];keys={(int(x['env_index']),int(x['env_episode_index'])) for x in ledger if x['in_quota']}
  mask=np.fromiter(((int(e),int(k)) in keys for e,k in zip(f['env'],f['episode'])),bool,count=len(f));f=f[mask];count+=len(f)
  q=f['p_quat'].astype(float);qn=(q*q).sum(1);tilt=np.degrees(np.arccos(np.clip(1-2*(q[:,0]**2+q[:,1]**2),-1,1)))
  acc['z'].append(f['p_pos'][:,2]);acc['vz'].append(f['p_vel'][:,2]);acc['tilt_deg'].append(tilt);acc['speed_xy'].append(np.linalg.norm(f['p_vel'][:,:2],axis=1));acc['target_z'].append(f['t_pos'][:,2]);acc['target_speed_xy'].append(np.linalg.norm(f['t_vel'][:,:2],axis=1));acc['quaternion_norm_error'].append(abs(qn-1))
  order=np.lexsort((f['frame'],f['episode'],f['env']));g=f[order];same=(g['env'][1:]==g['env'][:-1])&(g['episode'][1:]==g['episode'][:-1])&(g['frame'][1:]==g['frame'][:-1]+1)
  v=g['t_vel'].astype(float);acc['target_accel_endpoint_mps2'].append(np.linalg.norm(np.diff(v,axis=0),axis=1)[same]/.1)
  qq=g['p_quat'].astype(float);yaw=np.arctan2(2*(qq[:,3]*qq[:,2]+qq[:,0]*qq[:,1]),1-2*(qq[:,1]**2+qq[:,2]**2));dy=np.diff(yaw);dy=np.arctan2(np.sin(dy),np.cos(dy));acc['yaw_rate_interval_abs_rad_s'].append(np.abs(dy[same])/.1)
  displacement=np.diff(g['t_pos'].astype(float),axis=0);acc['target_velocity_displacement_residual_m'].append(np.linalg.norm(displacement-v[1:]*.1,axis=1)[same])
  summary=json.loads((p.parent/'occlusion_trace_summary.json').read_text());assert summary['frame_itemsize']==DT.itemsize
  h=hashlib.sha256(p.read_bytes()).hexdigest();assert h==summary['files']['occlusion_frames.bin'];report['files'][str(p.relative_to(RAW))]=h
  # Predetermined diagnostic sample: layout of env0 episode0 from every cell, including both arms.
  with np.load(p.parent/'occlusion_layouts.npz',allow_pickle=False) as z:
   rr=z['resets'];ii=int(rr[(rr['env']==0)&(rr['episode']==0)]['layout'][0]);a,b=map(int,z['offsets'][ii:ii+2]);c=z['center'][a:b].astype(float);hh=z['half'][a:b].astype(float)
   delta=np.abs(c[:,None,:2]-c[None,:,:2]);surface=np.maximum(delta-hh[:,None,:2]-hh[None,:,:2],0);dist=np.linalg.norm(surface,axis=2);iu=np.triu_indices(len(c),1);d=dist[iu];center=np.linalg.norm(c[:,None,:2]-c[None,:,:2],axis=2)[iu]
   report['layout_samples'].append({'cell':p.parent.name,'group':group,'layout_index':ii,'boxes':len(c),'height_min':float((2*hh[:,2]).min()),'height_max':float((2*hh[:,2]).max()),'bar_bottom_min':float((c[:,2]-hh[:,2]).min()),'bar_top_max':float((c[:,2]+hh[:,2]).max()),'overlapping_or_touching_pairs':int((d==0).sum()),'separate_pair_min_euclidean_surface_gap_m':float(d[d>0].min()),'separate_gap_below_yaw_invariant_body_diameter_count':int(((d>0)&(d<2*math.hypot(.14,.14))).sum()),'separate_gap_below_spawn_reserved_diameter_count':int(((d>0)&(d<2*((.13+.0635)*math.sqrt(2)+.45))).sum()),'min_positive_center_spacing_m':float(center[center>0].min())})
 report['groups'][group]={'frames':count}
 for name,values in acc.items():
  values=np.concatenate(values);report['groups'][group][name]={'min':float(values.min()),'max':float(values.max()),'p01_p50_p99':np.percentile(values,[1,50,99]).tolist(),'nonfinite':int((~np.isfinite(values)).sum())}
 report['groups'][group]['tilt_above45_frames']=int(sum((x>45).sum() for x in acc['tilt_deg']))
 report['groups'][group]['z_abs_error_above_0p1m_frames']=int(sum((abs(x-1)>.1).sum() for x in acc['z']))
xml=ET.parse(ROOT/'resources/robots/quad/quad_navrl_collide.urdf').getroot();mass=sum(float(x.attrib['value']) for x in xml.findall('.//mass'));box=[float(x) for x in xml.find("link[@name='base_link']/collision/geometry/box").attrib['size'].split()]
report['independent_algebra']={'urdf_mass_kg':mass,'collision_box_m':box,'yaw_invariant_radius_m':math.hypot(box[0]/2,box[1]/2),'rotation_invariant_3d_radius_m':np.linalg.norm(np.array(box)/2).item(),'body_width45yaw_m':.28*math.sqrt(2),'motor_max_total_N':8,'thrust_weight_ratio':8/(mass*9.81),'hover_thrust_at45tilt_N':mass*9.81/math.cos(math.pi/4),'steady_lateral_accel45_mps2':9.81,'ideal_target_tilt_at4mps2_deg':math.degrees(math.atan(4/9.81)),'target_turn_centripetal_accel_at1p5mps_150degps':1.5*math.radians(150),'target_stopping_distance_at1p5mps_4mps2':1.5**2/(2*4),'robot_command_xy_norm_ceiling':2.5*math.sqrt(2),'spawn_legacy_prop_radius_reserved_m':(.13+.0635)*math.sqrt(2)+.45,'two_0p8m_boxes_gap_at_diagonal_center_spacing1p6_m':1.6-.8*math.sqrt(2),'100Hz_time_step_s':.01,'10Hz_RL_dt_s':.1}
(OUT/'record_forensics.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({'groups':report['groups'],'algebra':report['independent_algebra'],'layout_samples':len(report['layout_samples']),'overlapping_sample_cells':sum(x['overlapping_or_touching_pairs']>0 for x in report['layout_samples']),'sample_min_separate_gap_m':min(x['separate_pair_min_euclidean_surface_gap_m'] for x in report['layout_samples'])},indent=1))
