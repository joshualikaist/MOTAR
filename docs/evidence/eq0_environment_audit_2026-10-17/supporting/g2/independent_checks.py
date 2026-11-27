"""EQ-0/G2: read-only record audit and independent scalar/ray mathematics.

No project imports, simulator, Warp, Torch, RNG, or writes outside this scratch folder.
This is a diagnostic reference, not a new renderer validation experiment.
"""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
from collections import Counter
import numpy as np

HERE = Path(tempfile.mkdtemp(prefix='motar-eq0-g2-'))
print('Fresh scratch output:', HERE)
REPO = Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1')
RAW = Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2')
SOURCE = '5ff285ad053f181c5a1c86dddb2462bb44e6327b'
G0 = '6441c1ff74b6d085d14ed5604d1d60d66a4634fe'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_text())

def qrotate(q, v):
    # Independent rotation matrix, not the cross-product implementation under audit.
    x,y,z,w = np.asarray(q, float)
    matrix=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return np.asarray(v, float) @ matrix.T

def blocked_reference(eye, target, centers, halves):
    # Intersect the six linear face inequalities with the segment parameter interval.
    a=np.zeros(len(centers));b=np.ones(len(centers));v=target-eye
    possible=np.ones(len(centers), bool)
    for axis in range(3):
        lower=centers[:,axis].astype(float)-halves[:,axis].astype(float)
        upper=centers[:,axis].astype(float)+halves[:,axis].astype(float)
        if abs(v[axis])<1e-15:
            possible &= (eye[axis]>=lower)&(eye[axis]<=upper)
        else:
            p=(lower-eye[axis])/v[axis];q=(upper-eye[axis])/v[axis]
            a=np.maximum(a,np.minimum(p,q));b=np.minimum(b,np.maximum(p,q))
    return bool(np.any(possible & (a<=b)))

def counts_and_lengths(values, acquisition):
    start=max(0,acquisition+1)
    if acquisition<0:return 0,[]
    spans=[];i=start
    while i<len(values):
        if not values[i]:i+=1;continue
        j=i+1
        while j<len(values) and values[j]:j+=1
        spans.append([i,j,(j-i)*.1]);i=j
    return len(spans),spans

def main():
    analysis=read(RAW/'analysis.json')
    receipt_keys=['NAVRL_PERCEPTION_PERTURB','NAVRL_DETECTOR_CHECKPOINT','NAVRL_CAMERA_WIDTH',
                  'NAVRL_CAMERA_HEIGHT','NAVRL_DETECTION_DROPOUT','NAVRL_DETECTION_LATENCY_S',
                  'NAVRL_RGB_NOISE_STD','NAVRL_DEPTH_NOISE_STD','NAVRL_P9_ERROR_MODEL',
                  'NAVRL_LIDAR_HBEAMS','NAVRL_LIDAR_VBEAMS','NAVRL_LIDAR_RANGE',
                  'NAVRL_CAM_FOV_SCALE_ERR','NAVRL_CAM_MOUNT_ROT_DEG','NAVRL_CAM_MOUNT_TRANS_M',
                  'NAVRL_LIDAR_RANGE_ONLY_UPDATE','NAVRL_LIDAR_TARGET_ASSOC']
    env_values={k:Counter() for k in receipt_keys}
    counts={k:Counter() for k in ['source_commit','source_dirty','render_alignment','capture_time','camera',
                                'target_render_mode','dynamic_mesh','distractors','empirical_error','dt_s']}
    summaries=[]
    for name in analysis['cells']:
        root=RAW/name;receipt=read(root/'receipt.json');summary=read(root/'occlusion_trace_summary.json')
        meta=summary['metadata'];condition=read(root/'result.json')['condition']
        for k in receipt_keys:env_values[k][receipt['env_snapshot_navrl'].get(k,'ABSENT')]+=1
        for k,value in [('source_commit',receipt['git_commit']),('source_dirty',meta['source_dirty']),
            ('render_alignment',meta['render_alignment']),('capture_time',meta['sensor_capture_time']),
            ('camera',json.dumps(meta['camera'],sort_keys=True)),('target_render_mode',condition['target_render_mode']),
            ('dynamic_mesh',condition['d8_dynamic_mesh_treatment']),('distractors',condition['distractor_count']),
            ('empirical_error',condition['p9_empirical_error_enabled']),('dt_s',meta['dt_s'])]:
            counts[k][str(value)]+=1
        summaries.append(summary)
    sources=['aerial_gym/task/navrl_task/navrl_detector.py','aerial_gym/task/navrl_task/navrl_perception.py',
             'aerial_gym/task/navrl_task/navrl_task.py','aerial_gym/task/navrl_task/kroc_eval_adapter.py',
             'aerial_gym/task/navrl_task/recovery_visibility_probe.py',
             'aerial_gym/config/task_config/navrl_task_config.py',
             'aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py',
             'aerial_gym/sensors/warp/warp_lidar.py','aerial_gym/sensors/warp/warp_sensor.py',
             'aerial_gym/sensors/warp/warp_kernels/warp_lidar_kernels.py',
             'motar_eval/occlusion_geometry.py','motar_eval/geometry.py','tools/occlusion_analysis.py',
             'tools/occlusion_confirmatory.py','tools/occlusion_confirmatory_analysis.py']
    source_rows=[]
    for relative in sources:
        measured=subprocess.check_output(['git','show',SOURCE+':'+relative],cwd=REPO)
        row={'path':relative,'current_sha256':sha(REPO/relative),
             'measurement_sha256':hashlib.sha256(measured).hexdigest()}
        row['current_equals_measurement']=row['current_sha256']==row['measurement_sha256']
        source_rows.append(row)
    g0_report=REPO/'results/occlusion_pilot_v1/OCC-G0_rerun1/g0_report.json'
    g0=read(g0_report)
    runtime_equal=subprocess.run(['git','diff','--quiet',G0,SOURCE,'--','aerial_gym','motar_eval','resources/robots'],cwd=REPO).returncode==0
    provenance={'measurement_source':SOURCE,'current_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
                'source_files':source_rows,'all_current_source_equal_measured':all(r['current_equals_measurement'] for r in source_rows),
                'g0_sha256':sha(g0_report),'g0_verdict':g0['verdict'],'g0_noninterference':g0['noninterference'],
                'runtime_equal_g0_to_measurement':runtime_equal,'g0_parity':g0['renderer_parity'],
                'cell_count':len(summaries),'recorded_values':counts,'receipt_env':env_values,
                'noise_application':'SOURCE_DERIVED_DISABLED: enable_perturbations=False; magnitudes alone do not activate noise/dropout',
                'independent_hardware_render_test':'NOT_TESTED','raw_rgb_depth_lidar_arrays':'NOT_RECORDED in OCC-P2 trace'}
    (HERE/'provenance.json').write_text(json.dumps(provenance,indent=2,sort_keys=True)+'\n')

    w,h=160,90;hfov,vfov=map(math.radians,[87,58]);fx=w/(2*math.tan(hfov/2));fy=h/(2*math.tan(vfov/2));cx,cy=(w-1)/2,(h-1)/2
    yy,xx=np.mgrid[:h,:w];rays=np.stack([np.ones_like(xx),-(xx-cx)/fx,-(yy-cy)/fy],axis=-1).astype(float)
    rays/=np.linalg.norm(rays,axis=-1,keepdims=True)
    cameras=[]
    for distance in [1,5,10,15,17,17.6,20]:
        target=np.array([distance,0.,0.]);dot=rays@target;disc=dot*dot-(distance*distance-.15**2)
        near=dot-np.sqrt(np.maximum(0,disc));valid=(disc>=0)&(near>=0)&(near<20.)
        pixels=int(valid.sum());estimate=float(near[valid].mean()+.15) if pixels else None
        cameras.append({'range_m':distance,'on_axis_target_pixels_no_obstacles':pixels,'camera_valid_min_two_pixels':pixels>=2,
                        'mean_surface_plus_radius_estimate_m':estimate,'center_range_bias_m':estimate-distance if estimate else None})
    mappings=[]
    for angle in range(-40,41,5):
        theta=math.radians(angle);u_linear=(hfov/2-theta)/hfov*(w-1);u_sample=int(np.rint(u_linear))
        expected=cx-fx*math.tan(theta);actual=math.degrees(math.atan((cx-u_sample)/fx))
        mappings.append({'lidar_bearing_deg':angle,'correct_pinhole_u':expected,'linear_u':u_linear,'sampled_u':u_sample,
                         'sampled_camera_bearing_deg':actual,'bearing_error_deg':actual-angle,'pixel_error':u_sample-expected})
    az=np.pi-np.arange(72)*(2*np.pi/72);el=np.radians([20,10,0,-10]);lidar_rays=np.array([[math.cos(a)*math.cos(e),math.sin(a)*math.cos(e),math.sin(e)] for e in el for a in az])
    lidar=[]
    for distance in [2,5,10,12]:
        for bearing in [0,2.5]:
            for elevation in [0,5]:
                b,e=map(math.radians,[bearing,elevation]);target=distance*np.array([math.cos(b)*math.cos(e),math.sin(b)*math.cos(e),math.sin(e)])
                dot=lidar_rays@target;disc=dot*dot-(distance*distance-.20**2);near=dot-np.sqrt(np.maximum(0,disc))
                valid=(disc>=0)&(near>=0)&(near<12)
                lidar.append({'center_range_m':distance,'bearing_deg':bearing,'elevation_deg':elevation,'hits':int(valid.sum())})
    math_checks={'camera_intrinsics':{'fx':fx,'fy':fy,'cx':cx,'cy':cy,'width':w,'height':h},
                 'camera_no_obstacle_sphere_results':cameras,'camera_ray_pixel_center_support_hfov_deg':2*math.degrees(math.atan(cx/fx)),
                 'camera_ray_pixel_center_support_vfov_deg':2*math.degrees(math.atan(cy/fy)),
                 'on_axis_camera_pixel_disappearance_center_range_m':.15/math.sqrt(1-rays[44,79,0]**2),
                 'fusion_mapping':mappings,'fusion_max_bearing_error_deg':max(abs(r['bearing_error_deg']) for r in mappings),
                 'fusion_body_to_camera_origin_front_wall_error_m':-.1,
                 'lidar_unique_azimuths':len(np.unique(az)),'lidar_spacing_deg':360/72,'lidar_elevations_deg':[20,10,0,-10],
                 'lidar_no_obstacle_sphere_results':lidar,'lidar_horizontal_halfbin_guaranteed_hit_limit_m':.2/math.sin(math.radians(2.5)),
                 'lidar_vertical_halfbin_guaranteed_hit_limit_m':.2/math.sin(math.radians(5)),
                 'lidar_on_axis_range_to_center_bias_from_radius_m':.15-.2,
                 'finite_rim_rays_cannot_prove_full_silhouette':True,
                 'note':'Independent double-precision reference math, not execution of the GPU renderer or proof of recorded per-frame sensor returns.'}
    (HERE/'independent_math.json').write_text(json.dumps(math_checks,indent=2,sort_keys=True)+'\n')

    dtype=np.dtype([('env','<u2'),('frame','<u2'),('episode','<u4'),('p_pos','<f4',(3,)),('p_vel','<f4',(3,)),
                    ('p_quat','<f4',(4,)),('p_vehicle_quat','<f4',(4,)),('t_pos','<f4',(3,)),('t_vel','<f4',(3,)),
                    ('flags','u1'),('probe_center_blocked','i1'),('probe_visible_pixels','<i4')])
    cases=[('moving/A3/E2_obstacle_aware__160bars__seed4309',53,3,6),('static/A1/E0_static__115bars__seed4307',60,0,32),
           ('static/A1/E0_static__160bars__seed4307',94,3,33),('static/A1/E0_static__205bars__seed4303',123,2,21),
           ('static/A1/E0_static__205bars__seed4305',78,3,4)]
    borders=[];sample_frames=0;fused_bad=0
    for cell,env,episode,frame in cases:
        root=RAW/cell;summary=read(root/'occlusion_trace_summary.json');meta=summary['metadata'];cam=meta['camera']
        for name,digest in summary['files'].items():assert sha(root/name)==digest
        all_frames=np.fromfile(root/'occlusion_frames.bin',dtype=dtype)
        sample_frames+=len(all_frames);flags=all_frames['flags'];fused_bad+=int(np.sum(((flags&2)>0)!=(((flags&1)>0)|((flags&32)>0))))
        fs=all_frames[(all_frames['env']==env)&(all_frames['episode']==episode)];fs.sort(order='frame')
        with np.load(root/'occlusion_layouts.npz',allow_pickle=False) as layout:
            reset=layout['resets'];index=int(reset[(reset['env']==env)&(reset['episode']==episode)]['layout'][0]);lo,hi=layout['offsets'][index:index+2]
            centers=layout['center'][lo:hi];halves=layout['half'][lo:hi];quats=layout['quat'][lo:hi]
            assert np.allclose(quats,[0,0,0,1])
        blocked=[];view=[];eyes=[];fov_margins=[]
        for row in fs:
            q=row['p_vehicle_quat'];eye=row['p_pos'].astype(float)+qrotate(q,cam['translation_m']);eyes.append(eye)
            target=row['t_pos'].astype(float);rel=qrotate([-q[0],-q[1],-q[2],q[3]],target-eye)
            margin=[rel[0]*math.tan(math.radians(cam['hfov_deg'])/2)-abs(rel[1]),rel[0]*math.tan(math.radians(cam['vfov_deg'])/2)-abs(rel[2]),cam['max_range_m']-np.linalg.norm(rel)]
            fov_margins.append(margin);view.append(rel[0]>0 and min(margin)>=0);blocked.append(blocked_reference(eye,target,centers,halves))
        blocked=np.array(blocked);view=np.array(view);flags=fs['flags'];camera=(flags&1)>0;lidar=(flags&32)>0;fused=(flags&2)>0
        acq=int(np.flatnonzero(fused)[0]) if fused.any() else -1
        base=view&blocked;online=((flags&8)>0)&~((flags&16)>0);probe=view&(fs['probe_center_blocked']==1)
        i=int(np.flatnonzero(fs['frame']==frame)[0]);eye=eyes[i];ray=fs['t_pos'][i].astype(float)-eye
        t0=(centers.astype(float)-halves.astype(float)-eye)/ray;t1=(centers.astype(float)+halves.astype(float)-eye)/ray
        enter=np.minimum(t0,t1).max(-1);leave=np.maximum(t0,t1).min(-1);gap=enter-leave
        valid=(enter<=1)&(leave>=0);closest=int(np.where(valid,abs(gap),np.inf).argmin())
        n_b,sp_b=counts_and_lengths(base,acq);n_o,sp_o=counts_and_lengths(online,acq);n_p,sp_p=counts_and_lengths(probe,acq)
        borders.append({'cell':cell,'env':env,'episode':episode,'frame':frame,'acquisition':acq,'offline_center_blocked':bool(blocked[i]),
            'offline_in_view':bool(view[i]),'online_in_view':bool(flags[i]&8),'online_visible':bool(flags[i]&16),
            'probe_center_blocked':int(fs['probe_center_blocked'][i]),'fov_range_margins_m':fov_margins[i],
            'nearest_boundary_gap_along_ray_m':float(gap[closest]*np.linalg.norm(ray)) if valid.any() else None,
            'interval_counts_offline_online_probe':[n_b,n_o,n_p],'interval_spans_offline_online_probe':[sp_b,sp_o,sp_p],
            'raw_trace_hashes_verified':True,'own_reference_not_project_functions':True})
    diagnostics={'known_unique_frames':len(borders),'rows':borders,'sampled_raw_frames':sample_frames,
                 'fused_flag_not_equal_camera_or_final_lidar':fused_bad,
                 'all_primary_counts_unchanged_in_affected_episodes':all(len(set(r['interval_counts_offline_online_probe']))==1 for r in borders),
                 'inference':'Tiny grazing/FOV margins support floating-point boundary sensitivity, not proof of the exact GPU arithmetic cause.'}
    (HERE/'boundary_audit.json').write_text(json.dumps(diagnostics,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'all112_source_verified':provenance['all_current_source_equal_measured'],'fusion_max_error_deg':math_checks['fusion_max_bearing_error_deg'],
                      'center_camera_disappearance_m':math_checks['on_axis_camera_pixel_disappearance_center_range_m'],
                      'border_frames':len(borders),'primary_counts_unchanged':diagnostics['all_primary_counts_unchanged_in_affected_episodes'],
                      'sampled_fused_consistency_errors':fused_bad,'sample_frames':sample_frames},indent=2))

if __name__=='__main__':main()
