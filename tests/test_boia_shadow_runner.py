"""CPU guard/supervisor tests. GPU queries and child execution use explicit stand-ins."""
import ast
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import boia_shadow as b


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.p=b.load(b.PLAN);self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.p['output_root']=str(Path(self.tmp.name)/'BOIA-SHADOW-Q01-V2')

    def test_fixed_N_seed_caps_lineage_and_resource_plan(self):
        b.validate_plan(self.p)
        for section,key,value in (('profile','num_envs',128),('profile','seed',4701),
                                  ('limits','disk_stop_bytes',8000000000),('limits','no_retry',False),
                                  ('limits','campaign_wall_s',3600),('limits','campaign_wall_s',720),
                                  ('limits','poll_s',.5),('profile','lineage','s_l1d_v1'),
                                  ('supersedes','config_sha256','0'*64),('supersedes','status','RUN'),
                                  ('runtime_env','NAVRL_PERCEPTION_PERTURB','1')):
            p=copy.deepcopy(self.p);p[section][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):b.validate_plan(p)
        for change in (lambda p:p['stages']['S1_NATIVE'].update(observer=True),
                       lambda p:p['stages']['S1_NATIVE'].update(snapshot_cap=1),
                       lambda p:p['stages']['S0'].update(wall_s=120),
                       lambda p:p['stage_order'].reverse(),
                       lambda p:p['comparisons']['native_vs_off'].update(mode='prefix'),
                       lambda p:p['inactive_runtime_env']['keys'].pop()):
            p=copy.deepcopy(self.p);change(p)
            with self.subTest(change=change),self.assertRaises(ValueError):b.validate_plan(p)

    def test_runtime_env_scrubs_inherited_risk_and_keeps_safe_process_contract(self):
        with patch.dict(os.environ,{'NAVRL_TARGET_DYNAMICS':'physical','AERIAL_GYM_SIM_NAME':'wrong',
                                    'NAVRL_RANDOM_EXTRA':'1','PYTHONPATH':'/outside','BOIA_SHADOW_OLD':'1','PATH':'/bin'}):
            e=b.process_env(self.p,123,'abc')
        self.assertEqual(e['NAVRL_TARGET_DYNAMICS'],'bounded');self.assertNotIn('NAVRL_RANDOM_EXTRA',e)
        self.assertEqual(e['PYTHONPATH'],str(ROOT));self.assertEqual(e['BOIA_SHADOW_PARENT_PID'],'123')
        self.assertNotIn('BOIA_SHADOW_OLD',e);self.assertEqual(e['PYTHONNOUSERSITE'],'1')

    def test_no_GPU_subcommand_without_explicit_switch(self):
        for command in ('s0','s1-native','s1-off','s1-on','campaign'):
            with self.subTest(command=command),patch.object(b,'run_stage')as launch:
                with self.assertRaisesRegex(ValueError,'GPU execution requires'):
                    b.main([command,'--approved-commit','a'*40])
                launch.assert_not_called()

    def test_worker_cannot_bypass_supervisor_or_import_Isaac(self):
        with patch.dict(os.environ,{},clear=True),patch.object(b,'guard_sources')as guard:
            with self.assertRaisesRegex(ValueError,'SUPERVISOR'):b.worker(self.p,'S0','a'*40)
            guard.assert_not_called()

    def test_preflight_is_import_only_and_never_invokes_device_or_launcher(self):
        code="import json,sys;from tools import boia_shadow as b;p=b.load(b.PLAN);x=b.preflight(p,b.git('rev-parse','HEAD'));assert 'isaacgym' not in sys.modules;assert 'torch' not in sys.modules;print(json.dumps({'status':x['status'],'no_gpu_import':True}))"
        e=dict(os.environ,PYTHONNOUSERSITE='1',CUDA_VISIBLE_DEVICES='')
        done=subprocess.run([sys.executable,'-B','-c',code],cwd=str(ROOT),env=e,capture_output=True,text=True,timeout=20)
        self.assertEqual(done.returncode,0,done.stderr)
        self.assertTrue(json.loads(done.stdout)['no_gpu_import'])

    def test_dirty_HEAD_config_and_untracked_guards(self):
        for values in (['wrong','',''],['a'*40,' M tracked.py',''],['a'*40,'','rogue.py']):
            with self.subTest(values=values),patch.object(b,'git',side_effect=values):
                with self.assertRaises(ValueError):b.guard_sources('a'*40,'b'*64)
        with patch.object(b,'git',side_effect=['a'*40,'','']),patch.object(b,'sha',return_value='wrong'):
            with self.assertRaisesRegex(ValueError,'CONFIG_CHANGED'):b.guard_sources('a'*40,'b'*64)

    def test_compare_rows_exact_including_missing_rows_and_keys(self):
        row={'key':{'episode':0,'action':0},'sensor':'raw','rng_after':'fixed','outcome':[False,False,False]}
        self.assertEqual(b.compare_rows([row],[dict(row)])['status'],'PASS')
        for change in ({**row,'sensor':'changed'},{**row,'reset_expected':True},{**row,'key':{'episode':0}}):
            result=b.compare_rows([row],[change])
            self.assertEqual((result['status'],result['index']),('MISMATCH',0));self.assertTrue(result['fields'])
        for left,right in (([row],[]),([],[row]),(None,[row]),([row],None)):
            self.assertEqual(b.compare_rows(left,right)['status'],'MISMATCH')

    def fake_stage(self, root, stage, host, parity=None, ended=0, **meta):
        d=root/stage;d.mkdir(parents=True)
        (d/'host.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in host))
        if parity is not None:(d/'parity.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in parity))
        observer=self.p['stages'][stage]['observer'];rms=self.p['frozen']['rms_digest_sha256']
        done=dict(stage=stage,status='COMPLETED',source='a'*40,config_sha=b.sha(b.PLAN),
                  checkpoint=self.p['frozen']['policy_f_sha256'],rms_start=rms,rms_end=rms,
                  ended_episodes=ended,actions=len(host)-1,runtime_attestation_start={'cudnn_benchmark':True},
                  hooks_during={'task.step':observer,'contact_chain':observer})
        done.update(meta);b.write_new(d/'process_done.json',done)

    def synthetic(self, case):
        p=copy.deepcopy(self.p);p['output_root']=str(Path(self.tmp.name)/case/'BOIA-SHADOW-Q01-V2')
        root=Path(p['output_root']);root.mkdir(parents=True);self.p,saved=p,self.p
        host=[{'kind':'INITIAL','returned':'r0'}]+[{'kind':'STEP','index':i,'state':{'own_position':'x%d'%i}} for i in range(30)]
        parity=[{'key':{'action':i},'sensor':'s%d'%i} for i in range(30)]
        stage={'S0':dict(host=host[:21],parity=parity[:20]),'S1_NATIVE':dict(host=host),
               'S1_OFF':dict(host=host,parity=parity),'S1_ON':dict(host=host,parity=parity)}
        changes={'native_row':lambda:stage['S1_NATIVE'].update(host=host[:5]+[{**host[5],'state':{'own_position':'moved'}}]+host[6:]),
                 'off_empty':lambda:stage['S1_OFF'].update(host=[]),
                 'on_numerics':lambda:stage['S1_ON'].update(runtime_attestation_start={'cudnn_benchmark':False}),
                 's0_ended_early':lambda:stage['S0'].update(ended=1),
                 'native_hooked':lambda:stage['S1_NATIVE'].update(hooks_during={'task.step':True,'contact_chain':True}),
                 'wrong_source':lambda:stage['S1_OFF'].update(source='b'*40)}
        if case in changes:changes[case]()
        for name,spec in stage.items():
            if case=='on_missing' and name=='S1_ON':continue
            if case=='on_online_mismatch' and name=='S1_ON':
                (root/name).mkdir();b.write_new(root/name/'failure.json',dict(status='STOPPED_INTEGRITY',reason='IntegrityError: OFF_ON_MISMATCH_actor',actions_completed=7))
                continue
            self.fake_stage(root,name,**spec)
        try:
            with patch.object(b,'guard_sources'):return b.verify(p,'a'*40)
        finally:self.p=saved

    def test_verify_verdicts_are_exact_and_never_attribute_cause(self):
        self.assertEqual(self.synthetic('pass')['verdict'],b.PASS);results={}
        for case,expected in (('native_row',b.MISMATCH),('off_empty',b.MISMATCH),('on_numerics',b.MISMATCH),
                              ('s0_ended_early',b.MISMATCH),('native_hooked',b.MISMATCH),('wrong_source',b.MISMATCH),
                              ('on_online_mismatch',b.MISMATCH),('on_missing',b.INCOMPLETE)):
            with self.subTest(case=case):
                result=results[case]=self.synthetic(case);self.assertEqual(result['verdict'],expected)
                self.assertNotEqual(result['claims']['observer_noninterference'] if case!='on_missing' else 'x',
                                    'SUPPORTED_EXACT_HOST_TRACE_THIS_PROFILE')
                if expected==b.MISMATCH:self.assertEqual(result['claims']['mismatch_cause'],'NOT_VERIFIED')
        row=results['native_row']['comparisons']['native_vs_off']['files']['host.jsonl']
        self.assertEqual((row['index'],row['fields']),(5,['state.own_position']))

    def test_verify_requires_campaign_root_and_never_overwrites(self):
        with patch.object(b,'guard_sources'):
            with self.assertRaisesRegex(ValueError,'CAMPAIGN_ROOT_MISSING'):b.verify(self.p,'a'*40)
        self.synthetic('pass')
        p=copy.deepcopy(self.p);p['output_root']=str(Path(self.tmp.name)/'pass'/'BOIA-SHADOW-Q01-V2')
        with patch.object(b,'guard_sources'),self.assertRaises(FileExistsError):b.verify(p,'a'*40)

    def test_stage_requires_completed_predecessor_and_no_retry(self):
        with patch.object(b,'preflight',return_value={'status':'PASS_CPU_PREFLIGHT','checks':{},'config_sha256':b.sha(b.PLAN)}):
            with self.assertRaises(FileNotFoundError):b.run_stage(self.p,'S1_OFF','a'*40)
            root=Path(self.p['output_root']);root.mkdir()
            with self.assertRaisesRegex(ValueError,'PATH_EXISTS'):b.run_stage(self.p,'S0','a'*40)
            b.write_new(root/'campaign_start.json',dict(source='a'*40,preflight={'config_sha256':b.sha(b.PLAN)}))
            b.write_new(root/'S0_supervisor.json',dict(status='COMPLETED',wall_s=1.))
            with self.assertRaises(FileNotFoundError):b.run_stage(self.p,'S1_OFF','a'*40)  # S1_NATIVE missing
            b.write_new(root/'S1_NATIVE_supervisor.json',dict(status='STOPPED_INTEGRITY',wall_s=1.))
            with self.assertRaisesRegex(ValueError,'PREVIOUS_STAGE_NOT_PASS'):b.run_stage(self.p,'S1_OFF','a'*40)

    def test_device_query_failure_keeps_partial_receipt_and_manifest(self):
        pref={'status':'PASS_CPU_PREFLIGHT','checks':{},'config_sha256':b.sha(b.PLAN)}
        with patch.object(b,'preflight',return_value=pref),patch.object(b,'gpu_state',side_effect=RuntimeError('device unavailable')):
            result=b.run_stage(self.p,'S0','a'*40)
        self.assertEqual(result['status'],'STOPPED_INTEGRITY')
        self.assertIn('device unavailable',result['reason'])
        root=Path(self.p['output_root']);self.assertTrue((root/'S0_RAW_MANIFEST.json').is_file())
        self.assertTrue((root/'S0_supervisor.json').is_file())
        with self.assertRaises(ValueError):
            with patch.object(b,'preflight',return_value=pref):b.run_stage(self.p,'S0','a'*40)

    def test_watchdog_stops_CPU_standin_and_does_not_make_task_timeout(self):
        class Proc:
            pid=999999;returncode=None
            def poll(self):return None
        pref={'status':'PASS_CPU_PREFLIGHT','checks':{},'config_sha256':b.sha(b.PLAN)}
        with patch.object(b,'preflight',return_value=pref),patch.object(b,'gpu_state',return_value=dict(name='RTX3070',total_mib=8192,used_mib=100)),patch.object(b.subprocess,'Popen',return_value=Proc()),patch.object(b,'_terminate')as stop,patch.object(b.time,'monotonic',side_effect=[0.,180.,180.]):
            result=b.run_stage(self.p,'S0','a'*40)
        stop.assert_called_once();self.assertEqual(result['status'],'STOPPED_INTEGRITY')
        self.assertIn('CENSORING_NOT_TASK_TIMEOUT',result['reason'])

    def test_output_disk_and_VRAM_guards_independent_of_worker(self):
        class Proc:
            pid=999999;returncode=None
            def poll(self):return None
        for cause in ('disk','output','vram'):
            p=copy.deepcopy(self.p);p['output_root']=str(Path(self.tmp.name)/cause/'BOIA-SHADOW-Q01')
            pref={'status':'PASS_CPU_PREFLIGHT','checks':{},'config_sha256':b.sha(b.PLAN)}
            def gpu():return dict(name='RTX3070',total_mib=8192,used_mib=7000 if cause=='vram' else 100)
            with self.subTest(cause=cause),patch.object(b,'preflight',return_value=pref),patch.object(b,'gpu_state',side_effect=gpu),patch.object(b.subprocess,'Popen',return_value=Proc()),patch.object(b,'guard_sources'),patch.object(b,'_terminate'),patch.object(b.shutil,'disk_usage',return_value=NS(free=1 if cause=='disk' else 9000000000)),patch.object(b,'directory_bytes',return_value=999999999 if cause=='output' else 0):
                result=b.run_stage(p,'S0','a'*40)
            self.assertEqual(result['status'],'STOPPED_INTEGRITY');self.assertIsNotNone(result['reason'])

    def test_supervisor_receipt_keeps_worker_first_cause(self):
        class Proc:
            pid=999999;returncode=1
            def poll(self):return 1
        root=Path(self.p['output_root'])
        def launch(*a,**kw):
            (root/'S0').mkdir();b.write_new(root/'S0'/'failure.json',dict(status='STOPPED_INTEGRITY',reason='RuntimeError: CUDA out of memory (stand-in)'))
            return Proc()
        pref={'status':'PASS_CPU_PREFLIGHT','checks':{},'config_sha256':b.sha(b.PLAN)}
        with patch.object(b,'preflight',return_value=pref),patch.object(b,'gpu_state',return_value=dict(name='RTX3070',total_mib=8192,used_mib=100)),patch.object(b.subprocess,'Popen',side_effect=launch),patch.object(b,'_terminate'),patch.object(b,'guard_sources'):
            result=b.run_stage(self.p,'S0','a'*40)
        self.assertEqual(result['status'],'STOPPED_INTEGRITY');self.assertIn('WORKER_NONZERO_EXIT_1',result['reason'])
        self.assertIn('CUDA out of memory',result['first_cause']);self.assertEqual(result['worker_returncode'],1)
        self.assertEqual(b.load(root/'S0_supervisor.json')['worker_failure']['reason'],result['first_cause'])

    def test_source_guard_runs_less_often_than_resource_poll(self):
        class Proc:
            pid=999999;returncode=0;polls=0
            def poll(self):
                self.polls+=1;return None if self.polls<=12 else 0
        root,clock,proc=Path(self.p['output_root']),iter(range(100000)),Proc()
        def launch(*a,**kw):
            (root/'S0').mkdir();b.write_new(root/'S0'/'process_done.json',dict(stage='S0',status='COMPLETED',source='a'*40,config_sha=b.sha(b.PLAN)))
            return proc
        pref={'status':'PASS_CPU_PREFLIGHT','checks':{},'config_sha256':b.sha(b.PLAN)}
        with patch.object(b,'preflight',return_value=pref),patch.object(b,'gpu_state',return_value=dict(name='RTX3070',total_mib=8192,used_mib=100)),patch.object(b.subprocess,'Popen',side_effect=launch),patch.object(b,'_terminate'),patch.object(b,'guard_sources')as guard,patch.object(b.time,'monotonic',side_effect=lambda:float(next(clock))),patch.object(b.time,'sleep')as sleep,patch.object(b.shutil,'disk_usage',return_value=NS(free=9000000000)),patch.object(b,'directory_bytes',return_value=0):
            result=b.run_stage(self.p,'S0','a'*40)
        self.assertEqual(result['status'],'COMPLETED',result['reason']);self.assertEqual(sleep.call_count,12)
        self.assertTrue(2<=guard.call_count-1<12)  # one extra guard after the worker exits
        self.assertTrue(all(c.args[0]<=self.p['limits']['poll_s'] for c in sleep.call_args_list))

    def test_preregistration_and_amendment_match_executable_plan(self):
        p=b.load(b.PLAN);md=(b.ROOT/p['preregistration']).read_text();amend=(b.ROOT/p['supersedes']['amendment']).read_text()
        for token in (p['run_id'],*b.STAGES,'1080',b.PASS,b.MISMATCH,b.INCOMPLETE,p['output_root']):
            self.assertIn(token,md)
        for token in (b.V1['config_sha256'],b.V1['source_commit'],'S1_NATIVE','WITHDRAWN_BEFORE_EXECUTION_NOT_RUN'):
            self.assertIn(token,amend)
        self.assertEqual(b.sha(b.ROOT/b.V1['config']),b.V1['config_sha256'])
        self.assertIn(p['preregistration'],b.CODE);self.assertIn(p['supersedes']['amendment'],b.CODE)

    def test_latency_summary_is_named_and_handles_empty_sample(self):
        self.assertEqual(b.latency_summary([])['n'],0)
        x=b.latency_summary([1.,2.,3.]);self.assertEqual((x['n'],x['P50'],x['maximum']),(3,2.,3.))
        self.assertAlmostEqual(x['P95'],2.9)

    def test_manifest_hashes_partial_data_without_modifying_it(self):
        d=Path(self.tmp.name)/'data';d.mkdir();p=d/'partial.txt';p.write_text('failed original\n');before=b.sha(p)
        b.manifest(d,d/'manifest.json');self.assertEqual(b.sha(p),before)
        self.assertEqual(b.load(d/'manifest.json')['files']['partial.txt']['sha256'],before)

    def test_exclusive_JSON_write_no_overwrite(self):
        p=Path(self.tmp.name)/'receipt.json';b.write_new(p,{'status':'FAIL'})
        with self.assertRaises(FileExistsError):b.write_new(p,{'status':'PASS'})
        self.assertEqual(b.load(p)['status'],'FAIL')

    def test_only_worker_function_contains_simulator_import(self):
        tree=ast.parse((ROOT/'tools/boia_shadow.py').read_text())
        for node in tree.body:
            if isinstance(node,ast.FunctionDef)and node.name=='worker':continue
            for child in ast.walk(node):
                if isinstance(child,ast.Import):self.assertFalse(any(x.name in ('isaacgym','torch')for x in child.names))

    def test_runtime_profile_handles_actual_InitialViewSpec_contract(self):
        import torch
        from motar_eval.initial_view import InitialViewSpec
        pc=NS(detection_latency_s=0.,camera_offset=torch.tensor([[.1,0.,.03]]),pose_noise_pos_m=0.,
              pose_noise_yaw_deg=0.,empirical_error=None,_detector_noise_active=False)
        detector=NS(height=90,width=160,detect_height=90,detect_width=160,target_radius=.15,
                    **{x:0. for x in ('app_hue_deg','app_light_gain','app_albedo_jitter','app_texture_std',
                                     'app_motion_blur','mount_rot_deg','mount_trans_m')})
        cfg=NS(height=4,width=72,max_range=12.,min_range=.2,target_radius=.20,
               sensor_noise=NS(enable_sensor_noise=False),yaw_only_attach=True,randomize_placement=False,
               normalize_range=True,return_pointcloud=False,pointcloud_in_world_frame=False,
               nominal_position=[0.,0.,0.],euler_frame_rot_deg=[0.,0.,0.],
               horizontal_fov_deg_min=-175.,horizontal_fov_deg_max=180.,vertical_fov_deg_min=-10.,vertical_fov_deg_max=20.)
        task=NS(perception=pc,detector=detector,task_config=NS(episode_len_steps=600),num_envs=1,
                task_obs={'observations':torch.zeros(1,898)},step_dt=.1,vision_mode=True,perception_mode=True,
                _target_distance_backend='direct_l2_v1',_target_dynamics='bounded',_physical_target=False,
                _target_behavior_level='e2_obstacle_aware',n_bars_active=70,_kroc_initial_view=InitialViewSpec('visible'),
                perception_cfg=NS(enable_perturbations=False),speed_governor_cfg=NS(mode='riskcap'),
                _kroc_search=None,interactive_mode=False,_kroc_adapter=None,
                sim_env=NS(robot_manager=NS(warp_sensor=NS(cfg=cfg)),sim_config=NS(sim=NS(dt=.01)),
                           cfg=NS(env=NS(num_physics_steps_per_env_step_mean=10,num_physics_steps_per_env_step_std=0.))))
        player=NS(model=NS(training=False),is_rnn=False)
        with patch.dict(os.environ,{'NAVRL_SENSOR_LINEAGE':'off'}):
            self.assertTrue(all(b.runtime_profile(task,player,self.p).values()))
            pc.detection_latency_s=.1
            with self.assertRaisesRegex(ValueError,'RUNTIME_PROFILE_FAILED'):b.runtime_profile(task,player,self.p)

    def test_worker_reference_load_error_preserves_original_failure_receipt(self):
        # Library/player are CPU stand-ins; no real Isaac import or CUDA call occurs.
        import torch
        from motar_eval import eq1d_telemetry as tel
        fake=ModuleType('eq1d_fixture_launcher');fake.build_player=lambda *a:(NS(model=NS()),{})
        fake.find_task=lambda player:NS();fake.check_origins=lambda:{};fake.find_rms=lambda model:NS()
        Path(self.p['output_root']).mkdir()
        env={'BOIA_SHADOW_PARENT_PID':str(os.getppid()),'BOIA_SHADOW_CONFIG_SHA':b.sha(b.PLAN)}
        with patch.dict(sys.modules,{'isaacgym':ModuleType('isaacgym'),'eq1d_fixture_launcher':fake}),patch.dict(os.environ,env),patch.object(b,'guard_sources'),patch.object(b,'runtime_profile',return_value={}),patch.object(tel,'rms_digest',return_value=self.p['frozen']['rms_digest_sha256']),patch.object(torch.cuda,'reset_peak_memory_stats')as cuda:
            with self.assertRaisesRegex(RuntimeError,'FileNotFoundError'):
                b.worker(self.p,'S1_ON','a'*40)
            cuda.assert_not_called()
        failure=b.load(Path(self.p['output_root'])/'S1_ON/failure.json')
        self.assertIsNone(failure['actions_completed']);self.assertNotIn('UnboundLocalError',failure['reason'])

    def test_static_hooks_match_real_source_boundaries(self):
        tree=ast.parse((ROOT/'aerial_gym/task/navrl_task/navrl_task.py').read_text())
        methods={n.name:n for c in tree.body if isinstance(c,ast.ClassDef)
                 for n in c.body if isinstance(n,ast.FunctionDef)}
        def calls(node,name):return sum(isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr==name for n in ast.walk(node))
        self.assertEqual(calls(methods['_process_obs_perception'],'observe'),1)
        self.assertEqual(calls(methods['_process_obs_perception'],'render_raw_rgbd'),1)
        for name in ('transform_action_to_command','compute_state_reward_and_terminations','post_reward_calculation_step'):
            self.assertEqual(calls(methods['step'],name),1,name)
        self.assertEqual(calls(methods['get_return_tuple'],'process_obs_for_task'),1)
        self.assertEqual(calls(methods['process_obs_for_task'],'_process_obs_perception'),1)


if __name__=='__main__':unittest.main()
