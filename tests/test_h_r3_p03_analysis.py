"""Independent synthetic qualification failures; never load historical experimental raw."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'tests'))
import h_r3_p03_analysis as a
from test_h_r3_p02_analysis import fixture_trace,detect


def fixture(n=3,steps=60):
    tr=fixture_trace(n,steps)
    for key in a.DIGEST_KEYS:
        if key not in tr:
            tr[key]=tr['digest'].copy()
            tr['initial_'+key]=tr['initial_digest'].copy()
    tr['pre_vel']=tr['actual_vel'].copy()
    tr['actual_angvel']=np.zeros_like(tr['actual_vel'])
    tr['tracker_active']=np.ones((steps,n),bool)
    tr['tracker_age']=np.zeros((steps,n),np.float32)
    tr['direction_valid']=np.ones((steps,n),bool)
    for key in ('natural_reset','forced_injected','budget_frozen'):
        tr[key]=np.zeros((steps,n),bool)
    for key in ('schedule_lateral_mps','reference_norm_mps'):
        tr[key]=np.ones((steps,n),np.float32)
    tr['candidate_scores']=np.ones((steps,n,5),np.float32)
    tr['selected_side']=np.ones((steps,n),np.float32)
    for key in ('decision_index','episode_id','event_id'):
        tr[key]=np.zeros((steps,n),np.int64)
    tr['budget_frozen'][10:30,:2]=True
    tr['decision_index'][10:30,:2]=np.arange(20)[:,None]
    tr['initial_camera']=np.zeros(n,bool)
    tr['initial_identity_correct']=np.zeros(n,bool)
    tr['event_schedule_lateral_mps']=np.ones((n,20),np.float32)
    tr['event_reference_norm_mps']=np.ones((n,20),np.float32)
    tr['event_budget_digest']=np.zeros((n,32),np.uint8)
    for k in range(steps):
        for env in range(n):
            tr['sham_sign'][k,env]=1. if hashlib.sha256(('HR3-P03|4801|%d|0|0|%d'%(env,tr['decision_index'][k,env])).encode('ascii')).digest()[0]&1 else -1.
    tr['sham_side']=tr['sham_sign'].copy()
    return tr


def write(path,tr,identity=True):
    path=Path(path);path.mkdir()
    np.savez_compressed(path/'h_r3_p03_trace.npz',**tr)
    meta={'arm':'B0','seed':4801,'identity_enabled':identity,'steps':len(tr['camera']),'num_envs':len(tr['t0']),
          'step_dt':.1,'window_steps':20,'nan_fields':{},'model_training_flag':False,'vision_mode':True,
          'rms_digest_start':'frozen','rms_digest_end':'frozen','sensor_lineage':'off'}
    n=len(tr['t0']);steps=len(tr['camera'])
    meta['geometry_integrity']={'status':'PASS','bitwise_geometry':True,'immutable_topology':True,'checks':steps+1,'reset_event_count':1}
    (path/'h_r3_p03_meta.json').write_text(json.dumps(meta))
    snapshot(path)
    event={'step':-1,'mode':'selected','env_ids':list(range(n)),'refit_env_ids':list(range(n)),
           'write_vertex_ranges':[[i*3,(i+1)*3] for i in range(n)],'refit_mesh_ids':list(range(n)),
           'unselected_vertices_bitwise_preserved':True}
    root={'step':-1,'kind':'indexed_flush','env_ids':list(range(n))}
    frames=[]
    for k in range(-1,steps):
        frames.append({'step':k,'reset_ids':list(range(n)) if k<0 else np.flatnonzero(tr['reset'][k]).tolist(),
                       'natural_reset_ids':[],'forced_injected':False,
                       'warp_event_start':0 if k<0 else 1,'warp_event_end':1,
                       'root_write_event_start':0 if k<0 else 1,'root_write_event_end':1})
    for name,value in (('frame_meta.json',frames),('geometry_events.json',[event]),('root_write_events.json',[root])):
        (path/name).write_text(json.dumps(value))
    files=[{'file':p.relative_to(path).as_posix(),'bytes':p.stat().st_size,'sha256':a.sha(p)} for p in sorted(path.rglob('*')) if p.is_file()]
    (path/'process_done.json').write_text(json.dumps({'process':path.name,'actor_obs_dim':898,'nan_fields':{},'files':files}))


def snapshot(path,delta=0):
    path=Path(path)/'snapshots';path.mkdir()
    n=3
    np.savez(path/'initial.npz',rgb=np.full((n,3,2,2),delta,np.float32),depth=np.zeros((n,2,2),np.float32),
             lidar=np.zeros((n,8),np.float32),actor_obs=np.zeros((n,898),np.float32),root_state=np.zeros((n,1,13),np.float32),
             vertices=np.zeros((n*3,3),np.float32),vertex_env_index=np.repeat(np.arange(n),3),
             vertex_actor_index=np.repeat(np.arange(n),3),vertex_ranges=np.array([[0,3],[3,6],[6,9]]),mesh_ids=np.arange(n))


class PrefixAndIsolationTest(unittest.TestCase):
    def test_complete_prefix_and_no_event_envs_are_required(self):
        traces={arm:fixture() for arm in a.ARMS}
        self.assertTrue(a.matched_prefix(traces)['equal'])
        traces['B5']['depth_digest'][45,2,0]^=1
        result=a.matched_prefix(traces)
        self.assertFalse(result['equal'])
        self.assertEqual(len(result['environments']),3)
        self.assertTrue(result['environments'][0]['prefix_matched'])
        self.assertFalse(result['environments'][2]['prefix_matched'])

    def test_initial_and_last_preloss_sensor_history_are_compared(self):
        for key,where in (('initial_roots_digest',(0,0)),('actor_history_digest',(9,0,0)),('rgb_digest',(9,0,0))):
            traces={arm:fixture() for arm in a.ARMS}
            traces['B4'][key][where]^=1
            self.assertFalse(a.matched_prefix(traces)['equal'],key)

    def test_postbranch_changes_allowed_but_t0_mismatch_is_global_failure(self):
        traces={arm:fixture() for arm in a.ARMS}
        traces['B3']['rgb_digest'][10:,0]^=1
        self.assertTrue(a.matched_prefix(traces)['equal'])
        traces['B3']['t0'][0]+=1
        self.assertFalse(a.matched_prefix(traces)['equal'])

    def test_forced_reset_exact_clock_and_unaffected_envs(self):
        normal=fixture(128,60);forced=copy.deepcopy(normal)
        forced['reset'][2,127]=forced['forced_injected'][2,127]=True
        forced['digest'][2:,127]^=1
        self.assertTrue(a.check_forced_reset(normal,forced)['equal'])
        self.assertTrue(a.compare_traces(normal,forced,(127,))['equal'])
        forced['lidar_digest'][3,126,0]^=1
        result=a.compare_traces(normal,forced,(127,))
        self.assertFalse(result['equal'])
        self.assertEqual(result['fields']['lidar_digest']['first_env'],126)
        normal['natural_reset'][2,1]=True
        self.assertFalse(a.check_forced_reset(normal,forced)['equal'])

    def test_numeric_windows_catch_sensor_change_even_if_digests_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            l,r=Path(tmp)/'l',Path(tmp)/'r';l.mkdir();r.mkdir()
            snapshot(l);snapshot(r,delta=.01)
            result=a.compare_snapshots(l,r)
            self.assertFalse(result['equal'])
            self.assertEqual(result['mismatches'],[{'frame':'initial.npz','field':'rgb'}])

    def test_missing_snapshot_and_tampered_completion_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            l,r=Path(tmp)/'l',Path(tmp)/'r'
            write(l,fixture());write(r,fixture())
            with self.assertRaises(ValueError):a.compare_snapshots(Path(tmp)/'missing1',Path(tmp)/'missing2')
            self.assertEqual(a.load_process(l)[0]['camera'].shape,(60,3))
            (l/'h_r3_p03_meta.json').write_text('{}')
            with self.assertRaises((ValueError,KeyError)):a.load_process(l)

    def test_subset_gate_supports_stop_after_each_new_arm(self):
        traces={'B0':fixture(),'B4':fixture()}
        self.assertTrue(a.matched_prefix(traces,required_arms=('B0','B4'))['equal'])
        traces['B4']['tracker_digest'][1,0,0]^=1
        self.assertFalse(a.matched_prefix(traces,required_arms=('B0','B4'))['equal'])

    def test_actual_refit_ids_and_root_chronology_are_independent_gates(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'process';tr=fixture();write(path,tr)
            meta=json.loads((path/'h_r3_p03_meta.json').read_text())
            self.assertEqual(a.validate_chronology(path,tr,meta)['status'],'PASS')
            events=json.loads((path/'geometry_events.json').read_text())
            events[0]['refit_env_ids']=[1]
            (path/'geometry_events.json').write_text(json.dumps(events))
            with self.assertRaisesRegex(ValueError,'refit selection'):
                a.validate_chronology(path,tr,meta)

    def test_unlisted_output_and_symlink_parent_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'process';write(path,fixture())
            (path/'unlisted.txt').write_text('tamper')
            with self.assertRaisesRegex(ValueError,'unlisted'):
                a.load_process(path)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'process';write(path,fixture())
            (path/'escape').symlink_to(Path(tmp),target_is_directory=True)
            with self.assertRaisesRegex(ValueError,'symlink'):
                a.load_process(path)

    def test_sham_assignment_uses_actual_full_event_episode_decision_key(self):
        tr=fixture()
        self.assertEqual(a.validate_sham_assignment(tr,4801)['status'],'PASS')
        tr['episode_id'][12,0]=1
        # A changed key must be checked even if its sign happens to coincide;
        # force opposite of the independently expected new sign.
        expected=1. if hashlib.sha256(b'HR3-P03|4801|0|1|0|2').digest()[0]&1 else -1.
        tr['sham_sign'][12,0]=tr['sham_side'][12,0]=-expected
        with self.assertRaisesRegex(ValueError,'assignment'):
            a.validate_sham_assignment(tr,4801)

    def test_perprocess_control_gate_rejects_forged_zero_budget_error(self):
        tr=fixture()
        self.assertEqual(a.validate_control_contract(tr,'B0')['status'],'PASS')
        tr['shadow_B5_nominal_xy'][14,0,0]=1.2
        tr['budget_error'][:]=0
        with self.assertRaisesRegex(ValueError,'independent archived nominal'):
            a.validate_control_contract(tr,'B0')


class MetricsTest(unittest.TestCase):
    def test_spatial_identity_terminal_first_and_physical_terminal_action(self):
        tr=fixture();detect(tr,12)
        tr['crashed'][12,0]=tr['reset'][12,0]=True
        tr['intervening'][13:30,0]=False
        tr['captured'][40,0]=True
        result=a.event_outcome(tr,0,10)
        self.assertFalse(result['reobserved'])
        self.assertTrue(result['variants']['p01_literal']['reobserved'])
        self.assertFalse(result['capture_by_w_plus_3s'])
        self.assertEqual(result['control_only']['steps'],3)
        self.assertAlmostEqual(result['control_only']['translation_m'],.3,places=5)
        self.assertEqual(result['control_only']['acceleration_sq_integral_m2_s3'],0)

    def test_actual_velocity_acceleration_stall_and_energy_labels(self):
        tr=fixture();tr['actual_vel'][10:30,0]=0
        result=a.event_outcome(tr,0,10)
        self.assertEqual(result['control_only']['stall_duration_s'],2)
        self.assertAlmostEqual(result['control_only']['acceleration_sq_integral_m2_s3'],200)
        self.assertIn('NOT_RECORDED',result['control_only']['mechanical_control_effort'])
        self.assertEqual(result['tracker_fresh_measurement_delay_s'],.1)

    def test_class_denominators_and_success_latency_remain_descriptive(self):
        traces={arm:fixture() for arm in a.ARMS}
        detect(traces['B3'],12)
        event={'results':{arm:a.event_outcome(tr,0,10) for arm,tr in traces.items()}}
        result=a.summarize([event],False)
        self.assertEqual(result['n'],1)
        self.assertEqual(result['interpretation'],'INVALID_INTEGRITY_FAILURE')
        self.assertEqual(result['contrasts']['B3-B2']['rate_diff'],1)
        self.assertNotIn('pvalue',result['contrasts']['B3-B2'])
        self.assertAlmostEqual(result['arms']['B3']['mean_reobservation_s_among_successes'],.3)

    def test_unequal_effort_and_empty_coverage_do_not_support_pure_viewpoint(self):
        traces={arm:fixture() for arm in a.ARMS}
        traces['B3']['actual_vel'][10:30,0]*=2
        event={'results':{arm:a.event_outcome(tr,0,10) for arm,tr in traces.items()}}
        result=a.effort_balance([event])
        self.assertEqual(result['B3-B5']['pure_viewpoint_attribution'],'INCONCLUSIVE')
        self.assertEqual(a.effort_balance([])['B3-B5']['pure_viewpoint_attribution'],'INCONCLUSIVE')

    def test_archived_common_schedule_exposure_and_invalid_direction_are_independent_gates(self):
        traces={arm:fixture() for arm in a.ARMS}
        for tr in traces.values():
            tr['budget_frozen'][10:30,:2]=True
            tr['decision_index'][10:30,:2]=np.arange(20)[:,None]
        prefix=a.matched_prefix(traces)
        self.assertFalse(a._budget_and_exposure(traces,prefix))
        traces['B4']['schedule_lateral_mps'][15,0]=2
        self.assertTrue(any('frozen archive' in f for f in a._budget_and_exposure(traces,prefix)))
        traces['B4']['schedule_lateral_mps'][15,0]=1
        traces['B4']['direction_valid'][15,0]=False
        traces['B4']['nominal_xy'][15,0]=0
        self.assertFalse(a._budget_and_exposure(traces,prefix))
        traces['B4']['nominal_xy'][15,0,0]=1
        self.assertTrue(any('did not abstain' in f for f in a._budget_and_exposure(traces,prefix)))

    def test_zero_both_effort_is_retained_as_equal_without_causal_claim(self):
        traces={arm:fixture() for arm in a.ARMS}
        for tr in traces.values():
            tr['actual_vel'][10:30,0]=0
            tr['pre_vel'][10:30,0]=0
            tr['actual_pos'][10:30,0]=tr['before_pos'][10:30,0]
        event={'results':{arm:a.event_outcome(tr,0,10) for arm,tr in traces.items()}}
        result=a.effort_balance([event])['B3-B5']
        self.assertEqual(result['metrics']['actual_xy_sq_integral']['both_zero_pairs_retained_as_equal'],1)
        self.assertEqual(result['metrics']['actual_xy_sq_integral']['mean_absolute_symmetric_relative_difference'],0)
        self.assertEqual(result['pure_viewpoint_attribution'],'NOT_ESTABLISHED')


if __name__=='__main__':unittest.main()
