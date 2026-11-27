"""Focused mechanics-analysis failure gates; fixture data are not simulator evidence."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('isolation_analysis',ROOT/'tools/h_r3_env_isolation_analysis.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def frame(n=128):
    f={k:np.zeros((n,1),np.float32) for k in a.NUMERIC}
    f.update(rgb=np.zeros((n,3,1,1),np.float32),depth=np.zeros((n,1,1),np.float32),lidar=np.zeros((n,1),np.float32),
             actor_obs=np.zeros((n,898),np.float32),root_state=np.zeros((n,1,13),np.float32),vertices=np.zeros((n,3),np.float32),
             vertex_env_index=np.arange(n,dtype=np.int64),vertex_actor_index=np.arange(n,dtype=np.int64),
             vertex_ranges=np.stack([np.arange(n),np.arange(1,n+1)],axis=1).astype(np.int64),mesh_ids=np.arange(1,n+1,dtype=np.uint64),
             reset=np.zeros(n,bool),natural_reset=np.zeros(n,bool))
    refresh(f)
    return f


def refresh(f):
    n=len(f['actor_obs'])
    for k in ('rgb','depth','lidar'):
        f[k+'_sha']=np.stack([a._hash(f[k][i]) for i in range(n)])
    f['state_sha']=np.stack([a._hash(f['actor_obs'][i]) for i in range(n)])
    f['vertices_sha']=np.stack([a._hash(f['vertices'][lo:hi]) for lo,hi in f['vertex_ranges']])


def write_fixture(root,small=False):
    (root/'cpu_report.json').write_text(json.dumps({'status':'PASS','tests':11}))
    for p,(mode,forced,evaluator) in a.PROCESSES.items():
        d=root/p;(d/'frames').mkdir(parents=True)
        (d/'process_meta.json').write_text(json.dumps({'id':p,'mode':mode,'force_reset':forced,'evaluator':evaluator,'seed':4602,
            'num_envs':128,'steps':40,'actor_obs_dim':898,'rms_digest_start':'frozen','rms_digest_end':'frozen','source_commit':'abc'}))
        frames=[];events=[];writes=[]
        for step in range(-1,40):
            f=frame(2 if small else 128);n=len(f['actor_obs']);start=len(events);wstart=len(writes);injected=forced and step==2 and n==128
            if step==-1:
                # Init is also an actual all-environment geometry/root reset transaction.
                f['reset'][:]=True
                ids=list(range(n))
                events.append({'step':step,'env_ids':ids,'write_vertex_ranges':f['vertex_ranges'].tolist(),'refit_env_ids':ids,
                  'refit_mesh_ids':f['mesh_ids'].tolist(),'vertices':[{'env':i,'changed_vertices':0} for i in ids]})
                writes.append({'step':step,'kind':'indexed_flush','env_ids':ids})
            if forced and step>=2 and n==128:
                f['vertices'][127,0]=1;f['root_state'][127,0,0]=1
                if mode=='legacy_audit':f['vertices'][0,0]=.01;f['rgb'][0]=.01;f['depth'][0]=.01;f['lidar'][0]=.01
            if injected:
                f['reset'][127]=True
                ids=[127]
                events.append({'step':step,'env_ids':ids,'write_vertex_ranges':f['vertex_ranges'][ids if mode=='selected' else np.arange(n)].tolist(),
                  'refit_env_ids':ids,'refit_mesh_ids':f['mesh_ids'][ids].tolist(),
                  'vertices':[{'env':i,'changed_vertices':int(i==127 or (mode=='legacy_audit' and i==0))} for i in range(n)]})
                writes.append({'step':step,'kind':'indexed_flush','env_ids':ids})
            refresh(f)
            np.savez(d/'frames'/('frame_init.npz' if step<0 else 'frame%04d.npz'%step),**f)
            frames.append({'step':step,'reset_ids':np.flatnonzero(f['reset']).tolist(),'natural_reset_ids':[],
                'forced_injected':injected,'warp_event_start':start,'warp_event_end':len(events),'root_write_event_start':wstart,'root_write_event_end':len(writes)})
        (d/'frame_meta.json').write_text(json.dumps(frames));(d/'geometry_events.json').write_text(json.dumps(events));(d/'root_write_events.json').write_text(json.dumps(writes))


class AnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name);write_fixture(cls.root)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def mutate(self,relative,edit):
        p=self.root/relative;original=p.read_bytes();f=a._load(p);edit(f);np.savez(p,**f)
        self.addCleanup(p.write_bytes,original)
    def test_controlled_fixture_qualifies_mechanics_not_historical_causality(self):
        r=a.analyse(self.root);self.assertEqual(r['verdict'],'GO_MECHANICS_QUALIFIED',r)
        self.assertEqual(r['comparisons']['fixed_isolation']['compared_envs'],127)
        self.assertEqual(r['comparisons']['evaluator_noninterference']['compared_envs'],128)
        self.assertFalse(r['causality']['historical_P02_numerical_cause_proven'])
    def test_unselected_sensor_failure_no_exclusion(self):
        def edit(f):f['rgb'][1]=1;refresh(f)
        self.mutate('fixed_forced/frames/frame0003.npz',edit)
        r=a.analyse(self.root);self.assertEqual(r['verdict'],'NO_GO');self.assertIn('rgb',r['comparisons']['fixed_isolation']['first_mismatch'])
    def test_evaluator_off_on_checks_selected_127_also(self):
        def edit(f):f['actor_obs'][127,0]=1;refresh(f)
        self.mutate('fixed_noeval/frames/frame0003.npz',edit)
        self.assertEqual(a.analyse(self.root)['verdict'],'NO_GO')
    def test_selected_reset_semantics_not_ignored(self):
        def edit(f):f['root_state'][127,0,0]=2
        self.mutate('fixed_forced/frames/frame0002.npz',edit)
        r=a.analyse(self.root);self.assertEqual(r['verdict'],'NO_GO');self.assertFalse(r['selected_reset_parity']['PASS'])
    def test_natural_reset_overlap_invalid(self):
        def edit(f):f['natural_reset'][0]=True
        self.mutate('fixed_normal/frames/frame0002.npz',edit)
        self.assertNotEqual(a.analyse(self.root)['verdict'],'GO_MECHANICS_QUALIFIED')
    def test_wrong_vertex_actor_map_rejected(self):
        def edit(f):f['vertex_actor_index'][0]=1
        self.mutate('fixed_normal/frames/frame0001.npz',edit)
        self.assertEqual(a.analyse(self.root)['verdict'],'INCOMPLETE')
    def test_numeric_hash_disagreement_rejected(self):
        def edit(f):f['depth'][0]=1
        self.mutate('fixed_normal/frames/frame0001.npz',edit)
        self.assertEqual(a.analyse(self.root)['verdict'],'INCOMPLETE')
    def test_cpu_not_pass_blocks_go(self):
        p=self.root/'cpu_report.json';old=p.read_bytes();self.addCleanup(p.write_bytes,old);p.write_text('{"status":"FAIL"}')
        self.assertEqual(a.analyse(self.root)['verdict'],'NO_GO')
    def test_small_mock_cannot_qualify_actual_n128(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);write_fixture(root,small=True)
            self.assertEqual(a.analyse(root)['verdict'],'INCOMPLETE')
    def test_bitwise_equality_detects_signed_zero(self):
        self.assertFalse(a._same(np.array([0.],np.float32),np.array([-0.],np.float32)))

if __name__=='__main__':unittest.main()
