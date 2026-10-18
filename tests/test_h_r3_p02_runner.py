"""CPU contracts for the P02 process matrix, unchanged historical paths and fail-closed outputs."""
import hashlib,json,subprocess,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
import h_r3_p02_run as run

class RunnerContractTest(unittest.TestCase):
    def test_only_two_prespecified_seeds_and_six_arms(self):
        p=run.load_plan();self.assertEqual(p['run']['seeds'],[4601,4602]);self.assertTrue(p['run']['no_adaptive_seed_expansion'])
        specs=run.process_specs(p,p['run']['seeds'],128,600)
        self.assertEqual(len(specs),12);self.assertEqual({x['hr3_arm'] for x in specs},{'B0','B1','B2','B3','B4','B5'})
        self.assertTrue(all(x['lineage']=='off' and x['num_envs']==128 and x['steps']==600 for x in specs))

    def test_frozen_p01_and_production_files_unchanged(self):
        paths=['motar_eval/h_r3_arms.py','tools/h_r3_run.py','tools/h_r3_launcher.py','tools/h_r3_analysis.py',
               'docs/plans/h_r3_pilot_v1.json','docs/plans/h_r3_pilot_v1.md',
               'aerial_gym/task/navrl_task/navrl_task.py','aerial_gym/task/navrl_task/navrl_perception.py',
               'motar_eval/e_l0.py']
        for p in paths:self.assertEqual((ROOT/p).read_bytes(),subprocess.check_output(['git','show','fbee900b:'+p],cwd=ROOT),p)

    def fixture(self,root):
        spec=run.process_specs(run.load_plan(),[4601],16,180)[0]
        meta=dict(arm='B0',seed=4601,num_envs=16,steps=180,sensor_lineage='off',step_dt=.1,rms_digest_start='x',rms_digest_end='x',
                  model_training_flag=False,vision_mode=True,origins={k:str(ROOT/k/'__init__.py') for k in ('aerial_gym','motar_eval')})
        return spec,meta,dict(rms_digest='x')

    def write(self,root,spec,meta,dim=898):
        p=Path(root);data=(json.dumps(meta)+'\n').encode();(p/'h_r3_p02_meta.json').write_bytes(data)
        done=dict(process=spec['id'],nan_fields=[],actor_obs_dim=dim,files=[dict(file='h_r3_p02_meta.json',bytes=len(data),sha256=hashlib.sha256(data).hexdigest())])
        (p/'process_done.json').write_text(json.dumps(done))

    def test_bound_completion_and_frozen_actor_pass(self):
        with tempfile.TemporaryDirectory() as t:
            spec,m,c=self.fixture(t);self.write(t,spec,m);self.assertEqual(run.validate(Path(t),spec,c)['actor_obs_dim'],898)

    def test_wrong_lineage_actor_seed_or_rms_fails(self):
        for key,val in [('sensor_lineage','s_l1c_v2'),('seed',4602),('rms_digest_end','changed')]:
            with tempfile.TemporaryDirectory() as t:
                spec,m,c=self.fixture(t);m[key]=val;self.write(t,spec,m)
                with self.assertRaises(ValueError):run.validate(Path(t),spec,c)
        with tempfile.TemporaryDirectory() as t:
            spec,m,c=self.fixture(t);self.write(t,spec,m,906)
            with self.assertRaises(ValueError):run.validate(Path(t),spec,c)

    def test_tampered_bound_output_fails(self):
        with tempfile.TemporaryDirectory() as t:
            spec,m,c=self.fixture(t);self.write(t,spec,m);(Path(t)/'h_r3_p02_meta.json').write_text('{}')
            with self.assertRaises(ValueError):run.validate(Path(t),spec,c)

if __name__=='__main__':unittest.main()
