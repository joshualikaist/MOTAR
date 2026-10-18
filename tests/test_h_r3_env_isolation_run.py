import sys,json,subprocess,unittest,tempfile,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
import h_r3_env_isolation_run as run
class QualificationRunTests(unittest.TestCase):
 def test_bounded_authorized_matrix(self):
  p=run.a.read_json(run.PLAN);s=run.specs(p)
  self.assertEqual(len(s),5);self.assertEqual({x['seed']for x in s},{4602});self.assertEqual({x['num_envs']for x in s},{128});self.assertEqual({x['steps']for x in s},{40})
  self.assertEqual(p['run']['forced_step'],2);self.assertEqual(p['run']['forced_env'],127)
  self.assertTrue(all(x['config']=='moving'and x['lineage']=='off'for x in s));self.assertEqual(sum(x['force_reset']for x in s),2)
 def test_old_runtime_sources_frozen(self):
  for p in ['tools/h_r3_p02_rng.py','tools/h_r3_p02_launcher.py','tools/h_r3_p02_analysis.py','aerial_gym/env_manager/warp_env_manager.py','aerial_gym/env_manager/IGE_env_manager.py','aerial_gym/task/navrl_task/navrl_task.py']:
   self.assertEqual((ROOT/p).read_bytes(),subprocess.check_output(['git','show','e6814a6:'+p],cwd=ROOT),p)
 def test_bound_completion_rejects_tampering(self):
  p=run.a.read_json(run.PLAN);spec=run.specs(p)[0]
  with tempfile.TemporaryDirectory()as d:
   out=Path(d);m=dict(spec,actor_obs_dim=898,model_training=False,step_dt=.1,rms_digest_start='r',rms_digest_end='r',policy_f_sha256='c',source_commit='h',origins={k:str(ROOT/k/'__init__.py')for k in ('aerial_gym','motar_eval')})
   data=json.dumps(m).encode();(out/'process_meta.json').write_bytes(data);done=dict(process=spec['id'],actor_obs_dim=898,nan_fields=[],files=[dict(file='process_meta.json',bytes=len(data),sha256=hashlib.sha256(data).hexdigest())]);(out/'process_done.json').write_text(json.dumps(done))
   (out/'frames').mkdir();(out/'frames/a.bin').write_bytes(b'raw');done['files'].append(dict(file='frames/a.bin',bytes=3,sha256=hashlib.sha256(b'raw').hexdigest()));(out/'process_done.json').write_text(json.dumps(done))
   checks=dict(rms_digest='r',policy_f_sha256='c',head='h');run.validate(out,spec,checks)
   (out/'frames/a.bin').write_bytes(b'bad')
   with self.assertRaises(ValueError):run.validate(out,spec,checks)
 def test_unsafe_nested_completion_rejected(self):
  with tempfile.TemporaryDirectory()as d:
   out=Path(d);spec=run.specs(run.a.read_json(run.PLAN))[0]
   for name in ('../escape','/tmp/escape','frames/../../escape'):
    (out/'process_done.json').write_text(json.dumps(dict(process=spec['id'],nan_fields=[],files=[dict(file=name,bytes=0,sha256='0')])) )
    with self.assertRaises(ValueError):run.validate(out,spec,{})
if __name__=='__main__':unittest.main()
