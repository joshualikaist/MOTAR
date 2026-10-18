"""Focused resource/stop/boundary contracts; imports do not build a simulator."""
import ast,json,subprocess,sys,tempfile,types,unittest
from pathlib import Path
from unittest import mock
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
import h_r3_p03_run as run

class RunnerTests(unittest.TestCase):
 def test_bounded_matrix_no_full_pilot(self):
  p=run.a.read_json(run.PLAN);ss=run.specs(p)
  self.assertEqual(len(ss),11);self.assertEqual([x['steps']for x in ss[:2]],[16,16]);self.assertTrue(all(x['steps']==600 for x in ss[2:]));self.assertEqual({x['seed']for x in ss},{4801});self.assertEqual({x['num_envs']for x in ss},{128});self.assertEqual(p['run']['total_wall_cap_s'],3600);self.assertEqual(p['run']['disk_min_free_gb'],8);self.assertEqual(p['run']['oom_reserve_mib'],512)
  self.assertEqual({x['hr3_arm']for x in ss if x['id'].startswith('long_B')},{'B0','B1','B2','B3','B4','B5'})
 def test_natural_reset_blindspot_rejected(self):
  import numpy as np
  tr={k:np.zeros((16,128),bool)for k in('natural_reset','reset')};spec=dict(id='short_normal')
  self.assertTrue(run.next_gate(Path('/unused'),spec,tr)['PASS']);tr['natural_reset'][2,56]=True
  with self.assertRaises(ValueError):run.next_gate(Path('/unused'),spec,tr)
 def test_failure_stops_no_second_gpu_launch(self):
  p=run.a.read_json(run.PLAN)
  checks=dict(head='head',policy_f_sha256=p['policy_f_sha256'])
  with tempfile.TemporaryDirectory()as tmp:
   out=Path(tmp)/'new'
   with mock.patch.object(run.base,'git',return_value=types.SimpleNamespace(stdout='')),mock.patch.object(run.a,'runtime_guard',return_value=(None,{})),mock.patch.object(run.base,'gpu_state',return_value=dict(name='RTX 3070',total_mib=8192,used_mib=500)),mock.patch.object(run.a,'supervised_launch',return_value=(3,.01,500,None))as launch,mock.patch.object(run.b,'process_env',return_value={}),mock.patch.object(run.base,'free_gb',return_value=20),mock.patch.object(run.base,'dir_bytes',return_value=0),mock.patch.object(run.base,'sha256_file',return_value=p['policy_f_sha256']),mock.patch.object(run.base,'write_manifest',return_value=0):
    result=run.run(p,out,checks)
   self.assertEqual(launch.call_count,1);self.assertEqual(result['status'],'STOPPED_INTEGRITY');self.assertIn('NONZERO_EXIT:short_normal',result['stop_reason']);self.assertEqual(len(result['processes']),1)
 def test_binding_change_rejected(self):
  ss=run.specs(run.a.read_json(run.PLAN));spec=ss[0];checks=dict(head='source',policy_f_sha256='checkpoint',rms_digest='rms')
  meta=dict(seed=4801,num_envs=128,steps=16,arm='B0',isolation_mode='runtime',force_reset=False,identity_enabled=True,source_commit='source',policy_f_sha256='checkpoint',rms_digest_start='rms',rms_digest_end='rms',origins={k:str(ROOT/k/'__init__.py')for k in('aerial_gym','motar_eval')},geometry_integrity=dict(status='PASS'))
  with mock.patch.object(run.analysis,'load_process',return_value=({},meta)):
   run.validate(Path('/unused'),spec,checks);meta['source_commit']='changed'
   with self.assertRaises(ValueError):run.validate(Path('/unused'),spec,checks)
 def test_online_control_inputs_exclude_evaluation_gt(self):
  tree=ast.parse((ROOT/'tools/h_r3_p03_launcher.py').read_text());nodes=[n for n in ast.walk(tree)if isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='inputs'for t in n.targets)]
  self.assertEqual(len(nodes),1);fields={n.attr for n in ast.walk(nodes[0].value)if isinstance(n,ast.Attribute)}
  self.assertNotIn('target_position',fields);self.assertNotIn('target_vel_w',fields);self.assertNotIn('silhouette_from_task',fields);self.assertIn('state',fields);self.assertIn('_lidar_distance_m',fields)
  self.assertNotIn('geometry',ast.dump(nodes[0].value));self.assertNotIn('identity',ast.dump(nodes[0].value))
 def test_original_sources_unchanged(self):
  for name in ('tools/h_r3_env_isolation.py','tools/h_r3_p02_rng.py','motar_eval/h_r3_p02_arms.py','motar_eval/h_r3_p02_identity.py','aerial_gym/env_manager/warp_env_manager.py','aerial_gym/task/navrl_task/navrl_task.py'):
   self.assertEqual((ROOT/name).read_bytes(),subprocess.check_output(['git','show','ce3d10a:'+name],cwd=ROOT))
if __name__=='__main__':unittest.main()
