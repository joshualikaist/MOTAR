"""C-v2 real production helper versus the committed NumPy oracle; clocks/old modes/firewall."""
import ast
import importlib
import math
from pathlib import Path
import subprocess
import sys
import unittest
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/audits'))
import eq1_common as common
import s_l1c_v2_reference as ref

BASE='aed6affbd2567feee79e658fbe5f07510b039f36'
P='aerial_gym/task/navrl_task/'


def module():
    common.load_production('aerial_gym.task.navrl_task.navrl_perception')
    return importlib.import_module('aerial_gym.task.navrl_task.navrl_lidar_association_v2')


def invoke(mod,scan,prediction,*,camera=False,active=True,dtype=torch.float64,bound=.0001):
    return mod.range_only_lidar_association_v2(torch.tensor(np.asarray(scan)[None],dtype=dtype),
       torch.tensor(ref.v1.ANGLES,dtype=dtype),torch.tensor(ref.v1.ELEVATIONS,dtype=dtype),
       torch.tensor(np.asarray(prediction)[None],dtype=dtype),torch.tensor([camera]),torch.tensor([active]),
       torch.tensor([ref.GATE],dtype=dtype),lidar_far_m=12.,noise_bound_m=bound)


class ModelTest(unittest.TestCase):
    def test_real_helper_matches_independent_positive_and_negative_decisions(self):
        with common.production():
            mod=module()
            for r in (1.16,1.5,2.,2.28,2.3,3.,6.):
                for el in (-5,0,5,10,15,20):
                    for b in (0,20,45,90):
                        target=r*ref.direction(b,el);scan,_=ref.v1.lidar_scan(sphere_center=target)
                        for offset in (-.15,0,.15):
                            prediction=target*(1+offset/r)
                            expected=ref.associate(scan,prediction,'V2-A')
                            out=invoke(mod,scan,prediction)
                            self.assertEqual(bool(out['valid'][0]),expected['valid'],(r,el,b,offset,expected))
                            if expected['valid']:
                                self.assertAlmostEqual(float(out['centre_range'][0]),expected['centre_range'],places=7)
            for r in (1.16,1.5,2.28,3,11.5):
                scan,_=ref.v1.lidar_scan([[r+.3,0,.5]],[[.3,.3,1.5]])
                prediction=np.array([r+.15,0,0])
                expected=ref.associate(scan,prediction,'V2-A')
                self.assertEqual(bool(invoke(mod,scan,prediction)['valid'][0]),expected['valid'])

    def test_float32_near_case_and_camera_active_gates(self):
        with common.production():
            mod=module();target=1.5*ref.direction(0,5);scan,_=ref.v1.lidar_scan(sphere_center=target)
            out=invoke(mod,scan,target,dtype=torch.float32)
            self.assertTrue(bool(out['valid'][0]));self.assertTrue(bool(out['v2_recovered'][0]))
            self.assertAlmostEqual(float(out['centre_range'][0]),1.5,delta=.001)
            self.assertFalse(bool(invoke(mod,scan,target,camera=True)['valid'][0]))
            self.assertFalse(bool(invoke(mod,scan,target,active=False)['valid'][0]))
            self.assertFalse(bool(invoke(mod,scan,target,bound=.03)['valid'][0]))

    def test_v2_cumulative_ab_but_d_disabled(self):
        with common.production(dict(common.receipt_env(),NAVRL_SENSOR_LINEAGE='s_l1c_v2')):
            current,_,_=common.build_perception(1)
            self.assertTrue(current.sensor_lineage_s_l1a and current.sensor_lineage_s_l1b and current.sensor_lineage_s_l1c)
            self.assertTrue(current.sensor_lineage_s_l1c_v2)
            self.assertFalse(current.sensor_lineage_s_l1d)
            self.assertEqual(current.tracker.memory_s,5.)

    def test_gt_firewall_and_unchanged_v1_covariance_functions(self):
        source=(ROOT/P/'navrl_lidar_association_v2.py').read_text()
        tree=ast.parse(source)
        identifiers={n.id.lower() for n in ast.walk(tree) if isinstance(n,ast.Name)}
        self.assertFalse(identifiers & {'target_position','target_mask','segmentation','semantic','provenance'})
        old=ast.parse(subprocess.check_output(['git','show',BASE+':'+P+'navrl_sensor_lineage.py'],text=True,cwd=ROOT))
        new=ast.parse((ROOT/P/'navrl_sensor_lineage.py').read_text())
        for name in ('range_only_lidar_association','range_only_covariance','measurement_validity'):
            a=next(x for x in old.body if isinstance(x,ast.FunctionDef) and x.name==name)
            b=next(x for x in new.body if isinstance(x,ast.FunctionDef) and x.name==name)
            self.assertEqual(ast.dump(a),ast.dump(b),name)

    def test_old_modes_actual_observe_equal_base(self):
        original=subprocess.check_output(['git','show',BASE+':'+P+'navrl_perception.py'],text=True,cwd=ROOT)
        target=np.array([3.,0,0]);scan,_=ref.v1.lidar_scan(sphere_center=target)
        for mode in ('off','s_l1a_v1','s_l1b_v1','s_l1c_v1','s_l1d_v1'):
            with self.subTest(mode=mode),common.production(dict(common.receipt_env(),NAVRL_SENSOR_LINEAGE=mode)),torch.random.fork_rng(devices=[]):
                torch.manual_seed(10);current,cfg,_=common.build_perception(1)
                namespace={'__name__':'_cv2_frozen_perception'};exec(compile(original,'frozen_perception','exec'),namespace)
                torch.manual_seed(10);old=namespace['NavRLPerceptionModule'](1,torch.device('cpu'),cfg.perception,.1,cfg.vision)
                for obj in (current,old):
                    obj.tracker.state[0,:3]=torch.tensor(target);obj.tracker.cov[0]=torch.eye(6)*.01
                    obj.tracker.active[0]=True;obj.tracker.age[0]=1.
                args=(torch.zeros(1,3,90,160),torch.full((1,90,160),10.),torch.tensor(scan,dtype=torch.float32).view(1,288),
                      torch.zeros(1,3),torch.zeros(1,3),torch.tensor([[0.,0.,0.,1.]]),torch.zeros(1),torch.zeros(1,4))
                a,_=current.observe(*args,cfg.max_velocity,cfg.flight_altitude,training=False)
                b,_=old.observe(*args,cfg.max_velocity,cfg.flight_altitude,training=False)
                self.assertTrue(torch.equal(a,b));self.assertTrue(torch.equal(current.tracker.state,old.tracker.state))
                self.assertTrue(torch.equal(current.tracker.cov,old.tracker.cov))

    def test_clock_covariance_and_expiry_on_real_v2(self):
        with common.production(dict(common.receipt_env(),NAVRL_SENSOR_LINEAGE='s_l1c_v2')):
            current,cfg,_=common.build_perception(1)
            target=1.5*ref.direction(0,5);scan,_=ref.v1.lidar_scan(sphere_center=target)
            tr=current.tracker;tr.state[0,:3]=torch.tensor(target);tr.cov[0]=torch.eye(6)
            tr.active[0]=True;tr.age[0]=1.
            before=tr.cov[0,:3,:3].double().numpy().copy()
            current._associate_lidar_target(torch.tensor(scan,dtype=torch.float32).view(1,288),torch.zeros(1,3),
                torch.tensor([[0.,0.,0.,1.]]),torch.zeros(1,dtype=torch.bool))
            after=tr.cov[0,:3,:3].double().numpy();u,t1,t2=ref.v1.orthonormal_basis(target)
            self.assertGreaterEqual(t1@after@t1/(t1@before@t1),.99)
            self.assertGreaterEqual(t2@after@t2/(t2@before@t2),.99);self.assertLess(u@after@u,u@before@u)
            def observe(lidar):
                return current.observe(torch.zeros(1,3,90,160),torch.full((1,90,160),10.),lidar,
                    torch.zeros(1,3),torch.zeros(1,3),torch.tensor([[0.,0.,0.,1.]]),torch.zeros(1),
                    torch.zeros(1,4),cfg.max_velocity,cfg.flight_altitude,training=False)
            current._s_l1c_identity_age[:]=0.
            lidar=torch.tensor(scan,dtype=torch.float32).view(1,288)
            for _ in range(10):obs,diag=observe(lidar)
            self.assertEqual(obs.shape[1],898)
            self.assertAlmostEqual(float(current._s_l1c_identity_age[0]),1.,places=5)
            self.assertEqual(float(tr.age[0]),0.);self.assertEqual(int(current._s_l1c_update_source[0]),2)
            for _ in range(50):observe(torch.full((1,288),12.))
            self.assertTrue(bool(tr.active[0]));observe(torch.full((1,288),12.));self.assertFalse(bool(tr.active[0]))

if __name__=='__main__':unittest.main()
