"""No-LiDAR measured-input contracts and actual CPU policy integration."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import camera_policy_runtime as R
import camera_policy_contract as C


def contract():
    # Geometry-only tests do not need local checkpoint files or production imports.
    return {'camera': {'width': 160, 'height': 90,
                       'fx': 80/math.tan(math.radians(33)), 'fy': 45/math.tan(math.radians(27)),
                       'cx': 79.5, 'cy': 44.5, 'translation_m': [.1, 0, .03]},
            'depth': {'minz_m': .4, 'maxz_m': 20., 'window': 5, 'density': .5},
            'observation': {'control_dt_s': .1}}


def sample():
    c = contract()
    f = {'rgb': np.zeros((90, 160, 3), np.uint8), 'depth_z_m': np.full((90, 160), 2., np.float32),
         'invalid': np.zeros((90, 160), bool), 'intrinsics': dict(c['camera']),
         'aligned': True, 'rectified': True, 'mount_flu_m': [.1, 0, .03],
         'camera_to_body_rotation': np.eye(3), 'rgb_t': 1., 'depth_t': 1.}
    s = {'pos': [0., 0., 1.], 'vel': [0., 0., 0.], 'quat_xyzw': [0., 0., 0., 1.],
         'yaw_rate_rps': 0., 't': 1.}
    return c, f, s


class MeasuredProjection(unittest.TestCase):
    def test_level_plane_radial_units_and_colour_without_lidar(self):
        c, f, s = sample()
        f['rgb'][:, :, 0] = np.arange(160)
        # 2026-11-12 fix r1: project_rgbd returns sensor_invalid and no_return separately.
        rgb, depth, invalid, no_return, cosine, report = R.project_rgbd(f, s, c)
        np.testing.assert_array_equal(rgb[0, 0]*255, f['rgb'][:, :, 0])
        np.testing.assert_allclose(depth[0]*cosine, 2., rtol=1e-6)
        self.assertFalse(invalid.any())
        self.assertFalse(no_return.any())
        self.assertFalse(report['synthetic_noise_applied'])
        self.assertNotIn('ranges', f)

    def test_gazebo_crop_preserves_camera_fov_and_ignores_poison_lidar(self):
        c, _, s = sample()
        raw = {'rgb': np.zeros((90, 160, 3), np.uint8), 'depth': np.full((90, 160), 2., np.float32),
               'rgb_t': 1., 'depth_t': 1., 'ranges': object(), 'lidar_t': object()}
        f = R.gazebo_frame(raw)
        rgb, depth, invalid, no_return, cosine, _ = R.project_rgbd(f, s, c)
        self.assertEqual(rgb.shape, (1, 3, 90, 160))
        self.assertFalse(invalid.any())
        self.assertFalse(no_return.any())
        np.testing.assert_allclose(depth[0]*cosine, 2., rtol=1e-6)
        # A simple HFOV=66 square-pixel replacement cannot cover trained VFOV54.
        f['intrinsics']['fx'] = f['intrinsics']['fy'] = c['camera']['fx']
        with self.assertRaisesRegex(R.InputError, 'cover'):
            R.project_rgbd(f, s, c)

    def test_invalid_depth_keeps_independent_mask_and_never_mutates(self):
        c, f, s = sample()
        f['invalid'][20:30, 20:30] = True
        f['depth_z_m'][0, :4] = [0., np.nan, np.inf, .1]
        f['depth_z_m'][1, 1] = 20.  # valid far range is not sensor-invalid
        original = copy.deepcopy(f)
        _, depth, invalid, no_return, _, report = R.project_rgbd(f, s, c)
        # Fix r1 (spec A): the former union mask (104) is split. +inf is optical z > MaxZ,
        # i.e. no-return; 0, NaN and 0.1 < MinZ stay sensor-invalid with the 100 masked.
        self.assertEqual(int(invalid.sum()), 103)
        self.assertEqual(int(no_return.sum()), 1)
        self.assertTrue(no_return[0, 0, 2])
        self.assertTrue(invalid[0, 0, 0] and invalid[0, 0, 1] and invalid[0, 0, 3])
        self.assertFalse((invalid & no_return).any())
        self.assertFalse(invalid[0, 1, 1] or no_return[0, 1, 1])
        self.assertTrue(np.all(depth[invalid | no_return] == 20.))
        self.assertEqual((report['sensor_invalid_pixels'], report['no_return_pixels']), (103, 1))
        np.testing.assert_array_equal(f['depth_z_m'], original['depth_z_m'])
        np.testing.assert_array_equal(f['invalid'], original['invalid'])

    def test_estimator_tilt_uses_measured_z_buffer_leaves_disocclusions_no_return(self):
        c, f, s = sample()
        angle = math.radians(20)
        s['quat_xyzw'] = [math.sin(angle/2), 0., 0., math.cos(angle/2)]
        _, depth, invalid, no_return, _, report = R.project_rgbd(f, s, c)
        self.assertEqual(report['projection'], 'measured-point-z-buffer')
        # Fix r1 (H1): pixels the tilted camera did not image are no-return, not sensor-invalid;
        # this all-valid frame has no sensor-invalid source pixel at all.
        self.assertGreater(int(no_return.sum()), 0)
        self.assertEqual(int(invalid.sum()), 0)
        self.assertGreater(int((~(invalid | no_return)).sum()), 0)
        self.assertTrue(np.isfinite(depth).all())
        self.assertTrue(np.all(depth[no_return] == 20.))

    def test_reject_alignment_geometry_and_oracle_state(self):
        c, f, s = sample()
        changes = [lambda x: x.update(aligned=False), lambda x: x.update(rectified=False),
                   lambda x: x.update(invalid=np.zeros((90, 160), np.uint8)),
                   lambda x: x.update(camera_to_body_rotation=np.zeros((3, 3))),
                   lambda x: x.update(rgb=x['rgb'].astype(np.float32)),
                   lambda x: x['intrinsics'].update(fx=float('nan'))]
        for change in changes:
            with self.subTest(change=change):
                ff = copy.deepcopy(f)
                change(ff)
                with self.assertRaises(R.InputError):
                    R.project_rgbd(ff, s, c)
        for key in ('tpos', 'truth', 'target_position', 'render_pos'):
            with self.subTest(key=key), self.assertRaises(R.InputError):
                R.validate_state(dict(s, **{key: [0., 0., 0.]}))


class InputTiming(unittest.TestCase):
    def runtime(self):
        r = R.CameraPolicyRuntime.__new__(R.CameraPolicyRuntime)
        r.contract, r._last_stamps, r._last_now = contract(), None, None
        return r

    def test_valid_progress_and_rejection_before_history_update(self):
        r = self.runtime()
        _, f, s = sample()
        r._last_stamps, r._last_now = r._validate_timing(f, s, 1.)
        for name, rgb_t, depth_t, st, now in [('duplicate', 1., 1., 1., 1.1),
                    ('stale', .8, .8, .8, 1.1), ('future', 1.2, 1.2, 1.2, 1.1),
                    ('skew', 1.05, 1.1, 1.1, 1.1), ('period', 1.3, 1.3, 1.3, 1.3)]:
            with self.subTest(name=name), self.assertRaises(R.InputError):
                r._validate_timing(dict(f, rgb_t=rgb_t, depth_t=depth_t), dict(s, t=st), now)
        ff, ss = dict(f, rgb_t=1.1, depth_t=1.1), dict(s, t=1.1)
        stamps, now = r._validate_timing(ff, ss, 1.1)
        np.testing.assert_array_equal(stamps, [1.1]*3)
        self.assertEqual(now, 1.1)


INTEGRATION = r'''
import copy, json, math, os, sys
import numpy as np
import tempfile
from pathlib import Path
from camera_policy_contract import build_contract, estimator_height_assertion
from camera_policy_runtime import CameraPolicyRuntime, InputError
# Repair r1 (F6): the default floor filter runs only with the operator's estimator-height assertion (CPU fixture).
evidence_dir=tempfile.TemporaryDirectory()
evidence=Path(evidence_dir.name)/'estimator_height_fixture.txt'
evidence.write_text('CPU test fixture: estimator z is the synthetic height above the floor. Not evidence.')
c=build_contract(int(sys.argv[1]),estimator_height=estimator_height_assertion(evidence))
os.environ['NAVRL_DETECTOR_CHECKPOINT']='unexpected-inherited-detector.pth'
try: CameraPolicyRuntime(c)
except ValueError: pass
else: raise AssertionError('inherited detector silently replaced trained detector')
assert not any(k=='aerial_gym' or k.startswith('aerial_gym.') for k in sys.modules)
del os.environ['NAVRL_DETECTOR_CHECKPOINT']
r=CameraPolicyRuntime(c)
f=dict(rgb=np.zeros((90,160,3),np.uint8),depth_z_m=np.full((90,160),2.,np.float32),
       invalid=np.zeros((90,160),bool), intrinsics=c['camera'], aligned=True,rectified=True,
       mount_flu_m=[.1,0,.03],camera_to_body_rotation=np.eye(3),rgb_t=1.,depth_t=1.)
s=dict(pos=[0,0,1.],vel=[0,0,0.],quat_xyzw=[0,0,0,1.],yaw_rate_rps=0.,t=1.)
first=r.step(f,s,now_s=1.)
assert len(first['observation'])==898 and not first['lidar_visible'] and not first['command_sent']
assert np.isfinite(first['observation']).all()
before=r.perception.step_count
try: r.step(f,s,now_s=1.1)
except InputError: pass
else: raise AssertionError('duplicate frame accepted')
assert r.perception.step_count==before
f['rgb_t']=f['depth_t']=s['t']=1.1
f['rgb'][40:50,75:85]=[255,0,0]
f['invalid'][10:20,10:20]=True
second=r.step(f,s,now_s=1.1)
assert second['depth_near_pixels']>0 and second['camera_visible']
assert not second['lidar_visible'] and not second['command_sent']
assert r.perception.step_count==before+1
assert np.isfinite(second['velocity_world_enu_mps']).all()
assert 'isaacgym' not in sys.modules and 'warp' not in sys.modules
assert not any('navrl_depth_model' in k for k in sys.modules)
action_function=r.audit.deterministic_actions
def broken(*args): raise RuntimeError('injected inference failure')
r.audit.deterministic_actions=broken
f['rgb_t']=f['depth_t']=s['t']=1.2
try: r.step(f,s,now_s=1.2)
except RuntimeError as e: assert str(e)=='injected inference failure'
else: raise AssertionError('failed inference returned a command')
assert r.failed
r.audit.deterministic_actions=action_function
try: r.step(f,s,now_s=1.2)
except RuntimeError as e: assert 'reset required' in str(e)
else: raise AssertionError('poisoned history reused')
from types import SimpleNamespace
from camera_estimator_link import EstimatorLink
from camera_companion import run_measured_control
from sitl_camera import ClockBinding
r.reset()
class Clock:
    def __init__(self): self.t=20.
    def __call__(self): return self.t
    def sleep(self,seconds): self.t+=seconds
class Message(SimpleNamespace):
    def get_srcSystem(self): return 1
    def get_srcComponent(self): return 1
    def get_type(self): return self.kind
clock=Clock()
sends=[]
connection=SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *a:sends.append(a)))
link=EstimatorLink(connection,system_id=1,component_id=1,clock=clock)
def service():
    ms=int(round((clock()-17.)*1000))
    for msg in (Message(kind='LOCAL_POSITION_NED',time_boot_ms=ms,x=0.,y=0.,z=-1.,vx=0.,vy=0.,vz=0.),
                Message(kind='ATTITUDE',time_boot_ms=ms,roll=0.,pitch=0.,yaw=math.pi/2,
                        rollspeed=0.,pitchspeed=0.,yawspeed=0.),
                Message(kind='HEARTBEAT',base_mode=128,custom_mode=6<<16)):
        link.accept(msg)
class Capture:
    def frames(self,*,deadline,service):
        service()
        return dict(f,rgb_t=clock(),depth_t=clock(),now_s=clock())
service()
composed=run_measured_control(r,link,Capture(),max_steps=3,service=service,
            clock_binding=ClockBinding(17.,'fixture-only measured clock'),clock=clock,sleep=clock.sleep)
assert composed['commands_proposed']==3 and composed['commands_published']==0 and sends==[]
assert len(composed['trace'])==3 and not r.failed
def reset_sequence():
    r.reset()
    assert r.perception.step_count==0
    frames=[]
    for i in range(7):
        t=30.+i*.1
        ss=dict(s,t=t,pos=[0.,.02*i,1.])
        ff=dict(f,rgb_t=t,depth_t=t)
        frames.append(r.step(ff,ss,now_s=t)['observation'])
    return np.asarray(frames)
reference=reset_sequence()
np.testing.assert_array_equal(reset_sequence(),reference)
print('CAMERA_CPU_RESULT '+json.dumps(dict(seed=c['seed'],steps=2,obs_dim=898,
       invalid_rejected=True,near_pixels=second['depth_near_pixels'],composed_steps=3,
       reset_sequence_frames=7,reset_bitwise_equal=True,commands_sent=0)))
'''


class ActualPolicyCPU(unittest.TestCase):
    def test_both_trained_seeds_in_fresh_cpu_process_without_simulator(self):
        root = Path(__file__).resolve().parents[1]
        if not Path('/home/fair/motar_tmp/camk2v3_20261108/train_s1/checkpoint').is_dir():
            self.skipTest('local trained camera checkpoints not distributed in public repository')
        env = {k: v for k, v in os.environ.items() if not k.startswith('NAVRL_')}
        env.update(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1',
                   MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', PYTHONPATH=str(root/'tools'))
        accepted = 0
        for seed in (1, 2):
            with self.subTest(seed=seed):
                result = subprocess.run([sys.executable, '-B', '-c', INTEGRATION, str(seed)],
                                        env=env, capture_output=True, text=True, timeout=120, cwd=str(root))
                self.assertEqual(result.returncode, 0, result.stdout[-2000:]+result.stderr[-4000:])
                self.assertIn('CAMERA_CPU_RESULT ', result.stdout)
                accepted += 1
        self.assertEqual(accepted, 2)


if __name__ == '__main__':
    unittest.main()
