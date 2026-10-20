"""Independent CPU fixtures for the real call-through observer and phase protocol."""
from dataclasses import fields, replace, FrozenInstanceError
import json
from pathlib import Path
import random
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval import boia_shadow as s


def values():
    tensors = [torch.zeros(shape, dtype=torch.float32) for shape in s.SHAPES]
    v = s.SensorValues(*tensors)
    for name in s.BOOL_FIELDS:
        object.__setattr__(v, name, torch.ones(1, dtype=torch.bool))
    v.depth.fill_(10); v.lidar.fill_(12); v.raw_lidar.fill_(1)
    v.vehicle_quat[:, 3] = 1; v.body_quat[:, 3] = 1
    v.covariance[:] = torch.eye(6)[None]
    v.position[:, 2] = 1; v.confidence.fill_(.8)
    return v


BIND = s.Binding("a" * 40, "b" * 64, "c" * 64)


def packet(episode=0, action=0, generation=0, decision=0, v=None, **kw):
    stamp = s.SensorTime(decision * .1, decision * .1, decision * .1, 10., "DECLARED_SIM_SCHEDULE")
    return s.make_packet(s.JoinKey(0, episode, action, generation, decision), BIND, stamp, stamp,
                         v or values(), ("YAW_ONLY", 84., 80., .1, 0., .03, "EUCLIDEAN_RAY_RANGE_M"), **kw)


def truth(key, **kw):
    data = dict(capture=False, crash=False, timeout=False, contact_now=False, height_now=False,
                oob_now=False, contact_substeps=0, contact_peak_n=0., substeps=10, threshold_n=.05,
                state_sha="d" * 64, reset_expected=False)
    data.update(kw)
    if any(data[x] for x in ("capture", "crash", "timeout")):
        data["reset_expected"] = True
    return s.TruthRecord(key, **data)


class FakePerception:
    def __init__(self):
        v = values()
        self.tracker = NS(state=v.mean.clone(), cov=v.covariance.clone(), age=v.age.clone(), active=v.active.clone())
        self._lidar_angles = torch.pi - torch.arange(72) * (2 * torch.pi / 72)
        self.hfov, self.vfov, self.fx, self.fy, self.cx, self.cy = 1.5, 1., 84., 80., 79.5, 44.5
        self.camera_offset = torch.tensor([[.1, 0., .03]])
        self.target_radius, self.max_camera_range, self.lidar_max_range, self.min_pixels = .15, 20., 12., 2
        self.robot_history, self.target_history, self.obstacle_history = torch.zeros(1, 5, 10), torch.zeros(1, 5, 16), torch.zeros(1, 5, 8, 12)
        self.calls, self.step_count = 0, 0

    def observe(self, rgb, depth, lidar_m, drone_pos_w, drone_vel_w, vehicle_quat,
                yaw_rate, previous_action, max_velocity, flight_altitude, training, env_bounds_min, env_bounds_max):
        self.calls += 1
        self.step_count += 1
        self.tracker.age += .1
        self.robot_history[:, 0, :3] = drone_pos_w
        cam = torch.tensor([self.calls % 2 == 1])
        actor = torch.full((1, 898), float(self.calls))
        self.last_result = (actor, dict(camera_visible=cam, visible=cam | self.tracker.active,
                                       confidence=torch.tensor([.8])))
        return self.last_result


class FakeSim:
    def __init__(self, task):
        self.task = task
        self.physics_step_callback = None
        self.cfg = NS(env=NS(collision_force_threshold=.05))
        self.robot_manager = NS(warp_sensor=NS(sensor=NS(capture=self.capture)))
        self.capture_count = 0

    def capture(self):
        self.capture_count += 1
        return self.task.obs_dict["depth_range_pixels"]

    def set_physics_step_callback(self, x):
        self.physics_step_callback = x

    def post_reward_calculation_step(self):
        if self.task.captured_now.item() or self.task.crashed_now.item() or self.task.truncations.item():
            ids = torch.tensor([0])
            self.task.obs_dict["robot_position"][:] = torch.tensor([[0., 0., 1.]])
            self.task.obs_dict["crashes"].zero_()
            if self.task.scenario.get("reset_mutates_lidar"):
                self.task.obs_dict["depth_range_pixels"].fill_(.5)
        else:
            ids = torch.empty(0, dtype=torch.long)
        self.robot_manager.warp_sensor.sensor.capture()
        return ids


class FakeTask:
    """Mimics audited order including in-place termination alias and delayed camera flag."""
    def __init__(self, scenario=None):
        v = values()
        self.num_envs, self.device = 1, "cpu"
        self.obs_dict = dict(robot_position=v.position.clone(), robot_linvel=v.velocity.clone(),
            robot_vehicle_orientation=v.vehicle_quat.clone(), robot_orientation=v.body_quat.clone(),
            robot_body_angvel=v.body_angvel.clone(), depth_range_pixels=v.raw_lidar.clone(),
            robot_contact_force_tensor=torch.zeros(1, 3), crashes=torch.zeros(1, dtype=torch.bool),
            env_bounds_min=torch.zeros(1, 3), env_bounds_max=torch.ones(1, 3)*40,
            navrl_target_position=torch.ones(1, 3), segmentation_pixels=torch.full((1, 1, 4, 72), 50))
        self.obs_dict['obstacle_position']=torch.zeros(1,4,3)
        self.obs_dict['obstacle_orientation']=torch.tensor([[[0.,0.,0.,1.]]]).repeat(1,4,1)
        self.perception = FakePerception()
        self._camera_visible_now = torch.zeros(1, dtype=torch.bool)
        self._visible_now = self._camera_visible_now.clone()
        self.prev_action, self.command = torch.zeros(1, 4), torch.zeros(1, 4)
        self.task_obs = dict(observations=torch.zeros(1, 898), states=torch.ones(1, 906))
        self.task_config = NS(lower_height_bound=.1, upper_height_bound=4., max_velocity=2.5, flight_altitude=1.)
        self.vis_cfg = NS(oob_margin=1.)
        self._kroc_adapter = None
        self.target_position, self.target_vel_w = torch.ones(1, 3)*6, torch.ones(1, 3)
        self.sim_env = FakeSim(self)
        self.truncations = torch.zeros(1, dtype=torch.bool)
        self.captured_now, self.crashed_now = torch.zeros(1, dtype=torch.bool), torch.zeros(1, dtype=torch.bool)
        self.steps, self.transforms, self.computes, self.resets, self.integral = 0, 0, 0, 0, 0.
        self.scenario = scenario or {}
        self.detector=NS(render_raw_rgbd=lambda *a,**kw:(torch.zeros(1,3,90,160),torch.ones(1,90,160)*10))

    def _lidar_distance_m(self):
        return torch.nan_to_num(self.obs_dict["depth_range_pixels"].reshape(1, 288), nan=1.).clamp(0, 1)*12

    def reset_idx(self, ids):
        self.resets += 1
        self.perception.tracker.age.zero_()

    def observe_current(self):
        p, od = self.perception, self.obs_dict
        rgb,depth=self.detector.render_raw_rgbd(od['robot_position'],od['robot_vehicle_orientation'],self.target_position)
        result = p.observe(rgb=rgb, depth=depth,
            lidar_m=self._lidar_distance_m(), drone_pos_w=od["robot_position"], drone_vel_w=od["robot_linvel"],
            vehicle_quat=od["robot_vehicle_orientation"], yaw_rate=od["robot_body_angvel"][:, 2],
            previous_action=self.prev_action, max_velocity=2.5, flight_altitude=1., training=False,
            env_bounds_min=od["env_bounds_min"], env_bounds_max=od["env_bounds_max"])
        self.task_obs["observations"][:] = result[0]
        self._camera_visible_now[:] = result[1]["camera_visible"]
        self._visible_now[:] = result[1]["visible"]
        return self.task_obs["observations"].clone()

    def reset(self):
        self.reset_idx(torch.tensor([0]))
        self.sim_env.robot_manager.warp_sensor.sensor.capture()
        return self.observe_current()

    def transform_action_to_command(self, action):
        self.transforms += 1
        self.integral += .1
        self.command[:] = action * 2.5
        self.command[:, 2] = self.integral
        return self.command

    def compute_state_reward_and_terminations(self):
        self.computes += 1
        self.captured_now[:] = self.scenario.get("capture", False)
        raw = self.obs_dict["crashes"].bool().clone()
        z = self.obs_dict["robot_position"][:, 2]
        raw |= (z < .1) | (z > 4)
        pos = self.obs_dict["robot_position"][:, :2]
        raw |= ((pos < -1) | (pos > 41)).any(1)
        self.crashed_now[:] = raw & ~self.captured_now
        self.obs_dict["crashes"][:] = self.crashed_now | self.captured_now  # actual alias hazard

    def step(self, action):
        self.transform_action_to_command(action)
        self.prev_action[:] = action
        cb = self.sim_env.physics_step_callback
        for substep in range(10):
            if cb is not None: cb()
            force = .1 if substep == self.scenario.get("contact_substep", -1) else 0.
            self.obs_dict["robot_contact_force_tensor"][:, 0] = force
            self.obs_dict["crashes"] |= torch.tensor([force > .05])
            if cb is not None and hasattr(cb, "post_physics_step"): cb.post_physics_step()
        self.obs_dict["robot_position"][:, 0] += .1
        if self.scenario.get("height"): self.obs_dict["robot_position"][:, 2] = -.1
        if self.scenario.get("oob"): self.obs_dict["robot_position"][:, 0] = 42.
        self.compute_state_reward_and_terminations()
        self.truncations[:] = self.scenario.get("timeout", False)
        ids = self.sim_env.post_reward_calculation_step()
        if len(ids):
            self.reset_idx(ids)
            self.sim_env.robot_manager.warp_sensor.sensor.capture()
        self.steps += 1
        if self.scenario.get('extra_render'):self.detector.render_raw_rgbd()
        return self.observe_current(), torch.tensor([0.]), torch.tensor([len(ids) > 0]), {}


def exercise(root, shadow=True, steps=3, scenario=None, reference=None, observed=True):
    t = FakeTask(scenario)
    policy_calls = 0
    log = s.EventLogger(root)
    r = s.Recorder(log, shadow, reference=reference)
    tap = s.EvaluatorTap(t, r, BIND)
    cm = tap if observed else nullcontext()
    try:
        with cm:
            obs = t.reset()
            for k in range(steps):
                tap.actor_rng_before = s.rng_fingerprint()
                action = torch.tanh(obs[:, :4] * .01)
                policy_calls += 1
                obs, _, done, _ = t.step(action)
                if bool(done.item()): break
            if observed: r.censor("PROCESS_ACTION_CAP")
    finally:
        log.close()
    return t, r, policy_calls


from contextlib import nullcontext


class PacketTests(unittest.TestCase):
    def test_detached_immutable_byte_copy(self):
        v = values(); v.position.requires_grad_(True); p = packet(v=v); before = p.fingerprint()
        with torch.no_grad(): v.position.fill_(99)
        self.assertEqual(p.fingerprint(), before)
        self.assertIs(type(p.tensor("position").data), bytes)
        with self.assertRaises(ValueError): p.tensor("position").array()[0, 0] = 1
        with self.assertRaises(FrozenInstanceError): p.key = None

    def test_shape_units_and_flags_reject(self):
        for name, val in (("depth", torch.zeros(1, 160, 90)), ("camera", torch.ones(1)),
                          ("lidar", torch.ones(1, 288)*13), ("depth", -torch.ones(1, 90, 160)),
                          ("depth",torch.ones(1,90,160,dtype=torch.long)),("camera",FakeTask()),
                          ("age", torch.tensor([-1.])), ("rgb", torch.ones(1, 3, 90, 160)*2)):
            with self.subTest(name=name), self.assertRaises(s.IntegrityError): packet(v=replace(values(), **{name:val}))

    def test_unknown_fields_and_object_handles_rejected(self):
        with self.assertRaises(TypeError): s.SensorValues(**dict(values().__dict__, target_gt=torch.zeros(1, 3)))
        p = packet()
        with self.assertRaises(s.IntegrityError): replace(p, values=p.values + (("semantic50", p.values[0][1]),))
        with self.assertRaises(s.IntegrityError): replace(p, calibration=(FakeTask(),))
        with self.assertRaises(s.IntegrityError): s.diagnose(dict(packet=p, evaluator=FakeTask()))
        with self.assertRaises(s.IntegrityError): s.diagnose(truth(p.key))

    def test_source_width_and_lineage(self):
        for binding in (("a"*64,"b"*64,"c"*64,"off"),("a"*40,"b"*40,"c"*64,"off"),("a"*40,"b"*64,"c"*64,"v2")):
            with self.assertRaises(s.IntegrityError): s.Binding(*binding)

    def test_raw_invalid_status_cannot_recover_true_no_hit(self):
        x = torch.ones(1, 1, 4, 72); x.reshape(-1)[:5] = torch.tensor([float('nan'),0.,.5,1.,float('inf')])
        p = packet(v=replace(values(), raw_lidar=x))
        self.assertEqual(s.lidar_categories(p.tensor("raw_lidar"))[:5],
                         ("INVALID","NEAR_OR_CLIPPED","FINITE_RETURN","FAR_OR_NO_RETURN","INVALID"))

    def test_timestamps_missing_stale_future_and_wrong_frame(self):
        p = packet()
        for change in (dict(capture_sim_s=None), dict(capture_sim_s=2),dict(decision_sim_s=1),
                       dict(frame="BODY_TILTED"),dict(depth_convention="OPTICAL_Z"),dict(provenance="MEASURED_NOW")):
            d = s.diagnose(replace(p, camera_time=replace(p.camera_time, **change)))
            self.assertTrue(d.sensor_reasons); self.assertEqual(d.utility,"ABSTAIN")

    def test_missing_utility_and_safety_never_fabricated(self):
        d = s.diagnose(packet())
        self.assertEqual((d.utility,d.safety,d.physical_safety,d.owner,d.action_applied),
                         ("ABSTAIN","INPUT_CERTIFICATE_UNAVAILABLE","NOT_ASSESSED","F",False))
        self.assertEqual(set(d.hypotheses), {"TARGET_SURFACE","FOREGROUND_OCCLUDER","FALSE_ASSOCIATION","UNKNOWN_OR_UNOBSERVED"})

    def test_tensor_storage_does_not_accept_objects_or_mutable_payload(self):
        with self.assertRaises(s.IntegrityError): s.TensorBytes((1,),"|O",b'12345678')
        with self.assertRaises(s.IntegrityError): s.TensorBytes((1,),"|u1",bytearray([1]))

    def test_metadata_cannot_hold_mutable_GT_or_source_handles(self):
        for payload in ({'privileged_target':[6.,0.,0.]},[1.],torch.zeros(1),FakeTask(),float('nan')):
            with self.subTest(payload=type(payload)),self.assertRaises(s.IntegrityError):
                s.SensorTime(payload,None,0.,10.)
        with self.assertRaises(s.IntegrityError):s.SensorTime(0.,0.,0.,10.,frame=FakeTask())
        p=packet()
        with self.assertRaises(s.IntegrityError):replace(p,values=tuple([k,v]for k,v in p.values))
        with self.assertRaises(s.IntegrityError):replace(p,calibration=(float('nan'),))

    def test_clock_and_frame_semantics_are_in_fingerprint_but_host_time_is_not(self):
        p=packet()
        for change in (dict(provenance='MISSING'),dict(frame='BODY_TILTED'),
                       dict(depth_convention='OPTICAL_Z'),dict(decision_sim_s=1.)):
            self.assertNotEqual(replace(p,camera_time=replace(p.camera_time,**change)).fingerprint(),p.fingerprint())
        self.assertEqual(replace(p,camera_time=replace(p.camera_time,host_arrival_monotonic_s=99.)).fingerprint(),p.fingerprint())
        with self.assertRaises(s.IntegrityError):s.SensorTime(10**1000,0.,0.,10.)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.log=s.EventLogger(Path(self.tmp.name)/'new'); self.addCleanup(self.log.close)
        self.r=s.Recorder(self.log,True); self.r.reset()

    def begin(self, p=None):
        self.r.observation(p or packet()); self.r.begin(torch.zeros(1,4),torch.zeros(1,898),'rng')
        self.r.transformed(torch.zeros(1,4))

    def test_initial_action0_and_duplicate_observation(self):
        self.r.observation(packet())
        with self.assertRaises(s.IntegrityError): self.r.observation(packet())
        self.assertEqual(self.r.pending.key.action,0)

    def test_missing_sensor_or_command_refused(self):
        with self.assertRaises(s.IntegrityError): self.r.begin(torch.zeros(1,4),torch.zeros(1,898),'rng')
        self.r.observation(packet()); self.r.begin(torch.zeros(1,4),torch.zeros(1,898),'rng')
        with self.assertRaises(s.IntegrityError): self.r.finish(truth(packet().key),'rng',{})

    def test_out_of_order_and_invalid_ID_rejected(self):
        for p in (packet(action=1),packet(episode=1),packet(generation=3)):
            with self.assertRaises(s.IntegrityError): self.r.observation(p)
        with self.assertRaises(s.IntegrityError): s.JoinKey(0,0,True,0,0)

    def test_reset_requires_old_terminal_record(self):
        self.begin()
        with self.assertRaises(s.IntegrityError): self.r.reset()
        self.r.finish(truth(packet().key),'rng',{})
        with self.assertRaises(s.IntegrityError): self.r.reset()

    def test_terminal_reset_and_new_sensor_join(self):
        self.begin(); self.r.finish(truth(packet().key,capture=True),'rng',{})
        with self.assertRaises(s.IntegrityError): self.r.observation(packet(action=1,generation=1,decision=1))
        self.r.reset(); p=packet(episode=1,action=0,generation=1,decision=1); self.r.observation(p)
        self.assertEqual(self.r.pending.key.episode,1)

    def test_capture_crash_height_oob_truth_table(self):
        # Independent expected outcomes; production compute is not called here.
        cases=[({},None),({'capture':True},True),({'crash':True},False),
               ({'capture':True,'contact_substeps':1,'contact_now':True},False),
               ({'capture':True,'height_now':True},False),({'capture':True,'oob_now':True},False),
               ({'timeout':True},False)]
        for i,(flags,expected) in enumerate(cases):
            with self.subTest(flags=flags):
                d=Path(self.tmp.name)/('case%d'%i); log=s.EventLogger(d); r=s.Recorder(log,False); r.reset()
                r.observation(packet());r.begin(torch.zeros(1,4),torch.zeros(1,898),'r');r.transformed(torch.zeros(1,4))
                r.finish(truth(packet().key,**flags),'r',{});log.close()
                row=json.loads((d/'evaluator.jsonl').read_text());self.assertEqual(row['episode']['collision_free_capture'],expected)

    def test_earlier_contact_blocks_later_capture(self):
        self.begin(); self.r.finish(truth(packet().key,contact_substeps=1),'r',{})
        p=packet(action=1,generation=1,decision=1);self.begin(p)
        self.r.finish(truth(p.key,capture=True),'r',{});self.log.close()
        rows=[json.loads(x) for x in (self.log.directory/'evaluator.jsonl').read_text().splitlines()]
        self.assertFalse(rows[-1]['episode']['collision_free_capture'])

    def test_process_cap_is_censor_not_timeout(self):
        self.begin();self.r.finish(truth(packet().key),'r',{});self.r.censor('PROCESS_ACTION_CAP');self.log.close()
        last=json.loads((self.log.directory/'events.jsonl').read_text().splitlines()[-1])
        self.assertEqual((last['kind'],last['timeout']),('CENSORED',False))

    def test_unsupported_substep_contract_rejected(self):
        self.begin()
        with self.assertRaises(s.IntegrityError):self.r.finish(truth(packet().key,substeps=9),'r',{})

    def test_logger_error_is_fatal_without_alternate_action(self):
        self.log.streams['sensors'].close();self.r.observation(packet()); action=torch.ones(1,4)
        with self.assertRaises(s.IntegrityError):self.r.begin(action,torch.zeros(1,898),'r')
        self.assertTrue(torch.equal(action,torch.ones(1,4)))

    def test_log_size_and_existing_path_fail_closed(self):
        with self.assertRaises(FileExistsError):s.EventLogger(self.log.directory)
        self.log.max_bytes=1
        with self.assertRaises(s.IntegrityError):self.log.event('too big')


class CallThroughTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)

    def run_case(self,name,**kw):
        return exercise(Path(self.tmp.name)/name,**kw)

    def test_exactly_once_counts_and_actor_policy_PI_unmodified(self):
        a,_,ac=self.run_case('plain',observed=False)
        b,r,bc=self.run_case('on')
        self.assertEqual((a.perception.calls,a.transforms,a.computes,ac),(b.perception.calls,b.transforms,b.computes,bc))
        self.assertEqual((b.perception.calls,b.transforms),(4,3));self.assertEqual(b.integral,.30000000000000004)
        for name in ('robot_position','robot_linvel','robot_orientation'):
            self.assertTrue(torch.equal(a.obs_dict[name],b.obs_dict[name]))
        self.assertTrue(torch.equal(a.task_obs['observations'],b.task_obs['observations']))
        self.assertTrue(torch.equal(a.command,b.command));self.assertEqual(r.completed,3)

    def test_current_returned_camera_flag_not_stale_task_flag(self):
        t,r,_=self.run_case('camera',steps=1)
        rows=[json.loads(x)for x in (r.logger.directory/'sensors.jsonl').read_text().splitlines()]
        self.assertEqual(rows[0]['values']['camera']['values'],[True])
        self.assertFalse(t._camera_visible_now.item())

    def test_OFF_ON_every_action_exact_and_stride_only_diagnostics(self):
        _,off,_=self.run_case('off',shadow=False,steps=7)
        ref=[json.loads(x)for x in (off.logger.directory/'parity.jsonl').read_text().splitlines()]
        _,on,_=self.run_case('on',steps=7,reference=ref)
        actual=[json.loads(x)for x in (on.logger.directory/'parity.jsonl').read_text().splitlines()]
        self.assertEqual(actual,ref);self.assertEqual(len(on.latencies),2);self.assertEqual(on.completed,7)

    def test_mismatch_stops_before_action_or_command_execution(self):
        _,off,_=self.run_case('off',shadow=False,steps=1)
        ref=[json.loads(x)for x in (off.logger.directory/'parity.jsonl').read_text().splitlines()]
        for field in ('actor','policy','sensor','rng_before','command'):
            changed=[dict(ref[0],**{field:'bad'})]
            with self.subTest(field=field),self.assertRaises(s.IntegrityError):
                self.run_case(field,steps=1,reference=changed)

    def test_rng_unchanged_by_observer(self):
        before=s.rng_fingerprint();self.run_case('on',steps=2);self.assertEqual(s.rng_fingerprint(),before)

    def test_extra_renderer_call_is_integrity_failure(self):
        with self.assertRaisesRegex(s.IntegrityError,'UNEXPECTED_OBSERVE_TRANSFORM_COMPUTE_COUNT'):
            self.run_case('extra',scenario={'extra_render':True},steps=1)

    def test_terminal_postreset_sensor_mutation_is_also_compared(self):
        _,off,_=self.run_case('off_reset',shadow=False,scenario={'capture':True},steps=1)
        ref=[json.loads(x)for x in (off.logger.directory/'parity.jsonl').read_text().splitlines()]
        self.assertEqual(ref[0]['post']['key']['episode'],1)
        with self.assertRaisesRegex(s.IntegrityError,'POST_RESET_OR_SENSOR_BOUNDARY'):
            self.run_case('on_reset',reference=ref,scenario={'capture':True,'reset_mutates_lidar':True},steps=1)

    def test_GT_poison_not_copied_into_sensor_packet(self):
        t=FakeTask(); log=s.EventLogger(Path(self.tmp.name)/'gt');r=s.Recorder(log,True)
        with s.EvaluatorTap(t,r,BIND):
            t.reset(); before=r.pending.fingerprint()
            t.target_position.fill_(999);t.obs_dict['navrl_target_position'].fill_(-999)
            t.obs_dict['segmentation_pixels'].zero_();t.task_obs['states'].fill_(500)
            self.assertEqual(r.pending.fingerprint(),before)
            self.assertFalse(any('target' in name or 'semantic' in name for name,_ in r.pending.values))
        log.close()

    def test_substep_contact_capture_preserved_despite_mask(self):
        t,r,_=self.run_case('contact',scenario=dict(capture=True,contact_substep=3),steps=1)
        row=json.loads((r.logger.directory/'evaluator.jsonl').read_text())
        self.assertEqual(row['episode']['contact_substeps'],1)
        self.assertFalse(row['episode']['collision_free_capture'])
        self.assertTrue(row['episode']['simultaneous_capture_failure'])
        self.assertEqual(row['record']['key']['episode'],0);self.assertEqual(r.pending.key.episode,1)
        self.assertFalse(row['record']['crash']);self.assertTrue(row['record']['capture'])

    def test_height_and_OOB_capture_before_reset(self):
        for name in ('height','oob'):
            _,r,_=self.run_case(name,scenario={name:True,'capture':True},steps=1)
            row=json.loads((r.logger.directory/'evaluator.jsonl').read_text())
            self.assertTrue(row['record'][name+'_now']);self.assertFalse(row['episode']['collision_free_capture'])

    def test_hook_lifecycle_restored_even_on_exception(self):
        t=FakeTask(); originals=(t.step,t.perception.observe,t.transform_action_to_command)
        log=s.EventLogger(Path(self.tmp.name)/'restore');r=s.Recorder(log,True)
        with self.assertRaisesRegex(RuntimeError,'probe'):
            with s.EvaluatorTap(t,r,BIND):raise RuntimeError('probe')
        self.assertEqual((t.step,t.perception.observe,t.transform_action_to_command),originals)
        self.assertIsNone(t.sim_env.physics_step_callback);log.close()

    def test_snapshot_keeps_sensor_bytes_and_does_not_draw_rng(self):
        log=s.EventLogger(Path(self.tmp.name)/'snap');r=s.Recorder(log,True,snapshot_cap=1);t=FakeTask()
        before=s.rng_fingerprint()
        with s.EvaluatorTap(t,r,BIND):t.reset();t.step(torch.zeros(1,4))
        log.close();self.assertEqual(before,s.rng_fingerprint())
        with np.load(log.directory/'sensor_000.npz',allow_pickle=False)as saved:
            self.assertEqual(saved['rgb'].shape,(1,3,90,160));self.assertNotIn('target_position',saved.files)

    def test_existing_substep_callback_chained_once(self):
        class Callback:
            calls=0;post=0
            def __call__(self):self.calls+=1
            def post_physics_step(self):self.post+=1
        t=FakeTask();cb=Callback();t.sim_env.physics_step_callback=cb
        log=s.EventLogger(Path(self.tmp.name)/'chain');r=s.Recorder(log,False)
        with s.EvaluatorTap(t,r,BIND):t.reset();t.step(torch.zeros(1,4))
        self.assertEqual((cb.calls,cb.post),(10,10));self.assertIs(t.sim_env.physics_step_callback,cb);log.close()

    def test_no_writer_capability_or_extra_detector_in_diagnostics(self):
        d=s.diagnose(packet());self.assertFalse(any(callable(x)or isinstance(x,(torch.Tensor,dict))for x in d.__dict__.values()))
        self.assertNotIn('action',d.__dict__);self.assertEqual(d.owner,'F')


if __name__=='__main__':unittest.main()
