"""Independent CPU fixtures for the real call-through observer and phase protocol."""
from contextlib import contextmanager
import copy
from dataclasses import fields, replace, FrozenInstanceError
import json
import os
from pathlib import Path
import random
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval import boia_shadow as s
from tools import boia_shadow as b


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
        self.sim_steps = torch.zeros(1, dtype=torch.long)

    def capture(self):
        self.capture_count += 1
        return self.task.obs_dict["depth_range_pixels"]

    def set_physics_step_callback(self, x):
        self.physics_step_callback = x

    def post_reward_calculation_step(self):
        if self.task.captured_now.item() or self.task.crashed_now.item() or self.task.truncations.item():
            ids = torch.tensor([0])
            self.sim_steps.zero_()
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
        # Like ExtractObsWrapper: the returned actor tensor IS the task buffer, overwritten next step.
        return self.task_obs["observations"]

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
        self.captured_now[:] = bool(self.scenario.get("capture", False) or
                                    self.steps >= self.scenario.get("capture_after", 10**9))
        raw = self.obs_dict["crashes"].bool().clone()
        z = self.obs_dict["robot_position"][:, 2]
        raw |= (z < .1) | (z > 4)
        pos = self.obs_dict["robot_position"][:, :2]
        raw |= ((pos < -1) | (pos > 41)).any(1)
        self.crashed_now[:] = raw & ~self.captured_now
        self.obs_dict["crashes"][:] = self.crashed_now | self.captured_now  # actual alias hazard

    def step(self, action):
        if self.scenario.get("native_rng"):  # native draws, like env_manager's random.gauss substeps
            random.gauss(10., 0.); np.random.rand(); torch.rand(1)
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
        self.sim_env.sim_steps += 1
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


def exercise(root, shadow=True, steps=3, scenario=None, reference=None, observed=True,
             tap_class=None, step_filter=None):
    """One stage through the production player loop `drive`; observed=False is S1_NATIVE."""
    t, calls, root = FakeTask(scenario), [0], Path(root)
    if observed:
        log = s.EventLogger(root)
        r = s.Recorder(log, shadow, reference=reference)
        tap = (tap_class or s.EvaluatorTap)(t, r, BIND)
    else:
        root.mkdir(parents=True)
        log = r = tap = None
    trace = s.HostTrace(root)

    def act(obs):
        calls[0] += 1
        return torch.tanh(obs[:, :4] * .01)

    # Late binding, as player.env_step reaches task.step: the tap replaces it on entry.
    step = (lambda action: t.step(action)) if step_filter is None else (lambda action: step_filter(t.step(action)))
    try:
        with (tap if observed else nullcontext()):
            s.drive(t, t.reset, act, step, trace, steps, tap, r)
            if observed: r.censor("PROCESS_ACTION_CAP")
    finally:
        trace.close()
        if log is not None: log.close()
    return t, r, calls[0]


from contextlib import nullcontext


def rows(directory, name):
    return [json.loads(x) for x in (Path(directory) / name).read_text().splitlines()]


@contextmanager
def fresh_rng_each_stage():
    """Fresh processes start from one seeded state: restore it before each stage and after."""
    saved = (random.getstate(), np.random.get_state(), torch.get_rng_state())

    def restore():
        random.setstate(saved[0]); np.random.set_state(saved[1]); torch.set_rng_state(saved[2])
    try:
        yield restore
    finally:
        restore()


class PlantNudgeTap(s.EvaluatorTap):
    """Fable G4-R6 cx1: an observer that moves the plant 6 mm per action, identically OFF and ON."""
    def _post(self, old):
        wrapped = super()._post(old)
        def nudged(*a, **kw):
            self.task.obs_dict["robot_position"][:, 0] += .006
            return wrapped(*a, **kw)
        return nudged


class ResetNudgeTap(s.EvaluatorTap):
    """Perturbs only the post-terminal reset state, i.e. only the last returned observation."""
    def _reset(self, old):
        wrapped = super()._reset(old)
        def nudged(ids, *a, **kw):
            out = wrapped(ids, *a, **kw)
            if self.recorder.episode >= 1:
                self.task.obs_dict["robot_position"][:, 1] += .006
            return out
        return nudged


class RngDrawTap(s.EvaluatorTap):
    """An observer that consumes one global torch draw per action."""
    def _compute(self, old):
        wrapped = super()._compute(old)
        def drawing(*a, **kw):
            torch.rand(1)
            return wrapped(*a, **kw)
        return drawing


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


class NativeBaselineTests(unittest.TestCase):
    """S1_NATIVE vs S1_OFF vs S1_ON with the production loop (Fable G4-R6 F1)."""
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)

    def stages(self, tap_class=None, scenario=None, steps=7):
        with fresh_rng_each_stage() as fresh:
            fresh();exercise(self.root/'native',observed=False,steps=steps,scenario=scenario)
            fresh();exercise(self.root/'off',shadow=False,steps=steps,scenario=scenario,tap_class=tap_class)
            fresh();exercise(self.root/'on',steps=steps,scenario=scenario,tap_class=tap_class,
                              reference=rows(self.root/'off','parity.jsonl'))
        return {k:self.root/k for k in ('native','off','on')}

    def cmp(self, a, c, name):
        return b.compare_rows(rows(a,name),rows(c,name))

    def test_native_stage_runs_without_any_observer_hook(self):
        t=FakeTask();original=(t.step,t.perception.observe,t.sim_env.physics_step_callback)
        self.assertFalse(any(s.hook_audit(t).values()))
        log=s.EventLogger(self.root/'hooks');r=s.Recorder(log,False)
        with s.EvaluatorTap(t,r,BIND):
            audit=s.hook_audit(t);self.assertEqual(len(audit),9);self.assertTrue(all(audit.values()))
        log.close();self.assertFalse(any(s.hook_audit(t).values()))
        self.assertEqual((t.step,t.perception.observe,t.sim_env.physics_step_callback),original)

    def test_normal_fixture_native_off_on_all_exact_including_native_rng_draws(self):
        d=self.stages(scenario={'native_rng':True,'capture_after':4})
        host=rows(d['native'],'host.jsonl');self.assertTrue(host[-1]['done']);self.assertEqual(len(host),6)
        self.assertEqual(self.cmp(d['native'],d['off'],'host.jsonl')['status'],'PASS')
        for name in ('parity.jsonl','host.jsonl'):
            self.assertEqual(self.cmp(d['off'],d['on'],name)['status'],'PASS')
        self.assertNotEqual(host[1]['rng_pre_policy'],host[2]['rng_pre_policy'])  # native draws are visible

    def test_cx1_constant_observer_perturbation_passes_off_on_but_fails_native_off(self):
        d=self.stages(tap_class=PlantNudgeTap)
        for name in ('parity.jsonl','host.jsonl'):
            self.assertEqual(self.cmp(d['off'],d['on'],name)['status'],'PASS')  # v1 gate is blind
        result=self.cmp(d['native'],d['off'],'host.jsonl')
        self.assertEqual((result['status'],result['index']),('MISMATCH',1))
        self.assertIn('state.own_position',result['fields'])

    def test_post_reset_perturbation_detected_on_terminal_returned_row(self):
        d=self.stages(tap_class=ResetNudgeTap,scenario={'capture_after':2})
        result=self.cmp(d['native'],d['off'],'host.jsonl');host=rows(d['native'],'host.jsonl')
        self.assertEqual((result['status'],result['index']),('MISMATCH',len(host)-1))
        self.assertTrue(host[-1]['done']);self.assertIn('state.own_position',result['fields'])

    def test_observer_rng_draw_detected(self):
        d=self.stages(tap_class=RngDrawTap)
        result=self.cmp(d['native'],d['off'],'host.jsonl')
        self.assertEqual(result['status'],'MISMATCH');self.assertIn('rng_post_step',result['fields'])

    def test_actor_hashed_before_policy_although_returned_obs_aliases_task_buffer(self):
        exercise(self.root/'alias',steps=4);host=rows(self.root/'alias','host.jsonl')
        for prev,cur in zip(host,host[1:]):
            self.assertEqual(cur['actor'],prev['returned']);self.assertNotEqual(cur['actor'],cur['returned'])

    def test_host_and_observer_must_agree_on_terminal_and_fingerprints(self):
        flip=lambda out:(out[0],out[1],~out[2],out[3])
        with self.assertRaisesRegex(s.IntegrityError,'HOST_OBSERVER_TERMINAL_JOIN_MISMATCH'):
            exercise(self.root/'flip',steps=2,step_filter=flip)
        with self.assertRaisesRegex(s.IntegrityError,'OBSERVER_AND_RECORDER'):
            s.drive(FakeTask(),None,None,None,None,1,tap=object())

    def test_empty_missing_short_and_long_traces_never_pass(self):
        d=self.stages(steps=3);a=rows(d['native'],'host.jsonl')
        for left,right,mode in ((a,[],'full'),([],a,'full'),(None,a,'full'),(a,None,'full'),
                                (a,a[:-1],'full'),(a[:-1],a,'full'),(a[:-1],a,'prefix')):
            with self.subTest(n=(None if left is None else len(left),None if right is None else len(right),mode)):
                self.assertEqual(b.compare_rows(left,right,mode)['status'],'MISMATCH')
        self.assertEqual(b.compare_rows(a[:-1],a,'prefix',left_censored=True)['status'],'PASS')
        self.assertEqual(b.compare_rows(a,a)['status'],'PASS')

    def test_host_trace_keeps_GT_as_hashes_and_refuses_reuse(self):
        exercise(self.root/'gt',steps=1);host=rows(self.root/'gt','host.jsonl')
        for row in host:
            self.assertTrue(all(len(v)==64 for v in row['state'].values()))
        self.assertNotIn('6.0',json.dumps(host))  # FakeTask target_position is 6.0 everywhere
        with self.assertRaises(FileExistsError):s.HostTrace(self.root/'gt')
        with self.assertRaises(s.IntegrityError):s.HostTrace(self.root/'missing')
        (self.root/'cap').mkdir();trace=s.HostTrace(self.root/'cap',max_bytes=10)
        with self.assertRaisesRegex(s.IntegrityError,'HOST_TRACE_OUTPUT_CAP'):trace.write({'kind':'too long'})
        trace.close()

    def test_runtime_attestation_reads_numerics_without_changing_them(self):
        flags=lambda:(torch.backends.cudnn.benchmark,torch.backends.cudnn.allow_tf32,
                      torch.backends.cuda.matmul.allow_tf32,torch.get_float32_matmul_precision())
        before=flags();a=s.runtime_attestation('cpu');self.assertEqual(flags(),before)
        self.assertEqual((a['cudnn_benchmark'],a['cudnn_allow_tf32'],a['matmul_allow_tf32'],a['float32_matmul_precision']),before)
        self.assertIsNone(a['device_name']);self.assertEqual(a['torch'],torch.__version__)


class FakePlayer:
    """rl_games player stand-in over FakeTask (CPU, deterministic)."""
    def __init__(self,task):self.env,self.model,self.is_rnn=task,NS(training=False),False
    def env_reset(self,env):return env.reset()
    def get_batch_size(self,obs,n):self.batch=n
    def get_action(self,obs,is_deterministic=False):
        assert is_deterministic;return torch.tanh(obs[:,:4]*.01)
    def env_step(self,env,action):return env.step(action)


class WorkerStandInTests(unittest.TestCase):
    """The real tools/boia_shadow.worker and verify, with CPU stand-ins for Isaac/player/CUDA."""
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);tmp=Path(self.tmp.name)
        self.p=copy.deepcopy(b.load(b.PLAN));self.p['output_root']=str(tmp/'BOIA-SHADOW-Q01-V2')
        Path(self.p['output_root']).mkdir()
        self.ckpt=tmp/'policy.pth';self.ckpt.write_bytes(b'frozen F stand-in')
        self.p['frozen']['policy_f_sha256']=b.sha(self.ckpt)

    def campaign(self, tap_class=None, scenario=None, stages=b.STAGES):
        from motar_eval import eq1d_telemetry as tel
        env={'BOIA_SHADOW_PARENT_PID':str(os.getppid()),'BOIA_SHADOW_CONFIG_SHA':b.sha(b.PLAN)}
        with fresh_rng_each_stage() as fresh:
            for stage in stages:
                fresh();task=FakeTask(scenario);player=FakePlayer(task)
                fake=ModuleType('eq1d_fixture_launcher');fake.build_player=lambda n,c:(player,{})
                fake.find_task=lambda pl:task;fake.check_origins=lambda:{};fake.find_rms=lambda m:NS()
                with patch.dict(sys.modules,{'isaacgym':ModuleType('isaacgym'),'eq1d_fixture_launcher':fake}),\
                     patch.dict(os.environ,env),patch.object(sys,'path',list(sys.path)),\
                     patch.object(b,'guard_sources'),patch.object(b,'checkpoint',return_value=self.ckpt),\
                     patch.object(b,'runtime_profile',return_value={}),\
                     patch.object(tel,'rms_digest',return_value=self.p['frozen']['rms_digest_sha256']),\
                     patch.object(torch.cuda,'reset_peak_memory_stats'),patch.object(torch.cuda,'synchronize'),\
                     patch.object(torch.cuda,'max_memory_allocated',return_value=0),\
                     patch.object(s,'EvaluatorTap',tap_class or s.EvaluatorTap):
                    b.worker(self.p,stage,'a'*40)
        with patch.object(b,'guard_sources'):
            return b.verify(self.p,'a'*40)

    def test_four_stages_through_real_worker_then_verify_pass(self):
        result=self.campaign(scenario={'capture_after':30})
        self.assertEqual(result['verdict'],b.PASS,json.dumps(result['comparisons'],indent=1)[:3000])
        root=Path(self.p['output_root'])
        native=b.load(root/'S1_NATIVE/process_done.json')
        self.assertFalse(any(native['hooks_during'].values()));self.assertIsNone(native['observe_calls'])
        self.assertEqual(sorted(x.name for x in (root/'S1_NATIVE').iterdir()),['host.jsonl','process_done.json'])
        self.assertEqual((native['actions'],native['ended_episodes']),(31,1))
        on=b.load(root/'S1_ON/process_done.json');self.assertTrue(all(on['hooks_during'].values()))
        self.assertEqual(on['qualification_scope']['wait_intervene_computation'],'NOT_QUALIFIED')
        self.assertEqual(result['comparisons']['replay_s0_vs_on']['files']['host.jsonl']['rows'],[21,32])
        self.assertEqual(result['claims']['observer_noninterference'],'SUPPORTED_EXACT_HOST_TRACE_THIS_PROFILE')
        with self.assertRaises(FileExistsError),patch.object(b,'guard_sources'):b.verify(self.p,'a'*40)

    def test_cx1_through_real_worker_is_reported_as_unattributed_mismatch(self):
        result=self.campaign(tap_class=PlantNudgeTap)
        self.assertEqual(result['verdict'],b.MISMATCH)
        self.assertEqual(result['comparisons']['off_vs_on']['status'],'PASS')
        self.assertEqual(result['comparisons']['native_vs_off']['status'],'MISMATCH')
        self.assertEqual(result['claims']['observer_noninterference'],'NOT_SUPPORTED')
        self.assertEqual(result['claims']['mismatch_cause'],'NOT_VERIFIED')

    @unittest.skipUnless(__import__('threading').current_thread() is __import__('threading').main_thread(),
                         'signal handlers need the main thread')
    def test_real_SIGTERM_mid_stage_leaves_failure_receipt_and_restores_handler(self):
        import signal
        class KilledPlayer(FakePlayer):
            calls=0
            def get_action(self,obs,is_deterministic=False):
                KilledPlayer.calls+=1
                if KilledPlayer.calls==3:os.kill(os.getpid(),signal.SIGTERM)
                return super().get_action(obs,is_deterministic)
        before=signal.getsignal(signal.SIGTERM)
        with patch(__name__+'.FakePlayer',KilledPlayer),self.assertRaisesRegex(RuntimeError,'SupervisorStop'):
            self.campaign(stages=('S0',))
        self.assertIs(signal.getsignal(signal.SIGTERM),before)
        failure=b.load(Path(self.p['output_root'])/'S0/failure.json')
        self.assertIn('SIGTERM_FROM_SUPERVISOR',failure['reason']);self.assertEqual(failure['actions_completed'],2)
        self.assertEqual(failure['host_rows'],3)  # INITIAL + two completed actions, flushed

    def test_incomplete_campaign_is_not_evaluated_as_pass(self):
        result=self.campaign(stages=('S0','S1_NATIVE'))
        self.assertEqual(result['verdict'],b.INCOMPLETE)
        self.assertEqual(result['comparisons']['off_vs_on']['status'],'NOT_AVAILABLE')


if __name__=='__main__':unittest.main()
