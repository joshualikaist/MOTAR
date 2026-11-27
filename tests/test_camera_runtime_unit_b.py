"""Unit B (2026-11-18): PX4 vertical fields forwarded gateway -> camera policy node, and the bounded refusal hold.

Goal 1. The PX4 gateway forwards VehicleLocalPosition z_valid, epv and dead_reckoning inside its own private Odometry
(camera_ros2_adapter.VERTICAL_ENCODING, pose.covariance slots), the same message whose pose the policy uses. The camera
node reads them only when its binding declares odometry_source = gateway_odometry_source(); absent or malformed fields
attach nothing, so the outdoor runtime refuses the frame with its existing named reasons (a zero hold).
Goal 2. A hold episode (consecutive zero-hold proposals without a policy command) is bounded by the outdoor contract's
PROVISIONAL ground_estimate.refusal_hold_max_s on the proposals' capture stamps; at the limit the session latches
refusal_hold_timeout, proposes nothing more and needs an explicit reset; the gateway then revokes on its stale path.

Everything here runs on CPU with ROS-shaped stand-in messages and synthetic scenes: no ROS graph, DDS, PX4, SITL,
Gazebo, camera or vehicle. Every negative check is paired with a positive control asserting a nonzero count.
"""
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'tests'))
import camera_policy_contract as C
import camera_policy_runtime as R
import camera_ros2_adapter as A
import camera_ros2_px4 as P
import camera_ros2_px4_node as N
from camera_ros2_session import HoldTimeout, RosPolicySession, SessionFault
import test_camera_runtime_fixes as F
import test_camera_runtime_fixes_r2 as G
import test_camera_runtime_outdoor_r1 as O
from test_camera_ros2_px4 import snapshot

PX4_MSG_DIR = Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/msg')
# The bound PX4 v1.14.3 message definitions before unit B (literal copy): unit B must not change them.
PRE_UNIT_B_MESSAGE_SHA256 = {
    "OffboardControlMode.msg": "7482f8af73a21de04427629051a87ce06cfb9bf3a580242d3cf9c212adf3f906",
    "TrajectorySetpoint.msg": "a9ed19116a311b89d41e8cd3e6fc49951ec23824126a102dd41d6f6b9e7b3cb6",
    "VehicleOdometry.msg": "f793b3b28c5ad65f9cf485b24816ba81de1fcaf9d5de903ffd75183930f95778",
    "VehicleLocalPosition.msg": "8fdca654db4b9245f71354dcbeffc8247b9ec4fe2a91735342bf89f8126c631a",
    "VehicleStatus.msg": "824ff9a94ee3600eff2f2dd45a3484d5bc805ff2d7fc56b701c754494a7f788d",
}
FIELDS = {'z_valid': True, 'epv': .12, 'dead_reckoning': False}


def declared_binding(path):
    binding = F.ros_binding(path)
    binding['odometry_source'] = A.gateway_odometry_source()
    return binding


def odometry(t, height=1., fields=FIELDS, *, pose_cov=None, twist_cov=None):
    """F.odometry_message (level, yaw 0) carrying the gateway encoding of `fields` (None: all NaN, i.e. absent)."""
    message = F.odometry_message(t, [0., 0., height], 0., [0., 0., 0.], 0.)
    pose, twist = A.encode_vertical_covariance(fields)
    message.pose.covariance = list(pose if pose_cov is None else pose_cov)
    message.twist.covariance = list(twist if twist_cov is None else twist_cov)
    return message


def gateway_odometry(state, world='odom', body='base_link'):
    """The pose and covariance fields Px4GatewayNode._publish_odometry writes for one polled state (ROS-shaped)."""
    sec, ns = divmod(int(round(state['t'] * 1e9)), 1000000000)
    x, y, z, w = state['quat_xyzw']
    pose_cov, twist_cov = N.odometry_covariances(state)
    return NS(header=NS(frame_id=world, stamp=NS(sec=sec, nanosec=ns)), child_frame_id=body,
              pose=NS(pose=NS(position=NS(x=state['pos'][0], y=state['pos'][1], z=state['pos'][2]),
                              orientation=NS(x=x, y=y, z=z, w=w)), covariance=pose_cov),
              twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.)), covariance=twist_cov))


def bridge_config():
    value = N.unconfigured_template()
    value.update(status='CONFIGURED', enable_offboard_publication=False)
    value['frames'] = dict(world='odom', body='base_link')
    return value


def px4_binding():
    return P.Px4MessageBinding(P.PX4_VERSION, P.PX4_MESSAGE_SHA256, 'CPU software fixture, not hardware qualification')


def px4_snapshot(now, mono, *, z=-1., **local):
    """test_camera_ros2_px4.snapshot (PX4 NED z = -1 m: ENU height 1 m) plus the vertical fields of local_position."""
    sample = snapshot(now, mono, z=z)
    sample['local_position'].update(local)
    return sample


_SCENE = {}


def outdoor_images():
    """A level synthetic outdoor scene (trained-equal source, tree line, textured ground, body z 1.0 m) as RGB and
    depth in millimetres (0 = invalid), rendered once."""
    if 'scene' not in _SCENE:
        frame, _ = O.outdoor_scene('trained_equal', 1., objects=[O.TREE_LINE])
        mm = np.where(frame['invalid'], 0., np.round(np.asarray(frame['depth_z_m'], np.float64) * 1000.))
        mm = np.where(mm > 65535, 0, mm).astype(np.uint16)
        _SCENE['scene'] = (frame['rgb'], mm, frame['intrinsics'])
    return _SCENE['scene']


class OutdoorGeometryRuntime:
    """Session stand-in whose step runs the PRODUCTION outdoor preprocess (projection, estimate_ground with PX4's
    vertical fields, filters, sky rule, blob_near) without the checkpoint, and records the states it received."""
    def __init__(self, contract=None):
        self.contract = contract or O.outdoor_contract()
        self.geometry = G.geometry_runtime(self.contract)
        self.states, self.grounds, self.events, self.failed = [], [], [], False

    def reset(self): self.events.append('reset'); self.failed = False
    def invalidate(self): self.failed = True
    def applied_zero_hold(self): self.events.append('zero_hold_applied')
    def height_band(self, state): return R.height_band(state, self.contract)

    def step(self, frame, state, *, now_s):
        self.states.append(dict(state))
        _, _, counts, _ = self.geometry.preprocess(frame, state)        # FrameRefused propagates
        self.grounds.append(counts['outdoor']['ground'])
        return dict(t=now_s, velocity_world_enu_mps=[.5, 0., 0.], yaw_rate_rps=0.)


def run_ticks(session, odometry_messages, start=3., wall0=100.):
    """Offer the outdoor images plus one odometry message per tick (10 Hz) and return (proposals, refusals)."""
    rgb, mm, K = outdoor_images()
    proposals = []
    for k, odom in enumerate(odometry_messages):
        t, wall = start + k * .1, wall0 + k * .1
        msgs = F.image_messages(t, rgb, mm, K)
        msgs['odometry'] = odom(t)
        for kind, message in msgs.items():
            session.offer(kind, message, received_monotonic_s=wall)
        item = session.prepare(now_ros_s=t + .02, now_monotonic_s=wall + .02)
        result = session.infer(item)
        proposals.append(session.finish(item, result, now_ros_s=t + .02, now_monotonic_s=wall + .02))
    return proposals


# ------------------------------------------------------------------------------------- Goal 1: gateway side
class GatewayForwardsVerticalFields(unittest.TestCase):
    def test_px4_message_bindings_are_unchanged(self):
        self.assertEqual(P.PX4_MESSAGE_SHA256, PRE_UNIT_B_MESSAGE_SHA256)
        self.assertEqual(P.PX4_VERSION, 'v1.14.3')

    def test_vertical_fields_are_validated_never_defaulted(self):
        good = [dict(z_valid=True, epv=.12, dead_reckoning=False), dict(z_valid=False, epv=0., dead_reckoning=True),
                dict(z_valid=np.bool_(True), epv=np.float32(.25), dead_reckoning=np.bool_(False)),
                dict(z_valid=True, epv=3, dead_reckoning=False)]
        accepted = 0
        for value in good:
            fields = P.vertical_fields(dict(snapshot()['local_position'], **value))
            self.assertEqual(set(fields), set(P.VERTICAL_FIELDS))
            self.assertIs(type(fields['z_valid']), bool)
            self.assertIs(type(fields['dead_reckoning']), bool)
            self.assertIs(type(fields['epv']), float)
            accepted += 1
        self.assertEqual(accepted, 4)                                                    # positive control
        bad = [{}, dict(z_valid=True, epv=.1), dict(z_valid=True, dead_reckoning=False),
               dict(epv=.1, dead_reckoning=False, z_valid=1), dict(epv=float('nan'), dead_reckoning=False, z_valid=True),
               dict(epv=-.1, dead_reckoning=False, z_valid=True), dict(epv=float('inf'), dead_reckoning=False, z_valid=True),
               dict(epv='0.1', dead_reckoning=False, z_valid=True), dict(epv=True, dead_reckoning=False, z_valid=True),
               dict(epv=.1, dead_reckoning=None, z_valid=True), dict(epv=.1, dead_reckoning=False, z_valid='true')]
        base = {k: v for k, v in snapshot()['local_position'].items() if k not in P.VERTICAL_FIELDS}
        for value in bad:
            with self.subTest(value=value), self.assertRaises(P.GatewayError):
                P.vertical_fields(dict(base, **value))
        self.assertEqual(len(bad), 11)

    def test_bridge_forwards_the_fields_of_the_same_snapshot_and_absent_fields_stay_all_nan(self):
        session = N.BridgeSession(bridge_config(), px4_binding())
        sample = px4_snapshot(100., 20., z_valid=True, epv=.31, dead_reckoning=False)
        for kind in ('odometry', 'local_position', 'status'):
            session.offer(kind, sample[kind], receipt=20.)
        state, payload = session.poll(now=100., wall=20.)
        self.assertIsNone(payload)
        self.assertEqual(state['vertical'], {'z_valid': True, 'epv': .31, 'dead_reckoning': False})
        self.assertAlmostEqual(state['t'], 99.995)
        pose, twist = N.odometry_covariances(state)
        self.assertEqual(pose[:4], [A.VERTICAL_ENCODING_TAG, 1., .31, 0.])
        self.assertTrue(all(math.isnan(v) for v in pose[4:] + twist))
        self.assertEqual((len(pose), len(twist)), (36, 36))
        diag = session.diagnostics()
        self.assertEqual(diag['odometry_vertical_encoding'], A.VERTICAL_ENCODING)
        self.assertEqual(diag['vertical_fields']['status'], 'FORWARDED')
        json.dumps(diag, allow_nan=False)
        # Negative (pre-unit-B fixture without epv/dead_reckoning): nothing forwarded, the Odometry is all NaN as before
        # unit B, and the gateway itself is unaffected (it still publishes the pose).
        session = N.BridgeSession(bridge_config(), px4_binding())
        sample = snapshot(100., 20., z=-1.)
        for kind in ('odometry', 'local_position', 'status'):
            session.offer(kind, sample[kind], receipt=20.)
        state, _ = session.poll(now=100., wall=20.)
        self.assertIsNotNone(state)
        self.assertIsNone(state['vertical'])
        self.assertIn('missing', state['vertical_problem'])
        pose, twist = N.odometry_covariances(state)
        self.assertTrue(all(math.isnan(v) for v in pose + twist))
        self.assertEqual(session.diagnostics()['vertical_fields']['status'], 'ABSENT')
        json.dumps(session.diagnostics(), allow_nan=False)

    def test_z_valid_false_is_still_refused_by_the_gateway(self):
        session = N.BridgeSession(bridge_config(), px4_binding())
        sample = px4_snapshot(100., 20., epv=.1, dead_reckoning=False)
        sample['local_position']['z_valid'] = False
        for kind in ('odometry', 'local_position', 'status'):
            session.offer(kind, sample[kind], receipt=20.)
        self.assertEqual(session.poll(now=100., wall=20.), (None, None))
        self.assertIn('validity lost', session.fault)
        control = N.BridgeSession(bridge_config(), px4_binding())                      # positive control
        sample = px4_snapshot(100., 20., z_valid=True, epv=.1, dead_reckoning=False)
        for kind in ('odometry', 'local_position', 'status'):
            control.offer(kind, sample[kind], receipt=20.)
        self.assertIsNotNone(control.poll(now=100., wall=20.)[0])
        self.assertIsNone(control.fault)

    def test_generated_type_check_covers_the_read_fields(self):
        """A px4_msgs build that generates epv or dead_reckoning differently fails at construction, even when its
        own .msg and generated class agree with each other (the full-field comparison alone would accept it)."""
        with tempfile.TemporaryDirectory() as tmp:
            accepted = 0
            for epv, dead, ok in (('float32', 'bool', True), ('float64', 'bool', False), ('float32', 'uint8', False)):
                path = Path(tmp) / ('m_%s_%s.msg' % (epv, dead))
                path.write_text('uint64 timestamp\nbool z_valid\n%s epv\n%s dead_reckoning\n' % (epv, dead))
                alias = {'float32': 'float', 'float64': 'double', 'bool': 'boolean'}
                generated = dict(timestamp='uint64', z_valid='boolean', epv=alias[epv], dead_reckoning=alias.get(dead, dead))
                cls = type('Generated', (), {'get_fields_and_field_types': staticmethod(lambda g=generated: dict(g))})
                required = {k: P.READ_FIELD_TYPES['VehicleLocalPosition'][k] for k in ('z_valid', 'epv', 'dead_reckoning')}
                N.validate_generated_type(cls, path)                            # consistent .msg / class: accepted
                with self.subTest(epv=epv, dead=dead):
                    if ok:
                        N.validate_generated_type(cls, path, required)
                        accepted += 1
                    else:
                        with self.assertRaisesRegex(ValueError, 'must be'):
                            N.validate_generated_type(cls, path, required)
            self.assertEqual(accepted, 1)                                                # positive control
        self.assertEqual(P.READ_FIELD_TYPES['VehicleLocalPosition']['epv'], 'float')
        self.assertEqual(P.READ_FIELD_TYPES['VehicleLocalPosition']['dead_reckoning'], 'boolean')

    def test_read_field_types_match_the_bound_px4_sources(self):
        if not PX4_MSG_DIR.is_dir():
            self.skipTest('local PX4 message source unavailable; installed px4_msgs check remains NOT_TESTED here')
        alias = {'float32': 'float', 'float64': 'double', 'bool': 'boolean'}
        checked = 0
        for name, required in P.READ_FIELD_TYPES.items():
            path = PX4_MSG_DIR / (name + '.msg')
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), P.PX4_MESSAGE_SHA256[name + '.msg'])
            generated = {}
            for line in path.read_text().splitlines():
                line = line.split('#', 1)[0].strip()
                if line and '=' not in line:
                    kind, field = line.split()[:2]
                    base, _, size = kind.partition('[')
                    generated[field] = alias.get(base, base) + ('[' + size if size else '')
            cls = type(name, (), {'get_fields_and_field_types': staticmethod(lambda g=generated: dict(g))})
            N.validate_generated_type(cls, path, required)
            checked += len(required)
        self.assertGreater(checked, 20)


# ------------------------------------------------------------------------------------ Goal 1: encoding
class VerticalEncoding(unittest.TestCase):
    def test_round_trip_and_absent(self):
        forwarded = 0
        for fields in (FIELDS, dict(z_valid=False, epv=0., dead_reckoning=True), dict(z_valid=True, epv=4.9,
                                                                                         dead_reckoning=False)):
            decoded = A.decode_vertical_covariance(odometry(3., fields=fields))
            self.assertEqual(decoded['status'], 'FORWARDED')
            self.assertEqual(decoded['fields'], fields)
            forwarded += 1
        self.assertEqual(forwarded, 3)
        absent = A.decode_vertical_covariance(odometry(3., fields=None))
        self.assertEqual((absent['status'], absent['fields']), ('ABSENT', None))

    def test_malformed_encodings_are_never_read(self):
        nan = float('nan')
        pose, twist = A.encode_vertical_covariance(FIELDS)

        def patched(values, index, value):
            values = list(values)
            values[index] = value
            return values
        cases = {
            'other version tag': dict(pose_cov=patched(pose, 0, -20261119.)),
            'filler not NaN': dict(pose_cov=patched(pose, 5, 0.)),
            'twist not NaN': dict(twist_cov=patched(twist, 0, 0.)),
            'z_valid 0.5': dict(pose_cov=patched(pose, 1, .5)),
            'dead_reckoning 2': dict(pose_cov=patched(pose, 3, 2.)),
            'epv negative': dict(pose_cov=patched(pose, 2, -.1)),
            'epv inf': dict(pose_cov=patched(pose, 2, float('inf'))),
            'epv NaN': dict(pose_cov=patched(pose, 2, nan)),
            'short covariance': dict(pose_cov=pose[:35]),
            # A real covariance from another producer: positive diagonal, never the tag.
            'real covariance': dict(pose_cov=list(np.diag([.01] * 6).ravel()), twist_cov=list(np.diag([.02] * 6).ravel())),
        }
        for label, change in cases.items():
            with self.subTest(label=label):
                decoded = A.decode_vertical_covariance(odometry(3., **change))
                self.assertEqual(decoded['status'], 'MALFORMED', label)
                self.assertIsNone(decoded['fields'])
        missing = F.odometry_message(3., [0., 0., 1.], 0., [0., 0., 0.], 0.)            # no covariance attribute
        self.assertEqual(A.decode_vertical_covariance(missing)['status'], 'MALFORMED')
        self.assertEqual(A.decode_vertical_covariance(odometry(3.))['status'], 'FORWARDED')  # positive control
        for fields in (dict(FIELDS, epv=float('nan')), dict(FIELDS, z_valid=1), dict(FIELDS, extra=1), {}, 'x'):
            with self.subTest(fields=fields), self.assertRaises(A.RosInputError):
                A.encode_vertical_covariance(fields)


# ---------------------------------------------------------------------- Goal 1: binding declaration and build_frame
class BindingDeclaration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'fixture.txt'

    def test_declaration_is_optional_exact_and_versioned(self):
        plain = A.validate_binding(F.ros_binding(self.path))                           # existing bindings: unchanged
        self.assertNotIn('odometry_source', plain)
        self.assertIsNone(A.declared_vertical_encoding(plain))
        declared = A.validate_binding(declared_binding(self.path))
        self.assertEqual(A.declared_vertical_encoding(declared), 'motar-px4-vertical-v1')
        self.assertEqual(declared['odometry_source'], {'producer': 'motar_px4_gateway',
                                                       'vertical_encoding': 'motar-px4-vertical-v1'})
        for value in ({'producer': 'motar_px4_gateway', 'vertical_encoding': 'motar-px4-vertical-v2'},
                      {'producer': 'other_odometry', 'vertical_encoding': 'motar-px4-vertical-v1'},
                      dict(A.gateway_odometry_source(), extra=1), 'motar_px4_gateway', None, True):
            binding = F.ros_binding(self.path)
            binding['odometry_source'] = value
            with self.subTest(value=value), self.assertRaisesRegex(A.RosInputError, 'odometry_source'):
                A.validate_binding(binding)

    def frame(self, binding, odom, **kw):
        rgb, mm, K = outdoor_images()
        msgs = F.image_messages(3., rgb, mm, K)
        return A.build_frame(msgs['rgb'], msgs['depth'], msgs['camera_info'], odom, binding, **kw)

    def test_fields_reach_the_state_only_with_the_declaration(self):
        declared = declared_binding(self.path)
        frame, state = self.frame(declared, odometry(3.), vertical_encoding=A.VERTICAL_ENCODING)
        self.assertEqual({k: state[k] for k in FIELDS}, FIELDS)                            # positive control
        self.assertEqual(frame['vertical_provenance']['status'], 'FORWARDED')
        self.assertEqual(frame['vertical_provenance']['odometry_t'], [3.])
        frame, state = self.frame(declared, odometry(3., fields=None), vertical_encoding=A.VERTICAL_ENCODING)
        self.assertEqual(set(state), {'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 't'})
        self.assertEqual(frame['vertical_provenance']['status'], 'ABSENT')
        # Without vertical_encoding (indoor sessions, undeclared bindings) the covariance is never read: even a message
        # without covariance builds exactly the five-field state, and no provenance is added.
        frame, state = self.frame(F.ros_binding(self.path), F.odometry_message(3., [0., 0., 1.], 0., [0., 0., 0.], 0.))
        self.assertEqual(set(state), {'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 't'})
        self.assertNotIn('vertical_provenance', frame)
        frame, state = self.frame(declared, odometry(3.))
        self.assertEqual(set(state), {'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 't'})
        # An undeclared binding can never be asked for the fields.
        with self.assertRaisesRegex(A.RosInputError, 'odometry_source'):
            self.frame(F.ros_binding(self.path), odometry(3.), vertical_encoding=A.VERTICAL_ENCODING)
        with self.assertRaisesRegex(A.RosInputError, 'odometry_source'):
            self.frame(declared, odometry(3.), vertical_encoding='motar-px4-vertical-v2')

    def test_interpolated_pose_combines_both_samples_conservatively(self):
        declared = declared_binding(self.path)
        before = odometry(2.99, fields=dict(z_valid=True, epv=.1, dead_reckoning=False))
        after = odometry(3.01, fields=dict(z_valid=True, epv=.3, dead_reckoning=True))
        frame, state = self.frame(declared, before, odometry_bracket=(before, after), vertical_encoding=A.VERTICAL_ENCODING)
        self.assertTrue(frame['pose_provenance']['pose_interpolated'])
        self.assertEqual({k: state[k] for k in FIELDS}, {'z_valid': True, 'epv': .3, 'dead_reckoning': True})
        after = odometry(3.01, fields=dict(z_valid=False, epv=.05, dead_reckoning=False))
        _, state = self.frame(declared, before, odometry_bracket=(before, after), vertical_encoding=A.VERTICAL_ENCODING)
        self.assertEqual({k: state[k] for k in FIELDS}, {'z_valid': False, 'epv': .1, 'dead_reckoning': False})
        after = odometry(3.01, fields=None)                                    # one sample without fields: none
        frame, state = self.frame(declared, before, odometry_bracket=(before, after), vertical_encoding=A.VERTICAL_ENCODING)
        self.assertNotIn('epv', state)
        self.assertEqual(frame['vertical_provenance']['status'], 'ABSENT')
        self.assertEqual([s['status'] for s in frame['vertical_provenance']['samples']], ['FORWARDED', 'ABSENT'])


# ------------------------------------------------------------------- Goal 1: session and production outdoor runtime
class OutdoorFramesReachTheRuntime(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def session(self, declared=True, contract=None):
        runtime = OutdoorGeometryRuntime(contract)
        binding = (declared_binding if declared else F.ros_binding)(Path(self.tmp.name) / 'fixture.txt')
        return RosPolicySession(runtime, binding), runtime

    def test_gateway_to_policy_node_chain_accepts_outdoor_frames(self):
        """The full CPU chain: PX4 mappings -> BridgeSession.poll -> the gateway's Odometry fields -> the policy
        session (declared binding) -> the production outdoor preprocess, which accepts the ground with PX4's fields."""
        def chain(local):
            bridge = N.BridgeSession(bridge_config(), px4_binding())
            session, runtime = self.session()
            rgb, mm, K = outdoor_images()
            proposals = []
            for k in range(8):
                now, wall = 100. + k * .1, 20. + k * .1
                sample = px4_snapshot(now, wall, **local)
                for kind in ('odometry', 'local_position', 'status'):
                    bridge.offer(kind, sample[kind], receipt=wall)
                state, _ = bridge.poll(now=now, wall=wall)
                self.assertIsNotNone(state, bridge.fault)
                msgs = F.image_messages(state['t'], rgb, mm, K)
                msgs['odometry'] = gateway_odometry(state)
                for kind, message in msgs.items():
                    session.offer(kind, message, received_monotonic_s=wall)
                item = session.prepare(now_ros_s=now + .01, now_monotonic_s=wall + .01)
                proposals.append(session.finish(item, session.infer(item), now_ros_s=now + .01,
                                                now_monotonic_s=wall + .01))
            return proposals, runtime, session
        proposals, runtime, session = chain(dict(z_valid=True, epv=.2, dead_reckoning=False))
        self.assertEqual(len(runtime.grounds), 8)                                           # positive control
        self.assertTrue(all(g['accepted'] for g in runtime.grounds))
        self.assertTrue(all((s['z_valid'], s['epv'], s['dead_reckoning']) == (True, .2, False) for s in runtime.states))
        self.assertEqual([p.get('refusal') for p in proposals], [None] * 8)
        self.assertEqual([p['velocity_enu_mps'] for p in proposals[5:]], [[.5, 0., 0.]] * 3)  # after the re-entry hold
        self.assertEqual(session.diagnostics()['vertical_fields']['status'], 'FORWARDED')
        # Paired negative: a local position without epv/dead_reckoning -> nothing forwarded -> every frame refused.
        proposals, runtime, session = chain({})
        self.assertEqual([p['refusal'] for p in proposals], ['estimator_vertical_fields_absent'] * 8)
        self.assertEqual(runtime.grounds, [])
        self.assertEqual(session.diagnostics()['vertical_fields']['status'], 'ABSENT')

    def test_named_refusals_for_absent_malformed_undeclared_and_bad_values(self):
        pose, _ = A.encode_vertical_covariance(FIELDS)
        filler = list(pose)
        filler[7] = 0.
        cases = {
            'forwarded': (True, lambda t: odometry(t), None),
            'undeclared': (False, lambda t: odometry(t), 'estimator_vertical_fields_absent'),
            'absent': (True, lambda t: odometry(t, fields=None), 'estimator_vertical_fields_absent'),
            'malformed': (True, lambda t: odometry(t, pose_cov=filler), 'estimator_vertical_fields_absent'),
            'z invalid': (True, lambda t: odometry(t, fields=dict(FIELDS, z_valid=False)), 'estimator_z_invalid'),
            'epv large': (True, lambda t: odometry(t, fields=dict(FIELDS, epv=.9)), 'estimator_epv_exceeds_limit'),
            'dead reckoning': (True, lambda t: odometry(t, fields=dict(FIELDS, dead_reckoning=True)),
                               'estimator_dead_reckoning'),
        }
        accepted = 0
        for label, (declared, odom, reason) in cases.items():
            session, runtime = self.session(declared)
            proposals = run_ticks(session, [odom] * 3)
            with self.subTest(label=label):
                self.assertIsNone(session.fault)
                if reason is None:
                    self.assertEqual(len(runtime.grounds), 3)
                    self.assertTrue(all(g['accepted'] for g in runtime.grounds))
                    accepted += len(runtime.grounds)
                else:
                    self.assertEqual([p['refusal'] for p in proposals], [reason] * 3)
                    self.assertTrue(all(p['velocity_enu_mps'] == [0., 0., 0.] and p['hold'] for p in proposals))
                    self.assertEqual(session.status, 'HOLD_FRAME_REFUSED')
        self.assertEqual(accepted, 3)                                                     # positive control

    def test_indoor_session_never_reads_the_fields(self):
        runtime = G.BandRuntime()
        session = RosPolicySession(runtime, declared_binding(Path(self.tmp.name) / 'fixture.txt'))
        self.assertIsNone(session.vertical_encoding)
        self.assertIsNone(session.hold_limit_s)
        captured = []
        original = runtime.step
        runtime.step = lambda f, s, now_s: (captured.append(set(s)), original(f, s, now_s=now_s))[1]
        proposals = run_ticks(session, [lambda t: odometry(t)] * 3)
        self.assertEqual(captured, [{'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 't'}] * 3)
        self.assertEqual([p['velocity_enu_mps'] for p in proposals], [[.2, .1, 0.]] * 3)


# ------------------------------------------------------------------------------------- Goal 1: node configuration
class NodeNeedsTheDeclaration(unittest.TestCase):
    def test_outdoor_contract_loads_only_with_the_declared_gateway_odometry(self):
        if not O.CHECKPOINT_DIR.is_dir():
            self.skipTest('local trained camera checkpoint not distributed in the public repository')
        import camera_ros2_node as node
        self.assertFalse(node.ROS_PATH_FORWARDS_VERTICAL_FIELDS)            # the undeclared path still forwards none
        outdoor = C.build_contract(1, outdoor=C.outdoor_blocks())
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            evidence = tmp / 'estimator_height_fixture.txt'
            evidence.write_text('CPU test fixture, not evidence')
            indoor = C.build_contract(1, estimator_height=C.estimator_height_assertion(evidence))
            results = {}
            for name, contract, declared in (('outdoor_declared', outdoor, True), ('outdoor_plain', outdoor, False),
                                             ('indoor_declared', indoor, True), ('indoor_plain', indoor, False)):
                path = tmp / ('%s.json' % name)
                path.write_text(json.dumps(contract))
                binding = (declared_binding if declared else F.ros_binding)(tmp / 'binding_fixture.txt')
                config = {'format': 'motar-ros2-node-v1', 'binding': binding,
                          'outputs': {'velocity_topic': '/motar/policy/velocity_enu',
                                      'diagnostic_topic': '/motar/policy/diagnostics'},
                          'policy_contract_path': str(path),
                          'policy_contract_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                config_path = tmp / ('%s_node.json' % name)
                config_path.write_text(json.dumps(config))
                try:
                    results[name] = node.load_node_config(config_path, ROOT)[1]
                except ValueError as exc:
                    results[name] = str(exc)
                with contextlib.redirect_stdout(io.StringIO()) as stdout:
                    code = node.main(config_path=config_path, source_root=ROOT, check_only=True)
                results[name + '_check'] = (code, json.loads(stdout.getvalue()))
            bad = dict(config, binding=dict(declared_binding(tmp / 'binding_fixture.txt'),
                                            odometry_source={'producer': 'motar_px4_gateway',
                                                             'vertical_encoding': 'motar-px4-vertical-v2'}))
            bad['policy_contract_path'] = str(tmp / 'outdoor_declared.json')
            bad['policy_contract_sha256'] = hashlib.sha256((tmp / 'outdoor_declared.json').read_bytes()).hexdigest()
            (tmp / 'bad_node.json').write_text(json.dumps(bad))
            with self.assertRaisesRegex(ValueError, 'odometry_source'):
                node.load_node_config(tmp / 'bad_node.json', ROOT)
        self.assertIsInstance(results['outdoor_declared'], dict)                            # positive control
        code, summary = results['outdoor_declared_check']
        self.assertEqual((code, summary['status'], summary['profile']), (0, 'CONFIG_VALID', 'outdoor'))
        self.assertEqual(summary['odometry_source'], A.gateway_odometry_source())
        self.assertEqual(summary['refusal_hold_max_s'], {'value': 3.0, 'status': 'PROVISIONAL'})
        self.assertTrue(results['outdoor_plain'].startswith('UNCONFIGURED: ' + node.OUTDOOR_NODE_REFUSAL))
        self.assertEqual((results['outdoor_plain_check'][0], results['outdoor_plain_check'][1]['status']),
                         (2, 'NOT_CONFIGURED'))
        for name in ('indoor_declared', 'indoor_plain'):
            self.assertIsInstance(results[name], dict, name)
            self.assertEqual(results[name + '_check'][1]['status'], 'CONFIG_VALID')
            self.assertNotIn('profile', results[name + '_check'][1])


# --------------------------------------------------------------------------------- Goal 2: contract field
class HoldLimitContractField(unittest.TestCase):
    def outdoor(self, value=None):
        contract = C._metadata(1, ROOT)
        contract['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        contract.update(C.outdoor_blocks())
        if value is not None:
            contract['ground_estimate']['refusal_hold_max_s'] = value
        return contract

    def test_default_is_provisional_three_seconds_within_inclusive_bounds(self):
        self.assertEqual(C.GROUND_ESTIMATE_OUTDOOR_DEFAULT['refusal_hold_max_s'], {'value': 3.0, 'status': 'PROVISIONAL'})
        self.assertEqual(C.GROUND_ESTIMATE_BOUNDS['refusal_hold_max_s'], (1.0, 10.0, False))
        self.assertIn('refusal_hold_max_s', C.INCLUSIVE_LOW_BOUNDS)
        accepted = 0
        for value in (1.0, 3.0, 10.0, 1, 10, 5.5):
            contract = self.outdoor(C.provisional(value))
            with self.subTest(value=value):
                self.assertEqual(C.validate_contract(contract, verify_files=False), [])
                self.assertEqual(R.refusal_hold_limit_s(contract), float(value))
                accepted += 1
        self.assertEqual(accepted, 6)                                                     # positive control
        bad = [C.provisional(.999), C.provisional(10.001), C.provisional(0.), C.provisional(-3.),
               C.provisional(float('nan')), C.provisional(float('inf')), C.provisional(True), C.provisional('3.0'),
               {'value': 3.0, 'status': 'CALIBRATED'}, 3.0, {'value': 3.0}, {'value': 3.0, 'status': 'PROVISIONAL', 'x': 1}]
        for value in bad:
            contract = self.outdoor(value)
            with self.subTest(value=value):
                self.assertTrue(any('refusal_hold_max_s' in p for p in C.validate_contract(contract, verify_files=False)))
                with self.assertRaisesRegex(ValueError, 'refusal_hold_max_s'):
                    R.refusal_hold_limit_s(contract)
        missing = self.outdoor()
        del missing['ground_estimate']['refusal_hold_max_s']
        self.assertTrue(C.validate_contract(missing, verify_files=False))
        with self.assertRaisesRegex(ValueError, 'refusal_hold_max_s'):
            R.refusal_hold_limit_s(missing)
        # Indoor contracts carry no limit and need none (control: still valid and unchanged).
        indoor = C._metadata(1, ROOT)
        indoor['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        self.assertEqual(C.validate_contract(indoor, verify_files=False), [])
        self.assertIsNone(R.refusal_hold_limit_s(indoor))
        self.assertIsNone(R.refusal_hold_limit_s(F.contract(ground=True)))

    def test_session_refuses_an_outdoor_contract_without_a_valid_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime = O.RefusalRuntime()
            del runtime.contract['ground_estimate']['refusal_hold_max_s']
            with self.assertRaisesRegex(ValueError, 'refusal_hold_max_s'):
                RosPolicySession(runtime, F.ros_binding(Path(tmp) / 'fixture.txt'))
            runtime = O.RefusalRuntime()                                                  # positive control
            session = RosPolicySession(runtime, F.ros_binding(Path(tmp) / 'fixture.txt'))
            self.assertEqual(session.hold_limit_s, 3.0)


# ------------------------------------------------------------------------------------- Goal 2: session timeout
class BoundedRefusalHold(unittest.TestCase):
    tick = G.AltitudeGating.tick

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.make()

    def make(self, limit=None):
        self.runtime = O.RefusalRuntime()
        if limit is not None:
            self.runtime.contract['ground_estimate']['refusal_hold_max_s'] = C.provisional(limit)
        self.session = RosPolicySession(self.runtime, F.ros_binding(Path(self.tmp.name) / 'fixture.txt'))
        self.runtime.events.clear()
        self.k = 0

    def start(self):
        """Session start: the re-entry hold (5 frames), then the first policy command (episode ended)."""
        for _ in range(R.REFUSAL_REENTRY_FRAMES):
            self.assertTrue(self.tick(1.)[0]['hold'])
        proposal, _ = self.tick(1.)
        self.assertEqual(proposal['velocity_enu_mps'], [.8, -.4, 0.])
        self.assertIsNone(self.session.hold_episode_s)

    def hold_until_timeout(self, limit_ticks):
        """Refuse every frame; returns the number of held proposals before HoldTimeout."""
        self.runtime.refuse = 'ground_not_found'
        held = 0
        with self.assertRaises(HoldTimeout) as raised:
            for _ in range(limit_ticks + 5):
                proposal, _ = self.tick(1.)
                self.assertTrue(proposal['hold'])
                self.assertAlmostEqual(proposal['hold_episode_s'], held * .1, places=6)
                held += 1
        return held, raised.exception

    def test_timeout_fires_at_the_limit_and_not_before(self):
        self.start()
        held, exc = self.hold_until_timeout(30)
        self.assertEqual(held, 30)                                   # 0.0 .. 2.9 s held; the 3.0 s tick latches
        self.assertEqual(exc.reason, 'refusal_hold_timeout')
        record = self.session.hold_timeout
        self.assertAlmostEqual(record['episode_s'], 3.0, places=6)
        self.assertEqual((record['limit_s'], record['limit_status'], record['held_proposals']), (3.0, 'PROVISIONAL', 30))
        self.assertEqual((record['refused_frames'], record['last_refusal']), (31, 'ground_not_found'))
        self.assertEqual(self.session.status, 'FAULT')
        self.assertTrue(self.session.fault.startswith('refusal_hold_timeout'))
        self.assertTrue(self.runtime.failed)
        diag = self.session.diagnostics()
        # The node publishes this dict plus last_error: every key it published before unit B is still there.
        self.assertLessEqual({'state', 'altitude', 'epoch', 'sequence', 'vehicle_commands_sent', 'refusal',
                              'consecutive_refusals', 'reentry_hold_remaining'}, set(diag))
        self.assertEqual(diag['state'], 'FAULT')
        self.assertEqual(diag['hold_timeout']['reason'], 'refusal_hold_timeout')
        self.assertEqual((diag['hold_limit_s'], diag['hold_limit_status']), (3.0, 'PROVISIONAL'))
        json.dumps(diag, allow_nan=False)
        for limit, ticks in ((1.0, 10), (10.0, 100)):                      # the contract bounds
            self.make(limit)
            self.start()
            with self.subTest(limit=limit):
                self.assertEqual(self.hold_until_timeout(ticks)[0], ticks)

    def test_latch_needs_an_explicit_fresh_session(self):
        self.start()
        self.hold_until_timeout(30)
        self.runtime.refuse = None
        for _ in range(3):                                                  # no automatic re-entry: nothing proposed
            with self.assertRaises(SessionFault):
                self.tick(1.)
        self.assertEqual(self.session.status, 'FAULT')
        epoch = self.session.epoch
        self.session.reset()                                                # the explicit fresh session
        self.assertGreater(self.session.epoch, epoch)
        self.assertIsNone(self.session.hold_timeout)
        self.k += 3
        self.start()                                                        # positive control: commands again
        self.assertEqual(self.session.status, 'RUNNING_PROPOSALS')

    def test_intermittent_refusals_do_not_restart_the_episode(self):
        """review r3: a refusal every 3rd frame held forever. Now the episode runs on and times out."""
        self.start()
        held = 0
        with self.assertRaises(HoldTimeout):
            for k in range(60):
                self.runtime.refuse = 'ground_ambiguous' if k % 3 == 0 else None
                proposal, _ = self.tick(1.)
                self.assertTrue(proposal['hold'])                          # never a policy command in between
                held += 1
        self.assertEqual(held, 30)
        self.assertEqual(self.session.hold_timeout['refused_frames'], 11)
        # Control: with a policy command between the refusals each episode stays short and never times out.
        self.make()
        self.start()
        commands = 0
        for k in range(60):
            self.runtime.refuse = 'ground_ambiguous' if k % 7 == 0 else None
            proposal, _ = self.tick(1.)
            commands += not proposal.get('hold', False)
            self.assertLess(self.session.hold_episode_s or 0., .6)
        self.assertGreater(commands, 0)
        self.assertIsNone(self.session.fault)

    def test_a_policy_command_ends_the_episode(self):
        self.start()
        self.runtime.refuse = 'ground_not_found'
        for _ in range(25):
            self.tick(1.)
        self.assertAlmostEqual(self.session.hold_episode_s, 2.4, places=6)
        self.runtime.refuse = None
        for _ in range(R.REFUSAL_REENTRY_FRAMES):                            # re-entry holds continue the episode
            self.assertTrue(self.tick(1.)[0]['hold'])
        self.assertAlmostEqual(self.session.hold_episode_s, 2.9, places=6)
        proposal, _ = self.tick(1.)                                          # the policy command: episode over
        self.assertEqual(proposal['velocity_enu_mps'], [.8, -.4, 0.])
        self.assertIsNone(self.session.hold_episode_s)
        self.runtime.refuse = 'ground_not_found'
        for _ in range(25):                                                  # a new episode starts from 0
            proposal, _ = self.tick(1.)
        self.assertAlmostEqual(proposal['hold_episode_s'], 2.4, places=6)
        self.assertIsNone(self.session.fault)

    def test_altitude_gated_ticks_do_not_end_the_episode(self):
        self.start()
        self.runtime.refuse = 'ground_not_found'
        for _ in range(20):                                                  # held 0.0 .. 1.9 s
            self.tick(1.)
        for _ in range(5):                                                   # gated: no proposal, no reset of the timer
            self.assertIsNone(self.tick(0.)[0])
        self.assertEqual(self.session.status, 'WAITING_ALTITUDE')
        held = 0
        with self.assertRaises(HoldTimeout):
            for _ in range(10):
                self.tick(1.)
                held += 1
        self.assertEqual(held, 5)                                            # 2.5 .. 2.9 s held; 3.0 s latches
        self.assertAlmostEqual(self.session.hold_timeout['episode_s'], 3.0, places=6)

    def test_indoor_sessions_never_hold_or_time_out(self):
        runtime = G.BandRuntime()
        session = RosPolicySession(runtime, F.ros_binding(Path(self.tmp.name) / 'fixture_indoor.txt'))
        self.session, self.runtime, self.k = session, runtime, 0
        self.assertIsNone(session.hold_limit_s)
        commands = 0
        for _ in range(110):                                                 # 11 s > the 10 s upper bound
            proposal, _ = self.tick(1.)
            self.assertNotIn('hold', proposal)
            commands += 1
        self.assertEqual(commands, 110)                                       # positive count
        self.assertIsNone(session.hold_episode_s)
        self.assertIsNone(session.hold_timeout)
        self.assertEqual(session.status, 'RUNNING_PROPOSALS')
        self.assertEqual(session.diagnostics()['hold_limit_status'], None)
        # Fail closed: a hold in a session without a limit (never produced by an indoor runtime) faults, never unbounded.
        class Refusing(G.BandRuntime):
            def step(self, frame, state, *, now_s):
                raise R.FrameRefused('ground_not_found')
        self.session = RosPolicySession(Refusing(), F.ros_binding(Path(self.tmp.name) / 'fixture_indoor2.txt'))
        self.runtime, self.k = self.session.runtime, 0
        with self.assertRaisesRegex(ValueError, 'without a refusal-hold limit'):
            self.tick(1.)
        self.assertEqual(self.session.status, 'FAULT')

    def test_the_gateway_revokes_on_its_stale_path_after_the_last_hold(self):
        gate, _ = G.activate(G.fence())
        for k in range(1, 6):                                                # zero-hold proposals reach the gateway
            payload = G.send(gate, k, (0., 0.), (0., 0.), yaw=0.)
            self.assertEqual(G.enu_velocity(payload), (0., 0.))
        self.assertEqual(gate.state, 'ACTIVE')                               # positive control
        revoked_after = None
        for k in range(6, 10):                                               # timeout: no further proposal
            now, mono = 101.1 + k / 10, 21.1 + k / 10
            payload = gate.update(G.snap(now, mono, (0., 0.), nav=P.OFFBOARD), None, now_s=now, now_monotonic_s=mono)
            if payload is None:
                revoked_after = k - 5
                break
        self.assertEqual(revoked_after, 2)                                   # 0.2 s after the last held proposal
        self.assertEqual(gate.state, 'REVOKED')
        self.assertIn('stale', gate.reason)


class RefusalHoldEpisodeUnit(unittest.TestCase):
    def test_episode_rules(self):
        episode = R.RefusalHoldEpisode(1.0)
        results = [episode.hold(10. + .1 * k) for k in range(11)]
        self.assertEqual([r[1] for r in results], [False] * 10 + [True])
        episode.command()
        self.assertEqual(episode.hold(12.)[0], 0.)
        with self.assertRaisesRegex(ValueError, 'increase'):
            episode.hold(12.)
        with self.assertRaisesRegex(ValueError, 'without a refusal-hold limit'):
            R.RefusalHoldEpisode(None).hold(1.)
        for bad in (0., -1., float('nan'), True, '3'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                R.RefusalHoldEpisode(bad)


# ------------------------------------------------------------------------------------------- Goal 2: replays
class ReplaysReportTheTimeout(unittest.TestCase):
    def npz(self, n):
        return {'camera_metadata': np.array(json.dumps({'intrinsics': F.TRAINED})), 'rgb': np.zeros((n, 2, 2, 3), np.uint8),
                'depth_z_m': np.zeros((n, 2, 2), np.float32), 'invalid': np.zeros((n, 2, 2), bool),
                'rgb_t': np.arange(n) * .1, 'depth_t': np.arange(n) * .1, 'pos': np.zeros((n, 3)), 'vel': np.zeros((n, 3)),
                'quat_xyzw': np.tile([0., 0., 0., 1.], (n, 1)), 'yaw_rate_rps': np.zeros(n), 'state_t': np.arange(n) * .1,
                'now_s': np.arange(n) * .1, 'z_valid': np.ones(n, bool), 'epv': np.full(n, .1),
                'dead_reckoning': np.zeros(n, bool)}

    def replay(self, refuse_every):
        class Fake:
            contract = O.outdoor_contract()
            def __init__(self): self.calls = 0
            def reset(self): pass
            def applied_zero_hold(self): pass
            def step(self, frame, state, *, now_s):
                self.calls += 1
                if refuse_every == 1 or self.calls % refuse_every == 1:
                    raise R.FrameRefused('ground_not_found')
                return {'t': float(now_s), 'velocity_world_enu_mps': [1., 0., 0.], 'yaw_rate_rps': .2}
        with tempfile.TemporaryDirectory() as tmp:
            report = R.replay(Fake(), self.npz(40), Path(tmp) / 'out.jsonl')
            lines = [json.loads(x) for x in (Path(tmp) / 'out.jsonl').read_text().splitlines()]
        return report, lines

    def test_npz_replay_marks_where_the_session_would_latch(self):
        report, lines = self.replay(1)                                       # every frame refused
        self.assertEqual(report['refusal_hold_max_s'], 3.0)
        self.assertAlmostEqual(report['refusal_hold_timeout_capture_s'], 3.0, places=6)
        marked = [k for k, x in enumerate(lines) if x.get('refusal_hold_timeout')]
        self.assertEqual(marked, [30])
        self.assertAlmostEqual(lines[29]['hold_episode_s'], 2.9, places=6)
        report, lines = self.replay(9)                                       # control: commands between refusals
        self.assertIsNone(report['refusal_hold_timeout_capture_s'])
        self.assertGreater(sum(1 for x in lines if not x.get('hold')), 0)
        self.assertLess(report['longest_hold_episode_s'], 1.)

    def test_oakd_replay_reports_the_timeout_frame(self):
        import oakd_outdoor_replay as T
        contract = C._metadata(1, ROOT)
        contract['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        contract.update(C.outdoor_blocks())
        contract['ground_estimate']['refusal_hold_max_s'] = C.provisional(1.0)
        self.assertEqual(C.validate_contract(contract, verify_files=False), [])
        with tempfile.TemporaryDirectory() as tmp:
            meta = O.write_recording(Path(tmp) / 'rec', [({'objects': [O.TREE_LINE]}, 1., 0., 0.)] * 12)
            rows, summary = T.replay_recording(Path(tmp) / 'rec', contract)      # control: accepted frames
            self.assertEqual(summary['accepted'], 12)
            self.assertIsNone(summary['refusal_hold_timeout'])
            self.assertAlmostEqual(summary['longest_hold_episode_s'], .4, places=6)
            self.assertEqual([r['hold_episode_s'] is None for r in rows], [False] * 5 + [True] * 7)
            # epv above the contract's epv_max_m: every frame refused, so the session would latch at 1.0 s.
            (Path(tmp) / 'rec' / 'recording.json').write_text(json.dumps(dict(meta, camera_height_uncertainty_m=.9)))
            rows, summary = T.replay_recording(Path(tmp) / 'rec', contract)
        self.assertEqual(summary['refusal_reasons'], {'estimator_epv_exceeds_limit': 12})
        self.assertEqual(summary['refusal_hold_max_s'], 1.0)
        self.assertEqual(summary['refusal_hold_timeout']['index'], 10)
        self.assertAlmostEqual(summary['refusal_hold_timeout']['t'], 11.0, places=6)
        self.assertEqual([k for k, r in enumerate(rows) if r.get('refusal_hold_timeout')], [10])


# --------------------------------------------------------------------------------------------------- records
class UnitBRecords(unittest.TestCase):
    def test_readme_describes_the_encoding_declaration_and_bounded_hold(self):
        text = (ROOT / 'ros2/motar_camera_policy/README.md').read_text()
        section = text[text.index('## Outdoor profile (2026-11-13, opt-in)'):]
        for term in ('motar-px4-vertical-v1', '`pose.covariance[0]`', 'tag -20261118.0', '"odometry_source"',
                     '"producer": "motar_px4_gateway"', '`refusal_hold_timeout`', '`hold_episode_s`',
                     '`hold_timeout`', 'COM_OBL_RC_ACT', '`~/reset_policy_session`', 'READ_FIELD_TYPES',
                     'outdoor_vertical_fields_not_forwarded', 'estimator_vertical_fields_absent'):
            self.assertIn(term, section)
        self.assertEqual(A.VERTICAL_ENCODING_TAG, -20261118.0)
        self.assertIn('| ground_estimate | refusal_hold_max_s | %s |'
                      % C.threshold_value(C.GROUND_ESTIMATE_OUTDOOR_DEFAULT, 'refusal_hold_max_s'), section)
        low, high, _ = C.GROUND_ESTIMATE_BOUNDS['refusal_hold_max_s']
        self.assertIn('bounds [%s, %s] s' % (low, high), section)
        self.assertIn('[%s, %s] s, both ends' % (low, high), section)
        self.assertNotIn('The hold has\nno time limit', section)
        evidence = (ROOT / 'docs/evidence/sitl_2026-11-05/camera_runtime/README.md').read_text()
        length, digest = O.PRE_README['docs/evidence/sitl_2026-11-05/camera_runtime/README.md']
        self.assertEqual(hashlib.sha256(evidence[:length].encode()).hexdigest(), digest)
        unit = evidence[evidence.index('## Unit B, 2026-11-18'):]
        for term in ('motar-px4-vertical-v1', 'refusal_hold_timeout', 'PROVISIONAL', 'NOT_TESTED'):
            self.assertIn(term, unit)
        self.assertIn('%s s by default (PROVISIONAL, bounds [%s, %s] s)'
                      % (C.threshold_value(C.GROUND_ESTIMATE_OUTDOOR_DEFAULT, 'refusal_hold_max_s'), low, high),
                      unit.replace('\n  ', ' '))


if __name__ == '__main__':
    unittest.main()
