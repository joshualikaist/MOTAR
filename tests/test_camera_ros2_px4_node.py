"""Bridge composition controls without ROS, DDS, vehicle or flight execution."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import camera_ros2_px4 as P
import camera_ros2_px4_node as N
from test_camera_ros2_px4 import snapshot, command


def binding():
    return P.Px4MessageBinding(P.PX4_VERSION, P.PX4_MESSAGE_SHA256, 'CPU software fixture, not hardware qualification')


def config(enabled=False):
    value = N.unconfigured_template()
    value.update(status='CONFIGURED', enable_offboard_publication=enabled)
    value['frames'] = dict(world='map', body='base_link')
    if enabled:
        # Repair r1 (F8): publication needs an enabled geofence or an explicit waiver. These bridge tests
        # do not exercise the fence (test_camera_runtime_fixes_r2 / _repair_r1 do).
        value['geofence_waiver'] = dict(reason='CPU bridge fixture; geofence tested separately')
    return value


def offer(session, now=100., wall=20., **changes):
    sample = snapshot(now, wall, **changes)
    for kind in ('odometry', 'local_position', 'status'):
        session.offer(kind, sample[kind], receipt=wall)
    return sample


class BridgeTests(unittest.TestCase):
    def test_observation_has_no_offboard_payload_and_preserves_measured_capture(self):
        session = N.BridgeSession(config(), binding())
        self.assertEqual(session.poll(now=100., wall=20.), (None, None))
        offer(session)
        state, payload = session.poll(now=100., wall=20.)
        self.assertAlmostEqual(state['t'], 99.995)
        self.assertEqual(state['pos'], [20.,10.,3.4])
        self.assertIsNone(payload)
        self.assertEqual(session.poll(now=100.02, wall=20.02), (None, None))
        with self.assertRaisesRegex(ValueError, 'observation only'):
            session.prime(now=100.02, wall=20.02)

    def test_manual_handover_tracks_prime_z_then_holds_transition_z(self):
        session = N.BridgeSession(config(True), binding())
        offer(session, z=-1.)
        state, _ = session.poll(now=100., wall=20.)
        self.assertIsNotNone(state)
        initial = session.prime(now=100., wall=20.)
        self.assertEqual(initial['trajectory_setpoint']['position'][2], -1.)
        pairs = [initial]
        for i in range(1, 14):
            now, wall = 100.+i*.1, 20.+i*.1
            offer(session, now, wall, z=-1.-i*.1, nav=2 if i<11 else 14)
            session.offer_command(command(now), receipt=wall)
            state, pair = session.poll(now=now, wall=wall)
            self.assertIsNotNone(state)
            self.assertIsNotNone(pair, session.fault)
            pairs.append(pair)
            expected = -1.-min(i,11)*.1
            self.assertAlmostEqual(pair['trajectory_setpoint']['position'][2], expected)
        self.assertEqual(session.gate.state, 'ACTIVE')
        self.assertEqual(len(pairs), 14)
        self.assertEqual(pairs[-1]['trajectory_setpoint']['velocity'][:2], [2.,1.])
        session.revoke('operator stop')
        self.assertEqual(session.poll(now=101.4, wall=21.4), (None,None))

    def test_prime_rejects_disarmed_wrong_mode_and_implicit_intent(self):
        for changes in (dict(armed=1), dict(nav=0), dict(nav=14)):
            session = N.BridgeSession(config(True), binding())
            offer(session, **changes)
            with self.assertRaises(ValueError):
                session.prime(now=100., wall=20.)
            self.assertEqual(session.gate.state, 'REVOKED')
        gate = P.Px4OffboardGateway(binding(), command_frame_id='map', geofence=P.geofence_template())
        with self.assertRaisesRegex(ValueError, 'explicit operator'):
            gate.prime(snapshot(), now_s=100., now_monotonic_s=20.)

    def test_reset_stops_odometry_before_rebased_pose_can_escape(self):
        session = N.BridgeSession(config(), binding())
        offer(session)
        self.assertIsNotNone(session.poll(now=100., wall=20.)[0])
        newer = snapshot(100.1,20.1)
        newer['odometry']['reset_counter'] = 1
        newer['odometry']['position'][0] = 1000.
        for kind in ('odometry','local_position','status'):
            session.offer(kind,newer[kind],receipt=20.1)
        self.assertEqual(session.poll(now=100.1, wall=20.1), (None,None))
        self.assertIn('estimator reset',session.fault)

    def test_duplicate_capture_cannot_renew_receipt_and_changed_payload_revokes(self):
        session = N.BridgeSession(config(),binding())
        sample = offer(session)
        session.poll(now=100.,wall=20.)
        sample['odometry']['timestamp'] += 100000
        self.assertFalse(session.offer('odometry',sample['odometry'],receipt=20.1))
        self.assertEqual(session.receipts['odometry'],20.)
        sample['odometry']['position'][0] += 1
        self.assertFalse(session.offer('odometry',sample['odometry'],receipt=20.1))
        self.assertEqual(session.poll(now=100.1,wall=20.1),(None,None))
        self.assertIn('changed payload',session.fault)

    def test_stale_clock_jump_mode_loss_and_disarm_stop_both_outputs(self):
        for fault in ('stale','clock','disarm'):
            with self.subTest(fault=fault):
                session=N.BridgeSession(config(True),binding())
                offer(session)
                session.poll(now=100.,wall=20.)
                session.prime(now=100.,wall=20.)
                if fault=='disarm':
                    offer(session,100.1,20.1,armed=1)
                now,wall = (100.1,20.2) if fault=='clock' else (100.2,20.2) if fault=='stale' else (100.1,20.1)
                self.assertEqual(session.poll(now=now,wall=wall),(None,None))
                self.assertIsNotNone(session.fault)
                self.assertEqual(session.gate.state,'REVOKED')

    def test_duplicate_policy_does_not_renew_and_wrong_frame_rejected_at_handover(self):
        session=N.BridgeSession(config(True),binding())
        c=command(100.)
        session.offer_command(c,receipt=20.)
        session.offer_command(c,receipt=20.1)
        self.assertEqual(session.command_receipt,20.)
        c['velocity_enu_mps'][0] += 1.
        session.offer_command(c,receipt=20.1)
        self.assertIn('payload changed',session.fault)


class ConfigurationTests(unittest.TestCase):
    def test_software_message_hash_binding_accepts_source_and_rejects_tamper(self):
        source=Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/msg')
        if not source.is_dir(): self.skipTest('local reference source unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            for name in P.PX4_MESSAGE_SHA256:
                (tmp/name).write_bytes((source/name).read_bytes())
            value=config()
            value['px4_msgs_msg_dir']=str(tmp)
            value['firmware']['source']='CPU fixture source schema only, clock/hardware NOT_TESTED'
            path=tmp/'config.json'
            path.write_text(json.dumps(value))
            configured,b=N.load_node_config(path,ROOT)
            self.assertFalse(configured['enable_offboard_publication'])
            self.assertEqual(b.metadata()['dds_delivery'],'NOT_TESTED')
            value['enable_offboard_publication']=True
            value['geofence_waiver']=dict(reason='CPU fixture')
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'flight-check'):
                N.load_node_config(path,ROOT)
            (tmp/'VehicleStatus.msg').write_text('tampered')
            with self.assertRaisesRegex(ValueError,'differs'):
                N.verify_message_definitions(tmp)

    def test_generated_type_shape_matches_bound_definition_and_refuses_different_field(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'msg.msg'
            p.write_text('uint64 timestamp\nbool valid\nfloat32[3] velocity\nuint8 CONSTANT=2 # ignored\n')
            class Good:
                @staticmethod
                def get_fields_and_field_types():
                    return dict(timestamp='uint64',valid='boolean',velocity='float[3]')
            N.validate_generated_type(Good,p)
            class Wrong:
                @staticmethod
                def get_fields_and_field_types():
                    return dict(timestamp='uint64',valid='boolean',velocity='float[4]')
            with self.assertRaisesRegex(ValueError,'generated'):
                N.validate_generated_type(Wrong,p)

    def test_unconfigured_gateway_entry_never_imports_ros_or_policy(self):
        package=ROOT/'ros2/motar_camera_policy/motar_camera_policy'
        code='''
import importlib.abc,sys
class Deny(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,path=None,target=None):
  if fullname.split('.')[0] in {'rclpy','torch','numpy','px4_msgs','depthai','pymavlink'}: raise AssertionError(fullname)
sys.meta_path.insert(0,Deny())
from entry import gateway_main
raise SystemExit(gateway_main(sys.argv[1:]))
'''
        result=subprocess.run([sys.executable,'-B','-c',code,'--source-root',str(ROOT),
            '--config',str(ROOT/'ros2/motar_camera_policy/config/px4_unconfigured.json'),'--check'],
            cwd=package,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=10)
        self.assertEqual(result.returncode,2,result.stderr)
        self.assertEqual(json.loads(result.stdout)['ros_graph'],'NOT_STARTED')


if __name__=='__main__':
    unittest.main()
