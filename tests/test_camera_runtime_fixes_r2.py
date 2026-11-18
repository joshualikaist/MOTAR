"""Camera-runtime deployment fixes r2 (2026-11-12): gateway geofence, ROS coverage, REP 117 depth, altitude gating.

G: the PX4 gateway limits the published XY velocity to a measured inner box (the trained arena had no walls).
H: the ROS image path derives source coverage from CameraInfo K, D, R, P, as tools/oakd_camera.py does on device.
I: 32FC1 +Inf is no-return (REP 117), -Inf and NaN stay sensor-invalid, 16UC1 0 stays invalid.
J: outside the floor/ceiling filter band the ROS session waits (WAITING_ALTITUDE) without calling the runtime.

Every check has a control. Synthetic scenes and messages only: nothing here is real-sensor, PX4, ROS-graph,
Gazebo or flight evidence. The geofence latency values are test fixtures, not measurements.
"""
import contextlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'tests'))
import camera_policy_runtime as R
import camera_ros2_adapter as A
import camera_ros2_px4 as P
import camera_ros2_px4_node as N
from camera_ros2_session import RosPolicySession, SessionFault
import test_camera_runtime_fixes as F
from test_camera_ros2_px4 import snapshot, command

CHECKPOINT_DIR = Path('/home/fair/motar_tmp/camk2v3_20261108/train_s1/checkpoint')
PX4_MSG_SOURCE = Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/msg')
A_BRAKE = P.GOVERNOR_BRAKE_MPS2


def _load(name, path, register=False):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    if register:   # dataclasses with postponed annotations need their (private) module registered
        sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# Production blob_near, loaded under a private name (no aerial_gym package import in the shared process).
DV = _load('depth_validity_r2_under_test', ROOT / 'aerial_gym/task/navrl_task/navrl_depth_validity.py')


# ------------------------------------------------------------------------------------------------ G geofence
def binding():
    return P.Px4MessageBinding(P.PX4_VERSION, P.PX4_MESSAGE_SHA256, 'CPU software fixture, not hardware qualification')


def fence(latency=.2, box=(-5., 5., -5., 5.), **changes):
    block = dict(P.geofence_template(), enabled=True, x_min_m=box[0], x_max_m=box[1], y_min_m=box[2],
                 y_max_m=box[3], latency_s=latency)
    block.update(changes)
    return block


def snap(now, mono, xy, *, nav=2, z=-3.4, stale_odometry=False):
    """snapshot() with the vehicle at ENU (x east, y north); PX4 NED position is (north, east, down)."""
    s = snapshot(now, mono, nav=nav, z=z)
    s['odometry']['position'] = [float(xy[1]), float(xy[0]), z]
    if stale_odometry:
        s['receipts']['odometry'] = mono - .2
    return s


def cmd(now, v=(1., 2.), yaw=.3):
    return dict(command(now), velocity_enu_mps=[float(v[0]), float(v[1]), 0.], yaw_rate_enu_rps=yaw)


def activate(geofence, xy=(0., 0.), first=None):
    gate = P.Px4OffboardGateway(binding(), command_frame_id='map', geofence=geofence)
    gate.prime(snap(100., 20., xy), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
    for i in range(1, 11):
        now, mono = 100. + i / 10, 20. + i / 10
        if gate.update(snap(now, mono, xy), now_s=now, now_monotonic_s=mono) is None:
            raise AssertionError(gate.reason)
    result = gate.update(snap(101.1, 21.1, xy, nav=P.OFFBOARD), first or cmd(101.1),
                         command_received_monotonic_s=21.1, now_s=101.1, now_monotonic_s=21.1)
    if gate.state != 'ACTIVE' or result is None:
        raise AssertionError(gate.reason)
    return gate, result


def send(gate, k, xy, v=(1., 2.), yaw=.3, **kw):
    """k-th update after activation (10 Hz), carrying a fresh command."""
    now, mono = 101.1 + k / 10, 21.1 + k / 10
    return gate.update(snap(now, mono, xy, nav=P.OFFBOARD, **kw), cmd(now, v, yaw),
                       command_received_monotonic_s=mono, now_s=now, now_monotonic_s=mono)


def enu_velocity(payload):
    north, east, _ = payload['trajectory_setpoint']['velocity']
    return east, north


def bits(value):
    """Bit-exact view of a payload (NaN-safe): every float becomes its IEEE-754 bytes."""
    if isinstance(value, float):
        return struct.pack('<d', value).hex()
    if isinstance(value, (list, tuple)):
        return [bits(v) for v in value]
    if isinstance(value, dict):
        return {k: bits(v) for k, v in value.items()}
    return value


def v_max(d, a, tau):
    return -a * tau + math.sqrt(a * a * tau * tau + 2. * a * d)


def stop_overshoot(distance, speed, tau, a, dt=1e-3):
    """Double-integrator rollout: hold `speed` for the latency, then brake at a. Exact constant-acceleration
    steps; returns the farthest travel minus `distance` (> 0 means the wall was crossed)."""
    x, t, v = 0., 0., float(speed)
    farthest = 0.
    while t < tau - 1e-15:
        step = min(dt, tau - t)
        x, t = x + v * step, t + step
        farthest = max(farthest, x)
    while v > 0.:
        step = min(dt, v / a)
        x, v = x + v * step - .5 * a * step * step, max(0., v - a * step)
        farthest = max(farthest, x)
        if v < 1e-12:
            break
    return farthest - distance


class GeofenceConfiguration(unittest.TestCase):
    def test_template_is_disabled_without_stand_in_values_and_decel_is_the_trained_governor(self):
        template = N.unconfigured_template()['geofence']
        self.assertEqual(template, P.geofence_template())
        on_disk = json.loads((ROOT / 'ros2/motar_camera_policy/config/px4_unconfigured.json').read_text())
        self.assertEqual(on_disk['geofence'], template)
        self.assertIs(template['enabled'], False)
        self.assertEqual(template['frame'], 'px4_local_enu')
        self.assertTrue(all(template[k] is None for k in ('x_min_m', 'x_max_m', 'y_min_m', 'y_max_m', 'latency_s')))
        for seed in (1, 2):
            report = json.loads((ROOT / ('docs/evidence/camonly_2026-11-04/camk2/train_report_v3_s%d.json' % seed))
                                .read_text())
            self.assertEqual(float(report['realveh_receipt']['env_navrl']['NAVRL_SPEED_GOVERNOR_BRAKE_MPS2']),
                             template['decel_mps2'])
        self.assertFalse(P.Geofence.from_config(template).enabled)

    def test_enabled_fence_with_missing_nonfinite_or_narrow_values_refuses(self):
        good = P.Geofence.from_config(fence())                       # positive control
        self.assertTrue(good.enabled)
        self.assertEqual((good.x_min_m, good.x_max_m, good.latency_s), (-5., 5., .2))
        self.assertTrue(P.Geofence.from_config(fence(box=(0., .5, 0., .5))).enabled)   # exactly 0.5 m wide
        bad = []
        for key in ('x_min_m', 'x_max_m', 'y_min_m', 'y_max_m', 'decel_mps2', 'latency_s'):
            bad += [('%s None' % key, fence(**{key: None})), ('%s nan' % key, fence(**{key: float('nan')})),
                    ('%s inf' % key, fence(**{key: float('inf')})), ('%s bool' % key, fence(**{key: True})),
                    ('%s text' % key, fence(**{key: '1.0'}))]
        bad += [('narrow x', fence(box=(0., .49, -5., 5.))), ('narrow y', fence(box=(-5., 5., 1., 1.4))),
                ('inverted', fence(box=(5., -5., -5., 5.))), ('latency 0', fence(latency=0.)),
                ('negative decel', fence(decel_mps2=-1.)), ('frame', fence(frame='px4_local_ned')),
                ('enabled 1', fence(enabled=1)), ('missing key', {k: v for k, v in fence().items() if k != 'latency_s'}),
                ('extra key', dict(fence(), margin_m=.3)), ('not a mapping', None)]
        for label, block in bad:
            with self.subTest(label), self.assertRaises(P.GatewayError):
                P.Geofence.from_config(block)
        self.assertEqual(len(bad), 40)
        # The gateway itself requires an explicit fence and refuses an invalid one before any output exists.
        with self.assertRaises(TypeError):
            P.Px4OffboardGateway(binding(), command_frame_id='map')
        for block in (None, fence(x_min_m=None)):
            with self.assertRaises(P.GatewayError):
                P.Px4OffboardGateway(binding(), command_frame_id='map', geofence=block)
        config = N.unconfigured_template()
        config.update(status='CONFIGURED', enable_offboard_publication=True, frames=dict(world='map', body='base_link'),
                      geofence=fence(y_max_m=None))
        with self.assertRaisesRegex(ValueError, 'measured y_max_m'):
            N.BridgeSession(config, binding())
        config['geofence'] = fence()
        self.assertTrue(N.BridgeSession(config, binding()).gate.geofence.enabled)   # positive control

    def test_node_configuration_refuses_an_enabled_fence_without_a_box_before_starting(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            value = N.unconfigured_template()
            value.update(status='CONFIGURED', frames=dict(world='map', body='base_link'),
                         px4_msgs_msg_dir=str(tmp), geofence=fence(x_min_m=None, x_max_m=None))
            value['firmware']['source'] = 'CPU fixture source schema only, clock/hardware NOT_TESTED'
            path = tmp / 'config.json'
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'measured x_min_m'):
                N.load_node_config(path, ROOT)
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                self.assertEqual(N.main(config_path=path, source_root=ROOT, check_only=True), 2)
            report = json.loads(stdout.getvalue())
            self.assertEqual(report['status'], 'NOT_CONFIGURED')
            self.assertIn('geofence', report['reason'])
            # A configuration without the required block is refused as a schema change.
            del value['geofence']
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'schema'):
                N.load_node_config(path, ROOT)
            if not PX4_MSG_SOURCE.is_dir():
                self.skipTest('local PX4 message source unavailable for the positive control')
            for name in P.PX4_MESSAGE_SHA256:
                (tmp / name).write_bytes((PX4_MSG_SOURCE / name).read_bytes())
            for block, expected in ((fence(), 'ENABLED'), (P.geofence_template(), 'DISABLED')):
                value['geofence'] = block
                path.write_text(json.dumps(value))
                configured, _ = N.load_node_config(path, ROOT)
                self.assertEqual(configured['geofence'], block)
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout):
                    self.assertEqual(N.main(config_path=path, source_root=ROOT, check_only=True), 0)
                self.assertEqual(json.loads(stdout.getvalue())['geofence'], expected)


class GeofenceClamp(unittest.TestCase):
    def test_centre_of_the_box_passes_commands_bit_exactly(self):
        commands = [(1., 2.), (-2.5, 2.5), (2.5, -2.5), (.3, -.7), (0., 0.), (-1.7320508075688772, .1)]
        fenced, _ = activate(fence(box=(-10., 10., -10., 10.)))
        plain, _ = activate(P.geofence_template())               # control: geofence disabled
        for k, v in enumerate(commands, 1):
            a, b = send(fenced, k, (0., 0.), v, yaw=-.4), send(plain, k, (0., 0.), v, yaw=-.4)
            with self.subTest(v=v):
                self.assertIsNotNone(a)
                self.assertEqual(bits(a), bits(b))
                self.assertEqual(bits(a), bits(P.encode_setpoint(cmd(101.1 + k / 10, v, -.4), hold_z_ned_m=-3.4,
                                                                timestamp_us=a['trajectory_setpoint']['timestamp'])))
                record = fenced.geofence_record
                self.assertFalse(record['geofence_clamped'])
                self.assertEqual(bits(record['pre_clamp_velocity_xy_mps']), bits(record['post_clamp_velocity_xy_mps']))
                self.assertEqual(record['output'], 'policy')
                self.assertFalse(plain.geofence_record['enabled'])

    def test_approach_at_2p5_follows_the_formula_and_the_stop_never_crosses_the_wall(self):
        cases = 0
        for tau in (.2, .4):
            for d in (3., 1.5, .5):
                # Four walls: (position, command toward the wall, index of the normal component).
                for xy, v, axis in (((5. - d, 0.), (2.5, 0.), 0), ((-5. + d, 0.), (-2.5, 0.), 0),
                                    ((0., 5. - d), (0., 2.5), 1), ((0., -5. + d), (0., -2.5), 1)):
                    gate, _ = activate(fence(latency=tau), xy=(0., 0.))
                    out = enu_velocity(send(gate, 1, xy, v))
                    allowed = v_max(d, A_BRAKE, tau)
                    expected = min(2.5, allowed)
                    with self.subTest(tau=tau, d=d, xy=xy):
                        self.assertAlmostEqual(abs(out[axis]), expected, places=12)
                        self.assertEqual(math.copysign(1., out[axis]), math.copysign(1., v[axis]))
                        self.assertEqual(out[1 - axis], 0.)
                        self.assertEqual(gate.geofence_record['geofence_clamped'], allowed < 2.5)
                        self.assertEqual(gate.geofence_record['pre_clamp_velocity_xy_mps'], [float(v[0]), float(v[1])])
                        # The published speed, held for the latency and braked at a, stops before the wall.
                        self.assertLessEqual(stop_overshoot(d, abs(out[axis]), tau, A_BRAKE), 1e-9)
                        # Control: the unclamped 2.5 m/s command crosses the wall unless 2.5 <= v_max.
                        unclamped = stop_overshoot(d, 2.5, tau, A_BRAKE)
                        if allowed < 2.5:
                            self.assertGreater(unclamped, .01)
                        else:
                            self.assertLessEqual(unclamped, 0.)
                    cases += 1
        self.assertEqual(cases, 24)
        # 3 m is outside the stopping distance at both latencies (pass-through); 1.5 m and 0.5 m are clamped.
        self.assertGreater(v_max(3., A_BRAKE, .4), 2.5)
        self.assertLess(v_max(1.5, A_BRAKE, .2), 2.5)

    def test_parallel_component_yaw_rate_and_altitude_hold_are_untouched_while_the_normal_is_clamped(self):
        # Repair r1 (F2): the clamp is one direction-preserving scale s, so the parallel (north) component now
        # slows together with the normal one (it was bit-exact 1.5 before). Yaw rate and z hold stay untouched.
        gate, _ = activate(fence(latency=.3))
        plain, _ = activate(P.geofence_template())
        out = send(gate, 1, (4.5, 0.), (2., 1.5), yaw=.7)
        ref = send(plain, 1, (4.5, 0.), (2., 1.5), yaw=.7)
        east, north = enu_velocity(out)
        s = east / 2.
        self.assertLess(s, 1.)
        self.assertAlmostEqual(north, 1.5 * s, places=12)                       # same direction
        # The binding (east) axis meets its wall exactly under straight-line braking at total deceleration a.
        speed = math.hypot(east, north)
        self.assertAlmostEqual(east * .3 + speed * east / (2. * A_BRAKE), .5, places=12)
        self.assertEqual(bits(out['trajectory_setpoint']['yawspeed']), bits(ref['trajectory_setpoint']['yawspeed']))
        self.assertEqual(bits(out['trajectory_setpoint']['position']), bits(ref['trajectory_setpoint']['position']))
        self.assertEqual(bits(out['offboard_control_mode']), bits(ref['offboard_control_mode']))
        record = gate.geofence_record
        self.assertTrue(record['geofence_clamped'])
        self.assertEqual(record['pre_clamp_velocity_xy_mps'], [2., 1.5])
        self.assertEqual(record['post_clamp_velocity_xy_mps'], [east, north])
        self.assertAlmostEqual(record['scale'], s, places=12)
        self.assertEqual([a['clamped'] for a in record['axes']], [True, False])
        # Never increases a component, also at the wall itself (d = 0) where the outward part becomes 0.
        out = send(gate, 2, (5., -5.), (2., -1.), yaw=.7)
        self.assertEqual(enu_velocity(out), (0., 0.))
        self.assertIsNone(gate.geofence_fault)       # on the boundary is not a breach (strictly outside is)

    def test_inward_and_along_wall_commands_are_unchanged_near_a_wall(self):
        gate, _ = activate(fence(latency=.3))
        plain, _ = activate(P.geofence_template())
        for k, (xy, v) in enumerate((((4.5, 0.), (-2.5, 0.)), ((4.5, 0.), (0., 2.5)), ((0., -4.9), (1., 2.4)),
                                     ((-4.9, 4.9), (2.5, -.5))), 1):
            with self.subTest(xy=xy, v=v):
                self.assertEqual(bits(send(gate, k, xy, v)), bits(send(plain, k, xy, v)))
                self.assertFalse(gate.geofence_record['geofence_clamped'])
        # Control: the same positions with the command turned toward the near wall are clamped.
        self.assertTrue(abs(enu_velocity(send(gate, 5, (4.5, 0.), (2.5, 0.)))[0]) < 2.5)
        self.assertTrue(gate.geofence_record['geofence_clamped'])

    def test_narrow_box_limits_the_far_wall_component_too(self):
        # Decision r2-G1: each component is limited against the wall it points at. In a 1 m box, 0.1 m from
        # x_min, +x points away from the nearer wall but at a wall 0.9 m away.
        tau = .2
        gate, _ = activate(fence(latency=tau, box=(0., 1., -5., 5.)), xy=(.5, 0.))
        east, _ = enu_velocity(send(gate, 1, (.1, 0.), (2.5, 0.)))
        self.assertAlmostEqual(east, v_max(.9, A_BRAKE, tau), places=12)
        self.assertLessEqual(stop_overshoot(.9, east, tau, A_BRAKE), 1e-9)
        # Control: a nearer-wall-only rule would pass 2.5 m/s here, and that stop crosses x_max by > 0.5 m.
        self.assertGreater(stop_overshoot(.9, 2.5, tau, A_BRAKE), .5)


class GeofenceBreachAndFreshness(unittest.TestCase):
    def test_breach_sends_zero_velocity_with_altitude_hold_latches_and_resets_only_inside(self):
        gate, before = activate(fence())
        hold = before['trajectory_setpoint']['position']
        out = send(gate, 1, (5.01, 0.), (-2., 1.))         # strictly outside, even with an inward command
        self.assertIsNotNone(out)
        self.assertEqual(gate.state, 'ACTIVE')                # streams stay up
        self.assertEqual(enu_velocity(out), (0., 0.))
        self.assertEqual(out['trajectory_setpoint']['yawspeed'], 0.)
        self.assertEqual(bits(out['trajectory_setpoint']['position']), bits(hold))
        self.assertIn('geofence breach', gate.geofence_fault)
        self.assertEqual(gate.geofence_record['output'], 'neutral_geofence_fault')
        # Latched: back inside, still zero until an explicit reset.
        out = send(gate, 2, (0., 0.), (1., 2.))
        self.assertEqual(enu_velocity(out), (0., 0.))
        self.assertIsNotNone(gate.geofence_fault)
        # A reset outside is refused and the fault stays latched (gateway not revoked).
        now, mono = 101.33, 21.33
        with self.assertRaisesRegex(P.GatewayError, 'still outside'):
            gate.reset_geofence(snap(now, mono, (0., 5.2), nav=P.OFFBOARD), now_s=now, now_monotonic_s=mono)
        self.assertIsNotNone(gate.geofence_fault)
        self.assertEqual(gate.state, 'ACTIVE')
        # Reset inside clears it; commands work again (bit-exact at the centre).
        now, mono = 101.36, 21.36
        cleared = gate.reset_geofence(snap(now, mono, (0., 0.), nav=P.OFFBOARD), now_s=now, now_monotonic_s=mono)
        self.assertIn('geofence breach', cleared)
        self.assertIsNone(gate.geofence_fault)
        out = send(gate, 3, (0., 0.), (1., 2.))
        self.assertEqual(enu_velocity(out), (1., 2.))
        with self.assertRaisesRegex(P.GatewayError, 'no geofence fault'):
            gate.reset_geofence(snap(now + .1, mono + .1, (0., 0.), nav=P.OFFBOARD), now_s=now + .1,
                                now_monotonic_s=mono + .1)
        # Control: the same excursion with the fence disabled is passed through.
        plain, _ = activate(P.geofence_template())
        self.assertEqual(enu_velocity(send(plain, 1, (5.01, 0.), (-2., 1.))), (-2., 1.))
        self.assertIsNone(plain.geofence_fault)

    def test_breach_during_priming_latches_and_bridge_reset_uses_the_session_snapshot(self):
        config = N.unconfigured_template()
        config.update(status='CONFIGURED', enable_offboard_publication=True, frames=dict(world='map', body='base_link'),
                      geofence=fence())
        session = N.BridgeSession(config, binding())

        def offer(now, wall, xy, nav=2):
            sample = snap(now, wall, xy, nav=nav)
            for kind in ('odometry', 'local_position', 'status'):
                session.offer(kind, sample[kind], receipt=wall)

        offer(100., 20., (6., 0.))
        session.poll(now=100., wall=20.)
        session.prime(now=100., wall=20.)
        self.assertIsNotNone(session.gate.geofence_fault)    # primed outside the box: latched, not revoked
        self.assertIsNone(session.fault)
        with self.assertRaisesRegex(ValueError, 'still outside'):
            session.reset_geofence(now=100., wall=20.)
        offer(100.1, 20.1, (0., 0.))
        _, pair = session.poll(now=100.1, wall=20.1)
        self.assertIsNotNone(pair)
        self.assertEqual(session.reset_geofence(now=100.1, wall=20.1)[:16], 'geofence breach ')
        diag = session.diagnostics()
        self.assertIsNone(diag['geofence_fault'])
        self.assertTrue(diag['geofence']['enabled'])
        json.dumps(diag, allow_nan=False)
        # Observation-only bridge: no primed gateway, nothing to reset (control).
        config.update(enable_offboard_publication=False)
        session = N.BridgeSession(config, binding())
        offer(100., 20., (6., 0.))
        session.poll(now=100., wall=20.)
        with self.assertRaisesRegex(ValueError, 'primed or active'):
            session.reset_geofence(now=100., wall=20.)
        self.assertIsNone(session.fault)

    def test_stale_or_missing_position_is_rejected_before_the_geofence_sees_it(self):
        for xy in ((7., 0.), (0., 0.)):
            gate, _ = activate(fence())
            with self.subTest(xy=xy):
                self.assertIsNone(send(gate, 1, xy, stale_odometry=True))
                self.assertEqual(gate.state, 'REVOKED')
                self.assertIn('receipt', gate.reason)
                self.assertIsNone(gate.geofence_fault)          # a stale position is never evaluated
        # Control: the same outside position, fresh, is a latched breach and the streams stay up.
        gate, _ = activate(fence())
        self.assertIsNotNone(send(gate, 1, (7., 0.)))
        self.assertIsNotNone(gate.geofence_fault)
        self.assertEqual(gate.state, 'ACTIVE')
        # Missing position fields revoke.
        gate, _ = activate(fence())
        sample = snap(101.2, 21.2, (0., 0.), nav=P.OFFBOARD)
        del sample['odometry']['position']
        self.assertIsNone(gate.update(sample, cmd(101.2), command_received_monotonic_s=21.2, now_s=101.2,
                                      now_monotonic_s=21.2))
        self.assertEqual(gate.state, 'REVOKED')
        # Reset with a stale estimator revokes instead of clearing.
        gate, _ = activate(fence())
        send(gate, 1, (7., 0.))
        with self.assertRaises(P.GatewayError):
            gate.reset_geofence(snap(101.25, 21.25, (0., 0.), nav=P.OFFBOARD, stale_odometry=True),
                                now_s=101.25, now_monotonic_s=21.25)
        self.assertEqual(gate.state, 'REVOKED')
        # Prime with a stale estimator refuses as before.
        gate = P.Px4OffboardGateway(binding(), command_frame_id='map', geofence=fence())
        with self.assertRaises(P.GatewayError):
            gate.prime(snap(100., 20., (0., 0.), stale_odometry=True), now_s=100., now_monotonic_s=20.,
                       manual_hover_handover=True)
        self.assertEqual(gate.state, 'REVOKED')


# --------------------------------------------------------------------------------- H coverage, I REP 117
TRAINED = F.TRAINED


def distorted_info(t, k1=.45, model='plumb_bob'):
    info = F.image_messages(t, np.zeros((90, 160, 3), np.uint8), np.zeros((90, 160), np.uint16), TRAINED)['camera_info']
    info.d, info.distortion_model = [k1, 0., 0., 0., 0.], model
    return info


def wall_scene():
    frame, _ = F.render('trained_equal', 1., wall_x=10.)
    return frame['rgb'].copy(), np.asarray(frame['depth_z_m'], np.float64)


def ros_frame(tmp, depth_msg=None, info=None, t=3., z_m=None, rgb=None):
    rgb_wall, z_wall = wall_scene()
    rgb = rgb_wall if rgb is None else rgb
    z_m = z_wall if z_m is None else z_m
    msgs = F.image_messages(t, rgb, np.round(z_m * 1000.).astype(np.uint16), TRAINED)
    if depth_msg is not None:
        msgs['depth'] = depth_msg
    if info is not None:
        msgs['camera_info'] = info
    odom = F.odometry_message(t, [0., 0., 1.], 0., [0., 0., 0.], 0.)
    return A.build_frame(msgs['rgb'], msgs['depth'], msgs['camera_info'], odom, F.ros_binding(Path(tmp) / 'fx.txt'))


def float_depth(t, z):
    return NS(header=F.header('camera_optical', t), width=160, height=90, step=640, encoding='32FC1',
              is_bigendian=0, data=np.ascontiguousarray(z.astype('<f4')).tobytes())


class NoGoalPerception:
    """Perception stand-in for preprocess-only tests: its segmenter scores every pixel 0, so no pixel is
    goal-coloured (these scenes have no red target). The production segmenter (repair r1, F5) is exercised
    by the actual-policy classes."""
    pixel_threshold = .55
    max_camera_range = 20.

    @staticmethod
    def segmenter(rgb, depth, max_depth):
        import torch
        return torch.zeros(depth.shape)


def geometry_runtime(contract):
    """The production CameraPolicyRuntime methods without the checkpoint: geometry contract, production
    blob_near (loaded privately) and the no-goal perception stand-in."""
    import torch
    rt = object.__new__(R.CameraPolicyRuntime)
    rt.contract, rt.validity, rt.torch, rt.perception = contract, DV, torch, NoGoalPerception()
    return rt


def preprocess(frame, state, ground=False):
    """Production CameraPolicyRuntime.preprocess (projection, height filter, blob_near, blanking) without
    the checkpoint: the method runs with the geometry contract and the production blob_near."""
    _, depth, counts, projection = geometry_runtime(F.contract(ground=ground)).preprocess(frame, state)
    return depth[0].numpy(), counts, projection


class RosCoverage(unittest.TestCase):
    def test_distorted_camera_info_border_is_no_return_and_the_same_border_without_coverage_is_near(self):
        info = distorted_info(3.)
        coverage, provenance = A.camera_info_coverage(info, width=160, height=90)
        border = ~coverage
        self.assertEqual(provenance['case'], 'undistortion_remap')
        self.assertGreater(int(border.sum()), 500)
        self.assertTrue(border[0, 0] and border[-1, -1] and border[45, 0] and coverage[45, 80])
        rgb, z = wall_scene()
        z = np.where(border, 0., z)                       # what a rectifying driver delivers outside the raw image
        rgb[border] = 0
        with tempfile.TemporaryDirectory() as tmp:
            frame, state = ros_frame(tmp, info=info, z_m=z, rgb=rgb)
        self.assertEqual(frame['coverage_provenance']['case'], 'undistortion_remap')
        np.testing.assert_array_equal(frame['coverage'], coverage)
        self.assertTrue(frame['invalid'][border].all())
        depth, counts, projection = preprocess(frame, state)
        self.assertTrue(projection['coverage_mask'])
        self.assertEqual(counts['near_pixels'], 0)
        self.assertEqual(int((depth < 1.).sum()), 0)
        self.assertEqual(counts['no_return_pixels'], int(border.sum()))
        self.assertEqual(counts['sensor_invalid_pixels'], 0)
        # Control: the same frame without the coverage mask (the pre-r2 ROS path) reads the border as near.
        legacy = dict(frame)
        del legacy['coverage']
        depth, counts, projection = preprocess(legacy, state)
        self.assertFalse(projection['coverage_mask'])
        self.assertGreater(counts['near_pixels'], 500)
        near = depth < 1.
        self.assertGreater(int(near.sum()), 500)
        self.assertFalse((near & coverage).any())         # every near pixel is a border pixel

    def test_zero_distortion_gives_all_true_coverage_and_matches_the_device_path(self):
        for d in ([], [0., 0., 0., 0., 0.]):
            info = distorted_info(3.)
            info.d = d
            coverage, provenance = A.camera_info_coverage(info, width=160, height=90)
            with self.subTest(d=d):
                self.assertTrue(coverage.all())
                self.assertEqual(coverage.shape, (90, 160))
                self.assertEqual(provenance['case'], 'zero_distortion_all_covered')
        with tempfile.TemporaryDirectory() as tmp:
            frame, _ = ros_frame(tmp)
        self.assertTrue(frame['coverage'].all())
        self.assertEqual(frame['coverage_provenance']['case'], 'zero_distortion_all_covered')
        # Device-path parity: tools/oakd_camera.py's Rectifier (R = I, P = K) yields the identical mask.
        oakd = sys.modules.get('oakd_camera_r2_parity') or _load('oakd_camera_r2_parity', ROOT / 'tools/oakd_camera.py',
                                                                 register=True)
        k = [[TRAINED['fx'], 0., TRAINED['cx']], [0., TRAINED['fy'], TRAINED['cy']], [0., 0., 1.]]
        borders = {}
        for k1 in (.45, -.3, .1):
            device = oakd.Rectifier(k, [k1, 0., 0., 0., 0.], width=160, height=90)
            _, _, _, device_coverage = device.apply(np.zeros((90, 160, 3), np.uint8), np.zeros((90, 160), np.uint16))
            ros_coverage, _ = A.camera_info_coverage(distorted_info(3., k1), width=160, height=90)
            with self.subTest(k1=k1):
                np.testing.assert_array_equal(ros_coverage, device_coverage)
            borders[k1] = int((~device_coverage).sum())
        self.assertGreater(borders[.45], 500)      # non-vacuous: the parity holds on a real border
        self.assertGreater(borders[.1], 0)

    def test_coverage_is_cached_per_camera_info_content_read_only_and_fails_closed_for_unknown_models(self):
        first, _ = A.camera_info_coverage(distorted_info(3.), width=160, height=90)
        again, _ = A.camera_info_coverage(distorted_info(7.), width=160, height=90)   # new stamp, same content
        self.assertIs(first, again)
        self.assertFalse(first.flags.writeable)
        other, _ = A.camera_info_coverage(distorted_info(3., .2), width=160, height=90)
        self.assertIsNot(other, first)
        self.assertLess(int((~other).sum()), int((~first).sum()))
        for model in ('equidistant', '', None):
            with self.subTest(model=model), self.assertRaisesRegex(A.RosInputError, 'model'):
                A.camera_info_coverage(distorted_info(3., model=model), width=160, height=90)
        bad = distorted_info(3.)
        bad.d = [.1, float('nan'), 0., 0., 0.]
        with self.assertRaisesRegex(A.RosInputError, 'finite'):
            A.camera_info_coverage(bad, width=160, height=90)
        del bad.d
        with self.assertRaisesRegex(A.RosInputError, 'D missing'):
            A.camera_info_coverage(bad, width=160, height=90)
        self.assertIsNotNone(A.camera_info_coverage(distorted_info(3., model='rational_polynomial'),
                                                    width=160, height=90)[0])   # positive control


class Rep117Depth(unittest.TestCase):
    PATCH = (slice(10, 40), slice(60, 100))            # 30 x 40 px, inside the trained FOV, above the horizon

    def frame_with_patch(self, value):
        _, z = wall_scene()
        z = z.astype(np.float32)
        z[self.PATCH] = value
        with tempfile.TemporaryDirectory() as tmp:
            return ros_frame(tmp, depth_msg=float_depth(3., z))

    def test_plus_inf_is_no_return_while_minus_inf_and_nan_patches_are_near(self):
        frame, state = self.frame_with_patch(np.inf)
        self.assertFalse(frame['invalid'].any())
        self.assertTrue(np.isposinf(frame['depth_z_m'][self.PATCH]).all())
        depth, counts, _ = preprocess(frame, state)
        self.assertEqual(counts['near_pixels'], 0)
        self.assertEqual(counts['sensor_invalid_pixels'], 0)
        self.assertEqual(counts['no_return_pixels'], 1200)
        self.assertTrue(np.all(depth[self.PATCH] == 20.))
        for value in (-np.inf, np.nan):
            frame, state = self.frame_with_patch(value)
            with self.subTest(value=value):
                self.assertTrue(frame['invalid'][self.PATCH].all())
                depth, counts, _ = preprocess(frame, state)
                self.assertEqual(counts['sensor_invalid_pixels'], 1200)
                self.assertGreater(counts['near_pixels'], 1000)
                self.assertLess(float(depth[25, 80]), .5)
        # Control: finite valid depth everywhere gives neither class.
        frame, state = self.frame_with_patch(3.)
        _, counts, _ = preprocess(frame, state)
        self.assertEqual((counts['near_pixels'], counts['no_return_pixels'], counts['sensor_invalid_pixels']), (0, 0, 0))

    def test_16uc1_zero_is_unchanged_and_stays_near_when_dense(self):
        rgb, z = wall_scene()
        mm = np.round(z * 1000.).astype(np.uint16)
        mm[self.PATCH] = 0
        msgs = F.image_messages(3., rgb, mm, TRAINED)
        decoded, invalid = A.decode_depth(msgs['depth'])
        legacy = mm.astype(np.float32) * np.float32(.001)
        np.testing.assert_array_equal(invalid, legacy <= 0)                 # the pre-r2 rule, bit for bit
        np.testing.assert_array_equal(decoded, np.where(legacy <= 0, np.float32(0.), legacy))
        self.assertEqual(int(invalid.sum()), 1200)
        with tempfile.TemporaryDirectory() as tmp:
            frame, state = ros_frame(tmp, z_m=np.where(mm == 0, 0., z))
        _, counts, _ = preprocess(frame, state)
        self.assertEqual(counts['sensor_invalid_pixels'], 1200)
        self.assertGreater(counts['near_pixels'], 1000)


# ------------------------------------------------------------------------------------------- J altitude gate
class BandRuntime:
    """Runtime stand-in with the production height_band; step refuses out-of-band frames like the runtime."""
    def __init__(self, ceiling=None):
        self.contract = F.contract(ground=True, ceiling=ceiling)
        self.events, self.failed = [], False

    def reset(self): self.events.append('reset'); self.failed = False
    def invalidate(self): self.failed = True
    def height_band(self, state): return R.height_band(state, self.contract)

    def step(self, frame, state, *, now_s):
        band = R.height_band(state, self.contract)
        if not band['in_band']:
            raise R.InputError(band['reason'])
        self.events.append('step')
        return dict(t=now_s, velocity_world_enu_mps=[.2, .1, 0.], yaw_rate_rps=.3)


class AltitudeGating(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.runtime = BandRuntime()
        self.session = RosPolicySession(self.runtime, F.ros_binding(Path(self.tmp.name) / 'fixture.txt'))
        self.runtime.events.clear()
        self.k = 0

    def tick(self, height, *, gap=.1):
        t, wall = 3. + self.k * .1 + (gap - .1), 100. + self.k * .1 + (gap - .1)
        self.k += 1
        msgs = F.image_messages(t, np.zeros((90, 160, 3), np.uint8), np.full((90, 160), 5000, np.uint16), TRAINED)
        msgs['odometry'] = F.odometry_message(t, [0., 0., height], 0., [0., 0., 0.], 0.)
        for kind, message in msgs.items():
            self.session.offer(kind, message, received_monotonic_s=wall)
        item = self.session.prepare(now_ros_s=t + .02, now_monotonic_s=wall + .02)
        if item is None:
            return 'NO_ITEM', None
        result = self.session.infer(item)
        return self.session.finish(item, result, now_ros_s=t + .02, now_monotonic_s=wall + .02), item

    def test_on_the_ground_no_proposal_no_fault_then_climb_starts_a_fresh_history(self):
        for _ in range(5):
            proposal, item = self.tick(0.)                  # estimator z 0: camera at 0.03 m <= margin 0.10 m
            self.assertIsNone(proposal)
            self.assertIsNone(self.session.fault)
            self.assertEqual(self.session.status, 'WAITING_ALTITUDE')
        self.assertEqual(self.runtime.events, [])             # never called: no history advance
        self.assertIn('ground margin', self.session.altitude['reason'])
        self.assertAlmostEqual(self.session.altitude['camera_height_m'], .03)
        proposal, item = self.tick(1.)                         # climbed into the band
        self.assertIsNotNone(proposal)
        self.assertTrue(item.fresh_history)
        self.assertEqual(self.runtime.events, ['reset', 'step'])   # fresh history, exactly as a reset does
        self.assertEqual(self.session.status, 'RUNNING_PROPOSALS')
        proposal, item = self.tick(1.)
        self.assertIsNotNone(proposal)
        self.assertFalse(item.fresh_history)
        self.assertEqual(self.runtime.events, ['reset', 'step', 'step'])   # history continues in band
        # Descending below the band mid-session waits again without a fault; the next climb is fresh again.
        self.assertIsNone(self.tick(.05)[0])
        self.assertEqual(self.session.status, 'WAITING_ALTITUDE')
        self.assertIsNone(self.session.fault)
        self.assertIsNotNone(self.tick(1.)[0])
        self.assertEqual(self.runtime.events[-2:], ['reset', 'step'])
        # Control: without the gate, the same ground frame reaches the runtime and is refused (the old fault).
        state = F.scene_state(0.)
        with self.assertRaisesRegex(R.InputError, 'ground margin'):
            self.runtime.step({}, state, now_s=1.)

    def test_ceiling_band_waits_and_a_first_in_band_frame_without_gating_is_not_reset(self):
        self.runtime.contract = F.contract(ground=True, ceiling=3.)
        self.assertIsNotNone(self.tick(1.)[0])               # in band from the start: no extra reset
        self.assertEqual(self.runtime.events, ['step'])
        self.assertIsNone(self.tick(2.9)[0])                 # camera at 2.93 m >= 3.0 - 0.1
        self.assertIn('ceiling band', self.session.altitude['reason'])
        self.assertIsNone(self.session.fault)
        self.assertIsNotNone(self.tick(1.)[0])
        self.assertEqual(self.runtime.events, ['step', 'reset', 'step'])

    def test_stale_streams_and_timing_faults_still_fault_while_waiting(self):
        self.assertIsNone(self.tick(0.)[0])
        self.assertEqual(self.session.status, 'WAITING_ALTITUDE')
        # Positive control: fresh inputs at 10 Hz keep waiting without a fault.
        self.assertIsNone(self.tick(0.)[0])
        self.session.watchdog(now_ros_s=3.15, now_monotonic_s=100.15)
        self.assertIsNone(self.session.fault)
        # A stale input stream (odometry stops; images continue) faults while waiting.
        msgs = F.image_messages(3.2, np.zeros((90, 160, 3), np.uint8), np.full((90, 160), 5000, np.uint16), TRAINED)
        for kind in ('rgb', 'depth', 'camera_info'):
            self.session.offer(kind, msgs[kind], received_monotonic_s=100.2)
        with self.assertRaisesRegex(SessionFault, 'input stream: odometry'):
            self.session.watchdog(now_ros_s=3.26, now_monotonic_s=100.26)
        self.assertEqual(self.session.status, 'FAULT')
        self.assertEqual(self.runtime.events, [])
        # A late tick (10 Hz period exceeded) faults while waiting.
        self.setUp()
        self.assertIsNone(self.tick(0.)[0])
        with self.assertRaisesRegex(SessionFault, '10 Hz'):
            self.tick(0., gap=.13)
        self.assertIsNotNone(self.session.fault)
        # A ROS clock pause faults while waiting.
        self.setUp()
        self.assertIsNone(self.tick(0.)[0])
        self.session.watchdog(now_ros_s=3.02, now_monotonic_s=100.02)
        with self.assertRaisesRegex(SessionFault, 'paused'):
            self.session.watchdog(now_ros_s=3.02, now_monotonic_s=100.2)
        # Malformed estimator state still faults (not gated): non-unit quaternion.
        self.setUp()
        msgs = F.image_messages(3., np.zeros((90, 160, 3), np.uint8), np.full((90, 160), 5000, np.uint16), TRAINED)
        msgs['odometry'] = F.odometry_message(3., [0., 0., 0.], 0., [0., 0., 0.], 0.)
        msgs['odometry'].pose.pose.orientation.w = 2.
        for kind, message in msgs.items():
            self.session.offer(kind, message, received_monotonic_s=100.)
        item = self.session.prepare(now_ros_s=3.02, now_monotonic_s=100.02)
        with self.assertRaises(Exception):
            self.session.infer(item)
        self.session.fail_inference('malformed odometry')
        self.assertEqual(self.session.status, 'FAULT')


# ------------------------------------------------------------------------ actual policy (seed 1), one process
ACTUAL = r'''
import json, math, sys, tempfile
from pathlib import Path
import numpy as np
import test_camera_runtime_fixes as F
import test_camera_runtime_fixes_r2 as G
from camera_policy_contract import build_contract, estimator_height_assertion
from camera_policy_runtime import CameraPolicyRuntime, InputError
import camera_ros2_adapter as A
from camera_ros2_session import RosPolicySession

_evidence_dir = tempfile.TemporaryDirectory()
_evidence = Path(_evidence_dir.name) / 'estimator_height_fixture.txt'
_evidence.write_text('CPU test fixture: synthetic scenes put estimator z at the height above the floor. Not evidence.')
# Floor filter ON (default contract), with the repair-r1 (F6) estimator-height assertion as a CPU fixture.
rt = CameraPolicyRuntime(build_contract(1, estimator_height=estimator_height_assertion(_evidence)))
inside = np.abs(rt.bearings.numpy()) <= math.radians(33.0)
out = {}

def scan_row(res):
    obs = np.asarray(res['observation'])
    nearest = (obs[:288].reshape(4, 72) * 12.).min(axis=0)
    return dict(near=res['near_pixels'], no_return=res['no_return_pixels'], si=res['sensor_invalid_pixels'],
                infov_lt1=int((nearest[inside] < 1.).sum()), infov=int(inside.sum()))

# J: ground -> climb through the ROS session with the real runtime.
f, _ = F.render('trained_equal', 1., wall_x=10., sphere=(5., 0., 1.03, .15))
mm = np.round(np.asarray(f['depth_z_m'], np.float64) * 1000.).astype(np.uint16)
heights = [1., 1., 0., 0., 0., 1., 1.]
rows = []
with tempfile.TemporaryDirectory() as tmp:
    binding = F.ros_binding(Path(tmp) / 'fixture.txt')
    session = RosPolicySession(rt, binding)
    frames = []
    for i, h in enumerate(heights):
        t, wall = 3. + .1 * i, 100. + .1 * i
        msgs = F.image_messages(t, f['rgb'], mm, F.TRAINED)
        msgs['odometry'] = F.odometry_message(t, [0., 0., h], 0., [0., 0., 0.], 0.)
        frames.append((msgs, t))
        for kind, message in msgs.items():
            session.offer(kind, message, received_monotonic_s=wall)
        item = session.prepare(now_ros_s=t + .02, now_monotonic_s=wall + .02)
        before = (rt.perception.step_count, rt._last_now)
        result = session.infer(item)
        proposal = session.finish(item, result, now_ros_s=t + .02, now_monotonic_s=wall + .02)
        rows.append(dict(h=h, status=session.status, fault=session.fault, proposal=proposal is not None,
                         fresh=item.fresh_history, steps_before=before[0], steps_after=rt.perception.step_count,
                         last_now_unchanged=before[1] == rt._last_now, obs=result.get('observation')))
    def fresh_obs(i):
        msgs, t = frames[i]
        frame, state = A.build_frame(msgs['rgb'], msgs['depth'], msgs['camera_info'], msgs['odometry'], binding)
        rt.reset()
        return rt.step(frame, state, now_s=t + .02)['observation']
    out['fresh_equal_after_climb'] = fresh_obs(5) == rows[5]['obs']
    out['fresh_equal_first_tick'] = fresh_obs(0) == rows[0]['obs']
    out['fresh_differs_with_history'] = fresh_obs(1) != rows[1]['obs']      # control: history matters
    msgs, t = frames[2]
    frame, state = A.build_frame(msgs['rgb'], msgs['depth'], msgs['camera_info'], msgs['odometry'], binding)
    rt.reset()
    try:
        rt.step(frame, state, now_s=t + .02)
        out['ground_direct_step'] = 'ACCEPTED'
    except InputError as exc:
        out['ground_direct_step'] = 'REFUSED: ' + str(exc)
for r in rows:
    r.pop('obs')
out['rows'] = rows

# H and I at policy level: the full step on ROS-built frames.
info = G.distorted_info(3.)
coverage, _ = A.camera_info_coverage(info, width=160, height=90)
rgb, z = G.wall_scene()
z = np.where(~coverage, 0., z)
rgb[~coverage] = 0
with tempfile.TemporaryDirectory() as tmp:
    frame, state = G.ros_frame(tmp, info=info, z_m=z, rgb=rgb)
legacy = dict(frame)
del legacy['coverage']
for name, fr in (('border coverage', frame), ('border no coverage', legacy)):
    rt.reset()
    out[name] = scan_row(rt.step(fr, state, now_s=3.02))
for name, value in (('patch +inf', np.inf), ('patch -inf', -np.inf), ('patch nan', np.nan), ('patch 3m', 3.)):
    zz = G.wall_scene()[1].astype(np.float32)
    zz[10:40, 60:100] = value
    with tempfile.TemporaryDirectory() as tmp:
        fr, st = G.ros_frame(tmp, depth_msg=G.float_depth(3., zz))
    rt.reset()
    out[name] = scan_row(rt.step(fr, st, now_s=3.02))
print('FIX_R2_RESULT ' + json.dumps(out, allow_nan=False))
'''


class ActualPolicySeed1R2(unittest.TestCase):
    """Real CAM-K2-v3 seed-1 checkpoint through RosPolicySession / CameraPolicyRuntime.step, one fresh CPU process."""

    @classmethod
    def setUpClass(cls):
        if not CHECKPOINT_DIR.is_dir():
            raise unittest.SkipTest('local trained camera checkpoint not distributed in the public repository')
        env = {k: v for k, v in os.environ.items() if not k.startswith('NAVRL_')}
        env.update(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
                   OPENBLAS_NUM_THREADS='1', PYTHONPATH=os.pathsep.join([str(ROOT / 'tools'), str(ROOT / 'tests')]))
        done = subprocess.run([sys.executable, '-B', '-c', ACTUAL], env=env, capture_output=True, text=True,
                              timeout=900, cwd=str(ROOT))
        if done.returncode != 0:
            raise AssertionError(done.stdout[-2000:] + done.stderr[-4000:])
        line = [x for x in done.stdout.splitlines() if x.startswith('FIX_R2_RESULT ')]
        cls.out = json.loads(line[-1][len('FIX_R2_RESULT '):])

    def test_j_ground_frames_wait_without_fault_or_history_and_the_climb_starts_fresh(self):
        rows = self.out['rows']
        self.assertEqual([r['status'] for r in rows], ['RUNNING_PROPOSALS'] * 2 + ['WAITING_ALTITUDE'] * 3
                         + ['RUNNING_PROPOSALS'] * 2)
        self.assertEqual([r['proposal'] for r in rows], [True, True, False, False, False, True, True])
        self.assertTrue(all(r['fault'] is None for r in rows))
        for r in rows[2:5]:
            self.assertEqual(r['steps_after'], r['steps_before'])     # no history advance while waiting
            self.assertTrue(r['last_now_unchanged'])
        self.assertGreater(rows[1]['steps_after'], rows[1]['steps_before'])   # control: a step advances it
        self.assertEqual([r['fresh'] for r in rows], [False, False, False, True, True, True, False])
        self.assertTrue(self.out['fresh_equal_after_climb'])
        self.assertTrue(self.out['fresh_equal_first_tick'])
        self.assertTrue(self.out['fresh_differs_with_history'])
        self.assertIn('REFUSED', self.out['ground_direct_step'])
        self.assertIn('ground margin', self.out['ground_direct_step'])

    def test_h_coverage_removes_the_border_phantom_in_the_actual_observation(self):
        fixed, legacy = self.out['border coverage'], self.out['border no coverage']
        self.assertEqual(fixed['near'], 0)
        self.assertEqual(fixed['infov_lt1'], 0)
        self.assertGreater(fixed['no_return'], 500)
        self.assertGreater(legacy['near'], 500)
        self.assertGreater(legacy['infov_lt1'], 0)

    def test_i_plus_inf_patch_is_free_space_and_minus_inf_nan_patches_are_near(self):
        self.assertEqual(self.out['patch +inf']['near'], 0)
        self.assertEqual(self.out['patch +inf']['infov_lt1'], 0)
        self.assertEqual(self.out['patch +inf']['si'], 0)
        # The floor filter (on in the default contract) also removes the synthetic wall below z = 0.1 m;
        # the +inf patch adds exactly its 1200 pixels to the no-return count of the valid-patch control.
        self.assertEqual(self.out['patch +inf']['no_return'] - self.out['patch 3m']['no_return'], 1200)
        self.assertEqual(self.out['patch 3m']['near'], 0)
        for name in ('patch -inf', 'patch nan'):
            self.assertGreater(self.out[name]['near'], 1000, name)
            self.assertGreater(self.out[name]['infov_lt1'], 0, name)


if __name__ == '__main__':
    unittest.main()
