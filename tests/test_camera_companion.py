"""Measured capture -> estimator-only control composition; no USB/ports opened."""
import sys
from pathlib import Path
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from camera_companion import MeasuredRgbdClient, run_measured_control
from camera_estimator_link import EstimatorLink
from sitl_camera import ClockBinding, CameraControlError


class Message(SimpleNamespace):
    def get_srcSystem(self): return 1
    def get_srcComponent(self): return 1
    def get_type(self): return self.kind


class Clock:
    def __init__(self): self.t = 20.
    def __call__(self): return self.t
    def sleep(self, value): self.t += value


class Capture:
    def __init__(self, clock): self.clock, self.calls, self.stale = clock, 0, False
    def frames(self, *, deadline, service):
        service()
        self.calls += 1
        now = self.clock()
        return dict(rgb='measured RGB', depth_z_m='measured optical depth', invalid='same-frame mask',
                    rgb_t=now-(1. if self.stale else 0.), depth_t=now, now_s=now)


class Runtime:
    def __init__(self): self.calls, self.failed = 0, False
    def invalidate(self): self.failed = True
    def step(self, frame, state, *, now_s):
        assert frame['invalid'] == 'same-frame mask' and 'tpos' not in state
        self.calls += 1
        return dict(velocity_world_enu_mps=[.2, .1, 0.], yaw_rate_rps=.1)


class Composition(unittest.TestCase):
    def interfaces(self, publish=False):
        clock = Clock()
        sends = []
        conn = SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *a: sends.append(a)))
        link = EstimatorLink(conn, system_id=1, component_id=1, publish=publish, clock=clock)
        def service():
            boot_ms = int(round((clock()-17.)*1000))
            for m in (Message(kind='LOCAL_POSITION_NED', time_boot_ms=boot_ms, x=0., y=0., z=-1., vx=0., vy=0., vz=0.),
                      Message(kind='ATTITUDE', time_boot_ms=boot_ms, roll=0., pitch=0., yaw=0., rollspeed=0., pitchspeed=0., yawspeed=0.),
                      Message(kind='HEARTBEAT', base_mode=128, custom_mode=6<<16)):
                link.accept(m)
        service()  # run_control requires an already-ready estimator snapshot.
        return clock, link, Capture(clock), Runtime(), service, sends

    def run_segment(self, args):
        clock, link, capture, runtime, service, _ = args
        return run_measured_control(runtime, link, capture, max_steps=3, service=service,
                 clock_binding=ClockBinding(17., 'fixture measured boot-to-host offset'), clock=clock, sleep=clock.sleep)

    def test_positive_host_clock_segment_is_passive_by_default(self):
        args = self.interfaces()
        report = self.run_segment(args)
        self.assertEqual(report['commands_proposed'], 3)
        self.assertEqual(report['commands_published'], 0)
        self.assertEqual(args[-1], [])
        self.assertEqual(args[2].calls, 3)
        self.assertEqual(args[3].calls, 3)
        self.assertTrue(all('sim_now' not in x['sensor_t'] for x in report['trace']))

    def test_explicit_sender_and_rejected_frame_never_get_a_policy_command(self):
        args = self.interfaces(publish=True)
        report = self.run_segment(args)
        self.assertEqual(report['commands_published'], 3)
        self.assertEqual(len(args[-1]), 3)
        args = self.interfaces(publish=True)
        args[2].stale = True
        with self.assertRaises(CameraControlError): self.run_segment(args)
        self.assertEqual(args[-1], [])
        self.assertEqual(args[3].calls, 0)
        self.assertTrue(args[3].failed)

    def test_bounded_transactions_required_and_no_reuse_after_error(self):
        args = self.interfaces()
        client = MeasuredRgbdClient(args[2])
        with self.assertRaises(RuntimeError): client.frames()
        with self.assertRaises(TimeoutError):
            with client.bounded_io(args[0]()+.1, lambda: None, clock=args[0]):
                args[0].sleep(.11)
        self.assertTrue(client.failed)
        with self.assertRaises(RuntimeError):
            with client.bounded_io(args[0]()+.1, lambda: None, clock=args[0]): pass


if __name__ == '__main__':
    unittest.main()
