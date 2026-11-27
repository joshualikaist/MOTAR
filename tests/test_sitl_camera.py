"""Estimator-only camera driver: valid commands plus rejected stale/mode-loss controls."""
import contextlib
import importlib.util
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("sitl_camera_tested", ROOT / "tools/sitl_camera.py")
C = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = C
SPEC.loader.exec_module(C)


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class Link:
    def __init__(self, clock):
        self.clock = clock
        self.boot = 0.0
        self.hb = (128, 6 << 16)
        self.hb_wall_t = 0.0
        self.commands = []
        self.update()

    @property
    def truth(self):
        raise AssertionError("truth must never be read")

    def update(self):
        self.pos = (self.boot, 4.0, 3.0, -1.0, 2.0, 1.0, 0.0)
        self.att = (self.boot, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, -0.2)
        self.pos_wall_t = self.clock()
        self.att_wall_t = self.clock()
        self.hb_wall_t = self.clock()

    def snapshot(self):
        # The fake has no receiver thread; a real owner copies these under its receiver lock.
        return {key: getattr(self, key, None) for key in ("pos", "att", "hb", "pos_wall_t", "att_wall_t", "hb_wall_t")}

    def service(self):
        self.boot += 0.01
        self.update()

    def setpoint(self, *args):
        self.commands.append(args)


class Client:
    def __init__(self, link, offset=0.0):
        self.link, self.offset = link, offset
        self.budgets = []
        self.reused = False
        self.stale = False

    @contextlib.contextmanager
    def bounded_io(self, deadline, service, clock):
        self.budgets.append(deadline)
        service()
        yield
        service()

    def frames(self):
        now = self.link.boot + self.offset
        stamp = 0.01 + self.offset if self.reused else now - 0.005
        if self.stale:
            stamp = now - 1.0
        return {"sim_now": now, "rgb_t": stamp, "depth_t": stamp, "rgb": "rgb", "depth": "depth"}


class Runtime:
    def __init__(self):
        self.calls = []
        self.after_step = lambda: None

    def step(self, frame, state, *, now_s):
        self.calls.append((frame, state, now_s))
        self.after_step()
        return {"velocity_world_enu_mps": [1.0, 2.0, 0.0], "yaw_rate_rps": 0.3}


class CameraDriverTest(unittest.TestCase):
    def setup_interfaces(self):
        clock = Clock()
        link = Link(clock)
        return clock, link, Client(link), Runtime()

    def run_driver(self, clock, link, client, runtime, **kwargs):
        opts = dict(max_steps=3, service=link.service, clock_binding=C.ClockBinding(0.0, "fake measured binding"),
                    frame_adapter=lambda packet: {"rgb": packet["rgb"], "depth_z_m": packet["depth"],
                                                  "invalid": "same-frame-mask"},
                    clock=clock, sleep=clock.sleep)
        opts.update(kwargs)
        return C.run_control(runtime, link, client, **opts)

    def test_three_camera_only_commands_use_estimator_and_correct_frames(self):
        clock, link, client, runtime = self.setup_interfaces()
        trace = self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(trace), 3)
        self.assertEqual(len(link.commands), 3)
        self.assertEqual(len(client.budgets), 3)
        self.assertEqual(link.commands[0], (C.TM_POS_Z_VEL_XY_YAWRATE, 0.0, 0.0, -1.0, 2.0, 1.0, -0.3))
        for frame, state, _ in runtime.calls:
            self.assertEqual(set(state), {"pos", "vel", "quat_xyzw", "yaw_rate_rps", "t"})
            self.assertEqual(state["pos"], [3.0, 4.0, 1.0])
            self.assertEqual(state["vel"], [1.0, 2.0, 0.0])
            self.assertAlmostEqual(state["yaw_rate_rps"], 0.2)
            self.assertAlmostEqual(state["quat_xyzw"][3], 1.0)
            self.assertEqual(frame["invalid"], "same-frame-mask")
            self.assertNotIn("ranges", frame)

    def test_stale_frame_stops_before_any_policy_or_command(self):
        clock, link, client, runtime = self.setup_interfaces()
        client.stale = True
        with self.assertRaisesRegex(C.CameraControlError, "stale/future"):
            self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(runtime.calls), 0)
        self.assertEqual(len(link.commands), 0)

    def test_reused_frame_and_mode_loss_reject_before_next_publication(self):
        clock, link, client, runtime = self.setup_interfaces()
        runtime.after_step = lambda: setattr(client, "reused", True)
        with self.assertRaisesRegex(C.CameraControlError, "reused"):
            self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(link.commands), 1)
        clock, link, client, runtime = self.setup_interfaces()
        runtime.after_step = lambda: setattr(link, "hb", (128, 0))
        with self.assertRaisesRegex(C.CameraControlError, "OFFBOARD"):
            self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(link.commands), 0)

    def test_explicit_nonzero_clock_translation_and_heartbeat_timestamp_required(self):
        clock, link, client, runtime = self.setup_interfaces()
        client.offset = 17.0
        trace = self.run_driver(clock, link, client, runtime,
                                clock_binding=C.ClockBinding(17.0, "measured fake17"))
        self.assertEqual(len(trace), 3)
        self.assertGreater(trace[0]["state"]["t"], 17.0)
        del link.hb_wall_t
        with self.assertRaisesRegex(C.CameraControlError, "timestamp unavailable"):
            C.estimator_state(link, C.ClockBinding(17.0, "measured fake17"), clock=clock)
        with self.assertRaises(ValueError):
            C.ClockBinding(0.0, "")

    def test_full_attitude_conversion_and_no_future_heartbeat(self):
        clock, link, _, _ = self.setup_interfaces()
        link.att = (0.0, 0.2, -0.1, 0.0, 0.0, 0.0, 0.4)
        result = C.estimator_state(link, C.ClockBinding(0.0, "measured fake"), clock=clock)
        self.assertAlmostEqual(sum(x * x for x in result["quat_xyzw"]), 1.0)
        self.assertNotEqual(result["quat_xyzw"][:2], [0.0, 0.0])
        self.assertAlmostEqual(result["yaw_rate_rps"], -0.4)
        link.hb_wall_t = clock() + 0.1
        with self.assertRaisesRegex(C.CameraControlError, "future heartbeat"):
            C.estimator_state(link, C.ClockBinding(0.0, "measured fake"), clock=clock)

    def test_slow_compute_and_unbounded_output_do_not_publish(self):
        clock, link, client, runtime = self.setup_interfaces()
        def slow_step():
            clock.sleep(0.2)
            link.update()  # fresh receipts isolate the independent inference-duration bound
        runtime.after_step = slow_step
        with self.assertRaisesRegex(C.CameraControlError, "compute/control tick late"):
            self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(link.commands), 0)
        clock, link, client, runtime = self.setup_interfaces()
        runtime.step = lambda *a, **kw: {"velocity_world_enu_mps": [float("nan"), 0, 0], "yaw_rate_rps": 0}
        with self.assertRaisesRegex(C.CameraControlError, "non-finite"):
            self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(link.commands), 0)

    def test_frozen_estimator_with_fresh_heartbeat_rejects_first_matching_frame(self):
        for stale_field in ("pos_wall_t", "att_wall_t"):
            clock, link, client, runtime = self.setup_interfaces()
            clock.value = 1.0
            link.update()
            setattr(link, stale_field, clock() - 0.151)
            def fresh_heartbeat_only():
                link.hb_wall_t = clock()
            with self.subTest(stale_field=stale_field):
                with self.assertRaisesRegex(C.CameraControlError, "stale/future .* receipt"):
                    self.run_driver(clock, link, client, runtime, service=fresh_heartbeat_only)
                self.assertEqual(runtime.calls, [])
                self.assertEqual(link.commands, [])
                self.assertEqual(client.budgets, [])
        # Boot time need not resemble monotonic time: explicit receipt clocks remain fresh.
        clock, link, client, runtime = self.setup_interfaces()
        clock.value = 10000.0
        link.update()
        self.assertEqual(len(self.run_driver(clock, link, client, runtime)), 3)

    def test_each_receipt_timestamp_is_required_and_future_values_rejected(self):
        for field in ("pos_wall_t", "att_wall_t", "hb_wall_t"):
            clock, link, _, _ = self.setup_interfaces()
            delattr(link, field)
            with self.subTest(field=field), self.assertRaisesRegex(C.CameraControlError, "timestamp unavailable"):
                C.estimator_state(link, C.ClockBinding(0.0, "fake"), clock=clock)
            setattr(link, field, clock() + 0.001)
            with self.subTest(future=field), self.assertRaisesRegex(C.CameraControlError, "stale/future"):
                C.estimator_state(link, C.ClockBinding(0.0, "fake"), clock=clock)
            setattr(link, field, clock())
            self.assertEqual(C.estimator_state(link, C.ClockBinding(0.0, "fake"), clock=clock)["t"], 0.0)

    def test_atomic_snapshot_is_required_and_individual_fields_are_not_read(self):
        clock, link, _, _ = self.setup_interfaces()
        captured = link.snapshot()
        class SnapshotOnly:
            def snapshot(self):
                return captured
            def __getattr__(self, name):
                raise AssertionError("non-atomic field read %s" % name)
        self.assertEqual(C.estimator_state(SnapshotOnly(), C.ClockBinding(0.0, "fake"), clock=clock)["pos"],
                         [3.0, 4.0, 1.0])
        link.snapshot = None
        with self.assertRaisesRegex(C.CameraControlError, "atomic estimator snapshot unavailable"):
            C.estimator_state(link, C.ClockBinding(0.0, "fake"), clock=clock)

    def test_attitude_capture_alignment_uses_stricter_sensor_skew_bound(self):
        for lag, accepted in ((0.04, False), (0.015, True)):
            clock, link, client, runtime = self.setup_interfaces()
            link.boot = 1.0
            link.update()
            def lagging_attitude():
                link.service()
                link.att = (link.att[0] - lag, *link.att[1:])
            with self.subTest(lag=lag):
                if accepted:
                    self.assertEqual(len(self.run_driver(clock, link, client, runtime,
                                                         service=lagging_attitude)), 3)
                else:
                    with self.assertRaisesRegex(C.CameraControlError, "capture skew"):
                        self.run_driver(clock, link, client, runtime, service=lagging_attitude)
                    self.assertEqual(runtime.calls, [])
                    self.assertEqual(link.commands, [])

    def test_telemetry_stales_during_inference_without_command_publication(self):
        clock, link, client, runtime = self.setup_interfaces()
        def stale_position_during_step():
            clock.sleep(0.151)
            link.att_wall_t = link.hb_wall_t = clock()
        runtime.after_step = stale_position_during_step
        with self.assertRaisesRegex(C.CameraControlError, "stale/future position receipt"):
            self.run_driver(clock, link, client, runtime)
        self.assertEqual(len(runtime.calls), 1)
        self.assertEqual(link.commands, [])

    def test_camera_capture_future_to_packet_clock_is_not_hidden_by_newer_estimator(self):
        state = {"t": 10.02, "attitude_t": 10.02}
        with self.assertRaisesRegex(C.CameraControlError, "future RGB-D capture"):
            C._packet_times({"sim_now": 10.0, "rgb_t": 10.01, "depth_t": 10.0}, state, None, 0.15)
        self.assertEqual(C._packet_times({"sim_now": 10.02, "rgb_t": 10.01, "depth_t": 10.0},
                                         state, None, 0.15)[0], 10.02)


if __name__ == "__main__":
    unittest.main()
