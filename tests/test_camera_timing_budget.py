"""The real-flight capture-age allowance equals what the policy was trained with (2026-11-23).

camera_policy_contract.capture_age_bound_for(steps, dt) = (steps + 1) x dt + 0.05 s; capture_age_bound_s(contract)
reads the trained delay from the contract, and accepts only 0 until training records its action delay (review r1
P2-1). The runtime, the ROS session and the PX4 gateway use the bound for capture ages and keep MAX_AGE_S (0.15 s) for
receipt and cadence checks (review r1 P2-2). A policy trained without delay (CAM-K2 v3) keeps exactly 0.15 s. The
widened paths are exercised by patching the bound, since no lineage can carry a nonzero delay yet. Every refusal has a
positive control."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

import camera_policy_contract as C                                           # noqa: E402


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


GP = _load("camera_ros2_px4_test_helpers_timing", "tests/test_camera_ros2_px4.py")
P = GP.P                                    # the gateway module the helpers were built with


def contract(delay=None, dt=0.1):
    env = {} if delay is None else {C.TRAINED_DELAY_ENV: delay}
    return {"runtime_env": env, "observation": {"control_dt_s": dt}}


class BoundTest(unittest.TestCase):
    def test_no_trained_delay_keeps_exactly_the_former_bound(self):
        for c in (contract(), contract("0"), None, {}):
            self.assertEqual(C.capture_age_bound_s(c), 0.15)

    def test_arithmetic_for_delay_steps(self):
        self.assertEqual(C.capture_age_bound_for(0, 0.1), 0.15)
        self.assertEqual(C.capture_age_bound_for(1, 0.1), 0.25)
        self.assertEqual(C.capture_age_bound_for(2, 0.1), 0.35)
        self.assertEqual(C.capture_age_bound_for(3, 0.1), 0.45)
        for steps, dt in ((4, 0.1), (-1, 0.1), (True, 0.1), (1.0, 0.1), (3, 0.2), (0, 0.0), (0, -0.1),
                          (0, float("nan"))):
            with self.subTest(steps=steps, dt=dt), self.assertRaises(ValueError):
                C.capture_age_bound_for(steps, dt)

    def test_nonzero_delay_refused_until_training_records_it(self):
        for delay in ("1", "2", "3"):
            with self.subTest(delay=delay), self.assertRaisesRegex(ValueError, "not supported"):
                C.capture_age_bound_s(contract(delay))

    def test_malformed_delay_is_refused(self):
        for bad in ("4", "-1", "1.5", "two", 2, "", "٣"):            # last: a non-ASCII digit
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                C.capture_age_bound_s(contract(bad))
        self.assertEqual(C.capture_age_bound_s(contract("0")), 0.15)      # positive control


class GatewayTest(unittest.TestCase):
    def test_constructor_bounds(self):
        for bad in (0.1, 0.6, True, "0.35", float("nan")):
            with self.subTest(bad=bad), self.assertRaises(P.GatewayError):
                P.Px4OffboardGateway(GP.binding(), command_frame_id="map", geofence=P.geofence_template(),
                                     capture_age_s=bad)
        gate = P.Px4OffboardGateway(GP.binding(), command_frame_id="map", geofence=P.geofence_template(),
                                    capture_age_s=0.35)
        self.assertEqual(gate.capture_age_s, 0.35)
        self.assertEqual(P.Px4OffboardGateway(GP.binding(), command_frame_id="map",
                                              geofence=P.geofence_template()).capture_age_s, P.MAX_AGE_S)

    def _activated(self, capture_age_s):
        gate = P.Px4OffboardGateway(GP.binding(), command_frame_id="map", geofence=P.geofence_template(),
                                    capture_age_s=capture_age_s)
        gate.prime(GP.snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        for i in range(1, 11):
            now, mono = 100. + i / 10, 20. + i / 10
            self.assertIsNotNone(gate.update(GP.snapshot(now, mono), now_s=now, now_monotonic_s=mono), gate.reason)
        return gate

    def test_a_0_3_s_old_capture_passes_only_with_a_matching_bound(self):
        now, mono = 101.1, 21.1
        cmd = dict(GP.command(now), capture_stamp_s=now - 0.30)
        wide = self._activated(0.35)
        self.assertIsNotNone(wide.update(GP.snapshot(now, mono, nav=P.OFFBOARD), cmd,
                                         command_received_monotonic_s=mono, now_s=now, now_monotonic_s=mono),
                             wide.reason)
        self.assertEqual(wide.state, "ACTIVE")
        narrow = self._activated(P.MAX_AGE_S)                       # a policy trained without delay
        self.assertIsNone(narrow.update(GP.snapshot(now, mono, nav=P.OFFBOARD), cmd,
                                        command_received_monotonic_s=mono, now_s=now, now_monotonic_s=mono))
        self.assertEqual(narrow.state, "REVOKED")

    def test_the_wide_bound_still_revokes_beyond_it_and_on_a_stale_receipt(self):
        now, mono = 101.1, 21.1
        too_old = self._activated(0.35)
        self.assertIsNone(too_old.update(GP.snapshot(now, mono, nav=P.OFFBOARD),
                                         dict(GP.command(now), capture_stamp_s=now - 0.36),
                                         command_received_monotonic_s=mono, now_s=now, now_monotonic_s=mono))
        self.assertEqual(too_old.state, "REVOKED")
        stale_receipt = self._activated(0.35)                       # receipt keeps MAX_AGE_S
        self.assertIsNone(stale_receipt.update(GP.snapshot(now, mono, nav=P.OFFBOARD),
                                               dict(GP.command(now), capture_stamp_s=now - 0.2),
                                               command_received_monotonic_s=mono - 0.2, now_s=now,
                                               now_monotonic_s=mono))
        self.assertEqual(stale_receipt.state, "REVOKED")


class SessionTest(unittest.TestCase):
    def _session(self, runtime_contract, bound=None):
        import tempfile
        from test_camera_ros2_session import Runtime, binding
        import camera_ros2_session as S
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        runtime = Runtime()
        runtime.contract = runtime_contract
        if bound is None:
            return S.RosPolicySession(runtime, binding(Path(tmp.name) / "fixture.txt"))
        with mock.patch.object(S, "capture_age_bound_s", return_value=bound):
            return S.RosPolicySession(runtime, binding(Path(tmp.name) / "fixture.txt"))

    def test_bound_from_the_runtime_contract(self):
        self.assertEqual(self._session(None).capture_age_s, 0.15)
        self.assertEqual(self._session(contract("0")).capture_age_s, 0.15)
        with self.assertRaises(ValueError):                          # an unsupported delay refuses the session
            self._session(contract("2"))

    def _offer(self, session, t, receipt):
        from test_camera_ros2_session import messages
        for kind, message in messages(t).items():
            session.offer(kind, message, received_monotonic_s=receipt)

    def test_a_0_3_s_old_bundle_is_used_only_with_a_wide_bound(self):
        for bound, usable in ((0.35, True), (None, False)):
            session = self._session(None, bound)
            self._offer(session, 3.0, 100.25)                       # receipt 0.05 s before the tick
            item = session.prepare(now_ros_s=3.30, now_monotonic_s=100.30)    # capture 0.30 s old
            with self.subTest(bound=bound):
                if not usable:
                    self.assertIsNone(item)
                    continue
                self.assertIsNotNone(item)
                proposal = session.finish(item, session.infer(item), now_ros_s=3.30, now_monotonic_s=100.30)
                self.assertAlmostEqual(proposal["valid_until_s"] - proposal["capture_stamp_s"], 0.35, places=9)

    def test_receipt_age_at_publication_keeps_0_15_s(self):
        import camera_ros2_session as S
        ok = self._session(None, 0.35)
        self._offer(ok, 3.0, 100.12)
        item = ok.prepare(now_ros_s=3.25, now_monotonic_s=100.25)
        self.assertIsNotNone(item)
        self.assertIsNotNone(ok.finish(item, ok.infer(item), now_ros_s=3.25, now_monotonic_s=100.25))
        late = self._session(None, 0.35)
        self._offer(late, 3.0, 100.12)
        item = late.prepare(now_ros_s=3.25, now_monotonic_s=100.25)
        self.assertIsNotNone(item)
        with self.assertRaisesRegex(S.SessionFault, "expired"):     # receipt 0.20 s old > 0.15 s
            late.finish(item, late.infer(item), now_ros_s=3.32, now_monotonic_s=100.32)


class RuntimeTest(unittest.TestCase):
    def _runtime(self):
        import camera_policy_runtime as R
        r = object.__new__(R.CameraPolicyRuntime)
        r.contract, r._last_stamps, r._last_now = contract(), None, None
        return R, r

    def _timing(self, r, age):
        now = 10.0
        stamp = now - age
        return r._validate_timing({"rgb_t": stamp, "depth_t": stamp}, {"t": stamp}, now)

    def test_runtime_uses_the_contract_bound(self):
        R, r = self._runtime()
        self.assertEqual(r.capture_age_bound_s, 0.15)
        self._timing(r, 0.149)
        _, r = self._runtime()
        with self.assertRaises(R.InputError):
            self._timing(r, 0.16)
        import camera_policy_contract
        _, r = self._runtime()
        with mock.patch.object(camera_policy_contract, "capture_age_bound_s", return_value=0.35):
            self.assertEqual(r.capture_age_bound_s, 0.35)
            self._timing(r, 0.30)
            _, r2 = self._runtime()
            with self.assertRaises(R.InputError):
                self._timing(r2, 0.36)


if __name__ == "__main__":
    unittest.main()
