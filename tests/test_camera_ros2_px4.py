"""Pure message/ownership checks; no ROS installation, graph, simulator, GPU or vehicle."""
import hashlib
import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("camera_ros2_px4_tested", ROOT / "tools/camera_ros2_px4.py")
P = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = P
SPEC.loader.exec_module(P)


def binding():
    return P.Px4MessageBinding(P.PX4_VERSION, dict(P.PX4_MESSAGE_SHA256), "explicit local message-build fixture")


def snapshot(now=100., mono=20., *, nav=2, armed=2, z=-3.4):
    timestamp, sample = int(round(now*1e6)), int(round((now-0.005)*1e6))
    return {"odometry": {"timestamp": timestamp, "timestamp_sample": sample, "pose_frame": 1, "velocity_frame": 1,
                          "position": [10., 20., z], "velocity": [1., 2., -0.3], "q": [1., 0., 0., 0.],
                          "angular_velocity": [0.1, 0.2, 0.3], "reset_counter": 0},
            "local_position": dict(timestamp=timestamp, timestamp_sample=sample, xy_valid=True, z_valid=True,
                                   v_xy_valid=True, v_z_valid=True, heading_good_for_control=True,
                                   **{name: 0 for name in P.RESET_FIELDS}),
            "status": {"timestamp": timestamp, "nav_state": nav, "arming_state": armed, "failsafe": False},
            "receipts": {name: mono for name in ("odometry", "local_position", "status")}}


def command(now):
    return {"capture_stamp_s": now-0.01, "frame_id": "map", "velocity_enu_mps": [1., 2., 0.],
            "yaw_rate_enu_rps": 0.3}


def decode(s, now=100., mono=20.):
    return P.decode_estimator(s["odometry"], s["local_position"], s["status"],
                              now_s=now, now_monotonic_s=mono, receipts=s["receipts"])


def activate():
    gate = P.Px4OffboardGateway(binding(), command_frame_id="map")
    gate.prime(snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
    for i in range(1, 11):
        now, mono = 100.+i/10, 20.+i/10
        result = gate.update(snapshot(now, mono), now_s=now, now_monotonic_s=mono)
        if result is None:
            raise AssertionError(gate.reason)
    now, mono = 101.1, 21.1
    result = gate.update(snapshot(now, mono, nav=P.OFFBOARD), command(now),
                         command_received_monotonic_s=mono, now_s=now, now_monotonic_s=mono)
    if gate.state != "ACTIVE" or result is None:
        raise AssertionError(gate.reason)
    return gate


class MessageBindingTest(unittest.TestCase):
    def test_schema_binding_is_explicit_and_never_claims_hardware_qualification(self):
        good = binding()
        self.assertEqual(good.metadata()["hardware_compatibility"], "NOT_TESTED")
        self.assertEqual(good.metadata()["dds_delivery"], "NOT_TESTED")
        for version, hashes, source in (("main", P.PX4_MESSAGE_SHA256, "fixture"),
                                        (P.PX4_VERSION, {}, "fixture"),
                                        (P.PX4_VERSION, P.PX4_MESSAGE_SHA256, "")):
            with self.subTest(version=version, source=source), self.assertRaises(P.GatewayError):
                P.Px4MessageBinding(version, hashes, source)
        with self.assertRaises(TypeError):
            good.message_sha256["VehicleStatus.msg"] = "x"
        with self.assertRaises(P.GatewayError):
            P.Px4OffboardGateway(None, command_frame_id="map")

    def test_pinned_definitions_match_local_primary_source_when_available(self):
        msg = Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/msg")
        if not msg.is_dir():
            self.skipTest("local PX4 message source unavailable; hardware compatibility remains NOT_TESTED")
        for name, digest in P.PX4_MESSAGE_SHA256.items():
            self.assertEqual(hashlib.sha256((msg / name).read_bytes()).hexdigest(), digest, name)


class EstimatorDecoderTest(unittest.TestCase):
    def test_north_and_east_attitudes_and_enu_flu_units(self):
        s = snapshot()
        result = decode(s)
        self.assertEqual(set(result), {"pos", "vel", "quat_xyzw", "yaw_rate_rps", "t"})
        self.assertEqual(result["pos"], [20., 10., 3.4])
        self.assertEqual(result["vel"], [2., 1., 0.3])
        self.assertAlmostEqual(result["yaw_rate_rps"], -0.3)
        self.assertAlmostEqual(result["t"], 99.995)
        np.testing.assert_allclose(result["quat_xyzw"], [0., 0., math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)
        s["odometry"]["q"] = [math.sqrt(0.5), 0., 0., math.sqrt(0.5)]
        np.testing.assert_allclose(decode(s)["quat_xyzw"], [0., 0., 0., 1.], atol=1e-12)
        # A 180-degree roll exercises the quaternion branch away from a positive rotation trace.
        s["odometry"]["q"] = [0., 1., 0., 0.]
        q = decode(s)["quat_xyzw"]
        self.assertAlmostEqual(sum(x*x for x in q), 1.)
        self.assertGreater(abs(q[0])+abs(q[1]), 1.)

    def test_required_validity_frames_and_body_rates_have_positive_control(self):
        changes = [("odometry", "pose_frame", 2), ("odometry", "velocity_frame", 3),
                   ("odometry", "q", [1., 1., 0., 0.]),
                   ("odometry", "angular_velocity", [0., 0., float("nan")]),
                   ("status", "failsafe", True), ("status", "arming_state", True)]
        changes += [("local_position", field, False) for field in
                    ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control")]
        for section, field, value in changes:
            s = snapshot()
            s[section][field] = value
            with self.subTest(section=section, field=field), self.assertRaises(P.GatewayError):
                decode(s)
        s = snapshot()
        del s["odometry"]["reset_counter"]
        with self.assertRaises(P.GatewayError):
            decode(s)
        self.assertGreater(decode(snapshot())["pos"][2], 0.)

    def test_capture_and_receipt_ages_are_different_clock_domains(self):
        self.assertAlmostEqual(decode(snapshot(mono=123456.), mono=123456.)["t"], 99.995)
        for section in ("odometry", "local_position", "status"):
            s = snapshot()
            s["receipts"][section] = 20.001
            with self.subTest(future=section), self.assertRaisesRegex(P.GatewayError, "receipt"):
                decode(s)
            s["receipts"][section] = 17.
            with self.subTest(stale=section), self.assertRaisesRegex(P.GatewayError, "receipt"):
                decode(s)
        for stamp in (99.8, 100.001):
            s = snapshot()
            s["odometry"]["timestamp_sample"] = int(stamp*1e6)
            with self.subTest(stamp=stamp), self.assertRaises(P.GatewayError):
                decode(s)
        s = snapshot()
        s["local_position"]["timestamp_sample"] -= 40000
        with self.assertRaisesRegex(P.GatewayError, "skew"):
            decode(s)


class SetpointEncoderTest(unittest.TestCase):
    def test_mixed_xy_velocity_and_current_z_hold_with_explicit_nans(self):
        result = P.encode_setpoint(command(100.), hold_z_ned_m=-3.4, timestamp_us=100000000)
        self.assertEqual(set(result), {"offboard_control_mode", "trajectory_setpoint"})
        mode, target = result["offboard_control_mode"], result["trajectory_setpoint"]
        self.assertTrue(mode["position"])
        self.assertTrue(all(mode[name] is False for name in ("velocity", "acceleration", "attitude", "body_rate", "actuator")))
        self.assertTrue(all(math.isnan(x) for x in target["position"][:2]))
        self.assertEqual(target["position"][2], -3.4)
        self.assertEqual(target["velocity"][:2], [2., 1.])
        self.assertTrue(math.isnan(target["velocity"][2]))
        self.assertTrue(all(math.isnan(x) for x in target["acceleration"]+target["jerk"]+[target["yaw"]]))
        self.assertEqual(target["yawspeed"], -0.3)

    def test_policy_bounds_and_missing_fields_refuse_without_arming_messages(self):
        for change in ({"velocity_enu_mps": [1., 2., 1.]}, {"velocity_enu_mps": [4., 4., 0.]},
                       {"velocity_enu_mps": [float("nan"), 0., 0.]}, {"yaw_rate_enu_rps": 3.1}):
            with self.subTest(change=change), self.assertRaises(P.GatewayError):
                P.encode_setpoint(dict(command(100.), **change), hold_z_ned_m=-3.4, timestamp_us=100000000)
        self.assertEqual(P.encode_setpoint(command(100.), hold_z_ned_m=-3.4, timestamp_us=100000000)
                          ["trajectory_setpoint"]["timestamp"], 100000000)


class PublicationGatewayTest(unittest.TestCase):
    def test_disabled_then_continuous_neutral_prime_then_manual_active(self):
        g = P.Px4OffboardGateway(binding(), command_frame_id="map")
        self.assertIsNone(g.update({}, now_s=100., now_monotonic_s=20.))
        result = g.prime(snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        self.assertEqual(g.state, "PRIMING")
        self.assertEqual(result["trajectory_setpoint"]["position"][2], -3.4)
        self.assertEqual(result["trajectory_setpoint"]["velocity"][:2], [0., 0.])
        active = activate()
        self.assertEqual(active.state, "ACTIVE")
        self.assertEqual(active._command_contract, "oldest_capture_twist_enu_v1")

    def test_elapsed_time_without_continuous_proposals_cannot_claim_priming(self):
        g = P.Px4OffboardGateway(binding(), command_frame_id="map")
        g.prime(snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        self.assertIsNone(g.update(snapshot(101.1, 21.1, nav=14), command(101.1),
                                   command_received_monotonic_s=21.1, now_s=101.1, now_monotonic_s=21.1))
        self.assertEqual(g.state, "REVOKED")
        self.assertIn("stream interrupted", g.reason)
        self.assertEqual(activate().state, "ACTIVE")

    def test_early_offboard_or_unarmed_activation_refuses(self):
        g = P.Px4OffboardGateway(binding(), command_frame_id="map")
        g.prime(snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        self.assertIsNone(g.update(snapshot(100.1, 20.1, nav=14), command(100.1),
                                   command_received_monotonic_s=20.1, now_s=100.1, now_monotonic_s=20.1))
        self.assertIn("before continuous priming", g.reason)
        g = P.Px4OffboardGateway(binding(), command_frame_id="map")
        with self.assertRaisesRegex(P.GatewayError, "POSITION hover before OFFBOARD"):
            g.prime(snapshot(nav=14), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        g = P.Px4OffboardGateway(binding(), command_frame_id="map")
        g.prime(snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        for i in range(1, 11):
            g.update(snapshot(100+i/10, 20+i/10), now_s=100+i/10, now_monotonic_s=20+i/10)
        self.assertIsNone(g.update(snapshot(101.1, 21.1, nav=14, armed=1), command(101.1),
                                   command_received_monotonic_s=21.1, now_s=101.1, now_monotonic_s=21.1))
        self.assertIn("disarmed during manual-hover handover", g.reason)
        self.assertEqual(activate().state, "ACTIVE")

    def test_duplicate_command_does_not_renew_first_receipt_or_capture(self):
        g = activate()
        duplicate = command(101.1)
        original_receipt = g._command_receipt
        result = g.update(snapshot(101.2, 21.2, nav=14), duplicate,
                           command_received_monotonic_s=21.2, now_s=101.2, now_monotonic_s=21.2)
        self.assertIsNotNone(result)
        self.assertEqual(g._command_receipt, original_receipt)
        self.assertIsNone(g.update(snapshot(101.3, 21.3, nav=14), duplicate,
                                   command_received_monotonic_s=21.3, now_s=101.3, now_monotonic_s=21.3))
        self.assertEqual(g.state, "REVOKED")
        self.assertIsNone(g.update(snapshot(101.4, 21.4, nav=14), command(101.4),
                                   command_received_monotonic_s=21.4, now_s=101.4, now_monotonic_s=21.4))

    def test_same_capture_changed_payload_and_wrong_frame_revoke(self):
        for cmd in (dict(command(101.1), velocity_enu_mps=[0., 0., 0.]),
                    dict(command(101.2), frame_id="unbound_odom")):
            g = activate()
            self.assertIsNone(g.update(snapshot(101.2, 21.2, nav=14), cmd,
                                       command_received_monotonic_s=21.2, now_s=101.2, now_monotonic_s=21.2))
            self.assertEqual(g.state, "REVOKED")
        self.assertEqual(activate().state, "ACTIVE")

    def test_capture_ttl_and_receipt_ttl_are_independently_required(self):
        for capture, receipt in ((101.2-0.151, 21.2), (101.201, 21.2), (101.19, 21.0)):
            g = activate()
            cmd = dict(command(101.2), capture_stamp_s=capture)
            with self.subTest(capture=capture, receipt=receipt):
                self.assertIsNone(g.update(snapshot(101.2, 21.2, nav=14), cmd,
                                           command_received_monotonic_s=receipt, now_s=101.2, now_monotonic_s=21.2))
                self.assertEqual(g.state, "REVOKED")
        g = activate()
        self.assertIsNotNone(g.update(snapshot(101.2, 21.2, nav=14), command(101.2),
                                      command_received_monotonic_s=21.2, now_s=101.2, now_monotonic_s=21.2))

    def test_duplicate_estimator_sample_does_not_renew_receive_freshness(self):
        g = activate()
        repeated = snapshot(101.2, 21.2, nav=14)
        repeated["odometry"]["timestamp_sample"] = int(round(101.095*1e6))
        repeated["local_position"]["timestamp_sample"] = int(round(101.095*1e6))
        self.assertIsNotNone(g.update(repeated, command(101.2), command_received_monotonic_s=21.2,
                                      now_s=101.2, now_monotonic_s=21.2))
        self.assertEqual(g._message_receipts["odometry"][1], 21.1)
        repeated["odometry"]["timestamp"] = repeated["local_position"]["timestamp"] = int(round(101.3*1e6))
        repeated["status"]["timestamp"] = int(round(101.3*1e6))
        repeated["receipts"] = dict.fromkeys(repeated["receipts"], 21.3)
        self.assertIsNone(g.update(repeated, command(101.3), command_received_monotonic_s=21.3,
                                   now_s=101.3, now_monotonic_s=21.3))
        self.assertEqual(g.state, "REVOKED")

    def test_changed_pose_under_same_estimator_capture_is_rejected(self):
        g = activate()
        changed = snapshot(101.2, 21.2, nav=14)
        changed["odometry"]["timestamp_sample"] = int(round(101.095*1e6))
        changed["odometry"]["position"][0] += 1.
        self.assertIsNone(g.update(changed, command(101.2), command_received_monotonic_s=21.2,
                                   now_s=101.2, now_monotonic_s=21.2))
        self.assertIn("same capture identity", g.reason)
        self.assertEqual(activate().state, "ACTIVE")

    def test_operator_disable_is_latched(self):
        g = activate()
        g.disable()
        self.assertEqual(g.state, "REVOKED")
        self.assertIsNone(g.update(snapshot(101.2, 21.2, nav=14), command(101.2),
                                   command_received_monotonic_s=21.2, now_s=101.2, now_monotonic_s=21.2))
        with self.assertRaises(P.GatewayError):
            g.prime(snapshot(101.2, 21.2), now_s=101.2, now_monotonic_s=21.2, manual_hover_handover=True)

    def test_mode_loss_failsafe_reset_or_stale_telemetry_stop_both_streams(self):
        variants = []
        for change in ("mode", "disarm", "failsafe", "reset", "stale"):
            s = snapshot(101.2, 21.2, nav=14)
            if change == "mode":
                s["status"]["nav_state"] = 2
            elif change == "disarm":
                s["status"]["arming_state"] = 1
            elif change == "failsafe":
                s["status"]["failsafe"] = True
            elif change == "reset":
                s["local_position"]["xy_reset_counter"] += 1
            else:
                s["receipts"]["odometry"] = 20.9
            variants.append((change, s))
        for change, s in variants:
            g = activate()
            with self.subTest(change=change):
                self.assertIsNone(g.update(s, command(101.2), command_received_monotonic_s=21.2,
                                           now_s=101.2, now_monotonic_s=21.2))
                self.assertEqual(g.state, "REVOKED")
                self.assertIsNone(g._command)
        self.assertEqual(activate().state, "ACTIVE")

    def test_clock_jumps_or_stream_gaps_latch_revocation(self):
        for now, mono in ((101.3, 21.2), (101.0, 21.2), (101.2, 21.0), (101.3, 21.3)):
            g = activate()
            with self.subTest(now=now, mono=mono):
                self.assertIsNone(g.update(snapshot(now, mono, nav=14), command(now),
                                           command_received_monotonic_s=mono, now_s=now, now_monotonic_s=mono))
                self.assertEqual(g.state, "REVOKED")
        self.assertEqual(activate().state, "ACTIVE")

    def test_native_capture_contract_can_activate_without_inventing_twist_stamps(self):
        g = P.Px4OffboardGateway(binding(), command_frame_id="map")
        g.prime(snapshot(), now_s=100., now_monotonic_s=20., manual_hover_handover=True)
        for i in range(1, 11):
            g.update(snapshot(100+i/10, 20+i/10), now_s=100+i/10, now_monotonic_s=20+i/10)
        cmd = {"velocity_world_enu_mps": [1., 2., 0.], "yaw_rate_rps": 0.3, "t": 101.1,
               "rgb_t": 101.09, "depth_t": 101.091, "state_t": 101.095}
        self.assertIsNotNone(g.update(snapshot(101.1, 21.1, nav=14), cmd,
                                      command_received_monotonic_s=21.1, now_s=101.1, now_monotonic_s=21.1))
        self.assertEqual(g._capture, (101.09, 101.091, 101.095))
        self.assertIsNone(g.update(snapshot(101.2, 21.2, nav=14), command(101.2),
                                   command_received_monotonic_s=21.2, now_s=101.2, now_monotonic_s=21.2))
        self.assertIn("source contract changed", g.reason)


if __name__ == "__main__":
    unittest.main()
