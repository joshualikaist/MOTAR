"""Ducktyped ROS message controls; no rclpy, graph, camera or policy construction."""
import copy
import hashlib
import importlib.util
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("camera_ros2_adapter_tested", ROOT / "tools/camera_ros2_adapter.py")
C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(C)


def header(frame, sec=3, nanos=0):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanos))


def image(array, encoding, *, big=False, padding=0):
    rows = [array[y].tobytes() + bytes(padding) for y in range(array.shape[0])]
    return NS(header=header("camera_optical"), height=array.shape[0], width=array.shape[1],
              step=len(rows[0]), is_bigendian=int(big), encoding=encoding, data=b"".join(rows))


def info():
    # A real sensor_msgs/CameraInfo always carries D; empty D = no distortion (all pixels covered).
    return NS(header=header("camera_optical"), width=2, height=2, binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
              k=[90., 0, 0.5, 0, 90., 0.5, 0, 0, 1], r=np.eye(3).ravel().tolist(),
              p=[20., 0, 0.5, 0, 0, 21., 0.5, 0, 0, 0, 1, 0], d=[], distortion_model="")


def odometry(yaw=0.0):
    return NS(header=header("odom"), child_frame_id="base_link",
              pose=NS(pose=NS(position=NS(x=1., y=2., z=1.),
                              orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))),
              twist=NS(twist=NS(linear=NS(x=1., y=0., z=0.), angular=NS(x=0., y=0., z=0.3))))


class RosConvertersTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path = Path(self.tmp.name) / "fixture_only_measurement.txt"
        path.write_text("CPU fixture measurement, not actual deployment calibration")
        proof = {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        self.binding = C.unconfigured_template()
        self.binding.update(status="CONFIGURED", frames={"world": "odom", "body": "base_link", "optical": "camera_optical"})
        self.binding["geometry"] = {"mount_flu_m": [0.1, 0, 0.03],
                                    "optical_to_body_rotation": C.CAMERA_FLU_TO_OPTICAL.T.tolist(),
                                    "provenance": dict(proof)}
        self.binding["producer"] = {"rectified": True, "depth_aligned_to_rgb": True, "provenance": dict(proof)}
        self.binding["clock"]["source"] = "fake shared ROS acquisition clock"
        self.rgb = np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], np.uint8)
        self.z = np.array([[1000, 0], [2500, 400]], np.uint16)

    def test_rgb8_bgr8_padding_and_exact_buffer(self):
        for encoding, values in (("rgb8", self.rgb), ("bgr8", self.rgb[:, :, ::-1])):
            msg = image(values, encoding, padding=5)
            decoded = C.decode_rgb(msg)
            np.testing.assert_array_equal(decoded, self.rgb)
            self.assertTrue(decoded.flags.owndata)
            msg.data += b"\x00"
            with self.assertRaisesRegex(C.RosInputError, "buffer size"):
                C.decode_rgb(msg)
        bad = image(self.rgb, "rgb8")
        bad.step -= 1
        with self.assertRaisesRegex(C.RosInputError, "row step"):
            C.decode_rgb(bad)

    def test_depth_mm_both_endians_padding_and_float_invalid(self):
        for big in (False, True):
            source = self.z.astype(">u2" if big else "<u2")
            z, invalid = C.decode_depth(image(source, "16UC1", big=big, padding=3))
            np.testing.assert_allclose(z, [[1., 0.], [2.5, 0.4]], atol=1e-6)
            np.testing.assert_array_equal(invalid, [[False, True], [False, False]])
        # REP 117 (r2, 2026-11-12): +Inf is "no return within range", not a sensor-invalid return. It used to
        # be pinned here as invalid, which made a dense +Inf region read as a 0.4 m surface under blob_near.
        # It now stays z = +inf with invalid False, which the runtime classifies as z > MaxZ -> no-return.
        # -Inf (too close), NaN (invalid) and a finite z <= 0 remain sensor-invalid with z = 0.
        source = np.array([[1.5, float("nan")], [-1., float("inf")]], ">f4")
        z, invalid = C.decode_depth(image(source, "32FC1", big=True, padding=4))
        np.testing.assert_array_equal(z, [[1.5, 0], [0, np.inf]])
        np.testing.assert_array_equal(invalid, [[False, True], [True, False]])
        source = np.array([[float("-inf"), 0.], [2.5, float("inf")]], "<f4")
        z, invalid = C.decode_depth(image(source, "32FC1"))
        np.testing.assert_array_equal(z, [[0, 0], [2.5, np.inf]])
        np.testing.assert_array_equal(invalid, [[True, True], [False, False]])
        with self.assertRaisesRegex(C.RosInputError, "encoding"):
            C.decode_depth(image(self.z, "mono16"))

    def test_camera_info_uses_rectified_p_instead_of_raw_k(self):
        result = C.rectified_camera_info(info(), expected_frame="camera_optical", width=2, height=2)
        self.assertEqual(result["fx"], 20.)
        self.assertEqual(result["fy"], 21.)
        for change in ("frame", "binning", "roi", "offset", "rotation", "uncalibrated"):
            bad = info()
            if change == "frame": bad.header.frame_id = "other"
            if change == "binning": bad.binning_x = 2
            if change == "roi": bad.roi.x_offset = 1
            if change == "offset": bad.p[3] = -0.2
            if change == "rotation": bad.r = [0, -1, 0, 1, 0, 0, 0, 0, 1]
            if change == "uncalibrated": bad.k[0] = 0
            with self.assertRaises(C.RosInputError, msg=change):
                C.rectified_camera_info(bad, expected_frame="camera_optical", width=2, height=2)

    def test_enu_flu_child_velocity_rotates_once_to_world(self):
        state = C.odometry_state(odometry(math.pi/2), expected_world_frame="odom", expected_body_frame="base_link")
        self.assertEqual(set(state), {"pos", "vel", "quat_xyzw", "yaw_rate_rps", "t"})
        np.testing.assert_allclose(state["vel"], [0, 1, 0], atol=1e-12)
        self.assertEqual(state["pos"], [1., 2., 1.])
        self.assertEqual(state["yaw_rate_rps"], 0.3)
        self.assertEqual(state["t"], 3.)
        bad = odometry()
        bad.child_frame_id = "base_frd"
        with self.assertRaisesRegex(C.RosInputError, "child_frame_id"):
            C.odometry_state(bad, expected_world_frame="odom", expected_body_frame="base_link")
        bad = odometry()
        bad.pose.pose.orientation.w = 0.
        with self.assertRaisesRegex(C.RosInputError, "normalized"):
            C.odometry_state(bad, expected_world_frame="odom", expected_body_frame="base_link")

    def test_complete_frame_preserves_stamps_and_optical_basis_becomes_identity(self):
        self.assertEqual(C.validate_binding(self.binding), self.binding)
        frame, state = C.build_frame(image(self.rgb, "rgb8"), image(self.z, "16UC1"), info(), odometry(), self.binding)
        np.testing.assert_array_equal(frame["rgb"], self.rgb)
        np.testing.assert_allclose(frame["camera_to_body_rotation"], np.eye(3), atol=1e-12)
        self.assertEqual((frame["rgb_t"], frame["depth_t"], state["t"]), (3., 3., 3.))
        self.assertTrue(frame["invalid"][0, 1])
        self.assertNotIn("ranges", frame)
        self.assertTrue(frame["aligned"] and frame["rectified"])

    def test_odometry_bracket_interpolates_to_rgb_time_and_refuses_non_bracketing_pairs(self):
        rgb, depth, cam = image(self.rgb, "rgb8"), image(self.z, "16UC1"), info()
        before, after = odometry(), odometry()
        before.header.stamp = NS(sec=2, nanosec=990000000)
        after.header.stamp = NS(sec=3, nanosec=10000000)
        after.pose.pose.position.x = 1.04
        frame, state = C.build_frame(rgb, depth, cam, after, self.binding, odometry_bracket=(before, after))
        self.assertTrue(frame["pose_provenance"]["pose_interpolated"])
        self.assertEqual(state["t"], 3.0)
        self.assertAlmostEqual(state["pos"][0], 1.02, places=9)
        self.assertEqual(set(state), {"pos", "vel", "quat_xyzw", "yaw_rate_rps", "t"})
        # Without a bracket the nearest sample is used as before (control).
        frame, state = C.build_frame(rgb, depth, cam, after, self.binding)
        self.assertFalse(frame["pose_provenance"]["pose_interpolated"])
        self.assertAlmostEqual(frame["pose_provenance"]["pose_skew_s"], 0.01, places=9)
        self.assertEqual(state["pos"][0], 1.04)
        with self.assertRaisesRegex(C.RosInputError, "extrapolation"):
            C.build_frame(rgb, depth, cam, after, self.binding, odometry_bracket=(after, after))

    def test_frame_alignment_clock_skew_and_unconfigured_binding_rejected(self):
        rgb, depth, cam, odom = image(self.rgb, "rgb8"), image(self.z, "16UC1"), info(), odometry()
        with self.assertRaisesRegex(C.RosInputError, "UNCONFIGURED"):
            C.build_frame(rgb, depth, cam, odom, C.unconfigured_template())
        depth.header.stamp.nanosec = 26000000
        with self.assertRaisesRegex(C.RosInputError, "skew"):
            C.build_frame(rgb, depth, cam, odom, self.binding)
        depth.header.stamp.nanosec = 0
        depth.header.frame_id = "unregistered_depth_optical"
        with self.assertRaisesRegex(C.RosInputError, "frame_id"):
            C.build_frame(rgb, depth, cam, odom, self.binding)
        bad = copy.deepcopy(self.binding)
        bad["producer"]["depth_aligned_to_rgb"] = False
        with self.assertRaisesRegex(C.RosInputError, "attest"):
            C.validate_binding(bad)

    def test_binding_provenance_is_checked_before_callbacks(self):
        good = C.validate_binding(self.binding)
        self.assertIsNot(good, self.binding)
        bad = copy.deepcopy(self.binding)
        bad["unrecognized_transform"] = "ned"
        with self.assertRaisesRegex(C.RosInputError, "keys differ"):
            C.validate_binding(bad)
        bad = copy.deepcopy(self.binding)
        bad["geometry"]["unrecognized_transform"] = "ned"
        with self.assertRaisesRegex(C.RosInputError, "keys differ"):
            C.validate_binding(bad)
        Path(good["geometry"]["provenance"]["path"]).write_text("changed")
        with self.assertRaisesRegex(C.RosInputError, "SHA256 changed"):
            C.validate_binding(self.binding)
        # Callback conversion performs structural checks, not repeated file I/O.
        C.validate_binding(self.binding, verify_files=False)

    def test_malformed_ros_stamp_is_never_replaced_with_now(self):
        self.assertEqual(C.stamp_seconds(NS(sec=3, nanosec=500000000)), 3.5)
        for sec, nanos in ((0, 0), (-1, 0), (True, 0), (3.0, 0), (3, 1000000000), (3, -1)):
            with self.assertRaisesRegex(C.RosInputError, "timestamp"):
                C.stamp_seconds(NS(sec=sec, nanosec=nanos))


if __name__ == "__main__":
    unittest.main()
