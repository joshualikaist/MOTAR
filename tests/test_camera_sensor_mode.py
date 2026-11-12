"""CPU-only RGB-D S4 transport/model checks; no simulator, policy or aerial_gym import."""
import contextlib
import importlib.util
import pathlib
import socket
import struct
import threading
import time
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


G = load("camera_sensor_mode_gazebo", "tools/sitl_s4_gazebo.py")
M = load("camera_sensor_mode_model", "tools/sitl_s4_model.py")


def packet(*, count=1, apply=10.1, now=10.2, rgb_stamp=10.2, depth_stamp=10.2,
           lidar=False, rgb=True, depth=True):
    """Wire-v1 absence uses zero dimensions and -1 LiDAR time, as the optional plugin does."""
    ranges = np.full((G.H, G.W), 8.0, dtype="<f8").tobytes() if lidar else b""
    colour = np.full((G.CAM_H, G.CAM_W, 3), 42, dtype=np.uint8).tobytes() if rgb else b""
    distance = np.full((G.CAM_H, G.CAM_W), 4.0, dtype="<f4").tobytes() if depth else b""
    dims = ((G.H, G.W) if lidar else (0, 0)) + ((G.CAM_W, G.CAM_H) if rgb else (0, 0)) \
        + ((G.CAM_W, G.CAM_H) if depth else (0, 0))
    return G.HDR.pack(b"MS4F", 1, now, now if lidar else -1.0, rgb_stamp, depth_stamp, apply,
                      count, *dims) + ranges + colour + distance


class WirePeer:
    """A tiny request/response peer for the real S4Client, retaining the exact wire contract."""
    def __init__(self, frames, delay=0.0):
        self.frames = list(frames)
        self.delay = delay
        self.commands = []
        self.errors = []
        self.delay_active = threading.Event()
        self.client_socket, self.peer = socket.socketpair()
        self.client_socket.settimeout(0.5)
        self.peer.settimeout(0.5)
        self.client = G.S4Client.__new__(G.S4Client)
        self.client.s = self.client_socket
        self.client._budget = None
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def read(self, n):
        buf = bytearray()
        while len(buf) < n:
            block = self.peer.recv(n - len(buf))
            if not block:
                return None
            buf.extend(block)
        return bytes(buf)

    def serve(self):
        sent_baseline, applied, index = False, False, 0
        try:
            while True:
                command = self.read(1)
                if command is None:
                    return
                self.commands.append(command)
                if command == b"T":
                    xyz = self.read(24)
                    if xyz is None:
                        return
                    if struct.unpack("<3d", xyz) != (1.0, 2.0, 3.0) or not sent_baseline or applied:
                        raise AssertionError("unexpected target request")
                    applied = True
                    self.peer.sendall(struct.pack("<d", 10.05))
                elif command == b"R":
                    if not sent_baseline:
                        frame = packet(count=0, apply=-1.0, now=10.0, rgb_stamp=10.0,
                                       depth_stamp=10.0, rgb=False, depth=False)
                        sent_baseline = True
                    elif not applied:
                        raise AssertionError("poll before target request")
                    else:
                        frame = self.frames[min(index, len(self.frames) - 1)]
                        index += 1
                    if self.delay:
                        self.delay_active.set()
                        time.sleep(self.delay)
                        self.delay_active.clear()
                    self.peer.sendall(frame)
                else:
                    raise AssertionError("unexpected request %r" % command)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            # Expected when a failed readiness transaction closes its stream.
            return
        except BaseException as exc:
            self.errors.append(exc)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.client.close()
        self.peer.close()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            raise AssertionError("wire peer did not stop")
        if self.errors:
            raise AssertionError(self.errors)


class TestRgbdReadiness(unittest.TestCase):
    @staticmethod
    def wait(peer, mode="rgbd", **kwargs):
        return G.wait_initial_target(peer.client, lambda: peer.client.set_target(1, 2, 3),
                                     sensor_mode=mode, timeout_s=kwargs.pop("timeout_s", 0.4), **kwargs)

    def test_real_wire_accepts_absent_lidar_without_policy_or_tracker_calls(self):
        with WirePeer([packet()]) as peer:
            peer.client.policy = mock.Mock()
            peer.client.tracker = mock.Mock()
            frame, report = self.wait(peer)
            self.assertIsNone(frame["ranges"])
            self.assertEqual(frame["lidar_t"], -1.0)
            self.assertEqual(frame["rgb"].shape, (90, 160, 3))
            self.assertEqual(frame["depth"].shape, (90, 160))
            self.assertEqual(report["sensor_mode"], "rgbd")
            self.assertEqual(report["required_payloads"], ["rgb", "depth"])
            self.assertEqual(report["preflight_packets"], 2)
            self.assertEqual(peer.commands, [b"R", b"T", b"R"])
            self.assertEqual(peer.client.policy.mock_calls, [])
            self.assertEqual(peer.client.tracker.mock_calls, [])

    def test_legacy_default_requires_lidar_and_accepts_complete_frame(self):
        with WirePeer([packet()]) as peer:
            with self.assertRaises(G.ReadinessTimeout):
                G.wait_initial_target(peer.client, lambda: peer.client.set_target(1, 2, 3), timeout_s=0.12)
            self.assertEqual(peer.client.s.fileno(), -1)
        with WirePeer([packet(lidar=True)]) as peer:
            frame, report = G.wait_initial_target(peer.client, lambda: peer.client.set_target(1, 2, 3),
                                                   timeout_s=0.4)
            self.assertEqual(frame["ranges"].shape, (4, 72))
            self.assertEqual(report["required_payloads"], ["rgb", "depth", "ranges"])

    def test_absent_stale_or_future_camera_never_releases_barrier(self):
        bad = {"missing_rgb": dict(rgb=False), "missing_depth": dict(depth=False),
               "stale_rgb": dict(rgb_stamp=10.1), "stale_depth": dict(depth_stamp=10.1),
               "future_rgb": dict(rgb_stamp=10.3), "future_depth": dict(depth_stamp=10.3)}
        for name, change in bad.items():
            with self.subTest(name=name), WirePeer([packet(**change)]) as peer:
                with self.assertRaises(G.ReadinessTimeout):
                    self.wait(peer, timeout_s=0.12)
                self.assertEqual(peer.client.s.fileno(), -1)
        with WirePeer([packet()]) as peer:
            self.assertEqual(self.wait(peer)[1]["target_count"], 1)

    def test_nonfinite_required_camera_time_is_rejected(self):
        for field in ("rgb_stamp", "depth_stamp"):
            with self.subTest(field=field), WirePeer([packet(**{field: float("nan")})]) as peer:
                with self.assertRaises(G.ReadinessError):
                    self.wait(peer)
        with WirePeer([packet()]) as peer:
            self.assertGreater(self.wait(peer)[1]["polls"], 0)

    def test_hold_is_serviced_during_real_socket_waits(self):
        serviced_during_wait = []
        with WirePeer([packet()], delay=0.04) as peer:
            def service():
                serviced_during_wait.append(peer.delay_active.is_set())
            self.wait(peer, maintain_hold=service)
        self.assertTrue(any(serviced_during_wait))
        self.assertGreater(len(serviced_during_wait), 3)

    def test_rgbd_dict_contract_does_not_require_lidar_metadata(self):
        class DictClient:
            def __init__(self):
                self.calls = 0

            @contextlib.contextmanager
            def bounded_io(self, *args, **kwargs):
                yield

            def frames(self):
                self.calls += 1
                return dict(sim_now=10.0 if self.calls == 1 else 10.2,
                            target_t=-1 if self.calls == 1 else 10.1, target_count=int(self.calls > 1),
                            rgb_t=10.0 if self.calls == 1 else 10.2,
                            depth_t=10.0 if self.calls == 1 else 10.2,
                            rgb=np.ones((1,), np.uint8), depth=np.ones((1,), np.float32))

        frame, report = G.wait_initial_target(DictClient(), lambda: 10.05, sensor_mode="rgbd")
        self.assertNotIn("lidar_t", frame)
        self.assertEqual(report["target_count"], 1)
        with self.assertRaises(G.ReadinessError):
            G.wait_initial_target(DictClient(), lambda: 10.05)


class TestSensorModel(unittest.TestCase):
    SOURCE = ("<sdf version='1.6'><model name='navrl_quad'><link name='base_link'>"
              "<inertial><mass>0.25</mass></inertial>"
              "<collision name='body'><geometry><box><size>1 1 1</size></box></geometry></collision>"
              "<visual name='body'><geometry><box><size>1 1 1</size></box></geometry></visual>"
              "</link></model></sdf>")

    def test_rgbd_model_removes_sensor_without_changing_physics(self):
        root = ET.fromstring(M.model_sdf(self.SOURCE, sensor_mode="rgbd"))
        model = root.find("model")
        self.assertEqual(model.attrib["name"], "navrl_quad_rgbd")
        sensors = model.findall("./link/sensor")
        self.assertEqual({s.attrib["name"] for s in sensors}, {"motar_rgb", "motar_depth"})
        self.assertFalse(any(s.attrib["type"] == "gpu_ray" for s in sensors))
        self.assertEqual(model.findall(".//visual"), [])
        original = ET.fromstring(self.SOURCE)
        for name in ("inertial", "collision"):
            actual, expected = root.find(".//" + name), original.find(".//" + name)
            actual.tail = expected.tail = None  # sensor insertion adds formatting outside the physics element
            self.assertEqual(ET.tostring(actual), ET.tostring(expected))
        legacy = ET.fromstring(M.model_sdf(self.SOURCE))
        self.assertEqual(legacy.find("model").attrib["name"], "navrl_quad_s4")
        self.assertEqual(len(legacy.findall(".//sensor")), 3)
        self.assertEqual(legacy.find(".//sensor[@name='motar_lidar']").attrib["type"], "gpu_ray")

    def test_rgbd_world_has_no_lidar_plugin_requirement(self):
        world = ET.fromstring(G.world_text_s4("<sdf><world name='default'></world></sdf>",
                                              sensor_mode="rgbd", model="navrl_quad_rgbd"))
        plugin = world.find("./world/plugin")
        self.assertEqual(plugin.attrib['filename'], 'libmotar_rgbd_bridge.so')
        self.assertIsNone(plugin.find("lidar"))
        self.assertEqual(plugin.findtext("rgb"), "default::navrl_quad_rgbd::base_link::motar_rgb")
        self.assertEqual(plugin.findtext("depth"), "default::navrl_quad_rgbd::base_link::motar_depth")
        legacy = ET.fromstring(G.world_text_s4("<sdf><world name='default'></world></sdf>"))
        self.assertEqual(legacy.find('./world/plugin').attrib['filename'], 'libmotar_s4_bridge.so')
        self.assertEqual(legacy.findtext("./world/plugin/lidar"), G.SENSOR_SCOPE + "motar_lidar")

    def test_unknown_modes_are_rejected(self):
        for mode in ("", "none", "rgb", "no_lidar"):
            with self.subTest(mode=mode):
                with self.assertRaises(ValueError):
                    M.model_sdf(self.SOURCE, sensor_mode=mode)
                with self.assertRaises(ValueError):
                    G.world_text_s4("<world></world>", sensor_mode=mode)
                client = mock.Mock()
                with self.assertRaises(ValueError):
                    G.wait_initial_target(client, mock.Mock(), sensor_mode=mode)
                client.bounded_io.assert_not_called()
        self.assertIsNotNone(ET.fromstring(M.model_sdf(self.SOURCE, sensor_mode="rgbd")))


if __name__ == "__main__":
    unittest.main()
