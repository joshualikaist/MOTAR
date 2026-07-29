import importlib.util
import inspect
from pathlib import Path
import types
import unittest

import torch

_MODULE_PATH = Path(__file__).parents[1] / "aerial_gym/task/navrl_task/navrl_perception.py"
_SPEC = importlib.util.spec_from_file_location("navrl_perception_standalone", _MODULE_PATH)
_PERCEPTION = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_PERCEPTION)
NavRLPerceptionModule = _PERCEPTION.NavRLPerceptionModule
STRUCTURED_OBS_DIM = _PERCEPTION.STRUCTURED_OBS_DIM


def _configs():
    camera = types.SimpleNamespace(
        detector_max_range=20.0,
        detector_hfov_deg=87.0,
        detector_vfov_deg=58.0,
        camera_width=80,
        camera_height=45,
        camera_translation=[0.1, 0.0, 0.03],
        camera_target_radius=0.15,
        tracker_memory_s=5.0,
    )
    perception = types.SimpleNamespace(
        lidar_max_range=4.0,
        min_target_pixels=2,
        pixel_threshold=0.55,
        detection_dropout_prob=0.0,
        rgb_noise_std=0.0,
        depth_noise_std=0.0,
        history_interval_s=0.5,
        detector_checkpoint="",
    )
    return camera, perception


class NavRLPerceptionTest(unittest.TestCase):
    def setUp(self):
        camera, perception = _configs()
        self.module = NavRLPerceptionModule(2, "cpu", perception, 0.1, camera)
        self.rgb = torch.full((2, 3, 45, 80), 0.15)
        self.depth = torch.full((2, 45, 80), 10.0)
        self.rgb[0, 0, 20:24, 38:43] = 0.88
        self.rgb[0, 1, 20:24, 38:43] = 0.08
        self.rgb[0, 2, 20:24, 38:43] = 0.045
        self.depth[0, 20:24, 38:43] = 6.0
        self.lidar = torch.full((2, 144), 4.0)
        self.lidar[0, 18] = 2.0
        self.pos = torch.zeros(2, 3)
        self.vel = torch.zeros(2, 3)
        self.quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(2, 1)

    def _observe(self):
        return self.module.observe(
            self.rgb,
            self.depth,
            self.lidar,
            self.pos,
            self.vel,
            self.quat,
            torch.zeros(2),
            torch.zeros(2, 4),
            2.0,
            1.0,
            training=False,
        )

    def test_raw_rgbd_produces_track_without_semantic_input(self):
        obs, diag = self._observe()
        self.assertEqual(tuple(obs.shape), (2, STRUCTURED_OBS_DIM))
        self.assertTrue(bool(diag["visible"][0]))
        self.assertFalse(bool(diag["visible"][1]))
        self.assertGreater(float(diag["confidence"][0]), 0.5)
        self.assertTrue(torch.isfinite(obs).all())

    def test_actor_perception_api_has_no_oracle_argument(self):
        names = set(inspect.signature(self.module.observe).parameters)
        forbidden = {"target_position", "target_velocity", "target_mask", "semantic_id"}
        self.assertFalse(names & forbidden)

    def test_uncertainty_grows_during_occlusion(self):
        self._observe()
        p_seen = torch.diagonal(self.module.tracker.cov[0]).sum().item()
        self.rgb[:] = 0.15
        for _ in range(4):
            _, diag = self._observe()
        p_hidden = torch.diagonal(self.module.tracker.cov[0]).sum().item()
        self.assertFalse(bool(diag["visible"][0]))
        self.assertGreater(p_hidden, p_seen)
        self.assertGreater(float(diag["track_age"][0]), 0.0)

    def test_lidar_range_can_continue_camera_initialized_track(self):
        self.depth[0, 20:24, 38:43] = 3.0
        self.lidar[0] = 4.0
        self._observe()
        self.rgb[:] = 0.15
        # Target sits dead ahead (bearing 0). Under the PHYSICAL bin convention (bearing
        # decreases with the index: 180 - 10*j at 36 beams) that is bin 18, not bin 17 --
        # the old fixture value 17 encoded the mirrored table this test must now reject.
        self.lidar[0, 18] = 2.85
        _, diag = self._observe()
        self.assertFalse(bool(diag["camera_visible"][0]))
        self.assertTrue(bool(diag["lidar_visible"][0]))
        self.assertTrue(bool(diag["visible"][0]))


if __name__ == "__main__":
    unittest.main()
