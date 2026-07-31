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
select_cluster_sector_obstacles = _PERCEPTION.select_cluster_sector_obstacles


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


class ClusterSectorSelectorTest(unittest.TestCase):
    def setUp(self):
        self.bearings = torch.deg2rad(torch.arange(-120.0, 125.0, 5.0))
        self.max_range = 12.0

    def _select(self, scan, *, slots=4, sectors=4):
        return select_cluster_sector_obstacles(
            torch.tensor([scan], dtype=torch.float32),
            self.bearings,
            max_range=self.max_range,
            max_obstacles=slots,
            token_fov_deg=240.0,
            cluster_gap_m=0.45,
            num_sectors=sectors,
        )

    def test_adjacent_surface_returns_consume_one_slot(self):
        scan = [self.max_range] * len(self.bearings)
        # Three adjacent 5-degree returns at ~2 m are one physical surface cluster.
        scan[10:13] = [2.10, 2.00, 2.08]
        scan[20] = 3.0
        scan[30] = 4.0
        ranges, indices, valid = self._select(scan)
        picked = indices[0, valid[0]].tolist()
        self.assertEqual(sum(index in (10, 11, 12) for index in picked), 1)
        self.assertEqual(valid.sum().item(), 3)
        self.assertTrue(torch.all(ranges[0, 1:] >= ranges[0, :-1]))

    def test_each_nonempty_sector_keeps_its_nearest_cluster(self):
        scan = [self.max_range] * len(self.bearings)
        # One representative in every 60-degree sector. Index 7 is a second, farther cluster in
        # the first sector and must not displace coverage of the far fourth-sector obstacle.
        for index, distance in ((2, 2.0), (7, 2.5), (16, 3.0), (28, 4.0), (43, 8.0)):
            scan[index] = distance
        _, indices, valid = self._select(scan)
        picked = set(indices[0, valid[0]].tolist())
        self.assertEqual(picked, {2, 16, 28, 43})

    def test_empty_sectors_are_filled_by_remaining_clusters(self):
        scan = [self.max_range] * len(self.bearings)
        # All returns lie in one sector but are separated by no-return beams, hence three clusters.
        for index, distance in ((3, 2.0), (6, 3.0), (9, 4.0)):
            scan[index] = distance
        ranges, indices, valid = self._select(scan)
        self.assertEqual(valid.sum().item(), 3)
        self.assertEqual(set(indices[0, valid[0]].tolist()), {3, 6, 9})
        self.assertEqual(ranges[0, valid[0]].tolist(), [2.0, 3.0, 4.0])


if __name__ == "__main__":
    unittest.main()
