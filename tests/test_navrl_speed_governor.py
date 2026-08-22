import math
import importlib.util
from pathlib import Path
import unittest

import torch

_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "aerial_gym/task/navrl_task/speed_governor.py"
)
_SPEC = importlib.util.spec_from_file_location("navrl_speed_governor_for_test", _MODULE_PATH)
_GOVERNOR = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_GOVERNOR)
SpeedGovernorConfig = _GOVERNOR.SpeedGovernorConfig
apply_speed_governor = _GOVERNOR.apply_speed_governor
directional_lidar_clearance = _GOVERNOR.directional_lidar_clearance


class SpeedGovernorTests(unittest.TestCase):
    def test_off_is_byte_equal(self):
        command = torch.tensor([[2.5, 2.5], [0.0, 0.0]])
        clearance = torch.tensor([0.5, 0.2])
        governed, diag = apply_speed_governor(
            command, clearance, SpeedGovernorConfig(mode="off")
        )
        self.assertTrue(torch.equal(governed, command))
        self.assertTrue(torch.equal(diag["scale"], torch.ones(2)))

    def test_fixed_cap_preserves_direction_and_norm(self):
        command = torch.tensor([[3.0, 4.0]])
        governed, _ = apply_speed_governor(
            command,
            torch.tensor([12.0]),
            SpeedGovernorConfig(mode="fixed", fixed_cap_mps=2.0),
        )
        self.assertTrue(torch.allclose(governed, torch.tensor([[1.2, 1.6]])))
        self.assertAlmostEqual(float(governed.norm()), 2.0, places=6)

    def test_clearance_mode_slows_only_inside_slow_zone(self):
        config = SpeedGovernorConfig(
            mode="clearance", hard_margin_m=0.5, slow_distance_m=3.0,
            free_speed_cap_mps=2.5,
        )
        command = torch.tensor([[2.5, 0.0], [2.5, 0.0], [1.0, 0.0]])
        governed, diag = apply_speed_governor(
            command, torch.tensor([0.5, 1.5, 4.0]), config
        )
        self.assertAlmostEqual(float(governed[0].norm()), 0.0, places=6)
        self.assertAlmostEqual(float(governed[1].norm()), 1.0, places=6)
        self.assertAlmostEqual(float(governed[2].norm()), 1.0, places=6)
        self.assertLess(float(diag["stopping_margin_requested_m"][1]), 0.0)

    def test_ttc_mode_enforces_requested_time_headway(self):
        config = SpeedGovernorConfig(mode="ttc", hard_margin_m=0.5, ttc_s=1.0)
        command = torch.tensor([[3.0, 0.0]])
        governed, diag = apply_speed_governor(command, torch.tensor([1.5]), config)
        self.assertAlmostEqual(float(governed.norm()), 1.0, places=6)
        self.assertAlmostEqual(float(diag["scale"]), 1.0 / 3.0, places=6)

    def test_riskcap_never_forces_a_stop_and_releases_in_open_space(self):
        config = SpeedGovernorConfig(
            mode="riskcap",
            fixed_cap_mps=2.0,
            free_speed_cap_mps=3.5,
            slow_distance_m=3.0,
            release_distance_m=5.0,
        )
        command = torch.tensor([[3.5, 0.0], [3.5, 0.0], [3.5, 0.0], [1.0, 0.0]])
        governed, diag = apply_speed_governor(
            command, torch.tensor([0.45, 3.0, 4.0, 0.45]), config
        )
        self.assertTrue(
            torch.allclose(
                governed.norm(dim=1), torch.tensor([2.0, 2.0, 2.75, 1.0]), atol=1e-6
            )
        )
        self.assertTrue(torch.all(diag["executed_speed_mps"] > 0.0))

    def test_directional_clearance_uses_command_frame(self):
        bearings = torch.tensor([math.pi / 2.0, 0.0, -math.pi / 2.0])
        scan = torch.full((2, 1, 3), 12.0)
        scan[0, 0, 1] = 2.0  # forward obstacle
        scan[1, 0, 0] = 3.0  # left obstacle
        command = torch.tensor([[1.0, 0.0], [0.0, 1.0]])
        clearance = directional_lidar_clearance(
            scan,
            bearings,
            command,
            max_range_m=12.0,
            path_half_width_m=0.45,
            vertical_fov_deg=(0.0, 0.0),
        )
        self.assertTrue(torch.allclose(clearance, torch.tensor([2.0, 3.0]), atol=1e-5))

    def test_target_return_is_not_a_collision_obstacle(self):
        bearings = torch.tensor([0.0])
        scan = torch.tensor([[[1.0]]])
        target = torch.ones_like(scan, dtype=torch.bool)
        clearance = directional_lidar_clearance(
            scan,
            bearings,
            torch.tensor([[1.0, 0.0]]),
            max_range_m=12.0,
            path_half_width_m=0.45,
            target_return_mask=target,
            vertical_fov_deg=(0.0, 0.0),
        )
        self.assertAlmostEqual(float(clearance), 12.0, places=6)

    def test_vertical_slant_range_is_projected(self):
        bearings = torch.tensor([0.0])
        scan = torch.tensor([[[12.0], [2.0]]])
        clearance = directional_lidar_clearance(
            scan,
            bearings,
            torch.tensor([[1.0, 0.0]]),
            max_range_m=12.0,
            path_half_width_m=0.45,
            vertical_fov_deg=(0.0, 60.0),
        )
        self.assertAlmostEqual(float(clearance), 1.0, places=5)

    def test_default_vertical_angles_follow_warp_tensor_row_order(self):
        bearings = torch.tensor([0.0])
        scan = torch.tensor([[[2.0], [12.0], [12.0], [12.0]]])
        clearance = directional_lidar_clearance(
            scan,
            bearings,
            torch.tensor([[1.0, 0.0]]),
            max_range_m=12.0,
            path_half_width_m=0.45,
        )
        self.assertAlmostEqual(
            float(clearance), 2.0 * math.cos(math.radians(20.0)), places=5
        )

    def test_environment_contract_fails_closed(self):
        with self.assertRaises(ValueError):
            SpeedGovernorConfig.from_environ({"NAVRL_SPEED_GOVERNOR": "magic"})
        with self.assertRaises(ValueError):
            SpeedGovernorConfig.from_environ(
                {
                    "NAVRL_SPEED_GOVERNOR": "clearance",
                    "NAVRL_SPEED_GOVERNOR_MARGIN_M": "3",
                    "NAVRL_SPEED_GOVERNOR_SLOW_M": "2",
                }
            )
        with self.assertRaises(ValueError):
            SpeedGovernorConfig.from_environ(
                {
                    "NAVRL_SPEED_GOVERNOR": "riskcap",
                    "NAVRL_SPEED_GOVERNOR_SLOW_M": "3",
                    "NAVRL_SPEED_GOVERNOR_RELEASE_M": "3",
                }
            )


if __name__ == "__main__":
    unittest.main()
