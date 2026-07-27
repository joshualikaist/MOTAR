import importlib.util
from pathlib import Path
import tempfile
import unittest

import torch


_ROOT = Path(__file__).parents[1]
_MODULE_PATH = (
    _ROOT
    / "aerial_gym/rl_training/rl_games/navrl_checkpoint_preflight.py"
)
_SPEC = importlib.util.spec_from_file_location("navrl_checkpoint_preflight_standalone", _MODULE_PATH)
_PREFLIGHT = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_PREFLIGHT)


def _checkpoint():
    return {
        "epoch": 15000,
        "model": {"weight": torch.tensor([1.0])},
        "optimizer": {"state": {0: {"exp_avg": torch.tensor([0.1])}}},
        "assymetric_vf_nets": {"weight": torch.tensor([2.0])},
        "assymetric_vf_optimizer": {"state": {0: {"exp_avg": torch.tensor([0.2])}}},
        "env_state": {
            "num_task_steps": 480000,
            "n_bars_active": 65,
            "cfg_lidar_max_range": 12.0,
            "cfg_max_obstacles": 8,
            "cfg_token_fov_deg": 240.0,
            "cfg_obstacle_suppress_deg": 10.0,
            "cfg_lidar_hbeams": 72,
            "cfg_lidar_vbeams": 4,
        },
    }


class NavRLCheckpointPreflightTest(unittest.TestCase):
    def _save(self, checkpoint):
        tmp = tempfile.NamedTemporaryFile(suffix=".pth", delete=False)
        tmp.close()
        self.addCleanup(Path(tmp.name).unlink, missing_ok=True)
        torch.save(checkpoint, tmp.name)
        return tmp.name

    def test_valid_density_resume(self):
        path = self._save(_checkpoint())
        info = _PREFLIGHT.inspect_checkpoint(
            path,
            max_epochs=45000,
            density_final=110,
            expected_contract={"cfg_token_fov_deg": 240.0, "cfg_max_obstacles": 8},
        )
        self.assertEqual(info["epoch"], 15000)
        self.assertEqual(info["bars"], 65)

    def test_max_epochs_must_extend_checkpoint(self):
        path = self._save(_checkpoint())
        with self.assertRaisesRegex(_PREFLIGHT.CheckpointPreflightError, "must exceed"):
            _PREFLIGHT.inspect_checkpoint(path, max_epochs=15000, density_final=110)

    def test_policy_contract_mismatch_is_rejected(self):
        path = self._save(_checkpoint())
        with self.assertRaisesRegex(_PREFLIGHT.CheckpointPreflightError, "mismatch"):
            _PREFLIGHT.inspect_checkpoint(
                path,
                max_epochs=45000,
                density_final=110,
                expected_contract={"cfg_token_fov_deg": 360.0},
            )

    def test_nonfinite_checkpoint_is_rejected(self):
        checkpoint = _checkpoint()
        checkpoint["optimizer"]["state"][0]["exp_avg"] = torch.tensor([float("nan")])
        path = self._save(checkpoint)
        with self.assertRaisesRegex(_PREFLIGHT.CheckpointPreflightError, "non-finite"):
            _PREFLIGHT.inspect_checkpoint(path, max_epochs=45000, density_final=110)


if __name__ == "__main__":
    unittest.main()
