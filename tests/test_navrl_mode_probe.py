"""CPU-only tests for the frozen-policy symmetric-corridor diagnostic."""

import json
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest

import torch


os.environ.update(
    {
        "NAVRL_LIDAR_HBEAMS": "72",
        "NAVRL_LIDAR_VBEAMS": "4",
        "NAVRL_MAX_OBSTACLES": "8",
    }
)

ROOT = Path(__file__).resolve().parents[1]
# Load the two pure-torch files without importing aerial_gym.__init__, which eagerly imports Isaac
# Gym.  This keeps the test genuinely CPU-only and follows test_ppo_update_safety.py's convention.
for name in ("aerial_gym", "aerial_gym.rl_training", "aerial_gym.rl_training.rl_games"):
    sys.modules.setdefault(name, types.ModuleType(name))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


load(
    "aerial_gym.rl_training.rl_games.ppo_update_safety",
    ROOT / "aerial_gym/rl_training/rl_games/ppo_update_safety.py",
)
PROBE = load(
    "aerial_gym.rl_training.rl_games.navrl_mode_probe",
    ROOT / "aerial_gym/rl_training/rl_games/navrl_mode_probe.py",
)
ModeProbeRecorder = PROBE.ModeProbeRecorder
build_probe_observations = PROBE.build_probe_observations


class ModeProbeTest(unittest.TestCase):
    def test_fixture_is_exact_mirror_pair_and_symmetric_midpoint(self):
        reference = torch.zeros(3, 898)
        fixtures, contract = build_probe_observations(reference, offset_deg=5.0)
        self.assertEqual(set(fixtures), {"symmetric", "left", "right"})
        self.assertEqual(tuple(fixtures["left"].shape), (1, 898))
        self.assertEqual(contract["symmetric_reflection_max_abs"], 0.0)
        self.assertEqual(contract["left_right_reflection_max_abs"], 0.0)
        self.assertGreater(float((fixtures["left"] - fixtures["right"]).abs().sum()), 0.0)

    def _outputs(self, symmetric, left, right, sigma=0.4):
        payload = {}
        for arm, action in (("symmetric", symmetric), ("left", left), ("right", right)):
            tensor = torch.tensor([action], dtype=torch.float32)
            payload[arm] = {
                "action": tensor,
                "mu": tensor.clone(),
                "sigma": torch.full_like(tensor, sigma),
            }
        return payload

    def test_preregistered_positive_pattern_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = ModeProbeRecorder(Path(directory) / "probe.json", max_velocity_mps=2.5)
            recorder.record(
                self._outputs(
                    [0.02, 0.01, 0.0, 0.0],
                    [0.40, 0.40, 0.0, 0.20],
                    [0.40, -0.40, 0.0, -0.20],
                ),
                {"symmetric_reflection_max_abs": 0.0, "left_right_reflection_max_abs": 0.0},
            )
            payload = recorder.write()
            self.assertEqual(
                payload["verdict"], "MODE_AVERAGING_SUPPORTED_FOR_COUNTERFACTUAL_FIXTURE"
            )
            self.assertTrue(all(payload["checks"].values()))
            self.assertEqual(json.loads((Path(directory) / "probe.json").read_text())["samples"], 1)

    def test_policy_chirality_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = ModeProbeRecorder(Path(directory) / "probe.json", max_velocity_mps=2.5)
            recorder.record(
                self._outputs(
                    [0.02, 0.01, 0.0, 0.0],
                    [0.40, 0.40, 0.0, 0.20],
                    [0.40, 0.35, 0.0, 0.20],
                ),
                {"symmetric_reflection_max_abs": 0.0, "left_right_reflection_max_abs": 0.0},
            )
            payload = recorder.payload()
            self.assertEqual(payload["verdict"], "INCONCLUSIVE_POLICY_CHIRALITY")
            self.assertFalse(payload["checks"]["policy_reflection_quality"])

    def test_schema_drift_and_overwrite_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "base structured schema"):
            build_probe_observations(torch.zeros(1, 946))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "probe.json"
            path.write_text("occupied", encoding="utf-8")
            recorder = ModeProbeRecorder(path, max_velocity_mps=2.5)
            recorder.record(
                self._outputs([0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]),
                {"symmetric_reflection_max_abs": 0.0, "left_right_reflection_max_abs": 0.0},
            )
            with self.assertRaisesRegex(RuntimeError, "refusing to overwrite"):
                recorder.write()


if __name__ == "__main__":
    unittest.main()
