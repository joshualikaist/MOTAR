import importlib.util
from pathlib import Path
import unittest


_ROOT = Path(__file__).parents[1]
_MODULE_PATH = (
    _ROOT / "aerial_gym/rl_training/rl_games/reward_stable_early_stop.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "reward_stable_early_stop_standalone", _MODULE_PATH
)
_GUARD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_GUARD)

_CURRICULUM_PATH = (
    _ROOT / "aerial_gym/task/navrl_task/navrl_curriculum.py"
)
_CURRICULUM_SPEC = importlib.util.spec_from_file_location(
    "navrl_curriculum_standalone", _CURRICULUM_PATH
)
_CURRICULUM = importlib.util.module_from_spec(_CURRICULUM_SPEC)
_CURRICULUM_SPEC.loader.exec_module(_CURRICULUM)


class DensityCaptureGuardTest(unittest.TestCase):
    def setUp(self):
        self.cfg = {
            "window_epochs": 4,
            "min_epochs_at_density": 4,
            "min_peak_capture": 0.5,
            "drop_absolute": 0.25,
            "patience_epochs": 2,
        }

    def run_rates(self, rates, bars=85, state=None):
        stop = False
        for epoch, rate in enumerate(rates, 1):
            stop, state = _GUARD.density_capture_collapse_should_stop(
                self.cfg, epoch, rate, bars, state
            )
        return stop, state

    def test_same_density_catastrophic_drop_stops(self):
        stop, state = self.run_rates([0.70] * 6 + [0.05] * 6)
        self.assertTrue(stop)
        self.assertEqual(state["bars"], 85)
        self.assertGreaterEqual(state["collapse_peak"], 0.69)
        self.assertLess(state["collapse_capture"], 0.45)

    def test_density_promotion_resets_reference(self):
        stop, state = self.run_rates([0.70] * 8, bars=85)
        self.assertFalse(stop)
        stop, state = self.run_rates([0.42] * 8, bars=90, state=state)
        self.assertFalse(stop)
        self.assertEqual(state["bars"], 90)

    def test_ordinary_noise_does_not_stop(self):
        stop, _ = self.run_rates(
            [0.68, 0.64, 0.71, 0.62, 0.66, 0.69, 0.61, 0.67, 0.65, 0.70]
        )
        self.assertFalse(stop)

    def test_nonfinite_capture_stops_immediately(self):
        stop, state = self.run_rates([float("nan")])
        self.assertTrue(stop)
        self.assertTrue(state["nan_stop"])


class DensityDwellTest(unittest.TestCase):
    def test_threshold_ramp_endpoints_and_midpoint(self):
        threshold = _CURRICULUM.density_threshold_at
        self.assertAlmostEqual(threshold(70, 70, 300, 0.85, 0.70), 0.85)
        self.assertAlmostEqual(threshold(185, 70, 300, 0.85, 0.70), 0.775)
        self.assertAlmostEqual(threshold(300, 70, 300, 0.85, 0.70), 0.70)

    def test_promotion_is_blocked_at_999_and_allowed_at_1000_epochs(self):
        horizon = 32
        start = 6400
        self.assertFalse(
            _CURRICULUM.density_dwell_ready(
                start + 999 * horizon,
                start,
                horizon,
                1000,
            )
        )
        self.assertTrue(
            _CURRICULUM.density_dwell_ready(
                start + 1000 * horizon,
                start,
                horizon,
                1000,
            )
        )

    def test_promotion_resets_the_level_clock(self):
        previous = 32000
        promoted_at = 64000
        reset = _CURRICULUM.density_level_start_after_promotion(
            previous,
            promoted_at,
            promoted=True,
        )
        self.assertEqual(reset, promoted_at)
        self.assertFalse(
            _CURRICULUM.density_dwell_ready(
                promoted_at + 999 * 32,
                reset,
                32,
                1000,
            )
        )

    def test_checkpoint_restore_keeps_clock_and_legacy_is_conservative(self):
        restore = _CURRICULUM.restore_density_level_start_steps
        self.assertEqual(
            restore({"density_level_start_steps": 12345}, 50000),
            12345,
        )
        self.assertEqual(restore({}, 50000), 50000)
        self.assertEqual(
            restore({"density_level_start_steps": 99999}, 50000),
            50000,
        )


if __name__ == "__main__":
    unittest.main()
