"""CPU-only independent geometry and audit CLI contracts."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import torch


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("_mt_cpu_verify", ROOT / "tools/kroc2027_motion_verify.py")
VERIFY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFY)


class MotionVerifyTest(unittest.TestCase):
    def test_continuous_sweep_catches_corner_cut_with_safe_endpoints(self):
        # Endpoints lie outside the rectangle; the connecting segment crosses it.
        self.assertGreater(VERIFY.point_rect_distance([-2.0, 0.0], [0.0, 0.0], [.5, .5]), 0)
        self.assertGreater(VERIFY.point_rect_distance([2.0, 0.0], [0.0, 0.0], [.5, .5]), 0)
        self.assertEqual(VERIFY.swept_rect_distance([-2.0, 0.0], [2.0, 0.0], [0.0, 0.0], [.5, .5]), 0)
        self.assertAlmostEqual(VERIFY.swept_rect_distance([-2, 2], [2, 2], [0, 0], [.5, .5]), 1.5)

    def test_float64_subtraction_oracle_near_40_m(self):
        a = torch.tensor([[[39.9, 39.9], [40.0, 40.0]]])
        b = torch.tensor([[[39.9001, 39.9001], [39.0, 39.0]]])
        result = VERIFY.reference_distances(a, b)
        self.assertGreater(float(result[0, 0, 0]), 0)
        self.assertAlmostEqual(float(result[0, 1, 1]), 2.0 ** .5, places=6)

    def test_cuda_option_is_refused_without_loading_motion(self):
        with mock.patch.object(VERIFY, "load_motion") as loader:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                VERIFY.main(["--device", "cuda"])
            loader.assert_not_called()

    def test_existing_output_and_results_path_are_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "untouched.json"
            path.write_text("original")
            for out in [path, ROOT / "results/new-unapproved-report.json"]:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    VERIFY.main(["--out", str(out)])
            self.assertEqual(path.read_text(), "original")

    def test_cpu_synthetic_cli_never_queries_cuda_or_consumes_rng(self):
        output = io.StringIO()
        before = torch.random.get_rng_state().clone()
        with mock.patch.object(torch.cuda, "is_available", side_effect=AssertionError("CUDA query")):
            with contextlib.redirect_stdout(output):
                result = VERIFY.main(["--device", "cpu", "--suite", "synthetic", "--steps", "2"])
        report = json.loads(output.getvalue())
        self.assertEqual(result, 0)
        self.assertEqual(report["synthetic"]["status"], "CPU_SYNTHETIC_PASS")
        self.assertFalse(report["gpu_started"])
        self.assertTrue(torch.equal(before, torch.random.get_rng_state()))

    def test_oracle_patch_restored_if_executor_raises(self):
        original = torch.cdist
        motion = mock.Mock()
        motion.bounded_drone_target_step.side_effect = RuntimeError("fixture")
        with self.assertRaises(RuntimeError):
            VERIFY.execute(motion, torch.zeros(1, 2), torch.zeros(1, 2), torch.zeros(1, 2),
                           torch.ones(1), torch.empty(1, 0, 2), torch.ones(1),
                           "legacy_cdist", oracle=True)
        self.assertIs(torch.cdist, original)


if __name__ == "__main__":
    unittest.main()
