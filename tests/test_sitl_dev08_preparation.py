"""Static dev08 preparation: preserve native configuration and fail closed on drift.

No simulator, checkpoint, bridge or historical runner is imported. Run these in
the shared PUBLIC_CPU suite too; the tool must not change its environment.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("dev08_preparation", ROOT / "tools/sitl_s3b_dev08.py")
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)


class Dev08PreparationTest(unittest.TestCase):
    def test_default_native_matches_both_historical_reports_without_mutation(self):
        before_env, before_path = dict(os.environ), list(sys.path)
        native = D.prepare()
        candidate = D.prepare(arm="candidate")
        self.assertTrue(native["native_params"])
        self.assertEqual(native["selected_arm"], "native")
        self.assertEqual(native["selected_params"], native["native_params"])
        self.assertEqual(candidate["selected_params"], candidate["candidate_params"])
        self.assertEqual(native["native_params"], candidate["native_params"])
        self.assertEqual(dict(os.environ), before_env)
        self.assertEqual(sys.path, before_path)

    def test_candidate_changes_only_three_gains_and_keeps_timing(self):
        data = D.prepare(arm="candidate")
        a, b = data["native_params"], data["selected_params"]
        delta = {k for k in set(a) | set(b) if a.get(k) != b.get(k)}
        self.assertEqual(delta, {"MPC_XY_VEL_P_ACC", "MC_YAWRATE_P", "MC_YAWRATE_I"})
        self.assertEqual([b[k] for k in ("MPC_XY_VEL_P_ACC", "MC_YAWRATE_P", "MC_YAWRATE_I")],
                         [2.0, 0.2, 0.1])
        self.assertEqual(data["unchanged_constants"]["DT"], 0.1)
        self.assertEqual(data["unchanged_constants"]["ODE_ITERS"], 100)
        self.assertEqual(data["gain_delta"]["MC_YAWRATE_P"]["native"], "NOT_EXPLICITLY_OVERRIDDEN")

    def test_selected_native_copy_cannot_modify_the_input(self):
        original = D.prepare()["native_params"]
        saved = copy.deepcopy(original)
        native, candidate = D.variant_params(original), D.variant_params(original, "candidate")
        native["MPC_XY_VEL_P_ACC"] = -1
        candidate["MC_ROLLRATE_P"] = -1
        self.assertEqual(original, saved)
        with self.assertRaises(ValueError):
            D.variant_params(original, "typo")

    def test_intact_input_accepted_and_each_provenance_drift_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for rel in D.PINS:
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes((ROOT / rel).read_bytes())
            self.assertEqual(len(D.prepare(root)["cells"]), 2)  # positive control
            rejected = 0
            for rel in D.PINS:
                path = root / rel
                original = path.read_bytes()
                path.write_bytes(original + b"\n")
                with self.assertRaisesRegex(ValueError, "historical input changed"):
                    D.prepare(root)
                rejected += 1
                path.write_bytes(original)
            self.assertEqual(rejected, 4)
            self.assertEqual(D.prepare(root)["measurement_status"], "NOT_RUN")

    def test_literal_reader_does_not_execute_source_and_rejects_changed_contract(self):
        source = (ROOT / D.RUNNER).read_text()
        self.assertTrue(D.literal_contract(source)["PARAMS"])
        harmless_to_ast = "raise RuntimeError('must not execute')\n" + source
        self.assertEqual(D.literal_contract(harmless_to_ast), D.literal_contract(source))
        for changed in (source.replace("DT = 0.1", "DT = 0.2"),
                        source.replace("ODE_ITERS = 100", "ODE_ITERS = 10"),
                        source + "\nDT = 0.1\n"):
            with self.assertRaises(ValueError):
                D.literal_contract(changed)

    def test_known_short_target_is_reported_without_relabeling_dev07(self):
        data = D.prepare()
        self.assertEqual(data["input_readiness"], "BLOCKED_INCOMPLETE_TARGET_TRACE")
        self.assertEqual(data["target_coverage_shortfalls"],
                         [{"bars": 70, "env": 5, "recorded_frames": 208, "required_frames": 600}])
        self.assertEqual(len(data["cells"]["70"]["envs"]), 10)
        self.assertEqual(data["cells"]["70"]["target_frames_in_historical_report"]["5"], 208)
        self.assertFalse(data["cells"]["70"]["trace_payload_verified"])
        self.assertFalse(data["execution_supported"])
        self.assertEqual(data["runtime_status"], "NOT_IMPLEMENTED")

    def test_cli_has_only_static_native_and_explicit_candidate_modes(self):
        script = str(ROOT / "tools/sitl_s3b_dev08.py")
        for args, arm in (([], "native"), (["--arm", "candidate"], "candidate")):
            proc = subprocess.run([sys.executable, "-B", script] + args,
                                  capture_output=True, text=True, timeout=10)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            data = json.loads(proc.stdout)
            self.assertEqual(data["selected_arm"], arm)
            self.assertFalse(data["execution_supported"])
        for args in (["--execute"], ["--arm", "invalid"]):
            proc = subprocess.run([sys.executable, "-B", script] + args,
                                  capture_output=True, text=True, timeout=10)
            self.assertEqual(proc.returncode, 2)
            self.assertFalse(proc.stdout)


if __name__ == "__main__":
    unittest.main()
