"""No simulator/torch/checkpoint loading: policy provenance and fail-closed metadata."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("camera_policy_contract_tested", ROOT / "tools/camera_policy_contract.py")
C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(C)


class CameraPolicyContractTest(unittest.TestCase):
    def contract(self, seed=1):
        value = C._metadata(seed, ROOT)
        value["sources"] = {rel: "a" * 64 for rel in C.SOURCE_PATHS}
        return value

    def test_both_pinned_seeds_accept_and_remain_distinct(self):
        values = [self.contract(seed) for seed in (1, 2)]
        for value in values:
            self.assertEqual(C.validate_contract(value, verify_files=False), [])
            self.assertEqual(value["observation"]["dim"], 898)
            self.assertEqual(value["runtime_env"]["NAVRL_SENSOR_ABLATION"], "no_lidar")
            self.assertEqual(value["qualification"]["hardware"], "NOT_TESTED")
        self.assertNotEqual(values[0]["checkpoint"]["sha256"], values[1]["checkpoint"]["sha256"])

    def test_missing_environment_extra_inherited_override_and_wrong_seed_rejected(self):
        valid = self.contract()
        mutations = []
        bad = copy.deepcopy(valid)
        del bad["runtime_env"]["NAVRL_MAX_VELOCITY"]
        mutations.append(bad)
        bad = copy.deepcopy(valid)
        bad["runtime_env"]["NAVRL_SENSOR_LINEAGE"] = "s_l1c_v2"
        mutations.append(bad)
        bad = copy.deepcopy(valid)
        bad["seed"] = 2
        mutations.append(bad)
        bad = copy.deepcopy(valid)
        bad["observation"]["no_lidar"] = False
        mutations.append(bad)
        bad = copy.deepcopy(valid)
        bad["camera"]["vfov_deg"] = 40.0
        mutations.append(bad)
        bad = copy.deepcopy(valid)
        bad["qualification"]["hardware"] = "PASS"
        mutations.append(bad)
        self.assertEqual(C.validate_contract(valid, verify_files=False), [])
        for bad in mutations:
            self.assertTrue(C.validate_contract(bad, verify_files=False))

    def test_runtime_source_bytes_and_checkpoint_bytes_are_checked(self):
        contract = self.contract()
        ck = contract["checkpoint"]
        manifest = {"verdict": "TRAINED", "unit": "CAM-K2 v3", "seed": 1,
                    "robot": "navrl_quad", "checkpoint_sha256": ck["sha256"]}
        original_sha = C.sha256_file

        def digest(path):
            path = str(path)
            if path == ck["path"]:
                return ck["sha256"]
            if path == ck["manifest_path"]:
                return ck["manifest_sha256"]
            if path.endswith("train_report_v3_s1.json"):
                return original_sha(path)
            return "a" * 64

        original_read = Path.read_text

        def read(path, *a, **kw):
            return json.dumps(manifest) if str(path) == ck["manifest_path"] else original_read(path, *a, **kw)

        with mock.patch.object(C, "sha256_file", side_effect=digest), mock.patch.object(Path, "read_text", read):
            self.assertEqual(C.validate_contract(contract), [])
            changed = copy.deepcopy(contract)
            changed["sources"][C.SOURCE_PATHS[0]] = "b" * 64
            self.assertIn("runtime source changed: " + C.SOURCE_PATHS[0], C.validate_contract(changed))
            with mock.patch.object(C, "sha256_file", side_effect=lambda p: "b" * 64 if str(p) == ck["path"] else digest(p)):
                self.assertTrue(any("checkpoint SHA256 differs" in x for x in C.validate_contract(contract)))

    def test_malformed_contracts_do_not_accept(self):
        for seed in (True, None, "1", 0, 3):
            value = self.contract()
            value["seed"] = seed
            self.assertTrue(C.validate_contract(value, verify_files=False))
        self.assertTrue(C.validate_contract([]))
        value = self.contract()
        value["sources"] = {"../../external": "a" * 64}
        self.assertTrue(C.validate_contract(value, verify_files=False))
        value = self.contract()
        value["observation"]["no_lidar"] = 1
        self.assertTrue(C.validate_contract(value, verify_files=False))

    def test_load_returns_validated_contract_and_rejects_missing_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            value = self.contract()
            path.write_text(json.dumps(value))
            self.assertEqual(C.load_contract(path, verify_files=False), value)
            del value["sources"][C.SOURCE_PATHS[0]]
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, "runtime source set"):
                C.load_contract(path, verify_files=False)

    def test_cli_requires_explicit_seed_and_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text("preserve")
            with self.assertRaises(SystemExit):
                C.main(["--output", str(path)])
            with mock.patch.object(C, "build_contract", return_value=self.contract()):
                with self.assertRaises(SystemExit):
                    C.main(["--output", str(path), "--seed", "1"])
            self.assertEqual(path.read_text(), "preserve")


if __name__ == "__main__":
    unittest.main()
