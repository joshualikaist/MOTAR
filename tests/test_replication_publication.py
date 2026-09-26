"""Publication status and file counts must survive removal of workstation-only raw evidence."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import build_result_manifest as manifest
import build_tm_publication as publication


class PublicationTest(unittest.TestCase):
    def test_status_is_derived_from_frozen_analysis(self):
        result = publication.build()
        self.assertEqual(json.loads((publication.RESULT / "summary.json").read_text()), result)
        self.assertEqual(result["status"], "REPLICATED")
        self.assertEqual(result["scope"], "INDEPENDENT_BIAS_CORRECTED_REPLICATION")
        self.assertEqual((result["valid_cells"], result["primary_episodes"], result["fresh_seeds"]),
                         (112, 229376, 7))
        self.assertFalse(result["ppo_training_started"])
        self.assertFalse(result["pooled_with_historical"])

    def test_manifest_ignores_local_raw_files_and_caches(self):
        relative = "results/target_motion_replication_7seed"
        files = subprocess.check_output(["git", "ls-files", "--", relative], cwd=ROOT,
                                        text=True).splitlines()
        with tempfile.TemporaryDirectory() as tmp:
            clean = Path(tmp)
            for name in files:
                destination = clean / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / name, destination)
            directory = clean / relative
            def fake_git(*args, **kwargs):
                if args[0] == "ls-files":
                    return "\n".join(files)
                if args[0] == "rev-parse":
                    return ".git"
                return "2026-09-26T00:00:00Z"
            contract = manifest.external_contract()
            with patch.object(manifest, "ROOT", clean), patch.object(manifest, "git", fake_git), \
                    patch.object(manifest, "external_contract", return_value=contract):
                before = manifest.describe(directory, tracked={directory.name})
                (directory / "ignored_raw.jsonl").write_text("private raw evidence\n")
                (directory / "__pycache__").mkdir()
                (directory / "__pycache__/ignored.pyc").write_bytes(b"cache")
                after = manifest.describe(directory, tracked={directory.name})
            self.assertEqual(before, after)
            self.assertEqual(after["files"], len(files))
            self.assertEqual(after["status"], "REPLICATED")
            self.assertEqual(after["external_raw_evidence"]["raw_files"], 562)
            self.assertEqual(after["external_raw_evidence"]["external_raw_file_count"], 561)
            self.assertEqual(after["external_raw_evidence"]["tracked_raw_paths"], ["preflight_main.json"])
            self.assertEqual(after["external_raw_evidence"]["archives"][0]["members"], 563)


if __name__ == "__main__":
    unittest.main()
