"""S-L1A provenance for the frozen EQ-1A geometry record (CPU).

EQ-1A asserted that live HEAD still equals its measurement source. The approved opt-in S-L1A
lineage edits ``navrl_detector.py`` and ``navrl_perception.py``, so that live identity is false by
construction once S-L1A is committed. These checks keep the historical claim exact instead: the
frozen EQ-1A record binds to immutable measurement-source blobs, those bytes reached the S-L1A base
unchanged, and only the two approved runtime paths may now differ. The live EQ-1A counterexamples
and the S-L1A OFF tests exercise the default-OFF behaviour; no EQ-1 criterion or oracle moves here.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import eq1_common as common  # noqa: E402
from history_helpers import require_research_history  # noqa: E402

FROZEN = ROOT / "docs/evidence/eq1_validation_2026-10-18/eq1a_geometry.json"
PLAN = json.loads((ROOT / "docs/plans/s_l1a_v1.json").read_text())
MEASURED = {
    "aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py",
    "aerial_gym/task/navrl_task/navrl_detector.py",
    "aerial_gym/task/navrl_task/navrl_perception.py",
    "motar_eval/occlusion_geometry.py",
}
S_L1A_RUNTIME = {
    "aerial_gym/task/navrl_task/navrl_detector.py",
    "aerial_gym/task/navrl_task/navrl_perception.py",
}


def frozen_source():
    return json.loads(FROZEN.read_text())["source"]


class EQ1AMeasurementProvenanceTest(unittest.TestCase):
    def setUp(self):
        require_research_history(ROOT, common.MEASUREMENT_SOURCE, PLAN["base_sha"])

    def test_frozen_record_binds_immutable_measurement_blobs(self):
        src = frozen_source()
        self.assertEqual(src["measurement_source"], common.MEASUREMENT_SOURCE)
        self.assertTrue(src["identical_to_measurement_source"], src)
        self.assertEqual(set(src["files_sha256"]), MEASURED)
        for relative, expected_sha256 in src["files_sha256"].items():
            with self.subTest(path=relative):
                blob = subprocess.check_output(
                    ["git", "-C", str(ROOT), "show", src["measurement_source"] + ":" + relative])
                self.assertEqual(hashlib.sha256(blob).hexdigest(), expected_sha256)

    def test_measured_source_reached_s_l1a_base_unchanged(self):
        rc = subprocess.run(["git", "-C", str(ROOT), "diff", "--quiet", common.MEASUREMENT_SOURCE,
                             PLAN["base_sha"], "--", *sorted(MEASURED)]).returncode
        self.assertEqual(rc, 0)

    def test_only_approved_s_l1a_runtime_paths_may_differ_live(self):
        recorded = frozen_source()["files_sha256"]
        for relative in sorted(MEASURED - S_L1A_RUNTIME):
            with self.subTest(path=relative):
                self.assertEqual(hashlib.sha256((ROOT / relative).read_bytes()).hexdigest(), recorded[relative])


if __name__ == "__main__":
    unittest.main()
