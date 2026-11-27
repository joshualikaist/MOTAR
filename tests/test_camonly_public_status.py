"""Camera-only line on the public surfaces: every number and verdict is bound to its committed record.

docs/status/data/camonly-status.json is rebuilt from the evidence records by
tools/build_camonly_public_status.py; README.md, docs/status/index.html, docs/status/evidence.html and
docs/EVIDENCE.md must quote its display strings and verdict labels. CPU only, no simulator.
"""
import copy
import json
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import build_camonly_public_status as build  # noqa: E402
import check_public_docs as docs  # noqa: E402

STATUS = ROOT / "docs/status/data/camonly-status.json"
SURFACES = ("README.md", "docs/status/index.html", "docs/status/evidence.html", "docs/EVIDENCE.md")
# Display keys every surface must carry (the core reading of CAM-S3-E and the outdoor limits).
CORE = ("w_crash", "w_capture", "s0_crash", "w_crash_wilson", "w_minus_s0_crash", "w_minus_s0_crash_ci",
        "c2_total", "outdoor_ground_1p5", "outdoor_sky_phantom", "w_minus_s0_crash_seedblock_ci",
        "outdoor_ground_1p5_sector", "outdoor_sky_phantom_sector", "outdoor_frame_bins")
LABELS = ("q1_label", "pointer", "results_review")


def norm(text):
    return " ".join(text.replace("&nbsp;", " ").split())


class CamonlyStatusTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads(STATUS.read_text(encoding="utf-8"))
        cls.pages = {name: norm((ROOT / name).read_text(encoding="utf-8")) for name in SURFACES}

    def test_status_file_is_rebuilt_from_records(self):
        self.assertEqual(build.build(), self.data)
        for path in self.data["sources"].values():
            self.assertTrue((ROOT / path).is_file(), path)

    def test_recorded_verdicts(self):
        labels = self.data["labels"]
        self.assertEqual(labels["q1_label"], "BAR_NOT_MET_POINT")
        self.assertEqual(labels["q1_wilson"], "ABOVE_20")
        self.assertEqual(labels["pointer"], "STRUCTURAL_NEXT")
        self.assertEqual(labels["w_minus_s0_status"], "NO_IMPROVEMENT")
        self.assertEqual(labels["results_review"], "PASS_WITH_LIMITS")
        self.assertEqual(labels["runtime_review"], "GO")
        self.assertEqual(labels["c2_reading"], "REPORTED")

    def test_surfaces_quote_the_record(self):
        display = self.data["display"]
        for name, text in self.pages.items():
            for key in CORE:
                self.assertIn(display[key], text, (name, key))
            for key in LABELS:
                self.assertIn(self.data["labels"][key], text, (name, key))
            self.assertRegex(text, r"shadow test", name)
            # Review r1 P3-4/5/6/12: the rule, sky mask, interval types, parity scope and earlier FAIL are named.
            for phrase in ("S2b native column rule", "S3-W sector rule", "heuristic sky mask", "episode bootstrap",
                           "seed-block", "7 analytic CPU scenes", "review r3 FAIL"):
                self.assertIn(phrase, text, (name, phrase))

    def test_frame_bins_is_the_frame_bin_count(self):
        # Review r1 P3-3: 239 frames x 13 bins, distinct from the T5 near count.
        v = self.data["values"]
        self.assertEqual(v["outdoor_frame_bins"], v["outdoor_frames"] * 13)
        record = json.loads((ROOT / build.SOURCES["outdoor"]).read_text())
        near = record["q1_sky"]["G1.5"]["overall"]["native"]["T5"]["near_count"]
        self.assertNotEqual(v["outdoor_frame_bins"], near)

    def test_altered_record_is_detected(self):
        # Negative control: a changed record value no longer matches the committed status, and a shifted
        # display string is absent from every surface.
        record = json.loads((ROOT / build.SOURCES["cams3e"]).read_text())
        tampered = copy.deepcopy(record)
        tampered["cells"]["W"]["crash"]["pooled_pct"] += 1.0
        shifted = build.pct(tampered["cells"]["W"]["crash"]["pooled_pct"])
        self.assertNotEqual(shifted, self.data["display"]["w_crash"])
        for text in self.pages.values():
            self.assertNotIn("crash " + shifted, text)

    def test_readme_numbers_bind_to_headline_values(self):
        values = docs.headline_values()
        for key in ("w_crash_pct", "s0_crash_pct", "c2_total_crash_pp", "outdoor_sky_phantom_pct"):
            self.assertTrue(any(abs(v - self.data["values"][key]) < 1e-9 for v in values), key)
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertEqual(docs.landing_number_errors(readme, values), [])
        self.assertTrue(docs.landing_number_errors(readme.replace("25.4 %", "21.4 %"), values))

    def test_no_flight_or_upgrade_wording(self):
        for name, text in self.pages.items():
            low = text.lower()
            for phrase in ("flew autonomously", "flight-validated", "flight validated", "state of the art",
                           "state-of-the-art result", "bar met"):
                self.assertNotIn(phrase, low, (name, phrase))
            self.assertNotRegex(low, r"s3-w (?:meets|passes) the", name)


if __name__ == "__main__":
    unittest.main()
