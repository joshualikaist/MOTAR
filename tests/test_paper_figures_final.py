"""The paper figure family (docs/assets/paper/final/) is complete, accessible and bound to its sources.

The value and hash checks need only the standard library, so they run in the CPU CI profile. The
byte-exact rebuild runs only where the recorded matplotlib version is installed.
"""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FINAL = ROOT / "docs/assets/paper/final"
sys.path.insert(0, str(ROOT / "tools"))
import build_paper_figures as figures  # noqa: E402

STEMS = [stem for stem, _ in figures.FIGURES]
SVG = "{http://www.w3.org/2000/svg}"


def svg_text(stem):
    root = ET.parse(FINAL / f"{stem}.svg").getroot()
    return " ".join(t.text or "" for t in root.iter(SVG + "text")) + " " + (root.findtext(SVG + "desc") or "")


class FigureFamilyTest(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((FINAL / "manifest.json").read_text(encoding="utf-8"))

    def test_six_paper_figures_and_the_system_map_in_three_formats(self):
        """SVG is tracked; PNG and PDF are rebuilt on demand and pinned by hash in the manifest."""
        self.assertEqual(STEMS[:6], ["fig1-motar-overview", "fig2-perception-policy", "fig3-safety",
                                     "fig4-target-motion", "fig5-observation-contract", "fig6-positioning"])
        for stem in STEMS:
            svg = FINAL / f"{stem}.svg"
            self.assertTrue(svg.is_file(), svg)
            self.assertEqual(hashlib.sha256(svg.read_bytes()).hexdigest(), self.manifest["files"][svg.name])
            for ext, magic in (("png", b"\x89PNG"), ("pdf", b"%PDF-")):
                name = f"{stem}.{ext}"
                self.assertRegex(self.manifest["files"][name], "^[0-9a-f]{64}$", name)
                path = FINAL / name
                if path.is_file():  # a local rebuild must match the recorded hash
                    self.assertTrue(path.read_bytes().startswith(magic), name)
                    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), self.manifest["files"][name])

    def test_rendered_copies_are_not_tracked(self):
        ignored = (ROOT / ".gitignore").read_text(encoding="utf-8")
        for pattern in ("docs/assets/paper/final/*.png", "docs/assets/paper/final/*.pdf"):
            self.assertIn(pattern, ignored)

    def test_svg_accessibility_and_real_text(self):
        for stem in STEMS:
            root = ET.parse(FINAL / f"{stem}.svg").getroot()
            self.assertEqual(root.get("role"), "img", stem)
            self.assertTrue(root.findtext(SVG + "title"), stem)
            self.assertGreater(len(root.findtext(SVG + "desc") or ""), 80, stem)
            self.assertFalse(list(root.iter(SVG + "image")), "no embedded raster in " + stem)
            self.assertGreater(len(list(root.iter(SVG + "text"))), 10, "text must stay text in " + stem)

    def test_boundaries_are_drawn_into_the_figures(self):
        expectations = {
            "fig1-motar-overview": ("−4.57 pp", "+0.73 pp", "INCONCLUSIVE", "−1.4903 pp", "E0 −5.37",
                                    "E2 +1.35", "replicated · 7 fresh seeds, bias-corrected", "−48.967 pp", "MATERIAL_LOSS", "causality NOT TESTED",
                                    "schematic", "Simulation only", "not a causal chain", "policy R (not F)",
                                    "episode-level 95% CI · 2 evaluation seeds", "3 training seeds"),
            "fig2-perception-policy": ("6.2%", "−4.57 pp", "+0.73 pp", "INCONCLUSIVE", "GOODNESS-OF-FIT PASS",
                                       "unit: 9 blocks of 15 s", "not independent", "episode-level 95% CI",
                                       "training seeds (n = 3)", "frozen policy F"),
            "fig3-safety": ("schematic", "−1.4903 pp", "15/15", "neither is a collision guarantee",
                            "seed-level 95% CI [−2.42, −0.57]", "not replicates", "frozen policy F"),
            "fig4-target-motion": ("schematic", "−5.37 pp", "−3.45 pp", "+1.35 pp", "7 fresh seeds · bias-corrected quota",
                                   "229,376 primary episodes", "unit: evaluation seed", "replication, n = 7",
                                   "original, n = 3", "not pooled", "5/7 seeds at 160 bars, 4/7 at 205 (exploratory)",
                                   "diagnostic only"),
            "fig5-observation-contract": ("−48.967 pp", "MATERIAL_LOSS", "CAUSALITY NOT TESTED",
                                          "ref5in D1 checkpoint", "unit: paired evaluation seed (n = 3)",
                                          "not policy F", "data flow in the experiment, not a causal mechanism"),
            "fig6-positioning": ("not a performance score", "Class A matched external comparisons = 0",
                                 "Real hardware"),
        }
        for stem, phrases in expectations.items():
            text = svg_text(stem)
            for phrase in phrases:
                self.assertIn(phrase, text, f"{stem}: {phrase}")
        for stem in STEMS:
            for claim in ("causes", "outperform", "state-of-the-art performance", "significant"):
                self.assertNotIn(claim, svg_text(stem).lower(), f"{stem}: {claim}")

    def test_paper_figures_are_drawn_at_print_size_with_legible_text(self):
        """Figures 1-6 are 7 in wide, so SVG point sizes are printed sizes: nothing below 7 pt."""
        import re
        for stem in STEMS[:6]:
            text = (FINAL / f"{stem}.svg").read_text(encoding="utf-8")
            self.assertRegex(text, r'<svg[^>]* width="504pt"', stem)
            sizes = [float(x) for x in re.findall(r"font(?:-size)?:[^;\"]*?([\d.]+)px", text)]
            self.assertTrue(sizes, stem)
            self.assertGreaterEqual(min(sizes), 7.0, f"{stem}: text below 7 pt at print size")
            normal = sum(1 for x in sizes if x >= 8.0) / len(sizes)
            self.assertGreaterEqual(normal, 0.5, f"{stem}: most text should print at 8 pt or more")

    def test_recorded_values_equal_the_canonical_sources_now(self):
        self.assertEqual(self.manifest["values"], json.loads(json.dumps(figures.collect_values())))

    def test_sources_have_not_changed_since_the_build(self):
        for path, digest in self.manifest["sources"].items():
            self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), digest,
                             f"{path} changed: rebuild with tools/build_paper_figures.py")

    def test_values_match_the_registry_headlines(self):
        v = self.manifest["values"]
        reg = v["registry"]
        self.assertAlmostEqual(v["target_motion"]["contrasts_vs_H"]["E2"]["mean_pp"],
                               reg["target_motion_generalization"]["value_pp"], places=3)
        self.assertAlmostEqual(v["safety"]["pooled_pp"], reg["safety_filter_geometry"]["value_pp"], places=4)
        self.assertAlmostEqual(v["d8b"]["primary_pp"], reg["d8b_mesh_observation"]["value_pp"], places=3)
        self.assertAlmostEqual(v["p10"]["readapt_mean_pp"], reg["p10_policy_readaptation"]["value_pp"], places=2)
        self.assertAlmostEqual(v["p10"]["frozen_cost_pp"], reg["perception_error_cost_frozen"]["value_pp"], places=2)
        self.assertEqual(round(v["e3s"]["median_of_block_medians_pct"], 1), reg["e3s_size_range"]["value_percent"])
        self.assertEqual(v["class_a_count"], 0)
        self.assertEqual(v["p10"]["status"], "INCONCLUSIVE")
        self.assertEqual(v["d8b"]["verdict"], "MATERIAL_LOSS")
        self.assertNotEqual(v["d8b"]["checkpoint_sha256"][:8], "f7022139", "D8b used a different checkpoint")
        self.assertEqual(v["safety"]["negative_cells"], v["safety"]["k"])
        registry = {r["id"]: r for r in json.loads((ROOT / "docs/quantitative_positioning_registry.json")
                                                     .read_text(encoding="utf-8"))["internal_motar"]}
        for got, want in zip(v["safety"]["seed_ci"], registry["safety_filter_geometry"]["seed_level_ci95_pp"]):
            self.assertAlmostEqual(got, want, places=4)
        self.assertEqual(v["safety"]["seeds"], [523, 527, 531])
        self.assertEqual(v["p10"]["frozen_cost_eval_seeds"], [541, 547])
        self.assertFalse(v["real_hardware_reported"]["MOTAR"])

    def test_method_labels_come_from_the_relation_document(self):
        relation = (ROOT / "docs/relation_to_published_systems_2026-09-16.md").read_text(encoding="utf-8")
        keys = {"NavRL": ("PPO", "velocity-obstacle"), "Elastic Tracker": ("EKF", "visibility-aware"),
                "Fast-Tracker": ("EKF", "kinodynamic search"), "OPEN": ("MAPPO", "LSTM evader prediction"),
                "YOPO": ("Guidance-learned motion primitives",)}
        for work, terms in keys.items():
            row = next(line for line in relation.splitlines() if line.startswith("| [" + work + "]"))
            for term in terms:
                self.assertIn(term, row, work)

    def test_byte_exact_rebuild_when_the_recorded_engine_is_available(self):
        try:
            import matplotlib
        except ImportError:
            self.skipTest("matplotlib not installed; values and hashes were checked instead")
        if matplotlib.__version__ != self.manifest["matplotlib"]:
            self.skipTest(f"matplotlib {matplotlib.__version__} differs from recorded {self.manifest['matplotlib']}")
        files = figures.render_all()
        for name, data in files.items():
            if name.endswith(".svg") or name == "manifest.json":
                self.assertEqual((FINAL / name).read_bytes(), data, name)
            else:
                self.assertEqual(hashlib.sha256(data).hexdigest(), self.manifest["files"][name], name)


if __name__ == "__main__":
    unittest.main()
