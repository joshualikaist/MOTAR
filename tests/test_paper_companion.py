"""Publication guards for the review paper/deck; standard library only, no Office/GPU dependencies."""
import hashlib
import json
from pathlib import Path
import re
import sys
import unittest
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import check_public_docs as public_docs

PAPER = ROOT / "paper/MOTAR_draft_2026-09-26.md"
PPTX = ROOT / "presentations/MOTAR_overview_2026-09-26.pptx"
DOCX = ROOT / "paper/MOTAR_draft_2026-09-26.docx"
NS = {"a": "http://schemas.openxmlformats.org/drawingml/2006/main",
      "p": "http://schemas.openxmlformats.org/presentationml/2006/main",
      "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main"}


def pp(value, digits=2):
    return ("{:+." + str(digits) + "f}").format(value).replace("-", "−")


class PaperCompanionTest(unittest.TestCase):
    def setUp(self):
        self.paper = PAPER.read_text()
        self.manifest = json.loads((ROOT / "presentations/companion_manifest.json").read_text())
        self.values = json.loads((ROOT / "docs/assets/paper/final/manifest.json").read_text())["values"]

    def test_source_and_output_integrity(self):
        for group in ("sources", "files"):
            for path, expected in self.manifest[group].items():
                self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), expected, path)
        self.assertEqual(len(self.paper.split()), self.manifest["paper_word_count"])
        for slide in self.manifest["slides"]:
            for path in slide["sources"]:
                self.assertTrue((ROOT / path).is_file(), path)

    def test_paper_structure_and_all_six_figures(self):
        numbers = re.findall(r"^## (\d+)\. ", self.paper, re.M)
        self.assertEqual(numbers, [str(n) for n in range(1, 12)])
        figures = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", self.paper)
        self.assertEqual(len(figures), 6)
        for n, path in enumerate(figures, 1):
            self.assertTrue(Path(path).name.startswith("fig" + str(n) + "-"))
            self.assertTrue((PAPER.parent / path).is_file())

    def test_current_effects_bound_to_canonical_records(self):
        with zipfile.ZipFile(PPTX) as archive:
            slide = ET.fromstring(archive.read("ppt/slides/slide7.xml"))
            text = " ".join(node.text or "" for node in slide.findall(".//a:t", NS))
        tm = self.values["target_motion"]
        for effect in tm["contrasts_vs_H"].values():
            for value in [effect["mean_pp"]] + effect["bca95_pp"]:
                self.assertIn(pp(value), self.paper)
                self.assertIn(pp(value), text)
            for p in (effect["exact_p"], effect["holm_p"]):
                self.assertIn(str(p), self.paper)
                self.assertIn(str(p), text)
        self.assertIn(format(tm["episodes"], ","), text)
        self.assertIn("resolution boundary", text)
        self.assertIn("exploratory", text)
        for key, field, digits in (("p10", "frozen_cost_pp", 2), ("p10", "readapt_mean_pp", 2),
                                   ("safety", "pooled_pp", 4), ("d8b", "primary_pp", 3)):
            self.assertIn(pp(self.values[key][field], digits), self.paper)

    def test_deck_is_editable_with_complete_notes_and_limits(self):
        with zipfile.ZipFile(PPTX) as archive:
            slides = [p for p in archive.namelist() if re.fullmatch(r"ppt/slides/slide\d+\.xml", p)]
            self.assertEqual(len(slides), 12)
            self.assertEqual(self.manifest["slide_count"], 12)
            all_text = []
            for n in range(1, 13):
                slide = ET.fromstring(archive.read("ppt/slides/slide%d.xml" % n))
                text = " ".join(t.text or "" for t in slide.findall(".//a:t", NS))
                self.assertGreater(len(text), 100)
                # An editable deck must not be a stack of page-sized screenshots.
                self.assertGreater(len(slide.findall(".//p:sp", NS)), len(slide.findall(".//p:pic", NS)))
                notes = ET.fromstring(archive.read("ppt/notesSlides/notesSlide%d.xml" % n))
                note_text = " ".join(t.text or "" for t in notes.findall(".//a:t", NS))
                self.assertIn("Sources (repository-relative):", note_text)
                self.assertGreater(len(note_text), 300)
                all_text.append(text)
            text = " ".join(all_text)
            for limit in ("INCONCLUSIVE", "MATERIAL_LOSS", "NOT_TESTED", "WITHHELD_SEMANTIC_MISMATCH",
                          "Class A matched external comparisons = 0", "Simulation only", "resolution boundary"):
                self.assertIn(limit, text)

    def test_word_has_native_text_figures_and_clean_metadata(self):
        with zipfile.ZipFile(DOCX) as archive:
            body = ET.fromstring(archive.read("word/document.xml"))
            text = " ".join(t.text or "" for t in body.findall(".//w:t", NS))
            self.assertGreater(len(text.split()), 4000)
            self.assertEqual(len(body.findall(".//w:drawing", NS)), 6)
            for term in ("MATERIAL_LOSS", "NOT_TESTED", "INCONCLUSIVE", "WITHHELD_SEMANTIC_MISMATCH"):
                self.assertIn(term, text)
            metadata = archive.read("docProps/core.xml").decode()
            self.assertIn("Minseok Lee", metadata)
            self.assertIn("Hyondong Oh", metadata)
            for legacy in ("Electric Arc Furnace", "voltage sag", "KIEE"):
                self.assertNotIn(legacy, metadata + text)

    def test_paper_and_authoring_local_links_resolve(self):
        for path in (PAPER, ROOT / "paper/PAPER_TODO_2026-09-26.md", ROOT / "presentations/README.md"):
            self.assertEqual(public_docs.local_link_errors(path.read_text(), path, ROOT), [])

    def test_readme_and_site_expose_current_review_artifacts(self):
        for path in (ROOT / "README.md", ROOT / "docs/status/index.html", ROOT / "docs/status/evidence.html"):
            text = path.read_text()
            for artifact in (PAPER, PPTX):
                self.assertIn(artifact.relative_to(ROOT).as_posix(), text)
        for limit in ("INCONCLUSIVE", "NOT_TESTED", "resolution boundary", "WITHHELD_SEMANTIC_MISMATCH"):
            self.assertIn(limit, self.paper)


if __name__ == "__main__":
    unittest.main()
