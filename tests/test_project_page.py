"""Project-page contracts for docs/status/index.html (the concise paper project page, 2026-09-24).

The detailed tables, lifecycle registry and old figure set live on docs/status/evidence.html and are
tested in test_research_overview.py. No browser, simulator or network is used here.
"""
from html import unescape
from html.parser import HTMLParser
from pathlib import Path
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "docs/status"
sys.path.insert(0, str(ROOT / "tools"))
import check_public_docs as docs  # noqa: E402

THESIS = ("MOTAR studies the full evidence chain from measured perception uncertainty to policy "
          "behaviour, safety geometry, target-motion generalization, and observation sensitivity "
          "under frozen UAV policies.")
KEY_MESSAGE = ("<strong>Target-motion effects replicated under a bias-corrected evaluation.</strong> Target motion "
               "was not a monotonic difficulty ladder: the static target produced the lowest capture rate, while "
               "the obstacle-aware target produced the highest.")
SECTIONS = ["question", "findings", "system", "perception-policy", "safety", "target-motion",
            "observation", "related", "reproducibility", "demo"]
FIGURES = ["fig1-motar-overview", "fig2-perception-policy", "fig3-safety", "fig4-target-motion",
           "fig5-observation-contract", "fig6-positioning"]


class Page(HTMLParser):
    def __init__(self, text):
        super().__init__()
        self.ids, self.nav, self.images, self._nav = [], [], [], False
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag == "nav":
            self._nav = True
        if "id" in a:
            self.ids.append(a["id"])
        if tag == "a" and self._nav:
            self.nav.append(a.get("href", ""))
        if tag == "img":
            self.images.append(a)

    def handle_endtag(self, tag):
        if tag == "nav":
            self._nav = False

    def handle_data(self, data):
        if self._nav and data.strip():
            self.nav.append("text:" + data.strip())


class ProjectPageTest(unittest.TestCase):
    def setUp(self):
        self.text = (SITE / "index.html").read_text(encoding="utf-8")
        self.page = Page(self.text)
        self.story = self.text.split('<section class="demo" id="demo"', 1)[0]

    def test_first_screen_states_what_how_finding_and_boundary(self):
        self.assertIn("<h1>MOTAR</h1>", self.text)
        self.assertIn("Moving Object Tracking And Rendezvous", self.text)
        self.assertIn("Reinforcement Learning for UAV Tracking and Close Approach in Random Obstacle Fields",
                      self.text)
        self.assertIn(THESIS, self.text)
        self.assertIn(KEY_MESSAGE, self.text)
        hero = self.text.split('<figure class="fig" id="fig1">', 1)[0]
        self.assertEqual(hero.count("<li>"), 3, "exactly three result call-outs above Figure 1")
        self.assertIn("Simulation only", hero)
        self.assertIn("no real-flight validation", hero)

    def test_primary_navigation(self):
        labels = [item[5:] for item in self.page.nav if item.startswith("text:")]
        self.assertEqual(labels, ["Overview", "Results", "Method", "Figures", "Evidence", "Code ↗"])

    def test_sections_in_order_and_demo_last(self):
        positions = [self.text.index(f'id="{name}"') for name in SECTIONS]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("figures", self.page.ids)
        self.assertEqual(len(self.page.ids), len(set(self.page.ids)))

    def test_figure_family_in_order_with_tracked_sources(self):
        positions = []
        for number, stem in enumerate(FIGURES, start=1):
            self.assertIn(f"<strong>Figure {number}.", self.story)
            positions.append(self.story.index(f"<strong>Figure {number}."))
            # SVG is the tracked format; PNG and PDF are rebuilt on demand (tests/test_paper_figures_final.py).
            self.assertIn(f"../assets/paper/final/{stem}.svg", self.story)
            self.assertTrue((ROOT / f"docs/assets/paper/final/{stem}.svg").is_file())
            for ext in ("png", "pdf"):
                self.assertNotIn(f"../assets/paper/final/{stem}.{ext}", self.story, "untracked render linked")
        self.assertEqual(positions, sorted(positions), "figure numbers must appear in order")
        self.assertNotIn("<strong>Figure 7.", self.story)
        for image in self.page.images:
            self.assertGreater(len(image.get("alt", "")), 15)
            if image["src"] != "../assets/paper/final/fig1-motar-overview.svg":
                self.assertEqual(image.get("loading"), "lazy", image["src"])

    def test_every_number_in_the_story_is_bound_to_a_record(self):
        prose = unescape(re.sub(r"<[^>]+>", " ", self.story))
        values = docs.headline_values()
        unbound = []
        for token in re.findall(r"[−+-]?\d+\.\d+", prose):
            shown = float(token.replace("−", "-"))
            digits = len(token.split(".")[1])
            if not any(abs(abs(shown) - abs(v)) <= 0.5 * 10 ** -digits + 1e-9 for v in values):
                unbound.append(token)
        self.assertEqual(unbound, [])

    def test_limits_travel_with_the_results(self):
        for phrase in ("INCONCLUSIVE", "MATERIAL_LOSS", "CAUSALITY NOT TESTED",
                       "Class A matched external comparisons: 0", "not a leaderboard",
                       "consistent with a first-acquisition / visibility mechanism",
                       "frozen policy R", "is not policy F", "not a DWA planner", "no collision guarantee",
                       "The original campaign is shown beside the replication and not pooled", "not independent replicates",
                       "Unit of inference: evaluation seed (n = 7)",
                       "Unit of inference: paired evaluation seed (n = 3)", "its 3,107 frames are not independent"):
            self.assertIn(phrase, self.story)
        for forbidden in ("state-of-the-art performance", "outperform", "causes the loss",
                          "guarantees safety", "recovers the loss"):
            self.assertNotIn(forbidden, self.story)

    def test_target_motion_section_scopes_the_target(self):
        section = self.text.split('id="target-motion"', 1)[1].split("</section>", 1)[0]
        self.assertIn("No target law reacts to the pursuer", section)
        self.assertIn("obstacle-aware is not adversarial", section)
        self.assertIn("target_motion_algorithm_2026-09-17.md", section)
        self.assertNotIn("A*", section)

    def test_demo_is_labelled_and_separate(self):
        demo = self.text.split('<section class="demo" id="demo"', 1)[1]
        self.assertIn("BROWSER GT PREVIEW · NOT PPO EVIDENCE", demo)
        self.assertIn("None of the results above depend on it", demo)
        self.assertIn('id="arena"', demo)
        self.assertNotIn('id="arena"', self.story)

    def test_accessibility_basics(self):
        self.assertIn('<html lang="en">', self.text)
        self.assertIn('class="skip-link" href="#main"', self.text)
        self.assertIn('<main id="main"', self.text)
        self.assertEqual(self.text.count("<h1>"), 1)

    def test_stylesheet_follows_the_design_system(self):
        css = (SITE / "site.css").read_text(encoding="utf-8")
        self.assertNotRegex(css, r"(?:linear|radial|conic)-gradient\(")
        for forbidden in ("box-shadow: 0 ", "backdrop-filter", "@import", "fonts.googleapis", "@font-face"):
            self.assertNotIn(forbidden, css)
        for radius in re.findall(r"border-radius:\s*(\d+)px", css):
            self.assertLessEqual(int(radius), 2)
        for required in ("prefers-reduced-motion", ":focus-visible", "@media (max-width: 760px)",
                         "overflow-x: auto"):
            self.assertIn(required, css)
        self.assertIn('href="site.css', self.text)

    def test_story_is_short(self):
        words = re.findall(r"[A-Za-z]+", unescape(re.sub(r"<[^>]+>", " ", self.story)))
        self.assertLessEqual(len(words), 2200)


if __name__ == "__main__":
    unittest.main()
