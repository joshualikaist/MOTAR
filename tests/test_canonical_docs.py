"""Canonical documentation surface: existence, one job per file, bounded README, source-bound numbers.

Added with the 2026-09-24 reorganisation. No simulator, GPU or network is used.
"""
from pathlib import Path
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import check_public_docs as docs  # noqa: E402

SOURCE_OF_TRUTH = [
    ("What is MOTAR?", "README.md"),
    ("What are we researching?", "PROJECT.md"),
    ("How does the system work?", "ARCHITECTURE.md"),
    ("How should agents modify it?", "AGENTS.md"),
    ("What results are established?", "docs/EVIDENCE.md"),
    ("How do I reproduce it?", "docs/REPRODUCIBILITY.md"),
    ("How do I operate it?", "docs/OPERATIONS.md"),
    ("What happened historically?", "docs/HISTORY.md"),
    ("How should the site and figures look?", "DESIGN_SYSTEM.md"),
]
CANONICAL = [path for _, path in SOURCE_OF_TRUTH] + ["docs/README.md"]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


class CanonicalFilesTest(unittest.TestCase):
    def test_every_canonical_file_exists(self):
        for path in CANONICAL:
            self.assertTrue((ROOT / path).is_file(), path)

    def test_docs_index_carries_the_source_of_truth_table(self):
        index = read("docs/README.md")
        for question, path in SOURCE_OF_TRUTH:
            row = next((line for line in index.splitlines() if line.startswith("| " + question)), None)
            self.assertIsNotNone(row, question)
            self.assertIn(Path(path).name, row, question)

    def test_canonical_links_resolve(self):
        for path in CANONICAL + ["docs/archive/README.md", "docs/cleanup/cleanup_record_2026-09-24.md",
                                  "docs/cleanup/new_file_budget.md", "CLAUDE.md"]:
            self.assertEqual(docs.local_link_errors(read(path), ROOT / path), [], path)

    def test_claude_md_is_a_pointer_to_agents_md(self):
        claude = read("CLAUDE.md")
        self.assertIn("@AGENTS.md", claude)
        self.assertLess(len(claude.splitlines()), 30, "CLAUDE.md must stay a short pointer")

    def test_agents_md_states_the_research_and_git_rules(self):
        agents = read("AGENTS.md")
        for rule in ("Never auto-start PPO training or GPU evaluation",
                     "Never silently change a frozen experiment",
                     "Never rewrite a negative result",
                     "Never promote NOT_TESTED",
                     "Never fabricate a missing metric",
                     "Never change a preregistration after measurement",
                     "must never enter the actor observation",
                     "No force-push", "Show the diff", "WORKLOG.md"):
            self.assertIn(rule, agents)

    def test_project_md_stays_a_definition_not_a_log(self):
        project = read("PROJECT.md")
        self.assertLessEqual(len(project.splitlines()), 250)
        self.assertNotRegex(project, r"(?m)^## 20\d\d-")


class ReadmeGuardTest(unittest.TestCase):
    """The README is the five-minute entry point and must not grow back into a research diary."""

    def setUp(self):
        self.text = read("README.md")

    def test_structure_is_the_landing_contract(self):
        headings = re.findall(r"(?m)^## (.+)$", self.text)
        self.assertEqual(headings, list(docs.LANDING_HEADINGS))
        self.assertTrue(self.text.startswith("# MOTAR\n"))

    def test_length_is_bounded(self):
        prose = re.sub(r"```.*?```", "", self.text, flags=re.S)
        self.assertLessEqual(len(self.text.splitlines()), 150)
        self.assertLessEqual(len(re.findall(r"\w+", prose)), 1500)

    def test_no_experiment_log_patterns(self):
        prose = re.sub(r"```.*?```", "", self.text, flags=re.S)
        self.assertNotRegex(prose, r"(?m)^#+ .*20\d\d-\d\d-\d\d", "dated headings belong in WORKLOG")
        dated_links = re.findall(r"\]\([^)]*_20\d\d-\d\d-\d\d[^)]*\)", self.text)
        self.assertLessEqual(len(dated_links), 3, "README should link canonical docs, not dated records")
        self.assertLessEqual(len(re.findall(r"!\[", self.text)), 2)

    def test_numbers_are_bound_to_canonical_records(self):
        self.assertEqual(docs.landing_number_errors(self.text, docs.headline_values()), [])
        self.assertTrue(docs.landing_number_errors(self.text.replace("−5.37", "−5.73"),
                                                   docs.headline_values()))

    def test_scope_boundary_and_limits_are_on_the_landing_page(self):
        for phrase in ("Simulation only", "no real-flight validation claim", "INCONCLUSIVE",
                       "NOT_TESTED", "MATERIAL_LOSS", "Class A matched comparisons = 0"):
            self.assertIn(phrase, self.text)


class EvidencePageTest(unittest.TestCase):
    """docs/EVIDENCE.md quotes each canonical value exactly as its record states it."""

    def assert_doc_and_source(self, doc_phrases, source, source_phrases):
        text = read("docs/EVIDENCE.md")
        for phrase in doc_phrases:
            self.assertIn(phrase, text)
        record = read(source)
        for phrase in source_phrases:
            self.assertIn(phrase, record, source)

    def test_perception_range(self):
        self.assert_doc_and_source(("**6.2 %**", "3,107 frames, 9 blocks"),
                                   "results/eth_ds5_e3s_2026-09-10/run/e3s_result.json",
                                   ('"median_of_block_medians": 0.0618', '"included_frames": 3107'))

    def test_perception_to_policy_and_readaptation(self):
        self.assert_doc_and_source(("**−4.57 pp**", "[−6.32, −2.82]", "**+0.73 pp**", "[−1.04, +2.50]",
                                    "INCONCLUSIVE", "−1.90, −2.71, −3.84 pp"),
                                   "results/perception_p10_seed_replication_2026-09-10/README.md",
                                   ("mean **+0.73 pp**", "CI **[−1.04, +2.50]**", "−4.57", "−1.90, −2.71,"))

    def test_safety_geometry(self):
        self.assert_doc_and_source(("**−1.4903 pp**", "[−1.8981, −1.0826]", "**15/15**"),
                                   "results/independent_verification_2026-09-07/README.md",
                                   ("−1.4903 percentage points", "[−1.8981, −1.0826]"))

    def test_target_motion(self):
        self.assert_doc_and_source(("**E0 −3.58 pp**", "E1 −2.82 pp", "**E2 +1.33 pp**", "+4.91 pp",
                                    "95.42 %", "1,667 of 1,747"),
                                   "docs/results/target_motion_generalization_2026-09-19.md",
                                   ("**+1.33 pp**", "**-3.58 pp**", "**-2.82 pp**", "**+4.91 pp**"))
        audit = read("docs/results/target_motion_visibility_reacquisition_audit_2026-09-19.json")
        self.assertIn('"never_acquired": 1667', audit)

    def test_observation_contract(self):
        self.assert_doc_and_source(("**−48.967 pp**", "[−50.113, −47.821]", "**MATERIAL_LOSS**",
                                    "**Causality NOT_TESTED**", "ref5in D1, ep1900, 70 bars"),
                                   "results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md",
                                   ("**-48.967 pp**", "[-50.113, -47.821]", "MATERIAL_LOSS"))

    def test_class_a_is_zero(self):
        text = read("docs/EVIDENCE.md")
        self.assertIn("**Class A = 0**", text)
        self.assertIn('"class_a_count": 0', read("docs/quantitative_positioning_registry.json"))


# Every surface a reader treats as current: canonical docs, both site pages, the paper drafts, the
# registry and its generated positioning document, and the text of the final figures.
CURRENT_SURFACES = [
    "README.md", "PROJECT.md", "ARCHITECTURE.md", "AGENTS.md", "DESIGN_SYSTEM.md", "CITATION.cff",
    "docs/README.md", "docs/EVIDENCE.md", "docs/HISTORY.md", "docs/OPERATIONS.md", "docs/REPRODUCIBILITY.md",
    "docs/status/index.html", "docs/status/evidence.html",
    "docs/paper_outline_2026-09-19.md", "docs/paper_evidence_spine_2026-09-19.md",
    "docs/paper_claim_evidence_matrix_2026-09-19.md",
    "docs/quantitative_positioning_registry.json", "docs/literature_quantitative_positioning_2026-09-18.md",
    "docs/cleanup/preserved_contract_paths.md", "docs/d8c_perception_path_audit_2026-09-24.md",
]


def surfaces():
    for path in CURRENT_SURFACES:
        yield path, read(path)
    for svg in sorted((ROOT / "docs/assets/paper/final").glob("*.svg")):
        yield str(svg.relative_to(ROOT)), re.sub(r"<[^>]+>", " ", svg.read_text(encoding="utf-8"))


def json_record(path):
    import json
    return json.loads(read(path))


class ClaimConsistencyTest(unittest.TestCase):
    """P0 claim audit of 2026-09-24: counts, contrast families, policy identity and inferential units."""

    def test_manifest_count_comes_from_the_manifest(self):
        manifest = json_record("results/MANIFEST.json")
        count = manifest["counts"]["results"]
        built = manifest["built_at_utc"][:10]
        from datetime import date
        stamp = date.fromisoformat(built)
        dates = (built, "%d %s %d" % (stamp.day, stamp.strftime("%B"), stamp.year))
        pattern = re.compile(r"(?:\*\*)?(\d{3})(?:\*\*|</span>)?\s+(?:indexed\s+)?result\s+"
                             r"(?:entries|directories|records)|manifest-count\">(\d+)<")
        seen = 0
        for path, text in surfaces():
            for match in pattern.finditer(text):
                seen += 1
                self.assertEqual(int(match.group(1) or match.group(2)), count, f"{path}: {match.group(0)}")
                window = text[max(0, match.start() - 250):match.end() + 250]
                self.assertTrue(any(d in window for d in dates), f"{path}: count without its build date {dates}")
        self.assertGreater(seen, 0, "no surface states the manifest count; the test would be vacuous")

    def test_no_unscoped_single_policy_claim(self):
        forbidden = [r"\bthe frozen (?:policy|checkpoint)\b(?! [FR]\b)", r"\bsame frozen (?:policy|checkpoint)\b",
                     r"\bsingle frozen (?:policy|checkpoint)\b", r"\bone frozen checkpoint\b",
                     r"\bone frozen policy\b(?!,? \(?[FR]\b)"]
        for path, text in surfaces():
            for pattern in forbidden:
                match = re.search(pattern, text, flags=re.IGNORECASE)
                self.assertIsNone(match, f"{path}: unscoped single-policy wording {match and match.group(0)!r}")

    def test_contrast_family_statements_are_true(self):
        summary = json_record("results/target_motion_e0_e2_2026-09-18/canonical_summary.json")
        rows = summary["contrasts_vs_H"] + summary["contrasts_within_E"]

        def consistent(values):
            return all(v > 0 for v in values) or all(v < 0 for v in values)
        capture = [r for r in rows if r["metric"] == "capture_rate"]
        self.assertEqual(len(capture), 6)
        self.assertTrue(all(consistent(list(r["seed_diffs"].values())) for r in capture))
        mixed = [(r["arm"], r["metric"]) for r in rows if not consistent(list(r["seed_diffs"].values()))]
        self.assertEqual(mixed, [("E2_obstacle_aware", "crash_rate")])
        # The preregistration's "12" are the exploratory per-density contrasts against H (Amendment 1 A1.3).
        raw = ROOT / "results/target_motion_e0_e2_evaluation_2026-09-18"
        rate = {}
        for cell in raw.glob("*__*bars__seed*/result.json"):
            arm, bars, seed = re.match(r"(\w+?)__(\d+)bars__seed(\d+)", cell.parent.name).groups()
            rate[arm, int(bars), int(seed)] = json_record(str(cell.relative_to(ROOT)))["outcome"]["capture_rate"]
        per_density = {(arm, bars): consistent([rate[arm, bars, s] - rate["H_historical", bars, s]
                                                for s in (4101, 4102, 4103)])
                       for arm in ("E0_static", "E1_cv", "E2_obstacle_aware") for bars in (70, 115, 160, 205)}
        self.assertEqual(len(per_density), 12)
        self.assertEqual(sorted(k for k, ok in per_density.items() if not ok),
                         [("E2_obstacle_aware", 115), ("E2_obstacle_aware", 160)])
        evidence = read("docs/EVIDENCE.md")
        for phrase in ("All six arm-level capture contrasts are 3/3 same sign", "10 of 12",
                       "E2 − H is mixed at 115 and 160 bars", "the E2 − H crash contrast is sign-mixed"):
            self.assertIn(phrase, evidence)
        for path, text in surfaces():
            for claim in (r"all (?:twelve|12) contrasts", r"all contrasts 3/3", r"every contrast is 3/3"):
                self.assertIsNone(re.search(claim, text, flags=re.IGNORECASE), f"{path}: {claim}")

    def test_frames_are_never_the_unit(self):
        for path, text in surfaces():
            for match in re.finditer(r"3,107 frames", text):
                window = text[max(0, match.start() - 220):match.end() + 220].lower()
                self.assertIn("block", window, path)
                self.assertRegex(window, r"one (?:real |eth ds5 )?flight", path)

    def test_safety_cells_are_not_presented_as_replicates(self):
        for path, text in surfaces():
            for match in re.finditer(r"15/15", text):
                window = text[max(0, match.start() - 260):match.end() + 260].lower()
                self.assertIn("seed", window, f"{path}: 15/15 without its seed structure")
        record = json_record("results/independent_verification_2026-09-07/recomputed.json")
        seed_t = record["contrasts"]["dwa_arc-riskcap"]["seed_t"]
        self.assertEqual(seed_t["df"], 2)
        row = next(r for r in json_record("docs/quantitative_positioning_registry.json")["internal_motar"]
                   if r["id"] == "safety_filter_geometry")
        self.assertEqual(row["seed_level_ci95_pp"], [round(seed_t["lo"], 4), round(seed_t["hi"], 4)])
        for path in ("README.md", "docs/EVIDENCE.md", "docs/status/index.html"):
            self.assertIn("[−2.42, −0.57]", read(path), path)

    def test_headline_intervals_state_their_unit(self):
        rows = {r["id"]: r for r in json_record("docs/quantitative_positioning_registry.json")["internal_motar"]}
        units = {"safety_filter_geometry": "15 seed x density cells", "perception_error_cost_frozen": "episode-level",
                 "p10_policy_readaptation": "training seed, n = 3", "d8b_mesh_observation": "paired evaluation seed",
                 "target_motion_generalization": "evaluation seed, n = 7"}
        for key, unit in units.items():
            self.assertIn(unit, rows[key]["ci95_unit"], key)
        p10 = json_record("results/perception_p10_seed_replication_2026-09-10/summary.json")
        cost = next(iter(p10["per_training_seed"].values()))["p9_cost_on_source"]
        self.assertEqual((cost["a"], cost["b"]), ("3161/4099", "3350/4101"))
        self.assertEqual(rows["perception_error_cost_frozen"]["ci95_pp"],
                         [round(cost["ci95"][0], 2), round(cost["ci95"][1], 2)])
        d8b = json_record("results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/summary.json")["primary"]
        self.assertEqual((d8b["unit"], d8b["df"]), ("paired evaluation seed", 2))


class TestProfilesTest(unittest.TestCase):
    """The documented default test command is a named profile, and its CUDA exclusions are explicit."""

    def test_profiles_are_documented_and_exact(self):
        import run_tests
        self.assertEqual(run_tests.PROFILES, ("PUBLIC_CPU", "GPU_REQUIRED", "FULL_RESEARCH"))
        for doc in ("AGENTS.md", "docs/OPERATIONS.md", "docs/REPRODUCIBILITY.md"):
            self.assertIn("tools/run_tests.py", read(doc), doc)
        for entry, reason in run_tests.GPU_REQUIRED.items():
            self.assertTrue((ROOT / "tests" / (entry.split(".")[0] + ".py")).is_file(), entry)
            self.assertIn("CUDA", reason, entry)
        self.assertNotIn("skip themselves", read("docs/OPERATIONS.md") + read("docs/REPRODUCIBILITY.md"))


class DesignTokensTest(unittest.TestCase):
    """One palette: DESIGN_SYSTEM.md, the site stylesheet and the figure generator agree."""

    def test_tokens_match(self):
        design = read("DESIGN_SYSTEM.md")
        css = read("docs/status/site.css")
        generator = read("tools/build_paper_figures.py")
        for token, hex_value, constant in (("--ink", "#1f1f1f", "INK"), ("--ink-2", "#4a4a4f", "INK2"),
                                           ("--muted", "#6e6e73", "MUTED"), ("--blue", "#2f6599", "BLUE"),
                                           ("--accent", "#c2562b", "ACCENT"), ("--blue-wash", "#e8eef6", "BLUE_WASH")):
            self.assertIn(f"`{token}` | `{hex_value}`", design)
            self.assertIn(f"{token}: {hex_value};", css)
            self.assertIn(f'{constant} = "{hex_value}"', generator)


if __name__ == "__main__":
    unittest.main()
