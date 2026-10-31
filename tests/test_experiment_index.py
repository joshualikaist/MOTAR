"""Experiment index: every experiment record belongs to exactly one family in docs/EXPERIMENTS.md.

A record is a preregistration, plan, evidence folder or result. The index groups them into families,
names the file to read first and says which families can be compared. This test keeps that map
complete, so a new experiment cannot be added without being connected to the ones before it.
No simulator, GPU or network is used.
"""
from fnmatch import fnmatch
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import check_public_docs as docs  # noqa: E402

INDEX = "docs/EXPERIMENTS.md"
# Records first committed on or after this date must be named literally in Members, not only by a
# wildcard.
INDEX_DATE = "2026-10-31"
FAMILY_HEADING = re.compile(r"^### `([a-z0-9-]+)` — ")
FIELD = re.compile(r"^- \*\*([A-Za-z ]+):\*\*(.*)$")
FIELDS = ("Period", "Question", "Policy", "Status", "Read first", "Lineage", "Compare with", "Members")
TICKED = re.compile(r"`([^`]+)`")
COMPARE_LINE = re.compile(r"none|`[a-z0-9-]+`(?:, `[a-z0-9-]+`)*")
TOP_LEVEL_RECORD = re.compile(r"^docs/[^/]+\.md$")
TOP_LEVEL_RECORD_NAME = re.compile(r"(prereg|_results?_|_table\.md$)")


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT), *args], check=True, capture_output=True, text=True).stdout


def tracked_files():
    return [p for p in git("ls-files", "-z").split("\0") if p]


def unit_of(path):
    """The record unit a tracked path belongs to, or None when it is outside the index's scope."""
    parts = path.split("/")
    if parts[0] == "results" and path != "results/MANIFEST.json":
        return f"results/{parts[1]}/" if len(parts) > 2 else path
    if parts[:2] in (["docs", "evidence"], ["docs", "research"]) and len(parts) > 2:
        return f"docs/{parts[1]}/{parts[2]}/" if len(parts) > 3 else path
    if parts[:2] in (["docs", "plans"], ["docs", "specs"], ["docs", "audits"], ["docs", "results"],
                     ["docs", "external_baselines"]):
        return path
    if TOP_LEVEL_RECORD.match(path) and TOP_LEVEL_RECORD_NAME.search(parts[1]):
        return path
    return None


def record_units(files):
    """One unit per result, evidence or research folder; one per file elsewhere."""
    return {u for u in map(unit_of, files) if u}


def first_added(files):
    """Unit -> date its first file was added (renames count as additions); '9999' if not committed."""
    dates, current = {}, None
    for line in git("log", "--no-renames", "--diff-filter=A", "--format=@%cs", "--name-only").splitlines():
        if line.startswith("@"):
            current = line[1:]
        elif line:
            dates[line] = current  # newest commit first, so the oldest addition wins
    units = {}
    for path in files:
        unit = unit_of(path)
        if unit:
            units[unit] = min(units.get(unit, "9999"), dates.get(path, "9999"))
    return units


def parse_index():
    """Family id -> {field: [backticked items]} plus the set of fields present; a field continues on
    its indented lines. Also returns every ### heading under ## Families."""
    families, headings, current, field, in_families = {}, [], None, None, False
    for line in (ROOT / INDEX).read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            in_families = line == "## Families"
        if line.startswith("#"):
            heading = FAMILY_HEADING.match(line)
            if in_families and line.startswith("### "):
                headings.append(line)
            current = heading.group(1) if heading and in_families else None
            if current:
                families[current] = {name: [] for name in FIELDS}
                families[current]["present"] = set()
                families[current]["raw"] = {}
            field = None
            continue
        match = FIELD.match(line)
        if match:
            field = match.group(1) if match.group(1) in FIELDS else None
            if current and field:
                families[current]["present"].add(field)
                families[current]["raw"][field] = match.group(2).strip()
            line = match.group(2)
        elif not line.startswith("  "):
            field = None
        if current and field:
            families[current][field] += TICKED.findall(line)
    return families, headings


class ExperimentIndexTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files = tracked_files()
        cls.units = record_units(cls.files)
        cls.families, cls.headings = parse_index()

    def owners(self, unit):
        return [fid for fid, fam in self.families.items() if any(fnmatch(unit, g) for g in fam["Members"])]

    def test_every_family_heading_parses_and_has_every_field(self):
        self.assertGreater(len(self.families), 10)
        self.assertEqual(len(self.headings), len(self.families), "a ### heading under Families does not parse")
        for fid, fam in self.families.items():
            self.assertEqual(sorted(set(FIELDS) - fam["present"]), [], f"{fid}: missing fields")
            for field in ("Read first", "Lineage", "Members"):
                self.assertTrue(fam[field], f"{fid}: {field} names no path")

    def test_every_record_belongs_to_exactly_one_family(self):
        missing = sorted(u for u in self.units if not self.owners(u))
        shared = sorted(f"{u} -> {self.owners(u)}" for u in self.units if len(self.owners(u)) > 1)
        self.assertEqual(missing, [], "records in no family; add them to docs/EXPERIMENTS.md")
        self.assertEqual(shared, [], "records in more than one family")

    def test_records_added_after_the_index_are_named_literally(self):
        if git("rev-parse", "--is-shallow-repository").strip() == "true":
            self.skipTest("shallow clone: first-add dates are unknown")
        literal = {g for fam in self.families.values() for g in fam["Members"]}
        added = first_added(self.files)
        self.assertGreater(len(added), 400)
        late = sorted(u for u, day in added.items() if day >= INDEX_DATE and u not in literal)
        self.assertEqual(late, [], f"records added on or after {INDEX_DATE} must be listed by name in Members")

    def test_every_member_pattern_matches_a_record(self):
        for fid, fam in self.families.items():
            for pattern in fam["Members"]:
                self.assertTrue(any(fnmatch(u, pattern) for u in self.units), f"{fid}: {pattern} matches nothing")

    def test_read_first_and_lineage_paths_exist(self):
        tracked = set(self.files)
        for fid, fam in self.families.items():
            for path in fam["Read first"] + [p for p in fam["Lineage"] if "/" in p]:
                exists = path in tracked or any(f.startswith(path.rstrip("/") + "/") for f in tracked)
                self.assertTrue(exists, f"{fid}: {path} is not a tracked file or folder")

    def test_read_first_belongs_to_its_family_or_is_outside_the_scope(self):
        for fid, fam in self.families.items():
            for path in fam["Read first"]:
                unit = unit_of(path)
                if unit:
                    self.assertEqual(self.owners(unit), [fid], f"{fid}: read-first {path} is another family's")

    def test_compare_with_names_known_families(self):
        for fid, fam in self.families.items():
            self.assertRegex(fam["raw"].get("Compare with", ""), r"^(?:%s)$" % COMPARE_LINE.pattern,
                             f"{fid}: Compare with must be `none` or backticked family ids")
            for other in fam["Compare with"]:
                self.assertIn(other, self.families, f"{fid}: unknown family {other}")
                self.assertNotEqual(other, fid)

    def test_a_result_and_its_manifest_preregistration_share_a_family(self):
        manifest = json.loads((ROOT / "results/MANIFEST.json").read_text(encoding="utf-8"))
        checked = 0
        for entry in manifest["results"]:
            folder = f"results/{entry['result_id'].split('/')[0]}/"
            for prereg in entry.get("referenced_preregistrations") or []:
                if not prereg.startswith("docs/") or folder not in self.units:
                    continue
                self.assertEqual(self.owners(folder), self.owners(prereg), f"{folder} and {prereg}")
                checked += 1
        self.assertGreater(checked, 0, "no manifest preregistration was compared; the test would be vacuous")

    def test_links_resolve(self):
        self.assertEqual(docs.local_link_errors((ROOT / INDEX).read_text(encoding="utf-8"), ROOT / INDEX), [])


if __name__ == "__main__":
    unittest.main()
