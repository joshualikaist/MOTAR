"""October BOIA progress claims: documentation only, no research/simulator imports."""
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import unittest
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]

class Links(HTMLParser):
    def __init__(self, text):
        super().__init__(convert_charrefs=True)
        self.ids = []; self.refs = []
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        d = dict(attrs)
        if 'id' in d:
            self.ids.append(d['id'])
        for key in ('href', 'src'):
            if key in d:
                self.refs.append(d[key])

class BOIAProgressDocs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads((ROOT / 'docs/status/data/boia-progress.json').read_text())
        cls.readme = (ROOT / 'README.md').read_text()
        cls.pages = {name: (ROOT / 'docs/status' / name).read_text()
                     for name in ('index.html', 'evidence.html')}

    def test_claims_have_pinned_source_metadata_and_no_unpublished_public_links(self):
        for source in self.data['sources'].values():
            self.assertRegex(source['sha256'], r'^[0-9a-f]{64}$')
            self.assertRegex(source['snapshot_commit'], r'^[0-9a-f]{40}$')
            self.assertRegex(source['file_commit'], r'^[0-9a-f]{40}$')
            self.assertFalse(source['path'].startswith('/'))
            self.assertIsNone(source['public_url'])
        for extract in self.data['source_extracts'].values():
            self.assertIn(extract['source'], self.data['sources'])
        for text in [self.readme] + list(self.pages.values()):
            for sha in (self.data['research_snapshot'], self.data['q02a']['source_commit'],
                        self.data['q02b']['source_commit']):
                self.assertNotIn('/commit/' + sha, text)
                self.assertNotIn('/blob/' + sha, text)

    def test_numbers_match_bound_source_extracts(self):
        d = self.data; q = d['q02a']; e = d['source_extracts']
        self.assertEqual(int(re.search(r'\d+', e['focused_cpu']['text']).group()), q['focused_cpu']['run'])
        match = re.search(r'(\d+) ran, (\d+) failures, (\d+) errors', e['research_public_cpu']['text'])
        self.assertEqual(tuple(map(int, match.groups())), tuple(q['research_public_cpu'][k]
                                                              for k in ('run', 'failures', 'errors')))
        self.assertEqual(e['review']['text'].split(': ')[1], q['review_verdict'])
        self.assertEqual(int(re.search(r'\| (\d+) OK', e['q02b_phase1_cpu']['text']).group(1)),
                         d['q02b']['recorded_cpu_tests'])
        evidence = self.pages['evidence.html']
        self.assertIn('{run}/{passed} focused CPU tests PASS'.format(**q['focused_cpu']), evidence)
        self.assertIn('{run:,} tests, {failures} failures, {errors} existing'.format(**q['research_public_cpu']), evidence)
        self.assertIn('Q02-B ' + str(d['q02b']['recorded_cpu_tests']), evidence)
        self.assertIn('112/112 tests OK', evidence)
        self.assertIn('3,177 run, 0 failures, 10 skips and 4 errors', re.sub(r'\s+', ' ', self.readme))
        self.assertIn('3,177 run, 0 failures, 10 skips, 4 errors', evidence)
        self.assertEqual(int(re.search(r'Ran (\d+) tests', e['latest_public_cpu']['text']).group(1)),
                         d['latest_public_cpu']['run'])
        q3 = e['q03_local_map_cpu']['text']
        self.assertEqual(int(re.search(r'\| (\d+) OK', q3).group(1)), d['q03']['local_map_cpu_tests'])
        self.assertIn('Map: 12 CPU tests', evidence)
        self.assertIn('planner: 15 CPU tests', evidence)

    def test_review_pass_never_becomes_perception_accuracy_identity_or_safety_pass(self):
        d = self.data
        self.assertEqual(d['q02a']['research_public_cpu']['aggregate'], 'FAIL')
        self.assertEqual(d['q02a']['residual_p2'], ['H1', 'H2'])
        self.assertEqual(d['q02a']['best_identity_verdict'], 'CONSISTENT_NOT_CONFIRMED')
        self.assertEqual(d['q02a']['utility'], 'ABSTAIN')
        self.assertFalse(d['q02b']['package_complete'])
        self.assertTrue(d['q02b']['independent_review_complete'])
        self.assertEqual(d['q02b']['independent_review_verdict'], 'FAIL')
        self.assertEqual(d['q02b']['review_findings'], {'p0': 0, 'p1': 2, 'p2': 9})
        self.assertFalse(d['q03']['physical_free_volume_certificate'])
        self.assertTrue(d['q03']['independent_review_complete'])
        self.assertEqual(d['q03']['planner_review'], 'FAIL_P1_2')
        self.assertEqual(d['latest_public_cpu']['aggregate'], 'FAIL')
        for text in [self.readme] + list(self.pages.values()):
            for phrase in ('NOT VALIDATED', 'NOT CONFIRMED', 'G5 HOLD', 'RGB-D', 'LiDAR'):
                self.assertIn(phrase, text)
            self.assertIn('shadow-only', text.lower())
            self.assertIn('First acquisition', text)
            self.assertNotIn('BOIA improves capture', text)
        for text in self.pages.values():
            self.assertIn(d['q02a']['detector_provenance'], text)

    def test_milestone_completion_is_consistent(self):
        self.assertTrue(self.data['q01']['recorded_gpu_run_completed'])
        self.assertEqual(self.data['milestones']['Q02-B'], 'CPU_TESTED_INDEPENDENT_REVIEW_FAIL_HOLD')
        self.assertEqual(self.data['milestones']['Q03'], 'CPU_TESTED_INDEPENDENT_REVIEW_FAIL_HOLD')
        for text in self.pages.values():
            ids = re.findall(r'data-boia-milestone="([^"]+)"', text)
            self.assertEqual(ids, ['Q01', 'Q02-A', 'Q02-B', 'Q03', 'Q04'])
            self.assertIn('Independent review FAIL', text)
            self.assertIn('HOLD', text)
        self.assertIn('D0 HOLD', self.pages['evidence.html'])
        self.assertIn('Design only', self.pages['index.html'])
        self.assertIn('REACQUIRE PROPOSAL — PLANNED; no online command', self.pages['index.html'])

    def test_assets_anchors_and_relative_html_links_exist(self):
        for name, text in self.pages.items():
            source = ROOT / 'docs/status' / name
            parsed = Links(text)
            self.assertEqual(len(parsed.ids), len(set(parsed.ids)), name)
            self.assertEqual(text.count('<h1>'), 1)
            for raw in parsed.refs:
                link = urlsplit(raw)
                if link.scheme or link.netloc:
                    continue
                target = (source.parent / unquote(link.path)).resolve() if link.path else source
                if target.is_dir():
                    target /= 'index.html'
                self.assertTrue(target.exists(), (name, raw))
                if link.fragment and target.suffix == '.html':
                    self.assertIn(unquote(link.fragment), Links(target.read_text()).ids, (name, raw))

    def test_public_release_and_browser_preview_remain_distinct(self):
        for text in self.pages.values():
            self.assertIn('https://github.com/joshualikaist/MOTAR-public', text)
            self.assertIn('local', text)
        landing = self.pages['index.html']
        for phrase in ('BROWSER GT PREVIEW', 'NOT PPO', 'NOT PHYSX', 'id="demo"'):
            self.assertIn(phrase, landing)

if __name__ == '__main__':
    unittest.main()
