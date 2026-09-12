"""Current overview contracts; historical detail contracts remain separately tested."""
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import struct
import unittest
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / 'docs/status'
FIG = ROOT / 'docs/assets/paper/overview-2026-09-13'


class Page(HTMLParser):
    def __init__(self, text):
        super().__init__()
        self.ids, self.links, self.images, self.scripts = [], [], [], []
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if 'id' in a:
            self.ids.append(a['id'])
        for name in ('href', 'src'):
            if name in a:
                self.links.append(a[name])
        if tag == 'img':
            self.images.append(a)
        if tag == 'script' and 'src' in a:
            self.scripts.append(a['src'])


class ResearchOverviewTest(unittest.TestCase):
    def setUp(self):
        self.text = (SITE / 'index.html').read_text()
        self.page = Page(self.text)

    def test_sections_and_accessible_images(self):
        self.assertEqual(len(self.page.ids), len(set(self.page.ids)))
        for section in ('overview','contributions','arena','system','perception',
                        'parameters','algorithms','evidence','external-data','next'):
            self.assertIn(section, self.page.ids)
        self.assertEqual(len(self.page.images), 7)
        for image in self.page.images:
            self.assertGreater(len(image.get('alt','')), 15)
        self.assertIn('본문 바로가기', self.text)
        self.assertIn('<noscript>', self.text)

    def test_all_active_local_links_and_html_fragments(self):
        for link in self.page.links:
            parsed = urlsplit(link)
            if parsed.scheme or parsed.netloc:
                continue
            target = (SITE / unquote(parsed.path or 'index.html')).resolve()
            self.assertIn(ROOT, target.parents, link)
            self.assertTrue(target.exists(), link)
            if parsed.fragment and target.suffix == '.html':
                self.assertIn(unquote(parsed.fragment), Page(target.read_text()).ids, link)

    def test_viewer_dom_and_script_order_preserved(self):
        archive = Page((SITE / 'archive-2026-09-13.html').read_text())
        for name in archive.ids:
            if name == 'stage' or name.startswith(('hud-','sl-','lbl-','btn-','cb-','sel-')):
                self.assertIn(name, self.page.ids)
        self.assertIn('id="motion-mode-note"', self.text)
        viewer = [s for s in archive.scripts if 'status_manifest' not in s]
        self.assertEqual([s for s in self.page.scripts if 'status_manifest' not in s], viewer)
        self.assertIn('value="routed-preview" selected', self.text)
        self.assertIn('NOT PhysX/PPO', self.text)

    def test_current_claim_boundaries(self):
        for phrase in ('SIMULATION ONLY', 'interception/capture', 'GT box가 아닌',
                       '블록별 절대 상대오차 중앙값들의 중앙값', '순효과는 확립되지',
                       'shortcut 감소는 미측정', '실사 파이프라인', '철회된 C3',
                       '후보를 선택', '지속적인 물체 ID'):
            self.assertIn(phrase, self.text)
        self.assertNotRegex(self.text, r'D[1-9][^<\n]{0,80}\b(?:GO|INCONCLUSIVE|NOT_STARTED|NOT_RUN)\b')
        data = json.loads((ROOT/'docs/status_manifest.json').read_text())
        for key, status in [('D6','INCONCLUSIVE'),('D7','GO'),('D8','NOT_STARTED'),('D9','NOT_RUN')]:
            self.assertEqual(data['track_d'][key]['status'], status)

    def test_defaults_are_not_presented_as_all_run_settings(self):
        for value in ('36 × 4 / 4 m','72 × 4 / 12 m','2.0 m/s','2.5 m/s','기본값 256','T=16','17-token'):
            self.assertIn(value, self.text)
        self.assertIn('52b5dd8', self.text)
        self.assertIn('receipt', self.text)

    def test_new_figure_formats_hashes_and_zip(self):
        manifest = json.loads((FIG/'manifest.json').read_text())
        self.assertEqual(len(manifest), 21)
        self.assertEqual(len(list(FIG.glob('*.svg'))), 7)
        for name, digest in manifest.items():
            data = (FIG/name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), digest, name)
            if name.endswith('.png'):
                self.assertEqual(struct.unpack('>II',data[16:24]), (3840,2160))
            if name.endswith('.pdf'):
                self.assertTrue(data.startswith(b'%PDF-'))
        with zipfile.ZipFile(FIG/'research-overview-figures.zip') as z:
            self.assertIsNone(z.testzip())
            self.assertEqual(len(z.namelist()), 22)
            for name in z.namelist():
                self.assertEqual(z.read(name),(FIG/name).read_bytes())

    def test_svg_structure_and_unimplemented_boundary(self):
        ns = {'s':'http://www.w3.org/2000/svg'}
        for path in FIG.glob('*.svg'):
            svg = ET.parse(path).getroot()
            self.assertEqual(svg.get('role'),'img')
            self.assertIsNotNone(svg.find('s:title',ns))
            self.assertIsNotNone(svg.find('s:desc',ns))
            self.assertEqual(len(svg.findall('.//s:g[@data-block]',ns)),6)
            self.assertFalse(svg.findall('.//s:image',ns))
            ids = [n.get('id') for n in svg.iter() if n.get('id')]
            self.assertEqual(len(ids),len(set(ids)))
        for name in ('perception-tracking','appearance-rendering'):
            self.assertIn('stroke-dasharray', (FIG/(name+'-block-diagram.svg')).read_text())

    def test_old_hash_pinned_packages_and_complete_detail_preserved(self):
        for name, digest in {
            'docs/assets/paper/manifest.json':'6941d5497f6d3c20879ad85d215375d5b21c5bca09fa23b3993efb729ca20b5d',
            'docs/assets/paper/motar-paper-block-diagrams.zip':'c2698cd6278026696585c394b741fd7f2be8fd51db6ab3661bcf8d80d8eb157c',
            'docs/status/archive-2026-09-13.html':'a96982824b82293172b73ac226ade7396b8773c62366c21e32a5f4694accf2bb'
        }.items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest)
        self.assertIn('archive-2026-09-13.html',self.text)


if __name__ == '__main__':
    unittest.main()
