"""Current publication site: independent source/figure checks, no research runtime.

Historical layout assertions remain on the explicitly archived evidence page.
"""
import csv
import hashlib
from html.parser import HTMLParser
import json
import math
from pathlib import Path
import struct
import unittest
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET
import zipfile

ROOT=Path(__file__).resolve().parents[1]
SITE=ROOT/'docs/status'
FIG=ROOT/'docs/assets/paper/redesign-2026-10-22'
def record(path):return json.loads((ROOT/path).read_text())

class Page(HTMLParser):
    def __init__(self,text):
        super().__init__();self.ids=[];self.links=[];self.images=[];self.headings=0;self.feed(text)
    def handle_starttag(self,tag,attrs):
        a=dict(attrs)
        if 'id' in a:self.ids.append(a['id'])
        for k in ['href','src']:
            if k in a:self.links.append(a[k])
        if tag=='img':self.images.append(a)
        if tag=='h1':self.headings+=1

class ProjectPageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pages={p:(SITE/p).read_text() for p in ['index.html','evidence.html','figures.html']}
        cls.m=record('docs/assets/paper/redesign-2026-10-22/manifest.json')

    def test_accessibility_links_and_unique_anchors(self):
        for name,text in self.pages.items():
            p=Page(text);self.assertEqual(p.headings,1,name);self.assertEqual(len(p.ids),len(set(p.ids)),name)
            self.assertIn('lang="en"',text);self.assertIn('href="#main"',text)
            for raw in p.links:
                u=urlsplit(raw)
                if u.scheme or u.netloc:continue
                target=(SITE/unquote(u.path)).resolve() if u.path else SITE/name
                self.assertIn(ROOT/'docs',target.parents,(name,raw));self.assertTrue(target.exists(),(name,raw))
                if u.fragment and target.suffix=='.html':self.assertIn(unquote(u.fragment),Page(target.read_text()).ids,(name,raw))
            for image in p.images:self.assertGreater(len(image.get('alt','')),40)

    def test_legacy_illustrations_are_replaced_on_current_pages(self):
        for name in ['index.html','figures.html']:
            p=Page(self.pages[name]);self.assertEqual(len(p.images),6,name)
            self.assertEqual([x['src'].split('/')[-1] for x in p.images],[f['id']+'.svg' for f in self.m['figures']])
            for old in ['../assets/paper/final/','../assets/paper/overview-2026-09-13/']:self.assertNotIn(old,self.pages[name])

    def test_source_sha_bindings(self):
        for s in self.m['sources'].values():self.assertEqual(hashlib.sha256((ROOT/s['path']).read_bytes()).hexdigest(),s['sha256'],s['path'])

    def test_vector_pdf_raster_and_bundle(self):
        self.assertEqual(len(self.m['figures']),6)
        for f in self.m['figures']:
            self.assertEqual(f['width_in'],7.2)
            for ext in ['svg','pdf','png']:
                name=f['id']+'.'+ext;data=(FIG/name).read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(),self.m['files'][name])
                if ext=='pdf':self.assertTrue(data.startswith(b'%PDF-'));self.assertIn(b'/FontFile2',data)
                if ext=='png':self.assertEqual(struct.unpack('>II',data[16:24]),(2160,round(300*f['height_in'])))
                if ext=='svg':
                    svg=ET.fromstring(data);self.assertEqual(svg.attrib['role'],'img')
                    self.assertTrue(svg.findall('.//{http://www.w3.org/2000/svg}text'))
                    self.assertTrue(svg.find('{http://www.w3.org/2000/svg}title').text)
        with zipfile.ZipFile(FIG/'motar-publication-figures.zip') as z:
            for n in list(self.m['files'])+['manifest.json']:self.assertEqual(z.read(n),(FIG/n).read_bytes())

    def test_all_112_density_rates_match_original_counts(self):
        with (ROOT/'results/target_motion_replication_7seed/cells.csv').open() as f:rows=list(csv.DictReader(f))
        v=self.m['values'];self.assertEqual(len(rows),112)
        self.assertEqual(v['episodes'],sum(int(r['primary_episodes']) for r in rows))
        for r in rows:
            x=v['density_capture_pct'][r['arm']][v['seeds'].index(int(r['seed']))][v['densities'].index(int(r['bars']))]
            self.assertAlmostEqual(x,100*int(r['primary_capture'])/int(r['primary_episodes']),places=12)
        self.assertIn('No added CI or hypothesis test',self.m['density_display'])

    def test_recorded_motion_intervals(self):
        s=record('results/target_motion_replication_7seed/analysis.json')
        self.assertEqual(self.m['values']['motion_primary'],s['estimators']['quota']['primary'])
        self.assertEqual(self.m['values']['seeds'],list(range(4104,4111)))
        self.assertNotIn('min_relative_distance_m',self.m['values'])

    def test_seed_safety_estimate_matches_its_interval(self):
        s=record('results/independent_verification_2026-09-07/recomputed.json')['contrasts']['dwa_arc-riskcap']
        self.assertEqual(self.m['values']['safety_seed'],s['seed_t']);self.assertEqual(self.m['values']['safety_pooled'],s['pooled'])
        self.assertNotEqual(s['seed_t']['mean'],s['pooled']['delta']);self.assertEqual(s['seed_t']['df'],2)
        self.assertIn('seed-t interval on the seed mean',self.pages['evidence.html'])

    def test_sensitivity_sources_and_replication_units(self):
        p=record('results/perception_p10_seed_replication_2026-09-10/summary.json')
        self.assertEqual(self.m['values']['readaptation'],p['R2_seed_level_t'])
        self.assertEqual(self.m['values']['frozen_cost'],p['per_training_seed']['811']['p9_cost_on_source'])
        self.assertEqual(self.m['values']['observation'],record('results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/summary.json')['primary'])
        for word in ['two evaluation seeds','Three training seeds','Policy R']:self.assertIn(word,self.pages['evidence.html'])

    def test_real_footage_values_and_nonindependent_frames(self):
        s=record('results/eth_ds5_e3s_2026-09-10/run/e3s_result.json');v=self.m['values']['e3s']
        self.assertEqual(v['frames'],s['data_quality']['included_frames'])
        self.assertEqual(v['median_pct'],100*s['primary']['gates']['G2_central']['median_of_block_medians'])
        self.assertIn('Frames are not independent replicates',self.pages['evidence.html'])

    def test_actor_dimensions_and_embedding_order(self):
        g=self.m['graph'];fields=g['actor_fields'];self.assertEqual(sum(x['flat'] for x in fields),898)
        for f in fields:self.assertEqual(math.prod(f['shape']),f['flat'])
        self.assertEqual(sum(x['tokens'] for x in fields)+1,17)
        self.assertIn(['flat898','tokens17'],g['baseline_edges']);self.assertNotIn(['tokens17','flat898'],g['baseline_edges'])
        self.assertEqual(fields[0]['name'],'Fused static scan');self.assertEqual(g['baseline_governor'],'riskcap')
        self.assertIn('overwritten',g['vertical_action']);self.assertEqual(g['tracker_expiry_s'],5.)

    def test_gt_firewall_and_shadow_boundary(self):
        g=self.m['graph'];self.assertFalse(g['boia_applies_action'])
        self.assertFalse(any('gt' in p.lower() for edge in g['baseline_edges'] for p in edge))
        for name in ['index.html','evidence.html']:
            for word in ['ORACLE_TRUE_POSE_RASTER','NOT VALIDATED','NOT CONFIRMED','G5 HOLD','shadow-only','Independent review FAIL']:self.assertIn(word,self.pages[name],name)
        self.assertIn('previous actor action, with XY updated after the governor',self.pages['index.html'])
        self.assertNotIn('previous applied-action information',self.pages['index.html'])

    def test_browser_gt_appendix_is_separate(self):
        demo=self.pages['index.html'].split('<section class="demo" id="demo"')[1]
        for phrase in ['BROWSER GT PREVIEW','NOT PPO','NOT PHYSX','None of the results above depend on it']:self.assertIn(phrase,demo)
        self.assertNotIn('id="arena"',self.pages['index.html'].split('<section class="demo"')[0])

    def test_visual_accessibility_rules(self):
        css=(SITE/'publication.css').read_text()
        for x in ['prefers-reduced-motion',':focus-visible','overflow:auto','max-width:600px']:self.assertIn(x,css)
        for x in ['@import','fonts.googleapis','@font-face','gradient(','box-shadow:']:self.assertNotIn(x,css)

if __name__=='__main__':unittest.main()
