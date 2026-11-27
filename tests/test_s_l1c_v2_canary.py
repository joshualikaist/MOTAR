"""CPU checks of the fixed canary matrix, scene geometry and independent covariance metric."""
import ast
import itertools
import json
import math
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import s_l1c_v2_run as run
import s_l1c_v2_analysis as analysis


def scene_functions():
    tree=ast.parse((ROOT/'tools/s_l1c_v2_launcher.py').read_text())
    nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('direction','scenes')]
    namespace=dict(np=np,math=math,itertools=itertools)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),'canary_scenes','exec'),namespace)
    return namespace['scenes']


class CanaryContractTest(unittest.TestCase):
    def test_exact_matrix_and_does_not_enable_d_for_v2(self):
        p=run.plan();specs=run.specs(p)
        self.assertEqual(len(specs),23);self.assertEqual(len({x['id'] for x in specs}),23)
        self.assertEqual(sum(x['batch']=='SENSORS' for x in specs),8)
        for s in specs:
            if s['batch']=='V3':self.assertEqual(s['seed'],4501 if s['config']=='static' else 4502)
            if s.get('variant','').startswith('v2_'):self.assertEqual(s['lineage'],'s_l1c_v2')

    def test_positive_fixture_counts_distance_elevation_bearing(self):
        make=scene_functions();rows=[]
        for group in range(3):
            part=make(dict(kind='positive',group=group));self.assertEqual(len(part),48);rows+=part
        self.assertEqual(len(rows),144)
        self.assertEqual({x['r'] for x in rows},{1.16,1.5,2,2.28,2.3,3})
        self.assertEqual({x['el'] for x in rows},{-5,0,5,10,15,20})
        for s in rows:self.assertAlmostEqual(np.linalg.norm(s['target']),s['r'])

    def test_original_replays_and_new_narrow_foreground_are_distinct(self):
        rows=scene_functions()(dict(kind='negative',group=0));self.assertEqual(len(rows),31)
        self.assertEqual(sum(bool(s.get('replay')) for s in rows),7)
        bars=[s for s in rows if s['id'].startswith('BAR_')]
        foreground=[s for s in rows if s['id'].startswith('FG_')]
        self.assertEqual(len(bars),12);self.assertEqual(len(foreground),12)
        self.assertEqual(sum(abs(s['bars'][0]['half'][0]-.01)<1e-12 for s in bars),6)
        self.assertTrue(all(s['target'] is not None for s in foreground))

    def test_covariance_metric_measures_unobserved_axes(self):
        for direction in ([1,0,0],[1,1,1],[0,1,1]):
            before=np.eye(3);after=analysis.ref.v1.range_only_posterior(before,direction)
            ratios=analysis.covariance_ratios(before,after,direction)
            self.assertLess(ratios[0],.01);self.assertGreaterEqual(min(ratios[1:]),.99)

    def test_every_prior_has_committed_manifest_copy_and_digest_file(self):
        for name,(root,pattern,copy) in run.PARENTS.items():
            self.assertTrue((ROOT/copy).is_file(),name)
            manifest=json.loads((root/'RAW_MANIFEST.json').read_text())
            paths={x['path'] for x in manifest['members']}
            for config in ('static','moving'):self.assertIn(pattern.format(config=config)+'/v3_steps.jsonl',paths)

if __name__=='__main__':unittest.main()
