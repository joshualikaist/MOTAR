import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import h_r3_p02_planning as p
class PlanningTest(unittest.TestCase):
    def test_larger_effect_requires_fewer_effective_events(self):
        self.assertGreater(p.paired_effective_n(.1,.4),p.paired_effective_n(.2,.4))
        self.assertGreater(p.paired_effective_n(.15,.6),p.paired_effective_n(.15,.2))
    def test_cluster_design_effect_and_no_execution(self):
        d=p.plan({'primary':{'per_seed':{'4601':{'n':0},'4602':{'n':7}}}})
        self.assertEqual(d['status'],'PLAN_ONLY_NOT_AUTHORIZED');self.assertEqual(d['seed_clusters'],2)
        group=[r for r in d['grid'] if r['effect_pp']==15 and r['discordance']==.4 and r['events_per_seed']==20]
        self.assertGreater(group[-1]['seed_count_approx'],group[0]['seed_count_approx'])
        self.assertEqual(d['observed_yield'][0]['geometric_events'],0)
    def test_invalid_assumptions_fail(self):
        with self.assertRaises(ValueError):p.paired_effective_n(.5,.2)
if __name__=='__main__':unittest.main()
