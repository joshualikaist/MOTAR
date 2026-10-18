"""Independent hand checks before S-L1C v2 candidate selection or production work."""
import importlib.util
import math
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('_cv2ref',ROOT/'tools/audits/s_l1c_v2_reference.py')
ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(ref)

class SphereHypothesisTest(unittest.TestCase):
    def test_two_row_sphere_is_rejected_by_v1_but_geometry_accepts(self):
        for r in (1.16,1.5,2.,2.28):
            target=r*ref.direction(0,5)
            scan,labels=ref.v1.lidar_scan(sphere_center=target)
            old=ref.associate(scan,target,'V2-0')
            new=ref.associate(scan,target,'V2-A')
            self.assertFalse(old['valid'],r)
            self.assertTrue(new['valid'],(r,new))
            self.assertEqual(labels[new['row'],new['column']],ref.v1.ref.SPHERE)
            self.assertAlmostEqual(new['centre_range'],r,places=10)

    def test_radial_prediction_bias_does_not_become_measured_range(self):
        target=1.5*ref.direction(20,5);scan,_=ref.v1.lidar_scan(sphere_center=target)
        for offset in (-.15,0,.15):
            out=ref.associate(scan,target*(1+offset/1.5),'V2-A')
            self.assertTrue(out['valid'],out)
            self.assertAlmostEqual(out['centre_range'],1.5,places=10)

    def test_vertical_surface_has_zero_vertical_hypothesis_loss(self):
        scan,_=ref.v1.lidar_scan([[1.4,0,.5]],[[.3,.3,1.5]])
        prediction=1.3*ref.direction(0,5)
        for candidate in ref.CANDIDATES[:3]:
            self.assertFalse(ref.associate(scan,prediction,candidate)['valid'])

    def test_large_noise_is_explicit_abstention_not_forced_match(self):
        target=1.5*ref.direction(0,5);scan,_=ref.v1.lidar_scan(sphere_center=target)
        out=ref.associate(scan,target,'V2-A',noise_bound_m=.03)
        self.assertFalse(out['valid'])
        self.assertTrue(out['decision'].startswith('ABSTAIN_'))

    def test_azimuth_candidate_cannot_assume_narrow_sphere_at_near_range(self):
        target=1.5*ref.direction(0,5);scan,_=ref.v1.lidar_scan(sphere_center=target)
        out=ref.associate(scan,target,'V2-B')
        self.assertFalse(out['valid']);self.assertEqual(out['decision'],'ABSTAIN_AZIMUTH')

    def test_existing_acceptances_keep_identical_range_and_row(self):
        for r in (1.5,3,6):
            target=np.array([r,0,0]);scan,_=ref.v1.lidar_scan(sphere_center=target)
            old=ref.associate(scan,target,'V2-0');self.assertTrue(old['valid'])
            for candidate in ref.CANDIDATES[1:]:
                new=ref.associate(scan,target,candidate)
                self.assertEqual(new,old)

    def test_camera_candidate_is_identity_unconfirmed_and_not_a_selection(self):
        target=1.5*ref.direction(0,5);scan,_=ref.v1.lidar_scan(sphere_center=target)
        out=ref.associate(scan,target,'V2-C')
        self.assertTrue(out['valid']);self.assertIn('CONTRACT_INELIGIBLE',out['decision'])

    def test_invalid_or_no_return_does_not_add_acceptance(self):
        for value in (12.,np.nan,np.inf):
            scan=np.full((4,72),value)
            self.assertFalse(ref.associate(scan,np.array([1.5,0,.1]),'V2-A')['valid'])
        with self.assertRaises(ValueError):ref.associate(np.zeros((3,72)),np.zeros(3))
        with self.assertRaises(ValueError):ref.associate(np.ones((4,72)),np.zeros(3),noise_bound_m=-1)

    def test_noise_does_not_invent_returns_or_mutate_inputs(self):
        scan=np.full((4,72),12.);scan[0,0]=2.
        copy=scan.copy();a=ref.perturb_scan(scan,.01,1);b=ref.perturb_scan(scan,.01,-1)
        np.testing.assert_array_equal(scan,copy)
        np.testing.assert_array_equal(a[scan==12],12.)
        self.assertAlmostEqual(a[0,0]+b[0,0],4.)

    def test_selection_requires_all_gates_and_never_selects_clock_candidate(self):
        import sys
        sys.path.insert(0,str(ROOT/'tools'))
        from s_l1c_v2_cpu import select_candidate
        gates={c:dict(recall=True,negatives=True) for c in ref.CANDIDATES}
        scores={c:dict(new_true=10,false_accepts=0,new_range_mae_m=0.) for c in ref.CANDIDATES}
        self.assertEqual(select_candidate(gates,scores),'V2-A')
        gates['V2-A']['negatives']=False
        self.assertEqual(select_candidate(gates,scores),'V2-B')
        gates['V2-B']['recall']=False
        self.assertIsNone(select_candidate(gates,scores))

if __name__=='__main__':unittest.main()
