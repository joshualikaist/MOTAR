"""Focused P03 selected-geometry instrumentation tests (actual Warp CPU rays)."""
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tests')]
import test_h_r3_env_isolation as oracle
from h_r3_p03_isolation import AuditIsolation, RuntimeIsolation, attach


class P03IsolationQualification(unittest.TestCase):
    def test_runtime_audit_exact_geometry_ray_and_reset_parity(self):
        pairs = [(oracle.fixture(), mode) for mode in (AuditIsolation, RuntimeIsolation)]
        records = [mode(w, 3) for w, mode in pairs]
        for w, _ in pairs:
            w.unfolded_env_vec_root_tensor[1, 3:7] = torch.tensor([.01, .02, .03, .999])
            w.reset_idx(torch.arange(3))
            w.unfolded_env_vec_root_tensor[1, 1] = 4
            w.unfolded_env_vec_root_tensor[1, 3:7] = torch.tensor([0., 0., np.sin(.21), np.cos(.21)])
            w.reset_idx(torch.tensor([0, 2]))
        a, b = [w for w, _ in pairs]
        self.assertTrue(torch.equal(a.vertex_maps_per_env_updated.view(torch.int32),
                                    b.vertex_maps_per_env_updated.view(torch.int32)))
        for env in range(3):
            origins = [[-2, 0, .1], [-2, 4, .1], [-2, .17, .21]]
            np.testing.assert_array_equal(oracle.query(a.warp_mesh_per_env[env], origins),
                                          oracle.query(b.warp_mesh_per_env[env], origins))
            world = oracle.independent_world(b, env)
            want = [oracle.oracle_ray(world, b.global_env_mesh_list[env].faces, o, [1, 0, 0])
                    for o in origins]
            np.testing.assert_allclose(oracle.query(b.warp_mesh_per_env[env], origins),
                                       want, atol=2e-6, rtol=0)
        for rec in records:
            self.assertEqual(rec.events[-1]['refit_env_ids'], [0, 2])
            self.assertEqual(rec.check_integrity()['status'], 'PASS')

    def test_unselected_root_drift_preserves_geometry_and_rays_in_both_modes(self):
        for mode in (AuditIsolation, RuntimeIsolation):
            w = oracle.fixture(); rec = mode(w, 3); w.reset_idx(torch.arange(3))
            before = w.vertex_maps_per_env_updated.clone()
            rays = oracle.query(w.warp_mesh_per_env[0], [[-2, 0, .1]])
            w.unfolded_env_vec_root_tensor[1, 1] = 4
            w.unfolded_env_vec_root_tensor[1, 3:7] = torch.tensor([.001, .002, .003, .9999])
            w.unfolded_env_vec_root_tensor[4, 0] = 6
            w.reset_idx(torch.tensor([1]))
            self.assertTrue(torch.equal(before[:16].view(torch.int32),
                                        w.vertex_maps_per_env_updated[:16].view(torch.int32)))
            np.testing.assert_array_equal(rays, oracle.query(w.warp_mesh_per_env[0], [[-2, 0, .1]]))
            self.assertTrue(rec.events[-1]['unselected_vertices_bitwise_preserved'])

    def test_runtime_guard_rejects_value_and_signed_zero_mutations(self):
        for mode in (AuditIsolation, RuntimeIsolation):
            for mutation in ('vertex', 'signed_zero', 'mapping', 'original', 'faces'):
                w = oracle.fixture(); rec = mode(w, 3)
                if mutation == 'vertex': w.vertex_maps_per_env_updated[0, 0] += 1
                if mutation == 'signed_zero': w.vertex_maps_per_env_updated[0, 0] = -0.
                if mutation == 'mapping': w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR[0] = 2
                if mutation == 'original': w.VERTEX_MAPS_PER_ENV_ORIGINAL[0, 0] += 1
                if mutation == 'faces':
                    w.warp_mesh_per_env[0].indices.assign(np.ones_like(w.warp_mesh_per_env[0].indices.numpy()))
                with self.assertRaises(RuntimeError, msg=(mode.__name__, mutation)):
                    rec.check_integrity()

    def test_storage_mesh_mapping_and_layout_mutations_fail_closed(self):
        for mutation in ('storage', 'data', 'meshes', 'ids', 'count', 'num_envs'):
            w = oracle.fixture(); rec = RuntimeIsolation(w, 3)
            if mutation == 'storage': w.vertex_maps_per_env_updated = w.vertex_maps_per_env_updated.clone()
            if mutation == 'data': w.vertex_maps_per_env_updated.data = w.vertex_maps_per_env_updated.clone()
            if mutation == 'meshes': w.warp_mesh_per_env.reverse()
            if mutation == 'ids': w.CONST_WARP_MESH_ID_LIST.reverse()
            if mutation == 'count': w.global_vertex_counter += 1
            if mutation == 'num_envs': w.num_envs += 1
            with self.assertRaises(RuntimeError, msg=mutation): rec.check_integrity()

    def test_unselected_write_during_transform_is_rejected_after_reset(self):
        for mode in (AuditIsolation, RuntimeIsolation):
            w = oracle.fixture(); rec = mode(w, 3); w.reset_idx(torch.arange(3))
            transform = rec._transform
            def bad_transform(*args):
                w.vertex_maps_per_env_updated[0, 0] += 1
                return transform(*args)
            rec._transform = bad_transform
            with self.assertRaises(RuntimeError): w.reset_idx(torch.tensor([1]))

    def test_nonfinite_selected_geometry_is_not_accepted_as_matching_nan_bits(self):
        for mode in (AuditIsolation, RuntimeIsolation):
            w = oracle.fixture(); rec = mode(w, 3)
            w.unfolded_env_vec_root_tensor[1, 0] = float('nan')
            with self.assertRaises(RuntimeError): w.reset_idx(torch.tensor([0]))

    def test_runtime_has_no_full_vertex_downloads_after_installation(self):
        w = oracle.fixture(); rec = RuntimeIsolation(w, 3)
        base = sys.modules[RuntimeIsolation.__mro__[2].__module__]
        real_array = base._array
        def forbid_geometry(value):
            self.assertIsNot(value, w.vertex_maps_per_env_updated)
            self.assertIsNot(value, w.VERTEX_MAPS_PER_ENV_ORIGINAL)
            self.assertIsNot(value, w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR)
            return real_array(value)
        with mock.patch.object(base, '_array', side_effect=forbid_geometry), \
                mock.patch.object(base, '_sha', side_effect=AssertionError('runtime SHA forbidden')):
            w.reset_idx(torch.arange(3))
            w.reset_idx(torch.tensor([1]))
            rec.check_integrity()
        self.assertNotIn('vertices', rec.events[-1])

    def test_six_hundred_cpu_steps_do_not_substitute_for_gpu_qualification(self):
        w = oracle.fixture(); rec = RuntimeIsolation(w, 3); w.reset_idx(torch.arange(3))
        for step in range(600):
            w.unfolded_env_vec_root_tensor[1, 0] += 1e-6
            if step % 17 == 0:
                env = (step // 17) % 3
                w.reset_idx(torch.tensor([env]))
            rec.check_integrity()
        self.assertEqual(len(rec.events), 37)
        self.assertGreaterEqual(rec.integrity_checks, 600)

    def test_bad_ids_fail_before_update_and_close_restores_instance(self):
        for mode in (AuditIsolation, RuntimeIsolation):
            w = oracle.fixture(); original = w.reset_idx; rec = mode(w, 3)
            for ids in ([-1], [3], [0, 0], [.5]):
                with self.assertRaises(ValueError): w.reset_idx(ids)
            self.assertEqual(rec.events, [])
            rec.close(); self.assertIs(w.reset_idx, original)

    def test_attach_modes_and_explicit_diagnostic_snapshot(self):
        for name in ('audit', 'runtime'):
            w = oracle.fixture()
            task = types.SimpleNamespace(sim_env=types.SimpleNamespace(
                warp_env=w, IGE_env=types.SimpleNamespace(num_assets_per_env=3)))
            rec = attach(task, name); w.reset_idx(torch.arange(3))
            snap = rec.snapshot()
            self.assertEqual(snap['vertices'].shape, (29, 3))
            self.assertEqual(snap['vertex_ranges'].tolist(), [[0, 16], [16, 24], [24, 29]])
            self.assertEqual(rec.instrumentation_mode, name)
            self.assertEqual(rec.check_integrity()['status'], 'PASS')
            with self.assertRaises(RuntimeError): attach(task, name)
        with self.assertRaises(ValueError): attach(task, 'legacy')


if __name__ == '__main__':
    unittest.main()
