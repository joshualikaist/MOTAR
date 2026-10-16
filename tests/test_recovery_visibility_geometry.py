"""Recovery probe reference geometry and isolated actual Warp CPU mesh checks; no simulator."""
import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "aerial_gym/task/navrl_task/recovery_visibility_probe.py"
SPEC = importlib.util.spec_from_file_location("_recovery_visibility_test_module", SOURCE)
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def detector(**overrides):
    values = dict(num_envs=1, target_use_oriented_box=False, detect_decoupled=False,
                  target_render_mode="analytic_flat", _dynamic_mesh_treatment=None,
                  mesh_ids=SimpleNamespace(shape=(1,), device="cpu"),
                  _origins=[[0., 0., 0.]], _orientations=[[0., 0., 0., 1.]],
                  _targets=[[4., 0., 0.]], target_mask=[[[1, 0], [0, 0]]],
                  hfov=math.radians(87), vfov=math.radians(58), max_range=20.)
    values.update(overrides)
    return SimpleNamespace(**values)


class ReferenceGeometryTest(unittest.TestCase):
    def test_actual_camera_pose_and_yaw_quaternion(self):
        s = math.sqrt(.5)
        row = probe.camera_center_geometry([1, 2, 3], [0, 0, s, s], [1, 7, 3],
                                           math.radians(87), math.radians(58), 12.)
        self.assertTrue(row["target_center_in_fov"])
        self.assertTrue(row["target_center_in_range"])
        self.assertAlmostEqual(row["center_range_m"], 5.)
        reversed_row = probe.camera_center_geometry([1, 2, 3], [0, 0, -s, s], [1, 7, 3],
                                                    math.radians(87), math.radians(58), 12.)
        self.assertFalse(reversed_row["target_center_in_fov"])

    def test_fov_range_and_vertical_exit_are_independent(self):
        def row(point):
            return probe.camera_center_geometry([0, 0, 0], [0, 0, 0, 1], point, 1., 1., 12.)
        self.assertFalse(row([-1, 0, 0])["target_center_in_fov"])
        self.assertTrue(row([-1, 0, 0])["target_center_in_range"])
        self.assertTrue(row([15, 0, 0])["target_center_in_fov"])
        self.assertFalse(row([15, 0, 0])["target_center_in_range"])
        self.assertFalse(row([2, 0, 4])["target_center_in_fov"])
        self.assertFalse(row([0, 0, 0])["target_center_in_range"])

    def test_nonunit_or_nonfinite_geometry_is_rejected(self):
        for quaternion in ([0, 0, 0, 2], [0, 0, math.nan, 1]):
            with self.assertRaises(ValueError):
                probe.camera_center_geometry([0, 0, 0], quaternion, [2, 0, 0], 1., 1., 12.)

    def test_renderer_fov_scale_is_not_replaced_with_nominal_intrinsics(self):
        d = detector(hfov=1., vfov=1., fov_scale_err=.5)
        actual = probe.rendered_frustum(d)
        point = [4., 4. * math.tan(.6), 0.]
        self.assertFalse(probe.camera_center_geometry([0, 0, 0], [0, 0, 0, 1], point,
                                                     1., 1., 20.)["target_center_in_fov"])
        self.assertTrue(probe.camera_center_geometry([0, 0, 0], [0, 0, 0, 1], point,
                                                    *actual)["target_center_in_fov"])

    def test_explicit_triangle_mesh_front_behind_and_clear(self):
        vertices = [(2, -1, -1), (2, 1, -1), (2, 1, 1), (2, -1, 1)]
        triangles = [0, 1, 2, 0, 2, 3]
        self.assertTrue(probe.reference_mesh_blocked([0, 0, 0], [4, 0, 0], vertices, triangles))
        self.assertFalse(probe.reference_mesh_blocked([0, 0, 0], [1, 0, 0], vertices, triangles))
        self.assertFalse(probe.reference_mesh_blocked([0, 2, 0], [4, 2, 0], vertices, triangles))
        self.assertTrue(probe.reference_aabb_blocked([0, 0, 0], [4, 0, 0], [2, -1, -1], [2.1, 1, 1]))
        self.assertFalse(probe.reference_aabb_blocked([0, 2, 0], [4, 2, 0], [2, -1, -1], [2.1, 1, 1]))

    def test_triangle_edge_grazing_and_parallel_do_not_use_aabb_as_oracle(self):
        a, b, c = [2, -1, -1], [2, 1, -1], [2, 0, 1]
        self.assertAlmostEqual(probe.segment_triangle_intersection([0, 0, 0], [4, 0, 0], a, b, c), .5)
        self.assertIsNone(probe.segment_triangle_intersection([0, 0, 0], [0, 1, 0], a, b, c))
        tree = ast.parse(SOURCE.read_text())
        snapshot = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                        and node.name == "snapshot")
        self.assertNotIn("reference_aabb_blocked", ast.get_source_segment(SOURCE.read_text(), snapshot))


class ProbeContractsTest(unittest.TestCase):
    def test_off_import_and_factory_initialize_nothing(self):
        script = ("import importlib.util,sys; "
                  "s=importlib.util.spec_from_file_location('independent_probe',%r); "
                  "m=importlib.util.module_from_spec(s);s.loader.exec_module(m); "
                  "assert m.build_visibility_probe(object()) is None; "
                  "assert not any(x in sys.modules for x in ('torch','warp','isaacgym','aerial_gym'))") % str(SOURCE)
        subprocess.run([sys.executable, "-B", "-c", script], check=True, timeout=30)

    def test_unknown_geometry_does_not_initialize_backend(self):
        cases = [(dict(target_use_oriented_box=True), {}), (dict(detect_decoupled=True), {}),
                 (dict(target_render_mode="mesh_shaded"), {}),
                 (dict(_dynamic_mesh_treatment=object()), {}), ({}, dict(target_dynamics="physical")),
                 ({}, dict(scene_contract="moving_assets"))]
        with mock.patch.object(probe, "_load_warp_kernel", side_effect=AssertionError("backend loaded")):
            for changes, options in cases:
                obj = probe.build_visibility_probe(detector(**changes), enabled=True, **options)
                row = obj.snapshot(("episode", 0), render_token=("episode", 0))[0]
                self.assertEqual(row["status"], "UNKNOWN")
                self.assertIsNone(row["center_los_blocked"])
                self.assertEqual(row["full_visual_occlusion"], "NOT_RECORDED")

    def test_missing_or_stale_tokens_are_unknown_without_probe_or_rng(self):
        obj = probe.build_visibility_probe(detector(), enabled=True)
        state = random.getstate()
        with mock.patch.object(probe, "_load_warp_kernel", side_effect=AssertionError("backend loaded")):
            for token, rendered in ((None, None), (("e", 1), None), (("e", 1), ("e", 0))):
                row = obj.snapshot(token, render_token=rendered)[0]
                self.assertEqual(row["reason"], "MISSING_OR_MISMATCHED_RENDER_TOKEN")
                self.assertEqual(row["sensor_capture_time_s"], "NOT_RECORDED")
        self.assertEqual(state, random.getstate())

    def test_invalidate_clears_only_owned_cache(self):
        d = detector()
        obj = probe.build_visibility_probe(d, enabled=True)
        obj._last_token, obj._last_rows = ("e", 0), [{"marker": "old"}]
        before = dict(d.__dict__)
        obj.invalidate()
        self.assertIsNone(obj._last_token)
        self.assertIsNone(obj._last_rows)
        self.assertEqual(d.__dict__, before)
        with mock.patch.object(probe, "_load_warp_kernel", side_effect=AssertionError("backend loaded")):
            self.assertEqual(obj.snapshot(("e", 0), render_token=("e", 0))[0]["reason"],
                             "INVALIDATED_RENDER_TOKEN")


class ActualWarpCpuMeshTest(unittest.TestCase):
    def test_cpu_kernel_matches_independent_mesh_reference_and_preserves_inputs(self):
        if importlib.util.find_spec("warp") is None:
            self.skipTest("NOT_TESTED: Warp unavailable for isolated CPU mesh query")
        script = r'''
import importlib.util, json, math, os, random, sys
from types import SimpleNamespace
import numpy as np
import torch
import warp as wp
wp.config.kernel_cache_dir = sys.argv[2]
wp.init()
spec=importlib.util.spec_from_file_location("probe_cpu_mesh",sys.argv[1])
p=importlib.util.module_from_spec(spec);sys.modules[spec.name]=p;spec.loader.exec_module(p)
vertices=[(2,-1,-1),(2,1,-1),(2,1,1),(2,-1,1)]
indices=[0,1,2,0,2,3]
points=wp.array(vertices,dtype=wp.vec3,device="cpu")
triangles=wp.array(indices,dtype=wp.int32,device="cpu")
mesh=wp.Mesh(points=points,indices=triangles)
origins=torch.tensor([[0.,0.,0.],[0.,0.,0.],[0.,2.,0.]])
targets=torch.tensor([[4.,0.,0.],[1.,0.,0.],[4.,2.,0.]])
quats=torch.tensor([[0.,0.,0.,1.]]*3)
mask=torch.tensor([[[1,0],[0,0]],[[0,0],[0,0]],[[0,1],[0,1]]],dtype=torch.int32)
d=SimpleNamespace(num_envs=3,target_use_oriented_box=False,detect_decoupled=False,
 target_render_mode="analytic_flat",_dynamic_mesh_treatment=None,
 mesh_ids=wp.array([mesh.id]*3,dtype=wp.uint64,device="cpu"),_origins=origins,
 _orientations=quats,_targets=targets,target_mask=mask,hfov=math.radians(87),
 vfov=math.radians(58),max_range=20.)
before=[x.clone() for x in (origins,targets,quats,mask)]
rng=torch.random.get_rng_state().clone();python_rng=random.getstate();numpy_rng=np.random.get_state()
obj=p.build_visibility_probe(d,enabled=True)
rows=obj.snapshot(("e",0),render_token=("e",0))
expected=[p.reference_mesh_blocked(a,b,vertices,indices) for a,b in zip(origins.tolist(),targets.tolist())]
assert [r["center_los_blocked"] for r in rows]==expected==[True,False,False],rows
assert [r["rendered_target_visible_pixels"] for r in rows]==[1,0,2]
assert rows[0]["full_visual_occlusion"]=="NOT_RECORDED" # blocked centre with visible silhouette support
assert rows[1]["center_los_blocked"] is False and rows[1]["rendered_target_visible_pixels"]==0
assert rows[0]["camera_origin_world"]==origins[0].tolist()
assert all(torch.equal(a,b) for a,b in zip(before,(origins,targets,quats,mask)))
assert torch.equal(rng,torch.random.get_rng_state()) and python_rng==random.getstate()
after=np.random.get_state();assert numpy_rng[0]==after[0] and np.array_equal(numpy_rng[1],after[1]) and numpy_rng[2:]==after[2:]
rows[0]["center_los_blocked"]=False
assert obj.snapshot(("e",0),render_token=("e",0))[0]["center_los_blocked"] is True
obj.invalidate()
assert obj.snapshot(("e",1),render_token=("e",0))[0]["status"]=="UNKNOWN"
assert 'isaacgym' not in sys.modules and 'aerial_gym' not in sys.modules
print("RECOVERY_VISIBILITY_WARP_CPU_PASS")
'''
        with tempfile.TemporaryDirectory(prefix="motar-recovery-warp-cpu-") as cache:
            env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
            done = subprocess.run([sys.executable, "-B", "-c", script, str(SOURCE), cache],
                                  env=env, capture_output=True, text=True, timeout=45)
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertIn("RECOVERY_VISIBILITY_WARP_CPU_PASS", done.stdout)


if __name__ == "__main__":
    unittest.main()
