"""R2 CPU unit tests only: synthetic tensors and mocked Warp, not rendering experiments."""
import ast
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch, Mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from renderer_validation.scene import Camera, MeshScene, Appearance, box_fixture, sample_appearance
from renderer_validation.gbuffer import (finalize_gbuffer, validated_poses, checked_kernel_source,
                                         KERNEL_SHA256, WarpGBufferRenderer, load_camera_kernels)
from renderer_validation.shading import shade
from renderer_validation.validation import evaluate, geometry_equal, tensor_hash
import run_renderer_validation as runner


def buffers():
    scene, camera = box_fixture(), Camera(width=3, height=1)
    ranges = torch.tensor([[[2., 3., 1000.]]])
    normals = torch.tensor([[[[0., 0., -1.], [1., 0., 0.], [0., 0., 0.]]]])
    faces = torch.tensor([[[0, 6, -1]]], dtype=torch.int32)
    return scene, camera, ranges, normals, faces


def material(scene):
    return Appearance(np.ones((1, scene.material_count, 3)), np.ones((1, scene.material_count)),
                      [[0, 0, -1]], [0.2], [0.8])


class SceneContractTest(unittest.TestCase):
    def test_fixture_is_two_generic_boxes(self):
        s = box_fixture()
        self.assertEqual(s.vertices.shape, (16, 3))
        self.assertEqual(s.triangles.shape, (24, 3))
        self.assertEqual(s.material_count, 4)
        self.assertEqual(set(s.face_instance), {0, 1})

    def test_scene_owns_immutable_copies(self):
        s = box_fixture()
        vertices = s.vertices.copy()
        new = replace(s, vertices=vertices)
        vertices[:] = 0
        self.assertTrue(np.array_equal(new.vertices, s.vertices))
        with self.assertRaises(ValueError):
            new.vertices[0] = 0

    def test_reject_float_face_indices(self):
        s = box_fixture()
        with self.assertRaises(ValueError):
            replace(s, triangles=s.triangles.astype(float))

    def test_reject_out_of_bounds_indices(self):
        s = box_fixture()
        f = s.triangles.copy()
        f[0, 0] = len(s.vertices)
        with self.assertRaises(ValueError):
            replace(s, triangles=f)

    def test_reject_degenerate_triangles(self):
        s = box_fixture()
        f = s.triangles.copy()
        f[0, 0] = f[0, 1]
        with self.assertRaises(ValueError):
            replace(s, triangles=f)

    def test_reject_negative_or_missing_face_metadata(self):
        s = box_fixture()
        for v in (np.full(24, -1), np.zeros(23, dtype=int)):
            with self.assertRaises(ValueError):
                replace(s, face_material=v)

    def test_camera_rejects_bad_contract(self):
        for kw in ({"width": 0}, {"height": 2.5}, {"far_range_m": 1000},
                   {"far_range_m": float("nan")}, {"horizontal_fov_deg": 180}):
            with self.assertRaises(ValueError):
                Camera(**kw)

    def test_camera_projection_even_and_odd(self):
        c = Camera(width=4, height=2)
        self.assertEqual(float(c.optical_projection()[1, 2]), 1.0)
        odd = Camera(width=3, height=3)
        self.assertLess(float(odd.optical_projection().max()), 1.0)
        self.assertEqual(odd.inverse_intrinsics().shape, (4, 4))

    def test_appearance_seed_and_batch_prefix(self):
        a, b = sample_appearance(42, 2, 4), sample_appearance(42, 5, 4)
        self.assertTrue(np.array_equal(a.base_color, b.base_color[:2]))
        self.assertTrue(np.array_equal(a.light_direction, b.light_direction[:2]))
        self.assertTrue(np.array_equal(a.base_color, sample_appearance(42, 2, 4).base_color))
        self.assertFalse(np.array_equal(a.base_color[0], a.base_color[1]))

    def test_appearance_does_not_modify_global_rng(self):
        state = np.random.get_state()
        sample_appearance(2, 2, 3)
        later = np.random.get_state()
        self.assertTrue(np.array_equal(state[1], later[1]))
        self.assertEqual(state[2:], later[2:])

    def test_appearance_validation(self):
        a = material(box_fixture())
        for kw in ({"light_direction": [[0, 0, 0]]}, {"ambient": [-1]},
                   {"base_color": np.full((1, 4, 3), np.nan)},
                   {"kd": np.ones((1, 2))}, {"directional": [np.inf]}):
            with self.assertRaises(ValueError):
                replace(a, **kw)

    def test_pose_contract(self):
        p, q = validated_poses([[0, 0, 0]], [[0, 0, 0, 1]], 1)
        self.assertEqual(p.shape, (1, 3))
        self.assertEqual(q.shape, (1, 4))
        for quat in ([[0, 0, 0, 0]], [[1, 1, 1, 1]], [[np.nan, 0, 0, 1]]):
            with self.assertRaises(ValueError):
                validated_poses([[0, 0, 0]], quat, 1)


class GeometryAndShadingTest(unittest.TestCase):
    def setUp(self):
        self.scene, self.camera, self.ranges, self.normals, self.faces = buffers()
        self.g = finalize_gbuffer(self.ranges, self.normals, self.faces, self.scene, self.camera)
        self.a = material(self.scene)

    def test_face_zero_is_valid_and_miss_separate(self):
        self.assertEqual(self.g.valid.tolist(), [[[True, True, False]]])
        self.assertEqual(self.g.instance_id.tolist(), [[[0, 0, -1]]])
        self.assertEqual(self.g.range_m[0, 0, 2].item(), 0)
        self.assertEqual(self.g.normal_world[0, 0, 2].tolist(), [0, 0, 0])

    def test_depth_is_optical_projection_not_range(self):
        expected = self.ranges[..., :2] * torch.tensor(self.camera.optical_projection()[..., :2])
        self.assertTrue(torch.equal(self.g.depth_m[..., :2], expected))
        self.assertTrue((self.g.depth_m[..., :2] < self.g.range_m[..., :2]).all())

    def test_owned_outputs_not_overwritten_by_next_raw_frame(self):
        self.ranges.fill_(55)
        self.normals.zero_()
        self.faces.fill_(22)
        self.assertEqual(self.g.range_m[0, 0, 0].item(), 2)
        self.assertEqual(self.g.face_id[0, 0, 0].item(), 0)
        self.assertEqual(self.g.normal_world[0, 0, 0, 2].item(), -1)

    def test_two_pass_mismatch_rejected(self):
        self.ranges[0, 0, 0] = 1000
        with self.assertRaises(RuntimeError):
            finalize_gbuffer(self.ranges, self.normals, self.faces, self.scene, self.camera)

    def test_invalid_index_rejected_before_lookup(self):
        for index in (-2, 24):
            f = self.faces.clone()
            f[0, 0, 0] = index
            with self.assertRaises(ValueError):
                finalize_gbuffer(self.ranges, self.normals, f, self.scene, self.camera)

    def test_nonfinite_and_zero_hit_normals_rejected(self):
        for value in (0., float("nan")):
            normal = self.normals.clone()
            normal[0, 0, 0] = value
            with self.assertRaises((ValueError, RuntimeError)):
                finalize_gbuffer(self.ranges, normal, self.faces, self.scene, self.camera)

    def test_dtype_and_shape_rejected(self):
        for raw in (self.ranges.double(), self.ranges[..., :2]):
            with self.assertRaises(ValueError):
                finalize_gbuffer(raw, self.normals, self.faces, self.scene, self.camera)

    def test_flat_rgb_contract(self):
        rgb = shade(self.g, self.scene, self.a, "flat")
        self.assertEqual(tuple(rgb.shape), (1, 1, 3, 3))
        self.assertEqual(rgb.dtype, torch.float32)
        self.assertFalse(rgb.requires_grad)
        self.assertTrue(torch.equal(rgb[0, 0, :2], torch.ones(2, 3)))
        self.assertTrue(torch.equal(rgb[0, 0, 2], torch.zeros(3)))

    def test_lambertian_known_tensor_values(self):
        rgb = shade(self.g, self.scene, self.a)
        torch.testing.assert_close(rgb[0, 0, 0], torch.ones(3))
        torch.testing.assert_close(rgb[0, 0, 1], torch.full((3,), 0.2))

    def test_light_change_changes_only_rgb(self):
        before = self.g.depth_m.clone()
        a2 = replace(self.a, light_direction=[[1, 0, 0]])
        self.assertFalse(torch.equal(shade(self.g, self.scene, self.a), shade(self.g, self.scene, a2)))
        self.assertTrue(torch.equal(before, self.g.depth_m))

    def test_material_change_locality(self):
        colors = self.a.base_color.copy()
        colors[:, 0] *= 0.5
        other = replace(self.a, base_color=colors)
        a, b = shade(self.g, self.scene, self.a), shade(self.g, self.scene, other)
        self.assertFalse(torch.equal(a[..., 0, :], b[..., 0, :]))
        self.assertTrue(torch.equal(a[..., 1:, :], b[..., 1:, :]))

    def test_instance_id_renumbering_does_not_change_rgb(self):
        other = replace(self.scene, face_instance=self.scene.face_instance + 100)
        g = finalize_gbuffer(self.ranges, self.normals, self.faces, other, self.camera)
        self.assertFalse(torch.equal(g.instance_id, self.g.instance_id))
        self.assertTrue(torch.equal(shade(g, other, self.a), shade(self.g, self.scene, self.a)))

    def test_face_renumbering_with_tables_preserves_rgb(self):
        order = np.arange(23, -1, -1)
        other = MeshScene(self.scene.vertices, self.scene.triangles[order],
                          self.scene.face_material[order], self.scene.face_instance[order])
        face = torch.where(self.faces >= 0, 23 - self.faces, -1)
        g = finalize_gbuffer(self.ranges, self.normals, face, other, self.camera)
        self.assertTrue(torch.equal(shade(g, other, self.a), shade(self.g, self.scene, self.a)))

    def test_repeated_shading_exact_and_input_preserved(self):
        a = shade(self.g, self.scene, self.a)
        self.assertTrue(torch.equal(a, shade(self.g, self.scene, self.a)))
        self.assertEqual(self.a.light_direction[0, 2], -1)

    def test_bad_shading_parameters_rejected(self):
        for kw in ({"mode": "debug"}, {"background": (1, 0, 0, 1)}, {"background": (2, 0, 0)}):
            with self.assertRaises(ValueError):
                shade(self.g, self.scene, self.a, **kw)

    def test_r3_rule_passes_known_contract_tensor(self):
        # 4 pixels, two faces/instances and normals lit differently by the two fixed arms.
        camera = Camera(width=4, height=1)
        r = torch.full((1, 1, 4), 2.0)
        n = torch.tensor([[[[0., 0., -1.], [0., 0., -1.],
                            [1., 0., -1.], [1., 0., -1.]]]])
        f = torch.tensor([[[0, 1, 6, 7]]], dtype=torch.int32)
        g = finalize_gbuffer(r, n, f, self.scene, camera)
        # Formal minimum is 50 selected pixels, so a tiny fixture must fail only that check.
        result = evaluate(self.scene, g, g)
        self.assertEqual(result["run_verdict"], "FAIL")
        self.assertFalse(result["checks"]["selected_material_has_50_pixels"])
        self.assertTrue(result["checks"]["flat_variance_at_most_1e_12"])
        self.assertTrue(result["checks"]["rerender_geometry_exact"])
        self.assertEqual(tensor_hash(g.depth_m), result["array_sha256"]["depth"])

    def test_geometry_comparison_is_field_complete(self):
        self.assertTrue(geometry_equal(self.g, self.g))
        self.assertFalse(geometry_equal(self.g, replace(self.g, depth_m=self.g.depth_m + 1)))


class IsolationAndDriverTest(unittest.TestCase):
    def test_source_hash_and_dependency_boundary(self):
        self.assertEqual(hashlib.sha256(checked_kernel_source()).hexdigest(), KERNEL_SHA256)
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "drift.py"
            p.write_bytes(b"import warp as wp\n")
            with self.assertRaises(RuntimeError):
                checked_kernel_source(p)

    def test_source_loader_with_fake_warp_never_imports_package(self):
        wp = types.ModuleType("warp")
        wp.constant = lambda x: x
        wp.int32, wp.uint64 = int, int
        wp.vec3, wp.quat, wp.mat44 = object, object, object
        wp.array = lambda **kw: object
        wp.kernel = lambda f: f
        name = "_renderer_validation_camera_" + KERNEL_SHA256[:16]
        before = set(sys.modules)
        try:
            with patch.dict(sys.modules, {"warp": wp}):
                cls = load_camera_kernels()
                self.assertTrue(callable(cls.draw_optimized_kernel_normal_faceID))
                self.assertIs(cls, load_camera_kernels())
                self.assertFalse(any(n.startswith("aerial_gym") for n in set(sys.modules) - before))
        finally:
            sys.modules.pop(name, None)

    def test_warp_driver_dispatch_with_fake_kernel_outputs(self):
        wp = types.ModuleType("warp")
        wp.init = Mock()
        wp.vec3, wp.quat, wp.int32, wp.uint64, wp.float32 = object, object, object, object, object
        wp.from_torch = lambda tensor, **kw: tensor
        wp.array = lambda values, **kw: values
        wp.Mesh = lambda **kw: types.SimpleNamespace(id=1)
        wp.mat44 = lambda *args: args
        wp.synchronize_device = Mock()
        calls = []
        def launch(kernel, dim, inputs, device):
            calls.append((kernel, dim, inputs, device))
            if kernel == "normal":
                inputs[5][..., 2] = -1
                inputs[6].zero_()
            else:
                inputs[5].fill_(2)
        wp.launch = launch
        kernels = types.SimpleNamespace(draw_optimized_kernel_normal_faceID="normal",
                                        draw_optimized_kernel_depth_range="range")
        with patch.dict(sys.modules, {"warp": wp}), patch(
                "renderer_validation.gbuffer.load_camera_kernels", return_value=kernels):
            renderer = WarpGBufferRenderer(box_fixture(), Camera(width=3, height=2), 2, "cpu")
            renderer.set_camera_poses([[0, 0, 0]] * 2, [[0, 0, 0, 1]] * 2)
            g = renderer.render()
        self.assertEqual(tuple(g.depth_m.shape), (2, 2, 3))
        self.assertEqual([c[0] for c in calls], ["normal", "range"])
        self.assertTrue(calls[0][2][-1])  # world-frame normal
        self.assertFalse(calls[1][2][-1])  # same RANGE cutoff, not optical far plane
        self.assertEqual(calls[0][2][4], calls[1][2][4])
        wp.synchronize_device.assert_called_once_with("cpu")

    def test_cli_help_is_lightweight_and_no_execution(self):
        script = ROOT / "tools/run_renderer_validation.py"
        command = "import runpy,sys; sys.argv=[%r,'--help'];\n" % str(script)
        command += "try: runpy.run_path(sys.argv[0],run_name='__main__')\nexcept SystemExit as e: assert e.code == 0\n"
        command += "assert not any(n in sys.modules for n in ['torch','warp','aerial_gym'])"
        result = subprocess.run([sys.executable, "-B", "-c", command], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_output_budget_rejects_large_dump(self):
        args = runner.parser().parse_args(["--output", "unused", "--num-scenes", "128",
                                          "--width", "480", "--height", "270"])
        with self.assertRaises(ValueError):
            runner.check_budget(args)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                runner.main(["--output", directory])

    def test_tensor_export_is_separate_arrays_and_no_overwrite(self):
        scene, camera, r, n, f = buffers()
        g = finalize_gbuffer(r, n, f, scene, camera)
        rgb = shade(g, scene, material(scene))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame.npz"
            receipt = runner.export_frame(path, g, rgb, rgb)
            with np.load(path, allow_pickle=False) as frame:
                self.assertEqual(frame["rgb_flat"].shape[-1], 3)
                self.assertEqual(frame["face_id"].dtype, np.int32)
                self.assertEqual(frame["valid"].dtype, np.bool_)
            self.assertEqual(receipt["file_sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
            with self.assertRaises(FileExistsError):
                runner.export_frame(path, g, rgb, rgb)

    def test_no_production_imports_or_training_code(self):
        for path in (ROOT / "tools/renderer_validation").glob("*.py"):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or "").startswith(("aerial_gym", "isaacgym")))
                if isinstance(node, ast.Import):
                    self.assertFalse(any(a.name.startswith(("aerial_gym", "isaacgym")) for a in node.names))

    def test_import_isolation_in_fresh_process(self):
        command = "import sys; sys.path.insert(0,%r); " % str(ROOT / "tools")
        command += "import renderer_validation.gbuffer,renderer_validation.shading,torch; "
        command += "assert 'warp' not in sys.modules; assert 'aerial_gym' not in sys.modules; "
        command += "assert 'isaacgym' not in sys.modules; assert not torch.cuda.is_initialized()"
        result = subprocess.run([sys.executable, "-B", "-c", command], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_mocked_cli_receipt_is_unassessed_and_records_sources(self):
        scene, camera, r, n, f = buffers()
        g = finalize_gbuffer(r, n, f, scene, camera)
        renderer = types.SimpleNamespace(
            wp=types.SimpleNamespace(__version__="MOCK", config=types.SimpleNamespace(kernel_cache_dir="MOCK")),
            render=lambda: g)
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "fresh"
            with patch("builtins.print"), patch("renderer_validation.gbuffer.WarpGBufferRenderer", return_value=renderer), patch(
                    "runtime_fingerprint.runtime_fingerprint", return_value={"python": "MOCK"}):
                runner.main(["--output", str(out), "--device", "cpu", "--width", "3", "--height", "1"])
            receipt = json.loads((out / "receipt.json").read_text())
            self.assertEqual(receipt["status"], "RENDERED_UNASSESSED")
            self.assertEqual(receipt["experiment_verdict"], "NOT_EVALUATED")
            self.assertEqual(receipt["training"], "NOT_SUPPORTED")
            self.assertEqual(receipt["benchmark"], "NOT_RUN")
            self.assertEqual(len(receipt["files"]), 2)
            self.assertIn("tools/renderer_validation/shading.py", receipt["source"]["source_sha256"])
            self.assertEqual(receipt["files"][0]["arrays"], receipt["files"][1]["arrays"])

    def test_mocked_failure_never_writes_success_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "fresh"
            with patch("renderer_validation.gbuffer.WarpGBufferRenderer", side_effect=RuntimeError("MOCK")):
                with self.assertRaises(RuntimeError):
                    runner.main(["--output", str(out), "--device", "cpu"])
            self.assertFalse((out / "receipt.json").exists())
            self.assertEqual(json.loads((out / "failure.json").read_text())["status"], "FAILED_INCOMPLETE")

    def test_source_drift_rejects_completed_mock_frames(self):
        scene, camera, r, n, f = buffers()
        g = finalize_gbuffer(r, n, f, scene, camera)
        renderer = types.SimpleNamespace(
            wp=types.SimpleNamespace(__version__="MOCK", config=types.SimpleNamespace(kernel_cache_dir="MOCK")),
            render=lambda: g)
        before = {"git_commit": "old", "source_sha256": {"code": "a"}}
        after = {"git_commit": "old", "source_sha256": {"code": "b"}}
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "fresh"
            with patch("renderer_validation.gbuffer.WarpGBufferRenderer", return_value=renderer), patch(
                    "runtime_fingerprint.runtime_fingerprint", return_value={}), patch.object(
                    runner, "source_record", side_effect=[before, after]):
                with self.assertRaisesRegex(RuntimeError, "Source changed"):
                    runner.main(["--output", str(out), "--device", "cpu", "--width", "3", "--height", "1"])
            self.assertTrue((out / "frame_0001.npz").exists())
            self.assertFalse((out / "receipt.json").exists())
            self.assertEqual(json.loads((out / "failure.json").read_text())["status"], "FAILED_INCOMPLETE")


if __name__ == "__main__":
    unittest.main()
