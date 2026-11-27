"""U2 synthetic whitelist, no-write, copy and static information-boundary tests."""
import ast
from dataclasses import fields
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import access  # noqa: E402
from motar_eval.boia import types as records  # noqa: E402

for module in (access, records):
    assert str(Path(module.__file__).resolve()).startswith(str(ROOT) + "/")


def fake_inputs(n=3):
    dtype = torch.float64
    yaw = torch.linspace(-1., 1., n, dtype=dtype)
    quat = torch.stack((yaw * 0, yaw * 0, (yaw / 2).sin(), (yaw / 2).cos()), -1)
    obs = {
        "robot_position": torch.arange(n * 3, dtype=dtype).reshape(n, 3),
        "robot_linvel": torch.full((n, 3), .25, dtype=dtype),
        "robot_vehicle_orientation": quat,
        "robot_body_angvel": torch.full((n, 3), .1, dtype=dtype),
        "navrl_target_position": torch.ones(n, 3, dtype=dtype),
        "segmentation_pixels": torch.ones(n, 2, 3, dtype=dtype),
        "obstacle_position": torch.ones(n, 4, 3, dtype=dtype),
    }
    task = SimpleNamespace(obs_dict=obs, _camera_visible_now=torch.ones(n, dtype=torch.bool),
                           _scan=torch.full((n, 288), 12., dtype=dtype),
                           task_obs={"states": torch.ones(n, 906, dtype=dtype)})
    task._lidar_distance_m = lambda: task._scan
    for name in access.FORBIDDEN_ATTRS:
        if name not in ("task_obs",):
            setattr(task, name, torch.ones(n, 3, dtype=dtype))
    task.detector = SimpleNamespace(**{
        name: torch.ones(n, 2, 3, dtype=dtype)
        for name in ("target_mask", "target_depth", "_targets", "_targets_wp",
                     "_target_orientations", "distractor_mask", "distractor_depth", "obstacle_depth")
    })
    tracker = SimpleNamespace(state=torch.zeros(n, 6, dtype=dtype),
                              cov=torch.eye(6, dtype=dtype)[None].repeat(n, 1, 1),
                              active=torch.ones(n, dtype=torch.bool), age=torch.zeros(n, dtype=dtype))
    perception = SimpleNamespace(
        tracker=tracker, _lidar_angles=math.pi - torch.arange(72, dtype=dtype) * (2 * math.pi / 72),
        hfov=math.radians(87), vfov=math.radians(58), fx=84.3, fy=81.2, cx=79.5, cy=44.5,
        camera_offset=torch.tensor([[.10, 0., .03]], dtype=dtype), target_radius=.15,
        max_camera_range=20., lidar_max_range=12., min_pixels=2,
    )
    action = torch.zeros(n, 4, dtype=dtype)
    return task, perception, action, records.BoiaParams(tau_on=.1)


def saved_tensors(value, prefix=""):
    result = {}
    if isinstance(value, torch.Tensor):
        result[prefix] = value.detach().clone()
    elif isinstance(value, dict):
        for key, item in value.items():
            result.update(saved_tensors(item, prefix + "/" + str(key)))
    elif isinstance(value, SimpleNamespace):
        result.update(saved_tensors(vars(value), prefix))
    return result


def assert_inputs_equal(test, left, right):
    for field in fields(records.OnlineInputs):
        a, b = getattr(left, field.name), getattr(right, field.name)
        if isinstance(a, torch.Tensor):
            test.assertTrue(torch.equal(a, b), field.name)
        else:
            test.assertEqual(a, b, field.name)


def check_access_ast(source):
    """Fail closed on forbidden reads and dynamic reflection/dictionary lookup.

    Only a literal, top-level, immutable blocklist declaration is exempt. This is
    a focused source guard, not a general proof about adversarial Python programs.
    """
    tree = ast.parse(source)
    declarations = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            if isinstance(node.targets[0], ast.Name) and node.targets[0].id == "FORBIDDEN_ATTRS":
                if not isinstance(node.value, ast.Tuple) or not all(
                        isinstance(item, ast.Constant) and isinstance(item.value, str)
                        for item in node.value.elts):
                    raise AssertionError("blocklist must be a literal immutable tuple")
                declarations.append(node)
    if len(declarations) != 1:
        raise AssertionError("one top-level blocklist declaration is required")
    exempt = {id(node) for node in ast.walk(declarations[0])}
    forbidden = set(access.FORBIDDEN_ATTRS)
    allowed_attributes = {
        "task": {"obs_dict", "_lidar_distance_m", "_camera_visible_now"},
        "perception": {"tracker", "_lidar_angles", "camera_offset", "hfov", "vfov", "fx", "fy",
                       "cx", "cy", "target_radius", "max_camera_range", "lidar_max_range", "min_pixels"},
        "tracker": {"state", "cov", "active", "age"},
    }
    allowed_keys = {"robot_position", "robot_linvel", "robot_vehicle_orientation", "robot_body_angvel"}
    reflection = {"getattr", "hasattr", "setattr", "delattr", "vars", "dir", "eval", "exec",
                  "compile", "__import__"}
    for node in ast.walk(tree):
        if id(node) in exempt:
            continue
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if node.value in forbidden or node.value.startswith("detect_"):
                raise AssertionError("forbidden string outside audit declaration")
        if isinstance(node, ast.Attribute):
            if node.attr in forbidden or node.attr.startswith("detect_") or node.attr in (
                    "__dict__", "__getattribute__", "__getattr__"):
                raise AssertionError("forbidden attribute access")
            if isinstance(node.value, ast.Name) and node.value.id in allowed_attributes:
                if node.attr not in allowed_attributes[node.value.id]:
                    raise AssertionError("attribute is outside the explicit input whitelist")
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id in reflection:
                raise AssertionError("dynamic reflection is not allowed")
            if isinstance(node.func, ast.Attribute) and node.func.attr in ("get", "__getitem__"):
                raise AssertionError("indirect mapping lookup is not allowed")
        if isinstance(node, ast.Subscript):
            key = node.slice
            if isinstance(key, ast.Index):  # Python 3.8 compatibility.
                key = key.value
            if not isinstance(key, (ast.Constant, ast.Slice, ast.Tuple, ast.UnaryOp)):
                raise AssertionError("computed dictionary/index lookup is not allowed")
            if isinstance(key, ast.Constant) and isinstance(key.value, str) and key.value not in allowed_keys:
                raise AssertionError("dictionary key is outside the explicit input whitelist")


class AccessTest(unittest.TestCase):
    def test_exact_shapes_calibration_and_raw_lidar_contract(self):
        inp = access.read_online(*fake_inputs())
        expected = {"pos": (3, 3), "vel": (3, 3), "vehicle_quat": (3, 4), "yaw": (3,),
                    "body_angvel": (3, 3), "lidar_m": (3, 288), "angles": (72,), "elevations": (4,),
                    "mu": (3, 6), "Sigma": (3, 6, 6), "active": (3,), "age": (3,),
                    "camera_visible": (3,), "policy_action": (3, 4)}
        for name, shape in expected.items():
            self.assertEqual(tuple(getattr(inp, name).shape), shape)
        self.assertEqual(inp.cam.camera_offset, (.10, 0., .03))
        self.assertEqual(inp.cam.min_pixels, 2)
        self.assertEqual(inp.lidar_far_m, 12.)
        torch.testing.assert_close(inp.elevations * (180 / math.pi),
                                   torch.tensor([20., 10., 0., -10.], dtype=torch.float64))

    def test_yaw_matches_independent_rotated_forward_vector(self):
        task, perception, action, params = fake_inputs(5)
        yaw = torch.tensor([-math.pi, -.4, 0., .8, math.pi], dtype=torch.float64)
        q = torch.stack((yaw * 0, yaw * 0, (yaw / 2).sin(), (yaw / 2).cos()), -1)
        task.obs_dict["robot_vehicle_orientation"] = q
        result = access.read_online(task, perception, action, params).yaw
        # The body-forward axis in world coordinates from quaternion rotation.
        forward_x = 1 - 2 * (q[:, 1].square() + q[:, 2].square())
        forward_y = 2 * (q[:, 0] * q[:, 1] + q[:, 3] * q[:, 2])
        torch.testing.assert_close(result.cos(), forward_x, atol=1e-15, rtol=0)
        torch.testing.assert_close(result.sin(), forward_y, atol=1e-15, rtol=0)

    def test_all_forbidden_mutations_leave_output_bitwise_equal(self):
        task, perception, action, params = fake_inputs()
        before = access.read_online(task, perception, action, params)
        generator = torch.Generator(device="cpu")
        generator.manual_seed(20261019)
        for name in access.FORBIDDEN_ATTRS:
            if name != "task_obs":
                value = getattr(task, name)
                value.copy_(torch.randn(value.shape, generator=generator, dtype=value.dtype))
        for key in ("navrl_target_position", "segmentation_pixels", "obstacle_position"):
            value = task.obs_dict[key]
            value.copy_(torch.randn(value.shape, generator=generator, dtype=value.dtype))
        for value in vars(task.detector).values():
            value.copy_(torch.randn(value.shape, generator=generator, dtype=value.dtype))
        task.task_obs["states"].copy_(torch.randn(3, 906, generator=generator, dtype=torch.float64))
        assert_inputs_equal(self, before, access.read_online(task, perception, action, params))

    def test_reads_actual_calibration_not_parameter_defaults(self):
        task, perception, action, params = fake_inputs()
        perception.min_pixels = 3
        perception.max_camera_range = 17.
        perception.camera_offset[0, 0] = .25
        result = access.read_online(task, perception, action, params)
        self.assertEqual(result.cam.min_pixels, 3)
        self.assertEqual(result.cam.max_camera_range, 17.)
        self.assertEqual(result.cam.camera_offset, (.25, 0., .03))
        self.assertEqual(params.min_pixels, 2.)

    def test_reader_does_not_write_any_source_tensor(self):
        task, perception, action, params = fake_inputs()
        before = saved_tensors({"task": task, "perception": perception, "action": action})
        access.read_online(task, perception, action, params)
        after = saved_tensors({"task": task, "perception": perception, "action": action})
        self.assertEqual(set(before), set(after))
        for key in before:
            self.assertTrue(torch.equal(before[key], after[key]), key)

    def test_copies_detach_and_do_not_alias_or_change_source(self):
        task, perception, action, params = fake_inputs()
        task.obs_dict["robot_position"].requires_grad_(True)
        perception.camera_offset.requires_grad_(True)
        saved = saved_tensors({"task": task, "perception": perception, "action": action})
        inp = access.read_online(task, perception, action, params)
        for field in fields(inp):
            item = getattr(inp, field.name)
            if isinstance(item, torch.Tensor):
                self.assertFalse(item.requires_grad)
                item.fill_(False if item.dtype == torch.bool else 0.)
        after = saved_tensors({"task": task, "perception": perception, "action": action})
        for key in saved:
            self.assertTrue(torch.equal(saved[key], after[key]), key)
        perception.camera_offset.detach().add_(1.)
        self.assertEqual(inp.cam.camera_offset, (.10, 0., .03))

    def test_repeated_reads_are_independent(self):
        args = fake_inputs()
        one, two = access.read_online(*args), access.read_online(*args)
        one.mu.fill_(100.)
        self.assertTrue(bool((two.mu == 0).all()))

    def test_lidar_method_called_once_and_never_rendered(self):
        task, perception, action, params = fake_inputs()
        calls = []
        def scan():
            calls.append(1)
            return task._scan
        task._lidar_distance_m = scan
        def forbidden(*args, **kwargs):
            raise AssertionError("second perception or render consume")
        task.detector = SimpleNamespace(render_raw_rgbd=forbidden)
        perception.observe = forbidden
        access.read_online(task, perception, action, params)
        self.assertEqual(len(calls), 1)

    def test_rejects_wrong_shapes_and_missing_names_without_fallback(self):
        for field in ("robot_position", "robot_linvel", "robot_vehicle_orientation", "robot_body_angvel"):
            task, perception, action, params = fake_inputs()
            task.obs_dict[field] = torch.zeros(3, 2, dtype=torch.float64)
            with self.subTest(field=field), self.assertRaises(ValueError):
                access.read_online(task, perception, action, params)
        task, perception, action, params = fake_inputs()
        del task.obs_dict["robot_body_angvel"]
        with self.assertRaises(KeyError):
            access.read_online(task, perception, action, params)
        for owner, name, bad in (("task", "_scan", torch.zeros(3, 72)),
                                 ("perception", "camera_offset", torch.zeros(3)),
                                 ("perception", "_lidar_angles", torch.zeros(71)),
                                 ("tracker", "cov", torch.zeros(3, 3, 3)),
                                 ("tracker", "state", torch.zeros(3, 3)),
                                 ("tracker", "age", torch.zeros(3, 1))):
            task, perception, action, params = fake_inputs()
            obj = task if owner == "task" else (perception.tracker if owner == "tracker" else perception)
            setattr(obj, name, bad)
            with self.subTest(name=name), self.assertRaises(ValueError):
                access.read_online(task, perception, action, params)
        args = list(fake_inputs())
        args[2] = torch.zeros(3, 3)
        with self.assertRaises(ValueError):
            access.read_online(*args)

    def test_rejects_nonfinite_and_wrong_flag_dtype(self):
        task, perception, action, params = fake_inputs()
        task.obs_dict["robot_position"][0, 0] = float("nan")
        with self.assertRaises(ValueError):
            access.read_online(task, perception, action, params)
        task, perception, action, params = fake_inputs()
        perception.tracker.active = torch.ones(3)
        with self.assertRaises(TypeError):
            access.read_online(task, perception, action, params)
        task, perception, action, params = fake_inputs()
        task._camera_visible_now = torch.ones(3)
        with self.assertRaises(TypeError):
            access.read_online(task, perception, action, params)

    def test_rejects_invalid_calibration_scan_and_quaternion(self):
        for key, value in (("min_pixels", 1.5), ("fx", 0.), ("hfov", math.pi),
                           ("vfov", float("nan")), ("lidar_max_range", -1.)):
            task, perception, action, params = fake_inputs()
            setattr(perception, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                access.read_online(task, perception, action, params)
        task, perception, action, params = fake_inputs()
        task._scan[0, 0] = 13.
        with self.assertRaises(ValueError):
            access.read_online(task, perception, action, params)
        task, perception, action, params = fake_inputs()
        task.obs_dict["robot_vehicle_orientation"].zero_()
        with self.assertRaises(ValueError):
            access.read_online(task, perception, action, params)
        task, perception, action, params = fake_inputs()
        task.obs_dict["robot_vehicle_orientation"].mul_(2.)
        with self.assertRaises(ValueError):
            access.read_online(task, perception, action, params)

    def test_read_preserves_global_rng_dtype_and_tf32_flag(self):
        before = torch.get_rng_state().clone()
        dtype = torch.get_default_dtype()
        tf32 = torch.backends.cuda.matmul.allow_tf32
        access.read_online(*fake_inputs())
        self.assertTrue(torch.equal(before, torch.get_rng_state()))
        self.assertEqual(dtype, torch.get_default_dtype())
        self.assertEqual(tf32, torch.backends.cuda.matmul.allow_tf32)

    def test_static_whitelist_source_and_literal_audit_exception(self):
        check_access_ast(Path(access.__file__).read_text())
        self.assertIsInstance(access.FORBIDDEN_ATTRS, tuple)
        self.assertEqual(len(set(access.FORBIDDEN_ATTRS)), len(access.FORBIDDEN_ATTRS))
        check_access_ast('FORBIDDEN_ATTRS = ("target_position",)\nx = task.obs_dict["robot_position"]')

    def test_static_guard_rejects_forbidden_reads_and_computed_access(self):
        bad = ["x = task.target_position", 'x = task.obs_dict["navrl_target_position"]',
               'x = getattr(task, "target_position")', 'x = getattr(task, "target_" + "position")',
               'x = task.obs_dict["target_" + "position"]', 'x = task.obs_dict[key]',
               'x = task.obs_dict.get("robot_position")', 'x = task.__dict__',
               'x = task.detector.detect_target', 'x = "segmentation_pixels"',
               'x = eval("task.robot_position")', 'x = task.additional_input',
               'x = perception.unlisted_calibration', 'x = task.obs_dict["unlisted_key"]']
        for code in bad:
            with self.subTest(code=code), self.assertRaises(AssertionError):
                check_access_ast('FORBIDDEN_ATTRS = ("target_position",)\n' + code)
        with self.assertRaises(AssertionError):
            check_access_ast('FORBIDDEN_ATTRS = ["target_position"]')
        with self.assertRaises(AssertionError):
            check_access_ast('def f():\n    FORBIDDEN_ATTRS = ("target_position",)')


if __name__ == "__main__":
    unittest.main()
