"""Actual-call motion bundles: CPU round-trip, complete batches and legacy isolation."""
import ast
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import torch

from test_kroc2027_adapter_contract import make_task, recorder_env

ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


adapter = load(ROOT / "aerial_gym/task/navrl_task/kroc_eval_adapter.py", "_mt_packet_adapter")
tm = load(ROOT / "aerial_gym/task/navrl_task/target_motion.py", "_mt_packet_target_motion")


def call_capture(backend="direct_l2_v1"):
    n, b = 3, 4
    # Deliberately non-contiguous views, with nonzero storage offsets; far obstacles stay included.
    positions = torch.tensor([[99., 10., 10., 88.], [99., 20., 20., 88.], [99., 30., 30., 88.]])
    old = positions[:, 1:3]
    bars_base = torch.tensor([[[99., 15., 15.], [99., 35., 35.], [99., 2., 35.], [99., 35., 2.]]] * n)
    bars = bars_base[:, :, 1:3]
    kwargs = dict(old_xy=old, current_velocity=torch.zeros(n, 2),
                  desired_velocity=torch.tensor([[1., 0.]] * n), speed_limit=torch.ones(n),
                  dt=0.1, bars_xy=bars, lo=torch.ones(n, 2), hi=torch.full((n, 2), 39.),
                  clearance=0.77, turn_sign=torch.tensor([-1., 1., -1.]),
                  max_accel=torch.full((n,), 4.), max_turn_rate=torch.full((n,), math.radians(150.)),
                  lookahead_s=1.0, bars_half_extents_xy=None,
                  exact_aabb_clearance=False, hard_epsilon_m=0., return_certificate=True,
                  certificate_row_ids=None, distance_backend=backend)
    result = tm.bounded_drone_target_step(**kwargs)
    return {"schema": adapter.FULL_PACKET_SCHEMA,
            "kwargs": {key: value.detach().clone() if isinstance(value, torch.Tensor) else value
                       for key, value in kwargs.items()},
            "input_layouts": {key: {"shape": list(value.shape), "stride": list(value.stride()),
                                      "storage_offset": value.storage_offset(),
                                      "dtype": str(value.dtype), "device": str(value.device)}
                              for key, value in kwargs.items() if isinstance(value, torch.Tensor)},
            "metadata": {"model_id": "bounded_planar_drone_v1_rollout_direct_l2_v1"
                         if backend == "direct_l2_v1" else "bounded_planar_drone_v1_rollout",
                         "behavior_level": "e1", "distance_backend": backend,
                         "torch_version": str(torch.__version__), "device": "cpu",
                         "matmul_allow_tf32": False, "cudnn_allow_tf32": False,
                         "float32_matmul_precision": "highest", "task_step": 42,
                         "env_ids": torch.tensor([0, 1, 2]), "obstacle_slots": torch.tensor([4, 5, 6, 7])},
            "result": {"new_xy": result[0], "new_velocity": result[1], "steered": result[2],
                       "immediate_feasible": result[3], "certificate": result[4]}}


class MotionBundleTest(unittest.TestCase):
    def test_packet_switch_is_separately_opted_in(self):
        self.assertEqual(adapter.motion_packet_schema({}), "legacy_v1")
        self.assertEqual(adapter.motion_packet_schema({adapter.PACKET_SCHEMA_VAR: "full_v2"}), "legacy_v1")
        self.assertEqual(adapter.motion_packet_schema({adapter.PACKETS_VAR: "1"}), "legacy_v1")
        self.assertEqual(adapter.motion_packet_schema({adapter.PACKETS_VAR: "1",
                                                       adapter.PACKET_SCHEMA_VAR: "full_v2"}), "full_v2")
        with self.assertRaises(ValueError):
            adapter.motion_packet_schema({adapter.PACKETS_VAR: "1", adapter.PACKET_SCHEMA_VAR: "typo"})

    def test_full_call_roundtrip_replays_all_rows_both_backends(self):
        for backend in ("legacy_cdist", "direct_l2_v1"):
            original = call_capture(backend)
            rng = torch.random.get_rng_state().clone()
            encoded = adapter.encode_motion_call_bundle(original)
            bundle = adapter.decode_motion_packet_value(json.loads(json.dumps(encoded)))
            self.assertTrue(torch.equal(torch.random.get_rng_state(), rng))
            self.assertEqual(bundle["kwargs"]["old_xy"].shape, (3, 2))
            self.assertEqual(bundle["kwargs"]["bars_xy"].shape, (3, 4, 2))
            self.assertEqual(bundle["kwargs"]["old_xy"].stride(), (4, 1))
            self.assertEqual(bundle["kwargs"]["old_xy"].storage_offset(), 1)
            self.assertEqual(bundle["kwargs"]["bars_xy"].stride(), (12, 3, 1))
            self.assertTrue(torch.equal(bundle["metadata"]["obstacle_slots"], torch.tensor([4, 5, 6, 7])))
            self.assertTrue(torch.equal(bundle["kwargs"]["turn_sign"], torch.tensor([-1., 1., -1.])))
            replay = tm.bounded_drone_target_step(**bundle["kwargs"])
            for actual, key in zip(replay[:4], ("new_xy", "new_velocity", "steered", "immediate_feasible")):
                self.assertTrue(torch.equal(actual, bundle["result"][key]), (backend, key))
            for key, value in replay[4].items():
                captured = bundle["result"]["certificate"][key]
                if isinstance(value, torch.Tensor):
                    self.assertTrue(torch.equal(value, captured), (backend, key))
                else:
                    self.assertEqual(value, captured)

    def test_dtype_broadcast_and_empty_layout_are_preserved(self):
        examples = [torch.tensor([1., 2.], dtype=torch.float64).expand(3, 2),
                    torch.empty((0, 2), dtype=torch.float32), torch.tensor(True),
                    torch.tensor([2, 4], dtype=torch.int16)]
        for tensor in examples:
            decoded = adapter.decode_motion_packet_value(adapter._packet_encode(tensor))
            self.assertTrue(torch.equal(decoded, tensor))
            self.assertEqual(decoded.dtype, tensor.dtype)
            self.assertEqual(decoded.stride(), tensor.stride())

    def test_missing_actual_inputs_fail_instead_of_inference(self):
        with self.assertRaises(RuntimeError):
            adapter.encode_motion_call_bundle(None)
        for section, field in (("kwargs", "turn_sign"), ("kwargs", "distance_backend"),
                               ("input_layouts", "bars_xy"), ("metadata", "obstacle_slots"),
                               ("result", "certificate")):
            capture = call_capture()
            del capture[section][field]
            with self.assertRaises(RuntimeError):
                adapter.encode_motion_call_bundle(capture)

    def test_hashed_bundle_rejects_changed_bytes(self):
        raw = json.dumps(adapter.encode_motion_call_bundle(call_capture())).encode()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bundle.json"
            path.write_bytes(raw)
            digest = hashlib.sha256(raw).hexdigest()
            self.assertEqual(adapter.load_motion_call_bundle(path, digest)["kwargs"]["dt"], .1)
            path.write_bytes(raw + b" ")
            with self.assertRaises(ValueError):
                adapter.load_motion_call_bundle(path, digest)


class FullPacketRecorderTest(unittest.TestCase):
    def recorder(self, directory):
        task = make_task(n=3)
        task._target_distance_backend = "direct_l2_v1"
        task._target_motion_model = "bounded_planar_drone_v1_rollout_direct_l2_v1"
        task._mt_precision_last_call = call_capture()
        env = recorder_env(directory, NAVRL_KROC2027_PACKETS="1", NAVRL_KROC2027_PACKET_SCHEMA="full_v2")
        with mock.patch.dict(os.environ, env):
            rec = adapter.build_kroc_adapter(task)
        self.addCleanup(rec.finalize)
        return task, rec

    def test_multiple_violation_rows_share_full_bundle_without_executor_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.recorder(directory)
            capture = task._mt_precision_last_call
            originals = {key: value.clone() for key, value in capture["kwargs"].items()
                         if isinstance(value, torch.Tensor)}
            old_rng = torch.random.get_rng_state().clone()
            # The writer is forbidden from making a fresh reduced-batch CUDA/CPU executor call.
            with mock.patch.object(adapter, "_target_motion_module", side_effect=AssertionError("replay")):
                rec._write_packets(torch.tensor([0, 2]), capture["result"]["new_xy"],
                                   capture["result"]["new_velocity"], torch.zeros(3))
                rec._write_packets(torch.tensor([1]), capture["result"]["new_xy"],
                                   capture["result"]["new_velocity"], torch.zeros(3))
            rec.finalize()
            self.assertTrue(torch.equal(old_rng, torch.random.get_rng_state()))
            for key, value in originals.items():
                self.assertTrue(torch.equal(value, capture["kwargs"][key]))
            rows = [json.loads(row) for row in rec.packets_path.read_text().splitlines()]
            self.assertEqual(len(rows), 3)
            self.assertEqual(len({row["bundle_sha256"] for row in rows}), 1)
            self.assertEqual(len(list((Path(directory) / adapter.FULL_PACKET_BUNDLES).glob("*.json"))), 1)
            self.assertEqual([row["executor_row_index"] for row in rows], [0, 2, 1])
            self.assertTrue(all(row["full_obstacle_shape"] == [3, 4, 2] for row in rows))
            bundle = adapter.load_motion_call_bundle(Path(directory) / rows[0]["bundle_file"],
                                                     rows[0]["bundle_sha256"])
            self.assertEqual(bundle["kwargs"]["bars_xy"].shape[1], 4)  # far bars were not truncated
            cert = adapter.decode_motion_packet_value(rows[1]["selected_certificate"])
            self.assertEqual(cert["selected_index"].item(), capture["result"]["certificate"]["selected_index"][2])
            summary = json.loads(rec.summary_path.read_text())
            self.assertEqual(summary["violation_packets"]["schema"], "full_v2")
            self.assertEqual(summary["context"]["cfg_target_distance_backend"], "direct_l2_v1")

    def test_actual_boundary_capture_is_required_even_if_legacy_snapshot_exists(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.recorder(directory)
            task._mt_precision_last_call = None
            rec._snapshot = {"not": "the actual executor inputs"}
            with self.assertRaises(RuntimeError):
                rec._write_packets(torch.tensor([0]), torch.zeros(3, 2), torch.zeros(3, 2), torch.zeros(3))
            self.assertEqual(rec.packet_count, 0)
            rec.finalize()

    def test_nonfinite_executed_state_is_preserved_for_diagnosis(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.recorder(directory)
            capture = task._mt_precision_last_call
            capture["result"]["new_xy"][0, 0] = float("nan")
            capture["result"]["new_velocity"][0, 1] = float("nan")
            rec._write_full_packets(
                torch.tensor([0]), capture["result"]["new_xy"],
                capture["result"]["new_velocity"], torch.ones(3),
                event_type="motion_gate_batch", batch_motion_gate_trigger=True)
            row = json.loads(rec.packets_path.read_text().splitlines()[0])
            bundle = adapter.load_motion_call_bundle(
                Path(directory) / row["bundle_file"], row["bundle_sha256"])
            self.assertTrue(torch.isnan(bundle["result"]["new_xy"][0, 0]))
            self.assertTrue(torch.isnan(bundle["result"]["new_velocity"][0, 1]))
            self.assertEqual(row["row_violation_status"], "NOT_RECORDED")
            rec.finalize()

    def test_stale_or_mutated_call_is_rejected_before_packet_write(self):
        for cause in ("position", "velocity", "task_step", "unknown_env", "duplicate_env"):
            with tempfile.TemporaryDirectory() as directory:
                task, rec = self.recorder(directory)
                capture = task._mt_precision_last_call
                xy = capture["result"]["new_xy"].clone()
                velocity = capture["result"]["new_velocity"].clone()
                if cause == "position":
                    xy[0, 0] += .01
                elif cause == "velocity":
                    velocity[0, 1] += .01
                elif cause == "task_step":
                    task.num_task_steps = 43
                elif cause == "unknown_env":
                    capture["metadata"]["env_ids"][0] = 9
                else:
                    capture["metadata"]["env_ids"][0] = 1
                with self.assertRaises(RuntimeError):
                    rec._write_packets(torch.tensor([0]), xy, velocity, torch.zeros(3))
                self.assertEqual(rec.packet_count, 0)
                self.assertEqual(rec.packets_path.read_text(), "")
                self.assertFalse((Path(directory) / adapter.FULL_PACKET_BUNDLES).exists())
                rec.finalize()

    def test_motion_gate_only_trigger_records_full_batch_without_row_blame(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.recorder(directory)
            capture = task._mt_precision_last_call
            task.obs_dict["obstacle_position"][:, :, :2] = 100.
            task.target_position[:, :2] = capture["kwargs"]["old_xy"]
            task.target_vel_w[:, :2] = capture["kwargs"]["current_velocity"]
            rec.on_reset(torch.arange(3))
            task.target_position[:, :2] = capture["result"]["new_xy"]
            task.target_vel_w[:, :2] = capture["result"]["new_velocity"]
            with mock.patch.object(rec.motion, "step", return_value=True):
                # Record, rather than halt, to inspect the packet (same option as a canary).
                rec.on_violation = "record"
                rec.after_target_advance()
            rec.finalize()
            rows = [json.loads(row) for row in rec.packets_path.read_text().splitlines()]
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["event_type"], "motion_gate_batch")
            self.assertTrue(rows[0]["batch_motion_gate_trigger"])
            self.assertEqual(rows[0]["row_violation_status"], "NOT_RECORDED")
            self.assertEqual(rows[0]["full_batch_shape"], [3, 2])

    def test_task_captures_preclone_layout_and_all_actual_kwargs_before_execution(self):
        tree = ast.parse((ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text())
        function = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                        and node.name == "_advance_target")
        source = ast.get_source_segment((ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text(), function)
        self.assertIn('"input_layouts"', source)
        self.assertIn('"obstacle_slots"', source)
        self.assertIn('"env_ids"', source)
        self.assertIn("bounded_drone_target_step(**bounded_kwargs)", source)
        self.assertLess(source.index("self._mt_precision_last_call ="),
                        source.index("bounded_drone_target_step(**bounded_kwargs)"))


if __name__ == "__main__":
    unittest.main()
