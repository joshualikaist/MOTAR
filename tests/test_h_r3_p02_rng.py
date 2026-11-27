"""CPU-only fixed-N RNG independence with adaptive resets and discarded actor samples."""
import random
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from h_r3_p02_rng import FixedNRandomness, keyed_seed  # noqa: E402


def sample_at_one_callsite(task, ids):
    return task._sample_waypoints(torch.tensor(ids, dtype=torch.long))


class FakeManager:
    def __init__(self, task):
        self.task = task
        self.cfg = SimpleNamespace(env=SimpleNamespace(num_physics_steps_per_env_step_std=0))
        self.IGE_env = FakeIGE(task.num_envs)
        self.robot_manager = SimpleNamespace(
            robot=SimpleNamespace(cfg=SimpleNamespace(), controller=SimpleNamespace(cfg=SimpleNamespace()),
                                  robot_euler_angles=torch.zeros(task.num_envs, 3)),
            warp_sensor=SimpleNamespace(cfg=SimpleNamespace()), imu_sensor=None, camera_sensor=None)

    def reset_idx(self, ids):
        # Match the active simulator's full-N draws followed by selected-row writes.
        values = torch.rand(self.task.num_envs, 3)
        values += torch.as_tensor(np.random.random((self.task.num_envs, 3)), dtype=torch.float32)
        values += random.random()
        for env in ids.tolist():
            # Different envs take different rejection paths; selected-row scopes must isolate it.
            for _ in range(1 + env * 2):
                torch.rand(11)
                np.random.random(5)
                random.random()
        self.task.state[ids] = values[ids]
        # Reproduce original robot.update_states' unwanted refresh of every env row.
        self.robot_manager.robot.robot_euler_angles[:] = self.task.state + 10.0

    def render(self, render_components):
        if render_components != "sensors":
            raise AssertionError("unexpected render")
        self.task.raw = self.task.state.clone()


class FakeGym:
    def __init__(self):
        self.indexed, self.full = [], []
        self.accept = True

    def set_actor_root_state_tensor_indexed(self, sim, root, ids, count):
        self.indexed.append((sim, root, ids.clone(), count))
        return self.accept

    def set_actor_root_state_tensor(self, sim, root):
        self.full.append((sim, root))
        return True


class FakeIGE:
    def __init__(self, n):
        self.sim_has_dof = False
        self.gym, self.sim = FakeGym(), object()
        self.global_tensor_dict = {"unfolded_env_asset_state_tensor": torch.zeros(n * 3, 13)}

    def write_to_sim(self):
        return self.gym.set_actor_root_state_tensor(
            self.sim, self.global_tensor_dict["unfolded_env_asset_state_tensor"])


class FakeTask:
    def __init__(self, n=4):
        self.num_envs, self.device = n, "cpu"
        self.state = torch.zeros(n, 3)
        self.goal = torch.zeros(n, 2)
        self.raw = torch.zeros(n, 3)
        self.sim_env = FakeManager(self)
        self._target_dynamics = "bounded"
        self._target_behavior = {"effective_pattern": "waypoint"}
        self._target_behavior_level = "e2_obstacle_aware"
        self.general_spawn_mode = self.vision_mode = self.perception_mode = True
        self.perception_cfg = SimpleNamespace(enable_perturbations=False)
        self.perception = SimpleNamespace()
        self.detector = SimpleNamespace()
        self.observation_updates = 0
        self.syncs = 0

    def reset_idx(self, ids):
        self.goal[ids] = self._sample_waypoints(ids)
        self.sim_env.robot_manager.robot.robot_euler_angles[:] = self.state + 20.0
        for env in ids.tolist():
            for _ in range(env + 1):
                torch.randn(7)
                random.gauss(0.0, 1.0)
                np.random.random(3)

    def _sample_waypoints(self, ids):
        return torch.rand(len(ids), 2)

    def _sync_target_to_sensor(self):
        self.syncs += 1

    def get_return_tuple(self):
        self.observation_updates += 1
        raise AssertionError("diagnostic must not advance all-env observations")


def reset(task, ids):
    task.sim_env.reset_idx(torch.tensor(ids, dtype=torch.long))
    task.reset_idx(torch.tensor(ids, dtype=torch.long))


def initial(seed=4601, order=(0, 1, 2, 3), split=False):
    task = FakeTask()
    rng = FixedNRandomness(task, seed)
    if split:
        for env in order:
            reset(task, [env])
    else:
        reset(task, order)
    return task, rng


def numpy_state_equal(a, b):
    return a[0] == b[0] and np.array_equal(a[1], b[1]) and a[2:] == b[2:]


class IsolationTest(unittest.TestCase):
    def test_stable_seed_keys(self):
        self.assertEqual(keyed_seed(4601, "waypoint", 2, 3), keyed_seed(4601, "waypoint", 2, 3))
        self.assertNotEqual(keyed_seed(4601, "waypoint", 2, 3), keyed_seed(4601, "waypoint", 3, 2))
        self.assertNotEqual(keyed_seed(4601, "waypoint"), keyed_seed(4602, "waypoint"))

    def test_reset_order_and_partition_do_not_change_rows(self):
        baseline, a = initial()
        shuffled, b = initial(order=(3, 1, 0, 2), split=True)
        self.assertTrue(torch.equal(baseline.state, shuffled.state))
        self.assertTrue(torch.equal(baseline.goal, shuffled.goal))
        self.assertEqual(a.episodes, b.episodes)
        self.assertEqual(a.episodes, [0, 0, 0, 0])
        self.assertFalse(any(a.pending))

    def test_waypoints_order_split_and_extra_env_draws(self):
        a, ra = initial()
        b, rb = initial()
        with ra.step(5):
            expected = sample_at_one_callsite(a, [1, 3])
        with rb.step(5):
            sample_at_one_callsite(b, [0])
            sample_at_one_callsite(b, [0])
            got3 = sample_at_one_callsite(b, [3])
            got1 = sample_at_one_callsite(b, [1])
        self.assertTrue(torch.equal(expected, torch.cat((got1, got3))))

    def test_forced_unrelated_reset_preserves_future_draws_state_and_raw(self):
        def rollout(extra_reset):
            task, rng = initial()
            records = []
            for k in range(8):
                with rng.action():
                    torch.randn(17 + 3 * k)
                    random.random()
                    np.random.random(12)
                with rng.step(k):
                    exogenous = torch.randn(4, 3)
                    exogenous += torch.as_tensor(np.random.random((4, 3)), dtype=torch.float32)
                    exogenous += random.gauss(0.0, 1.0)
                    task.state += exogenous
                    if extra_reset and k == 2:
                        rng.force_reset([0])
                    task.goal[[1, 3]] = sample_at_one_callsite(task, [1, 3])
                    task.sim_env.render("sensors")
                records.append((task.state[1:].clone(), task.goal[1:].clone(),
                                task.raw[1:].clone(), rng.probe()[1:].clone(),
                                task.sim_env.robot_manager.robot.robot_euler_angles[1:].clone()))
            self.assertEqual(task.observation_updates, 0)
            return records
        baseline, forced = rollout(False), rollout(True)
        for expected, got in zip(baseline, forced):
            for x, y in zip(expected, got):
                self.assertTrue(torch.equal(x, y))

    def test_reset_preserves_other_rows_stale_derived_kinematics(self):
        task, rng = initial()
        robot = task.sim_env.robot_manager.robot
        robot.robot_euler_angles[:] = -123.0
        rng.force_reset([2])
        self.assertTrue(torch.equal(robot.robot_euler_angles[[0, 1, 3]], torch.full((3, 3), -123.0)))
        self.assertTrue(torch.equal(robot.robot_euler_angles[2], task.state[2] + 20.0))

    def test_scope_restores_python_numpy_torch_even_on_exception(self):
        _, rng = initial()
        random.seed(42)
        np.random.seed(43)
        torch.random.default_generator.manual_seed(44)
        python_state, numpy_state, torch_state = random.getstate(), np.random.get_state(), torch.get_rng_state()
        with self.assertRaisesRegex(RuntimeError, "fixture"):
            with rng.step(2):
                random.random()
                np.random.random(30)
                torch.rand(20)
                raise RuntimeError("fixture")
        self.assertEqual(python_state, random.getstate())
        self.assertTrue(numpy_state_equal(numpy_state, np.random.get_state()))
        self.assertTrue(torch.equal(torch_state, torch.get_rng_state()))

    def test_actor_samples_and_probe_leave_step_draws_identical(self):
        _, rng = initial()
        with rng.step(11):
            a = torch.randn(9)
            with rng.action():
                torch.randn(1000)
                random.random()
                np.random.random(13)
            rng.probe()
            b = torch.randn(9)
        with rng.step(11):
            self.assertTrue(torch.equal(a, torch.randn(9)))
            self.assertTrue(torch.equal(b, torch.randn(9)))

    def test_episode_token_advanced_once_at_manager_and_consumed_at_task(self):
        task, rng = initial()
        task.sim_env.reset_idx(torch.tensor([2]))
        self.assertEqual(rng.episodes, [0, 0, 1, 0])
        self.assertTrue(rng.pending[2])
        task.reset_idx(torch.tensor([2]))
        self.assertEqual(rng.episodes, [0, 0, 1, 0])
        self.assertFalse(any(rng.pending))
        with self.assertRaisesRegex(RuntimeError, "matching manager"):
            task.reset_idx(torch.tensor([2]))
        task.sim_env.reset_idx(torch.tensor([2]))
        with self.assertRaisesRegex(RuntimeError, "pending task"):
            task.sim_env.reset_idx(torch.tensor([2]))

    def test_invalid_ids_and_duplicate_install_fail(self):
        task, _ = initial()
        for ids in ([0, 0], [-1], [4], [0.5]):
            with self.assertRaises(ValueError):
                task.sim_env.reset_idx(ids)
        with self.assertRaisesRegex(RuntimeError, "already installed"):
            FixedNRandomness(task, 4601)


class ProfileTest(unittest.TestCase):
    def test_missing_irrelevant_optional_attributes_use_defaults_and_flags_recorded(self):
        _, rng = initial()
        flags = rng.metadata()["checked_flags"]
        self.assertFalse(flags["perception.enable_perturbations"])
        self.assertFalse(flags["lidar.sensor_noise"])
        self.assertFalse(flags["robot.disturbance"])
        self.assertFalse(rng.metadata()["batch_size_invariant"])

    def test_unaudited_optional_modes_fail_before_installing_hooks(self):
        edits = (
            lambda t: setattr(t.perception_cfg, "enable_perturbations", True),
            lambda t: setattr(t.perception, "pose_noise_pos_m", 0.1),
            lambda t: setattr(t.perception, "_detector_noise_active", True),
            lambda t: setattr(t.perception, "empirical_error", object()),
            lambda t: setattr(t.detector, "mount_rot_deg", 1.0),
            lambda t: setattr(t, "_target_route_enabled", True),
            lambda t: t._target_behavior.update(effective_pattern="cv"),
            lambda t: setattr(t, "_target_dynamics", "physical"),
            lambda t: setattr(t.sim_env.robot_manager.robot.cfg, "disturbance",
                              SimpleNamespace(enable_disturbance=True)),
            lambda t: setattr(t.sim_env.robot_manager.warp_sensor.cfg, "sensor_noise",
                              SimpleNamespace(enable_sensor_noise=True)),
            lambda t: setattr(t.sim_env.cfg.env, "num_physics_steps_per_env_step_std", 0.1),
        )
        for edit in edits:
            task = FakeTask()
            original = task.reset_idx
            edit(task)
            with self.assertRaisesRegex(RuntimeError, "unaudited configuration"):
                FixedNRandomness(task, 4601)
            self.assertEqual(original, task.reset_idx)
            self.assertFalse(hasattr(task, "_h_r3_p02_rng"))


class IndexedResetTest(unittest.TestCase):
    def _fixture(self):
        task = FakeTask()
        manager_reset, task_reset = task.sim_env.reset_idx, task.reset_idx

        def manager_with_writes(ids):
            manager_reset(ids)
            task.sim_env.IGE_env.write_to_sim()
            task.sim_env.IGE_env.write_to_sim()

        def task_with_writes(ids):
            task_reset(ids)
            task.sim_env.IGE_env.write_to_sim()
            task.sim_env.IGE_env.write_to_sim()

        task.sim_env.reset_idx, task.reset_idx = manager_with_writes, task_with_writes
        return task, FixedNRandomness(task, 4601)

    @mock.patch("h_r3_p02_rng._unwrap_tensor", side_effect=lambda tensor: tensor)
    def test_selected_env_actor_union_submitted_once_with_original_state_buffer(self, _unwrap):
        task, rng = self._fixture()
        ige = task.sim_env.IGE_env
        task.sim_env.reset_idx(torch.tensor([3, 1]))
        self.assertEqual(len(ige.gym.indexed), 0)
        self.assertEqual(len(ige.gym.full), 0)
        task.reset_idx(torch.tensor([3, 1]))
        self.assertEqual(len(ige.gym.indexed), 1)
        sim, root, ids, count = ige.gym.indexed[0]
        self.assertIs(sim, ige.sim)
        self.assertIs(root, ige.global_tensor_dict["unfolded_env_asset_state_tensor"])
        self.assertEqual(ids.dtype, torch.int32)
        self.assertEqual(ids.tolist(), [3, 4, 5, 9, 10, 11])
        self.assertEqual(count, 6)
        self.assertEqual(rng.metadata()["pending_root_envs"], [])
        self.assertTrue(ige.write_to_sim())
        self.assertEqual(len(ige.gym.full), 1)
        self.assertEqual(len(ige.gym.indexed), 1)

    @mock.patch("h_r3_p02_rng._unwrap_tensor", side_effect=lambda tensor: tensor)
    def test_failed_indexed_setter_is_integrity_failure(self, _unwrap):
        task, _rng = self._fixture()
        task.sim_env.IGE_env.gym.accept = False
        task.sim_env.reset_idx(torch.tensor([2]))
        with self.assertRaisesRegex(RuntimeError, "setter rejected"):
            task.reset_idx(torch.tensor([2]))

    def test_dof_and_nondivisible_actor_layout_fail_before_hook_install(self):
        task = FakeTask()
        task.sim_env.IGE_env.sim_has_dof = True
        with self.assertRaisesRegex(RuntimeError, "has_dof"):
            FixedNRandomness(task, 4601)
        task = FakeTask()
        task.sim_env.IGE_env.global_tensor_dict["unfolded_env_asset_state_tensor"] = torch.zeros(11, 13)
        original = task.reset_idx
        with self.assertRaisesRegex(RuntimeError, "actor root tensor"):
            FixedNRandomness(task, 4601)
        self.assertEqual(original, task.reset_idx)


if __name__ == "__main__":
    unittest.main()
