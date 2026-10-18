"""Opt-in H-R3 P02 fixed-N RNG isolation; imports launch no simulator or GPU work.

Install after the common player construction and before its first env_reset. The original
reset implementations still generate full-N tensors, so this intentionally makes no claim of
batch-size invariance. Production classes and the P01 launcher are never modified.
"""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
import inspect
import json
import random
import sys

import numpy as np
import torch


SCHEMA = "p02.fixedN.seeded-isolated.v1"
DERIVED_KINEMATICS = ("robot_euler_angles", "robot_vehicle_orientation", "robot_vehicle_linvel",
                      "robot_body_linvel", "robot_body_angvel")


def keyed_seed(seed, *coordinates):
    """Stable seed: neither process hash randomization nor arm identity is an input."""
    payload = json.dumps([SCHEMA, int(seed), *coordinates], separators=(",", ":"),
                         ensure_ascii=True).encode("ascii")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "little") & ((1 << 63) - 1)


def _unwrap_tensor(tensor):
    # Lazy import keeps this runner helper importable by public CPU tests without Isaac Gym.
    from isaacgym import gymtorch

    return gymtorch.unwrap_tensor(tensor)


class FixedNRandomness:
    """Instance-local reset/waypoint hooks plus restored step/action RNG scopes.

    ``step(k)`` wraps the normal environment step. ``action()`` protects discarded actor
    samples, and can be entered before or inside ``step(k)``. ``force_reset`` is solely for
    the preregistered unrelated-env diagnostic; it does not advance perception for other envs.
    """

    def __init__(self, task, seed, validate=True):
        if getattr(task, "_h_r3_p02_rng", None) is not None:
            raise RuntimeError("P02 RNG hooks already installed")
        self.task, self.seed = task, int(seed)
        self.num_envs = int(task.num_envs)
        self.device = torch.device(task.device)
        self.logical_step = -1
        self.episodes = [-1] * self.num_envs
        self.pending = [False] * self.num_envs
        self._phase = "runtime"
        self._reset_env_id = None
        self._pending_root_envs = set()
        self._calls = {}
        self._totals = {"manager_reset": 0, "task_reset": 0, "waypoint": 0,
                        "step": 0, "action": 0, "forced_reset": 0, "indexed_root_write": 0}
        self.checked_flags = self._validate() if validate else {"validation": "CPU_FIXTURE_ONLY"}
        self._cuda_devices = []
        if self.device.type == "cuda":
            index = self.device.index
            self._cuda_devices = [torch.cuda.current_device() if index is None else index]
        self._manager_reset = task.sim_env.reset_idx
        self._task_reset = task.reset_idx
        self._waypoints = task._sample_waypoints
        self._ige = task.sim_env.IGE_env
        self._write_to_sim = self._ige.write_to_sim
        if bool(getattr(self._ige, "sim_has_dof", False)):
            raise RuntimeError("P02 indexed reset writes require the audited no-DOF robot")
        root = self._ige.global_tensor_dict["unfolded_env_asset_state_tensor"]
        if root.ndim != 2 or root.shape[0] % self.num_envs != 0 or root.shape[1] != 13:
            raise RuntimeError("P02 actor root tensor must have complete equal-N rows of 13 states")
        self._actors_per_env = int(root.shape[0] // self.num_envs)
        if self._actors_per_env < 1:
            raise RuntimeError("P02 actor root tensor has no actors")
        task.sim_env.reset_idx = self._reset_manager
        task.reset_idx = self._reset_task
        task._sample_waypoints = self._sample_waypoints
        self._ige.write_to_sim = self._write_selected_roots
        task._h_r3_p02_rng = self

    def _validate(self):
        """Pin the audited active profile; optional absent knobs have their production defaults."""
        task = self.task
        checked, bad = {}, []

        def require(name, value, expected):
            checked[name] = value
            if value != expected:
                bad.append("%s=%r (requires %r)" % (name, value, expected))

        require("target_dynamics", getattr(task, "_target_dynamics", None), "bounded")
        behavior = getattr(task, "_target_behavior", {})
        require("target_pattern", behavior.get("effective_pattern"), "waypoint")
        require("target_behavior_level", getattr(task, "_target_behavior_level", None), "e2_obstacle_aware")
        require("target_route_enabled", bool(getattr(task, "_target_route_enabled", False)), False)
        require("physical_target", bool(getattr(task, "_physical_target", False)), False)
        require("general_eval_mode", bool(getattr(task, "general_eval_mode", False)), False)
        require("general_spawn_mode", bool(getattr(task, "general_spawn_mode", False)), True)
        require("vision_mode", bool(getattr(task, "vision_mode", False)), True)
        require("perception_mode", bool(getattr(task, "perception_mode", False)), True)
        cfg = getattr(task, "perception_cfg", None)
        require("perception.enable_perturbations", bool(getattr(cfg, "enable_perturbations", False)), False)
        perception = getattr(task, "perception", None)
        require("perception.present", perception is not None, True)
        for name in ("pose_noise_pos_m", "pose_noise_yaw_deg", "detection_latency_s"):
            require("perception." + name, float(getattr(perception, name, 0.0)), 0.0)
        require("perception.detector_noise_active", bool(getattr(perception, "_detector_noise_active", False)), False)
        require("perception.empirical_error", getattr(perception, "empirical_error", None) is not None, False)
        module = sys.modules.get(type(perception).__module__)
        require("perception.geofence_actor", bool(getattr(module, "GEOFENCE_ACTOR", False)), False)
        for name in ("GEOFENCE_NOISE_STD_M", "GEOFENCE_DROPOUT"):
            require("perception." + name.lower(), float(getattr(module, name, 0.0)), 0.0)
        detector = getattr(task, "detector", None)
        require("detector.present", detector is not None, True)
        for name in ("app_hue_deg", "app_light_gain", "app_albedo_jitter", "app_texture_std",
                     "app_motion_blur", "mount_rot_deg", "mount_trans_m"):
            require("detector." + name, float(getattr(detector, name, 0.0)), 0.0)
        manager = task.sim_env
        require("physics.has_dof", bool(getattr(manager.IGE_env, "sim_has_dof", False)), False)
        env_cfg = getattr(getattr(manager, "cfg", None), "env", None)
        require("physics_substep_std", float(getattr(env_cfg, "num_physics_steps_per_env_step_std", float("nan"))), 0.0)
        robot_manager = manager.robot_manager
        robot = robot_manager.robot
        robot_cfg = getattr(robot, "cfg", None)
        disturbance = getattr(robot_cfg, "disturbance", None)
        require("robot.disturbance", bool(getattr(disturbance, "enable_disturbance", False)), False)
        require("robot.imu_sensor", getattr(robot_manager, "imu_sensor", None) is not None, False)
        require("robot.native_camera_sensor", getattr(robot_manager, "camera_sensor", None) is not None, False)
        controller_cfg = getattr(getattr(robot, "controller", None), "cfg", None)
        require("controller.randomize_params", bool(getattr(controller_cfg, "randomize_params", False)), False)
        sensor = getattr(robot_manager, "warp_sensor", None)
        sensor_cfg = getattr(sensor, "cfg", None)
        require("lidar.present", sensor is not None, True)
        require("lidar.randomize_placement", bool(getattr(sensor_cfg, "randomize_placement", False)), False)
        noise = getattr(sensor_cfg, "sensor_noise", None)
        require("lidar.sensor_noise", bool(getattr(noise, "enable_sensor_noise", False)), False)
        if bad:
            raise RuntimeError("P02 RNG unaudited configuration: " + "; ".join(bad))
        return checked

    @contextmanager
    def _scope(self, stream, *coordinates):
        seed = keyed_seed(self.seed, stream, *coordinates)
        python_state, numpy_state = random.getstate(), np.random.get_state()
        try:
            with torch.random.fork_rng(devices=self._cuda_devices, enabled=True):
                random.seed(seed)
                np.random.seed(seed & 0xFFFFFFFF)
                # torch.manual_seed also touches every CUDA device. Seed only the generators
                # whose states this scope owns and restores, including the CPU generator.
                torch.random.default_generator.manual_seed(seed)
                for device in self._cuda_devices:
                    torch.cuda.default_generators[device].manual_seed(seed)
                yield
        finally:
            random.setstate(python_state)
            np.random.set_state(numpy_state)

    def _ids(self, env_ids):
        if env_ids is None:
            ids = list(range(self.num_envs))
        else:
            ids = torch.as_tensor(env_ids).detach().cpu().reshape(-1).tolist()
        if any(int(i) != i or i < 0 or i >= self.num_envs for i in ids) or len(set(ids)) != len(ids):
            raise ValueError("P02 reset/waypoint env ids must be unique valid integer rows")
        return [int(i) for i in ids]

    def _row(self, env_id):
        return torch.tensor([env_id], dtype=torch.long, device=self.device)

    def _write_selected_roots(self):
        if self._reset_env_id is None:
            if self._pending_root_envs:
                raise RuntimeError("P02 full root write during an unfinished reset transaction")
            return self._write_to_sim()
        # Original manager and task reset methods can request the same write repeatedly.
        # Delay them until every reset row's task stage is finished, then submit one union.
        self._pending_root_envs.add(self._reset_env_id)

    def _flush_selected_roots(self):
        if not self._pending_root_envs:
            return
        if bool(getattr(self._ige, "sim_has_dof", False)):
            raise RuntimeError("P02 indexed reset writes reject DOF state")
        root = self._ige.global_tensor_dict["unfolded_env_asset_state_tensor"]
        if root.shape != (self.num_envs * self._actors_per_env, 13):
            raise RuntimeError("P02 actor root layout changed after RNG attachment")
        # Retain the int32 tensor reference across unwrap and setter; Gym does not copy it.
        actor_ids = torch.tensor([env_id * self._actors_per_env + actor
                                  for env_id in sorted(self._pending_root_envs)
                                  for actor in range(self._actors_per_env)],
                                 dtype=torch.int32, device=root.device)
        ok = self._ige.gym.set_actor_root_state_tensor_indexed(
            self._ige.sim, _unwrap_tensor(root), _unwrap_tensor(actor_ids), actor_ids.numel())
        if not ok:
            raise RuntimeError("P02 indexed actor-root setter rejected reset rows")
        self._totals["indexed_root_write"] += 1
        self._pending_root_envs.clear()

    @contextmanager
    def _preserve_other_kinematics(self, env_id):
        # Original robot.update_states writes all rows, including other envs whose normal
        # post-physics derived buffers have not yet advanced. A reset must not advance them.
        robot = self.task.sim_env.robot_manager.robot
        saved = []
        for name in DERIVED_KINEMATICS:
            value = getattr(robot, name, None)
            if isinstance(value, torch.Tensor) and value.ndim > 0 and value.shape[0] == self.num_envs:
                saved.append((value, value.clone()))
        prior_reset_env = self._reset_env_id
        self._reset_env_id = env_id
        try:
            yield
        finally:
            self._reset_env_id = prior_reset_env
            for value, original in saved:
                other = torch.arange(self.num_envs, device=value.device) != env_id
                value[other] = original[other]

    def _reset_manager(self, env_ids=None, *args, **kwargs):
        ids = self._ids(env_ids)
        if any(self.pending[i] for i in ids):
            raise RuntimeError("P02 manager reset before pending task reset was consumed")
        for env_id in ids:
            self.episodes[env_id] += 1
            self.pending[env_id] = True
            with self._scope("manager_reset", self.logical_step, env_id, self.episodes[env_id]):
                with self._preserve_other_kinematics(env_id):
                    self._manager_reset(self._row(env_id), *args, **kwargs)
            self._totals["manager_reset"] += 1

    def _reset_task(self, env_ids, *args, **kwargs):
        ids = self._ids(env_ids)
        if any(not self.pending[i] for i in ids):
            raise RuntimeError("P02 task reset requires the matching manager episode token")
        prior_phase = self._phase
        self._phase = "reset"
        try:
            for env_id in ids:
                with self._scope("task_reset", self.logical_step, env_id, self.episodes[env_id]):
                    with self._preserve_other_kinematics(env_id):
                        self._task_reset(self._row(env_id), *args, **kwargs)
                self.pending[env_id] = False
                self._totals["task_reset"] += 1
        finally:
            self._phase = prior_phase
        if not any(self.pending):
            self._flush_selected_roots()

    def _sample_waypoints(self, env_ids, *args, **kwargs):
        ids = self._ids(env_ids)
        if not ids:
            return self._waypoints(torch.empty(0, dtype=torch.long, device=self.device), *args, **kwargs)
        frame = inspect.currentframe()
        try:
            caller = frame.f_back
            callsite = "%s:%d" % (caller.f_code.co_name, caller.f_lineno)
        finally:
            del frame
        rows = []
        for env_id in ids:
            episode = self.episodes[env_id]
            if episode < 0:
                raise RuntimeError("P02 waypoint requested before its manager episode token")
            step = self.logical_step
            key = (env_id, episode, step, self._phase, callsite)
            invocation = self._calls.get(key, 0)
            self._calls[key] = invocation + 1
            with self._scope("waypoint", *key, invocation):
                rows.append(self._waypoints(self._row(env_id), *args, **kwargs))
            self._totals["waypoint"] += 1
        return torch.cat(rows, dim=0)

    @contextmanager
    def step(self, logical_step):
        """Use the same exogenous full-N stream at this step in every arm."""
        self.logical_step = int(logical_step)
        if self.logical_step < 0:
            raise ValueError("P02 logical step must be nonnegative")
        with self._scope("step", self.logical_step):
            yield
        if any(self.pending):
            raise RuntimeError("P02 step ended with an unconsumed reset episode token")
        self._totals["step"] += 1

    @contextmanager
    def action(self):
        """Discard model-sampling RNG changes while retaining deterministic action outputs."""
        with self._scope("actor_discard", self.logical_step):
            yield
        self._totals["action"] += 1

    def probe(self):
        """Synthetic uniform/normal draws for evidence; does not perturb any live RNG state."""
        rows = []
        for env_id, episode in enumerate(self.episodes):
            with self._scope("probe", self.logical_step, env_id, episode):
                rows.append(torch.cat((torch.rand(2, device=self.device),
                                       torch.randn(2, device=self.device))))
        return torch.stack(rows)

    def force_reset(self, env_ids):
        """Diagnostic only: reset selected rows without an extra all-env perception update."""
        ids = self._ids(env_ids)
        rows = torch.tensor(ids, device=self.device, dtype=torch.long)
        self.task.sim_env.reset_idx(rows)
        self.task.reset_idx(rows)
        self.task._sync_target_to_sensor()
        self.task.sim_env.render(render_components="sensors")
        self._totals["forced_reset"] += len(ids)

    def metadata(self):
        return {"schema": SCHEMA, "seed": self.seed, "fixed_num_envs": self.num_envs,
                "batch_size_invariant": False, "constructor_rng": "COMMON_UNMODIFIED_INITIALIZATION",
                "logical_step": self.logical_step, "episode_id": list(self.episodes),
                "pending_task_reset": list(self.pending), "calls": dict(self._totals),
                "pending_root_envs": sorted(self._pending_root_envs),
                "checked_flags": dict(self.checked_flags),
                "unaffected_reset_rows_preserved": list(DERIVED_KINEMATICS),
                "reset_root_setter": "INSTANCE_ONLY_ONCE_PER_RESET_TRANSACTION_INDEXED_ENV_UNION_ALL_ACTORS",
                "actors_per_env": self._actors_per_env,
                "waypoint_callsite_keys": sorted({key[4] for key in self._calls}),
                "probe": "SYNTHETIC_UNIFORM2_NORMAL2_NOT_PHYSICAL_SENSOR_NOISE"}
