"""MOTAR side of the PX4 SITL bridge (docs/plans/sitl_validation_plan_v1.md, S3a/S3b). CPU only, no Isaac Gym.

Builds Policy F's observation outside the simulator with MOTAR's own production code, loaded from this tree:
  - LiDAR 72 x 4: MOTAR's Warp LiDAR kernel through MOTAR-cplanner/cplanner/motar/replay.py (CPU device), against a
    mesh of the episode's bars and the 0.20 m analytic target sphere; range limits and normalisation as production;
    sensor at the robot position with the yaw-only attitude production derives from the full quaternion;
  - RGB-D: the production renderer `NavRLTargetDetector.render_raw_rgbd` (navrl_detector.py), instantiated on the CPU
    with a stand-in warp env that carries the same merged bar mesh;
  - perception: the production `NavRLPerceptionModule.observe` (navrl_perception.py), one env per bridge;
  - policy: the real `NavRLPpoPlayerContinuous` restored from Policy F's checkpoint (tools/navrl_reflection_offline_
    audit.build_player), deterministic action.
The package `aerial_gym` is stood in by bare packages whose __path__ points at this tree (the offline audit's pattern),
so the real module files run unmodified and no simulator can be constructed. Environment: the cell receipt's NAVRL_*
snapshot is applied before any production import (navrl_perception reads it at module level).
"""
from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
CPLANNER = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-cplanner")
F_CKPT = ROOT / ("aerial_gym/rl_training/rl_games/runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/"
                 "nn/last_gen_ppo_ep_25000_rew_39.742134.pth")
TRAIN_CONFIG = ROOT / "aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml"
_STUBS = (("aerial_gym", "aerial_gym"), ("aerial_gym.task", "aerial_gym/task"),
          ("aerial_gym.task.navrl_task", "aerial_gym/task/navrl_task"), ("aerial_gym.utils", "aerial_gym/utils"),
          ("aerial_gym.control", "aerial_gym/control"), ("aerial_gym.rl_training", "aerial_gym/rl_training"),
          ("aerial_gym.rl_training.rl_games", "aerial_gym/rl_training/rl_games"),
          ("aerial_gym.config", "aerial_gym/config"), ("aerial_gym.config.task_config", "aerial_gym/config/task_config"))
_LOADED = {}


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def setup(env_snapshot):
    """Apply the NAVRL_* contract, stand in the aerial_gym packages, and import the production modules once."""
    if _LOADED:
        return _LOADED
    for k, v in env_snapshot.items():
        if k.startswith("NAVRL_"):
            os.environ[k] = str(v)
    os.environ["NAVRL_PERCEPTION_PERTURB"] = "0"
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    for name, rel in _STUBS:
        mod = sys.modules.get(name)
        if mod is None or not hasattr(mod, "__path__"):
            mod = types.ModuleType(name)
            mod.__path__ = [str(ROOT / rel)]
            sys.modules[name] = mod
    sys.modules["aerial_gym"].AERIAL_GYM_DIRECTORY = str(ROOT)
    import torch                                                       # noqa: F401
    import warp as wp
    wp.init()
    _LOADED["perception"] = _load("aerial_gym.task.navrl_task.navrl_perception",
                                  ROOT / "aerial_gym/task/navrl_task/navrl_perception.py")
    _LOADED["detector"] = _load("aerial_gym.task.navrl_task.navrl_detector",
                                ROOT / "aerial_gym/task/navrl_task/navrl_detector.py")
    _LOADED["task_config"] = _load("sitl_bridge_navrl_task_config",
                                   ROOT / "aerial_gym/config/task_config/navrl_task_config.py").task_config
    rp = _load("sitl_bridge_cplanner_replay", CPLANNER / "cplanner/motar/replay.py")
    rp.KERNEL_FILE = ROOT / "aerial_gym/sensors/warp/warp_kernels/warp_lidar_kernels.py"
    _LOADED["replay"] = rp
    _LOADED["audit"] = _load("sitl_bridge_offline_audit", ROOT / "tools/navrl_reflection_offline_audit.py")
    return _LOADED


class _WarpEnvStub:
    def __init__(self, mesh_id):
        self.CONST_WARP_MESH_ID_LIST = [mesh_id]


class Episode:
    """One env, one episode: a fixed bar layout; call `observe(state)` once per 0.1 s step from the episode's start."""

    def __init__(self, centers, halves, quats, step_dt=0.1):
        import numpy as np
        import torch
        import warp as wp
        M = _LOADED
        rp = M["replay"]
        self.torch = torch
        self.scene = rp.Scene(np.asarray(centers, np.float64), np.asarray(halves, np.float64), np.asarray(quats, np.float64))
        tc = M["task_config"]
        self.tc = tc
        self.device = torch.device("cpu")
        # the renderer passes its device to Warp, which takes a device string (production: "cuda:0")
        self.detector = M["detector"].NavRLTargetDetector(_WarpEnvStub(self.scene.mesh.id), 1, "cpu", tc.vision,
                                                          step_dt)
        self.perception = M["perception"].NavRLPerceptionModule(num_envs=1, device=self.device, cfg=tc.perception,
                                                                step_dt=step_dt, camera_cfg=tc.vision)
        self.perception.reset_idx(torch.tensor([0]))

    def lidar_m(self, pos, full_quat, target_pos):
        rp = _LOADED["replay"]
        _, sq = rp.sensor_yaw(full_quat)
        scan = self.scene.scan(pos, sq, target_pos)                         # (4, 72) normalised, < 0.2 m -> -1
        t = self.torch.from_numpy(scan).clamp(0.0, 1.0) * float(self.tc.lidar_max_range)
        return t.reshape(1, -1).float()

    def observe(self, s, step_count=None):
        """s: dict of 1-D float arrays: pos, vel, vquat, fquat, angvel, tpos, tquat (optional), prev_action,
        bounds_min, bounds_max. step_count: the perception module's global history counter before this call."""
        torch = self.torch
        T = lambda x: torch.as_tensor(x, dtype=torch.float32).reshape(1, -1)          # noqa: E731
        if step_count is not None:
            self.perception.step_count = int(step_count)
        tq = T(s["tquat"]) if s.get("tquat") is not None else None
        rgb, depth = self.detector.render_raw_rgbd(T(s["pos"]), T(s["vquat"]), T(s["tpos"]), tq)
        lidar = self.lidar_m(s["pos"], s["fquat"], s["tpos"])
        obs, diag = self.perception.observe(
            rgb=rgb, depth=depth, lidar_m=lidar, drone_pos_w=T(s["pos"]), drone_vel_w=T(s["vel"]),
            vehicle_quat=T(s["vquat"]), yaw_rate=T(s["angvel"])[:, 2] / float(self.tc.yaw_rate_max),
            previous_action=T(s["prev_action"]), max_velocity=self.tc.max_velocity,
            flight_altitude=self.tc.flight_altitude, training=False, env_bounds_min=T(s["bounds_min"]),
            env_bounds_max=T(s["bounds_max"]))
        return obs, diag, lidar


class Policy:
    """Policy F on the CPU: the real player, deterministic action, as navrl_players selects it."""

    def __init__(self, ckpt=F_CKPT):
        import torch
        audit = _LOADED["audit"]
        audit.install_cpu_only_aerial_gym_packages()
        state = torch.load(str(ckpt), map_location="cpu", weights_only=False)
        schema = audit.read_checkpoint_schema(state)
        # rl_games' restore calls torch.load without map_location; on a CPU-only process map its storages to the
        # CPU for the duration of the restore only
        original_load = torch.load

        def cpu_load(*a, **kw):
            kw.setdefault("map_location", "cpu")
            return original_load(*a, **kw)
        torch.load = cpu_load
        try:
            self.player = audit.build_player(ckpt, schema, "cpu", TRAIN_CONFIG)
        finally:
            torch.load = original_load
        self.audit = audit
        self.schema = schema

    def act(self, obs):
        return self.audit.deterministic_actions(self.player, obs)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
