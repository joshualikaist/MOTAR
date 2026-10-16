"""KROC2027 bridge between the NavRL task and ``motar_eval`` (evaluation only, off by default).

Two independent switches, both read from the environment once, at task construction:

* ``NAVRL_KROC2027_INITIAL_VIEW=visible|hidden`` (with ``NAVRL_KROC2027_INITIAL_DISTANCE_M``) replaces
  the general-spawn target draw with the Experiment A initial condition: the target 10 m away at the
  same altitude, clear line of sight, and the pursuer facing it (visible) or turned 180 degrees
  (hidden). It changes the reset distribution and nothing else.
* ``NAVRL_KROC2027_RECORDER=1`` records, per episode, the acquisition chronology, ground-truth
  visibility, the target-motion gates G1/G2, swept target clearance, speeds and the initial state,
  into ``kroc2027_episodes.jsonl`` and ``kroc2027_summary.json`` next to ``NAVRL_BULK_EVAL_JSON``.
  It reads detached tensors only, draws no randomness, writes into no task buffer, and nothing it
  computes reaches an observation, a reward or a termination (AGENTS.md observation contract).

* ``NAVRL_KROC2027_SEARCH=b1`` wraps the unchanged policy with the B1 yaw-scanning supervisor
  (``motar_eval.search_supervisor``): it rewrites the normalised action before the command is built,
  from actor-side signals only (fused measurement, track state, measured LiDAR clearance).

With no variable set, ``initial_view_from_env``, ``build_kroc_adapter`` and
``build_search_supervisor`` return ``None``, the task's guarded hooks do nothing, and
``motar_eval`` is never imported. The module itself imports
only the standard library and torch, so CPU tests load it by file path without Isaac Gym.
"""

from __future__ import annotations

import atexit
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import torch

VIEW_VAR = "NAVRL_KROC2027_INITIAL_VIEW"
DISTANCE_VAR = "NAVRL_KROC2027_INITIAL_DISTANCE_M"
RECORDER_VAR = "NAVRL_KROC2027_RECORDER"
SEARCH_VAR = "NAVRL_KROC2027_SEARCH"
PACKETS_VAR = "NAVRL_KROC2027_PACKETS"
PACKET_SCHEMA_VAR = "NAVRL_KROC2027_PACKET_SCHEMA"
FULL_PACKET_SCHEMA = "motar.bounded-executor-call.v2"
FULL_PACKET_BUNDLES = "kroc2027_motion_inputs"
PACKETS_FILE = "kroc2027_violations.jsonl"
PACKET_CAP = 200
PACKET_RING = 20
PACKET_BAR_RADIUS_M = 3.0
ON_VIOLATION_VAR = "NAVRL_KROC2027_ON_VIOLATION"
RECORDER_SCHEMA = "motar.kroc2027-recorder.v1"
EPISODES_FILE = "kroc2027_episodes.jsonl"
SUMMARY_FILE = "kroc2027_summary.json"
RESPAWN_ROUNDS = 16
# The pursuer's physical horizontal limit in the KROC2027 study; Policy F can command more.
PURSUER_REFERENCE_SPEED_MPS = 2.5


def _truthy(value) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _off(value) -> bool:
    return str(value).strip().lower() in ("", "0", "off", "none", "false")


def motion_packet_schema(environ=None):
    """Full capture is opt-in under the existing packet switch; legacy is unchanged."""
    environ = os.environ if environ is None else environ
    if not _truthy(environ.get(PACKETS_VAR, "0")):
        return "legacy_v1"
    schema = environ.get(PACKET_SCHEMA_VAR, "legacy_v1").strip().lower()
    if schema not in ("legacy_v1", "full_v2"):
        raise ValueError("%s must be legacy_v1 or full_v2" % PACKET_SCHEMA_VAR)
    return schema


def _packet_encode(value, layout=None):
    """Lossless logical tensor values plus the actual call's pre-clone layout."""
    if isinstance(value, torch.Tensor):
        info = dict(layout or {})
        return {"__tensor__": True, "dtype": info.get("dtype", str(value.dtype)),
                "device": info.get("device", str(value.device)),
                "shape": list(info.get("shape", value.shape)),
                "stride": list(info.get("stride", value.stride())),
                "storage_offset": int(info.get("storage_offset", value.storage_offset())),
                "values": value.detach().cpu().reshape(-1).tolist()}
    if isinstance(value, dict):
        return {key: _packet_encode(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_packet_encode(item) for item in value]
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    raise TypeError("unsupported motion packet value: %s" % type(value).__name__)


def decode_motion_packet_value(value):
    """Decode tensors on CPU, retaining dtype, logical shape, stride and storage offset.

    Recorded CUDA/TF32 metadata describes the original execution; CPU replay is a diagnostic,
    not an assertion that CUDA numerical choices are reproduced by CPU.
    """
    if isinstance(value, dict) and value.get("__tensor__") is True:
        name = value["dtype"].split(".")[-1]
        dtype = getattr(torch, name, None)
        if not isinstance(dtype, torch.dtype):
            raise ValueError("unsupported packet tensor dtype %r" % value["dtype"])
        shape, stride = tuple(value["shape"]), tuple(value["stride"])
        offset = int(value["storage_offset"])
        if len(shape) != len(stride) or offset < 0 or any(x < 0 for x in shape + stride):
            raise ValueError("invalid motion packet tensor layout")
        flat = torch.tensor(value["values"], dtype=dtype)
        expected = math.prod(shape)
        if flat.numel() != expected:
            raise ValueError("motion packet tensor value count differs from its shape")
        size = offset + 1 + sum((d - 1) * s for d, s in zip(shape, stride)) if expected else offset
        storage = torch.empty(size, dtype=dtype)
        tensor = torch.as_strided(storage, shape, stride, offset)
        if expected:
            indices = torch.full(shape, offset, dtype=torch.long)
            for axis, (length, step) in enumerate(zip(shape, stride)):
                view = [1] * len(shape)
                view[axis] = length
                indices += torch.arange(length).reshape(view) * step
            storage[indices.reshape(-1)] = flat
        return tensor
    if isinstance(value, dict):
        return {key: decode_motion_packet_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [decode_motion_packet_value(item) for item in value]
    return value


def encode_motion_call_bundle(capture):
    """Serialize the actual full-batch call; never regenerate inputs or re-run an executor."""
    required = {"old_xy", "current_velocity", "desired_velocity", "speed_limit", "dt", "bars_xy",
                "lo", "hi", "clearance", "turn_sign", "max_accel", "max_turn_rate", "lookahead_s",
                "bars_half_extents_xy", "exact_aabb_clearance", "hard_epsilon_m", "return_certificate",
                "certificate_row_ids", "distance_backend"}
    if not isinstance(capture, dict) or capture.get("schema") != FULL_PACKET_SCHEMA:
        raise RuntimeError("full_v2 needs an actual bounded executor call capture")
    kwargs = capture.get("kwargs", {})
    missing = required - set(kwargs)
    layouts = capture.get("input_layouts", {})
    layout_fields = {"shape", "stride", "storage_offset", "dtype", "device"}
    missing_layouts = {name for name, value in kwargs.items()
                       if isinstance(value, torch.Tensor)
                       and (name not in layouts or not layout_fields <= set(layouts[name]))}
    metadata = capture.get("metadata", {})
    missing_metadata = {"model_id", "behavior_level", "distance_backend", "torch_version", "device",
                        "task_step", "matmul_allow_tf32", "cudnn_allow_tf32", "float32_matmul_precision",
                        "env_ids", "obstacle_slots"} - set(metadata)
    result = capture.get("result", {})
    missing_result = {"new_xy", "new_velocity", "steered", "immediate_feasible", "certificate"} - set(result)
    if missing or missing_layouts or missing_metadata or missing_result or result.get("certificate") is None:
        raise RuntimeError("incomplete full_v2 capture: kwargs=%s layouts=%s metadata=%s result=%s"
                           % (sorted(missing), sorted(missing_layouts), sorted(missing_metadata),
                              sorted(missing_result)))
    n = kwargs["old_xy"].shape[0]
    if kwargs["bars_xy"].shape[0] != n or metadata["env_ids"].numel() != n:
        raise RuntimeError("full_v2 batch or environment mapping is inconsistent")
    b = kwargs["bars_xy"].shape[1]
    if tuple(metadata["obstacle_slots"].shape) not in ((b,), (n, b)):
        raise RuntimeError("full_v2 obstacle slot mapping does not cover the full ordered batch")
    if metadata["distance_backend"] != kwargs["distance_backend"]:
        raise RuntimeError("full_v2 metadata and actual distance backend disagree")
    for name, value in kwargs.items():
        if isinstance(value, torch.Tensor) and (list(value.shape) != list(layouts[name]["shape"])
                                                or str(value.dtype) != layouts[name]["dtype"]):
            raise RuntimeError("full_v2 saved tensor differs from the actual input layout: %s" % name)
    return {"schema": FULL_PACKET_SCHEMA,
            "kwargs": {key: _packet_encode(value, layouts.get(key)) for key, value in kwargs.items()},
            "input_layouts": _packet_encode(layouts), "result": _packet_encode(result),
            "metadata": _packet_encode(metadata),
            "replay_scope": "CPU_DIRECT_DIAGNOSTIC_NOT_CUDA_BITWISE_GUARANTEE"}


def load_motion_call_bundle(path, expected_sha256):
    """Read a content-addressed full call and reject changed bytes before CPU decoding."""
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise ValueError("motion input bundle SHA-256 mismatch")
    bundle = json.loads(raw)
    if bundle.get("schema") != FULL_PACKET_SCHEMA:
        raise ValueError("unsupported motion input bundle schema")
    return decode_motion_packet_value(bundle)


def initial_view_from_env(environ=None):
    """The Experiment A initial-view spec, or ``None`` when the switch is off."""
    environ = os.environ if environ is None else environ
    view = environ.get(VIEW_VAR, "")
    if _off(view):
        return None
    from motar_eval.initial_view import InitialViewSpec

    return InitialViewSpec(view=view.strip().lower(), distance_m=float(environ.get(DISTANCE_VAR, "10")))


def sample_initial_view_for_task(task, env_ids, b_min, b_max, spec, footprint_rule):
    """Draw the Experiment A target for ``env_ids`` and set the pursuer yaw; return the goal [n, 3].

    ``footprint_rule`` is navrl_task's ``_spawn_footprint_clearance_accepted``: the same per-obstacle
    surface-clearance test the drone spawn uses, applied to the target in every arm. Pursuers with
    no admissible target are respawned through the task's own spawn sampler, up to
    ``RESPAWN_ROUNDS`` times; after that the reset fails closed.
    """
    from motar_eval.initial_view import sample_initial_view

    device = task.device
    n = len(env_ids)
    lo_slot = task._solid_obstacle_offset
    hi_slot = task._bar_offset + task.n_bars_active
    margin = float(task._spawn_required_surface_margin_m())
    camera_t = tuple(float(v) for v in getattr(task.vis_cfg, "camera_translation", (0.10, 0.0, 0.03)))
    goal = torch.zeros((n, 3), device=device)
    rounds = torch.zeros(n, dtype=torch.long, device=device)
    respawns = torch.zeros(n, dtype=torch.long, device=device)
    bearing = torch.zeros(n, device=device)
    pending = torch.ones(n, dtype=torch.bool, device=device)
    for attempt in range(RESPAWN_ROUNDS + 1):
        rows = pending.nonzero(as_tuple=False).squeeze(-1)
        ids = env_ids[rows]
        pursuer = task.obs_dict["robot_position"][ids].clone()
        center = task.obs_dict["obstacle_position"][ids, lo_slot:hi_slot, :]
        half = task.obs_dict["asset_collision_half_extents"][ids, lo_slot:hi_slot, :]

        def clearance_ok(candidates, sub, center=center, half=half):
            m, k = candidates.shape[0], candidates.shape[1]
            return footprint_rule(
                candidates.reshape(m * k, 2),
                center[sub, :, 0:2].repeat_interleave(k, dim=0),
                half[sub, :, 0:2].repeat_interleave(k, dim=0),
                margin,
            ).reshape(m, k)

        out = sample_initial_view(
            pursuer, center, half, b_min[rows, 0:2], b_max[rows, 0:2], camera_t, spec,
            clearance_ok, lambda shape: torch.rand(shape, device=device),
        )
        accepted = out["accepted"]
        done_rows = rows[accepted]
        goal[done_rows] = out["target_xyz"][accepted].to(goal.dtype)
        bearing[done_rows] = out["bearing"][accepted].to(bearing.dtype)
        rounds[done_rows] = out["rounds_used"][accepted]
        respawns[done_rows] = attempt
        task.obs_dict["robot_orientation"][env_ids[done_rows]] = out["quat_xyzw"][accepted].to(
            task.obs_dict["robot_orientation"].dtype
        )
        pending[done_rows] = False
        if not bool(pending.any()):
            break
        if attempt < RESPAWN_ROUNDS:
            task._randomize_general_drone_spawn(env_ids[pending.nonzero(as_tuple=False).squeeze(-1)])
    if bool(pending.any()):
        raise RuntimeError(
            "KROC2027 initial view: no admissible %.1f m target for %d envs after %d respawns"
            % (spec.distance_m, int(pending.sum()), RESPAWN_ROUNDS)
        )
    goal[:, 2] = float(task.task_config.flight_altitude)
    stats = getattr(task, "_kroc_initial_view_stats", None)
    if stats is None:
        stats = {
            "rounds": torch.zeros(task.num_envs, dtype=torch.long, device=device),
            "respawns": torch.zeros(task.num_envs, dtype=torch.long, device=device),
            "bearing": torch.zeros(task.num_envs, device=device),
        }
        task._kroc_initial_view_stats = stats
    stats["rounds"][env_ids] = rounds
    stats["respawns"][env_ids] = respawns
    stats["bearing"][env_ids] = bearing
    return goal


def build_kroc_adapter(task):
    """The recorder when ``NAVRL_KROC2027_RECORDER`` is on, else ``None``."""
    if not _truthy(os.environ.get(RECORDER_VAR, "0")):
        return None
    return KrocRecorder(task)


def build_search_supervisor(task, environ=None):
    """The B1 yaw-scanning supervisor when ``NAVRL_KROC2027_SEARCH=b1``, else ``None``."""
    environ = os.environ if environ is None else environ
    mode = environ.get(SEARCH_VAR, "")
    if _off(mode):
        return None
    if mode.strip().lower() != "b1":
        raise RuntimeError("%s must be b1 or off, got %r" % (SEARCH_VAR, mode))
    from motar_eval.search_supervisor import SearchSpec, YawScanSupervisor

    supervisor = YawScanSupervisor(
        task.num_envs, task.device,
        yaw_rate_max_rad_s=float(task.task_config.yaw_rate_max),
        hard_margin_m=float(task.speed_governor_cfg.hard_margin_m),
        spec=SearchSpec(dt_s=float(task.step_dt)),
    )
    return TaskSearch(task, supervisor)


class TaskSearch:
    """Feeds the supervisor the actor-side signals the policy itself observes; nothing else."""

    def __init__(self, task, supervisor):
        self.task = task
        self.supervisor = supervisor

    def apply(self, actions):
        """Start of step, after the observation is final: the actions that will be executed."""
        t = self.task
        fused = t._visible_now.detach()
        track = t.perception.tracker.active.detach()
        scan = getattr(t.perception, "last_scan_nearest", None)
        if scan is None:
            clearance = torch.full((t.num_envs,), math.nan, device=t.device)
        else:
            clearance = scan.detach().amin(dim=1)
        return self.supervisor.step(actions, fused, track, clearance)

    def on_reset(self, env_ids):
        self.supervisor.on_reset(env_ids)

    @property
    def active(self):
        return self.supervisor.active

    def episode_records(self, env_ids):
        return self.supervisor.episode_records(env_ids)


def _target_motion_module():
    """The task's bounded executor, without importing the aerial_gym package in CPU tests."""
    name = "aerial_gym.task.navrl_task.target_motion"
    if name in sys.modules:
        return sys.modules[name]
    import importlib.util

    spec = importlib.util.spec_from_file_location("_kroc2027_target_motion",
                                                  Path(__file__).resolve().parent / "target_motion.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _module_origin(name):
    module = sys.modules.get(name)
    return str(Path(module.__file__).resolve()) if module is not None and getattr(module, "__file__", None) else ""


class KrocRecorder:
    """Per-episode KROC2027 records. Read-only with respect to the task (see the module docstring)."""

    def __init__(self, task):
        from motar_eval import contracts, events, geometry, scene_record

        self._contracts, self._events_mod = contracts, events
        self._geometry, self._scene = geometry, scene_record
        self.task = task
        n, device = task.num_envs, task.device
        self.num_envs, self.device = n, device
        output = os.environ.get("NAVRL_BULK_EVAL_JSON", "").strip()
        if not output:
            raise RuntimeError("NAVRL_KROC2027_RECORDER=1 needs NAVRL_BULK_EVAL_JSON for its sidecar files")
        self.cell_dir = Path(output).resolve().parent
        self.cell_dir.mkdir(parents=True, exist_ok=True)
        self.episodes_path = self.cell_dir / EPISODES_FILE
        self.summary_path = self.cell_dir / SUMMARY_FILE
        self.on_violation = os.environ.get(ON_VIOLATION_VAR, "stop").strip().lower()
        if self.on_violation not in ("stop", "record"):
            raise RuntimeError("%s must be stop or record" % ON_VIOLATION_VAR)
        self.context = {
            "cell_id": self.cell_dir.name,
            "run_id": os.environ.get("NAVRL_KROC2027_RUN_ID", ""),
            "arm": os.environ.get("NAVRL_KROC2027_ARM", ""),
            "seed": int(os.environ.get("NAVRL_SEED", "-1")),
            "bars": int(os.environ.get("NAVRL_NUM_BARS", str(getattr(task, "n_bars_active", -1)))),
            "target_behavior_level": os.environ.get("NAVRL_TARGET_BEHAVIOR_LEVEL", ""),
            "initial_view": os.environ.get(VIEW_VAR, "off"),
            "initial_distance_m": os.environ.get(DISTANCE_VAR, ""),
            "protocol_sha256": os.environ.get("NAVRL_KROC2027_PROTOCOL_SHA256", ""),
            "checkpoint_sha256": os.environ.get("NAVRL_EVAL_CHECKPOINT", ""),
        }
        vis = task.vis_cfg
        self.hfov = math.radians(float(vis.detector_hfov_deg))
        self.vfov = math.radians(float(getattr(vis, "detector_vfov_deg", 58.0)))
        self.max_range = float(getattr(vis, "detector_max_range", 20.0))
        self.camera_t = tuple(float(v) for v in getattr(vis, "camera_translation", (0.10, 0.0, 0.03)))
        self.proxy_radius = float(getattr(vis, "camera_target_radius", 0.15))
        self.dt = float(task.step_dt)
        self.motion = contracts.MotionGateMonitor(
            n, device, limits=contracts.MotionLimits(dt_s=self.dt), context=self._env_context
        )
        self.events = events.AcquisitionEventMonitor(n, device)
        self.episode_index = [0] * n
        self.initial = [None] * n
        self.records = 0
        self._handle = open(self.episodes_path, "w", encoding="utf-8")
        f32 = torch.float32

        def zeros(dtype=f32):
            return torch.zeros(n, dtype=dtype, device=device)

        self._prev_target = torch.zeros(n, 2, device=device)
        self._prev_robot = torch.zeros(n, 2, device=device)
        self._has_prev_robot = zeros(torch.bool)
        self._swept_min = torch.full((n,), math.inf, device=device)
        self._target_speed_max = zeros()
        self._capture_dist_min = torch.full((n,), math.inf, device=device)
        self._steps = zeros(torch.long)
        self._speed = {key: zeros() for key in (
            "requested_sum", "requested_max", "commanded_sum", "commanded_max",
            "actual_sum", "actual_max", "displacement_max", "search_xy_max", "search_yaw_max",
            "search_body_yaw_rate_max")}
        self._search_checks = {key: zeros(torch.long) for key in (
            "feedback_xy", "feedback_yaw", "suppressed_yaw", "active_while_acquired")}
        self.search_counts = {"episodes": 0, "episodes_with_search": 0, "invariant_violations": 0,
                              "trigger_violations": 0, "feedback_violations": 0,
                              "suppression_violations": 0, "return_violations": 0}
        self._over = {key: zeros(torch.long) for key in ("commanded", "actual", "displacement")}
        self.swept = {"judged_steps": 0, "violations": 0, "wall_violations": 0,
                      "min_clearance_m": math.inf, "first_violation": None}
        self.initial_view_counts = {"episodes": 0, "rounds": {}, "respawns": {},
                                    "initially_in_view": 0, "initially_out_of_view": 0,
                                    "failed_closed": 0}
        self.packets_enabled = _truthy(os.environ.get(PACKETS_VAR, "0"))
        self.packet_schema = motion_packet_schema()
        if self.packets_enabled and self.packet_schema == "full_v2":
            self.context["cfg_target_distance_backend"] = str(getattr(
                task, "_target_distance_backend", "NOT_RECORDED"))
            self.context["target_motion_model_id"] = str(getattr(
                task, "_target_motion_model", "NOT_RECORDED"))
        self.packet_count = 0
        self._snapshot = None
        if self.packets_enabled:
            self.packets_path = self.cell_dir / PACKETS_FILE
            self._packet_handle = open(self.packets_path, "w", encoding="utf-8")
            self._ring = torch.full((n, PACKET_RING, 2), math.nan, device=device)
        self._finalized = False
        self.import_origins = {
            "aerial_gym": _module_origin("aerial_gym"),
            "motar_eval": _module_origin("motar_eval"),
            "kroc_eval_adapter": str(Path(__file__).resolve()),
        }
        atexit.register(self._finalize_at_exit)

    # ------------------------------------------------------------ helpers
    def _env_context(self, env):
        seed, bars = self.context["seed"], self.context["bars"]
        key = self._scene.episode_key(max(seed, 0), max(bars, 0), env, self.episode_index[env])
        return {"episode_key": key, "cell_id": self.context["cell_id"]}

    def _obstacles(self):
        t = self.task
        lo, hi = t._solid_obstacle_offset, t._bar_offset + t.n_bars_active
        return (t.obs_dict["obstacle_position"][:, lo:hi, :].detach(),
                t.obs_dict["asset_collision_half_extents"][:, lo:hi, :].detach())

    def _geometric_view(self, env_ids=None):
        t, g = self.task, self._geometry
        pos = t.obs_dict["robot_position"].detach()
        quat = t.obs_dict["robot_vehicle_orientation"].detach()
        target = t.target_position.detach()
        center, half = self._obstacles()
        if env_ids is not None:
            pos, quat, target, center, half = pos[env_ids], quat[env_ids], target[env_ids], \
                center[env_ids], half[env_ids]
        eye = g.camera_eye(pos, quat, self.camera_t)
        in_fov = g.in_camera_frustum(eye, quat, target, self.hfov, self.vfov, self.max_range)
        clear = g.los_clear(eye, target, center, half)
        return in_fov, in_fov & clear, clear

    # ------------------------------------------------------------ hooks
    def on_reset(self, env_ids):
        """End of reset_idx: the starting state of a new episode is final here."""
        t = self.task
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if env_ids.numel() == 0:
            return
        target_xy = t.target_position[:, 0:2].detach()
        target_v = t.target_vel_w[:, 0:2].detach()
        self.motion.reset(env_ids, target_xy, target_v)
        self._prev_target[env_ids] = target_xy[env_ids]
        robot_xy = t.obs_dict["robot_position"][:, 0:2].detach()
        self._prev_robot[env_ids] = robot_xy[env_ids]
        self._has_prev_robot[env_ids] = True
        self._swept_min[env_ids] = math.inf
        self._target_speed_max[env_ids] = 0.0
        self._capture_dist_min[env_ids] = math.inf
        self._steps[env_ids] = 0
        for store in self._speed.values():
            store[env_ids] = 0.0
        for store in self._over.values():
            store[env_ids] = 0
        for store in self._search_checks.values():
            store[env_ids] = 0
        if self.packets_enabled:
            self._ring[env_ids] = math.nan
        in_fov, visible, clear = self._geometric_view(env_ids)
        center, half = self._obstacles()
        stats = getattr(t, "_kroc_initial_view_stats", None)
        host = {
            "pos": t.obs_dict["robot_position"][env_ids].detach().double().cpu().tolist(),
            "quat": t.obs_dict["robot_vehicle_orientation"][env_ids].detach().double().cpu().tolist(),
            "target": t.target_position[env_ids].detach().double().cpu().tolist(),
            "tv": t.target_vel_w[env_ids, 0:2].detach().double().cpu().tolist(),
            "nominal": t._tm_speed[env_ids].detach().double().cpu().tolist()
            if hasattr(t, "_tm_speed") else [0.0] * len(env_ids),
            "in_fov": in_fov.cpu().tolist(), "visible": visible.cpu().tolist(),
            "clear": clear.cpu().tolist(),
            "center": center[env_ids].double().cpu().tolist(),
            "half": half[env_ids].double().cpu().tolist(),
        }
        if stats is not None:
            host["rounds"] = stats["rounds"][env_ids].cpu().tolist()
            host["respawns"] = stats["respawns"][env_ids].cpu().tolist()
        for row, env in enumerate(env_ids.cpu().tolist()):
            quat = host["quat"][row]
            yaw = math.atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                             1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
            key = self._env_context(env)["episode_key"]
            scene = self._scene.scene_hash(host["center"][row], host["half"][row])
            treatment = {"view": self.context["initial_view"],
                         "distance_m": self.context["initial_distance_m"],
                         "arm": self.context["arm"],
                         "target_behavior_level": self.context["target_behavior_level"]}
            record = self._scene.initial_state_record(
                key, scene, host["pos"][row], yaw, host["target"][row], host["tv"][row],
                host["nominal"][row], treatment)
            record["initial_geo_in_fov"] = bool(host["in_fov"][row])
            record["initial_geo_visible"] = bool(host["visible"][row])
            record["initial_los_clear"] = bool(host["clear"][row])
            if stats is not None:
                record["initial_view_rounds"] = int(host["rounds"][row])
                record["initial_view_respawns"] = int(host["respawns"][row])
            self.initial[env] = record

    def after_target_advance(self):
        """After _advance_target: judge the target step (G1, G2, swept clearance, wall)."""
        t, g = self.task, self._geometry
        target_xy = t.target_position[:, 0:2].detach()
        target_v = t.target_vel_w[:, 0:2].detach()
        nominal = t._tm_speed.detach() if hasattr(t, "_tm_speed") else torch.zeros(self.num_envs, device=self.device)
        violated = self.motion.step(target_xy, target_v, nominal)
        center, half = self._obstacles()
        clearance = g.swept_proxy_clearance(self._prev_target, target_xy, center[..., 0:2], half[..., 0:2])
        lo = t.obs_dict["env_bounds_min"][:, 0:2]
        hi = t.obs_dict["env_bounds_max"][:, 0:2]
        outside = ((target_xy < lo) | (target_xy > hi)).any(dim=1)
        hits = clearance < self.proxy_radius
        step_speed = (target_xy - self._prev_target).norm(dim=1) / self.dt
        self._target_speed_max = torch.maximum(self._target_speed_max, step_speed)
        self._swept_min = torch.minimum(self._swept_min, clearance)
        self._prev_target = target_xy.clone()
        n_hits, n_out = int(hits.sum().item()), int(outside.sum().item())
        self.swept["judged_steps"] += self.num_envs
        finite = clearance[torch.isfinite(clearance)]
        if finite.numel():
            self.swept["min_clearance_m"] = min(self.swept["min_clearance_m"], float(finite.min().item()))
        if n_hits or n_out:
            self.swept["violations"] += n_hits
            self.swept["wall_violations"] += n_out
            if self.swept["first_violation"] is None:
                env = int(torch.nonzero(hits | outside, as_tuple=False)[0, 0].item())
                self.swept["first_violation"] = {
                    "env_id": env, "clearance_m": float(clearance[env].item()),
                    "outside_arena": bool(outside[env].item()),
                    "target_xy": target_xy[env].tolist(), "context": self._env_context(env)}
        if self.packets_enabled and self.packet_count < PACKET_CAP:
            if self.packet_schema == "full_v2" and (violated or n_hits or n_out):
                bad = torch.nonzero(hits | outside, as_tuple=False).flatten()[: PACKET_CAP - self.packet_count]
                event_type = "swept_geometry"
                if bad.numel() == 0:
                    # The motion monitor reports a batch trigger, not every offending row. One
                    # context row references all original inputs/outputs without inventing blame.
                    capture = getattr(self.task, "_mt_precision_last_call", None)
                    encode_motion_call_bundle(capture)  # fail closed before reading the mapping
                    bad = capture["metadata"]["env_ids"][:1]
                    event_type = "motion_gate_batch"
                self._write_packets(bad, target_xy, target_v, clearance,
                                    event_type=event_type, batch_motion_gate_trigger=bool(violated))
            elif n_hits or n_out:
                bad = torch.nonzero(hits | outside, as_tuple=False).flatten()[: PACKET_CAP - self.packet_count]
                self._write_packets(bad, target_xy, target_v, clearance)
        if self.packets_enabled:
            self._ring = torch.roll(self._ring, shifts=-1, dims=1)
            self._ring[:, -1] = target_xy
        if (violated or n_hits or n_out) and self.on_violation == "stop":
            self.finalize(reason="motion contract violated")
            raise RuntimeError("KROC2027 target-motion contract violated; see %s" % self.summary_path)

    def _snapshot_target_inputs(self):
        """Executor inputs for the next target step; the target state does not change in between."""
        if self.packet_schema == "full_v2":
            # Captured at the actual executor call, rather than reconstructed at observation time.
            return
        t = self.task
        grab = lambda name: getattr(t, name).detach().clone() if hasattr(t, name) else None  # noqa: E731
        self._snapshot = {
            "pos": t.target_position[:, 0:2].detach().clone(),
            "vel": t.target_vel_w[:, 0:2].detach().clone(),
            "speed": grab("_tm_speed"), "pattern": grab("_tm_pattern"),
            "waypoint": grab("_tm_waypoint"), "cv_vel": grab("_tm_cv_vel"), "avoid": grab("_tm_avoid_sign"),
        }

    def _write_packets(self, env_ids, new_xy, new_v, clearance, *, event_type="swept_geometry",
                       batch_motion_gate_trigger=False):
        """One violation packet per env: inputs, the executor's own flag and a deterministic replay."""
        if self.packet_schema == "full_v2":
            self._write_full_packets(env_ids, new_xy, new_v, clearance, event_type=event_type,
                                     batch_motion_gate_trigger=batch_motion_gate_trigger)
            return
        t, s = self.task, self._snapshot
        if s is None:
            return
        tm = _target_motion_module()
        cfg = t.tm
        old, v0, speed = s["pos"][env_ids], s["vel"][env_ids], s["speed"][env_ids]
        desired = s["cv_vel"][env_ids].clone()
        waypoint_rows = s["pattern"][env_ids] == 1
        if bool(waypoint_rows.any()):
            to_wp = s["waypoint"][env_ids][waypoint_rows] - old[waypoint_rows]
            desired[waypoint_rows] = to_wp / to_wp.norm(dim=1, keepdim=True).clamp(min=1e-6) \
                * speed[waypoint_rows].unsqueeze(1)
        bars = t.obs_dict["obstacle_position"][env_ids, t._bar_offset:t._bar_offset + t.n_bars_active, 0:2]
        half = t.obs_dict["asset_collision_half_extents"][env_ids, t._bar_offset:t._bar_offset + t.n_bars_active, 0:2]
        m = float(t.cur.wall_margin)
        lo = t.obs_dict["env_bounds_min"][env_ids, 0:2] + m
        hi = t.obs_dict["env_bounds_max"][env_ids, 0:2] - m
        clear = float(t._target_planner_clearance())
        accel = torch.full_like(speed, float(cfg.max_accel))
        turn = torch.full_like(speed, math.radians(float(cfg.max_turn_rate_deg)))
        lookahead = float(cfg.avoidance_lookahead_s)
        replay_xy, replay_v, _, immediate, cert = tm.bounded_drone_target_step(
            old, v0, desired, speed, self.dt, bars, lo, hi, clear, s["avoid"][env_ids], accel, turn, lookahead,
            None, return_certificate=True)
        # Every candidate's first step, as the executor builds and screens them.
        angles = torch.tensor([math.radians(a) for a in tm.BOUNDED_TURN_ANGLES_DEG], device=old.device, dtype=old.dtype)
        base_norm = desired.norm(dim=1, keepdim=True)
        fallback = torch.zeros_like(desired)
        fallback[:, 0] = 1.0
        base = torch.where(base_norm > 1e-6, desired / base_norm.clamp(min=1e-6), fallback)
        cos, sin = torch.cos(angles).view(1, -1), torch.sin(angles).view(1, -1)
        dirs = torch.stack((base[:, 0:1] * cos - base[:, 1:2] * sin, base[:, 0:1] * sin + base[:, 1:2] * cos), dim=2)
        scales = torch.tensor((1.0, 0.5, 0.25), device=old.device, dtype=old.dtype)
        cands = (dirs.unsqueeze(1) * scales.view(1, -1, 1, 1) * speed.view(-1, 1, 1, 1)).reshape(len(env_ids), -1, 2)
        cands = torch.cat((cands, torch.zeros(len(env_ids), 1, 2, device=old.device, dtype=old.dtype)), dim=1)
        k = cands.shape[1]
        rep = lambda x: x.unsqueeze(1).expand(-1, k, *x.shape[1:]).reshape(-1, *x.shape[1:])  # noqa: E731
        v1 = tm.limit_planar_velocity(rep(v0), cands.reshape(-1, 2), rep(speed), self.dt, rep(accel), rep(turn))
        p1 = (rep(old) + v1 * self.dt).reshape(len(env_ids), k, 2)
        centre = torch.cdist(p1, bars).amin(dim=2) if bars.shape[1] else torch.full(p1.shape[:2], math.inf)
        inside = ((p1 >= lo.unsqueeze(1)) & (p1 <= hi.unsqueeze(1))).all(dim=2)
        first_safe = (centre >= clear + 1e-4) & inside
        flag = getattr(t, "_tm_last_step_feasible", None)
        host = lambda x: x.detach().double().cpu().tolist()  # noqa: E731
        for row, env in enumerate(env_ids.tolist()):
            near = ((bars[row] - old[row]).norm(dim=1) <= PACKET_BAR_RADIUS_M).nonzero(as_tuple=False).flatten()
            ring = [p for p in host(self._ring[env]) if not math.isnan(p[0])]
            packet = {
                "context": self._env_context(env), "monitor_step": self.motion.steps,
                "pattern": int(s["pattern"][env].item()), "speed_mps": float(speed[row].item()),
                "old_xy": host(old[row]), "old_v": host(v0[row]), "desired_v": host(desired[row]),
                "new_xy": host(new_xy[env]), "new_v": host(new_v[env]),
                "swept_clearance_m": float(clearance[env].item()),
                "task_step_feasible_flag": bool(flag[env].item()) if flag is not None else None,
                "replay": {"xy": host(replay_xy[row]), "immediate_feasible": bool(immediate[row].item()),
                           "selected_index": int(cert["selected_index"][row].item()),
                           "candidate_count": int(cert["candidate_count"]),
                           "safe_prefix_steps": int(cert["safe_prefix_steps"][row].item()),
                           "horizon_steps": int(cert["horizon_steps"]),
                           "full_horizon_safe": bool(cert["full_horizon_safe"][row].item()),
                           "matches_executed": bool((replay_xy[row] - new_xy[env]).norm().item() < 1e-4)},
                "candidates_first_step_safe": int(first_safe[row].sum().item()),
                "candidates_best_first_step_centre_m": float(centre[row].max().item()),
                "planner_clearance_m": clear, "wall_margin_m": m,
                "bounds_lo": host(lo[row]), "bounds_hi": host(hi[row]),
                "bars_within_3m": {"centre": host(bars[row][near]), "half": host(half[row][near])},
                "recent_positions": ring,
            }
            self._packet_handle.write(json.dumps(packet, sort_keys=True) + "\n")
        self._packet_handle.flush()
        self.packet_count += len(env_ids)

    def _write_full_packets(self, env_ids, new_xy, new_v, clearance, *, event_type="swept_geometry",
                            batch_motion_gate_trigger=False):
        """One shared full-batch bundle per call, with small per-violation row references.

        No executor is called here: certificates and outputs are those of the original call.
        Historical legacy packets lack these full inputs and cannot be upgraded by inference.
        """
        capture = getattr(self.task, "_mt_precision_last_call", None)
        bundle = encode_motion_call_bundle(capture)
        env_map = capture["metadata"]["env_ids"].detach().cpu().tolist()
        if len(set(env_map)) != len(env_map):
            raise RuntimeError("full_v2 executor environment mapping contains duplicates")
        rows = {int(env): row for row, env in enumerate(env_map)}
        task_step = getattr(self.task, "num_task_steps", None)
        if task_step is not None and capture["metadata"].get("task_step") != int(task_step):
            raise RuntimeError("full_v2 executor capture belongs to a different task step")
        requested_envs = env_ids.detach().cpu().tolist()
        for env in requested_envs:
            if env not in rows:
                raise RuntimeError("violating environment was absent from the actual executor call")
            row = rows[env]
            # Nonfinite states are themselves a motion-gate counterexample. Preserve matching
            # NaNs for diagnosis rather than treating the same recorded NaN as a stale call.
            if not torch.allclose(capture["result"]["new_xy"][row], new_xy[env],
                                  rtol=0.0, atol=0.0, equal_nan=True) or not torch.allclose(
                    capture["result"]["new_velocity"][row], new_v[env],
                    rtol=0.0, atol=0.0, equal_nan=True):
                raise RuntimeError("full_v2 captured result differs from the executed target state")
        raw = (json.dumps(bundle, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        digest = hashlib.sha256(raw).hexdigest()
        relative = Path(FULL_PACKET_BUNDLES) / (digest + ".json")
        path = self.cell_dir / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with open(path, "xb") as handle:
                handle.write(raw)
        except FileExistsError:
            if path.read_bytes() != raw:
                raise RuntimeError("existing motion input bundle differs from its content hash")
        n = len(env_map)
        certificate = capture["result"]["certificate"]

        def selected(value, row):
            if isinstance(value, torch.Tensor) and value.ndim > 0 and value.shape[0] == n:
                return _packet_encode(value[row])
            if isinstance(value, dict):
                return {key: selected(item, row) for key, item in value.items()}
            return _packet_encode(value)

        for env in requested_envs:
            row = rows[env]
            packet = {
                "schema": "motar.kroc2027-violation.full_v2",
                "event_type": event_type,
                "batch_motion_gate_trigger": bool(batch_motion_gate_trigger),
                "row_violation_status": ("NOT_RECORDED" if event_type == "motion_gate_batch"
                                         else "SELECTED_BY_GEOMETRY_MONITOR"),
                "context": self._env_context(env), "monitor_step": self.motion.steps,
                "env_id": env, "executor_row_index": row,
                "bundle_file": relative.as_posix(), "bundle_sha256": digest,
                "model_id": capture["metadata"]["model_id"],
                "distance_backend": capture["metadata"]["distance_backend"],
                "full_batch_shape": list(capture["kwargs"]["old_xy"].shape),
                "full_obstacle_shape": list(capture["kwargs"]["bars_xy"].shape),
                "selected_certificate": selected(certificate, row),
                "executed_xy": _packet_encode(new_xy[env]),
                "executed_velocity": _packet_encode(new_v[env]),
                "swept_clearance_m": float(clearance[env].item()),
                "capture_complete": True,
                "replay_status": "NOT_RUN",
            }
            self._packet_handle.write(json.dumps(packet, sort_keys=True) + "\n")
        self._packet_handle.flush()
        self.packet_count += len(env_ids)

    def after_physics(self, actions, command):
        """After sim_env.step: requested, commanded and actual horizontal pursuer speed."""
        t = self.task
        vmax = float(t.task_config.max_velocity)
        requested = (torch.clamp(actions[:, 0:2].detach(), -1.0, 1.0) * vmax).norm(dim=1)
        commanded = command[:, 0:2].detach().norm(dim=1)
        actual = t.obs_dict["robot_linvel"][:, 0:2].detach().norm(dim=1)
        robot_xy = t.obs_dict["robot_position"][:, 0:2].detach()
        disp = torch.where(self._has_prev_robot, (robot_xy - self._prev_robot).norm(dim=1) / self.dt,
                           torch.zeros_like(actual))
        self._prev_robot = robot_xy.clone()
        self._has_prev_robot[:] = True
        ref = PURSUER_REFERENCE_SPEED_MPS + 1e-6
        for name, value in (("requested", requested), ("commanded", commanded), ("actual", actual)):
            self._speed[name + "_sum"] += value
            self._speed[name + "_max"] = torch.maximum(self._speed[name + "_max"], value)
        self._speed["displacement_max"] = torch.maximum(self._speed["displacement_max"], disp)
        search = getattr(t, "_kroc_search", None)
        if search is not None:
            active = search.active
            yaw = command[:, 3].detach().abs()
            body = t.obs_dict["robot_body_angvel"][:, 2].detach().abs() \
                if "robot_body_angvel" in t.obs_dict else torch.zeros_like(yaw)
            for name, value in (("search_xy_max", commanded), ("search_yaw_max", yaw),
                                ("search_body_yaw_rate_max", body)):
                self._speed[name] = torch.where(active, torch.maximum(self._speed[name], value),
                                                self._speed[name])
            # Task-path checks (B1): what the actor will observe as its previous action equals
            # what was executed; suppression really zeroes the yaw; search never runs on a step
            # whose observation already had a fused measurement with an active track.
            prev = getattr(t, "prev_action", None)
            sup = search.supervisor
            if prev is not None:
                self._search_checks["feedback_xy"] += (
                    active & (prev[:, 0:2].detach().abs().amax(dim=1) > 1e-6)).long()
                self._search_checks["feedback_yaw"] += (
                    active & ((prev[:, 3].detach() - sup.yaw_out).abs() > 1e-6)).long()
            self._search_checks["suppressed_yaw"] += (sup.suppressed & (yaw > 1e-6)).long()
            pending = getattr(self, "_pending_observation", None)
            if pending is not None:
                self._search_checks["active_while_acquired"] += (
                    active & pending[1].bool() & pending[2].bool()).long()
        self._over["commanded"] += (commanded > ref).long()
        self._over["actual"] += (actual > ref).long()
        self._over["displacement"] += (disp > ref).long()
        self._steps += 1

    def observe_capture_distance(self, seg_dist):
        self._capture_dist_min = torch.minimum(self._capture_dist_min, seg_dist.detach())

    def after_perception(self, diagnostics):
        """After perception.observe: keep this observation's signals until an action uses them.

        Counting happens in ``before_action``, so an observation is counted exactly when it drives
        an action -- the convention of the task's own first-acquisition telemetry -- and a repeated
        observation at reset cannot shift the indices.
        """
        visible = diagnostics["visible"].detach().clone()
        camera = diagnostics.get("camera_visible", visible).detach().clone()
        tracker = getattr(getattr(self.task, "perception", None), "tracker", None)
        track = tracker.active.detach().clone() if tracker is not None else torch.zeros_like(visible)
        in_fov, geo_visible, _ = self._geometric_view()
        self._pending_observation = (camera, visible, track, in_fov, geo_visible)
        if self.packets_enabled:
            self._snapshot_target_inputs()

    def before_action(self):
        """Start of step: the observation that drives this action enters the chronology."""
        pending = getattr(self, "_pending_observation", None)
        if pending is None:
            raise RuntimeError("KROC2027 recorder: an action without a preceding observation")
        self.events.observe(*pending)

    def on_episode_end(self, finished, successes, crashes, timeouts):
        """Before the ledger records and before any reset: one record per finished episode."""
        finished = finished.detach().to(torch.bool)
        if not bool(finished.any()):
            return
        env_ids = torch.nonzero(finished, as_tuple=False).flatten()
        capture = successes.detach().to(torch.bool)[env_ids].cpu().tolist()
        crash = (crashes.detach() if crashes.dtype == torch.bool else crashes.detach() > 0)[env_ids].cpu().tolist()
        timeout = timeouts.detach().to(torch.bool)[env_ids].cpu().tolist()
        event_records = self.events.finish(env_ids)
        end_dist = self.task.ep_min_goal_dist[env_ids].detach().double().cpu().tolist()
        host = {k: v[env_ids].detach().double().cpu().tolist() for k, v in self._speed.items()}
        over = {k: v[env_ids].cpu().tolist() for k, v in self._over.items()}
        steps = self._steps[env_ids].cpu().tolist()
        swept = self._swept_min[env_ids].double().cpu().tolist()
        tspeed = self._target_speed_max[env_ids].double().cpu().tolist()
        capdist = self._capture_dist_min[env_ids].double().cpu().tolist()
        # The task's own first-acquisition telemetry (bulk mode), kept for a per-episode cross-check.
        task_first = {}
        for name, attr in (("fused", "_fa_ep_first_fused"), ("camera", "_fa_ep_first_camera")):
            values = getattr(self.task, attr, None)
            task_first[name] = values[env_ids].cpu().tolist() if values is not None else None
        search = getattr(self.task, "_kroc_search", None)
        search_records = search.episode_records(env_ids) if search is not None else None
        checks = {k: v[env_ids].cpu().tolist() for k, v in self._search_checks.items()}
        search_limit = (search.supervisor.spec.yaw_rate_rad_s + 1e-6) if search is not None else None
        for row, env in enumerate(env_ids.cpu().tolist()):
            flags = (bool(capture[row]), bool(crash[row]), bool(timeout[row]))
            outcome = ("capture", "crash", "timeout")[flags.index(True)] if sum(flags) == 1 else "invalid"
            n = max(1, steps[row])
            record = {
                "env_index": env,
                "env_episode_index": self.episode_index[env],
                "outcome": outcome,
                "rl_dt_s": self.dt,
                "actions": steps[row],
                "end_step_min_distance_m": end_dist[row],
                "swept_min_distance_m": capdist[row] if math.isfinite(capdist[row]) else None,
                "target_swept_min_obstacle_clearance_m": swept[row] if math.isfinite(swept[row]) else None,
                "target_step_speed_max_mps": tspeed[row],
                "pursuer_requested_speed_mean_mps": host["requested_sum"][row] / n,
                "pursuer_requested_speed_max_mps": host["requested_max"][row],
                "pursuer_commanded_speed_mean_mps": host["commanded_sum"][row] / n,
                "pursuer_commanded_speed_max_mps": host["commanded_max"][row],
                "pursuer_actual_speed_mean_mps": host["actual_sum"][row] / n,
                "pursuer_actual_speed_max_mps": host["actual_max"][row],
                "pursuer_displacement_speed_max_mps": host["displacement_max"][row],
                "steps_commanded_over_2p5": over["commanded"][row],
                "steps_actual_over_2p5": over["actual"][row],
                "steps_displacement_over_2p5": over["displacement"][row],
            }
            record.update(event_records[row])
            first = record.get("first_obs_fused_valid")
            record["first_acquisition_latency_s"] = (first - 1) * self.dt if first is not None else None
            for name, values in task_first.items():
                if values is not None:
                    record["task_first_%s_obs" % name] = values[row] if values[row] >= 0 else None
            if search_records is not None:
                record.update(search_records[row])
                record["search_commanded_xy_max_mps"] = host["search_xy_max"][row]
                record["search_yaw_rate_max_rad_s"] = host["search_yaw_max"][row]
                record["search_body_yaw_rate_max_rad_s"] = host["search_body_yaw_rate_max"][row]
                for key in checks:
                    record["search_check_" + key] = checks[key][row]
                self.search_counts["episodes"] += 1
                if record["search_search_steps"] > 0:
                    self.search_counts["episodes_with_search"] += 1
                start = record.get("search_initial_start_step")
                first = record.get("first_obs_fused_valid")
                # Initial search exactly 1.0 s after reset, and only with nothing measured before:
                # observations 1..11 drive actions 0..10, so the first fused one must be >= 12.
                trigger_bad = start is not None and (start != 10 or (first is not None and first <= 11))
                feedback_bad = checks["feedback_xy"][row] + checks["feedback_yaw"][row] > 0
                suppression_bad = checks["suppressed_yaw"][row] > 0
                return_bad = checks["active_while_acquired"][row] > 0
                motion_bad = host["search_xy_max"][row] > 1e-6 or host["search_yaw_max"][row] > search_limit
                self.search_counts["trigger_violations"] += int(trigger_bad)
                self.search_counts["feedback_violations"] += int(feedback_bad)
                self.search_counts["suppression_violations"] += int(suppression_bad)
                self.search_counts["return_violations"] += int(return_bad)
                if motion_bad or trigger_bad or feedback_bad or suppression_bad or return_bad:
                    self.search_counts["invariant_violations"] += 1
            initial = self.initial[env]
            if initial is not None:
                record["initial"] = initial
                self.initial_view_counts["episodes"] += 1
                for key, field in (("rounds", "initial_view_rounds"), ("respawns", "initial_view_respawns")):
                    if field in initial:
                        bucket = str(initial[field])
                        self.initial_view_counts[key][bucket] = self.initial_view_counts[key].get(bucket, 0) + 1
                if initial.get("initial_geo_visible"):
                    self.initial_view_counts["initially_in_view"] += 1
                else:
                    self.initial_view_counts["initially_out_of_view"] += 1
            self._handle.write(json.dumps(record, sort_keys=True) + "\n")
            self.episode_index[env] += 1
            self.records += 1
            self.initial[env] = None
        self._handle.flush()

    def _search_summary(self):
        search = getattr(self.task, "_kroc_search", None)
        if search is None:
            return {"enabled": False}
        from dataclasses import asdict
        from motar_eval.search_supervisor import SEARCH_CONTRACT

        return dict(self.search_counts, enabled=True, contract=SEARCH_CONTRACT,
                    spec=asdict(search.supervisor.spec),
                    hard_margin_m=search.supervisor.hard_margin_m,
                    yaw_action=search.supervisor.yaw_action)

    def _finalize_at_exit(self):
        """Interpreter-exit fallback (for runs without the ledger). A missing summary is the signal."""
        try:
            self.finalize(reason="interpreter exit")
        except OSError as exc:
            sys.stderr.write("KROC2027 recorder: summary not written at exit (%s)\n" % exc)

    def finalize(self, reason="complete"):
        """Write the summary once (atomically); safe to call again at interpreter exit."""
        if self._finalized:
            return
        self._finalized = True
        for handle in (self._handle, getattr(self, "_packet_handle", None)):
            try:
                if handle is not None:
                    handle.flush()
                    handle.close()
            except (OSError, ValueError):
                pass
        digest = hashlib.sha256(self.episodes_path.read_bytes()).hexdigest() if self.episodes_path.is_file() else None
        swept = dict(self.swept)
        if not math.isfinite(swept["min_clearance_m"]):
            swept["min_clearance_m"] = None
        swept["proxy_radius_m"] = self.proxy_radius
        summary = {
            "schema": RECORDER_SCHEMA,
            "reason": reason,
            "context": self.context,
            "import_origins": self.import_origins,
            "contracts": {"motion": self._contracts.CONTRACT_ID,
                          "events": self._events_mod.EVENTS_CONTRACT,
                          "scene": self._scene.SCENE_CONTRACT},
            "camera": {"hfov_deg": math.degrees(self.hfov), "vfov_deg": math.degrees(self.vfov),
                       "max_range_m": self.max_range, "translation_m": list(self.camera_t)},
            "episodes_recorded": self.records,
            "episodes_file": EPISODES_FILE,
            "episodes_sha256": digest,
            "motion_gate": self.motion.summary(),
            "swept_clearance": swept,
            "initial_view": self.initial_view_counts,
            "search": self._search_summary(),
            "violation_packets": ({"file": PACKETS_FILE, "written": self.packet_count, "cap": PACKET_CAP}
                                  if self.packets_enabled else {"enabled": False}),
            "on_violation": self.on_violation,
            "pursuer_reference_speed_mps": PURSUER_REFERENCE_SPEED_MPS,
            "finished_at": datetime.now(timezone.utc).isoformat(),
        }
        if self.packets_enabled and self.packet_schema == "full_v2":
            summary["violation_packets"].update(schema="full_v2", bundle_directory=FULL_PACKET_BUNDLES)
            # The runner's matmul precision as found at the end of the cell; read, never set.
            summary["matmul_precision"] = {
                "matmul_allow_tf32": bool(torch.backends.cuda.matmul.allow_tf32),
                "cudnn_allow_tf32": bool(torch.backends.cudnn.allow_tf32),
                "float32_matmul_precision": torch.get_float32_matmul_precision(),
                "torch_version": str(torch.__version__)}
        tmp = self.summary_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(summary, indent=1, sort_keys=True) + "\n")
        os.replace(tmp, self.summary_path)
