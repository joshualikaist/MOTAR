"""B1-R player-boundary action hook (docs/plans/prereg_b1r_classical_baseline_v1.md §2-§3).

Loaded only by ``tools/b1r_site/sitecustomize.py`` when ``MOTAR_B1R_ARM`` is set. ``install()``
registers a post-import patch for ``aerial_gym.rl_training.rl_games.navrl_players``; nothing is
imported early, so the Isaac-Gym-before-torch order of ``runner.py`` is unchanged. The patched
``NavRLPpoPlayerContinuous.get_action`` always runs the original method first (Policy F's
deterministic action), then computes the classical baseline from the online inputs in §2.1 and
returns it (B arms) or discards it (F_SHADOW). Everything downstream -- governor, controller,
recorder, episode ledger and its quota stop -- is inside ``task.step`` and unchanged.

The sidecar ``b1r_hook.json`` is written next to ``NAVRL_BULK_EVAL_JSON`` when ``player.run`` exits
(the episode ledger's quota ``SystemExit`` passes through it while CUDA and the simulator are still
alive); an ``atexit`` handler writes it only if that did not happen.
"""
from __future__ import annotations

import atexit
import hashlib
import importlib.abc
import json
import os
from pathlib import Path
import sys

TARGET_MODULE = "aerial_gym.rl_training.rl_games.navrl_players"
TARGET_CLASS = "NavRLPpoPlayerContinuous"
ARMS = ("F_SHADOW", "B-PN", "B-PP")
SIDECAR = "b1r_hook.json"
LIDAR_ELEVATIONS_DEG = (20.0, 10.0, 0.0, -10.0)
ROOT = Path(__file__).resolve().parents[1]
# The task attributes the hook reads (input firewall, prereg §2.1); a CPU test pins this list.
TASK_READS = ("obs_dict.robot_position", "obs_dict.robot_linvel", "obs_dict.robot_vehicle_orientation",
              "perception.tracker.state", "perception.tracker.active", "perception.tracker.memory_s",
              "perception._lidar_angles", "perception.last_target_like", "_lidar_distance_m",
              "task_config.max_velocity", "task_config.yaw_rate_max", "task_config.lidar_max_range",
              "vision_mode")


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_config(environ=os.environ):
    """Arm and controller parameters from the environment; fail closed on anything unexpected."""
    arm = environ.get("MOTAR_B1R_ARM", "").strip()
    if arm not in ARMS:
        raise RuntimeError("MOTAR_B1R_ARM must be one of %s, got %r" % (ARMS, arm))
    ctrl = environ.get("MOTAR_B1R_CONTROLLER", "").strip()
    if ctrl not in ("B-PN", "B-PP"):
        raise RuntimeError("MOTAR_B1R_CONTROLLER must be B-PN or B-PP, got %r" % ctrl)
    if arm != "F_SHADOW" and ctrl != arm:
        raise RuntimeError("controller %s does not match arm %s" % (ctrl, arm))
    return {"arm": arm, "controller": ctrl, "n_gain": float(environ["MOTAR_B1R_N"]),
            "v_nom": float(environ["MOTAR_B1R_VNOM"])}


class _PatchingLoader(importlib.abc.Loader):
    def __init__(self, inner, patch):
        self.inner, self.patch = inner, patch

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        self.patch(module)


class PostImportPatch(importlib.abc.MetaPathFinder):
    """Delegates the search for one module to the remaining finders and patches it after it runs."""

    def __init__(self, name, patch):
        self.name, self.patch, self.applied = name, patch, False

    def find_spec(self, fullname, path=None, target=None):
        if fullname != self.name or self.applied:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None and spec.loader is not None:
                spec.loader = _PatchingLoader(spec.loader, self._apply)
                return spec
        return None

    def _apply(self, module):
        self.patch(module)
        self.applied = True


class Controller:
    """Per-process state: config, counters and the sidecar; the controller itself is stateless."""

    def __init__(self, config, baseline):
        self.config, self.cb = config, baseline
        self.task = None
        self.calls = 0
        self.counts = {}
        self.static = {}
        self.elev = None
        self.prev_course = None    # course-rate diagnostic only; never fed back

    def bind(self, player):
        obj = player.env
        for _ in range(6):
            if hasattr(obj, "perception") and hasattr(obj, "obs_dict") and hasattr(obj, "_lidar_distance_m"):
                break
            obj = getattr(obj, "env")
        else:
            raise RuntimeError("b1r: task not reachable from the player")
        task, p = obj, self.cb.PARAMS
        checks = {
            "vision_mode": (bool(task.vision_mode), True),
            "max_velocity": (float(task.task_config.max_velocity), p["max_velocity"]),
            "yaw_rate_max": (float(task.task_config.yaw_rate_max), p["yaw_rate_max"]),
            "lidar_max_range": (float(task.task_config.lidar_max_range), p["lidar_far_m"]),
            "lidar_hbeams": (int(task.perception._lidar_angles.numel()), 72),
            "tracker_memory_s": (float(task.perception.tracker.memory_s), 5.0),
            "target_like_shape": (tuple(task.perception.last_target_like.shape[1:]), (4, 72)),
        }
        bad = {k: v for k, v in checks.items() if v[0] != v[1]}
        if bad:
            raise RuntimeError("b1r: task contract differs from the preregistration: %s" % bad)
        low, high = getattr(player, "actions_low", None), getattr(player, "actions_high", None)
        if getattr(player, "clip_actions", False) and low is not None:
            if not (bool((low == -1).all()) and bool((high == 1).all())):
                raise RuntimeError("b1r: player action range is not [-1, 1]")
        self.task = task
        self.static = {"task_checks": {k: v[0] for k, v in checks.items()}}

    def act(self, player, policy_action):
        import torch

        if self.task is None:
            self.bind(player)
        t, cb = self.task, self.cb
        od, perception = t.obs_dict, t.perception
        device = policy_action.device
        if self.elev is None:
            self.elev = torch.deg2rad(torch.tensor(LIDAR_ELEVATIONS_DEG, device=device))
        elev = self.elev
        action, info = cb.baseline_action(
            self.config["controller"], od["robot_position"], od["robot_linvel"], od["robot_vehicle_orientation"],
            perception.tracker.state, perception.tracker.active, t._lidar_distance_m(), perception._lidar_angles,
            elev, perception.last_target_like, n_gain=self.config["n_gain"], v_nom=self.config["v_nom"])
        self.calls += 1
        finite = torch.isfinite(action).all(dim=1)
        step = {"steps_env": action.shape[0], "non_finite": (~finite).sum(), "raw_out_of_bounds": info["raw_out_of_bounds"].sum(),
                "nonzero_z": (action[:, 2] != 0).sum(), "no_belief": info["no_belief"].sum(),
                "excluded_mask": info["excluded_mask"].sum(), "excluded_belief": info["excluded_belief"].sum(),
                "filter_active": info["filter_active"].sum(), "pn_saturated": info["pn_saturated"].sum(),
                "slow_course": info["slow"].sum(),
                "abs_diff_vs_policy": (action.to(policy_action.dtype) - policy_action).abs().sum()}
        # Course-rate diagnostic (prereg §7): realised |d psi_v|/dt against the commanded |clamped N*lambda_dot|
        # on consecutive active, non-slow steps. Approximate across resets; PROCESS_WINDOW.
        vel = od["robot_linvel"][:, 0:2]
        course = torch.atan2(vel[:, 1], vel[:, 0])
        usable = (~info["no_belief"]) & (~info["slow"])
        if self.prev_course is not None and self.config["controller"] == "B-PN":
            both = usable & self.prev_course[1]
            d = torch.atan2(torch.sin(course - self.prev_course[0]), torch.cos(course - self.prev_course[0])).abs()
            cmd = (self.config["n_gain"] * info["los_rate"]).clamp(-3.0, 3.0).abs()
            step["course_rate_pairs"] = both.sum()
            step["course_rate_realised_sum"] = torch.where(both, d / cb.PARAMS["dt_s"], torch.zeros_like(d)).sum()
            step["course_rate_commanded_sum"] = torch.where(both, cmd, torch.zeros_like(cmd)).sum()
        self.prev_course = (course, usable)
        for k, v in step.items():  # device tensors; one host sync at exit, not per step
            self.counts[k] = self.counts.get(k, 0) + v
        if self.config["arm"] == "F_SHADOW":
            return policy_action
        return action.to(dtype=policy_action.dtype)

    def summary(self):
        counts = {k: (float(v) if hasattr(v, "item") else v) for k, v in self.counts.items()}
        files = {name: sha256_file(ROOT / name) for name in (
            "motar_eval/classical_baseline.py", "tools/b1r_hook.py", "tools/b1r_site/sitecustomize.py")}
        origins = {}
        for mod in ("aerial_gym", "motar_eval"):
            m = sys.modules.get(mod)
            origins[mod] = str(Path(m.__file__).resolve()) if m is not None and getattr(m, "__file__", None) else ""
        site = sys.modules.get("sitecustomize")
        return {"schema": "motar.b1r-hook.v1", "pid": os.getpid(), "argv": list(sys.argv),
                "sitecustomize": str(Path(site.__file__).resolve()) if site is not None and getattr(site, "__file__", None) else "",
                "window": "PROCESS_WINDOW", "arm": self.config["arm"], "controller": self.config["controller"],
                "n_gain": self.config["n_gain"], "v_nom": self.config["v_nom"], "patched": True,
                "get_action_calls": self.calls, "counts": counts, "params": dict(self.cb.PARAMS),
                "source_sha256": files, "import_origins": origins, **self.static}


STATE = {"finder": None, "controller": None, "written_by": None}


def _load_baseline():
    import importlib

    return importlib.import_module("motar_eval.classical_baseline")


def patch_player_module(module, controller_factory):
    cls = getattr(module, TARGET_CLASS)
    original = cls.get_action

    def get_action(self, obs, is_deterministic=False):
        policy_action = original(self, obs, is_deterministic)
        return controller_factory().act(self, policy_action)

    get_action.__b1r_original__ = original
    cls.get_action = get_action
    original_run = cls.run

    def run(self, *args, **kwargs):
        try:
            return original_run(self, *args, **kwargs)
        finally:
            _write_sidecar("player_run_exit")

    run.__b1r_original__ = original_run
    cls.run = run


def _write_sidecar(trigger="atexit"):
    ctrl = STATE["controller"]
    if ctrl is None or STATE["written_by"] is not None:
        return
    STATE["written_by"] = trigger
    try:
        destination = os.environ.get("NAVRL_BULK_EVAL_JSON")
        if not destination:
            raise RuntimeError("controller acted without NAVRL_BULK_EVAL_JSON")
        path = Path(destination).parent / SIDECAR
        payload = ctrl.summary()
        payload["patch_applied"] = bool(STATE["finder"] and STATE["finder"].applied)
        payload["written_by"] = trigger
        with path.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    except BaseException as exc:
        # Exceptions from atexit alone do not change the process exit code.
        os.write(2, ("b1r sidecar failed: %r\n" % exc).encode())
        os._exit(3)


def install(environ=os.environ):
    config = read_config(environ)

    def factory():
        if STATE["controller"] is None:
            STATE["controller"] = Controller(config, _load_baseline())
        return STATE["controller"]

    finder = PostImportPatch(TARGET_MODULE, lambda module: patch_player_module(module, factory))
    sys.meta_path.insert(0, finder)
    STATE["finder"] = finder
    atexit.register(_write_sidecar)
    return finder
